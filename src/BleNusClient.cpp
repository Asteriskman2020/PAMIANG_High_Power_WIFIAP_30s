#include "BleNusClient.h"
#include <esp_task_wdt.h>

BleNusClient::BleNusClient()
    : _pClient(nullptr), _pWriteChar(nullptr), _pNotifyChar(nullptr),
      _addressKnown(false), _state(STATE_UNINIT), _lastAttemptMs(0),
      _lastConnectMs(0), _failedAttempts(0), _initialized(false), _busy(false),
      _totalSent(0), _totalErrors(0), _totalDropped(0),
      _ackReceived(false), _ackValue(0), _clientCB(this),
      _scanCB(this), _scanFound(false) {
    _targetName[0]   = '\0';
    _serviceUuid[0]  = '\0';
    _rxCharUuid[0]   = '\0';
    _txCharUuid[0]   = '\0';
}

BleNusClient::~BleNusClient() {
    disconnect();
}

bool BleNusClient::begin(const char* targetName, const char* serviceUuid,
                          const char* rxCharUuid, const char* txCharUuid) {
    if (_initialized) return true;

    strncpy(_targetName,  targetName,  sizeof(_targetName)  - 1);
    strncpy(_serviceUuid, serviceUuid, sizeof(_serviceUuid) - 1);
    strncpy(_rxCharUuid,  rxCharUuid,  sizeof(_rxCharUuid)  - 1);
    strncpy(_txCharUuid,  txCharUuid,  sizeof(_txCharUuid)  - 1);

    NimBLEDevice::init("PAMIANG-RX");
    NimBLEDevice::setMTU(BLE_MTU_REQUEST);

    _pClient = NimBLEDevice::createClient();
    if (!_pClient) {
        Serial.println(F("[BLE-CLI] Failed to create client"));
        return false;
    }

    _pClient->setClientCallbacks(&_clientCB, false);
    _pClient->setConnectTimeout(BLE_CONNECT_TIMEOUT_S);

    _initialized = true;
    _state = STATE_IDLE;
    _lastAttemptMs = 0;

    Serial.printf("[BLE-CLI] Initialized, target: %s\n", _targetName);
    return true;
}

void BleNusClient::transitionState(BleState newState) {
    if (_state != newState) {
        Serial.printf("[BLE-CLI] State: %d -> %d\n", _state, newState);
        _state = newState;
    }
}

void BleNusClient::loop() {
    if (!_initialized) return;

    switch (_state) {
        case STATE_UNINIT:
            break;

        case STATE_IDLE:
            transitionState(STATE_SCANNING);
            break;

        case STATE_SCANNING:
            if (doScan()) {
                transitionState(STATE_CONNECTING);
            } else {
                transitionState(STATE_WAIT_RECONNECT);
                _lastAttemptMs = millis();
            }
            break;

        case STATE_CONNECTING:
            if (doConnect()) {
                _failedAttempts = 0;
                transitionState(STATE_CONNECTED);
            } else {
                _failedAttempts++;
                if (_failedAttempts >= BLE_MAX_FAILED_ATTEMPTS) {
                    _addressKnown = false;
                    _failedAttempts = 0;
                }
                transitionState(STATE_WAIT_RECONNECT);
                _lastAttemptMs = millis();
            }
            break;

        case STATE_CONNECTED:
            if (!_pClient || !_pClient->isConnected()) {
                Serial.println(F("[BLE-CLI] Connection lost"));
                _pWriteChar  = nullptr;
                _pNotifyChar = nullptr;
                transitionState(STATE_WAIT_RECONNECT);
                _lastAttemptMs = millis();
            }
            break;

        case STATE_WAIT_RECONNECT: {
            uint32_t delay = BLE_RECONNECT_MIN_MS + (_failedAttempts * 2000);
            if (delay > BLE_RECONNECT_MAX_MS) delay = BLE_RECONNECT_MAX_MS;

            if (millis() - _lastAttemptMs >= delay) {
                if (_addressKnown) {
                    transitionState(STATE_CONNECTING);
                } else {
                    transitionState(STATE_SCANNING);
                }
            }
            break;
        }
    }
}

void BleNusClient::ScanCallbacks::onResult(NimBLEAdvertisedDevice* dev) {
    if (!dev) return;
    if (parent->_scanFound) return;

    std::string name = dev->getName();
    bool nameMatch = (name.length() > 0 && name == parent->_targetName);

    NimBLEUUID svcUuid(parent->_serviceUuid);
    bool svcMatch = false;
    if (dev->isAdvertisingService(svcUuid)) {
        svcMatch = true;
    }

    if (nameMatch) {
        parent->_scanAddress = dev->getAddress();
        parent->_scanFound = true;
        Serial.printf("[BLE-CLI] Found %s at %s (RSSI: %d)\n",
                      name.c_str(),
                      parent->_scanAddress.toString().c_str(),
                      dev->getRSSI());
        NimBLEDevice::getScan()->stop();
    } else if (svcMatch) {
        Serial.printf("[BLE-CLI] Ignored NUS device '%s' at %s (target: %s)\n",
                      name.length() > 0 ? name.c_str() : "(no name)",
                      dev->getAddress().toString().c_str(),
                      parent->_targetName);
    }
}

bool BleNusClient::doScan() {
    Serial.printf("[BLE-CLI] Scanning %us for '%s'...\n",
                  BLE_SCAN_DURATION_S, _targetName);
    esp_task_wdt_reset();

    _scanFound = false;

    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setAdvertisedDeviceCallbacks(&_scanCB, false);
    pScan->setActiveScan(true);
    pScan->setInterval(100);
    pScan->setWindow(99);
    pScan->start(BLE_SCAN_DURATION_S, false);

    pScan->clearResults();
    esp_task_wdt_reset();

    if (_scanFound) {
        _targetAddress = _scanAddress;
        _addressKnown  = true;
        return true;
    }

    Serial.println(F("[BLE-CLI] Target device not found"));
    return false;
}

bool BleNusClient::doConnect() {
    if (!_pClient) return false;

    if (_pClient->isConnected()) {
        return true;
    }

    if (!_addressKnown) return false;

    Serial.printf("[BLE-CLI] Connecting to %s...\n",
                  _targetAddress.toString().c_str());
    esp_task_wdt_reset();

    bool connected = false;
    if (_pClient->connect(_targetAddress)) {
        connected = true;
    }

    if (!connected) {
        Serial.println(F("[BLE-CLI] Connect failed"));
        return false;
    }

    Serial.printf("[BLE-CLI] Connected, MTU: %u\n", _pClient->getMTU());
    esp_task_wdt_reset();

    if (!discoverServices()) {
        Serial.println(F("[BLE-CLI] Service discovery failed"));
        _pClient->disconnect();
        return false;
    }

    _lastConnectMs = millis();
    return true;
}

bool BleNusClient::discoverServices() {
    NimBLERemoteService* pSvc = _pClient->getService(_serviceUuid);
    if (!pSvc) {
        Serial.printf("[BLE-CLI] Service %s not found\n", _serviceUuid);
        return false;
    }

    _pWriteChar = pSvc->getCharacteristic(_rxCharUuid);
    if (!_pWriteChar) {
        Serial.printf("[BLE-CLI] Write char %s not found\n", _rxCharUuid);
        return false;
    }

    if (!_pWriteChar->canWrite()) {
        Serial.println(F("[BLE-CLI] Write char not writable"));
        _pWriteChar = nullptr;
        return false;
    }

    _pNotifyChar = pSvc->getCharacteristic(_txCharUuid);
    if (_pNotifyChar && _pNotifyChar->canNotify()) {
        _pNotifyChar->subscribe(true, BleNusClient::notifyCallback);
        Serial.println(F("[BLE-CLI] Subscribed to NUS TX (ACK channel)"));
    }

    Serial.println(F("[BLE-CLI] Service discovery complete"));
    return true;
}

void BleNusClient::notifyCallback(NimBLERemoteCharacteristic*,
                                    uint8_t* pData, size_t length, bool) {
    if (length >= 1) {
        Serial.printf("[BLE-CLI] ACK received: 0x%02X\n", pData[0]);
    }
}

void BleNusClient::ClientCallbacks::onConnect(NimBLEClient*) {
    Serial.println(F("[BLE-CLI] Callback: connected"));
}

void BleNusClient::ClientCallbacks::onDisconnect(NimBLEClient*) {
    Serial.println(F("[BLE-CLI] Callback: disconnected"));
    parent->_pWriteChar  = nullptr;
    parent->_pNotifyChar = nullptr;
    if (parent->_state == STATE_CONNECTED) {
        parent->transitionState(STATE_WAIT_RECONNECT);
        parent->_lastAttemptMs = millis();
    }
}

bool BleNusClient::sendPacket(const BleTransport_UARTPacket& packet) {
    return sendBytes(reinterpret_cast<const uint8_t*>(&packet),
                     sizeof(BleTransport_UARTPacket));
}

bool BleNusClient::sendBytes(const uint8_t* data, size_t len) {
    if (!_initialized || _busy) return false;
    if (!data || len == 0) return false;

    if (_state != STATE_CONNECTED || !_pClient || !_pClient->isConnected()) {
        _totalDropped++;
        return false;
    }

    if (!_pWriteChar) {
        _totalErrors++;
        return false;
    }

    _busy = true;
    esp_task_wdt_reset();

    bool ok = _pWriteChar->writeValue(
        data,
        len,
        true
    );

    _busy = false;

    if (ok) {
        _totalSent++;
        Serial.printf("[BLE-CLI] Sent packet (%u bytes)\n", (unsigned)len);
    } else {
        _totalErrors++;
        Serial.println(F("[BLE-CLI] Write FAILED"));
    }

    return ok;
}

bool BleNusClient::isConnected() const {
    return _initialized && _state == STATE_CONNECTED &&
           _pClient && _pClient->isConnected();
}

void BleNusClient::disconnect() {
    if (_pClient) {
        if (_pClient->isConnected()) {
            _pClient->disconnect();
        }
    }
    _pWriteChar  = nullptr;
    _pNotifyChar = nullptr;
    transitionState(STATE_IDLE);
}

uint32_t BleNusClient::getTotalSent()    const { return _totalSent;    }
uint32_t BleNusClient::getTotalErrors()  const { return _totalErrors;  }
uint32_t BleNusClient::getTotalDropped() const { return _totalDropped; }

uint16_t BleNusClient::calculateChecksum(const uint8_t* data, size_t len) {
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}
