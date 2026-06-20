/* BLE handler — extracted from main.cpp */
#include "BleHandler.h"
#include "utilities.h"
#include "TxUtilities.h"
#include <esp_task_wdt.h>
#include <LittleFS.h>

/* ── Static trampoline for NUS notification callback ── */
static BleHandler* _bleInstance = nullptr;

static char bleFoldMacChar(char c) {
    if (c >= 'A' && c <= 'Z') return (char)(c - 'A' + 'a');
    return c;
}

static bool bleMacEquals(const char* a, const char* b) {
    if (!a || !b) return false;
    while (*a && *b) {
        if (bleFoldMacChar(*a++) != bleFoldMacChar(*b++)) return false;
    }
    return *a == '\0' && *b == '\0';
}

/* Trampoline: receives NUS data, accumulates into BleHandler private members.
 * Declared as friend in BleHandler.h so it can access _nusAccum etc. */
void nusTrampoline(NimBLERemoteCharacteristic*,
                    uint8_t* pData, size_t length, bool) {
    if (!_bleInstance) return;
    BleHandler* bh = _bleInstance;
    char* accum = bh->_nusAccum;
    int& len = bh->_nusAccumLen;
    for (size_t i = 0; i < length; i++) {
        if (len < NUS_BUF - 1)
            accum[len++] = (char)pData[i];
    }
    accum[len] = '\0';

    char* close = strrchr(accum, '}');
    if (close) {
        char* open = accum;
        for (char* p = accum; p <= close; p++)
            if (*p == '{') open = p;
        if (open < close) {
            int jsonLen = (int)(close - open) + 1;
            if (jsonLen < NUS_BUF) {
                memcpy(bh->_nusJsonReady, open, jsonLen);
                bh->_nusJsonReady[jsonLen] = '\0';
                bh->_nusNotified = true;
            }
        }
        int rem = len - (int)(close - accum) - 1;
        if (rem > 0) memmove(accum, close + 1, rem);
        len = (rem > 0) ? rem : 0;
        accum[len] = '\0';
    }
    if (len >= NUS_BUF - 1) len = 0;
}

/* ── Constructor ── */
BleHandler::BleHandler()
    : _ctx(nullptr), _active(false), _deviceCount(0), _lastScan(0),
      _targetSeenInScan(false),
      _connPending(false), _scanRequest(false), _dataReceived(false),
      _persistClient(nullptr), _lastRefresh(0),
      _isNus(false), _nusAccumLen(0), _nusNotified(false),
      _nusRxChar(nullptr), _nusLastMs(0), _nusLastAcceptedMs(0), _nusRefreshReq(false),
      _scanCB(this) {}

void BleHandler::begin(BleContext* ctx) {
    _ctx = ctx;
    _targetMac[0] = '\0';
    _savedMac[0]  = '\0';
    _lastSavedTs[0] = '\0';
}

/* ── Accessors ── */
BLEFoundDevice*     BleHandler::getDevices()      { return _devices; }
uint8_t*            BleHandler::getDeviceCount()  { return &_deviceCount; }
bool                BleHandler::wasTargetSeenInLastScan() const { return _targetSeenInScan; }
BLEConnectedDevice* BleHandler::getConnDev()      { return &_connDev; }
char*               BleHandler::getTargetMac()    { return _targetMac; }
char*               BleHandler::getSavedMac()     { return _savedMac; }
bool*               BleHandler::getConnPending()  { return &_connPending; }
bool*               BleHandler::getScanRequest()  { return &_scanRequest; }
bool*               BleHandler::getDataReceived() { return &_dataReceived; }
bool*               BleHandler::getIsActive()     { return &_active; }
bool*               BleHandler::getIsNus()        { return &_isNus; }
unsigned long*      BleHandler::getNusLastMs()    { return &_nusLastMs; }
bool*               BleHandler::getNusRefreshReq(){ return &_nusRefreshReq; }
volatile bool*      BleHandler::getNusNotified()  { return &_nusNotified; }
NimBLERemoteCharacteristic* BleHandler::getNusRxChar() { return _nusRxChar; }
unsigned long*      BleHandler::getLastRefresh() { return &_lastRefresh; }

/* ── Init BLE ── */
void BleHandler::init() {
    _ctx->nvs->begin("ws-cfg", true);
    bool bleEn = _ctx->nvs->getBool("bleEnable", false);
    _ctx->nvs->end();

    if (!bleEn) { Serial.println(F("[BLE] Client disabled")); return; }

    NimBLEDevice::init("WeatherStation-GW");
    NimBLEDevice::setPower(ESP_PWR_LVL_P3);
    _active = true;

    NimBLEScan* scan = NimBLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(&_scanCB, false);
    scan->setActiveScan(true);
    scan->setInterval(100);
    scan->setWindow(99);
    Serial.println(F("[BLE] NimBLE client ready (scanner)"));
}

/* ── Deinit BLE — clean up before state transition ── */
void BleHandler::deinit() {
    if (!_active) return;
    if (_persistClient) {
        if (_persistClient->isConnected())
            _persistClient->disconnect();
        NimBLEDevice::deleteClient(_persistClient);
        _persistClient = nullptr;
    }
    NimBLEDevice::deinit();
    _active = false;
    _connDev.connected = false;
    _connDev.charCount = 0;
    _isNus = false;
    _nusRxChar = nullptr;
    Serial.println(F("[BLE] Deinitialized"));
}

/* ── Scan ── */
void BleHandler::doScan() {
    _targetSeenInScan = false;
    if (!_active) return;
    _ctx->nvs->begin("ws-cfg", true);
    bool centralEn = _ctx->nvs->getBool("bleEnable", false);
    _ctx->nvs->end();
    if (!centralEn) return;

    Serial.println(F("[BLE] Scanning 3s..."));
    _deviceCount = 0;
    NimBLEScan* scan = NimBLEDevice::getScan();
    esp_task_wdt_reset();
    if (_yieldFn) _yieldFn();
    for (uint8_t slice = 0; slice < 3; slice++) {
        scan->start(1, slice > 0);
        esp_task_wdt_reset();
        if (_yieldFn) _yieldFn();
    }
    scan->clearResults();
    esp_task_wdt_reset();
    if (_yieldFn) _yieldFn();
    Serial.printf("[BLE] Found %u device(s)\n", _deviceCount);
}

/* ── Scan callback ── */
void BleHandler::ScanCB::onResult(NimBLEAdvertisedDevice* dev) {
    std::string macStr = dev->getAddress().toString();
    const char* mac = macStr.c_str();
    if (strlen(_parent->_targetMac) >= 17 && bleMacEquals(mac, _parent->_targetMac)) {
        _parent->_targetSeenInScan = true;
    }

    if (_parent->_deviceCount >= BLE_MAX_DEVICES) return;
    BLEFoundDevice& d = _parent->_devices[_parent->_deviceCount++];
    const char* nm = dev->getName().length() ? dev->getName().c_str() : "(unknown)";
    strncpy(d.name, nm, sizeof(d.name) - 1); d.name[sizeof(d.name)-1] = '\0';
    strncpy(d.mac, mac, sizeof(d.mac) - 1); d.mac[sizeof(d.mac)-1] = '\0';
    d.rssi = dev->getRSSI();
}

/* ── Connect and read ── */
void BleHandler::connectAndRead(const char* mac) {
    if (!_active || strlen(mac) < 17) return;
    Serial.printf("[BLE] Connecting to %s\n", mac);

    _connDev.connected = false;
    _connDev.charCount = 0;
    strncpy(_connDev.mac, mac, sizeof(_connDev.mac) - 1);
    _connDev.name[0] = '\0';

    for (uint8_t i = 0; i < _deviceCount; i++) {
        if (strcmp(_devices[i].mac, mac) == 0) {
            strncpy(_connDev.name, _devices[i].name, sizeof(_connDev.name) - 1);
            break;
        }
    }

    if (!_persistClient) _persistClient = NimBLEDevice::createClient();
    if (_persistClient->isConnected()) {
        _persistClient->disconnect();
        delay(300);
    }

    /* Shorter timeout when yield callback is set (WiFi AP active) */
    int maxAttempts = _yieldFn ? 1 : 3;
    _persistClient->setConnectTimeout(_yieldFn ? 5 : 30);

    esp_task_wdt_reset();
    bool connected = false;
    for (int att = 1; att <= maxAttempts && !connected; att++) {
        if (att > 1) {
            Serial.printf("[BLE] Retry connect attempt %d/%d (1s delay)\n", att, maxAttempts);
            for (int w = 0; w < 10; w++) {  /* 10 × 100ms = 1s, yielding each slice */
                if (_yieldFn) _yieldFn();
                delay(100);
            }
            esp_task_wdt_reset();
        }
        if (_yieldFn) _yieldFn();
        connected = _persistClient->connect(NimBLEAddress(mac));
        if (!connected) Serial.printf("[BLE] Connect attempt %d FAILED\n", att);
        if (_yieldFn) _yieldFn();
    }
    if (!connected) {
        Serial.printf("[BLE] Connect FAILED after %d attempt(s)\n", maxAttempts);
        return;
    }
    Serial.println(F("[BLE] Connected. Discovering services..."));
    esp_task_wdt_reset();

    memset(_charPtrs, 0, sizeof(_charPtrs));
        _isNus        = false;
        _nusRxChar    = nullptr;
        _nusAccumLen  = 0;
        _nusAccum[0]  = '\0';
        _nusNotified  = false;
        _nusLastAcceptedMs = 0;

    NimBLERemoteService* nusSvc = _persistClient->getService(NUS_SERVICE_UUID);
    Serial.printf("[BLE] NUS service: %s\n", nusSvc ? "FOUND" : "not found");

    if (nusSvc) {
        NimBLERemoteCharacteristic* nusTx = nusSvc->getCharacteristic(NUS_TX_UUID);
        NimBLERemoteCharacteristic* nusRx = nusSvc->getCharacteristic(NUS_RX_UUID);
        Serial.printf("[BLE] NUS TX: %s  RX: %s\n",
            nusTx ? "found" : "missing", nusRx ? "found" : "missing");
        if (nusTx && nusTx->canNotify()) {
            _bleInstance = this;  /* set trampoline target */
            nusTx->subscribe(true, nusTrampoline);
            Serial.println(F("[BLE] NUS TX subscribed OK"));
        }
        _nusRxChar = nusRx;
        _isNus     = true;
        _connDev.connected = true;
        _lastRefresh = millis();

        if (_nusRxChar && _nusRxChar->canWrite()) {
            _nusRxChar->writeValue((uint8_t*)"live", 4, false);
            Serial.println(F("[BLE] NUS: sent initial live request"));
        }

        strncpy(_savedMac, mac, sizeof(_savedMac) - 1);
        _ctx->nvs->begin("ws-cfg", false);
        _ctx->nvs->putString("bleSavedMac", mac);
        _ctx->nvs->end();
        Serial.println(F("[BLE] NUS connected, waiting for notifications"));
        return;
    }

    /* ── GATT mode ── */
    Serial.println(F("[BLE] GATT mode — reading characteristics"));
    auto* services = _persistClient->getServices(true);
    if (services) {
        for (auto* svc : *services) {
            if (_connDev.charCount >= BLE_CHAR_MAX) break;
            auto* chars = svc->getCharacteristics(true);
            if (!chars) continue;
            for (auto* ch : *chars) {
                if (_connDev.charCount >= BLE_CHAR_MAX) break;
                if (!ch->canRead()) continue;
                esp_task_wdt_reset();
                uint8_t idx = _connDev.charCount;
                BLECharData& cd = _connDev.chars[idx];
                strncpy(cd.uuid, ch->getUUID().toString().c_str(), sizeof(cd.uuid) - 1);
                std::string raw = ch->readValue();
                char hex[64] = ""; int hl = 0;
                for (size_t b = 0; b < raw.size() && hl < 60; b++)
                    hl += snprintf(hex + hl, sizeof(hex) - hl, "%02X ", (uint8_t)raw[b]);
                strncpy(cd.value, hex, sizeof(cd.value) - 1);
                char asc[32] = ""; int al = 0;
                for (size_t b = 0; b < raw.size() && al < 30; b++)
                    asc[al++] = (raw[b] >= 0x20 && raw[b] < 0x7F) ? raw[b] : '.';
                asc[al] = '\0';
                strncpy(cd.ascii, asc, sizeof(cd.ascii) - 1);
                _charPtrs[idx] = ch;
                _connDev.charCount++;
            }
        }
    }

    _connDev.connected = true;
    _lastRefresh = millis();
    Serial.printf("[BLE] GATT connected, %u readable chars\n", _connDev.charCount);

    if (_connDev.charCount > 0) {
        _dataReceived = true;
        Serial.println(F("[BLE] GATT data received — flagging state advance"));
    }

    strncpy(_savedMac, mac, sizeof(_savedMac) - 1);
    _ctx->nvs->begin("ws-cfg", false);
    _ctx->nvs->putString("bleSavedMac", mac);
    _ctx->nvs->end();
}

/* ── Refresh data ── */
void BleHandler::refreshData() {
    if (!_connDev.connected) return;
    if (!_persistClient || !_persistClient->isConnected()) {
        Serial.println(F("[BLE] Connection lost"));
        _connDev.connected = false;
        _isNus = false;
        return;
    }
    if (_isNus) {
        if (_nusRxChar && _nusRxChar->canWrite()) {
            _nusRxChar->writeValue((uint8_t*)"live", 4, false);
        }
        return;
    }
    for (uint8_t i = 0; i < _connDev.charCount; i++) {
        NimBLERemoteCharacteristic* ch = _charPtrs[i];
        if (!ch) continue;
        esp_task_wdt_reset();
        std::string raw = ch->readValue();
        BLECharData& cd = _connDev.chars[i];
        char hex[64] = ""; int hl = 0;
        for (size_t b = 0; b < raw.size() && hl < 60; b++)
            hl += snprintf(hex + hl, sizeof(hex) - hl, "%02X ", (uint8_t)raw[b]);
        strncpy(cd.value, hex, sizeof(cd.value) - 1);
        char asc[32] = ""; int al = 0;
        for (size_t b = 0; b < raw.size() && al < 30; b++)
            asc[al++] = (raw[b] >= 0x20 && raw[b] < 0x7F) ? raw[b] : '.';
        asc[al] = '\0';
        strncpy(cd.ascii, asc, sizeof(cd.ascii) - 1);
    }
}

/* ── Handle NUS notification (called from loop when _nusNotified is true) ── */
void BleHandler::handleNusNotification() {
    if (!_nusNotified) return;
    _nusNotified = false;
    unsigned long now = millis();
    bool isError = strstr(_nusJsonReady, "\"error\"") != nullptr;
    if (!isError && _nusLastAcceptedMs != 0 &&
        now - _nusLastAcceptedMs < BLE_NUS_SAMPLE_INTERVAL_MS) {
        return;
    }
    parseNusJson(_nusJsonReady);
    if (!isError) {
        _nusLastMs = now;
        _nusLastAcceptedMs = now;
    }
}

/* ── JSON field extractor ── */
void BleHandler::jsonField(const char* j, const char* key, char* out, size_t outLen) {
    char needle[40]; snprintf(needle, sizeof(needle), "\"%s\":", key);
    const char* p = strstr(j, needle);
    if (!p) { strncpy(out, "--", outLen - 1); return; }
    p += strlen(needle);
    if (*p == '"') {
        p++;
        const char* e = strchr(p, '"');
        if (!e) { strncpy(out, "--", outLen - 1); return; }
        size_t n = min((size_t)(e - p), outLen - 1);
        memcpy(out, p, n); out[n] = '\0';
    } else {
        const char* e1 = strchr(p, ',');
        const char* e2 = strchr(p, '}');
        const char* e  = (!e1 || (e2 && e2 < e1)) ? e2 : e1;
        if (!e) { strncpy(out, "--", outLen - 1); return; }
        size_t n = min((size_t)(e - p), outLen - 1);
        memcpy(out, p, n); out[n] = '\0';
    }
}

/* ── Parse NUS JSON ── */
void BleHandler::parseNusJson(const char* json) {
    if (strstr(json, "\"error\"") != nullptr) {
        Serial.printf("[BLE] NUS error response ignored: %.80s\n", json);
        return;
    }
    struct { const char* k; const char* l; } fields[] = {
        {"ts",     "Timestamp"},
        {"temp",   "Temperature (C)"},
        {"hum",    "Humidity (%)"},
        {"tmp117", "TMP117 (C)"},
        {"rain",   "Rainfall"},
        {"leaf",   "Leaf Wetness"},
        {"par",    "PAR Light"},
        {"soil",   "Soil Moisture"},
    };
    _connDev.charCount = 0;
    char val[32];
    for (auto& f : fields) {
        if (_connDev.charCount >= BLE_CHAR_MAX) break;
        BLECharData& cd = _connDev.chars[_connDev.charCount++];
        strncpy(cd.uuid, f.l, sizeof(cd.uuid) - 1);
        jsonField(json, f.k, val, sizeof(val));
        strncpy(cd.ascii, val, sizeof(cd.ascii) - 1);
        strncpy(cd.value, val, sizeof(cd.value) - 1);
    }
    Serial.printf("[BLE] NUS parsed %u fields from: %.80s\n",
                  _connDev.charCount, json);

    /* Parse into lastBleData */
    {
        char v[32];
        jsonField(json, "temp",   v, sizeof(v)); _ctx->lastBleData->ble_temp   = (int16_t)(atof(v) * 10);
        jsonField(json, "hum",    v, sizeof(v)); _ctx->lastBleData->ble_humi   = (uint16_t)(atof(v) * 10);
        jsonField(json, "tmp117", v, sizeof(v)); _ctx->lastBleData->ble_tmp117 = (int16_t)(atof(v) * 10);
        jsonField(json, "rain",   v, sizeof(v)); _ctx->lastBleData->ble_rain   = (uint16_t)(atoi(v));
        jsonField(json, "leaf",   v, sizeof(v)); _ctx->lastBleData->ble_leaf   = (uint16_t)(atoi(v));
        jsonField(json, "par",    v, sizeof(v)); _ctx->lastBleData->ble_par    = (uint16_t)(atoi(v));
        jsonField(json, "soil",   v, sizeof(v)); _ctx->lastBleData->ble_soil   = (uint16_t)(atoi(v));
        *_ctx->lastBleValid = true;
    }

    /* Use BLE timestamp for currentTime if available */
    {
        char ts[32] = "";
        jsonField(json, "ts", ts, sizeof(ts));
        if (ts[0]) {
            int d = 0, m = 0, y = 0, hh = 0, mm = 0, ss = 0;
            if (sscanf(ts, "%d-%d-%d %d:%d:%d", &d, &m, &y, &hh, &mm, &ss) == 6 && y >= 2024) {
                _ctx->currentTime->date   = (uint8_t)d;
                _ctx->currentTime->month  = (uint8_t)m;
                _ctx->currentTime->year   = (uint16_t)y;
                _ctx->currentTime->hour   = (uint8_t)hh;
                _ctx->currentTime->minute = (uint8_t)mm;
                _ctx->currentTime->second = (uint8_t)ss;
                snprintf(_ctx->currentTime->dateStr, sizeof(_ctx->currentTime->dateStr),
                         "%02d/%02d/%04d", d, m, y);
                snprintf(_ctx->currentTime->timeStr, sizeof(_ctx->currentTime->timeStr),
                         "%02d:%02d:%02d", hh, mm, ss);
                Serial.printf("[BLE] Time sync from BLE: %s %s\n",
                             _ctx->currentTime->dateStr, _ctx->currentTime->timeStr);
            }
        }
    }

    saveBleDataToCsv(json);

    _dataReceived = true;
    Serial.println(F("[BLE] Data received — flagging state advance"));
}

/* ── Save BLE data to CSV ── */
void BleHandler::saveBleDataToCsv(const char* json) {
    char ts[20] = "";
    jsonField(json, "ts", ts, sizeof(ts));
    if (!ts[0] || ts[0] == '-') return;

    if (strcmp(ts, _lastSavedTs) == 0) {
        Serial.println(F("[BLE-CSV] Duplicate timestamp — skip"));
        return;
    }

    char temp[12]="", hum[12]="", tmp117[12]="";
    char rain[12]="", leaf[12]="", par[12]="", soil[12]="";
    jsonField(json, "temp",   temp,   sizeof(temp));
    jsonField(json, "hum",    hum,    sizeof(hum));
    jsonField(json, "tmp117", tmp117, sizeof(tmp117));
    jsonField(json, "rain",   rain,   sizeof(rain));
    jsonField(json, "leaf",   leaf,   sizeof(leaf));
    jsonField(json, "par",    par,    sizeof(par));
    jsonField(json, "soil",   soil,   sizeof(soil));

    char dateStr[12] = "", timeStr[10] = "";
    int yr = 0, mo = 0, dy = 0;
    if (strlen(ts) >= 19 &&
        sscanf(ts, "%4d-%2d-%2d", &yr, &mo, &dy) == 3 && yr >= 2024) {
        snprintf(dateStr, sizeof(dateStr), "%02d/%02d/%04d", dy, mo, yr);
        strncpy(timeStr, ts + 11, 8); timeStr[8] = '\0';
    } else if (_ctx->currentTime->year > 0) {
        strncpy(dateStr, _ctx->currentTime->dateStr, sizeof(dateStr));
        strncpy(timeStr, _ctx->currentTime->timeStr, sizeof(timeStr));
    }

    char bleCsv[32];
    if (yr >= 2024)
        snprintf(bleCsv, sizeof(bleCsv), "/BLE-%02d-%02d-%04d.csv", dy, mo, yr);
    else if (_ctx->currentTime->year > 0)
        snprintf(bleCsv, sizeof(bleCsv), "/BLE-%02u-%02u-%04u.csv",
                 _ctx->currentTime->date, _ctx->currentTime->month, _ctx->currentTime->year);
    else
        strncpy(bleCsv, "/BLE-data.csv", sizeof(bleCsv));

    if (!LittleFS.exists(bleCsv))
        _ctx->memory->write(LittleFS, bleCsv,
            "Date,Time,Temperature(C),Humidity(%),TMP117(C),"
            "Rainfall,LeafWetness,PAR,SoilMoisture\r\n");

    char row[180];
    snprintf(row, sizeof(row), "%s,%s,%s,%s,%s,%s,%s,%s,%s\r\n",
             dateStr, timeStr, temp, hum, tmp117, rain, leaf, par, soil);
    _ctx->memory->append(LittleFS, bleCsv, row);

    strncpy(_lastSavedTs, ts, sizeof(_lastSavedTs) - 1);
    Serial.printf("[BLE-CSV] Saved to %s: %s", bleCsv, row);
}
