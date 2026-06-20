#include "AppManager.h"
#include "utilities.h"
#include <esp_task_wdt.h>
#include <esp_sleep.h>
#include <LittleFS.h>

static constexpr uint16_t TEMP_DATA_STALE_MAX_AGE_MINUTES = 1440;
static constexpr unsigned long BLE_AUTO_RECONNECT_GRACE_MS = 8000UL;

static WifiApServer* s_bleYieldWifiApServer = nullptr;

static void serviceWifiApDuringBle() {
    if (s_bleYieldWifiApServer && s_bleYieldWifiApServer->isActive()) {
        s_bleYieldWifiApServer->handleClient();
    }
    delay(0);
}

static bool reconnectScannedBleTarget(BleHandler& bleHandler) {
    if (!bleHandler.wasTargetSeenInLastScan()) return false;

    Serial.printf("[BLE] Target %s found during scan, reconnecting now\n",
                  bleHandler.getTargetMac());
    bleHandler.connectAndRead(bleHandler.getTargetMac());

    if (bleHandler.getConnDev()->connected) {
        Serial.println(F("[BLE] Reconnected from scan"));
        return true;
    }

    Serial.println(F("[BLE] Target found but reconnect failed, continuing scan window"));
    return false;
}


static int pruneStaleTempData(Memory& memory, const char* path, int recordCount) {
    if (recordCount <= 1) return recordCount;

    DataRecord newest;
    memset(&newest, 0, sizeof(newest));
    int latestIndex = recordCount - 1;
    if (latestIndex > 0xFFFF ||
        !memory.readDataRecordAt(path, (uint16_t)latestIndex, &newest)) {
        return recordCount;
    }

    uint16_t removed = memory.removeDataOlderThan(
        path,
        newest,
        TEMP_DATA_STALE_MAX_AGE_MINUTES
    );

    if (removed > 0) {
        Serial.printf("[LoRa] Removed %u stale DATA_TEMP row(s) older than 1 day\n", removed);
        return memory.countDataLines(path);
    }

    return recordCount;
}

/* ═══════════════════════════════════════════════════════════
 *  CONSTRUCTOR
 * ═══════════════════════════════════════════════════════════ */

AppManager::AppManager()
    : _rs485Serial(1)
    , _loraSerial(0)
    , _dailyCsv("/DATA.csv")
{
    memset(&_currentTime, 0, sizeof(_currentTime));
    snprintf(_currentTime.dateStr, sizeof(_currentTime.dateStr), "00/00/0000");
    snprintf(_currentTime.timeStr, sizeof(_currentTime.timeStr), "00:00:00");
}

/* ═══════════════════════════════════════════════════════════
 *  PUBLIC: setup()
 * ═══════════════════════════════════════════════════════════ */

void AppManager::setup() {
    initHardware();
    initFilesystem();
    initModules();
    initBle();
    initLoraPublisher();
    wireSystemStatus();
    scheduleBleAutoReconnect();

    Serial.println(F("===== SETUP COMPLETE ====="));
}

/* ═══════════════════════════════════════════════════════════
 *  PUBLIC: loop()
 * ═══════════════════════════════════════════════════════════ */

void AppManager::loop() {
    _txUtils.feedWDT();
    updateStateEntry();

    if (checkStateTimeout()) return;

    switch (_currentState) {
        case AppState::WIFI_AP:      handleStateWifiAp();       break;
        case AppState::LORA_INIT:    handleStateLoraInit();     break;
        case AppState::WEATHER:      handleStateWeather();      break;
        case AppState::SOIL:         handleStateSoil();         break;
        case AppState::SAVE:         handleStateSave();         break;
        case AppState::LORA_PUBLISH: handleStateLoraPublish();  break;
        case AppState::FINISH:       handleStateFinish();       break;
        default: transitionTo(AppState::FINISH); break;
    }
}

/* ═══════════════════════════════════════════════════════════
 *  INITIALIZATION SEQUENCE
 * ═══════════════════════════════════════════════════════════ */

void AppManager::initHardware() {
    Serial.begin(SERIAL_BAUDRATE);
    delay(500);

    esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
    esp_task_wdt_add(NULL);
    Serial.println(F("[WDT] Watchdog initialized"));
    Serial.printf("[FW] PAMIANG LoRa Weather Station v%s\n", FIRMWARE_VERSION);

    pinMode(BATT_PIN, INPUT);
    pinMode(PIN_DONE, OUTPUT);
    digitalWrite(PIN_DONE, LOW);

    _rs485Serial.begin(SERIAL_RS485, SERIAL_8N1, RS_485_RX_PIN, RS_485_TX_PIN);
    _modbusSensor.begin(&_rs485Serial);
    Serial.println(F("[SERIAL] RS485 UART1 started"));

    _loraSerial.begin(SERIAL_LORA, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
    Serial.println(F("[SERIAL] LoRa UART0 started"));

    _batteryVoltage = batteryRead();
    Serial.printf("[BATT] %u mV\n", _batteryVoltage);

    if (_batteryVoltage < LOW_BATTERY_SHUTDOWN_MV) {
        Serial.printf("[BATT] Low battery (%u mV < %u mV), shutting down\n",
                      _batteryVoltage, LOW_BATTERY_SHUTDOWN_MV);
        Serial.flush();
        _txUtils.pulseDoneTPL5110();

        Serial.println(F("[WARN] TPL5110 did not cut power after low battery, entering deep sleep"));
        Serial.flush();
        esp_deep_sleep_start();
        while (1) { delay(1000); }
    }
}

void AppManager::initFilesystem() {
    _txUtils.feedWDT();

    if (!LittleFS.begin(true)) {
        Serial.println(F("[FS] LittleFS FAILED"));
        return;
    }
    Serial.println(F("[FS] LittleFS  mounted"));

    _txUtils.feedWDT();

    if (!LittleFS.exists(TEMP_DATA_PATH)) {
        _memory.write(LittleFS, TEMP_DATA_PATH,
            "Date,Time,"
            "Soil_Humidity,Soil_Temperature,EC,PH,N,P,K,"
            "WindSpeed,WindDirection,Air_Humidity,Air_Temperature,"
            "CO2,Pressure,Illuminance,Rainfall,Solar,"
            "BLE_Temp,BLE_Humi,BLE_TMP117,BLE_Rain,BLE_Leaf,BLE_PAR,BLE_Soil\r\n");
    } else {
        File tf = LittleFS.open(TEMP_DATA_PATH, "r");
        String hdr = tf.readStringUntil('\n');
        tf.close();
        if (hdr.indexOf("BLE_Temp") < 0 || hdr.indexOf("BLE_DeltaT") >= 0) {
            LittleFS.remove(TEMP_DATA_PATH);
            _memory.write(LittleFS, TEMP_DATA_PATH,
                "Date,Time,"
                "Soil_Humidity,Soil_Temperature,EC,PH,N,P,K,"
                "WindSpeed,WindDirection,Air_Humidity,Air_Temperature,"
                "CO2,Pressure,Illuminance,Rainfall,Solar,"
                "BLE_Temp,BLE_Humi,BLE_TMP117,BLE_Rain,BLE_Leaf,BLE_PAR,BLE_Soil\r\n");
            Serial.println(F("[FS] Temp file recreated with BLE columns"));
        }
    }
    delay(100);
}

void AppManager::initModules() {
    _txUtilsCtx.nvs         = &_nvs;
    _txUtilsCtx.memory      = &_memory;
    _txUtilsCtx.modbusSensor = &_modbusSensor;
    _txUtilsCtx.currentTime  = &_currentTime;
    _txUtilsCtx.dailyCsv     = _dailyCsv;
    _txUtils.begin(&_txUtilsCtx);

    loadRuntimeConfig();
}

void AppManager::initBle() {
    _bleCtx.currentTime  = &_currentTime;
    _bleCtx.lastBleData  = &_lastBleData;
    _bleCtx.lastBleValid = &_lastBleValid;
    _bleCtx.memory       = &_memory;
    _bleCtx.nvs          = &_nvs;
    _bleHandler.begin(&_bleCtx);

    _txUtils.feedWDT();
    _bleHandler.init();
    _txUtils.feedWDT();
}

void AppManager::initLoraPublisher() {
    _loraPubCtx.loraHandler    = &_loraHandler;
    _loraPubCtx.loraSerial     = &_loraSerial;
    _loraPubCtx.loraAvailable  = &_loraAvailable;
    _loraPubCtx.batteryVoltage = &_batteryVoltage;
    _loraPubCtx.memory         = &_memory;
    _loraPubCtx.nvs            = &_nvs;
    _loraPubCtx.tempDataPath   = TEMP_DATA_PATH;
    _loraPubCtx.publishBatchSize = &_config.publishBatchSize;
    _loraPublisher.begin(&_loraPubCtx);
}

void AppManager::wireSystemStatus() {
    _sysStatus.sensor             = &_modbusSensor.currentSensor;
    _sysStatus.time               = &_currentTime;
    _sysStatus.battMv             = &_batteryVoltage;
    _sysStatus.bleActive          = _bleHandler.getIsActive();
    _sysStatus.memory             = &_memory;
    _sysStatus.bleDevices         = _bleHandler.getDevices();
    _sysStatus.bleDeviceCount     = _bleHandler.getDeviceCount();
    _sysStatus.bleMac[0]          = '\0';
    _sysStatus.bleConnDev         = _bleHandler.getConnDev();
    _sysStatus.bleTargetMac       = _bleHandler.getTargetMac();
    _sysStatus.bleConnPending     = _bleHandler.getConnPending();
    _sysStatus.bleSavedMac        = _bleHandler.getSavedMac();
    _sysStatus.bleScanRequest     = _bleHandler.getScanRequest();
    _sysStatus.bleIsNus           = _bleHandler.getIsNus();
    _sysStatus.bleNusLastMs       = _bleHandler.getNusLastMs();
    _sysStatus.bleNusRefreshReq   = _bleHandler.getNusRefreshReq();
    _sysStatus.bleDisconnectRequest = &_bleDisconnectReq;
    _sysStatus.rebootRequest      = &_webRebootRequest;

    if (!*_sysStatus.bleActive || _sysStatus.bleMac[0] == '\0')
        strncpy(_sysStatus.bleMac, "N/A", sizeof(_sysStatus.bleMac));
}

void AppManager::scheduleBleAutoReconnect() {
    _nvs.begin("ws-cfg", true);
    String savedMac = _nvs.getString("bleSavedMac", "");
    _nvs.end();

    if (savedMac.length() == 17) {
        char mac[18];
        strncpy(mac, savedMac.c_str(), sizeof(mac));
        mac[17] = '\0';
        strncpy(_bleHandler.getTargetMac(), mac, 18);
        strncpy(_bleHandler.getSavedMac(), mac, 18);
        *_bleHandler.getConnPending() = true;
        _bleAutoReconnectAfterMs = millis() + BLE_AUTO_RECONNECT_GRACE_MS;
        Serial.printf("[BLE] Auto-reconnect scheduled: %s\n", mac);
    }
}

/* ═══════════════════════════════════════════════════════════
 *  STATE MACHINE CORE
 * ═══════════════════════════════════════════════════════════ */

void AppManager::updateStateEntry() {
    if (_currentState != _prevState) {
        _stateEntryTime = millis();
        _prevState = _currentState;
    }
}

StateTimeout AppManager::getTimeoutForState(AppState state) const {
    switch (state) {
        case AppState::WIFI_AP:      return {WIFI_AP_TIMEOUT,       AppState::LORA_INIT};
        case AppState::LORA_INIT:    return {LORA_INIT_TIMEOUT_MS,  AppState::WEATHER};
        case AppState::WEATHER:      return {WEATHER_TIMEOUT,       AppState::SOIL};
        case AppState::SOIL:         return {SOIL_TIMEOUT,          AppState::SAVE};
        case AppState::SAVE:         return {10000UL,               AppState::LORA_PUBLISH};
        case AppState::LORA_PUBLISH: return {LORA_TX_TIMEOUT_MS,    AppState::FINISH};
        default:                     return {0UL,                   AppState::FINISH};
    }
}

bool AppManager::checkStateTimeout() {
    StateTimeout tmo = getTimeoutForState(_currentState);
    if (tmo.maxDurationMs == 0) return false;

    unsigned long elapsed = millis() - _stateEntryTime;
    if (elapsed < tmo.maxDurationMs) return false;

    if (_currentState == AppState::WIFI_AP && _bleRetryScanActive) {
        _stateEntryTime = millis();
        return false;
    }

    Serial.printf("[TMO] State %d timeout %lu ms -> %d\n",
                  (int)_currentState, elapsed, (int)tmo.fallbackState);

    if (_currentState == AppState::WIFI_AP && (_wifiApStarted || *_bleHandler.getIsActive())) {
        stopWifiApAndBle();
    }

    transitionTo(tmo.fallbackState);
    return true;
}

void AppManager::transitionTo(AppState newState) {
    if (_currentState != newState) {
        Serial.printf("[STATE] %d -> %d\n", (int)_currentState, (int)newState);
        _currentState = newState;
        _stateEntryTime = millis();
        _prevState = newState;
    }
}

/* ═══════════════════════════════════════════════════════════
 *  STATE HANDLERS
 * ═══════════════════════════════════════════════════════════ */

void AppManager::handleStateWifiAp() {
    if (!_wifiApStarted) {
        _wifiApStarted = true;
        *_bleHandler.getDataReceived() = false;
        _lastBleValid = false;
        memset(&_lastBleData, 0, sizeof(_lastBleData));
        _bleRetryScanActive = false;
        _bleRetryScanStart  = 0;
        _bleStoppedAfterNusSample = false;
        _webRebootRequest   = false;
        _webRebootRequestMs = 0;
        _bleDisconnectReq   = false;
        if (_wifiApServer.begin(&_sysStatus)) {
            s_bleYieldWifiApServer = &_wifiApServer;
            _bleHandler.setYieldFn(serviceWifiApDuringBle);
        }
        if (_config.sourceEspnow) espnowRxBegin();  // listen on the SoftAP channel
    }

    _wifiApServer.handleClient();

    if (_webRebootRequest) {
        if (_webRebootRequestMs == 0) {
            _webRebootRequestMs = millis();
            Serial.println(F("[WEB] Reboot queued"));
        }
        if (millis() - _webRebootRequestMs >= 1500UL) {
            Serial.println(F("[WEB] Rebooting now"));
            Serial.flush();
            ESP.restart();
        }
        delay(2);
        return;
    }

    if (_bleDisconnectReq) {
        _bleDisconnectReq = false;
        _bleHandler.deinit();
        _bleStoppedAfterNusSample = false;
    }

    if (*_bleHandler.getScanRequest()) {
        *_bleHandler.getScanRequest() = false;
        ensureBleActiveForWebRequest();
        _bleHandler.doScan();
    }

    if (*_bleHandler.getConnPending()) {
        if (_bleAutoReconnectAfterMs != 0 &&
            (long)(millis() - _bleAutoReconnectAfterMs) < 0) {
            delay(2);
            return;
        }

        _bleAutoReconnectAfterMs = 0;
        *_bleHandler.getConnPending() = false;
        ensureBleActiveForWebRequest();
        _bleHandler.connectAndRead(_bleHandler.getTargetMac());
        if (!_bleHandler.getConnDev()->connected) {
            Serial.println(F("[BLE] Retry failed, scanning for 1 minute"));
            _bleRetryScanActive = true;
            _bleRetryScanStart  = millis();
            _bleHandler.doScan();
            if (reconnectScannedBleTarget(_bleHandler)) {
                _bleRetryScanActive = false;
                _bleRetryScanStart  = 0;
            }
        } else {
            _bleRetryScanActive = false;
            _bleRetryScanStart  = 0;
        }
    }

    processBleNotifications();
    processEspNowData();
    processBlePeriodicRefresh();
    processBleReconnectRetry();
    processBleBackgroundScan();

    if (_wifiApServer.isTimedOut()) {
        stopWifiApAndBle();
        transitionTo(AppState::LORA_INIT);
    }

    delay(2);
}

void AppManager::handleStateLoraInit() {
    loadRuntimeConfig();
    delay(1000);

    if (_loraHandler.init(_loraSerial)) {
        _loraAvailable = true;
        Serial.println(F("[LoRa] E32-433T30S initialized OK"));
    } else {
        _loraAvailable = false;
        Serial.println(F("[LoRa] E32 init FAILED — will retry on publish"));
    }

    if (_currentTime.year == 0 || _currentTime.year < 2024) {
        if (!_txUtils.readLastTimeFromBackup(&_currentTime)) {
            memset(&_currentTime, 0, sizeof(timeStruct));
            snprintf(_currentTime.dateStr, sizeof(_currentTime.dateStr), "00/00/0000");
            snprintf(_currentTime.timeStr, sizeof(_currentTime.timeStr), "00:00:00");
        }
        _txUtils.incrementTime(&_currentTime, _config.fileIntervalMin);
        Serial.printf("[TIME] Fallback from LittleFS: %s %s\n",
                      _currentTime.dateStr, _currentTime.timeStr);
    } else {
        Serial.printf("[TIME] Using BLE time: %s %s\n",
                      _currentTime.dateStr, _currentTime.timeStr);
    }
    _txUtils.updateDailyCsv();

    transitionTo(AppState::WEATHER);
}

void AppManager::handleStateWeather() {
    if (_wifiApStarted || *_bleHandler.getIsActive()) {
        stopWifiApAndBle();
        Serial.println(F("[WiFi/BLE] Stopped before sensor read"));
    }

    if (!_config.sourceModbus) {
        _txUtils.clearWeatherData();
        clearSoilData();
        Serial.println(F("[SRC] Modbus disabled, skipping RS485 reads"));
        transitionTo(AppState::SAVE);
        return;
    }

    if (_weatherStateStart == 0) _weatherStateStart = millis();
    if (millis() - _weatherStateStart < WEATHER_SETTLE_DELAY) {
        _txUtils.feedWDT();
        delay(10);
        return;
    }

    if (_modbusSensor.read(WEATHER, _config.weatherSlaveId,
                           WEATHER_REGISTER_ADDR, WEATHER_REGISTER_LEN,
                           &_rs485Serial)) {
        if (DEBUG) {
            Serial.println(F("Weather Success!"));
            Serial.printf("  Wind Speed: %.1f m/s\n", _modbusSensor.currentSensor.windSpeed / 10.0);
            Serial.printf("  Wind Dir: %u Deg\n", _modbusSensor.currentSensor.windDir_Deg);
            Serial.printf("  Air Humidity: %.1f%%\n", _modbusSensor.currentSensor.air_humidity / 10.0);
            Serial.printf("  Air Temp: %.1f C\n", _modbusSensor.currentSensor.air_temperature / 10.0);
            Serial.printf("  CO2: %u ppm\n", _modbusSensor.currentSensor.CO2);
            Serial.printf("  Pressure: %.1f kPa\n", _modbusSensor.currentSensor.pressure / 10.0);
            Serial.printf("  Illuminance: %lu lux\n", (unsigned long)_modbusSensor.currentSensor.illuminance);
            Serial.printf("  Rainfall: %.1f mm\n", _modbusSensor.currentSensor.rainfall / 10.0);
            Serial.printf("  Solar: %u W/m2\n", _modbusSensor.currentSensor.solar);
        }
        _weatherStateStart = 0;
        transitionTo(AppState::SOIL);
        return;
    }

    if (millis() - _weatherStateStart >= WEATHER_TIMEOUT) {
        _txUtils.clearWeatherData();
        _weatherStateStart = 0;
        transitionTo(AppState::SOIL);
    }
}

void AppManager::handleStateSoil() {
    if (_soilStateStart == 0) _soilStateStart = millis();
    if (millis() - _soilStateStart < SOIL_SETTLE_DELAY) {
        _txUtils.feedWDT();
        delay(10);
        return;
    }

    if (_modbusSensor.read(SOIL, _config.soilSlaveId,
                           SOIL_REGISTER_ADDR, SOIL_REGISTER_LEN,
                           &_rs485Serial)) {
        if (DEBUG) {
            Serial.println(F("Soil Success!"));
            Serial.printf("  Moisture: %.1f%%\n", _modbusSensor.currentSensor.soil_humi / 10.0);
            Serial.printf("  Temp: %.1f C\n", _modbusSensor.currentSensor.soil_temp / 10.0);
            Serial.printf("  EC: %u uS/cm\n", _modbusSensor.currentSensor.soil_ec);
            Serial.printf("  PH: %.1f\n", _modbusSensor.currentSensor.soil_ph / 10.0);
            Serial.printf("  N: %u mg/kg\n", _modbusSensor.currentSensor.soil_N);
            Serial.printf("  P: %u mg/kg\n", _modbusSensor.currentSensor.soil_P);
            Serial.printf("  K: %u mg/kg\n", _modbusSensor.currentSensor.soil_K);
        }
        _soilStateStart = 0;
        transitionTo(AppState::SAVE);
        return;
    }

    if (millis() - _soilStateStart >= SOIL_TIMEOUT) {
        _soilStateStart = 0;
        transitionTo(AppState::SAVE);
    }
}

void AppManager::handleStateSave() {
    _nvs.begin("ws-cfg", true);
    uint8_t rollPct = (uint8_t)_nvs.getUInt("memRollover", STORAGE_ROLLOVER_PERCENT);
    bool rollEn = _nvs.getBool("memRolloverEn", true);
    _nvs.end();

    if (rollEn && _memory.isUsageOverThreshold(rollPct)) {
        String last = _txUtils.getLastDataLine(_dailyCsv);
        LittleFS.remove(_dailyCsv);
        _txUtils.updateDailyCsv();
        if (last.length()) {
            String e = last + "\r\n";
            _memory.append(LittleFS, _dailyCsv, e.c_str());
        }
    }

    // _lastBleData is fed by either the BLE-NUS pull or the ESP-NOW receiver.
    bool wantBleData = _config.sourceBle || _config.sourceEspnow;
    BleSensorData* blePtr = (wantBleData && _lastBleValid) ? &_lastBleData : nullptr;
    if (wantBleData && !_lastBleValid) {
        Serial.println(F("[BLE] No fresh BLE/ESP-NOW data this cycle, saving without those values"));
    }

    if (_memory.saveData(_dailyCsv, &_currentTime, &_modbusSensor.currentSensor, blePtr))
        Serial.println(F("[SAVE] Daily CSV OK"));

    if (_memory.saveData(TEMP_DATA_PATH, &_currentTime, &_modbusSensor.currentSensor, blePtr))
        Serial.println(F("[SAVE] Temp saved"));
    else
        Serial.println(F("[SAVE] Temp save FAILED"));

    transitionTo(AppState::LORA_PUBLISH);
}

void AppManager::handleStateLoraPublish() {
    bool dataPublished = false;
    bool loraTxOk = false;

    if (_config.publishMode == 0) {
        int targetBatch = (_config.publishBatchSize > 0) ? _config.publishBatchSize : PUBLISH_MAX_RECORDS;
        targetBatch = min(targetBatch, (int)PUBLISH_MAX_RECORDS);

        int tempRecords = _memory.countDataLines(TEMP_DATA_PATH);
        tempRecords = pruneStaleTempData(_memory, TEMP_DATA_PATH, tempRecords);
        if (tempRecords < targetBatch) {
            Serial.printf("[LoRa] Stack %d/%d records in temp file, waiting for full batch.\n",
                          tempRecords, targetBatch);
        } else if (_loraPublisher.publishData()) {
            dataPublished = true;
            loraTxOk = true;
            Serial.println(F("[LoRa] Data published OK."));
        } else {
            Serial.println(F("[LoRa] Publish failed. Data kept for next cycle."));
        }
    } else {
        Serial.println(F("[LoRa] Publish mode reserved, keeping data."));
    }

    if (shouldPublishHeartbeat(dataPublished)) {
        if (_loraPublisher.publishHeartbeat()) {
            loraTxOk = true;
        }
    }

    if (loraTxOk) {
        Serial.printf("[LoRa] TX settle %lu ms before shutdown\n", LORA_POST_TX_SETTLE_MS);
        unsigned long settleStart = millis();
        while (millis() - settleStart < LORA_POST_TX_SETTLE_MS) {
            _txUtils.feedWDT();
            delay(10);
        }
    }

    Serial.flush();
    transitionTo(AppState::FINISH);
}

void AppManager::handleStateFinish() {
    Serial.println(F("[DONE] Pulsing TPL5110"));
    Serial.flush();
    _txUtils.pulseDoneTPL5110();

    Serial.println(F("[WARN] TPL5110 did not cut power, entering deep sleep"));
    Serial.flush();
    esp_deep_sleep_start();

    while (1) { delay(1000); }
}

/* ═══════════════════════════════════════════════════════════
 *  BLE MANAGEMENT HELPERS (within WIFI_AP state)
 * ═══════════════════════════════════════════════════════════ */

void AppManager::processBleNotifications() {
    if (!*_bleHandler.getIsNus()) return;
    bool hadData = *_bleHandler.getDataReceived();
    _bleHandler.handleNusNotification();
    if (!hadData && *_bleHandler.getDataReceived()) {
        stopBleAfterNusSample();
    }
}

void AppManager::processEspNowData() {
    if (!espnowRxReady()) return;
    BleSensorData rx;
    if (!espnowRxPoll(rx)) return;          // nothing new this tick
    _lastBleData  = rx;                       // fold into the shared Sniffer-data slot
    _lastBleValid = true;                     // picked up by handleStateSave()
}

void AppManager::processBlePeriodicRefresh() {
    bool isNus = *_bleHandler.getIsNus();
    unsigned long refreshInterval = isNus ? BLE_NUS_SAMPLE_INTERVAL_MS : 5000UL;
    unsigned long* lastRefresh = _bleHandler.getLastRefresh();

    if (!_bleHandler.getConnDev()->connected) return;
    if (millis() - *lastRefresh < refreshInterval) return;

    *lastRefresh = millis();

    if (*_bleHandler.getNusRefreshReq()) {
        *_bleHandler.getNusRefreshReq() = false;
    }

    _bleHandler.refreshData();
}

void AppManager::processBleReconnectRetry() {
    if (!_bleRetryScanActive) return;

    if (millis() - _bleRetryScanStart >= BLE_RETRY_FAILED_SCAN_MS) {
        Serial.println(F("[BLE] Retry-failed scan window done, advancing to LoRa init"));
        stopWifiApAndBle();
        transitionTo(AppState::LORA_INIT);
        return;
    }

    if (*_bleHandler.getIsActive() && !_bleHandler.getConnDev()->connected) {
        static unsigned long lastScan = 0;
        if (millis() - lastScan >= BLE_RETRY_SCAN_INTERVAL_MS) {
            lastScan = millis();
            _bleHandler.doScan();
            if (reconnectScannedBleTarget(_bleHandler)) {
                _bleRetryScanActive = false;
                _bleRetryScanStart  = 0;
            }
        }
    }
}

void AppManager::processBleBackgroundScan() {
    if (_bleRetryScanActive) return;
    if (!*_bleHandler.getIsActive()) return;
    if (_bleHandler.getConnDev()->connected) return;

    static unsigned long lastBackgroundScan = 0;
    if (millis() - lastBackgroundScan >= 30000UL) {
        lastBackgroundScan = millis();
        _bleHandler.doScan();
        reconnectScannedBleTarget(_bleHandler);
    }
}

void AppManager::ensureBleActiveForWebRequest() {
    if (*_bleHandler.getIsActive()) return;

    _bleHandler.init();
    if (*_bleHandler.getIsActive()) {
        s_bleYieldWifiApServer = &_wifiApServer;
        _bleHandler.setYieldFn(serviceWifiApDuringBle);
        *_bleHandler.getDataReceived() = false;
        _bleStoppedAfterNusSample = false;
    }
}

/* ═══════════════════════════════════════════════════════════
 *  CONFIGURATION
 * ═══════════════════════════════════════════════════════════ */

void AppManager::loadRuntimeConfig() {
    _txUtils.loadRuntimeConfig(&_config.soilSlaveId, &_config.weatherSlaveId,
                               &_config.publishBatchSize, &_config.publishMode);

    _nvs.begin("ws-cfg", true);
    _config.sourceModbus    = _nvs.getBool("srcModbus", true);
    _config.sourceBle       = _nvs.getBool("srcBle", true);
    _config.sourceEspnow    = _nvs.getBool("srcEsp", true);
    _config.fileIntervalMin = (uint16_t)_nvs.getUInt("fileInterval", TIME_INCREMENT_MINUTES);
    _config.ntpEnable       = _nvs.getBool("ntpEnable", true);
    _config.heartbeatMode   = _nvs.getUChar("hbMode", HEARTBEAT_DEFAULT_MODE);
    _config.heartbeatEveryCycles = _nvs.getUChar("hbEvery", HEARTBEAT_DEFAULT_EVERY_CYCLES);
    _heartbeatCycleCount    = _nvs.getUInt("hbCycle", 0);
    _nvs.end();

    if (_config.fileIntervalMin < 1)     _config.fileIntervalMin = 1;
    if (_config.fileIntervalMin > 1440)  _config.fileIntervalMin = 1440;
    if (_config.heartbeatMode > HEARTBEAT_MODE_OFF)
        _config.heartbeatMode = HEARTBEAT_DEFAULT_MODE;
    if (_config.heartbeatEveryCycles < 1)
        _config.heartbeatEveryCycles = 1;
}

/* ═══════════════════════════════════════════════════════════
 *  LIFECYCLE HELPERS
 * ═══════════════════════════════════════════════════════════ */

void AppManager::saveHeartbeatCycleCount() {
    if (!_nvs.begin("ws-cfg", false)) return;
    _nvs.putUInt("hbCycle", _heartbeatCycleCount);
    _nvs.end();
}

bool AppManager::shouldPublishHeartbeat(bool dataPublished) {
    switch (_config.heartbeatMode) {
        case HEARTBEAT_MODE_EVERY_CYCLE:
            return true;

        case HEARTBEAT_MODE_NO_DATA_ONLY:
            return !dataPublished;

        case HEARTBEAT_MODE_OFF:
            return false;

        case HEARTBEAT_MODE_EVERY_N_CYCLES:
        case HEARTBEAT_MODE_N_CYCLES_NO_DATA: {
            if (_config.heartbeatMode == HEARTBEAT_MODE_N_CYCLES_NO_DATA && dataPublished) {
                if (_heartbeatCycleCount != 0) {
                    _heartbeatCycleCount = 0;
                    saveHeartbeatCycleCount();
                }
                return false;
            }

            _heartbeatCycleCount++;
            bool due = (_heartbeatCycleCount >= _config.heartbeatEveryCycles);
            if (due) {
                _heartbeatCycleCount = 0;
            }
            saveHeartbeatCycleCount();
            return due;
        }

        default:
            return false;
    }
}

void AppManager::stopWifiApAndBle() {
    _bleRetryScanActive = false;
    _bleRetryScanStart  = 0;
    _bleAutoReconnectAfterMs = 0;
    _bleHandler.setYieldFn(nullptr);
    s_bleYieldWifiApServer = nullptr;

    _bleHandler.deinit();
    espnowRxStop();                 // release ESP-NOW before the radio goes down
    delay(50);

    if (_wifiApStarted || _wifiApServer.isActive()) {
        _wifiApServer.stop();
        _wifiApStarted = false;
    }

    _wifiApStarted = false;
}

void AppManager::stopBleAfterNusSample() {
    if (!*_bleHandler.getIsActive()) return;

    Serial.println(F("[BLE] NUS sample received, stopping BLE radio; Web server remains active"));
    _bleRetryScanActive = false;
    _bleRetryScanStart  = 0;
    _bleAutoReconnectAfterMs = 0;
    _bleHandler.setYieldFn(nullptr);
    s_bleYieldWifiApServer = nullptr;
    _bleHandler.deinit();
    _bleStoppedAfterNusSample = true;
}

void AppManager::clearSoilData() {
    SensorData& s = _modbusSensor.currentSensor;
    s.soil_humi = 0;
    s.soil_temp = 0;
    s.soil_ec   = 0;
    s.soil_ph   = 0;
    s.soil_N    = 0;
    s.soil_P    = 0;
    s.soil_K    = 0;
}
