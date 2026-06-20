#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include "sensor_v2.h"
#include "Memory.h"
#include "LoRaE32Handler.h"
#include "LoRaPacket.h"
#include "LoRaPublisher.h"
#include "BleHandler.h"
#include "WifiApServer.h"
#include "TxUtilities.h"
#include "EspNowReceiver.h"

/**
 * @brief Strongly-typed system state enumeration.
 *
 * Represents the complete lifecycle of the weather station:
 * WIFI_AP -> LORA_INIT -> WEATHER -> SOIL -> SAVE -> LORA_PUBLISH -> FINISH
 */
enum class AppState : uint8_t {
    WIFI_AP        = 0,
    LORA_INIT      = 1,
    WEATHER        = 2,
    SOIL           = 3,
    SAVE           = 4,
    LORA_PUBLISH   = 5,
    FINISH         = 6
};

/** @brief Runtime configuration loaded from NVS. */
struct RuntimeConfig {
    uint8_t  soilSlaveId      = SOIL_SLAVE_ID;
    uint8_t  weatherSlaveId   = WEATHER_SLAVE_ID;
    uint8_t  publishBatchSize = PUBLISH_BATCH_SIZE;
    uint8_t  publishMode      = 0;
    bool     sourceModbus     = true;
    bool     sourceBle        = true;
    bool     sourceEspnow     = true;   // accept Sniffer readings over ESP-NOW
    uint16_t fileIntervalMin  = TIME_INCREMENT_MINUTES;
    bool     ntpEnable        = true;
    uint8_t  heartbeatMode    = HEARTBEAT_DEFAULT_MODE;
    uint8_t  heartbeatEveryCycles = HEARTBEAT_DEFAULT_EVERY_CYCLES;
};

/** @brief State timeout descriptor for automatic state advancement. */
struct StateTimeout {
    unsigned long maxDurationMs;
    AppState      fallbackState;
};

/**
 * @brief Top-level application manager for the PAMIANG TX Board.
 *
 * Owns all peripheral drivers, communication handlers, and application state.
 * Manages the main state machine, initialization sequence, and system lifecycle.
 *
 * Architecture:
 *   BLE NUS / WiFi AP ─> [AppManager] ─> RS485 Sensors ─> CSV + LoRa E32
 *
 * Usage:
 *   AppManager app;
 *   app.setup();
 *   for (;;) { app.loop(); }
 */
class AppManager {
public:
    AppManager();

    /** @brief One-time hardware and module initialization. Call from Arduino setup(). */
    void setup();

    /** @brief Run one iteration of the state machine. Call from Arduino loop(). */
    void loop();

private:
    /* ── Module Instances ── */
    RS485sensor     _modbusSensor;
    Memory          _memory;
    LoRaE32Handler  _loraHandler;
    WifiApServer    _wifiApServer;
    BleHandler      _bleHandler;
    LoRaPublisher   _loraPublisher;
    TxUtils         _txUtils;
    Preferences     _nvs;

    /* ── Serial Interfaces ── */
    HardwareSerial  _rs485Serial;
    HardwareSerial  _loraSerial;

    /* ── Application State ── */
    AppState   _currentState = AppState::WIFI_AP;
    AppState   _prevState    = AppState::LORA_INIT;
    unsigned long _stateEntryTime = 0;
    bool       _loraAvailable = false;
    bool       _wifiApStarted = false;
    uint16_t   _batteryVoltage = 0;

    /* ── Time Management ── */
    timeStruct _currentTime;

    /* ── File Paths ── */
    char _dailyCsv[24];
    static constexpr const char* TEMP_DATA_PATH = "/DATA_TEMP.csv";

    /* ── Sensor Timing ── */
    unsigned long _soilStateStart    = 0;
    unsigned long _weatherStateStart = 0;

    /* ── BLE Data ── */
    BleSensorData _lastBleData = {0, 0, 0, 0, 0, 0, 0};
    bool          _lastBleValid = false;

    /* ── Runtime Configuration ── */
    RuntimeConfig _config;
    uint32_t      _heartbeatCycleCount = 0;

    /* ── BLE Auto-Reconnect ── */
    unsigned long _bleAutoReconnectAfterMs = 0;
    bool          _bleRetryScanActive      = false;
    unsigned long _bleRetryScanStart       = 0;
    bool          _bleStoppedAfterNusSample = false;

    /* ── Web Server State ── */
    bool          _webRebootRequest    = false;
    unsigned long _webRebootRequestMs  = 0;
    bool          _bleDisconnectReq    = false;

    /* ── System Status (for WifiApServer) ── */
    SystemStatus  _sysStatus;

    /* ── Context Structs (passed to sub-modules) ── */
    TxUtilsCtx        _txUtilsCtx;
    BleContext        _bleCtx;
    LoRaPublisherCtx  _loraPubCtx;

    /* ═══════════════════════════════════════════════════════
     *  INITIALIZATION (private, called from setup())
     * ═══════════════════════════════════════════════════════ */

    void initHardware();
    void initFilesystem();
    void initModules();
    void initBle();
    void initLoraPublisher();
    void wireSystemStatus();
    void scheduleBleAutoReconnect();

    /* ═══════════════════════════════════════════════════════
     *  STATE MACHINE CORE
     * ═══════════════════════════════════════════════════════ */

    void updateStateEntry();
    bool checkStateTimeout();
    StateTimeout getTimeoutForState(AppState state) const;

    /* ═══════════════════════════════════════════════════════
     *  STATE HANDLERS
     * ═══════════════════════════════════════════════════════ */

    void handleStateWifiAp();
    void handleStateLoraInit();
    void handleStateWeather();
    void handleStateSoil();
    void handleStateSave();
    void handleStateLoraPublish();
    void handleStateFinish();

    /* ═══════════════════════════════════════════════════════
     *  BLE MANAGEMENT (within WIFI_AP state)
     * ═══════════════════════════════════════════════════════ */

    void processBleNotifications();
    void processEspNowData();   /* poll ESP-NOW receiver, fold into BLE data path */
    void processBlePeriodicRefresh();
    void processBleReconnectRetry();
    void processBleBackgroundScan();
    void ensureBleActiveForWebRequest();

    /* ═══════════════════════════════════════════════════════
     *  CONFIGURATION
     * ═══════════════════════════════════════════════════════ */

    void loadRuntimeConfig();
    void saveHeartbeatCycleCount();
    bool shouldPublishHeartbeat(bool dataPublished);

    /* ═══════════════════════════════════════════════════════
     *  LIFECYCLE HELPERS
     * ═══════════════════════════════════════════════════════ */

    void stopWifiApAndBle();
    void stopBleAfterNusSample();
    void clearSoilData();
    void transitionTo(AppState newState);
};
