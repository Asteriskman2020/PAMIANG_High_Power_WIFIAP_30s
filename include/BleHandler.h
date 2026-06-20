/* BLE handler — NimBLE scan, connect, NUS/GATT data collection.
 * Extracted from main.cpp. Encapsulates all BLE state and logic. */
#ifndef BLE_HANDLER_H_
#define BLE_HANDLER_H_

#include <Arduino.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include "sensor_v2.h"
#include "Memory.h"
#include "WifiApServer.h"   /* BLEFoundDevice, BLEConnectedDevice */

/* NUS UUIDs */
#define NUS_SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_TX_UUID      "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_RX_UUID      "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_BUF 400

/* Context: pointers to globals in main.cpp that BLE writes to.
 * Fill in setup() before calling begin(). */
struct BleContext {
    timeStruct*    currentTime;
    BleSensorData* lastBleData;
    bool*          lastBleValid;
    Memory*        memory;
    Preferences*   nvs;
};

class BleHandler {
public:
    BleHandler();

    /* Call once in setup() after filling BleContext */
    void begin(BleContext* ctx);

    /* ── Public API ── */
    void init();                          /* init NimBLE stack */
    void deinit();                        /* clean up NimBLE — call before state transition */
    void doScan();                        /* 3-second BLE scan */
    bool wasTargetSeenInLastScan() const; /* true if target MAC advertised in the last scan */
    void connectAndRead(const char* mac); /* connect to device, read data */
    void refreshData();                   /* periodic refresh of connected device */
    void handleNusNotification();         /* call from loop() when NUS data ready */

    /* ── Yield callback — keeps web server alive during BLE blocking ops ── */
    void setYieldFn(void (*fn)()) { _yieldFn = fn; }

    /* ── Accessors for SystemStatus wiring in main.cpp ── */
    BLEFoundDevice*     getDevices();
    uint8_t*            getDeviceCount();
    BLEConnectedDevice* getConnDev();
    char*               getTargetMac();
    char*               getSavedMac();
    bool*               getConnPending();
    bool*               getScanRequest();
    bool*               getDataReceived();
    bool*               getIsActive();
    bool*               getIsNus();
    unsigned long*      getNusLastMs();
    bool*               getNusRefreshReq();
    volatile bool*      getNusNotified();
    NimBLERemoteCharacteristic* getNusRxChar();
    unsigned long*      getLastRefresh();

private:
    BleContext* _ctx;
    bool        _active;    /* NimBLE initialized */

    /* ── Scan state ── */
    BLEFoundDevice  _devices[BLE_MAX_DEVICES];
    uint8_t         _deviceCount;
    unsigned long   _lastScan;
    bool            _targetSeenInScan;

    /* ── Connected device state ── */
    BLEConnectedDevice         _connDev;
    char                       _targetMac[18];
    char                       _savedMac[18];
    bool                       _connPending;
    bool                       _scanRequest;
    bool                       _dataReceived;
    NimBLEClient*              _persistClient;
    NimBLERemoteCharacteristic* _charPtrs[BLE_CHAR_MAX];
    unsigned long              _lastRefresh;

    /* ── NUS state ── */
    bool                       _isNus;
    char                       _nusAccum[NUS_BUF];
    int                        _nusAccumLen;
    char                       _nusJsonReady[NUS_BUF];
    volatile bool              _nusNotified;
    NimBLERemoteCharacteristic* _nusRxChar;
    unsigned long              _nusLastMs;
    unsigned long              _nusLastAcceptedMs;
    bool                       _nusRefreshReq;

    /* ── Yield callback for keeping web server responsive ── */
    void (*_yieldFn)() = nullptr;

    /* ── BLE data CSV dedup ── */
    char _lastSavedTs[20];

    /* ── Internal helpers ── */
    static void jsonField(const char* j, const char* key, char* out, size_t outLen);
    void parseNusJson(const char* json);
    void saveBleDataToCsv(const char* json);

    /* ── Scan callback ── */
    class ScanCB : public NimBLEAdvertisedDeviceCallbacks {
        BleHandler* _parent;
    public:
        ScanCB(BleHandler* p) : _parent(p) {}
        void onResult(NimBLEAdvertisedDevice* dev) override;
    };
    ScanCB _scanCB;

    /* Allow static trampoline to access private members */
    friend void nusTrampoline(NimBLERemoteCharacteristic*,
                               uint8_t*, size_t, bool);
};

#endif
