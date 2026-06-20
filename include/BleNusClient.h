#pragma once

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "BlePacketTypes.h"

class BleNusClient {
public:
    BleNusClient();
    ~BleNusClient();

    bool begin(const char* targetName, const char* serviceUuid,
               const char* rxCharUuid, const char* txCharUuid);
    void loop();

    bool sendPacket(const BleTransport_UARTPacket& packet);
    bool sendBytes(const uint8_t* data, size_t len);
    bool isConnected() const;
    void disconnect();

    uint32_t getTotalSent()     const;
    uint32_t getTotalErrors()   const;
    uint32_t getTotalDropped()  const;

private:
    enum BleState {
        STATE_UNINIT,
        STATE_IDLE,
        STATE_SCANNING,
        STATE_CONNECTING,
        STATE_CONNECTED,
        STATE_WAIT_RECONNECT
    };

    NimBLEClient*              _pClient;
    NimBLERemoteCharacteristic* _pWriteChar;
    NimBLERemoteCharacteristic* _pNotifyChar;
    NimBLEAddress              _targetAddress;
    bool                       _addressKnown;

    BleState  _state;
    uint32_t  _lastAttemptMs;
    uint32_t  _lastConnectMs;
    uint16_t  _failedAttempts;
    bool      _initialized;
    bool      _busy;

    char _targetName[32];
    char _serviceUuid[40];
    char _rxCharUuid[40];
    char _txCharUuid[40];

    uint32_t _totalSent;
    uint32_t _totalErrors;
    uint32_t _totalDropped;

    volatile bool _ackReceived;
    uint8_t       _ackValue;

    static constexpr uint16_t BLE_SCAN_DURATION_S     = 5;
    static constexpr uint32_t BLE_RECONNECT_MIN_MS    = 30000;
    static constexpr uint32_t BLE_RECONNECT_MAX_MS    = 300000;
    static constexpr uint16_t BLE_CONNECT_TIMEOUT_S   = 15;
    static constexpr uint16_t BLE_MAX_FAILED_ATTEMPTS = 10;
    static constexpr uint16_t BLE_WRITE_TIMEOUT_MS    = 8000;
    static constexpr uint16_t BLE_MTU_REQUEST         = 512;

    bool doScan();
    bool doConnect();
    bool discoverServices();
    void transitionState(BleState newState);

    static uint16_t calculateChecksum(const uint8_t* data, size_t len);

    class ClientCallbacks : public NimBLEClientCallbacks {
    public:
        BleNusClient* parent;
        ClientCallbacks(BleNusClient* p) : parent(p) {}
        void onConnect(NimBLEClient* pClient) override;
        void onDisconnect(NimBLEClient* pClient) override;
    };

    ClientCallbacks _clientCB;

    class ScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
    public:
        BleNusClient* parent;
        ScanCallbacks(BleNusClient* p) : parent(p) {}
        void onResult(NimBLEAdvertisedDevice* dev) override;
    };

    ScanCallbacks _scanCB;
    volatile bool _scanFound;
    NimBLEAddress _scanAddress;

    static void notifyCallback(NimBLERemoteCharacteristic* pChar,
                                uint8_t* pData, size_t length, bool isNotify);
};
