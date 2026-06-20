#pragma once
#include <stdint.h>

typedef struct {
    uint16_t soil_humi;
    int16_t  soil_temp;
    uint16_t soil_ec;
    uint8_t  soil_ph;
    uint16_t soil_N;
    uint16_t soil_P;
    uint16_t soil_K;
    uint16_t windSpeed;
    uint16_t windDir_Deg;
    uint16_t air_humidity;
    int16_t  air_temperature;
    uint16_t CO2;
    uint16_t pressure;
    uint32_t illuminance;
    uint16_t rainfall;
    uint16_t solar;
} BleTransport_SensorData;

typedef struct {
    int16_t  ble_temp;
    uint16_t ble_humi;
    int16_t  ble_tmp117;
    uint16_t ble_rain;
    uint16_t ble_leaf;
    uint16_t ble_par;
    uint16_t ble_soil;
} BleTransport_BleSensorData;

typedef struct {
    uint8_t hour;
    uint8_t minute;
    BleTransport_SensorData data;
    BleTransport_BleSensorData ble;
    uint8_t ble_valid;
    uint8_t valid;
    uint8_t checksum;
} BleTransport_DataRecord;

typedef struct {
    uint8_t  header1;
    uint8_t  header2;
    uint8_t  nodeID;
    uint8_t  date;
    uint8_t  month;
    uint8_t  year;
    uint16_t batteryVoltage;
    BleTransport_DataRecord records[6];
    uint16_t checksum;
} BleTransport_LoRaBatchPacket;

typedef struct {
    uint8_t  header1;
    uint8_t  header2;
    uint8_t  nodeID;
    uint16_t dataLen;
    BleTransport_LoRaBatchPacket packet;
    uint16_t checksum;
} BleTransport_UARTPacket;

typedef struct __attribute__((packed)) {
    uint8_t  header1;
    uint8_t  header2;
    uint8_t  cmdType;
    uint8_t  nodeID;
    uint16_t sequence;
    uint16_t batteryVoltage;
    uint32_t uptime;
    uint16_t heap;
    uint8_t  loraAvailable;
    uint8_t  bleConnected;
    uint16_t retryQueueDepth;
    uint32_t lastLoRaPacketAgeSec;
    uint16_t checksum;
} BleTransport_RxHeartbeatPacket;

#define BLE_PKT_HEADER1      0xAA
#define BLE_PKT_HEADER2      0xBB
#define BLE_PKT_TYPE_RX_HEARTBEAT 0xE2
#define BLE_PKT_RECORD_VALID 0xA5
#define BLE_PKT_NODE_ID      2
#define BLE_PKT_MAX_RECORDS  6
