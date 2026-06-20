/* Config Parameters — PAMIANG RX Board (LoRa receiver + BLE relay gateway)
 *
 * Hardware: Seeed XIAO ESP32-C3
 *   UART0  → USB Serial (debug)
 *   UART1  → LoRa E32-433T30S (D5 RX, D4 TX)
 *   A1     → Battery voltage divider (100K/100K)
 *   BLE    → NimBLE central (connects to LILYGO SIM800L NUS server)
 *
 * Data flow: LoRa E32-433T30S → ESP32 (parse) → BLE NUS → LILYGO SIM800L → MQTT
 *
 * Shared constants (WDT, MEMORY, PUBLISH, LORA) come from utilities.h
 * via sensor_v2.h.  This file only adds RX-specific config.
 */
#ifndef UTILITIES_RX_H_
#define UTILITIES_RX_H_

#include <Arduino.h>
#include <HardwareSerial.h>

/* ===== VERSION (override utilities.h) ===== */
#undef  FIRMWARE_VERSION
#undef  BUILD_DATE
#define FIRMWARE_VERSION    "0.3.0-rx-ble"
#define BUILD_DATE          "09-06-2026"

/* ===== SERIAL BAUDRATES ===== */
#define SERIAL_GSM      9600

/* ===== GSM SIM800L PINOUT ===== */
#define GSM_RX_PIN  D7
#define GSM_TX_PIN  D6

/* ===== BATTERY READ (RX uses A1, TX uses A0) ===== */
const uint8_t  BATT_PIN_RX = A0;
const uint32_t R1_RX       = 100000;
const uint32_t R2_RX       = 100000;

/* ===== GSM CONFIG ===== */
#define GSM_APN             "internet"
static constexpr unsigned long GSM_INIT_TIMEOUT_MS  = 60000;

/* ===== MQTT CONFIG ===== */
#define MQTT_BROKER         "119.59.103.220"
#define MQTT_PORT           1883
#define MQTT_TOPIC          "weather/Pamiang/Station_2"
#define MQTT_PONG_TOPIC     "weather/Pamiang/Pong_2"
#define MQTT_USER           ""
#define MQTT_PASSWORD       ""
#define MQTT_CLIENT_ID      "Pamiang-RX-02"
static constexpr unsigned long MQTT_PUBLISH_TIMEOUT  = 30000;

/* ===== BLE NUS CLIENT CONFIG ===== */
#define BLE_TARGET_DEVICE_NAME  "LILYGO_SIM800L"
#define BLE_NUS_SERVICE_UUID    "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_NUS_RX_CHAR_UUID    "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_NUS_TX_CHAR_UUID    "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_NODE_ID             2

#endif
