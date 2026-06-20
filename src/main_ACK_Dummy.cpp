/**
 * @file main_ACK_Dummy.cpp
 * @brief LoRa ACK range-test transmitter with dummy sensor data.
 *
 * This build intentionally does not start BLE, WiFi AP, web server, sensors,
 * LittleFS, or the normal AppManager state machine. It only sends LoRa data
 * packets in a loop so the RX board can ACK them during range testing.
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>

#include "LoRaE32Handler.h"
#include "LoRaPacket.h"
#include "utilities.h"

HardwareSerial LORA_SERIAL(0);

static LoRaE32Handler loraHandler;

static constexpr unsigned long DUMMY_TX_INTERVAL_MS = 15000UL;
static constexpr uint16_t DUMMY_BATTERY_MV = 4200;
static constexpr uint8_t DUMMY_DATE = 12;
static constexpr uint8_t DUMMY_MONTH = 6;
static constexpr uint8_t DUMMY_YEAR = 26;

static uint16_t txSequence = 0;
static unsigned long lastSendMs = 0;

static void disableUnusedRadios() {
    WiFi.persistent(false);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);

    Serial.println(F("[DUMMY] WiFi AP/WebServer disabled"));
    Serial.println(F("[DUMMY] BLE disabled: not initialized in this build"));
}

static void fillDummySensorData(SensorData& sensor, uint16_t seq) {
    memset(&sensor, 0, sizeof(sensor));

    sensor.soil_humi       = 500 + (seq % 50);     // 50.0-54.9 %
    sensor.soil_temp       = 250 + (seq % 20);     // 25.0-26.9 C
    sensor.soil_ec         = 1200 + (seq % 100);
    sensor.soil_ph         = 65;                   // 6.5
    sensor.soil_N          = 40 + (seq % 10);
    sensor.soil_P          = 20 + (seq % 8);
    sensor.soil_K          = 60 + (seq % 12);

    sensor.windSpeed       = 18 + (seq % 10);      // 1.8-2.7 m/s
    sensor.windDir_Deg     = (seq * 15) % 360;
    sensor.air_humidity    = 700 + (seq % 60);     // 70.0-75.9 %
    sensor.air_temperature = 300 + (seq % 30);     // 30.0-32.9 C
    sensor.CO2             = 420 + (seq % 40);
    sensor.pressure        = 1013;                 // 101.3 kPa
    sensor.illuminance     = 25000UL + (uint32_t)(seq % 1000);
    sensor.rainfall        = seq % 20;             // 0.0-1.9 mm
    sensor.solar           = 650 + (seq % 80);
}

static void fillDummyPacket(LoRaDataPacket& pkt, uint16_t seq) {
    memset(&pkt, 0, sizeof(pkt));

    pkt.hdr.magic = LORA_MAGIC;
    pkt.hdr.type  = LORA_PKT_DATA;
    pkt.hdr.seq   = seq;
    pkt.hdr.vt    = DUMMY_BATTERY_MV;

    // LoRaDataPacket carries only hour/minute, so move one minute per dummy packet.
    uint32_t totalMinutes = seq % (24UL * 60UL);
    pkt.date   = DUMMY_DATE;
    pkt.month  = DUMMY_MONTH;
    pkt.year   = DUMMY_YEAR;
    pkt.hour   = totalMinutes / 60;
    pkt.minute = totalMinutes % 60;

    fillDummySensorData(pkt.sensor, seq);

    pkt.ble_valid = 0;
    memset(&pkt.ble, 0, sizeof(pkt.ble));
}

static bool sendDummyPacket() {
    LoRaDataPacket pkt;
    fillDummyPacket(pkt, txSequence);

    Serial.printf("[DUMMY] TX seq=%u size=%u vt=%umV time=%02u:%02u\n",
                  pkt.hdr.seq, (unsigned)sizeof(pkt), pkt.hdr.vt,
                  pkt.hour, pkt.minute);

#ifdef LORA_USE_ACK
    for (uint8_t retry = 0; retry <= LORA_ACK_MAX_RETRIES; retry++) {
        if (retry > 0) {
            Serial.printf("[DUMMY] Retry %u/%u seq=%u\n",
                          retry, LORA_ACK_MAX_RETRIES, pkt.hdr.seq);
        }

        if (!loraHandler.send(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt))) {
            delay(LORA_ACK_TX_SETTLE);
            continue;
        }

        delay(LORA_ACK_TX_SETTLE);

        LoRaAckPacket ack;
        if (loraHandler.waitAck(ack, pkt.hdr.seq, LORA_ACK_TIMEOUT_MS)) {
            Serial.printf("[DUMMY] ACK OK seq=%u\n", pkt.hdr.seq);
            txSequence++;
            return true;
        }
    }

    Serial.printf("[DUMMY] ACK FAIL seq=%u after %u attempt(s)\n",
                  pkt.hdr.seq, LORA_ACK_MAX_RETRIES + 1);
    txSequence++;
    return false;
#else
    bool ok = loraHandler.send(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
    Serial.println(ok ? F("[DUMMY] TX OK") : F("[DUMMY] TX FAIL"));
    txSequence++;
    return ok;
#endif
}

void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    delay(1000);

    Serial.println();
    Serial.println(F("===== LoRa ACK Dummy Range Test ====="));
    Serial.printf("[DUMMY] Interval: %lu ms\n", DUMMY_TX_INTERVAL_MS);

    pinMode(PIN_DONE, OUTPUT);
    digitalWrite(PIN_DONE, LOW);

    disableUnusedRadios();

    LORA_SERIAL.setRxBufferSize(256);
    LORA_SERIAL.begin(SERIAL_LORA, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);

    if (loraHandler.init(LORA_SERIAL)) {
        Serial.println(F("[DUMMY] LoRa ready"));
    } else {
        Serial.println(F("[DUMMY] LoRa init failed, will retry in loop"));
    }

    lastSendMs = 0;
}

void loop() {
    unsigned long now = millis();
    if (lastSendMs == 0 || now - lastSendMs >= DUMMY_TX_INTERVAL_MS) {
        lastSendMs = now;

        if (!loraHandler.isReady()) {
            loraHandler.init(LORA_SERIAL);
        }

        sendDummyPacket();
        Serial.printf("[DUMMY] heap=%u next_seq=%u\n",
                      (unsigned)(ESP.getFreeHeap() & 0xFFFF), txSequence);
    }

    delay(10);
}
