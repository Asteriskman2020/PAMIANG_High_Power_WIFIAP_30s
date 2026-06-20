/**
 * @file    EspNowReceiver.cpp
 * @brief   ESP-NOW receiver — see EspNowReceiver.h.
 */
#include "EspNowReceiver.h"
#include <WiFi.h>
#include <esp_now.h>

/* Latest packet captured by the ISR-context recv callback. Touched from the
 * WiFi task, so flag + payload are volatile and read with interrupts brief. */
static volatile bool   s_fresh   = false;
static volatile uint32_t s_lastMs = 0;
static EspNowPacket    s_pkt     = {};
static bool            s_ready   = false;

// Arduino-ESP32 changed the recv-callback signature in core 3.x.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
static void onRecv(const esp_now_recv_info_t* /*info*/, const uint8_t* data, int len) {
#else
static void onRecv(const uint8_t* /*mac*/, const uint8_t* data, int len) {
#endif
    if (len != (int)sizeof(EspNowPacket)) return;
    const EspNowPacket* p = reinterpret_cast<const EspNowPacket*>(data);
    if (p->magic != ESPNOW_RX_MAGIC || p->version != ESPNOW_RX_VERSION) return;
    memcpy(&s_pkt, p, sizeof(s_pkt));
    s_lastMs = millis();
    s_fresh  = true;
}

bool espnowRxBegin() {
    if (s_ready) return true;

    // ESP-NOW needs the WiFi radio up; the SoftAP is started by WifiApServer
    // before this is called, so the interface is already running.
    if (esp_now_init() != ESP_OK) {
        Serial.println(F("[ESP-NOW] init failed"));
        return false;
    }
    if (esp_now_register_recv_cb(onRecv) != ESP_OK) {
        Serial.println(F("[ESP-NOW] register recv cb failed"));
        esp_now_deinit();
        return false;
    }
    s_ready = true;
    Serial.printf("[ESP-NOW] listening on WiFi ch %d\n", WiFi.channel());
    return true;
}

void espnowRxStop() {
    if (!s_ready) return;
    esp_now_unregister_recv_cb();
    esp_now_deinit();
    s_ready = false;
    s_fresh = false;
    Serial.println(F("[ESP-NOW] stopped"));
}

bool espnowRxReady() { return s_ready; }

uint32_t espnowRxLastMs() { return s_lastMs; }

bool espnowRxPoll(BleSensorData& out) {
    if (!s_fresh) return false;

    // Snapshot the shared packet with the recv callback briefly masked.
    noInterrupts();
    EspNowPacket p = s_pkt;
    s_fresh = false;
    interrupts();

    // Map Sniffer fields → BleSensorData (same units the BLE-NUS path uses;
    // temperature/humidity are already tenths, TMP117 isn't transmitted).
    out.ble_temp   = (int16_t)p.temperature;
    out.ble_humi   = p.humidity;
    out.ble_tmp117 = 0;
    out.ble_rain   = p.rainfall;
    out.ble_leaf   = p.leaf_wetness;
    out.ble_par    = p.par_light;
    out.ble_soil   = p.soil_moisture;

    Serial.printf("[ESP-NOW] rx from \"%s\"  T:%.1f H:%.1f rain:%u leaf:%u par:%u soil:%u\n",
                  p.host, p.temperature / 10.0f, p.humidity / 10.0f,
                  p.rainfall, p.leaf_wetness, p.par_light, p.soil_moisture);
    return true;
}
