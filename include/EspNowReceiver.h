/**
 * @file    EspNowReceiver.h
 * @brief   ESP-NOW receiver for Sniffer-Portal weather readings.
 *
 * Counterpart to the Sniffer-WatchDog `espnow_sender` module (FW >= 1.5.1).
 * That station broadcasts each reading as a packed EspNowPacket; this module
 * captures those frames and exposes the latest one as a BleSensorData, so it
 * drops straight into the existing BLE-NUS ingestion path in AppManager.
 *
 * CHANNEL NOTE: ESP-NOW only delivers frames received on the *current* WiFi
 * channel. This board listens on its SoftAP channel (WIFI_AP_CHANNEL, = 1).
 * The Sniffer transmits on its STA/router channel, so for reception to work
 * the Sniffer's router must be on channel 1 (or both on the same AP).
 */
#pragma once

#include <Arduino.h>
#include "sensor_v2.h"

/* ── Wire format — must match the Sniffer's espnow_sender.h verbatim ── */
static constexpr uint8_t ESPNOW_RX_MAGIC   = 0xA7;
static constexpr uint8_t ESPNOW_RX_VERSION = 1;

typedef struct __attribute__((packed)) {
    uint8_t  magic;          // == ESPNOW_RX_MAGIC
    uint8_t  version;        // == ESPNOW_RX_VERSION
    char     host[16];       // sender hostname (null-terminated, truncated)
    uint16_t year, month, date, hour, minute;
    uint16_t rainfall;
    uint16_t leaf_wetness;
    uint16_t par_light;
    uint16_t soil_moisture;
    uint16_t temperature;    // tenths of °C
    uint16_t humidity;       // tenths of %
} EspNowPacket;

/** @brief Init ESP-NOW and register the receive callback. Call after the
 *  WiFi SoftAP is up. Idempotent — safe to call repeatedly. */
bool espnowRxBegin();

/** @brief Tear down ESP-NOW. Call before stopping the SoftAP. */
void espnowRxStop();

/** @brief True if ESP-NOW is initialised and listening. */
bool espnowRxReady();

/** @brief Pop the most recent unread reading.
 *  @param out      filled with the mapped sensor values on success
 *  @return true if a fresh packet arrived since the last call, else false. */
bool espnowRxPoll(BleSensorData& out);

/** @brief millis() timestamp of the most recent valid packet (0 if none). */
uint32_t espnowRxLastMs();
