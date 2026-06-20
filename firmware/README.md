# Prebuilt Firmware — ESP-NOW weather link

Two boards, both Seeed XIAO ESP32-C3, built with PlatformIO on Arduino-ESP32
core **2.0.x** (`platform = espressif32@6.9.0`).

| File | Role | Source project | FW |
|------|------|----------------|----|
| `Sniffer_Portal_v1.5.1_sender_XIAO-ESP32-C3.bin` | **ESP-NOW sender** — Weather Station Extractor; broadcasts each reading over ESP-NOW (alongside MQTT / InfluxDB / BLE) | `Version1.5.1/Sniffer_Portal` (this repo) | 1.5.1 |
| `PAMIANG_WIFIAP_v3.0.0_receiver_XIAO-ESP32-C3.bin` | **ESP-NOW receiver** — PAMIANG High-Power LoRa station; ingests the sender's readings and folds them into its CSV + LoRa publish | `PAMIANG_High_Power_WIFIAP_30s` (`seeed_xiao_esp32c3_ack` env) | 3.0.0-lora |

These are **application images** (suitable for OTA upload, or `pio run -t upload`).
A fresh esptool flash also needs that project's `bootloader.bin` + `partitions.bin`;
the easiest path is `pio run -e <env> -t upload`.

## ESP-NOW link requirement
ESP-NOW only delivers frames on the **current WiFi channel**. The sender transmits
on its router/STA channel; the receiver listens on its SoftAP channel
(`WIFI_AP_CHANNEL`, default **1**). For reception to work, lock the sender's WiFi
router to **channel 1** (or set the receiver's `WIFI_AP_CHANNEL` to the router's
channel). The sender defaults to broadcast (`FF:FF:FF:FF:FF:FF`) — no pairing needed.
