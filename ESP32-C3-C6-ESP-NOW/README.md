# ESP-NOW Ping-Pong (ESP32-C3 SuperMini ↔ ESP32-C6 SuperMini)

The LoRa Ping-Pong test re-worked to run over **ESP-NOW** between two different
SuperMini boards. Each unit pings the other, measures round-trip time (RTT) and
packet loss, and hosts a WiFi-AP web portal showing **both** units' status.

## How it works
- **Same firmware**, two PlatformIO envs — `c3_supermini` (ID 1) and
  `c6_supermini` (ID 2). The build flags `PINGPONG_ID` and `UNIT_LABEL` are the
  only differences.
- Transport is **ESP-NOW broadcast** (no pairing). Every 2 s a unit broadcasts a
  PING (seq + send-time + its counters); the peer replies PONG echoing the
  send-time, so the pinger computes RTT. A ping with no PONG before the next one
  is counted lost.
- Whole packets (19 bytes, magic + XOR-CRC validated) arrive via the ESP-NOW
  receive callback into a small ring buffer and are processed in `loop()`.
- Each packet carries the sender's counters, so either portal shows both units.

## ⚠️ Channel — important
ESP-NOW only delivers on the **current WiFi channel**. Both units fix their
SoftAP (and therefore the radio) to `ESPNOW_CHANNEL` (default **1**) so they hear
each other. Both must use the same channel.

## Boards / core
- ESP32-C3 SuperMini → board `esp32-c3-devkitm-1`
- ESP32-C6 SuperMini → board `esp32-c6-devkitm-1`
- Built with `platform = espressif32` (**Arduino-ESP32 core 3.x** — required by
  the C6; the C3 builds on it too). Do **not** pin core 2.x here.

## Build & flash
```
pio run -e c3_supermini -t upload    # ESP32-C3 SuperMini  (Unit 1)
pio run -e c6_supermini -t upload    # ESP32-C6 SuperMini  (Unit 2)
```
Prebuilt images are in `bin/` (`..._C3-SuperMini_id1.bin`,
`..._C6-SuperMini_id2.bin`, plus `ESPNOW_PingPong_bins.zip`).

## Monitor
Each unit raises a WiFi AP `ESPNOW-PingPong-C3` / `ESPNOW-PingPong-C6`
(password `12345678`). Connect and open `http://192.168.4.1` for live status
(sent / round-trips / lost / success% / RTT last·min·max·avg + peer + LINK badge).
