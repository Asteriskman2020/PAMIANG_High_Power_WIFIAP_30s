# RX LoRa Gateway — Design Specification

**Date:** 2026-06-04
**Project:** PAMIANG High-Power LoRa Weather Station
**Board:** XIAO ESP32C3 + E32-433T30S (RX) + SIM800L (GSM)

## Purpose

A LoRa-to-GSM MQTT gateway that receives binary weather data packets from the PAMIANG TX board, stores them locally on LittleFS, and publishes batches to an MQTT broker via GSM.

## Hardware

| Component | Pins (XIAO ESP32C3) |
|-----------|---------------------|
| E32-433T30S (LoRa RX) | D1 (TX), D2 (RX), 9600 baud |
| SIM800L (GSM) | D6 (TX), D7 (RX), 9600 baud |
| Battery ADC | A1, 100k/100k divider |

No RS485 sensors, no BLE, no WiFi AP.

## LoRa RX Protocol

Packets are binary packed structs (little-endian) defined in `LoRaPacket.h`:

- **Magic byte:** `0xA5` identifies valid packets
- **Heartbeat (13 bytes):** type `0x01` — uptime + heap info
- **Data (44 or 61 bytes):** type `0x02` — timestamp + SensorData + optional BLE data

RX reads bytes from `LORA_SERIAL`, searches for magic `0xA5`, reads the type byte to determine packet length, then casts the buffer to the appropriate struct.

## State Machine

```
STATE_INIT → STATE_LORA_LISTEN ←───────────────────┐
                  │                                  │
                  │ packet received                  │
                  ▼                                  │
            STATE_SAVE → STATE_GSM_INIT → STATE_PUBLISH
                          (if not connected)    (if batch ready)
```

The gateway runs continuously (no TPL5110 deep sleep). After each publish cycle, it returns to listening.

### STATE_INIT
- Init Serial debug (115200)
- Init LittleFS
- Init LoRa UART (9600, D1/D2)
- Init GSM UART (9600, D6/D7)
- Read battery voltage
- Init WDT (45s)

### STATE_LORA_LISTEN
- Read available bytes from LoRa serial into ring buffer
- Scan for magic `0xA5`, then read type byte
- For heartbeat: log and discard
- For data packet: read full packet, validate, decode, advance to STATE_SAVE
- Feed WDT continuously
- Timeout: if no packet received within configurable interval, still loop (listen is perpetual)

### STATE_SAVE
- Convert `LoRaDataPacket` to `DataRecord`
- Save to LittleFS CSV (`/DATA_TEMP.csv`) using Memory module
- Advance to STATE_GSM_INIT (if GSM not connected) or STATE_PUBLISH (if connected)

### STATE_GSM_INIT
- Init SIM800L via GsmHandler
- Connect to GPRS network
- Advance to STATE_PUBLISH

### STATE_PUBLISH
- Count stored records in temp CSV
- If below batch threshold: return to STATE_LORA_LISTEN
- Build compact JSON payload (same format as Srisaket)
- Publish to MQTT topic `weather/Pamiang/Station_2`
- Remove published records from temp CSV
- Return to STATE_LORA_LISTEN

## MQTT Configuration

| Parameter | Value |
|-----------|-------|
| Broker | 119.59.103.220 |
| Port | 1883 |
| Topic | weather/Pamiang/Station_2 |
| Pong topic | weather/Pamiang/Station_2/pong |
| Username | kmutt |
| Password | kmutt@kmutt |
| Client ID | PCB_RX_1 |

## MQTT JSON Format

Same compact format as Srisaket TX:

**Heartbeat** (to `weather/Pamiang/Station_2/pong`):
```json
{"n":0,"alive":1,"vt":4200,"heap":45000,"uptime":120,"gsm_rssi":15,"fw":"0.1.0-rx"}
```

**Data batch** (to `weather/Pamiang/Station_2`):
```json
{"n":1,"seq":0,"vt":4200,"srs":15,"d":"040626","r":[
  {"d":"040626","t":"1420","sh":523,"st":275,"se":350,"ph":68,"sn":42,"sp":15,"sk":120,
   "ws":15,"wd":180,"ah":753,"at":312,"co2":450,"pr":1013,"il":25000,"rf":25,"so":350,
   "bt":275,"bh":653,"b117":260,"bd":15,"br":0,"blf":2,"bp":450,"bs":320}
]}
```

BLE fields (`bt`, `bh`, `b117`, `bd`, `br`, `blf`, `bp`, `bs`) only included when `ble_valid == 1`.

## Files

| Action | Path | Description |
|--------|------|-------------|
| New | `src/main_rx.cpp` | RX gateway main program |
| New | `include/utilities_rx.h` | RX-specific pin defs, MQTT config, constants |
| Copy | `include/GsmHandler.h` | From Srisaket_Version (GSM/MQTT handler) |
| Copy | `src/GsmHandler.cpp` | From Srisaket_Version |
| Modify | `platformio.ini` | Add `[env:rx_gateway]` with src_filter |
| Reuse | `include/LoRaPacket.h` | Binary packet structs (unchanged) |
| Reuse | `include/sensor_v2.h` | SensorData, BleSensorData, DataRecord (unchanged) |
| Reuse | `include/Memory.h` | LittleFS storage (unchanged) |
| Reuse | `src/Memory.cpp` | CSV read/write (unchanged) |

## File Exclusions for RX Build

The `[env:rx_gateway]` build target excludes:
- `src/main.cpp` (TX main — conflicts with main_rx)
- `src/LoRaE32Handler.cpp` (TX-only LoRa sender)
- `src/sensor_v2.cpp` (RS485 Modbus — not needed for RX)
- `src/WifiApServer.cpp` (WiFi AP — not needed for RX)

## Dependencies

```ini
lib_deps =
    4-20ma/ModbusMaster@^2.0.1    ; kept for sensor_v2.h compatibility
    h2zero/NimBLE-Arduino@^1.4.2  ; kept for sensor_v2.h compatibility
    vshymanskyy/TinyGSM@^0.11.7   ; GSM modem
    knolleary/PubSubClient@^2.8    ; MQTT client
```

## Error Handling

- **LoRa decode failure:** Log warning, discard partial packet, continue listening
- **GSM init failure:** Skip publish, keep data in temp CSV, retry on next cycle
- **MQTT publish failure:** Keep data in temp CSV, retry on next cycle
- **Battery low (<3200mV):** Log warning but continue (no TPL5110 on RX board)
- **WDT:** 45-second timeout, fed continuously in all states

## Publish Thresholds

| Parameter | Value | Description |
|-----------|-------|-------------|
| Batch size | 1 | Publish as soon as 1 record is available |
| Max records per publish | 6 | Limit per MQTT batch |
| Max MQTT payload | 950 bytes | SIM800L TCP limit |

## Version

`0.1.0-rx` — RX gateway firmware version.
