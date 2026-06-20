# LoRa Ping-Pong link test

Two **identical** Seeed XIAO ESP32-C3 + E32-433T30S units exchange PING/PONG
packets over LoRa and each hosts a WiFi-AP web portal showing both units' link
status (packet counts, loss, round-trip time). Derived from the
`PAMIANG_High_Power_WIFIAP_30s` LoRa wiring.

## How it works (short)
- Same firmware on both boards; the only difference is the build flag
  `PINGPONG_ID` (1 vs 2).
- The E32 runs in transparent UART mode (9600 8N1) on the same address/channel,
  so both units hear each other.
- Every 2 s a unit sends a **PING** (seq + send-time + its counters). The peer
  replies **PONG**, echoing the send-time so the pinger computes round-trip time.
  A ping with no pong before the next ping counts as **lost**.
- Each packet also carries the sender's counters, so either portal shows **both**
  units' status.

See `PingPong_Explained.pdf` for the full walk-through (packet format, state
machine, RTT/loss math, web portal, wiring).

## Wiring (per unit)
| E32-433T30S | XIAO ESP32-C3 |
|---|---|
| TXD | D5 (RX) |
| RXD | D4 (TX) |
| VCC | 3V3 (use a stout supply; TX bursts pull ~0.6-0.8 A) |
| GND | GND |
| M0, M1 | GND (transparent / normal mode) |

## Build & flash
```
pio run -e unitA -t upload   # board #1  (ID 1)
pio run -e unitB -t upload   # board #2  (ID 2)
```
Prebuilt images are in `bin/` (`PingPong_UnitA_id1...bin`, `PingPong_UnitB_id2...bin`,
plus `PingPong_bins.zip`). Built on Arduino-ESP32 core 2.0.x (`espressif32@6.9.0`).

## Monitor
Each unit raises a WiFi AP `PingPong-Unit1` / `PingPong-Unit2` (password
`12345678`). Connect and open `http://192.168.4.1` to see live status.
