/**
 * @file  main.cpp
 * @brief ESP-NOW Ping-Pong link test between an ESP32-C3 SuperMini (Unit 1)
 *        and an ESP32-C6 SuperMini (Unit 2).
 *
 * Same firmware on both boards; the build flag PINGPONG_ID (1/2) and the board
 * are the only differences. Instead of LoRa, packets travel over ESP-NOW:
 *
 *   - Every PING_INTERVAL_MS a unit broadcasts a PING (seq + send-time + its
 *     counters) addressed to the peer.
 *   - On receiving a PING addressed to it, a unit immediately replies PONG,
 *     echoing the send-time so the pinger computes round-trip time.
 *   - A ping with no matching PONG before the next ping counts as lost.
 *
 * ESP-NOW only delivers on the current WiFi channel, so both units fix their
 * SoftAP (and therefore the radio) to ESPNOW_CHANNEL. Each unit also serves a
 * web portal (http://192.168.4.1) showing BOTH units' status — the peer's
 * counters ride inside every packet.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ── Identity ────────────────────────────────────────────────
#ifndef PINGPONG_ID
#define PINGPONG_ID 1
#endif
#ifndef UNIT_LABEL
#define UNIT_LABEL "?"
#endif
static const uint8_t MY_ID   = PINGPONG_ID;
static const uint8_t PEER_ID = (PINGPONG_ID == 1) ? 2 : 1;

// ── ESP-NOW / radio ─────────────────────────────────────────
static const uint8_t  ESPNOW_CHANNEL = 1;          // both units MUST match
static uint8_t        BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ── Timing ──────────────────────────────────────────────────
static const uint32_t PING_INTERVAL_MS = 2000;
static const uint32_t LINK_TIMEOUT_MS  = 6000;

// ── Wire packet (same layout as the LoRa version) ───────────
static const uint8_t PP_MAGIC = 0x50;
static const uint8_t PP_PING   = 1;
static const uint8_t PP_PONG   = 2;

typedef struct __attribute__((packed)) {
  uint8_t  magic;     // PP_MAGIC
  uint8_t  type;      // PP_PING | PP_PONG
  uint8_t  src;       // sender unit id
  uint8_t  dst;       // target unit id
  uint16_t seq;       // sequence number
  uint32_t t_ms;      // pinger's send millis (echoed in pong for RTT)
  uint32_t src_sent;  // sender's total pings sent (peer monitoring)
  uint32_t src_rt;    // sender's total round trips (peer monitoring)
  uint8_t  crc;       // XOR of preceding bytes
} PingPongPacket;
static const size_t PP_SIZE = sizeof(PingPongPacket);

// ── Stats ───────────────────────────────────────────────────
struct Stats {
  uint32_t pingsSent = 0, roundTrips = 0, lost = 0, pingsRecv = 0;
  uint32_t lastRtt = 0, minRtt = 0xFFFFFFFF, maxRtt = 0;
  uint64_t sumRtt = 0;
  uint32_t peerSent = 0, peerRt = 0, peerLastMs = 0;
} st;

static bool     awaiting = false;
static uint16_t awaitSeq = 0, seqCounter = 0;

// ── Single-producer ring buffer: recv callback -> loop ──────
static volatile PingPongPacket rxRing[8];
static volatile uint8_t        rxHead = 0, rxTail = 0;

WebServer web(80);

static uint8_t crc8(const uint8_t* b, size_t n) {
  uint8_t c = 0; for (size_t i = 0; i < n; i++) c ^= b[i]; return c;
}

static void sendPacket(uint8_t type, uint16_t seq, uint32_t t_ms) {
  PingPongPacket p;
  p.magic = PP_MAGIC; p.type = type; p.src = MY_ID; p.dst = PEER_ID;
  p.seq = seq; p.t_ms = t_ms; p.src_sent = st.pingsSent; p.src_rt = st.roundTrips;
  p.crc = crc8((uint8_t*)&p, PP_SIZE - 1);
  esp_now_send(BCAST, (uint8_t*)&p, PP_SIZE);
}

// ESP-NOW receive callback (Arduino-ESP32 core 3.x signature). Runs in the
// WiFi task — just copy the frame into the ring; process it in loop().
static void onRecv(const esp_now_recv_info_t* /*info*/, const uint8_t* data, int len) {
  if (len != (int)PP_SIZE) return;
  uint8_t next = (rxHead + 1) % 8;
  if (next == rxTail) return;                 // ring full, drop
  memcpy((void*)&rxRing[rxHead], data, PP_SIZE);
  rxHead = next;
}

static void handlePacket(const PingPongPacket& p) {
  if (p.magic != PP_MAGIC) return;
  if (p.crc != crc8((const uint8_t*)&p, PP_SIZE - 1)) return;
  if (p.src != PEER_ID) return;
  st.peerLastMs = millis();
  st.peerSent = p.src_sent;
  st.peerRt   = p.src_rt;

  if (p.type == PP_PING && p.dst == MY_ID) {
    st.pingsRecv++;
    sendPacket(PP_PONG, p.seq, p.t_ms);       // echo t_ms for the pinger's RTT
    Serial.printf("[RX] PING seq=%u -> PONG\n", p.seq);
  } else if (p.type == PP_PONG && p.dst == MY_ID) {
    if (awaiting && p.seq == awaitSeq) {
      uint32_t rtt = millis() - p.t_ms;
      st.lastRtt = rtt; st.sumRtt += rtt;
      if (rtt < st.minRtt) st.minRtt = rtt;
      if (rtt > st.maxRtt) st.maxRtt = rtt;
      st.roundTrips++; awaiting = false;
      Serial.printf("[RX] PONG seq=%u RTT=%u ms\n", p.seq, rtt);
    }
  }
}

static void drainRx() {
  while (rxTail != rxHead) {
    PingPongPacket p;
    memcpy(&p, (const void*)&rxRing[rxTail], PP_SIZE);
    rxTail = (rxTail + 1) % 8;
    handlePacket(p);
  }
}

// ── Web portal ──────────────────────────────────────────────
static bool linkUp() { return st.peerLastMs && (millis() - st.peerLastMs) < LINK_TIMEOUT_MS; }
static uint32_t avgRtt() { return st.roundTrips ? (uint32_t)(st.sumRtt / st.roundTrips) : 0; }
static float successPct() {
  uint32_t a = st.roundTrips + st.lost; return a ? (100.0f * st.roundTrips / a) : 0.0f;
}

static void handleApi() {
  char buf[680];
  snprintf(buf, sizeof(buf),
    "{\"id\":%u,\"label\":\"%s\",\"peer\":%u,\"link_up\":%s,"
    "\"pings_sent\":%lu,\"round_trips\":%lu,\"lost\":%lu,\"pings_recv\":%lu,"
    "\"success_pct\":%.1f,\"last_rtt\":%lu,\"min_rtt\":%lu,\"max_rtt\":%lu,\"avg_rtt\":%lu,"
    "\"peer_sent\":%lu,\"peer_rt\":%lu,\"peer_last_s\":%lu,\"channel\":%u,\"uptime_s\":%lu}",
    MY_ID, UNIT_LABEL, PEER_ID, linkUp() ? "true" : "false",
    (unsigned long)st.pingsSent, (unsigned long)st.roundTrips, (unsigned long)st.lost,
    (unsigned long)st.pingsRecv, successPct(), (unsigned long)st.lastRtt,
    (unsigned long)(st.minRtt == 0xFFFFFFFF ? 0 : st.minRtt), (unsigned long)st.maxRtt,
    (unsigned long)avgRtt(), (unsigned long)st.peerSent, (unsigned long)st.peerRt,
    (unsigned long)(st.peerLastMs ? (millis() - st.peerLastMs) / 1000 : 0),
    ESPNOW_CHANNEL, (unsigned long)(millis() / 1000));
  web.send(200, "application/json", buf);
}

static void handleRoot() {
  static const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html><head>
<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>
<title>ESP-NOW Ping-Pong</title><style>
body{font-family:system-ui,Arial;background:#0f172a;color:#e2e8f0;margin:0;padding:18px}
h1{font-size:20px;margin:0 0 4px}.sub{color:#94a3b8;font-size:13px;margin-bottom:16px}
.link{display:inline-block;padding:4px 12px;border-radius:20px;font-size:13px;font-weight:600}
.up{background:#14532d;color:#86efac}.down{background:#7f1d1d;color:#fca5a5}
.grid{display:flex;gap:14px;flex-wrap:wrap}
.card{background:#1e293b;border-radius:12px;padding:16px;flex:1;min-width:260px}
.card h2{font-size:15px;margin:0 0 10px;color:#60a5fa}
.row{display:flex;justify-content:space-between;padding:5px 0;border-bottom:1px solid #334155;font-size:14px}
.row span:last-child{font-weight:600}.big{font-size:26px;color:#22c55e;font-weight:700}
.muted{color:#94a3b8}</style></head><body>
<h1>ESP-NOW Ping-Pong &mdash; Unit <span id='id'>?</span> (<span id='lab'>?</span>)</h1>
<div class='sub'>ESP-NOW ch <span id='ch'>?</span> &nbsp;|&nbsp; <span id='link' class='link down'>...</span>
&nbsp; uptime <span id='up'>0</span>s</div>
<div class='grid'>
<div class='card'><h2>This unit (local)</h2>
<div class='row'><span>Pings sent</span><span id='ps'>0</span></div>
<div class='row'><span>Round trips (pong back)</span><span id='rt'>0</span></div>
<div class='row'><span>Lost</span><span id='lo'>0</span></div>
<div class='row'><span>Pings answered (from peer)</span><span id='pr'>0</span></div>
<div class='row'><span>Success rate</span><span class='big' id='sp'>0%</span></div>
<div class='row'><span>RTT last / min / max / avg</span><span id='rtt'>0 ms</span></div>
</div>
<div class='card'><h2>Peer (Unit <span id='pid'>?</span>)</h2>
<div class='row'><span>Peer pings sent</span><span id='psnt'>0</span></div>
<div class='row'><span>Peer round trips</span><span id='prt'>0</span></div>
<div class='row'><span>Last heard</span><span id='pls'>never</span></div>
<div class='row muted'><span>Peer status</span><span id='pst'>unknown</span></div>
</div></div>
<script>
async function tick(){try{const d=await(await fetch('/api')).json();
g('id',d.id);g('lab',d.label);g('pid',d.peer);g('ch',d.channel);g('up',d.uptime_s);
const L=document.getElementById('link');
L.textContent=d.link_up?'LINK UP':'LINK DOWN';L.className='link '+(d.link_up?'up':'down');
g('ps',d.pings_sent);g('rt',d.round_trips);g('lo',d.lost);g('pr',d.pings_recv);
g('sp',d.success_pct.toFixed(1)+'%');
g('rtt',d.last_rtt+' / '+d.min_rtt+' / '+d.max_rtt+' / '+d.avg_rtt+' ms');
g('psnt',d.peer_sent);g('prt',d.peer_rt);
g('pls',d.link_up?d.peer_last_s+'s ago':'silent');g('pst',d.link_up?'online':'offline');
}catch(e){}}
function g(i,v){const e=document.getElementById(i);if(e)e.textContent=v;}
setInterval(tick,1000);tick();
</script></body></html>)HTML";
  web.send_P(200, "text/html", PAGE);
}

// ── Setup / loop ────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("\n[ESP-NOW PingPong] Unit %u (%s), peer %u\n", MY_ID, UNIT_LABEL, PEER_ID);

  // SoftAP fixes the radio to ESPNOW_CHANNEL and hosts the web portal.
  char ssid[28];
  snprintf(ssid, sizeof(ssid), "ESPNOW-PingPong-%s", UNIT_LABEL);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, "12345678", ESPNOW_CHANNEL);
  Serial.printf("[WiFi] AP \"%s\" ch%u  http://%s\n",
                ssid, ESPNOW_CHANNEL, WiFi.softAPIP().toString().c_str());

  if (esp_now_init() != ESP_OK) { Serial.println("[ESP-NOW] init failed"); return; }
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx   = WIFI_IF_AP;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  Serial.println("[ESP-NOW] ready (broadcast peer)");

  web.on("/", handleRoot);
  web.on("/api", handleApi);
  web.begin();
}

void loop() {
  web.handleClient();
  drainRx();

  static uint32_t lastPing = 0;
  if (millis() - lastPing >= PING_INTERVAL_MS) {
    lastPing = millis();
    if (awaiting) { st.lost++; Serial.printf("[TX] seq=%u lost\n", awaitSeq); }
    seqCounter++;
    awaitSeq = seqCounter;
    awaiting = true;
    st.pingsSent++;
    sendPacket(PP_PING, awaitSeq, millis());
    Serial.printf("[TX] PING seq=%u\n", awaitSeq);
  }
}
