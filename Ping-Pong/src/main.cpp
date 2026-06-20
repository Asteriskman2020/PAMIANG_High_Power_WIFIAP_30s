/**
 * @file  main.cpp
 * @brief LoRa Ping-Pong link test for two XIAO ESP32-C3 + E32-433T30S units.
 *
 * Both units run this one firmware; the build flag PINGPONG_ID (1 or 2) is the
 * only difference. They share one LoRa channel (E32 transparent UART mode), so
 * each hears the other.
 *
 *   - Every PING_INTERVAL_MS a unit sends a PING (seq + its send-time + its
 *     running counters) addressed to the peer.
 *   - On receiving a PING addressed to it, a unit immediately replies PONG,
 *     echoing the original send-time so the pinger can compute round-trip time.
 *   - On receiving the matching PONG, the pinger records a successful round
 *     trip and the RTT. A ping with no PONG before the next ping is counted lost.
 *
 * Each unit also runs a WiFi SoftAP + web portal (http://192.168.4.1) that shows
 * BOTH units' status — local counters plus the peer's counters, which ride along
 * inside every packet.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// ── Identity ────────────────────────────────────────────────
#ifndef PINGPONG_ID
#define PINGPONG_ID 1
#endif
static const uint8_t MY_ID   = PINGPONG_ID;
static const uint8_t PEER_ID = (PINGPONG_ID == 1) ? 2 : 1;

// ── LoRa E32 wiring (matches the PAMIANG board: D5=RX, D4=TX) ─
static const int  LORA_RX_PIN = D5;   // E32 TXD -> C3 RX
static const int  LORA_TX_PIN = D4;   // E32 RXD -> C3 TX
static const long LORA_BAUD   = 9600;
#define LORA Serial1

// ── Timing ──────────────────────────────────────────────────
static const uint32_t PING_INTERVAL_MS = 2000;   // how often we ping
static const uint32_t LINK_TIMEOUT_MS  = 6000;    // peer "down" if silent this long

// ── Wire packet (raw struct over transparent LoRa; both ends C3 LE) ──
static const uint8_t PP_MAGIC   = 0x50;   // 'P'
static const uint8_t PP_PING    = 1;
static const uint8_t PP_PONG    = 2;

typedef struct __attribute__((packed)) {
  uint8_t  magic;     // PP_MAGIC
  uint8_t  type;      // PP_PING | PP_PONG
  uint8_t  src;       // sender unit id
  uint8_t  dst;       // target unit id
  uint16_t seq;       // sequence number
  uint32_t t_ms;      // pinger's send millis (echoed in pong for RTT)
  uint32_t src_sent;  // sender's total pings sent      (for peer monitoring)
  uint32_t src_rt;    // sender's total round trips      (for peer monitoring)
  uint8_t  crc;       // XOR of all preceding bytes
} PingPongPacket;

static const size_t PP_SIZE = sizeof(PingPongPacket);

// ── Stats ───────────────────────────────────────────────────
struct Stats {
  uint32_t pingsSent   = 0;   // pings we transmitted
  uint32_t roundTrips  = 0;   // pongs received back for our pings
  uint32_t lost        = 0;   // pings with no pong before the next ping
  uint32_t pingsRecv   = 0;   // pings from peer we answered
  uint32_t lastRtt     = 0;
  uint32_t minRtt      = 0xFFFFFFFF;
  uint32_t maxRtt      = 0;
  uint64_t sumRtt      = 0;
  uint32_t peerSent    = 0;   // peer's reported counters (from packets)
  uint32_t peerRt      = 0;
  uint32_t peerLastMs  = 0;   // millis() of last packet from peer
} st;

// in-flight ping awaiting a pong
static bool     awaiting   = false;
static uint16_t awaitSeq   = 0;
static uint16_t seqCounter = 0;

WebServer web(80);

// ── Helpers ─────────────────────────────────────────────────
static uint8_t crc8(const uint8_t* b, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) c ^= b[i];
  return c;
}

static void sendPacket(uint8_t type, uint16_t seq, uint32_t t_ms) {
  PingPongPacket p;
  p.magic    = PP_MAGIC;
  p.type     = type;
  p.src      = MY_ID;
  p.dst      = PEER_ID;
  p.seq      = seq;
  p.t_ms     = t_ms;
  p.src_sent = st.pingsSent;
  p.src_rt   = st.roundTrips;
  p.crc      = crc8((uint8_t*)&p, PP_SIZE - 1);
  LORA.write((uint8_t*)&p, PP_SIZE);
  LORA.flush();
}

static void handlePacket(const PingPongPacket& p) {
  if (p.src != PEER_ID) return;          // ignore anything not from our peer
  st.peerLastMs = millis();
  st.peerSent   = p.src_sent;
  st.peerRt     = p.src_rt;

  if (p.type == PP_PING && p.dst == MY_ID) {
    // Answer immediately, echoing the pinger's send-time for its RTT calc.
    st.pingsRecv++;
    sendPacket(PP_PONG, p.seq, p.t_ms);
    Serial.printf("[RX] PING seq=%u from U%u -> replied PONG\n", p.seq, p.src);
  } else if (p.type == PP_PONG && p.dst == MY_ID) {
    if (awaiting && p.seq == awaitSeq) {
      uint32_t rtt = millis() - p.t_ms;   // p.t_ms is the millis WE stamped
      st.lastRtt = rtt;
      st.sumRtt += rtt;
      if (rtt < st.minRtt) st.minRtt = rtt;
      if (rtt > st.maxRtt) st.maxRtt = rtt;
      st.roundTrips++;
      awaiting = false;
      Serial.printf("[RX] PONG seq=%u  RTT=%u ms\n", p.seq, rtt);
    }
  }
}

// Frame the byte stream: scan for magic, accumulate PP_SIZE bytes, check crc.
static void pollLoRa() {
  static uint8_t  buf[64];
  static size_t   idx = 0;
  while (LORA.available()) {
    uint8_t b = (uint8_t)LORA.read();
    if (idx == 0) {
      if (b != PP_MAGIC) continue;       // resync: wait for a magic byte
      buf[idx++] = b;
    } else {
      buf[idx++] = b;
      if (idx == PP_SIZE) {
        idx = 0;
        PingPongPacket p;
        memcpy(&p, buf, PP_SIZE);
        if (p.crc == crc8(buf, PP_SIZE - 1)) handlePacket(p);
        else Serial.println("[RX] CRC fail (dropped)");
      }
    }
  }
}

// ── Web portal ──────────────────────────────────────────────
static bool linkUp() { return st.peerLastMs && (millis() - st.peerLastMs) < LINK_TIMEOUT_MS; }
static uint32_t avgRtt() { return st.roundTrips ? (uint32_t)(st.sumRtt / st.roundTrips) : 0; }
static float successPct() {
  uint32_t attempts = st.roundTrips + st.lost;
  return attempts ? (100.0f * st.roundTrips / attempts) : 0.0f;
}

static void handleApi() {
  char buf[640];
  snprintf(buf, sizeof(buf),
    "{\"id\":%u,\"peer\":%u,\"link_up\":%s,"
    "\"pings_sent\":%lu,\"round_trips\":%lu,\"lost\":%lu,\"pings_recv\":%lu,"
    "\"success_pct\":%.1f,\"last_rtt\":%lu,\"min_rtt\":%lu,\"max_rtt\":%lu,\"avg_rtt\":%lu,"
    "\"peer_sent\":%lu,\"peer_rt\":%lu,\"peer_last_s\":%lu,\"uptime_s\":%lu}",
    MY_ID, PEER_ID, linkUp() ? "true" : "false",
    (unsigned long)st.pingsSent, (unsigned long)st.roundTrips, (unsigned long)st.lost,
    (unsigned long)st.pingsRecv, successPct(), (unsigned long)st.lastRtt,
    (unsigned long)(st.minRtt == 0xFFFFFFFF ? 0 : st.minRtt), (unsigned long)st.maxRtt,
    (unsigned long)avgRtt(), (unsigned long)st.peerSent, (unsigned long)st.peerRt,
    (unsigned long)(st.peerLastMs ? (millis() - st.peerLastMs) / 1000 : 0),
    (unsigned long)(millis() / 1000));
  web.send(200, "application/json", buf);
}

static void handleRoot() {
  // Self-contained page; polls /api every second.
  static const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html><head>
<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>
<title>LoRa Ping-Pong</title><style>
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
<h1>LoRa Ping-Pong &mdash; Unit <span id='id'>?</span></h1>
<div class='sub'>E32-433T30S link test &nbsp;|&nbsp; <span id='link' class='link down'>...</span>
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
g('id',d.id);g('pid',d.peer);g('up',d.uptime_s);
const L=document.getElementById('link');
L.textContent=d.link_up?'LINK UP':'LINK DOWN';L.className='link '+(d.link_up?'up':'down');
g('ps',d.pings_sent);g('rt',d.round_trips);g('lo',d.lost);g('pr',d.pings_recv);
g('sp',d.success_pct.toFixed(1)+'%');
g('rtt',d.last_rtt+' / '+d.min_rtt+' / '+d.max_rtt+' / '+d.avg_rtt+' ms');
g('psnt',d.peer_sent);g('prt',d.peer_rt);
g('pls',d.link_up?d.peer_last_s+'s ago':'silent');
g('pst',d.link_up?'online':'offline');
}catch(e){}}
function g(i,v){const e=document.getElementById(i);if(e)e.textContent=v;}
setInterval(tick,1000);tick();
</script></body></html>)HTML";
  web.send_P(200, "text/html", PAGE);
}

// ── Setup / loop ────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n[PingPong] Unit %u (peer %u) starting\n", MY_ID, PEER_ID);

  LORA.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);

  char ssid[24];
  snprintf(ssid, sizeof(ssid), "PingPong-Unit%u", MY_ID);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, "12345678");
  Serial.printf("[WiFi] AP \"%s\"  http://%s\n", ssid, WiFi.softAPIP().toString().c_str());

  web.on("/", handleRoot);
  web.on("/api", handleApi);
  web.begin();
}

void loop() {
  web.handleClient();
  pollLoRa();

  static uint32_t lastPing = 0;
  if (millis() - lastPing >= PING_INTERVAL_MS) {
    lastPing = millis();
    if (awaiting) { st.lost++;                  // previous ping never got a pong
      Serial.printf("[TX] seq=%u lost (no pong)\n", awaitSeq); }
    seqCounter++;
    awaitSeq = seqCounter;
    awaiting = true;
    st.pingsSent++;
    sendPacket(PP_PING, awaitSeq, millis());
    Serial.printf("[TX] PING seq=%u\n", awaitSeq);
  }
}
