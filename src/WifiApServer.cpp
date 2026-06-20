#include "WifiApServer.h"
#include "utilities.h"
#include <LittleFS.h>
#include <esp_task_wdt.h>
#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <Update.h>


/* ── shared CSS injected once per page ───────────────────────── */
/* ── Dark Colorful theme ── */
static const char CSS[] PROGMEM = R"(
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:Arial,sans-serif;background:#0d1117;min-height:100vh;color:#e6edf3;font-size:14px}
.topbar{background:linear-gradient(90deg,#7c3aed,#2563eb,#0891b2);color:#fff;padding:6px 12px;display:flex;justify-content:space-between;align-items:center;font-size:13px}
.topbar .ver{color:#bfdbfe}
nav{background:#010409;display:flex;flex-wrap:wrap;gap:2px;padding:4px 6px;align-items:center}
nav a{padding:7px 14px;border-radius:4px;color:#fff;text-decoration:none;font-size:13px;font-weight:600}
nav a.live{background:#059669}nav a.live.on{background:#047857}
nav a.files{background:#0891b2}nav a.files.on{background:#0e7490}
nav a.log{background:#d97706}nav a.log.on{background:#b45309}
nav a.settings{background:#7c3aed}nav a.settings.on{background:#6d28d9}
nav a.about{background:#d97706}nav a.about.on{background:#b45309}
nav a.reboot{background:#ea580c;margin-left:auto}nav a.reboot:hover{background:#c2410c}
nav a.logout{background:#dc2626}nav a.logout:hover{background:#b91c1c}
.wrap{max-width:860px;margin:16px auto;padding:0 10px}
.card{background:#161b22;border-radius:10px;box-shadow:0 2px 14px rgba(0,0,0,.5);padding:16px;margin-bottom:14px;border:1px solid #30363d;border-top:3px solid #7c3aed}
.card h3{margin-bottom:10px;color:#c084fc;border-bottom:2px solid #3d2a6e;padding-bottom:6px}
.row{display:flex;flex-wrap:wrap;gap:8px;margin-bottom:6px}
.tag{background:#1f2937;color:#a78bfa;padding:3px 8px;border-radius:4px;font-size:12px;font-weight:600;border:1px solid #374151}
.tag.g{background:#052e16;color:#4ade80}
.tag.r{background:#450a0a;color:#f87171}
.tag.y{background:#451a03;color:#fbbf24}
label{display:block;margin-bottom:3px;font-weight:600;color:#a78bfa;font-size:13px}
input[type=text],input[type=password],input[type=number],input[type=url],input[type=email]{width:100%;padding:8px;border:1px solid #30363d;border-radius:5px;font-size:13px;margin-bottom:10px;background:#0d1117;color:#e6edf3}
input[type=range]{width:100%;margin-bottom:10px;accent-color:#7c3aed}
select{width:100%;padding:8px;border:1px solid #30363d;border-radius:5px;font-size:13px;margin-bottom:10px;background:#0d1117;color:#e6edf3}
.btn{display:inline-block;padding:8px 18px;border:none;border-radius:5px;cursor:pointer;font-size:13px;font-weight:600;text-decoration:none;color:#fff}
.btn-b{background:#2563eb}.btn-g{background:#059669}.btn-r{background:#dc2626}.btn-y{background:#d97706}.btn-p{background:#7c3aed}
.btn:hover{opacity:.85}
table{width:100%;border-collapse:collapse;font-size:13px}
th{background:linear-gradient(90deg,#7c3aed,#2563eb);color:#fff;padding:7px 8px;text-align:left}
td{padding:6px 8px;border-bottom:1px solid #21262d;color:#e6edf3}
tr:nth-child(even)td{background:#0d1117}
.login-wrap{min-height:100vh;display:flex;align-items:center;justify-content:center;background:#0d1117}
.login-box{background:#161b22;border-radius:14px;padding:36px;width:100%;max-width:360px;box-shadow:0 8px 28px rgba(124,58,237,.35);border:1px solid #30363d;border-top:4px solid #7c3aed}
.login-box h2{text-align:center;color:#c084fc;margin-bottom:4px}
.login-box .sub{text-align:center;color:#a78bfa;font-size:12px;margin-bottom:20px}
.alert{padding:8px 12px;border-radius:5px;margin-bottom:10px;font-size:13px}
.alert-r{background:#450a0a;color:#f87171}.alert-g{background:#052e16;color:#4ade80}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:10px}
@media(max-width:540px){.grid2{grid-template-columns:1fr}nav a{padding:5px 9px;font-size:12px}}
)";

/* ── helpers ─────────────────────────────────────────────────── */
WifiApServer::WifiApServer()
    : _server(nullptr), _st(nullptr), _startTime(0),
      _apTimeoutMs(300000UL), _noClientSince(0),
      _lastWebAccessMs(0), _webAccessSeen(false),
      _active(false), _tokenTime(0), _loggedIn(false) {
    _token[0] = '\0';
}

WifiApServer::~WifiApServer() { stop(); }

static uint32_t simpleRand(uint32_t seed) {
    seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
    return seed;
}

void WifiApServer::startSession() {
    uint32_t r = simpleRand(millis() ^ (uint32_t)esp_random());
    snprintf(_token, sizeof(_token), "%08lX%08lX",
             (unsigned long)r, (unsigned long)simpleRand(r));
    _tokenTime = millis();
    _loggedIn  = true;
}

void WifiApServer::clearSession() { _loggedIn = false; _token[0] = '\0'; }

void WifiApServer::markWebAccess() {
    _webAccessSeen = true;
    _lastWebAccessMs = millis();
}

bool WifiApServer::checkAuth() {
    if (!_loggedIn) { redirectTo("/login"); return false; }
    if (millis() - _tokenTime > WEB_SESSION_TIMEOUT_MS) {
        clearSession(); redirectTo("/login"); return false;
    }
    String cookie = _server->header("Cookie");
    if (cookie.indexOf(_token) < 0) { redirectTo("/login"); return false; }
    _tokenTime = millis(); /* refresh */
    return true;
}

void WifiApServer::redirectTo(const char* url) {
    _server->sendHeader("Location", url);
    _server->send(302, "text/plain", "");
}

String WifiApServer::fmtBytes(uint32_t b) {
    if (b < 1024) return String(b) + " B";
    if (b < 1024 * 1024) return String(b / 1024) + " KB";
    return String(b / 1048576) + " MB";
}

String WifiApServer::topBar() {
    String s = F("<div class='topbar'><span>All-in-One Weather Station</span>"
                 "<span class='ver'>v");
    s += FIRMWARE_VERSION;
    s += F("</span></div>");
    return s;
}

String WifiApServer::navBar(const char* active) {
    String s = F("<nav>");
    auto tab = [&](const char* cls, const char* href, const char* label) {
        s += "<a class='"; s += cls;
        if (strcmp(cls, active) == 0) s += " on";
        s += "' href='"; s += href; s += "'>"; s += label; s += "</a>";
    };
    tab("live",     "/live",     "Live");
    tab("settings", "/settings", "Settings");
    tab("files",    "/files",    "Files");
    tab("about",    "/about",    "About");
    s += F("<a class='reboot' href='/reboot' onclick=\"return confirm('Reboot device?')\">&#8635; Reboot</a>");
    s += F("<a class='logout' href='/logout'>Logout</a>");
    s += F("</nav>");
    return s;
}

void WifiApServer::sendPage(const char* title, const char* activeTab,
                             const String& body, bool withNav) {
    _server->setContentLength(CONTENT_LENGTH_UNKNOWN);
    _server->send(200, "text/html", "");
    _server->sendContent(F("<!DOCTYPE html><html><head>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>"));
    _server->sendContent(title);
    _server->sendContent(F(" - All-in-One-Weather-Station v" FIRMWARE_VERSION "</title><style>"));
    _server->sendContent(CSS);
    _server->sendContent(F("</style></head><body>"));
    _server->sendContent(topBar());
    if (withNav) _server->sendContent(navBar(activeTab));
    _server->sendContent(F("<div class='wrap'>"));
    _server->sendContent(body);
    _server->sendContent(F("</div></body></html>"));
}

/* ── event log ───────────────────────────────────────────────── */
String WifiApServer::eventLogPath() {
    if (!_st || !_st->time) return "/Event-00-00-0000.csv";
    char buf[28];
    snprintf(buf, sizeof(buf), "/Event-%02u-%02u-%04u.csv",
             _st->time->date, _st->time->month, _st->time->year);
    return String(buf);
}

void WifiApServer::appendEvent(const char* event) {
    if (!_st || !_st->time) return;
    esp_task_wdt_reset();
    String path = eventLogPath();
    bool newFile = !LittleFS.exists(path);
    File f = LittleFS.open(path, FILE_APPEND);
    if (!f) return;
    if (newFile) f.print("DateTime,Event\r\n");
    char line[160];
    snprintf(line, sizeof(line), "%02u/%02u/%04u %02u:%02u:%02u,%s\r\n",
             _st->time->date, _st->time->month, _st->time->year,
             _st->time->hour, _st->time->minute, _st->time->second,
             event);
    f.print(line);
    f.close();
}

bool WifiApServer::isDataFile(const String& name) {
    /* matches DD-MM-YYYY.csv  or  DATA*.csv */
    if (name == "DATA.csv") return true;
    if (name.length() == 14 && name.endsWith(".csv") &&
        name[2] == '-' && name[5] == '-') return true;
    return false;
}

/* ── begin / stop ────────────────────────────────────────────── */
bool WifiApServer::begin(SystemStatus* status) {
    if (_active) return true;
    _st = status;
    _prefs.begin("ws-cfg", false);

    Serial.printf("[WiFi] Starting AP: %s\n", WIFI_AP_SSID);
    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL)) {
        Serial.println(F("[WiFi] softAP FAILED"));
        WiFi.mode(WIFI_OFF);
        return false;
    }
    // Battery: the config AP is used at close range, so cut TX power from the
    // ~20 dBm default to 11 dBm to lower current draw during the wake window.
    WiFi.setTxPower(WIFI_POWER_11dBm);
    delay(100);
    Serial.printf("[WiFi] AP IP: %s\n", WiFi.softAPIP().toString().c_str());

    _server = new WebServer(80);
    const char* hdrs[] = {"Cookie"};
    _server->collectHeaders(hdrs, 1);

    auto tracked = [this](void (WifiApServer::*handler)()) {
        return [this, handler]() {
            markWebAccess();
            (this->*handler)();
        };
    };

    _server->on("/",          HTTP_GET,  tracked(&WifiApServer::hRoot));
    _server->on("/login",     HTTP_GET,  tracked(&WifiApServer::hLogin));
    _server->on("/login",     HTTP_POST, tracked(&WifiApServer::hLoginPost));
    _server->on("/logout",    HTTP_GET,  tracked(&WifiApServer::hLogout));
    _server->on("/live",         HTTP_GET, tracked(&WifiApServer::hLive));
    _server->on("/api/live",     HTTP_GET, tracked(&WifiApServer::hApiLive));
    _server->on("/ble/connect",   HTTP_GET, tracked(&WifiApServer::hBleConnect));
    _server->on("/ble/disconnect",HTTP_GET, tracked(&WifiApServer::hBleDisconnect));
    _server->on("/ble/scan",      HTTP_GET, tracked(&WifiApServer::hBleScan));
    _server->on("/ble/forget",    HTTP_GET, tracked(&WifiApServer::hBleForget));
    _server->on("/ble/refresh",   HTTP_GET, tracked(&WifiApServer::hBleRefresh));
    _server->on("/api/ble",       HTTP_GET, tracked(&WifiApServer::hApiBle));
    _server->on("/files",     HTTP_GET,  tracked(&WifiApServer::hFiles));
    _server->on("/download",  HTTP_GET,  tracked(&WifiApServer::hDownload));
    _server->on("/delete",    HTTP_POST, tracked(&WifiApServer::hDelete));
    _server->on("/deleteall", HTTP_POST, tracked(&WifiApServer::hDeleteAll));
    _server->on("/log",       HTTP_GET,  tracked(&WifiApServer::hLog));
    _server->on("/logdl",     HTTP_GET,  tracked(&WifiApServer::hLogDownload));
    _server->on("/settings",  HTTP_GET,  tracked(&WifiApServer::hSettings));
    _server->on("/setpwd",    HTTP_POST, tracked(&WifiApServer::hSetPassword));
    _server->on("/setap",     HTTP_POST, tracked(&WifiApServer::hSetAp));
    _server->on("/setble",    HTTP_POST, tracked(&WifiApServer::hSetBle));
    _server->on("/setsrc",    HTTP_POST, tracked(&WifiApServer::hSetSources));
    _server->on("/setsoil",   HTTP_POST, tracked(&WifiApServer::hSetSoil));
    _server->on("/setweath",  HTTP_POST, tracked(&WifiApServer::hSetWeather));
    _server->on("/setfile",   HTTP_POST, tracked(&WifiApServer::hSetFile));
    _server->on("/setntp",    HTTP_POST, tracked(&WifiApServer::hSetNtp));
    _server->on("/setmem",    HTTP_POST, tracked(&WifiApServer::hSetMem));
    _server->on("/ntpsync",   HTTP_GET,  tracked(&WifiApServer::hNtpSyncNow));
    _server->on("/about",     HTTP_GET,  tracked(&WifiApServer::hAbout));
    _server->on("/reboot",    HTTP_GET,  tracked(&WifiApServer::hReboot));
    _server->on("/update",    HTTP_GET,  tracked(&WifiApServer::hUpdate));
    _server->on("/update",    HTTP_POST, tracked(&WifiApServer::hUpdatePost),
                                         tracked(&WifiApServer::hUpdateUpload));
    _server->onNotFound(tracked(&WifiApServer::hNotFound));

    /* ArduinoOTA over AP */
    ArduinoOTA.setPassword(_prefs.getString("otapass", OTA_DEFAULT_PASSWORD).c_str());
    ArduinoOTA.begin();

    /* dynamic AP timeout from Preferences (minutes, 0=never) */
    uint32_t apMin = _prefs.getUInt("apTimeout", WIFI_AP_DEFAULT_IDLE_MIN);
    _apTimeoutMs  = (apMin == 0) ? 0UL : apMin * 60000UL;
    _noClientSince = 0;

    _server->begin();
    _startTime = millis();
    _lastWebAccessMs = _startTime;
    _webAccessSeen = false;
    _active    = true;
    Serial.printf("[WiFi] Web server started (timeout: %u min)\n", apMin);
    return true;
}

void WifiApServer::handleClient() {
    if (!_active || !_server) return;
    esp_task_wdt_reset();
    _server->handleClient();
    ArduinoOTA.handle();
}

void WifiApServer::stop() {
    bool wasActive = _active;

    _active = false;
    ArduinoOTA.end();

    if (_server) {
        _server->client().stop();
        delay(10);
        _server->close();
        _server->stop();
        delete _server;
        _server = nullptr;
        delay(20);
    }

    if (wasActive) {
        WiFi.softAPdisconnect(true);
        delay(50);
        WiFi.mode(WIFI_OFF);
        _prefs.end();
        Serial.println(F("[WiFi] AP stopped"));
    }
}

bool WifiApServer::isTimedOut() {
    if (!_active)           return true;

    if (!_webAccessSeen && (millis() - _startTime >= WEB_FIRST_ACCESS_TIMEOUT_MS)) {
        Serial.println(F("[WiFi] No web access within 30 seconds"));
        return true;
    }

    if (_apTimeoutMs == 0)  return false;  /* never */

    uint8_t clients = WiFi.softAPgetStationNum();
    if (clients > 0) {
        _noClientSince = 0;  /* client present — pause / reset countdown */
        return false;
    }
    /* no client connected */
    if (_noClientSince == 0) _noClientSince = millis();  /* start counting */
    return (millis() - _noClientSince >= _apTimeoutMs);
}

bool WifiApServer::isActive() { return _active; }

/* ═══════════════════════════════════════════════════════════════
   PAGE HANDLERS
   ═══════════════════════════════════════════════════════════════ */

void WifiApServer::hRoot() { redirectTo(_loggedIn ? "/live" : "/login"); }
void WifiApServer::hNotFound() { redirectTo("/login"); }
void WifiApServer::hLogout() {
    appendEvent("User logout");
    clearSession();
    redirectTo("/login");
}

/* ── LOGIN ─────────────────────────────────────────────────── */
void WifiApServer::hLogin() {
    String b = F("<div class='login-wrap'><div class='login-box'>"
        "<h2>&#127782; Weather Station</h2>"
        "<div class='sub'>All-in-One &nbsp;|&nbsp; v" FIRMWARE_VERSION "</div>"
        "<form method='POST' action='/login'>"
        "<label>Username</label>"
        "<input type='text' name='u' value='admin' required>"
        "<label>Password</label>"
        "<input type='password' name='p' required>"
        "<button class='btn btn-b' style='width:100%;padding:10px' type='submit'>Login</button>"
        "</form>"
        "</div></div>");
    _server->setContentLength(CONTENT_LENGTH_UNKNOWN);
    _server->send(200, "text/html", "");
    _server->sendContent(F("<!DOCTYPE html><html><head>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Login - All-in-One-Weather-Station v" FIRMWARE_VERSION "</title><style>"));
    _server->sendContent(CSS);
    _server->sendContent(F("</style></head><body>"));
    _server->sendContent(topBar());
    _server->sendContent(b);
    _server->sendContent(F("</body></html>"));
}

void WifiApServer::hLoginPost() {
    String u = _server->arg("u");
    String p = _server->arg("p");
    String storedPass = _prefs.getString("webpass", WEB_DEFAULT_PASS);
    String storedUser = WEB_DEFAULT_USER;

    char evBuf[80];
    String clientIp = _server->client().remoteIP().toString();

    if (u == storedUser && p == storedPass) {
        startSession();
        snprintf(evBuf, sizeof(evBuf), "Login SUCCESS user=%s ip=%s", u.c_str(), clientIp.c_str());
        appendEvent(evBuf);
        _server->sendHeader("Set-Cookie", String("sid=") + _token + "; Path=/");
        redirectTo("/live");
    } else {
        snprintf(evBuf, sizeof(evBuf), "Login FAILED user=%s ip=%s", u.c_str(), clientIp.c_str());
        appendEvent(evBuf);

        String b = F("<div class='login-wrap'><div class='login-box'>"
            "<h2>&#127782; Weather Station</h2>"
            "<div class='sub'>v" FIRMWARE_VERSION "</div>"
            "<div class='alert alert-r'>Invalid username or password</div>"
            "<form method='POST' action='/login'>"
            "<label>Username</label><input type='text' name='u' required>"
            "<label>Password</label><input type='password' name='p' required>"
            "<button class='btn btn-b' style='width:100%;padding:10px' type='submit'>Login</button>"
            "</form></div></div>");
        _server->setContentLength(CONTENT_LENGTH_UNKNOWN);
        _server->send(200, "text/html", "");
        _server->sendContent(F("<!DOCTYPE html><html><head>"
            "<meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>Login</title><style>"));
        _server->sendContent(CSS);
        _server->sendContent(F("</style></head><body>"));
        _server->sendContent(topBar());
        _server->sendContent(b);
        _server->sendContent(F("</body></html>"));
    }
}

/* ── LIVE ──────────────────────────────────────────────────── */
void WifiApServer::hLive() {
    if (!checkAuth()) return;
    String b;
    b.reserve(900);

    /* auto-refresh JS: sensor data every 30 s, BLE data every 5 s */
    b += F("<script>"
        "function rfSensor(){"
        "fetch('/api/live').then(r=>r.json()).then(d=>{"
        "for(var k in d){var e=document.getElementById(k);if(e)e.textContent=d[k];}"
        "})}"
        "function bleIco(n){"
        "if(n==='Timestamp')return'&#128336;';"
        "if(n.indexOf('Temp')>=0||n.indexOf('TMP')>=0)return'&#127777;';"
        "if(n.indexOf('Humid')>=0)return'&#128167;';"
        "if(n.indexOf('Delta')>=0)return'&#9651;';"
        "if(n.indexOf('Rain')>=0)return'&#127783;';"
        "if(n.indexOf('Leaf')>=0)return'&#127807;';"
        "if(n.indexOf('PAR')>=0)return'&#9728;';"
        "if(n.indexOf('Soil')>=0)return'&#127774;';"
        "return'&#128307;';}"
        "function reqNus(){"
        "fetch('/ble/refresh').catch(function(){});"
        "document.getElementById('nus-age').textContent='Requesting...';}"
        "function rfBle(){"
        "fetch('/api/ble').then(r=>r.json()).then(d=>{"
        "var div=document.getElementById('ble-live');"
        /* ── not connected ── */
        "if(!d.connected){"
        "div.innerHTML='<p style=\"color:#a78bfa;font-size:13px\">&#128268; Not connected &mdash; "
        "use Settings &#8594; BLE Client to scan and connect.</p>';return;}"
        /* ── connection header ── */
        "var h='<div class=\"row\" style=\"margin-bottom:8px;align-items:center\">';"
        "h+='<span class=\"tag g\">&#128994; Connected</span>';"
        "h+='<span class=\"tag\" style=\"font-family:monospace\">'+d.mac+'</span>';"
        "h+='<span class=\"tag\">'+d.name+'</span>';"
        "if(d.nus)h+='<span class=\"tag y\">&#128301; Sniffer NUS</span>';"
        "h+='<a class=\"btn btn-r\" style=\"padding:3px 10px;font-size:12px;margin-left:auto\" "
        "href=\"/ble/disconnect\">Disconnect</a></div>';"
        /* ── NUS path ── */
        "if(d.nus){"
        "if(!d.hasData){"
        /* waiting for first notification */
        "h+='<div style=\"padding:16px;text-align:center;color:#a78bfa\">';"
        "h+='&#9711; Connected to Sniffer Portal<br>"
        "<small>Waiting for first data notification (up to 10 s)&hellip;</small>';"
        "h+='<br><br><button class=\"btn btn-p\" style=\"padding:4px 12px;font-size:12px\" "
        "onclick=\"reqNus()\">&#8635; Request Now</button></div>';"
        "}else{"
        /* age bar */
        "var ageStr=d.age<5?'just now':d.age+'s ago';"
        "h+='<div style=\"display:flex;justify-content:space-between;align-items:center;"
        "font-size:11px;color:#a78bfa;margin-bottom:8px\">';"
        "h+='<span>Last update: <strong id=\"nus-age\">'+ageStr+'</strong></span>';"
        "h+='<button class=\"btn btn-p\" style=\"padding:2px 10px;font-size:11px\" "
        "onclick=\"reqNus()\">&#8635; Request Update</button></div>';"
        /* sensor grid */
        "h+='<div style=\"display:grid;grid-template-columns:repeat(3,1fr);gap:8px\">';"
        "d.chars.forEach(function(c){"
        "var ico=bleIco(c.uuid);"
        "var stale=d.age>30?'opacity:.55':'opacity:1';"
        "h+='<div style=\"background:#1a1629;border:1px solid #30363d;border-radius:6px;padding:10px;"
        "text-align:center;'+stale+'\">';"
        "h+='<div style=\"font-size:11px;color:#a78bfa;margin-bottom:3px\">';"
        "h+=ico+' '+c.uuid+'</div>';"
        "h+='<div style=\"font-size:17px;font-weight:700;color:#e6edf3\">'+c.ascii+'</div>';"
        "h+='</div>';});"
        "h+='</div>';}"
        /* ── GATT path ── */
        "}else{"
        "if(d.chars.length===0){"
        "h+='<p style=\"color:#a78bfa;font-size:13px\">Connected (GATT). No readable characteristics found.</p>';"
        "}else{"
        "h+='<table><tr><th>Characteristic UUID</th><th>Hex</th><th>ASCII</th></tr>';"
        "d.chars.forEach(function(c){"
        "h+='<tr><td style=\"font-family:monospace;font-size:11px\">'+c.uuid+'</td>';"
        "h+='<td style=\"font-family:monospace\">'+c.hex+'</td>';"
        "h+='<td>'+c.ascii+'</td></tr>';});"
        "h+='</table>';}}"
        "div.innerHTML=h;"
        "}).catch(function(){})}"
        "setInterval(rfSensor,30000);"
        "setInterval(rfBle,5000);"
        "rfBle();"
        "</script>");

    /* Status row */
    b += F("<div class='card'><h3>&#128268; System Status</h3><div class='row'>");
    b += F("<span class='tag g'>LoRa: E32-433T30S</span>");
    uint8_t apClients = WiFi.softAPgetStationNum();
    b += F("<span class='tag g'>WiFi: AP "); b += WIFI_AP_SSID;
    b += F(" ("); b += apClients; b += F(" client"); b += (apClients != 1 ? "s" : ""); b += F(")</span>");
    bool ble = _st->bleActive && *_st->bleActive;
    b += F("<span class='tag "); b += ble ? "g" : "y";
    b += F("'>BLE: "); b += ble ? "Active" : "Inactive"; b += F("</span>");
    b += F("</div></div>");

    /* Battery & Memory */
    b += F("<div class='grid2'>");
    b += F("<div class='card'><h3>&#128267; Battery</h3>");
    b += F("<span class='tag' id='batt'>");
    b += (_st->battMv ? *_st->battMv : 0);
    b += F(" mV</span></div>");

    b += F("<div class='card'><h3>&#128190; Memory</h3>");
    if (_st->memory) {
        uint64_t used  = LittleFS.usedBytes();
        uint64_t total = LittleFS.totalBytes();
        uint8_t  pct   = total ? (used * 100 / total) : 0;
        b += F("<span class='tag ");
        b += (pct > 80 ? "r" : pct > 60 ? "y" : "g");
        b += F("'>"); b += pct; b += F("% used</span> ");
        b += fmtBytes(used); b += F(" / "); b += fmtBytes(total);
    }
    b += F("</div></div>");

    /* Soil */
    b += F("<div class='card'><h3>&#127807; Soil Sensor</h3>"
           "<div style='display:grid;grid-template-columns:repeat(3,1fr);gap:8px'>");
    if (_st->sensor) {
        auto& s = *_st->sensor;
        auto cell = [&](const char* ico, const char* lbl, const char* id, String v, const char* u) {
            b += F("<div style='background:#1a1629;border:1px solid #30363d;border-radius:6px;"
                   "padding:10px;text-align:center'>"
                   "<div style='font-size:11px;color:#a78bfa;margin-bottom:3px'>"); b += ico; b += " "; b += lbl;
            b += F("</div><div style='font-size:17px;font-weight:700;color:#e6edf3'>"
                   "<span id='"); b += id; b += F("'>"); b += v; b += F("</span>");
            if (u[0]) { b += F("<span style='font-size:12px;color:#a78bfa;margin-left:3px'>"); b += u; b += F("</span>"); }
            b += F("</div></div>");
        };
        cell("&#128167;", "Moisture",    "sh",  String(s.soil_humi/10.0,1),  "%");
        cell("&#127777;", "Temperature", "st",  String(s.soil_temp/10.0,1),  "°C");
        cell("&#9889;",   "EC",          "se",  String(s.soil_ec),           "µS/cm");
        cell("&#9879;",   "pH",          "sp",  String(s.soil_ph/10.0,1),    "");
        cell("&#127807;", "Nitrogen",    "sn",  String(s.soil_N),            "mg/kg");
        cell("&#127807;", "Phosphorus",  "spv", String(s.soil_P),            "mg/kg");
        cell("&#127807;", "Potassium",   "sk",  String(s.soil_K),            "mg/kg");
    }
    b += F("</div></div>");

    /* Weather */
    b += F("<div class='card'><h3>&#9925; Weather Sensor</h3>"
           "<div style='display:grid;grid-template-columns:repeat(3,1fr);gap:8px'>");
    if (_st->sensor) {
        auto& s = *_st->sensor;
        auto cell = [&](const char* ico, const char* lbl, const char* id, String v, const char* u) {
            b += F("<div style='background:#1a1629;border:1px solid #30363d;border-radius:6px;"
                   "padding:10px;text-align:center'>"
                   "<div style='font-size:11px;color:#a78bfa;margin-bottom:3px'>"); b += ico; b += " "; b += lbl;
            b += F("</div><div style='font-size:17px;font-weight:700;color:#e6edf3'>"
                   "<span id='"); b += id; b += F("'>"); b += v; b += F("</span>");
            if (u[0]) { b += F("<span style='font-size:12px;color:#a78bfa;margin-left:3px'>"); b += u; b += F("</span>"); }
            b += F("</div></div>");
        };
        cell("&#128168;", "Wind Speed",  "ws",  String(s.windSpeed/10.0,1),          "m/s");
        cell("&#129517;", "Wind Dir",    "wd",  String(s.windDir_Deg),               "°");
        cell("&#128167;", "Humidity",    "ah",  String(s.air_humidity/10.0,1),       "%");
        cell("&#127777;", "Temp",        "at",  String(s.air_temperature/10.0,1),    "°C");
        cell("&#127783;", "CO&#8322;",   "co2", String(s.CO2),                       "ppm");
        cell("&#127760;", "Pressure",    "pr",  String(s.pressure/10.0,1),           "kPa");
        cell("&#9728;",   "Illuminance", "il",  String((unsigned long)s.illuminance),"lux");
        cell("&#127783;", "Rainfall",    "rf",  String(s.rainfall/10.0,1),           "mm");
        cell("&#128262;", "Solar",       "so",  String(s.solar),                     "W/m²");
    }
    b += F("</div></div>");

    /* BLE Sensor Data — live-refresh via JS every 5 s */
    b += F("<div class='card'><h3>&#128301; BLE Sensor Data"
           "<span style='float:right;font-size:11px;color:#a78bfa'>&#8635; auto</span></h3>"
           "<div id='ble-live'><p style='color:#1d4ed8;font-size:13px'>Loading&hellip;</p></div>"
           "</div>");

    sendPage("Live", "live", b);
}

void WifiApServer::hApiLive() {
    if (!_st || !_st->sensor) { _server->send(200, "application/json", "{}"); return; }
    char json[320];
    auto& s = *_st->sensor;
    snprintf(json, sizeof(json),
        "{\"batt\":\"%u mV\","
        "\"sh\":\"%.1f\",\"st\":\"%.1f\",\"se\":\"%u\","
        "\"sp\":\"%.1f\",\"sn\":\"%u\",\"spv\":\"%u\",\"sk\":\"%u\","
        "\"ws\":\"%.1f\",\"wd\":\"%u\",\"ah\":\"%.1f\",\"at\":\"%.1f\","
        "\"co2\":\"%u\",\"pr\":\"%.1f\",\"il\":\"%lu\",\"rf\":\"%.1f\",\"so\":\"%u\"}",
        (_st->battMv  ? *_st->battMv  : 0),
        s.soil_humi/10.0, s.soil_temp/10.0, s.soil_ec,
        s.soil_ph/10.0,   s.soil_N, s.soil_P, s.soil_K,
        s.windSpeed/10.0, s.windDir_Deg,
        s.air_humidity/10.0, s.air_temperature/10.0,
        s.CO2, s.pressure/10.0, (unsigned long)s.illuminance,
        s.rainfall/10.0, s.solar);
    _server->send(200, "application/json", json);
}

/* ── FILES ─────────────────────────────────────────────────── */
void WifiApServer::hFiles() {
    if (!checkAuth()) return;
    String b;
    b.reserve(700);

    b += F("<div class='card'><h3>&#128196; Data Files</h3>"
           "<table><tr><th>File</th><th>Size</th><th>Actions</th></tr>");

    File root = LittleFS.open("/");
    File file = root.openNextFile();
    bool any = false;
    while (file) {
        String fname = String(file.name());
        if (!fname.startsWith("/")) fname = "/" + fname;
        String bare = fname.substring(1);  /* strip leading / */
        if (isDataFile(bare)) {
            any = true;
            b += F("<tr><td>"); b += bare; b += F("</td><td>");
            b += fmtBytes(file.size());
            b += F("</td><td>"
                "<a class='btn btn-b' href='/download?f=");
            b += bare;
            b += F("'>&#11123; DL</a> "
                "<form style='display:inline' method='POST' action='/delete'>"
                "<input type='hidden' name='f' value='"); b += bare;
            b += F("'><button class='btn btn-r' type='submit' "
                "onclick=\"return confirm('Delete "); b += bare; b += F("?')\""
                ">&#128465;</button></form></td></tr>");
        }
        file = root.openNextFile();
    }
    if (!any) b += F("<tr><td colspan='3' style='text-align:center;color:#9ca3af'>No files</td></tr>");
    b += F("</table></div>");

    b += F("<form method='POST' action='/deleteall' "
           "onsubmit=\"return confirm('Delete ALL data files?')\">"
           "<button class='btn btn-r' type='submit'>&#128465; Delete ALL Files</button>"
           "</form>");

    sendPage("Files", "files", b);
}

void WifiApServer::hDownload() {
    if (!checkAuth()) return;
    String bare = _server->arg("f");
    if (bare.startsWith("/")) bare = bare.substring(1);
    if (!isDataFile(bare)) { _server->send(403, "text/plain", "Forbidden"); return; }
    String fname = "/" + bare;
    if (!LittleFS.exists(fname)) { _server->send(404, "text/plain", "Not found"); return; }
    File f = LittleFS.open(fname, "r");
    if (!f) { _server->send(500, "text/plain", "Open failed"); return; }
    String disp = "attachment; filename=\"" + bare + "\"";
    _server->sendHeader("Content-Disposition", disp);
    _server->sendHeader("Content-Length", String(f.size()));
    esp_task_wdt_reset();
    _server->streamFile(f, "text/csv");
    f.close();
}

void WifiApServer::hDelete() {
    if (!checkAuth()) return;
    String bare = _server->arg("f");
    if (bare.startsWith("/")) bare = bare.substring(1);
    bool allowed = isDataFile(bare) || (bare.startsWith("Event-") && bare.endsWith(".csv"));
    if (!allowed) { _server->send(403, "text/plain", "Forbidden"); return; }
    String fname = "/" + bare;
    char ev[80]; snprintf(ev, sizeof(ev), "Delete file %s", fname.c_str());
    if (LittleFS.remove(fname)) appendEvent(ev);
    redirectTo("/files");
}

void WifiApServer::hDeleteAll() {
    if (!checkAuth()) return;
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while (file) {
        String fname = String(file.name());
        file = root.openNextFile();
        if (!fname.startsWith("/")) fname = "/" + fname;
        String bare = fname.substring(1);
        if (isDataFile(bare)) LittleFS.remove(fname);
    }
    appendEvent("Delete ALL data files");
    redirectTo("/files");
}

/* ── LOG ───────────────────────────────────────────────────── */
void WifiApServer::hLog() {
    if (!checkAuth()) return;
    String b;
    b.reserve(600);
    b += F("<div class='card'><h3>&#128203; Event Log Files</h3>"
           "<table><tr><th>File</th><th>Size</th><th>Actions</th></tr>");

    File root = LittleFS.open("/");
    File file = root.openNextFile();
    bool any = false;
    while (file) {
        String fname = String(file.name());
        if (!fname.startsWith("/")) fname = "/" + fname;
        String bare = fname.substring(1);
        if (bare.startsWith("Event-") && bare.endsWith(".csv")) {
            any = true;
            b += F("<tr><td>"); b += bare; b += F("</td><td>");
            b += fmtBytes(file.size());
            b += F("</td><td>"
                "<a class='btn btn-y' href='/logdl?f="); b += bare;
            b += F("'>&#11123; DL</a> "
                "<form style='display:inline' method='POST' action='/delete'>"
                "<input type='hidden' name='f' value='"); b += bare;
            b += F("'><button class='btn btn-r' type='submit' "
                "onclick=\"return confirm('Delete?')\">&#128465;</button></form></td></tr>");
        }
        file = root.openNextFile();
    }
    if (!any) b += F("<tr><td colspan='3' style='text-align:center;color:#9ca3af'>No log files yet</td></tr>");
    b += F("</table></div>");

    /* Show last 20 lines of today's log */
    String today = eventLogPath();
    if (LittleFS.exists(today)) {
        b += F("<div class='card'><h3>Today's Log</h3>"
               "<table><tr><th>DateTime</th><th>Event</th></tr>");
        File lf = LittleFS.open(today, "r");
        if (lf) {
            lf.readStringUntil('\n'); /* skip header */
            String lines[20]; int n = 0, idx = 0;
            while (lf.available()) {
                String line = lf.readStringUntil('\n');
                line.trim();
                if (line.length() > 0) { lines[idx % 20] = line; idx++; n = min(idx, 20); }
            }
            lf.close();
            int start = (idx > 20) ? (idx % 20) : 0;
            for (int i = 0; i < n; i++) {
                String& ln = lines[(start + i) % 20];
                int c = ln.indexOf(',');
                if (c > 0) {
                    b += F("<tr><td>"); b += ln.substring(0, c);
                    b += F("</td><td>"); b += ln.substring(c + 1); b += F("</td></tr>");
                }
            }
        }
        b += F("</table></div>");
    }

    sendPage("Log", "log", b);
}

void WifiApServer::hLogDownload() {
    if (!checkAuth()) return;
    String bare = _server->arg("f");
    if (bare.startsWith("/")) bare = bare.substring(1);
    if (!bare.startsWith("Event-") || !bare.endsWith(".csv")) { _server->send(403, "text/plain", "Forbidden"); return; }
    String fname = "/" + bare;
    if (!LittleFS.exists(fname)) { _server->send(404, "text/plain", "Not found"); return; }
    File f = LittleFS.open(fname, "r");
    if (!f) { _server->send(500, "text/plain", "Open failed"); return; }
    String disp = "attachment; filename=\"" + bare + "\"";
    _server->sendHeader("Content-Disposition", disp);
    _server->sendHeader("Content-Length", String(f.size()));
    esp_task_wdt_reset();
    _server->streamFile(f, "text/csv");
    f.close();
}

/* ── SETTINGS HELPERS ─────────────────────────────────────── */
void WifiApServer::startSettingsPage(const String& msg) {
    _server->setContentLength(CONTENT_LENGTH_UNKNOWN);
    _server->send(200, "text/html", "");
    _server->sendContent(F("<!DOCTYPE html><html><head>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Settings - v" FIRMWARE_VERSION "</title><style>"));
    _server->sendContent(CSS);
    _server->sendContent(F("</style></head><body>"));
    _server->sendContent(topBar());
    _server->sendContent(navBar("settings"));
    _server->sendContent(F("<div class='wrap'>"));
    if (msg.length()) {
        bool ok = msg.startsWith("ok");
        _server->sendContent(F("<div class='alert "));
        _server->sendContent(ok ? "alert-g" : "alert-r");
        _server->sendContent(F("'>"));
        _server->sendContent(msg.substring(3));
        _server->sendContent(F("</div>"));
    }
}
void WifiApServer::endSettingsPage() {
    _server->sendContent(F("</div></body></html>"));
}
void WifiApServer::sendSettingsChunk(const String& html) {
    _server->sendContent(html);
}

/* ── SETTINGS PAGE ─────────────────────────────────────────── */
void WifiApServer::hSettings() {
    if (!checkAuth()) return;
    startSettingsPage(_server->arg("msg"));

    /* ── WiFi AP ── */
    uint32_t apMin = _prefs.getUInt("apTimeout", 5);
    {
        String c = F("<div class='card'><h3>&#128246; WiFi AP</h3>"
            "<p style='color:#6b7280;font-size:12px;margin-bottom:10px'>"
            "AP closes after the selected idle period with <strong>no connected client</strong>. "
            "Resets automatically when a client connects.</p>"
            "<form method='POST' action='/setap'>"
            "<label>Close AP after idle (no clients)</label>"
            "<select name='apt'>");
        auto opt = [&](uint32_t v, const char* lbl) {
            c += F("<option value='"); c += v;
            if (v == apMin) c += F("' selected>");
            else c += F("'>");
            c += lbl; c += F("</option>");
        };
        opt(1,"1 minute"); opt(5,"5 minutes"); opt(10,"10 minutes");
        opt(15,"15 minutes"); opt(30,"30 minutes"); opt(60,"1 hour"); opt(0,"Never");
        c += F("</select>"
            "<button class='btn btn-b' type='submit'>Save</button>"
            "</form></div>");
        sendSettingsChunk(c);
    }

    /* ── BLE Client ── */
    {
        bool   bleEn  = _prefs.getBool("bleEnable", true);
        String saved  = (_st->bleSavedMac && strlen(_st->bleSavedMac)) ? String(_st->bleSavedMac) : "";
        bool   isConn = _st->bleConnDev && _st->bleConnDev->connected;
        uint8_t cnt   = (_st->bleDeviceCount ? *_st->bleDeviceCount : 0);

        String c = F("<div class='card'><h3>&#128268; BLE Client</h3>"
            "<form method='POST' action='/setble' style='margin-bottom:12px'>"
            "<label style='display:flex;align-items:center;gap:8px;font-weight:normal;margin-bottom:10px'>"
            "<input type='checkbox' name='en' value='1'");
        if (bleEn) c += F(" checked");
        c += F("> Enable BLE Client &mdash; scan &amp; connect to BLE sensors</label>"
            "<button class='btn btn-b' type='submit'>Save</button>"
            "</form>"
            "<hr style='margin:10px 0;border-color:#ede9fe'>");

        /* saved device + forget */
        if (saved.length()) {
            c += F("<div class='row' style='margin-bottom:10px'>"
                   "<span class='tag' style='font-family:monospace'>Saved: "); c += saved;
            c += F("</span>"
                   "<a class='btn btn-r' style='padding:4px 10px;font-size:12px' "
                   "href='/ble/forget' onclick=\"return confirm('Forget saved device?')\">"
                   "&#10060; Forget</a></div>");
        }

        /* scan button */
        c += F("<a class='btn btn-p' href='/ble/scan'>&#128269; Scan Now</a>");

        /* scan results */
        if (!isConn && cnt > 0) {
            c += F("<table style='margin-top:10px'>"
                   "<tr><th>Name</th><th>MAC</th><th>RSSI</th><th></th></tr>");
            for (uint8_t i = 0; i < cnt; i++) {
                const BLEFoundDevice& dev = _st->bleDevices[i];
                c += F("<tr><td>"); c += dev.name;
                c += F("</td><td style='font-family:monospace;font-size:12px'>"); c += dev.mac;
                c += F("</td><td>"); c += dev.rssi; c += F(" dBm</td><td>"
                       "<a class='btn btn-p' style='padding:4px 10px;font-size:12px' "
                       "href='/ble/connect?mac="); c += dev.mac; c += F("'>Connect</a>"
                       "</td></tr>");
            }
            c += F("</table>");
        } else if (!isConn && cnt == 0) {
            c += F("<p style='color:#9ca3af;font-size:12px;margin-top:8px'>"
                   "No results yet &mdash; press Scan Now.</p>");
        }

        c += F("</div>");
        sendSettingsChunk(c);
    }

    /* ── Data Sources ── */
    {
        bool srcMod = _prefs.getBool("srcModbus", true);
        bool srcBle = _prefs.getBool("srcBle",    true);
        String c = F("<div class='card'><h3>&#128202; Data Sources</h3>"
            "<form method='POST' action='/setsrc'>"
            "<label style='display:flex;align-items:center;gap:8px;font-weight:normal;margin-bottom:8px'>"
            "<input type='checkbox' name='mod' value='1'");
        if (srcMod) c += F(" checked");
        c += F("> Modbus RS485</label>"
            "<label style='display:flex;align-items:center;gap:8px;font-weight:normal;margin-bottom:10px'>"
            "<input type='checkbox' name='ble' value='1'");
        if (srcBle) c += F(" checked");
        c += F("> BLE Sensor</label>"
            "<button class='btn btn-b' type='submit'>Save</button>"
            "</form></div>");
        sendSettingsChunk(c);
    }

    /* ── Soil Sensor (RS485) ── */
    {
        uint8_t soilId = _prefs.getUChar("soilSlaveId", SOIL_SLAVE_ID);
        String c = F("<div class='card'><h3>&#127793; Soil Sensor (RS485)</h3>"
            "<form method='POST' action='/setsoil'>"
            "<label>Slave ID (1-247)</label>"
            "<input type='number' name='sid' min='1' max='247' value='"); c += soilId;
        c += F("'>"
            "<button class='btn btn-b' type='submit'>Save</button>"
            "</form></div>");
        sendSettingsChunk(c);
    }

    /* ── Weather Sensor (RS485) ── */
    {
        uint8_t weathId = _prefs.getUChar("weathSlaveId", WEATHER_SLAVE_ID);
        String c = F("<div class='card'><h3>&#9925; Weather Sensor (RS485)</h3>"
            "<form method='POST' action='/setweath'>"
            "<label>Slave ID (1-247)</label>"
            "<input type='number' name='wid' min='1' max='247' value='"); c += weathId;
        c += F("'>"
            "<button class='btn btn-b' type='submit'>Save</button>"
            "</form></div>");
        sendSettingsChunk(c);
    }

    /* ── File & Publish ── */
    {
        uint32_t dInt = _prefs.getUInt("fileInterval", TIME_INCREMENT_MINUTES);
        uint8_t pubBatch = _prefs.getUChar("pubBatchSize", PUBLISH_BATCH_SIZE);
        uint8_t pubMode = _prefs.getUChar("pubMode", 0);
        uint8_t hbMode = _prefs.getUChar("hbMode", HEARTBEAT_DEFAULT_MODE);
        uint8_t hbEvery = _prefs.getUChar("hbEvery", HEARTBEAT_DEFAULT_EVERY_CYCLES);
        if (hbMode > HEARTBEAT_MODE_OFF) hbMode = HEARTBEAT_DEFAULT_MODE;
        if (hbEvery < 1) hbEvery = HEARTBEAT_DEFAULT_EVERY_CYCLES;
        String c = F("<div class='card'><h3>&#128196; File &amp; Publish</h3>"
            "<form method='POST' action='/setfile'>"
            "<label>Publish Mode</label>"
            "<div style='margin-bottom:12px'>"
            "<label style='display:flex;align-items:center;gap:8px;font-weight:normal;margin-bottom:6px'>"
            "<input type='radio' name='pm' value='0'");
        if (pubMode == 0) c += F(" checked");
        c += F("> LoRa Publish</label>"
            "<label style='display:flex;align-items:center;gap:8px;font-weight:normal'>"
            "<input type='radio' name='pm' value='1'");
        if (pubMode == 1) c += F(" checked");
        c += F("> Reserved</label>"
            "</div>"
            "<label>Fallback time increment (minutes)</label>"
            "<input type='number' name='di' min='1' max='1440' value='"); c += dInt;
        c += F("'>"
            "<div id='batch-row'");
        if (pubMode != 0) c += F(" style='display:none'");
        c += F(">"
            "<label>Publish batch size (records)</label>"
            "<input type='number' name='pbs' min='1' max='60' value='"); c += pubBatch;
        c += F("'>"
            "</div>"
            "<label>Heartbeat Policy</label>"
            "<select name='hbm' id='hbm'>");
        auto hbOpt = [&](uint8_t v, const char* lbl) {
            c += F("<option value='"); c += v;
            if (v == hbMode) c += F("' selected>");
            else c += F("'>");
            c += lbl; c += F("</option>");
        };
        hbOpt(HEARTBEAT_MODE_EVERY_CYCLE, "Every cycle");
        hbOpt(HEARTBEAT_MODE_EVERY_N_CYCLES, "Every N cycles");
        hbOpt(HEARTBEAT_MODE_NO_DATA_ONLY, "Only when no data sent");
        hbOpt(HEARTBEAT_MODE_N_CYCLES_NO_DATA, "Every N cycles when no data sent");
        hbOpt(HEARTBEAT_MODE_OFF, "Off");
        c += F("</select>"
            "<div id='hb-every-row'>"
            "<label>Heartbeat every (cycles)</label>"
            "<input type='number' name='hbn' min='1' max='240' value='"); c += hbEvery;
        c += F("'>"
            "</div>"
            "<button class='btn btn-b' type='submit'>Save</button>"
            "</form>"
            "<script>"
            "(function(){"
            "var r=document.querySelectorAll('input[name=pm]');"
            "function u(){document.getElementById('batch-row').style.display=r[0].checked?'':'none';}"
            "r.forEach(function(x){x.addEventListener('change',u);});u();"
            "var h=document.getElementById('hbm'),hr=document.getElementById('hb-every-row');"
            "function uh(){hr.style.display=(h.value=='1'||h.value=='3')?'':'none';}"
            "h.addEventListener('change',uh);uh();"
            "})();"
            "</script>"
            "</div>");
        sendSettingsChunk(c);
    }

    /* ── NTP ── */
    {
        bool ntpEn = _prefs.getBool("ntpEnable", true);
        String c = F("<div class='card'><h3>&#128336; NTP Sync</h3>"
            "<form method='POST' action='/setntp'>"
            "<label style='display:flex;align-items:center;gap:8px;font-weight:normal;margin-bottom:12px'>"
            "<input type='checkbox' name='en' value='1'");
        if (ntpEn) c += F(" checked");
        c += F("> Sync time via BLE NUS when connected</label>"
            "<button class='btn btn-b' type='submit'>Save</button>"
            "</form></div>");
        sendSettingsChunk(c);
    }

    /* ── Internal Memory ── */
    {
        uint8_t  memPct  = (uint8_t)_prefs.getUInt("memRollover", 80);
        bool     memEn   = _prefs.getBool("memRolloverEn", true);
        uint64_t fsUsed  = LittleFS.usedBytes();
        uint64_t fsTotal = LittleFS.totalBytes();
        uint8_t  fsPct   = fsTotal ? (uint8_t)(fsUsed * 100 / fsTotal) : 0;
        const char* barCol = (fsPct > 80) ? "#dc2626" : (fsPct > 60) ? "#d97706" : "#059669";
        String c = F("<div class='card'><h3>&#128190; Internal Memory</h3>");
        c += F("<div style='margin-bottom:14px'>"
               "<div style='display:flex;justify-content:space-between;"
               "font-size:12px;color:#a78bfa;margin-bottom:4px'>"
               "<span>Used: "); c += fmtBytes((uint32_t)fsUsed);
        c += F("</span><span>Free: "); c += fmtBytes((uint32_t)(fsTotal - fsUsed));
        c += F("</span><span>Total: "); c += fmtBytes((uint32_t)fsTotal);
        c += F("</span></div>"
               "<div style='background:#2d1a5c;border-radius:6px;height:18px;overflow:hidden'>"
               "<div style='background:"); c += barCol;
        c += F(";height:100%;width:"); c += fsPct;
        c += F("%'></div></div>"
               "<div style='text-align:center;font-size:12px;color:#c4b5fd;margin-top:3px'>"
               "<strong>"); c += fsPct;
        c += F("% used</strong></div></div>");
        c += F("<form method='POST' action='/setmem'>"
            "<label>Rollover at (%) <span id='mpv'>");  c += memPct; c += F("</span>%</label>"
            "<input type='range' name='pct' min='10' max='90' value='"); c += memPct;
        c += F("' oninput=\"document.getElementById('mpv').textContent=this.value\""
            " style='width:100%;margin-bottom:10px'>"
            "<label style='display:flex;align-items:center;gap:8px;font-weight:normal;margin-bottom:10px'>"
            "<input type='checkbox' name='en' value='1'");
        if (memEn) c += F(" checked");
        c += F("> Enable rollover (keep last entry)</label>"
            "<button class='btn btn-b' type='submit'>Save</button>"
            "</form></div>");
        sendSettingsChunk(c);
    }

    /* ── Change Password ── */
    {
        String c = F("<div class='card'><h3>&#128272; Change Password</h3>"
            "<form method='POST' action='/setpwd'>"
            "<label>New Password</label>"
            "<input type='password' name='p1' placeholder='New password' required>"
            "<label>Confirm Password</label>"
            "<input type='password' name='p2' placeholder='Confirm' required>"
            "<button class='btn btn-p' type='submit'>Save Password</button>"
            "</form></div>");
        sendSettingsChunk(c);
    }

    endSettingsPage();
}

/* ── SETTINGS SAVE HANDLERS ──────────────────────────────── */
void WifiApServer::hSetPassword() {
    if (!checkAuth()) return;
    String p1 = _server->arg("p1"), p2 = _server->arg("p2");
    if (p1 != p2 || p1.length() < 4) redirectTo("/settings?msg=er:Passwords don't match or too short");
    else { _prefs.putString("webpass", p1); appendEvent("Password changed"); redirectTo("/settings?msg=ok:Password updated"); }
}

void WifiApServer::hSetAp() {
    if (!checkAuth()) return;
    _prefs.putUInt("apTimeout", (uint32_t)_server->arg("apt").toInt());
    appendEvent("AP timeout updated");
    redirectTo("/settings?msg=ok:AP timeout saved");
}

void WifiApServer::hSetBle() {
    if (!checkAuth()) return;
    bool en = _server->arg("en") == "1";
    _prefs.putBool("bleEnable", en);
    if (!en && _st && _st->bleDisconnectRequest) *_st->bleDisconnectRequest = true;
    appendEvent("BLE Client settings updated");
    redirectTo("/settings?msg=ok:BLE settings saved");
}

void WifiApServer::hSetSources() {
    if (!checkAuth()) return;
    _prefs.putBool("srcModbus", _server->arg("mod") == "1");
    _prefs.putBool("srcBle",    _server->arg("ble") == "1");
    appendEvent("Data sources updated");
    redirectTo("/settings?msg=ok:Data sources saved");
}

void WifiApServer::hSetSoil() {
    if (!checkAuth()) return;
    int sid = _server->arg("sid").toInt();
    if (sid >= 1 && sid <= 247) _prefs.putUChar("soilSlaveId", (uint8_t)sid);
    appendEvent("Soil sensor updated");
    redirectTo("/settings?msg=ok:Soil sensor saved");
}

void WifiApServer::hSetWeather() {
    if (!checkAuth()) return;
    int wid = _server->arg("wid").toInt();
    if (wid >= 1 && wid <= 247) _prefs.putUChar("weathSlaveId", (uint8_t)wid);
    appendEvent("Weather sensor updated");
    redirectTo("/settings?msg=ok:Weather sensor saved");
}

void WifiApServer::hSetFile() {
    if (!checkAuth()) return;
    uint32_t di = (uint32_t)_server->arg("di").toInt();
    if (di < 1) di = 1;
    if (di > 1440) di = 1440;
    _prefs.putUInt("fileInterval", di);
    int pm = _server->arg("pm").toInt();
    if (pm == 0 || pm == 1) _prefs.putUChar("pubMode", (uint8_t)pm);
    int pbs = _server->arg("pbs").toInt();
    if (pbs >= 1 && pbs <= 60) _prefs.putUChar("pubBatchSize", (uint8_t)pbs);
    int hbm = _server->arg("hbm").toInt();
    if (hbm >= HEARTBEAT_MODE_EVERY_CYCLE && hbm <= HEARTBEAT_MODE_OFF) {
        _prefs.putUChar("hbMode", (uint8_t)hbm);
    }
    int hbn = _server->arg("hbn").toInt();
    if (hbn < 1) hbn = HEARTBEAT_DEFAULT_EVERY_CYCLES;
    if (hbn > 240) hbn = 240;
    _prefs.putUChar("hbEvery", (uint8_t)hbn);
    _prefs.putUInt("hbCycle", 0);
    appendEvent("File & publish settings updated");
    redirectTo("/settings?msg=ok:Settings saved");
}

void WifiApServer::hNtpSyncNow() {
    if (!checkAuth()) return;
    if (!_st || !_st->bleIsNus || !*_st->bleIsNus ||
        !_st->bleConnDev || !_st->bleConnDev->connected ||
        _st->bleConnDev->charCount == 0) {
        redirectTo("/settings?msg=er:BLE NUS not connected");
        return;
    }
    if (!_st->time) {
        redirectTo("/settings?msg=er:No time struct");
        return;
    }
    /* chars[0] = ts field, accepts "DD-MM-YYYY HH:MM:SS" or "YYYY-MM-DD HH:MM:SS" */
    const char* ts = _st->bleConnDev->chars[0].ascii;
    int a = 0, b = 0, c = 0, hr = 0, mn = 0, sc = 0;
    int yr = 0, mo = 0, dy = 0;
    if (sscanf(ts, "%d-%d-%d %d:%d:%d", &a, &b, &c, &hr, &mn, &sc) != 6) {
        redirectTo("/settings?msg=er:BLE timestamp invalid");
        return;
    }
    if (a >= 2020) { yr = a; mo = b; dy = c; }
    else           { dy = a; mo = b; yr = c; }
    if (yr < 2020 || mo < 1 || mo > 12 || dy < 1 || dy > 31 ||
        hr < 0 || hr > 23 || mn < 0 || mn > 59 || sc < 0 || sc > 59) {
        redirectTo("/settings?msg=er:BLE timestamp invalid");
        return;
    }
    _st->time->year   = (uint16_t)yr;
    _st->time->month  = (uint8_t)mo;
    _st->time->date   = (uint8_t)dy;
    _st->time->hour   = (uint8_t)hr;
    _st->time->minute = (uint8_t)mn;
    _st->time->second = (uint8_t)sc;
    snprintf(_st->time->dateStr, sizeof(_st->time->dateStr),
             "%02u/%02u/%04u", dy, mo, yr);
    snprintf(_st->time->timeStr, sizeof(_st->time->timeStr),
             "%02u:%02u:%02u", hr, mn, sc);
    appendEvent("NTP synced from BLE");
    redirectTo("/settings?msg=ok:Time synced from BLE");
}

void WifiApServer::hSetNtp() {
    if (!checkAuth()) return;
    _prefs.putBool("ntpEnable", _server->arg("en") == "1");
    appendEvent("NTP settings updated");
    redirectTo("/settings?msg=ok:NTP settings saved");
}

void WifiApServer::hSetMem() {
    if (!checkAuth()) return;
    uint32_t pct = (uint32_t)_server->arg("pct").toInt();
    if (pct < 10) pct = 10;
    if (pct > 90) pct = 90;
    _prefs.putUInt("memRollover",   pct);
    _prefs.putBool("memRolloverEn", _server->arg("en") == "1");
    appendEvent("Memory settings updated");
    redirectTo("/settings?msg=ok:Memory settings saved");
}

/* ── BLE CONNECT / DISCONNECT ────────────────────────────── */
void WifiApServer::hBleConnect() {
    if (!checkAuth()) return;
    String mac = _server->arg("mac");
    if (mac.length() == 17 && _st->bleTargetMac && _st->bleConnPending) {
        strncpy(_st->bleTargetMac, mac.c_str(), 17);
        _st->bleTargetMac[17] = '\0';
        *_st->bleConnPending = true;
        char ev[48]; snprintf(ev, sizeof(ev), "BLE connect request: %s", mac.c_str());
        appendEvent(ev);
    }
    redirectTo("/live");
}

void WifiApServer::hBleDisconnect() {
    if (!checkAuth()) return;
    if (_st->bleTargetMac)  { _st->bleTargetMac[0] = '\0'; }
    if (_st->bleConnDev)    { _st->bleConnDev->connected = false; _st->bleConnDev->charCount = 0; }
    if (_st->bleConnPending){ *_st->bleConnPending = false; }
    if (_st->bleDisconnectRequest) { *_st->bleDisconnectRequest = true; }
    appendEvent("BLE disconnected");
    redirectTo("/live");
}

void WifiApServer::hBleScan() {
    if (!checkAuth()) return;
    if (_st->bleScanRequest) *_st->bleScanRequest = true;
    /* Show "scanning" splash with auto-redirect */
    _server->setContentLength(CONTENT_LENGTH_UNKNOWN);
    _server->send(200, "text/html", "");
    _server->sendContent(F("<!DOCTYPE html><html><head><meta charset='UTF-8'>"
        "<meta http-equiv='refresh' content='4;url=/settings'>"
        "<style>body{font-family:Arial;display:flex;align-items:center;justify-content:center;"
        "min-height:100vh;background:#f5f3ff;text-align:center;color:#4c1d95}</style>"
        "</head><body><div>"
        "<h2>&#128268; Scanning for BLE devices&hellip;</h2>"
        "<p style='color:#a78bfa;margin-top:10px'>Returning to Settings in 4 s&hellip;</p>"
        "</div></body></html>"));
}

void WifiApServer::hBleForget() {
    if (!checkAuth()) return;
    Serial.println(F("[WEB] BLE forget requested"));
    _prefs.remove("bleSavedMac");
    _prefs.putString("bleSavedMac", "");
    if (_st) {
        if (_st->bleSavedMac)    { _st->bleSavedMac[0]  = '\0'; }
        if (_st->bleTargetMac)   { _st->bleTargetMac[0] = '\0'; }
        if (_st->bleConnDev)     { _st->bleConnDev->connected = false; _st->bleConnDev->charCount = 0; }
        if (_st->bleConnPending) { *_st->bleConnPending = false; }
        if (_st->bleDisconnectRequest) { *_st->bleDisconnectRequest = true; }
    }
    appendEvent("BLE forget saved device");
    _server->setContentLength(CONTENT_LENGTH_UNKNOWN);
    _server->send(200, "text/html", "");
    _server->sendContent(F("<!DOCTYPE html><html><head><meta charset='UTF-8'>"
        "<meta http-equiv='refresh' content='2;url=/settings?msg=ok:BLE%20device%20forgotten'>"
        "<style>body{font-family:Arial;display:flex;align-items:center;justify-content:center;"
        "min-height:100vh;background:#0d1117;color:#e6edf3;text-align:center}</style>"
        "</head><body><div>"
        "<h2>BLE device forgotten</h2>"
        "<p style='color:#a78bfa;margin-top:12px'>Returning to Settings...</p>"
        "</div></body></html>"));
    _server->sendContent("");
    _server->client().flush();
}
void WifiApServer::hApiBle() {
    if (!_st || !_st->bleConnDev) {
        _server->send(200, "application/json",
            F("{\"connected\":false,\"nus\":false,\"hasData\":false,\"age\":0}"));
        return;
    }
    BLEConnectedDevice& d = *_st->bleConnDev;
    bool nus      = _st->bleIsNus     && *_st->bleIsNus;
    bool hasData  = nus && d.charCount > 0;
    uint32_t age  = 0;
    if (hasData && _st->bleNusLastMs && *_st->bleNusLastMs > 0)
        age = (uint32_t)((millis() - *_st->bleNusLastMs) / 1000UL);

    String j = F("{\"connected\":");
    j += d.connected ? "true" : "false";
    j += F(",\"nus\":");     j += nus     ? "true" : "false";
    j += F(",\"hasData\":"); j += hasData ? "true" : "false";
    j += F(",\"age\":"); j += age;
    j += F(",\"mac\":\"");   j += d.mac;
    j += F("\",\"name\":\""); j += d.name;
    j += F("\",\"chars\":[");
    for (uint8_t i = 0; i < d.charCount; i++) {
        if (i > 0) j += ',';
        j += F("{\"uuid\":\"");    j += d.chars[i].uuid;
        j += F("\",\"hex\":\"");   j += d.chars[i].value;
        j += F("\",\"ascii\":\""); j += d.chars[i].ascii;
        j += F("\"}");
    }
    j += F("]}");
    _server->send(200, "application/json", j);
}

void WifiApServer::hBleRefresh() {
    if (!checkAuth()) return;
    if (_st->bleNusRefreshReq) *_st->bleNusRefreshReq = true;
    _server->send(200, "application/json", F("{\"ok\":true}"));
}

/* ── ABOUT ─────────────────────────────────────────────────── */
void WifiApServer::hAbout() {
    if (!checkAuth()) return;
    String b = F(
        "<div class='card' style='text-align:center;line-height:2'>"

        /* Institution */
        "<p style='font-size:13px;color:#6b7280;letter-spacing:.5px'>Created by</p>"
        "<p style='font-weight:700;font-size:16px;color:#4c1d95;margin-top:4px'>"
        "IDEA Laboratory @ KMUTT</p>"
        "<p style='color:#374151'>Electrical Engineering Department</p>"

        "<div style='margin:18px auto;width:60px;border-top:2px solid #3d2a6e'></div>"

        /* Project name */
        "<p style='font-size:20px;font-weight:700;color:#2563eb'>"
        "All-in-One Weather Station</p>"

        "<div style='margin:16px auto;width:40px;border-top:1px solid #e5e7eb'></div>"

        /* "By" */
        "<p style='color:#9ca3af;font-size:13px;letter-spacing:2px'>BY</p>"
        "<br>"

        /* Thai names */
        "<p style='font-size:15px'><strong>&#x0E2D;&#x0E32;&#x0E08;&#x0E32;&#x0E23;&#x0E22;&#x0E4C; &#x0E08;&#x0E31;&#x0E01;&#x0E23;&#x0E01;&#x0E24;&#x0E0A; &#x0E01;&#x0E31;&#x0E19;&#x0E17;&#x0E2D;&#x0E07;</strong></p>"
        "<p style='font-size:15px'><strong>&#x0E2D;&#x0E32;&#x0E08;&#x0E32;&#x0E23;&#x0E22;&#x0E4C; &#x0E18;&#x0E34;&#x0E23;&#x0E30;&#x0E28;&#x0E31;&#x0E01;&#x0E14;&#x0E34;&#x0E4C; &#x0E40;&#x0E2A;&#x0E20;&#x0E32;&#x0E01;&#x0E25;&#x0E48;&#x0E2D;&#x0E21;</strong></p>"
        "<p style='font-size:15px'><strong>&#x0E19;&#x0E32;&#x0E22; &#x0E01;&#x0E24;&#x0E15;&#x0E20;&#x0E32;&#x0E2A; &#x0E41;&#x0E08;&#x0E48;&#x0E21;&#x0E27;&#x0E34;&#x0E25;&#x0E31;&#x0E22;&#x0E1E;&#x0E31;&#x0E19;&#x0E18;&#x0E38;&#x0E4C;</strong></p>"
        "<br>"

        /* English names */
        "<p>Dr. Jakkrit Kunthong</p>"
        "<p>Aj. Tirasak Sapaklom</p>"
        "<p>Mr. Krittapat Jamvilaiphan</p>"

        "<div style='margin:16px auto;width:40px;border-top:1px solid #e5e7eb'></div>"

        "<p style='color:#6b7280;font-size:13px'>16 May 2026</p>"
        "<br>"
        "<span class='tag g'>Firmware v" FIRMWARE_VERSION "</span>"
        "&nbsp;"
        "<span class='tag'>Build " BUILD_DATE "</span>"
        "</div>"

        /* Reboot card */
        "<div class='card' style='text-align:center'>"
        "<h3 style='color:#dc2626;margin-bottom:12px'>&#9889; System Control</h3>"
        "<a class='btn' href='/update' style='margin-bottom:10px'>"
        "&#128295; Update Firmware (wireless)</a>"
        "<a class='btn btn-r' href='/reboot' "
        "onclick=\"return confirm('Reboot the device now?')\">"
        "&#128260; Reboot Device</a>"
        "</div>");

    sendPage("About", "about", b);
}

void WifiApServer::hReboot() {
    if (!checkAuth()) return;
    Serial.println(F("[WEB] Reboot requested"));
    appendEvent("Manual reboot via web UI");
    bool queued = false;
    if (_st && _st->rebootRequest) {
        *_st->rebootRequest = true;
        queued = true;
    }
    _server->setContentLength(CONTENT_LENGTH_UNKNOWN);
    _server->send(200, "text/html", "");
    _server->sendContent(F("<!DOCTYPE html><html><head>"
        "<meta charset='UTF-8'>"
        "<meta http-equiv='refresh' content='10;url=/login'>"
        "<style>body{font-family:Arial;display:flex;align-items:center;justify-content:center;"
        "min-height:100vh;background:#4c1d95;color:#fff;text-align:center}</style>"
        "</head><body>"
        "<div><h2>&#128260; Reboot command sent</h2>"
        "<p style='color:#93c5fd;margin-top:12px'>Reconnect to WeatherStation_AP in ~10 seconds</p>"
        "</div></body></html>"));
    _server->sendContent("");
    _server->client().flush();
    if (!queued) {
        delay(1500);
        ESP.restart();
    }
}

// ── Browser firmware update (wireless, no IDE/espota needed) ──────────────
void WifiApServer::hUpdate() {
    if (!checkAuth()) return;
    String h = F("<!DOCTYPE html><html><head><meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Firmware Update - v" FIRMWARE_VERSION "</title>"
        "<style>body{font-family:Arial;background:#0f172a;color:#e2e8f0;display:flex;"
        "align-items:center;justify-content:center;min-height:100vh;margin:0}"
        ".card{background:#1e293b;padding:28px;border-radius:14px;max-width:420px;width:90%;"
        "box-shadow:0 8px 24px rgba(0,0,0,.4)}h2{margin:0 0 6px}"
        ".sub{color:#94a3b8;font-size:13px;margin-bottom:18px}"
        "input[type=file]{width:100%;margin:10px 0;color:#e2e8f0}"
        ".btn{width:100%;padding:12px;border:0;border-radius:8px;background:#2563eb;"
        "color:#fff;font-size:15px;cursor:pointer}a{color:#60a5fa}"
        "#bar{height:8px;background:#334155;border-radius:6px;overflow:hidden;margin-top:14px;display:none}"
        "#fill{height:100%;width:0;background:#22c55e;transition:width .2s}</style></head><body>"
        "<div class='card'><h2>&#128295; Firmware Update</h2>"
        "<div class='sub'>Current: v" FIRMWARE_VERSION " &nbsp;|&nbsp; upload a new .bin</div>"
        "<form id='f' method='POST' action='/update' enctype='multipart/form-data'>"
        "<input type='file' name='firmware' accept='.bin' required>"
        "<button class='btn' type='submit'>Upload &amp; Flash</button></form>"
        "<div id='bar'><div id='fill'></div></div>"
        "<p style='margin-top:16px'><a href='/about'>&larr; Back</a></p>"
        "<script>const f=document.getElementById('f');f.onsubmit=function(e){"
        "e.preventDefault();const fd=new FormData(f);const x=new XMLHttpRequest();"
        "document.getElementById('bar').style.display='block';"
        "x.upload.onprogress=function(ev){if(ev.lengthComputable){"
        "document.getElementById('fill').style.width=(ev.loaded/ev.total*100)+'%';}};"
        "x.onload=function(){document.body.innerHTML='<div class=card><h2>'+"
        "(x.status==200?'&#9989; Updated':'&#10060; Failed')+'</h2><p class=sub>'+"
        "(x.status==200?'Rebooting&hellip; reconnect to the AP in ~15 s':x.responseText)+"
        "'</p></div>';};x.open('POST','/update');x.send(fd);};</script>"
        "</div></body></html>");
    _server->send(200, "text/html", h);
}

void WifiApServer::hUpdateUpload() {
    HTTPUpload& up = _server->upload();
    if (up.status == UPLOAD_FILE_START) {
        Serial.printf("[OTA-Web] start: %s\n", up.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
    } else if (up.status == UPLOAD_FILE_WRITE) {
        if (Update.write(up.buf, up.currentSize) != up.currentSize) Update.printError(Serial);
    } else if (up.status == UPLOAD_FILE_END) {
        if (Update.end(true)) Serial.printf("[OTA-Web] done: %u bytes\n", up.totalSize);
        else                  Update.printError(Serial);
    }
}

void WifiApServer::hUpdatePost() {
    bool ok = !Update.hasError();
    appendEvent(ok ? "Firmware updated via web upload" : "Web firmware update FAILED");
    _server->send(ok ? 200 : 500, "text/plain",
                  ok ? "OK" : "Update failed - check the .bin and retry");
    if (ok) { delay(800); ESP.restart(); }
}
