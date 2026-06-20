/* TX Board utility functions — extracted from main.cpp */
#include "TxUtilities.h"
#include "utilities.h"
#include <esp_task_wdt.h>
#include <LittleFS.h>

static uint64_t timeKey(const timeStruct& t) {
    return ((uint64_t)t.year * 10000000000ULL) +
           ((uint64_t)t.month * 100000000ULL) +
           ((uint64_t)t.date * 1000000ULL) +
           ((uint64_t)t.hour * 10000ULL) +
           ((uint64_t)t.minute * 100ULL) +
           t.second;
}

static bool parseCsvTimestamp(const String& line, timeStruct* outTime) {
    if (outTime == nullptr) return false;

    int fc = line.indexOf(',');
    if (fc < 0) return false;
    int sc = line.indexOf(',', fc + 1);
    if (sc < 0) return false;

    String ds = line.substring(0, fc);
    String ts = line.substring(fc + 1, sc);
    ds.trim();
    ts.trim();

    int d1 = ds.indexOf('/'), d2 = ds.lastIndexOf('/');
    int t1 = ts.indexOf(':'), t2 = ts.lastIndexOf(':');
    if (t1 < 0 || t2 < 0 || t1 == t2) {
        t1 = ts.indexOf('/');
        t2 = ts.lastIndexOf('/');
    }
    if (d1 < 0 || d2 < 0 || d1 == d2) return false;
    if (t1 < 0 || t2 < 0 || t1 == t2) return false;

    uint8_t date = (uint8_t)ds.substring(0, d1).toInt();
    uint8_t month = (uint8_t)ds.substring(d1 + 1, d2).toInt();
    uint16_t year = (uint16_t)ds.substring(d2 + 1).toInt();
    uint8_t hour = (uint8_t)ts.substring(0, t1).toInt();
    uint8_t minute = (uint8_t)ts.substring(t1 + 1, t2).toInt();
    uint8_t second = (uint8_t)ts.substring(t2 + 1).toInt();

    if (year < 2024 || month < 1 || month > 12 || date < 1 || date > 31) return false;
    if (hour > 23 || minute > 59 || second > 59) return false;

    outTime->date = date;
    outTime->month = month;
    outTime->year = year;
    outTime->hour = hour;
    outTime->minute = minute;
    outTime->second = second;
    snprintf(outTime->dateStr, sizeof(outTime->dateStr), "%02u/%02u/%04u",
             outTime->date, outTime->month, outTime->year);
    snprintf(outTime->timeStr, sizeof(outTime->timeStr), "%02u:%02u:%02u",
             outTime->hour, outTime->minute, outTime->second);
    return true;
}

static bool readLastTimeFromCsvPath(const char* path, timeStruct* outTime) {
    if (path == nullptr || !LittleFS.exists(path)) return false;

    File f = LittleFS.open(path, "r");
    if (!f) return false;

    f.readStringUntil('\n');
    String last = "";
    uint16_t n = 0;
    while (f.available()) {
        String line = f.readStringUntil('\n');
        n++;
        if ((n % 16) == 0) {
            esp_task_wdt_reset();
            delay(0);
        }
        line.trim();
        if (line.length()) last = line;
    }
    f.close();

    return parseCsvTimestamp(last, outTime);
}

static bool isDailyCsvPath(const String& path) {
    String name = path;
    if (!name.startsWith("/")) name = "/" + name;
    if (name.length() != 15) return false;
    if (name.charAt(3) != '-' || name.charAt(6) != '-') return false;
    if (!name.endsWith(".csv")) return false;

    for (int i = 1; i < 11; i++) {
        if (i == 3 || i == 6) continue;
        if (!isDigit(name.charAt(i))) return false;
    }
    return true;
}

TxUtils::TxUtils() : _ctx(nullptr) {}

void TxUtils::begin(TxUtilsCtx* ctx) {
    _ctx = ctx;
}

/* ── WDT ── */
void TxUtils::feedWDT() {
    esp_task_wdt_reset();
}

/* ── Weather ── */
void TxUtils::clearWeatherData() {
    auto& s = _ctx->modbusSensor->currentSensor;
    s.windSpeed = s.windDir_Deg = s.air_humidity = 0;
    s.air_temperature = s.CO2 = s.pressure = 0;
    s.illuminance = s.rainfall = s.solar = 0;
}

/* ── Time ── */
void TxUtils::incrementTime(timeStruct* t, uint16_t addMinutes) {
    if (t->date == 0 || t->month == 0 || t->month > 12 || t->year == 0) {
        t->date = 1; t->month = 1; t->year = 2024; t->hour = 0; t->minute = 0; t->second = 0;
    }
    int total = t->hour * 60 + t->minute + addMinutes;
    int carry = 0;
    while (total >= 1440) { total -= 1440; carry++; }
    t->hour = (uint8_t)(total / 60);
    t->minute = (uint8_t)(total % 60);
    t->second = 0;
    static const uint8_t dim[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    t->date += carry;
    uint8_t mi = (t->month >= 1 && t->month <= 12) ? (t->month - 1) : 0;
    while (t->date > dim[mi]) {
        t->date -= dim[mi]; t->month++;
        mi = (t->month >= 1 && t->month <= 12) ? (t->month - 1) : 0;
        if (t->month > 12) { t->month = 1; t->year++; mi = 0; }
    }
    snprintf(t->dateStr, sizeof(t->dateStr), "%02u/%02u/%04u", t->date, t->month, t->year);
    snprintf(t->timeStr, sizeof(t->timeStr), "%02u:%02u:%02u", t->hour, t->minute, t->second);
}

/* ── File helpers ── */
bool TxUtils::readLastTimeFromBackup(timeStruct* outTime) {
    if (outTime == nullptr) return false;

    bool found = false;
    uint64_t bestKey = 0;
    timeStruct candidate = {};

    _ctx->nvs->begin("ws-cfg", true);
    String csvPath = _ctx->nvs->getString("lastDailyCsv", _ctx->dailyCsv);
    _ctx->nvs->end();

    auto considerPath = [&](const char* path) {
        if (readLastTimeFromCsvPath(path, &candidate)) {
            uint64_t key = timeKey(candidate);
            if (!found || key > bestKey) {
                *outTime = candidate;
                bestKey = key;
                found = true;
            }
        }
    };

    considerPath(csvPath.c_str());
    considerPath(_ctx->dailyCsv);
    considerPath("/DATA.csv");

    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while (file) {
        String fname = String(file.name());
        if (!fname.startsWith("/")) fname = "/" + fname;
        file.close();

        if (isDailyCsvPath(fname)) {
            considerPath(fname.c_str());
        }

        file = root.openNextFile();
    }
    root.close();

    return found;
}

String TxUtils::getLastDataLine(const char* fileName) {
    if (!LittleFS.exists(fileName)) return "";
    File f = LittleFS.open(fileName, "r"); if (!f) return "";
    f.readStringUntil('\n');
    String last = ""; uint16_t n = 0;
    while (f.available()) {
        String line = f.readStringUntil('\n'); n++;
        if ((n % 16) == 0) { feedWDT(); delay(0); }
        line.trim(); if (line.length()) last = line;
    }
    f.close(); return last;
}

void TxUtils::updateDailyCsv() {
    if (_ctx->currentTime->year == 0) return;
    snprintf(_ctx->dailyCsv, 24, "/%02u-%02u-%04u.csv",
             _ctx->currentTime->date, _ctx->currentTime->month, _ctx->currentTime->year);
    _ctx->nvs->begin("ws-cfg", false);
    _ctx->nvs->putString("lastDailyCsv", _ctx->dailyCsv);
    _ctx->nvs->end();
    if (!LittleFS.exists(_ctx->dailyCsv)) {
        _ctx->memory->write(LittleFS, _ctx->dailyCsv,
            "Date,Time,"
            "Soil_Humidity,Soil_Temperature,EC,PH,N,P,K,"
            "WindSpeed,WindDirection,Air_Humidity,Air_Temperature,"
            "CO2,Pressure,Illuminance,Rainfall,Solar,"
            "BLE_Temp,BLE_Humi,BLE_TMP117,BLE_Rain,BLE_Leaf,BLE_PAR,BLE_Soil\r\n");
    } else {
        File df = LittleFS.open(_ctx->dailyCsv, "r");
        String hdr = df.readStringUntil('\n');
        df.close();
        if (hdr.indexOf("BLE_Temp") < 0 || hdr.indexOf("BLE_DeltaT") >= 0) {
            LittleFS.remove(_ctx->dailyCsv);
            _ctx->memory->write(LittleFS, _ctx->dailyCsv,
                "Date,Time,"
                "Soil_Humidity,Soil_Temperature,EC,PH,N,P,K,"
                "WindSpeed,WindDirection,Air_Humidity,Air_Temperature,"
                "CO2,Pressure,Illuminance,Rainfall,Solar,"
                "BLE_Temp,BLE_Humi,BLE_TMP117,BLE_Rain,BLE_Leaf,BLE_PAR,BLE_Soil\r\n");
            Serial.println(F("[FS] Daily CSV recreated with BLE columns"));
        }
    }
}

/* ── NVS runtime config ── */
void TxUtils::loadRuntimeConfig(uint8_t* soilId, uint8_t* weathId,
                                 uint8_t* batchSize, uint8_t* pubMode) {
    _ctx->nvs->begin("ws-cfg", true);
    *soilId   = _ctx->nvs->getUChar("soilSlaveId",  SOIL_SLAVE_ID);
    *weathId  = _ctx->nvs->getUChar("weathSlaveId", WEATHER_SLAVE_ID);
    *batchSize = _ctx->nvs->getUChar("pubBatchSize", PUBLISH_BATCH_SIZE);
    *pubMode  = _ctx->nvs->getUChar("pubMode",      0);
    _ctx->nvs->end();
    if (*batchSize < 1)  *batchSize = 1;
    if (*batchSize > 60) *batchSize = 60;
    if (*pubMode > 1)    *pubMode = 0;
}

/* ── TPL5110 done pin ── */
void TxUtils::pulseDoneTPL5110() {
    digitalWrite(PIN_DONE, LOW);
    delay(50);
    digitalWrite(PIN_DONE, HIGH);
    delay(500);
    digitalWrite(PIN_DONE, LOW);
    delay(50);
}
