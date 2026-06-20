/* PAMIANG RX Board v0.3.0 (LoRa Receiver + BLE Relay)
 *
 * Architecture:
 *   E32-433T30S (LoRa UART) ─[UART1]─> ESP32-C3 (parse LoRaDataPacket)
 *                                              │
 *                                              ├─> LittleFS CSV (local backup)
 *                                              └─> BLE NUS ─> LILYGO SIM800L ─> MQTT
 *
 * The E32-433T30S provides LoRa UART data. After successful
 * LoRa packet reception, data is forwarded to the LILYGO TTGO SIM800L via
 * BLE Nordic UART Service (NUS) using the UARTPacket wire format.
 *
 * State machine:
 *   STATE_LORA_INIT  -> STATE_BLE_INIT -> STATE_LORA_RX
 *   STATE_LORA_RX    (collects packets for a window, up to PUBLISH_MAX_RECORDS)
 *       -> STATE_SAVE -> STATE_LORA_RX  (loop)
 *       -> STATE_FINISH  (fatal recovery, then restart RX loop)
 *
 * BLE runs asynchronously via BleNusClient state machine.
 */
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <esp_task_wdt.h>
#include <esp_sleep.h>
#include <stddef.h>

#include "utilities_rx.h"
#include "sensor_v2.h"
#include "Memory.h"
#include "LoRaE32Handler.h"
#include "LoRaPacket.h"
#include "BleNusClient.h"
#include "BlePacketTypes.h"

/* ===== STATE MACHINE ===== */
typedef enum {
    STATE_LORA_INIT,
    STATE_BLE_INIT,
    STATE_LORA_RX,
    STATE_SAVE,
    STATE_FINISH,
} systemState;

/* ===== GLOBAL OBJECTS ===== */
LoRaE32Handler  loraHandler;
Memory          internalMemory;
Preferences     nvs;
HardwareSerial  LORA_SERIAL(1);
BleNusClient    bleClient;
systemState     currentState = STATE_LORA_INIT;

/* ===== GLOBAL VARIABLES ===== */
static bool     loraAvailable  = false;
static uint16_t batteryVoltage = 0;
static unsigned long stateEntryTime = 0;
static systemState   prevState      = STATE_LORA_RX;

static char dailyCsv[24] = "/DATA.csv";
static const char* temporarydata = "/DATA_TEMP.csv";
static const char* bleQueuePath = "/BLE_QUEUE.bin";
static const char* bleQueueSwapPath = "/BLE_QUEUE_SWAP.bin";

static DataRecord  rxRecords[PUBLISH_MAX_RECORDS];
static LoRaDataPacket rxPackets[PUBLISH_MAX_RECORDS];
static uint8_t     rxRecordCount = 0;
static unsigned long rxWindowStart = 0;
static unsigned long bleRetryNextFlushMs = 0;
static constexpr unsigned long BLE_RETRY_FLUSH_OK_INTERVAL_MS = 1000UL;
static constexpr unsigned long BLE_RETRY_FLUSH_FAIL_BACKOFF_MS = 10000UL;
static constexpr uint8_t BLE_RETRY_MAX_FLUSH_PER_PASS = 3;
static constexpr uint16_t BLE_PERSIST_QUEUE_MAX_PACKETS = 256;
static uint16_t rxHeartbeatSeq = 0;
static unsigned long lastRxHeartbeatMs = 0;
static unsigned long lastLoRaPacketMs = 0;
static unsigned long lastBatteryReadMs = 0;
static bool rxHeartbeatSentOnce = false;

/* RX window: collect packets for this long before saving/publishing */
static constexpr unsigned long RX_WINDOW_MS        = 10000UL;
static constexpr unsigned long RX_POLL_INTERVAL_MS = 1000UL;
static constexpr unsigned long RX_HEARTBEAT_INTERVAL_MS = 15UL * 60UL * 1000UL;
static constexpr unsigned long RX_HEARTBEAT_FIRST_DELAY_MS = 30000UL;
static constexpr unsigned long RX_BATTERY_CACHE_MS = 5UL * 60UL * 1000UL;
static constexpr uint16_t RX_LOW_BATTERY_SHUTDOWN_MV = LOW_BATTERY_SHUTDOWN_MV;
static constexpr uint16_t RX_BATTERY_ADC_FAULT_MV = 500;
static constexpr unsigned long RX_LOW_BATTERY_RECHECK_MIN = 30UL;
static constexpr uint64_t RX_LOW_BATTERY_RECHECK_US =
    (uint64_t)RX_LOW_BATTERY_RECHECK_MIN * 60ULL * 1000000ULL;
static constexpr unsigned long RX_IDLE_DELAY_MS = 20UL;
static constexpr unsigned long LORA_PARSE_TIME_BUDGET_MS = 25UL;
static constexpr uint16_t LORA_PARSE_MAX_BYTES_PER_CALL = 512;
static constexpr uint8_t LORA_DEDUP_CACHE_SIZE = PUBLISH_MAX_RECORDS * 8;

static uint32_t recentLoRaDataHashes[LORA_DEDUP_CACHE_SIZE];
static uint8_t recentLoRaDataHashCount = 0;
static uint8_t recentLoRaDataHashNext = 0;

static void feedWDT();

/* ===== SEND ACK ===== */
static void sendLoRaAck(uint16_t seq) {
    LoRaAckPacket ack;
    ack.magic = LORA_MAGIC;
    ack.type  = LORA_PKT_ACK;
    ack.seq   = seq;

    delay(LORA_ACK_RX_SETTLE);
    LORA_SERIAL.write((const uint8_t*)&ack, sizeof(ack));
    LORA_SERIAL.flush();
    Serial.printf("[LoRa-RX] Sent ACK for seq=%u (%u bytes)\n", seq, sizeof(ack));
}

static uint32_t hashLoRaDataPayload(const LoRaDataPacket& pkt) {
    const uint8_t* payload = reinterpret_cast<const uint8_t*>(&pkt) +
                             offsetof(LoRaDataPacket, date);
    const size_t payloadLen = sizeof(LoRaDataPacket) -
                              offsetof(LoRaDataPacket, date);

    uint32_t hash = 2166136261UL;
    for (size_t i = 0; i < payloadLen; i++) {
        hash ^= payload[i];
        hash *= 16777619UL;
    }

    return hash ? hash : 1UL;
}

static bool isDuplicateLoRaDataPacket(const LoRaDataPacket& pkt) {
    uint32_t hash = hashLoRaDataPayload(pkt);
    for (uint8_t i = 0; i < recentLoRaDataHashCount; i++) {
        if (recentLoRaDataHashes[i] == hash) return true;
    }
    return false;
}

static void rememberLoRaDataPacket(const LoRaDataPacket& pkt) {
    uint32_t hash = hashLoRaDataPayload(pkt);

    for (uint8_t i = 0; i < recentLoRaDataHashCount; i++) {
        if (recentLoRaDataHashes[i] == hash) return;
    }

    recentLoRaDataHashes[recentLoRaDataHashNext] = hash;
    recentLoRaDataHashNext = (uint8_t)((recentLoRaDataHashNext + 1) %
                                       LORA_DEDUP_CACHE_SIZE);
    if (recentLoRaDataHashCount < LORA_DEDUP_CACHE_SIZE) {
        recentLoRaDataHashCount++;
    }
}

static uint16_t persistentBleQueueCount() {
    if (!LittleFS.exists(bleQueuePath)) return 0;

    File file = LittleFS.open(bleQueuePath, "r");
    if (!file) return 0;

    size_t bytes = file.size();
    file.close();
    return (uint16_t)(bytes / sizeof(LoRaDataPacket));
}

static bool repairPersistentBleQueueFile() {
    if (!LittleFS.exists(bleQueuePath)) return true;

    File source = LittleFS.open(bleQueuePath, "r");
    if (!source) return false;

    size_t size = source.size();
    size_t validBytes = (size / sizeof(LoRaDataPacket)) * sizeof(LoRaDataPacket);
    if (validBytes == size) {
        source.close();
        return true;
    }

    Serial.printf("[BLE-QUEUE] Repairing partial queue tail (%u -> %u bytes)\n",
                  (unsigned)size, (unsigned)validBytes);

    if (validBytes == 0) {
        source.close();
        return LittleFS.remove(bleQueuePath);
    }

    LittleFS.remove(bleQueueSwapPath);
    File swap = LittleFS.open(bleQueueSwapPath, FILE_WRITE);
    if (!swap) {
        source.close();
        return false;
    }

    uint8_t buffer[128];
    size_t copied = 0;
    while (copied < validBytes) {
        size_t toRead = validBytes - copied;
        if (toRead > sizeof(buffer)) toRead = sizeof(buffer);

        size_t got = source.read(buffer, toRead);
        if (got == 0) break;
        if (swap.write(buffer, got) != got) {
            source.close();
            swap.close();
            LittleFS.remove(bleQueueSwapPath);
            return false;
        }
        copied += got;
        feedWDT();
    }

    source.close();
    swap.flush();
    swap.close();

    if (copied != validBytes) {
        LittleFS.remove(bleQueueSwapPath);
        return false;
    }

    if (!LittleFS.remove(bleQueuePath)) {
        LittleFS.remove(bleQueueSwapPath);
        return false;
    }

    return LittleFS.rename(bleQueueSwapPath, bleQueuePath);
}

static uint8_t persistentBleQueueDepthForStatus() {
    uint16_t count = persistentBleQueueCount();
    return (count > 255) ? 255 : (uint8_t)count;
}

static bool readPersistentBleQueuePacket(uint16_t index, LoRaDataPacket& pkt) {
    File file = LittleFS.open(bleQueuePath, "r");
    if (!file) return false;

    size_t offset = (size_t)index * sizeof(LoRaDataPacket);
    if (file.size() < offset + sizeof(LoRaDataPacket)) {
        file.close();
        return false;
    }

    if (!file.seek(offset, SeekSet)) {
        file.close();
        return false;
    }

    size_t got = file.read(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
    file.close();
    return got == sizeof(pkt);
}

static bool persistentBleQueueContains(const LoRaDataPacket& pkt) {
    uint32_t targetHash = hashLoRaDataPayload(pkt);
    uint16_t count = persistentBleQueueCount();

    for (uint16_t i = 0; i < count; i++) {
        LoRaDataPacket queued;
        if (!readPersistentBleQueuePacket(i, queued)) return false;
        if (hashLoRaDataPayload(queued) == targetHash) return true;
        if ((i % 16) == 0) feedWDT();
    }

    return false;
}

static bool appendPersistentBleQueuePacket(const LoRaDataPacket& pkt) {
    if (!repairPersistentBleQueueFile()) {
        Serial.println(F("[BLE-QUEUE] Queue repair failed before append"));
        return false;
    }

    uint16_t count = persistentBleQueueCount();
    if (count >= BLE_PERSIST_QUEUE_MAX_PACKETS) {
        Serial.printf("[BLE-QUEUE] Persistent queue full (%u/%u), not ACKing seq=%u\n",
                      count, BLE_PERSIST_QUEUE_MAX_PACKETS, pkt.hdr.seq);
        return false;
    }

    if (internalMemory.getAvailableSpace() <
        (MIN_FREE_SPACE_BYTES + sizeof(LoRaDataPacket))) {
        Serial.printf("[BLE-QUEUE] LittleFS low space, not ACKing seq=%u\n",
                      pkt.hdr.seq);
        return false;
    }

    File file = LittleFS.open(bleQueuePath, FILE_APPEND);
    if (!file) {
        Serial.println(F("[BLE-QUEUE] Failed to open persistent queue for append"));
        return false;
    }

    size_t written = file.write(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
    file.flush();
    file.close();

    if (written != sizeof(pkt)) {
        Serial.printf("[BLE-QUEUE] Append failed for seq=%u (%u/%u bytes)\n",
                      pkt.hdr.seq, (unsigned)written, (unsigned)sizeof(pkt));
        return false;
    }

    Serial.printf("[BLE-QUEUE] Persisted seq=%u (%u queued)\n",
                  pkt.hdr.seq, (unsigned)(count + 1));
    return true;
}

static bool removeFirstPersistentBleQueuePackets(uint16_t removeCount) {
    if (removeCount == 0) return true;
    if (!LittleFS.exists(bleQueuePath)) return false;

    uint16_t count = persistentBleQueueCount();
    if (removeCount >= count) {
        bool ok = LittleFS.remove(bleQueuePath);
        Serial.printf("[BLE-QUEUE] Removed all %u queued packet(s)\n", count);
        return ok;
    }

    File source = LittleFS.open(bleQueuePath, "r");
    if (!source) return false;

    LittleFS.remove(bleQueueSwapPath);
    File swap = LittleFS.open(bleQueueSwapPath, FILE_WRITE);
    if (!swap) {
        source.close();
        Serial.println(F("[BLE-QUEUE] Failed to open queue swap file"));
        return false;
    }

    size_t offset = (size_t)removeCount * sizeof(LoRaDataPacket);
    if (!source.seek(offset, SeekSet)) {
        source.close();
        swap.close();
        LittleFS.remove(bleQueueSwapPath);
        return false;
    }

    uint8_t buffer[128];
    while (source.available()) {
        size_t got = source.read(buffer, sizeof(buffer));
        if (got == 0) break;
        if (swap.write(buffer, got) != got) {
            source.close();
            swap.close();
            LittleFS.remove(bleQueueSwapPath);
            Serial.println(F("[BLE-QUEUE] Failed while rewriting queue"));
            return false;
        }
        feedWDT();
    }

    source.close();
    swap.flush();
    swap.close();

    if (!LittleFS.remove(bleQueuePath)) {
        LittleFS.remove(bleQueueSwapPath);
        Serial.println(F("[BLE-QUEUE] Failed to remove old queue file"));
        return false;
    }

    if (!LittleFS.rename(bleQueueSwapPath, bleQueuePath)) {
        Serial.println(F("[BLE-QUEUE] Failed to rename rewritten queue file"));
        return false;
    }

    Serial.printf("[BLE-QUEUE] Removed %u delivered packet(s), %u remain\n",
                  removeCount, (unsigned)(count - removeCount));
    return true;
}

/* ===== BATTERY READ ===== */
static uint16_t batteryReadRx() {
    esp_task_wdt_reset();
    delay(1000);
    esp_task_wdt_reset();

    Serial.println(F("[BATT-RX] Reading Battery Voltage..."));

    uint32_t sumMilliVolts = 0;
    const int numSamples = 64;
    for (int i = 0; i < numSamples; i++) {
        sumMilliVolts += analogReadMilliVolts(BATT_PIN_RX);
        delay(2);
    }

    float pinVoltage = (sumMilliVolts / (float)numSamples) / 1000.0f;
    float dividerRatio = (float)(R1_RX + R2_RX) / (float)R2_RX;
    float calibrationFactor = 1.000f;
    float batteryVoltage = pinVoltage * dividerRatio * calibrationFactor;
    uint16_t batteryMilliVolts = (uint16_t)(batteryVoltage * 1000.0f);

    Serial.printf("[BATT-RX] Pin A0: %.3f V | Battery: %.3f V (%u mV)\n",
                  (double)pinVoltage, (double)batteryVoltage, batteryMilliVolts);

    return batteryMilliVolts;
}

/* ===== HELPERS ===== */
static void feedWDT() { esp_task_wdt_reset(); }

static bool timeReached(unsigned long now, unsigned long target) {
    return (long)(now - target) >= 0;
}

static uint16_t refreshBatteryVoltage(bool force = false) {
    unsigned long now = millis();
    if (force || batteryVoltage == 0 || now - lastBatteryReadMs >= RX_BATTERY_CACHE_MS) {
        batteryVoltage = batteryReadRx();
        lastBatteryReadMs = now;
    }
    return batteryVoltage;
}

static void enterLowBatterySleep(uint16_t mv) {
    Serial.printf("[BATT-RX] Low battery (%u mV < %u mV), deep sleep for %lu min\n",
                  mv, RX_LOW_BATTERY_SHUTDOWN_MV, RX_LOW_BATTERY_RECHECK_MIN);
    Serial.flush();
    bleClient.disconnect();
    delay(100);
    esp_sleep_enable_timer_wakeup(RX_LOW_BATTERY_RECHECK_US);
    esp_deep_sleep_start();
    while (true) { delay(1000); }
}

static void enforceLowBatteryCutoff(bool forceRead = false) {
    uint16_t mv = refreshBatteryVoltage(forceRead);
    if (mv < RX_BATTERY_ADC_FAULT_MV) {
        Serial.printf("[BATT-RX] Battery read %u mV looks invalid; cutoff skipped\n", mv);
        return;
    }
    if (mv < RX_LOW_BATTERY_SHUTDOWN_MV) {
        enterLowBatterySleep(mv);
    }
}

static uint16_t bleChecksum(const uint8_t* data, size_t len) {
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

static bool sendRxHeartbeatToBle() {
    if (!bleClient.isConnected()) return false;
    if (persistentBleQueueCount() > 0) return false;

    unsigned long now = millis();
    enforceLowBatteryCutoff(true);

    BleTransport_RxHeartbeatPacket hb;
    memset(&hb, 0, sizeof(hb));
    hb.header1 = BLE_PKT_HEADER1;
    hb.header2 = BLE_PKT_HEADER2;
    hb.cmdType = BLE_PKT_TYPE_RX_HEARTBEAT;
    hb.nodeID = BLE_PKT_NODE_ID;
    hb.sequence = rxHeartbeatSeq;
    hb.batteryVoltage = batteryVoltage;
    hb.uptime = now / 1000UL;
    hb.heap = (uint16_t)(ESP.getFreeHeap() & 0xFFFF);
    hb.loraAvailable = loraAvailable ? 1 : 0;
    hb.bleConnected = bleClient.isConnected() ? 1 : 0;
    hb.retryQueueDepth = persistentBleQueueDepthForStatus();
    hb.lastLoRaPacketAgeSec = lastLoRaPacketMs
        ? (uint32_t)((now - lastLoRaPacketMs) / 1000UL)
        : 0xFFFFFFFFUL;
    hb.checksum = bleChecksum(
        reinterpret_cast<const uint8_t*>(&hb),
        offsetof(BleTransport_RxHeartbeatPacket, checksum)
    );

    bool ok = bleClient.sendBytes(reinterpret_cast<const uint8_t*>(&hb), sizeof(hb));
    if (ok) {
        Serial.printf("[RX-HB] Sent seq=%u vt=%umV uptime=%lus heap=%u q=%u\n",
                      hb.sequence, hb.batteryVoltage,
                      (unsigned long)hb.uptime, hb.heap, hb.retryQueueDepth);
        rxHeartbeatSeq++;
    } else {
        Serial.println(F("[RX-HB] Send failed"));
    }
    return ok;
}

static void serviceRxHeartbeat() {
    unsigned long now = millis();
    unsigned long interval = rxHeartbeatSentOnce
        ? RX_HEARTBEAT_INTERVAL_MS
        : RX_HEARTBEAT_FIRST_DELAY_MS;

    if (now - lastRxHeartbeatMs < interval) return;
    if (!bleClient.isConnected()) return;
    if (persistentBleQueueCount() > 0) return;

    if (sendRxHeartbeatToBle()) {
        lastRxHeartbeatMs = millis();
        rxHeartbeatSentOnce = true;
    }
}

static void copyText(char* dst, size_t dstSize, const char* src) {
    if (dstSize == 0) return;
    if (!src) src = "";
    strncpy(dst, src, dstSize - 1);
    dst[dstSize - 1] = '\0';
}

static void printReceivedDataPacket(const LoRaDataPacket& pkt) {
    BleSensorData emptyBle = {0, 0, 0, 0, 0, 0, 0};
    const BleSensorData& ble = pkt.ble_valid ? pkt.ble : emptyBle;

    char dateStr[12];
    char timeStr[9];
    uint16_t fullYear = (uint16_t)pkt.year + 2000;
    snprintf(dateStr, sizeof(dateStr), "%02u/%02u/%04u",
             pkt.date, pkt.month, fullYear);
    snprintf(timeStr, sizeof(timeStr), "%02u:%02u:%02u",
             pkt.hour, pkt.minute, 0);

    Serial.println(F("[LoRa-RX] Received data (CSV format):"));
    Serial.println(F("Date,Time,Soil_Humidity,Soil_Temperature,EC,PH,N,P,K,WindSpeed,WindDirection,Air_Humidity,Air_Temperature,CO2,Pressure,Illuminance,Rainfall,Solar,BLE_Temp,BLE_Humi,BLE_TMP117,BLE_Rain,BLE_Leaf,BLE_PAR,BLE_Soil"));
    Serial.printf(
        "%s,%s,"
        "%.1f,%.1f,%u,%.1f,%u,%u,%u,"
        "%.1f,%u,%.1f,%.1f,%u,%.1f,%lu,%.1f,%u,"
        "%.1f,%.1f,%.1f,%u,%u,%u,%u\n",
        dateStr, timeStr,
        (double)pkt.sensor.soil_humi / 10.0,
        (double)pkt.sensor.soil_temp / 10.0,
        pkt.sensor.soil_ec,
        (double)pkt.sensor.soil_ph / 10.0,
        pkt.sensor.soil_N,
        pkt.sensor.soil_P,
        pkt.sensor.soil_K,
        (double)pkt.sensor.windSpeed / 10.0,
        pkt.sensor.windDir_Deg,
        (double)pkt.sensor.air_humidity / 10.0,
        (double)pkt.sensor.air_temperature / 10.0,
        pkt.sensor.CO2,
        (double)pkt.sensor.pressure / 10.0,
        (unsigned long)pkt.sensor.illuminance,
        (double)pkt.sensor.rainfall / 10.0,
        pkt.sensor.solar,
        (double)ble.ble_temp / 10.0,
        (double)ble.ble_humi / 10.0,
        (double)ble.ble_tmp117 / 10.0,
        ble.ble_rain,
        ble.ble_leaf,
        ble.ble_par,
        ble.ble_soil);
}

/* ===== LORA PACKET PARSER =====
 * Reads bytes from LoRa UART, detects packet boundaries by magic byte 0xA5,
 * then reads the full struct based on packet type.
 */
static bool rxPacketAvailable() {
    return LORA_SERIAL.available() >= 1;
}

static bool readLoRaPacket(LoRaDataPacket& out) {
    unsigned long parseStart = millis();
    uint16_t scannedBytes = 0;

    while (LORA_SERIAL.available()) {
        if (++scannedBytes > LORA_PARSE_MAX_BYTES_PER_CALL ||
            millis() - parseStart > LORA_PARSE_TIME_BUDGET_MS) {
            return false;
        }

        uint8_t b = (uint8_t)LORA_SERIAL.read();
        if (b == LORA_MAGIC) {
            unsigned long t0 = millis();
            while (!LORA_SERIAL.available() && millis() - t0 < 500) { feedWDT(); delay(1); }
            if (!LORA_SERIAL.available()) return false;

            uint8_t pktType = (uint8_t)LORA_SERIAL.read();

            if (pktType == LORA_PKT_HEARTBEAT) {
                uint8_t buf[sizeof(LoRaHeartbeat) - 2];
                size_t need = sizeof(buf);
                size_t got = 0;
                t0 = millis();
                while (got < need && millis() - t0 < 1000) {
                    if (LORA_SERIAL.available()) {
                        buf[got++] = (uint8_t)LORA_SERIAL.read();
                    } else {
                        feedWDT(); delay(1);
                    }
                }
                if (got < need) {
                    Serial.println(F("[LoRa-RX] Heartbeat incomplete"));
                    return false;
                }

                LoRaHeartbeat hb;
                hb.hdr.magic = LORA_MAGIC;
                hb.hdr.type  = LORA_PKT_HEARTBEAT;
                hb.hdr.seq   = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
                hb.hdr.vt    = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
                hb.uptime    = (uint32_t)buf[4] | ((uint32_t)buf[5] << 8)
                             | ((uint32_t)buf[6] << 16) | ((uint32_t)buf[7] << 24);
                hb.heap      = (uint16_t)buf[8] | ((uint16_t)buf[9] << 8);

                Serial.printf("[LoRa-RX] Heartbeat seq=%u vt=%umV uptime=%lus heap=%u\n",
                              hb.hdr.seq, hb.hdr.vt,
                              (unsigned long)hb.uptime, hb.heap);
                lastLoRaPacketMs = millis();

                if (bleClient.isConnected()) {
                    bool ok = bleClient.sendBytes(reinterpret_cast<const uint8_t*>(&hb),
                                                  sizeof(LoRaHeartbeat));
                    if (ok) {
                        Serial.printf("[BLE-TX] Forwarded heartbeat seq=%u (%u bytes)\n",
                                      hb.hdr.seq, sizeof(LoRaHeartbeat));
                    } else {
                        Serial.println(F("[BLE-TX] Heartbeat forward failed"));
                    }
                }

                return false;

            } else if (pktType == LORA_PKT_DATA) {
                uint8_t buf[sizeof(LoRaDataPacket)];
                buf[0] = LORA_MAGIC;
                buf[1] = LORA_PKT_DATA;

                size_t need = sizeof(LoRaDataPacket) - 2;
                size_t got = 0;
                t0 = millis();
                while (got < need && millis() - t0 < 2000) {
                    if (LORA_SERIAL.available()) {
                        buf[2 + got] = (uint8_t)LORA_SERIAL.read();
                        got++;
                    } else {
                        feedWDT(); delay(1);
                    }
                }
                if (got < need) {
                    Serial.printf("[LoRa-RX] Data packet incomplete (%u/%u)\n",
                                  (unsigned)got, (unsigned)need);
                    return false;
                }

                memcpy(&out, buf, sizeof(out));

                Serial.printf("[LoRa-RX] Data seq=%u vt=%umV "
                              "%02u/%02u/%02u %02u:%02u "
                              "soil[h=%u t=%d] weather[ws=%u ah=%u at=%d] ble=%u\n",
                              out.hdr.seq, out.hdr.vt,
                              out.date, out.month, out.year,
                              out.hour, out.minute,
                              out.sensor.soil_humi, out.sensor.soil_temp,
                              out.sensor.windSpeed, out.sensor.air_humidity,
                              out.sensor.air_temperature,
                              out.ble_valid);
                lastLoRaPacketMs = millis();

                if (isDuplicateLoRaDataPacket(out) ||
                    persistentBleQueueContains(out)) {
                    Serial.printf("[LoRa-RX] Duplicate data ignored, ACK re-sent for seq=%u\n",
                                  out.hdr.seq);
                    sendLoRaAck(out.hdr.seq);
                    return false;
                }

                if (!appendPersistentBleQueuePacket(out)) {
                    Serial.printf("[LoRa-RX] Data seq=%u not persisted, ACK withheld\n",
                                  out.hdr.seq);
                    return false;
                }

                rememberLoRaDataPacket(out);
                printReceivedDataPacket(out);
                sendLoRaAck(out.hdr.seq);
                return true;
            } else {
                Serial.printf("[LoRa-RX] Unknown packet type 0x%02X\n", pktType);
                return false;
            }
        }
    }
    return false;
}

/* ===== DATA RECORD STORAGE ===== */
static void dataRecordFromPacket(const LoRaDataPacket& pkt, DataRecord& rec) {
    memset(&rec, 0, sizeof(rec));
    rec.date   = pkt.date;
    rec.month  = pkt.month;
    rec.year   = pkt.year;
    rec.hour   = pkt.hour;
    rec.minute = pkt.minute;
    rec.data   = pkt.sensor;
    rec.ble    = pkt.ble;
    rec.ble_valid = pkt.ble_valid;
    rec.valid = 1;
}

/* ===== BLE PACKET BUILDER =====
 * Converts collected DataRecord array into UARTPacket format
 * expected by the LILYGO SIM800L NUS receiver.
 *
 * Wire format follows BlePacketTypes.h on this board and packet_types.h
 * on the LILYGO SIM800L board.
 */
static void copySensorDataToTransport(const SensorData& src, BleTransport_SensorData& dst) {
    dst.soil_humi       = src.soil_humi;
    dst.soil_temp       = src.soil_temp;
    dst.soil_ec         = src.soil_ec;
    dst.soil_ph         = src.soil_ph;
    dst.soil_N          = src.soil_N;
    dst.soil_P          = src.soil_P;
    dst.soil_K          = src.soil_K;
    dst.windSpeed       = src.windSpeed;
    dst.windDir_Deg     = src.windDir_Deg;
    dst.air_humidity    = src.air_humidity;
    dst.air_temperature = src.air_temperature;
    dst.CO2             = src.CO2;
    dst.pressure        = src.pressure;
    dst.illuminance     = src.illuminance;
    dst.rainfall        = src.rainfall;
    dst.solar           = src.solar;
}

static void copyBleDataToTransport(const BleSensorData& src, BleTransport_BleSensorData& dst) {
    dst.ble_temp   = src.ble_temp;
    dst.ble_humi   = src.ble_humi;
    dst.ble_tmp117 = src.ble_tmp117;
    dst.ble_rain   = src.ble_rain;
    dst.ble_leaf   = src.ble_leaf;
    dst.ble_par    = src.ble_par;
    dst.ble_soil   = src.ble_soil;
}

static uint16_t bleCalculateChecksum(const uint8_t* data, size_t len) {
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

static bool buildBlePacket(const DataRecord* records, uint8_t count,
                            BleTransport_UARTPacket& pkt) {
    if (count == 0 || count > BLE_PKT_MAX_RECORDS) return false;

    memset(&pkt, 0, sizeof(pkt));

    pkt.header1 = BLE_PKT_HEADER1;
    pkt.header2 = BLE_PKT_HEADER2;
    pkt.nodeID  = BLE_NODE_ID;

    BleTransport_LoRaBatchPacket& batch = pkt.packet;
    batch.header1 = BLE_PKT_HEADER1;
    batch.header2 = BLE_PKT_HEADER2;
    batch.nodeID  = BLE_NODE_ID;
    batch.date    = records[0].date;
    batch.month   = records[0].month;
    batch.year    = records[0].year;
    batch.batteryVoltage = batteryVoltage / 10;

    uint8_t validCount = 0;
    for (uint8_t i = 0; i < count && i < BLE_PKT_MAX_RECORDS; i++) {
        const DataRecord& src = records[i];
        BleTransport_DataRecord& dst = batch.records[i];

        dst.hour   = src.hour;
        dst.minute = src.minute;
        dst.valid  = BLE_PKT_RECORD_VALID;
        dst.checksum = 0;

        copySensorDataToTransport(src.data, dst.data);
        copyBleDataToTransport(src.ble, dst.ble);
        dst.ble_valid = src.ble_valid;

        uint8_t recCrc = 0;
        const uint8_t* p = reinterpret_cast<const uint8_t*>(&dst);
        for (size_t j = 0; j < offsetof(BleTransport_DataRecord, checksum); j++) {
            recCrc += p[j];
        }
        dst.checksum = recCrc;
        validCount++;
    }

    for (uint8_t i = validCount; i < BLE_PKT_MAX_RECORDS; i++) {
        memset(&batch.records[i], 0, sizeof(BleTransport_DataRecord));
    }

    batch.checksum = bleCalculateChecksum(
        reinterpret_cast<const uint8_t*>(&batch),
        offsetof(BleTransport_LoRaBatchPacket, checksum)
    );

    pkt.dataLen = sizeof(BleTransport_LoRaBatchPacket);
    pkt.checksum = bleCalculateChecksum(
        reinterpret_cast<const uint8_t*>(&pkt),
        offsetof(BleTransport_UARTPacket, checksum)
    );

    return true;
}

static bool forwardToBle(const DataRecord* records, uint8_t count) {
    if (!bleClient.isConnected()) {
        Serial.println(F("[BLE-TX] Not connected, skipping BLE forward"));
        return false;
    }

    BleTransport_UARTPacket pkt;
    if (!buildBlePacket(records, count, pkt)) {
        Serial.println(F("[BLE-TX] Failed to build packet"));
        return false;
    }

    Serial.printf("[BLE-TX] Forwarding %u records via BLE (sizeof=%u)\n",
                  count, sizeof(BleTransport_UARTPacket));
    Serial.printf("[BLE-TX] Header: %02X %02X Node:%u\n",
                  pkt.header1, pkt.header2, pkt.nodeID);

    bool ok = bleClient.sendPacket(pkt);
    if (ok) {
        Serial.printf("[BLE-TX] Sent OK (total=%lu)\n",
                      (unsigned long)bleClient.getTotalSent());
    } else {
        Serial.printf("[BLE-TX] Send FAILED (errors=%lu dropped=%lu)\n",
                      (unsigned long)bleClient.getTotalErrors(),
                      (unsigned long)bleClient.getTotalDropped());
    }
    return ok;
}

static bool forwardLoRaPacketsToBle(const LoRaDataPacket* packets, uint8_t count) {
    (void)packets;
    (void)count;

    auto sendRawPacket = [](const LoRaDataPacket& pkt, uint8_t index, uint8_t total) {
        Serial.printf("[BLE-TX] Forwarding queued LoRa packet %u/%u seq=%u (%u bytes) ble=%u\n",
                      index, total, pkt.hdr.seq,
                      (unsigned)sizeof(LoRaDataPacket), pkt.ble_valid);

        bool ok = bleClient.sendBytes(reinterpret_cast<const uint8_t*>(&pkt),
                                      sizeof(LoRaDataPacket));
        if (!ok) {
            Serial.printf("[BLE-TX] Raw LoRa send failed seq=%u (errors=%lu dropped=%lu)\n",
                          pkt.hdr.seq,
                          (unsigned long)bleClient.getTotalErrors(),
                          (unsigned long)bleClient.getTotalDropped());
        }
        feedWDT();
        delay(20);
        return ok;
    };

    if (!bleClient.isConnected()) {
        Serial.printf("[BLE-TX] Not connected, %u packet(s) remain in persistent queue\n",
                      persistentBleQueueCount());
        return false;
    }

    uint16_t queueCount = persistentBleQueueCount();
    if (queueCount == 0) return true;

    uint8_t targetCount = (queueCount < BLE_RETRY_MAX_FLUSH_PER_PASS)
        ? (uint8_t)queueCount
        : BLE_RETRY_MAX_FLUSH_PER_PASS;

    Serial.printf("[BLE-QUEUE] Flushing up to %u/%u persistent packet(s)\n",
                  targetCount, queueCount);

    uint8_t sent = 0;
    while (sent < targetCount) {
        LoRaDataPacket pkt;
        if (!readPersistentBleQueuePacket(sent, pkt)) {
            Serial.printf("[BLE-QUEUE] Failed to read queued packet index %u\n", sent);
            break;
        }

        if (!sendRawPacket(pkt, (uint8_t)(sent + 1), targetCount)) {
            break;
        }

        sent++;
    }

    if (sent > 0 && !removeFirstPersistentBleQueuePackets(sent)) {
        Serial.println(F("[BLE-QUEUE] Sent packet(s), but failed to update queue file"));
        return false;
    }

    if (sent < targetCount) {
        Serial.printf("[BLE-QUEUE] Flush stopped after %u/%u packet(s)\n",
                      sent, targetCount);
        return false;
    }

    return true;
}

/* ===== SETUP ===== */
void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    delay(100);

    esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
    esp_task_wdt_add(NULL);
    Serial.println(F("[WDT] Watchdog initialized"));
    Serial.printf("[FW] PAMIANG RX Board v%s (BLE Relay)\n", FIRMWARE_VERSION);

    pinMode(BATT_PIN_RX, INPUT);

    enforceLowBatteryCutoff(true);
    Serial.printf("[BATT] %u mV\n", batteryVoltage);

    feedWDT();
    if (!LittleFS.begin(true)) {
        Serial.println(F("[FS] LittleFS FAILED"));
    } else {
        Serial.println(F("[FS] LittleFS mounted"));
        repairPersistentBleQueueFile();
        Serial.printf("[BLE-QUEUE] Persistent pending packets: %u\n",
                      persistentBleQueueCount());
    }

    bool recreateTemp = !LittleFS.exists(temporarydata);
    if (!recreateTemp) {
        File tf = LittleFS.open(temporarydata, "r");
        String hdr = tf.readStringUntil('\n');
        tf.close();
        recreateTemp = (hdr.indexOf("BLE_Temp") < 0 || hdr.indexOf("BLE_DeltaT") >= 0);
    }

    if (recreateTemp) {
        LittleFS.remove(temporarydata);
        internalMemory.write(LittleFS, temporarydata,
            "Date,Time,"
            "Soil_Humidity,Soil_Temperature,EC,PH,N,P,K,"
            "WindSpeed,WindDirection,Air_Humidity,Air_Temperature,"
            "CO2,Pressure,Illuminance,Rainfall,Solar,"
            "BLE_Temp,BLE_Humi,BLE_TMP117,BLE_Rain,BLE_Leaf,BLE_PAR,BLE_Soil\r\n");
    }

    Serial.println(F("===== SETUP COMPLETE ====="));
}

/* ===== MAIN LOOP ===== */
void loop() {
    feedWDT();

    bleClient.loop();
    enforceLowBatteryCutoff(false);

    unsigned long now = millis();
    if (persistentBleQueueCount() > 0 &&
        bleClient.isConnected() &&
        timeReached(now, bleRetryNextFlushMs)) {
        bool flushed = forwardLoRaPacketsToBle(nullptr, 0);
        bleRetryNextFlushMs = millis() + (flushed
            ? BLE_RETRY_FLUSH_OK_INTERVAL_MS
            : BLE_RETRY_FLUSH_FAIL_BACKOFF_MS);
    }

    serviceRxHeartbeat();

    if (currentState != prevState) {
        stateEntryTime = millis(); prevState = currentState;
    }

    switch (currentState) {

        /* ── LoRa INIT ── */
        case STATE_LORA_INIT: {
            LORA_SERIAL.setRxBufferSize(1024);
            LORA_SERIAL.begin(SERIAL_LORA, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
            delay(1000);
            feedWDT();

            if (loraHandler.init(LORA_SERIAL)) {
                loraAvailable = true;
                Serial.println(F("[LoRa] E32-433T30S initialized OK"));
            } else {
                loraAvailable = false;
                Serial.println(F("[LoRa] E32 init FAILED"));
            }

            currentState = STATE_BLE_INIT;
            break;
        }

        /* ── BLE INIT ── */
        case STATE_BLE_INIT: {
            Serial.println(F("[BLE] Initializing BLE NUS client..."));
            feedWDT();

            bool bleOk = bleClient.begin(
                BLE_TARGET_DEVICE_NAME,
                BLE_NUS_SERVICE_UUID,
                BLE_NUS_RX_CHAR_UUID,
                BLE_NUS_TX_CHAR_UUID
            );

            if (bleOk) {
                Serial.println(F("[BLE] NUS client initialized"));
            } else {
                Serial.println(F("[BLE] NUS client init FAILED"));
            }

            currentState = STATE_LORA_RX;
            break;
        }

        /* ── LoRa RECEIVE (collect packets for a window) ── */
        case STATE_LORA_RX: {
            if (rxWindowStart == 0) {
                rxWindowStart = millis();
                rxRecordCount = 0;
                Serial.println(F("[LoRa-RX] Listening for packets..."));
            }

            LoRaDataPacket pkt;
            while (readLoRaPacket(pkt) && rxRecordCount < PUBLISH_MAX_RECORDS) {
                rxPackets[rxRecordCount] = pkt;
                dataRecordFromPacket(pkt, rxRecords[rxRecordCount]);
                rxRecordCount++;
                Serial.printf("[LoRa-RX] Stored record %u/%u\n",
                              rxRecordCount, PUBLISH_MAX_RECORDS);
            }

            unsigned long elapsed = millis() - rxWindowStart;

            if (rxRecordCount >= PUBLISH_MAX_RECORDS ||
                (elapsed >= RX_WINDOW_MS && rxRecordCount > 0)) {
                Serial.printf("[LoRa-RX] Window done: %u records in %lu ms\n",
                              rxRecordCount, elapsed);
                currentState = STATE_SAVE;
                break;
            }

            if (elapsed >= RX_WINDOW_MS && rxRecordCount == 0) {
                rxWindowStart = millis();
            }

            feedWDT();
            delay(RX_IDLE_DELAY_MS);
            break;
        }

        /* ── SAVE received records to temp CSV + forward via BLE ── */
        case STATE_SAVE: {
            for (uint8_t i = 0; i < rxRecordCount; i++) {
                DataRecord& rec = rxRecords[i];
                timeStruct ts = {};
                ts.date   = rec.date;
                ts.month  = rec.month;
                ts.year   = (uint16_t)rec.year + 2000;
                ts.hour   = rec.hour;
                ts.minute = rec.minute;
                ts.second = 0;
                snprintf(ts.dateStr, sizeof(ts.dateStr), "%02u/%02u/%04u",
                         ts.date, ts.month, ts.year);
                snprintf(ts.timeStr, sizeof(ts.timeStr), "%02u:%02u:%02u",
                         ts.hour, ts.minute, ts.second);

                snprintf(dailyCsv, sizeof(dailyCsv), "/%02u-%02u-%04u.csv",
                         ts.date, ts.month, ts.year);

                internalMemory.saveData(dailyCsv, &ts, &rec.data,
                                        rec.ble_valid ? &rec.ble : nullptr);
                internalMemory.saveData(temporarydata, &ts, &rec.data,
                                        rec.ble_valid ? &rec.ble : nullptr);
            }

            Serial.printf("[SAVE] Saved %u records to CSV\n", rxRecordCount);

            if (rxRecordCount > 0) {
                forwardLoRaPacketsToBle(rxPackets, rxRecordCount);
            }

            rxWindowStart = 0;
            rxRecordCount = 0;

            currentState = STATE_LORA_RX;
            break;
        }

        /* ── FINISH ── */
        case STATE_FINISH: {
            bleClient.disconnect();
            Serial.println(F("[RECOVERY] Finish state reached, restarting always-on RX loop"));
            Serial.flush();
            rxWindowStart = 0;
            rxRecordCount = 0;
            delay(1000);
            currentState = STATE_LORA_INIT;
            break;
        }

        default: currentState = STATE_FINISH; break;
    }
}
