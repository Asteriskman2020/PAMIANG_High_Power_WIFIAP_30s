/* LoRa data publishing - extracted from main.cpp */
#include "LoRaPublisher.h"
#include "utilities.h"
#include "TxUtilities.h"
#include <esp_task_wdt.h>
#include <LittleFS.h>
#include <Preferences.h>

LoRaPublisher::LoRaPublisher()
    : _ctx(nullptr), _sequence(0) {}

void LoRaPublisher::begin(LoRaPublisherCtx* ctx) {
    _ctx = ctx;
}

static constexpr const char* LORA_QUEUE_NVS_NS = "lora-q";
static constexpr const char* LORA_QUEUE_HEAD_KEY = "headKey";
static constexpr const char* LORA_QUEUE_FAIL_KEY = "failCnt";

static uint32_t ramQueueHeadKey = 0;
static uint8_t ramQueueFailCount = 0;

static uint32_t makeFifoHeadKey(const DataRecord& record) {
    uint8_t miniChecksum = (uint8_t)(
        (record.data.soil_humi ^
         record.data.soil_ec ^
         record.data.CO2 ^
         (record.data.illuminance & 0xFFFF) ^
         record.data.solar) & 0x0F
    );

    return ((uint32_t)(record.year & 0x7F) << 25) |
           ((uint32_t)(record.month & 0x0F) << 21) |
           ((uint32_t)(record.date & 0x1F) << 16) |
           ((uint32_t)(record.hour & 0x1F) << 11) |
           ((uint32_t)(record.minute & 0x3F) << 5) |
           ((uint32_t)(record.ble_valid & 0x01) << 4) |
           miniChecksum;
}

static uint8_t noteFifoHeadFailedCycle(Preferences* prefs, uint32_t headKey) {
    if (prefs != nullptr && prefs->begin(LORA_QUEUE_NVS_NS, false)) {
        uint32_t storedKey = prefs->getUInt(LORA_QUEUE_HEAD_KEY, 0);
        uint8_t failCount = (storedKey == headKey)
                            ? prefs->getUChar(LORA_QUEUE_FAIL_KEY, 0)
                            : 0;
        if (failCount < 255) failCount++;
        prefs->putUInt(LORA_QUEUE_HEAD_KEY, headKey);
        prefs->putUChar(LORA_QUEUE_FAIL_KEY, failCount);
        prefs->end();
        return failCount;
    }

    if (ramQueueHeadKey != headKey) {
        ramQueueHeadKey = headKey;
        ramQueueFailCount = 0;
    }
    if (ramQueueFailCount < 255) ramQueueFailCount++;
    return ramQueueFailCount;
}

static void clearFifoHeadFailedCycles(Preferences* prefs) {
    ramQueueHeadKey = 0;
    ramQueueFailCount = 0;

    if (prefs == nullptr || !prefs->begin(LORA_QUEUE_NVS_NS, false)) return;

    uint32_t storedKey = prefs->getUInt(LORA_QUEUE_HEAD_KEY, 0);
    uint8_t failCount = prefs->getUChar(LORA_QUEUE_FAIL_KEY, 0);
    if (storedKey != 0 || failCount != 0) {
        prefs->remove(LORA_QUEUE_HEAD_KEY);
        prefs->remove(LORA_QUEUE_FAIL_KEY);
    }
    prefs->end();
}

static void handleFifoHeadFailure(LoRaPublisherCtx* ctx, const DataRecord& record) {
    if (ctx == nullptr || ctx->memory == nullptr || ctx->tempDataPath == nullptr) return;

    uint32_t headKey = makeFifoHeadKey(record);
    uint8_t failCycles = noteFifoHeadFailedCycle(ctx->nvs, headKey);
    Serial.printf("[LoRa] FIFO head failed cycle %u/%u; row remains queued.\n",
                  failCycles, LORA_FIFO_HEAD_MAX_FAILED_CYCLES);

    if (failCycles < LORA_FIFO_HEAD_MAX_FAILED_CYCLES) return;

    Serial.println(F("[LoRa] FIFO head failed too many cycles; moving it to queue tail."));
    if (ctx->memory->moveFirstDataLineToEnd(ctx->tempDataPath)) {
        clearFifoHeadFailedCycles(ctx->nvs);
        Serial.println(F("[LoRa] FIFO queue unblocked; next cycle will try the next row."));
    } else {
        Serial.println(F("[LoRa] FIFO rotation failed; row kept at queue head."));
    }
}

#ifdef LORA_USE_ACK

/* -- Send ACK packet via LoRa -- */
bool LoRaPublisher::sendAck(uint16_t seq) {
    LoRaAckPacket ack;
    ack.magic = LORA_MAGIC;
    ack.type  = LORA_PKT_ACK;
    ack.seq   = seq;

    bool ok = _ctx->loraHandler->send((const uint8_t*)&ack, sizeof(ack));
    if (ok) {
        Serial.printf("[LoRa-ACK] Sent ACK for seq=%u\n", seq);
    } else {
        Serial.printf("[LoRa-ACK] Failed to send ACK for seq=%u\n", seq);
    }
    return ok;
}

/* -- Publish data records via LoRa with ACK -- */
bool LoRaPublisher::publishData() {
    int recordCount = _ctx->memory->countDataLines(_ctx->tempDataPath);
    Serial.printf("[LoRa] %d records in temp file\n", recordCount);

    if (recordCount <= 0) {
        Serial.println(F("[LoRa] No records to publish."));
        return false;
    }
    if (!*_ctx->loraAvailable) {
        Serial.println(F("[LoRa] Re-initializing..."));
        if (_ctx->loraHandler->init(*_ctx->loraSerial)) {
            *_ctx->loraAvailable = true;
        } else {
            Serial.println(F("[LoRa] Init failed - cannot publish"));
            return false;
        }
    }

    int maxRecords = PUBLISH_MAX_RECORDS;
    int toPublish = (recordCount > maxRecords) ? maxRecords : recordCount;

    DataRecord records[PUBLISH_MAX_RECORDS];
    memset(records, 0, sizeof(records));
    if (!_ctx->memory->readDataRecords(_ctx->tempDataPath, records, toPublish)) {
        Serial.println(F("[LoRa] Failed to read temp records"));
        return false;
    }

    int published = 0;
    bool failedAtFifoHead = false;
    DataRecord failedHeadRecord;
    memset(&failedHeadRecord, 0, sizeof(failedHeadRecord));

    for (int i = 0; i < toPublish; i++) {
        if (records[i].valid != 1) continue;

        LoRaDataPacket pkt;
        memset(&pkt, 0, sizeof(pkt));

        pkt.hdr.magic = LORA_MAGIC;
        pkt.hdr.type  = LORA_PKT_DATA;
        pkt.hdr.seq   = _sequence;
        pkt.hdr.vt    = *_ctx->batteryVoltage;

        pkt.date   = records[i].date;
        pkt.month  = records[i].month;
        pkt.year   = records[i].year;
        pkt.hour   = records[i].hour;
        pkt.minute = records[i].minute;

        pkt.sensor = records[i].data;

        pkt.ble_valid = records[i].ble_valid;
        if (pkt.ble_valid)
            pkt.ble = records[i].ble;

        bool acked = false;
        for (uint8_t retry = 0; retry <= LORA_ACK_MAX_RETRIES; retry++) {
            if (retry > 0) {
                Serial.printf("[LoRa] Retry %u/%u for record %d\n",
                              retry, LORA_ACK_MAX_RETRIES, i + 1);
            }

            Serial.printf("[LoRa] Sending FIFO record %d/%d seq=%u (%u bytes)\n",
                          i + 1, toPublish, pkt.hdr.seq, sizeof(pkt));

            bool ok = _ctx->loraHandler->send((const uint8_t*)&pkt, sizeof(pkt));
            if (!ok) {
                Serial.printf("[LoRa] Send failed at record %d.\n", i + 1);
                delay(LORA_ACK_TX_SETTLE);
                continue;
            }

            delay(LORA_ACK_TX_SETTLE);

            LoRaAckPacket ack;
            acked = _ctx->loraHandler->waitAck(ack, pkt.hdr.seq, LORA_ACK_TIMEOUT_MS);
            if (acked) break;

            Serial.printf("[LoRa] No ACK for seq=%u (attempt %u/%u)\n",
                          pkt.hdr.seq, retry + 1, LORA_ACK_MAX_RETRIES + 1);
            esp_task_wdt_reset();
        }

        if (!acked) {
            Serial.printf("[LoRa] Gave up on record %d after %u attempts. Stopping.\n",
                          i + 1, LORA_ACK_MAX_RETRIES + 1);
            if (published == 0 && i == 0) {
                failedAtFifoHead = true;
                failedHeadRecord = records[i];
            }
            break;
        }

        _sequence++;
        published++;
        esp_task_wdt_reset();
        delay(500);
    }

    if (published > 0) {
        Serial.printf("[LoRa] Removing %d FIFO line(s) from CSV...\n", published);
        Serial.flush();
        _ctx->memory->removeFirstDataLines(_ctx->tempDataPath, published);
        clearFifoHeadFailedCycles(_ctx->nvs);
        Serial.printf("[LoRa] Published %d/%d record(s)\n", published, toPublish);
        Serial.flush();
    } else if (failedAtFifoHead) {
        handleFifoHeadFailure(_ctx, failedHeadRecord);
    }

    return (published > 0);
}

#else /* !LORA_USE_ACK - fire-and-forget with redundancy */

/* -- Publish data records via LoRa (no ACK, send each packet LORA_TX_REDUNDANCY times) -- */
bool LoRaPublisher::publishData() {
    int recordCount = _ctx->memory->countDataLines(_ctx->tempDataPath);
    Serial.printf("[LoRa] %d records in temp file\n", recordCount);

    if (recordCount <= 0) {
        Serial.println(F("[LoRa] No records to publish."));
        return false;
    }
    if (!*_ctx->loraAvailable) {
        Serial.println(F("[LoRa] Re-initializing..."));
        if (_ctx->loraHandler->init(*_ctx->loraSerial)) {
            *_ctx->loraAvailable = true;
        } else {
            Serial.println(F("[LoRa] Init failed - cannot publish"));
            return false;
        }
    }

    int maxRecords = PUBLISH_MAX_RECORDS;
    int toPublish = (recordCount > maxRecords) ? maxRecords : recordCount;

    DataRecord records[PUBLISH_MAX_RECORDS];
    memset(records, 0, sizeof(records));
    if (!_ctx->memory->readDataRecords(_ctx->tempDataPath, records, toPublish)) {
        Serial.println(F("[LoRa] Failed to read temp records"));
        return false;
    }

    int published = 0;
    bool failedAtFifoHead = false;
    DataRecord failedHeadRecord;
    memset(&failedHeadRecord, 0, sizeof(failedHeadRecord));

    for (int i = 0; i < toPublish; i++) {
        if (records[i].valid != 1) continue;

        LoRaDataPacket pkt;
        memset(&pkt, 0, sizeof(pkt));

        pkt.hdr.magic = LORA_MAGIC;
        pkt.hdr.type  = LORA_PKT_DATA;
        pkt.hdr.seq   = _sequence++;
        pkt.hdr.vt    = *_ctx->batteryVoltage;

        pkt.date   = records[i].date;
        pkt.month  = records[i].month;
        pkt.year   = records[i].year;
        pkt.hour   = records[i].hour;
        pkt.minute = records[i].minute;

        pkt.sensor = records[i].data;

        pkt.ble_valid = records[i].ble_valid;
        if (pkt.ble_valid)
            pkt.ble = records[i].ble;

        bool sendOk = false;
        for (uint8_t tx = 0; tx < LORA_TX_REDUNDANCY; tx++) {
            Serial.printf("[LoRa] Sending FIFO record %d/%d tx=%u/%u (%u bytes)\n",
                          i + 1, toPublish, tx + 1, LORA_TX_REDUNDANCY, sizeof(pkt));

            bool ok = _ctx->loraHandler->send((const uint8_t*)&pkt, sizeof(pkt));
            if (ok) {
                sendOk = true;
            } else {
                Serial.printf("[LoRa] Send failed at record %d tx %u.\n", i + 1, tx + 1);
            }
            esp_task_wdt_reset();
            if (tx < LORA_TX_REDUNDANCY - 1) {
                delay(LORA_TX_REDUND_DELAY);
            }
        }

        if (!sendOk) {
            Serial.printf("[LoRa] All %u TX attempts failed for record %d. Stopping.\n",
                          LORA_TX_REDUNDANCY, i + 1);
            if (published == 0 && i == 0) {
                failedAtFifoHead = true;
                failedHeadRecord = records[i];
            }
            break;
        }
        published++;
        delay(500);
    }

    if (published > 0) {
        Serial.printf("[LoRa] Removing %d FIFO line(s) from CSV...\n", published);
        Serial.flush();
        _ctx->memory->removeFirstDataLines(_ctx->tempDataPath, published);
        clearFifoHeadFailedCycles(_ctx->nvs);
        Serial.printf("[LoRa] Published %d/%d record(s) x%u each\n",
                      published, toPublish, LORA_TX_REDUNDANCY);
        Serial.flush();
    } else if (failedAtFifoHead) {
        handleFifoHeadFailure(_ctx, failedHeadRecord);
    }

    return (published > 0);
}

#endif /* LORA_USE_ACK */

/* -- Publish heartbeat via LoRa -- */
bool LoRaPublisher::publishHeartbeat() {
    if (!*_ctx->loraAvailable) return false;

    LoRaHeartbeat pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.hdr.magic  = LORA_MAGIC;
    pkt.hdr.type   = LORA_PKT_HEARTBEAT;
    pkt.hdr.seq    = _sequence++;
    pkt.hdr.vt     = *_ctx->batteryVoltage;
    pkt.uptime     = millis() / 1000;
    pkt.heap       = (uint16_t)(ESP.getFreeHeap() & 0xFFFF);

    Serial.printf("[LoRa] Heartbeat (%u bytes)\n", sizeof(pkt));
    bool ok = _ctx->loraHandler->send((const uint8_t*)&pkt, sizeof(pkt));
    if (ok) Serial.println(F("[LoRa] Heartbeat sent"));
    else    Serial.println(F("[LoRa] Heartbeat failed"));
    return ok;
}
