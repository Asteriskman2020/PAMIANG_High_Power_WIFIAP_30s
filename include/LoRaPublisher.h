/* LoRa data publishing - reads temp CSV, packs binary structs, sends via E32.
 * Extracted from main.cpp. */
#ifndef LORA_PUBLISHER_H_
#define LORA_PUBLISHER_H_

#include <Arduino.h>
#include "LoRaE32Handler.h"
#include "LoRaPacket.h"
#include "Memory.h"

class Preferences;

/* Context: pointers to globals in main.cpp.
 * Fill in setup() before calling begin(). */
struct LoRaPublisherCtx {
    LoRaE32Handler*  loraHandler;
    HardwareSerial*  loraSerial;      /* for re-init if needed */
    bool*            loraAvailable;
    uint16_t*        batteryVoltage;
    Memory*          memory;
    Preferences*     nvs;
    const char*      tempDataPath;    /* "/DATA_TEMP.csv" */
    uint8_t*         publishBatchSize;
};

class LoRaPublisher {
public:
    LoRaPublisher();

    /* Call once in setup() after filling LoRaPublisherCtx */
    void begin(LoRaPublisherCtx* ctx);

    /* Read temp CSV records, pack into LoRaDataPacket structs, send via E32.
     * When LORA_USE_ACK is defined: waits for ACK from RX with retries.
     * Without LORA_USE_ACK: fire-and-forget. */
    bool publishData();

    /* Send a small heartbeat struct via E32. */
    bool publishHeartbeat();

#ifdef LORA_USE_ACK
    /* Send an ACK packet for the given sequence number. */
    bool sendAck(uint16_t seq);
#endif

private:
    LoRaPublisherCtx* _ctx;
    uint16_t          _sequence;
};

#endif
