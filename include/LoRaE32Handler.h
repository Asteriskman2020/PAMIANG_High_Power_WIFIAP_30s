#ifndef LORA_E32_HANDLER_H_
#define LORA_E32_HANDLER_H_

#include "utilities.h"
#include "sensor_v2.h"
#include "LoRaPacket.h"

class LoRaE32Handler {
public:
    LoRaE32Handler();
    ~LoRaE32Handler();

    bool init(HardwareSerial& serial);
    bool send(const uint8_t* data, size_t len);
    bool sendString(const char* str);

    bool isReady();
    void sleep();   /* no-op without M0/M1 pins */
    void wakeup();  /* no-op without M0/M1 pins */

#ifdef LORA_USE_ACK
    void flushInput();
    bool waitAck(LoRaAckPacket& ack, uint16_t expectedSeq, unsigned long timeoutMs);
#endif

private:
    HardwareSerial* _serial;
    bool     _initialized;

    void     _flush();
#ifdef LORA_USE_ACK
    int      _readByte(unsigned long timeoutMs);
#endif
};

#endif