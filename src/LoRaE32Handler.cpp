#include "LoRaE32Handler.h"
#include "LoRaPacket.h"
#include <esp_task_wdt.h>

LoRaE32Handler::LoRaE32Handler()
    : _serial(nullptr), _initialized(false) {}

LoRaE32Handler::~LoRaE32Handler() {}

void LoRaE32Handler::_flush() {
    if (_serial) {
        while (_serial->available()) _serial->read();
        _serial->flush();
    }
}

bool LoRaE32Handler::init(HardwareSerial& serial) {
    _serial = &serial;
    _flush();

    /* No M0/M1/AUX pins - transparent serial mode only.
       Assume module is pre-configured and already in normal mode. */
    Serial.println(F("[LoRa] E32 serial initialised (transparent mode, no mode pins)"));
    _initialized = true;
    return _initialized;
}

bool LoRaE32Handler::send(const uint8_t* data, size_t len) {
    if (!_initialized || !_serial) {
        Serial.println(F("[LoRa] Not initialised"));
        return false;
    }

    _flush();

    size_t written = _serial->write(data, len);
    _serial->flush();

    if (written != len) {
        Serial.printf("[LoRa] Write incomplete: %u/%u\n", (unsigned)written, (unsigned)len);
        return false;
    }

    Serial.printf("[LoRa] Sent %u bytes\n", (unsigned)len);
    return true;
}

bool LoRaE32Handler::sendString(const char* str) {
    if (!str || !str[0]) return false;
    return send((const uint8_t*)str, strlen(str));
}

bool LoRaE32Handler::isReady() {
    return _initialized;
}

void LoRaE32Handler::sleep() {
    /* No M0/M1 pins - cannot enter sleep mode */
    Serial.println(F("[LoRa] sleep() ignored - no mode pins wired"));
}

void LoRaE32Handler::wakeup() {
    /* No M0/M1 pins - nothing to do */
    _flush();
}

#ifdef LORA_USE_ACK

void LoRaE32Handler::flushInput() {
    _flush();
}

int LoRaE32Handler::_readByte(unsigned long timeoutMs) {
    if (!_serial) return -1;
    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        if (_serial->available()) {
            return (int)((uint8_t)_serial->read());
        }
        delay(1);
    }
    return -1;
}

bool LoRaE32Handler::waitAck(LoRaAckPacket& ack, uint16_t expectedSeq, unsigned long timeoutMs) {
    if (!_initialized || !_serial) return false;

    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        if (_serial->available()) {
            uint8_t b = (uint8_t)_serial->read();
            if (b != LORA_MAGIC) continue;

            int typeByte = _readByte(timeoutMs - (millis() - start));
            if (typeByte < 0 || (uint8_t)typeByte != LORA_PKT_ACK) continue;

            int lo = _readByte(timeoutMs - (millis() - start));
            if (lo < 0) return false;
            int hi = _readByte(timeoutMs - (millis() - start));
            if (hi < 0) return false;

            ack.magic = LORA_MAGIC;
            ack.type  = LORA_PKT_ACK;
            ack.seq   = (uint16_t)((uint8_t)lo) | ((uint16_t)((uint8_t)hi) << 8);

            if (ack.seq == expectedSeq) {
                Serial.printf("[LoRa-ACK] Received ACK for seq=%u\n", expectedSeq);
                return true;
            }
            Serial.printf("[LoRa-ACK] Seq mismatch: got=%u expected=%u\n", ack.seq, expectedSeq);
        }
        delay(1);
    }

    Serial.printf("[LoRa-ACK] Timeout waiting for ACK seq=%u (%lu ms)\n",
                  expectedSeq, timeoutMs);
    return false;
}

#endif /* LORA_USE_ACK */