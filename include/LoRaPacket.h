/* LoRa binary packet definitions — shared between sender and receiver.
 * All structs are packed for deterministic wire format (little-endian).
 *
 * Packet layout on-air:
 *   LoRaHeader        (6 bytes)  — every packet starts here
 *   + type-specific payload
 *
 * Total sizes:
 *   Heartbeat:  6 + 6  = 12 bytes
 *   Data:       6 + 53 = 59 bytes
 */
#ifndef LORA_PACKET_H_
#define LORA_PACKET_H_

#include <stdint.h>
#include "sensor_v2.h"

#define LORA_MAGIC          0xA5

/* Packet types */
#define LORA_PKT_HEARTBEAT  0x01
#define LORA_PKT_DATA       0x02
#define LORA_PKT_ACK        0x03

/* ── Header (6 bytes) ── */
typedef struct __attribute__((packed)) {
    uint8_t  magic;      /* LORA_MAGIC — identifies valid packet */
    uint8_t  type;       /* LORA_PKT_HEARTBEAT or LORA_PKT_DATA  */
    uint16_t seq;        /* sequence number (wraps at 65535)      */
    uint16_t vt;         /* battery voltage in mV                  */
} LoRaHeader;

/* Heartbeat payload (6 bytes) -> total 12 bytes */
typedef struct __attribute__((packed)) {
    LoRaHeader hdr;
    uint32_t   uptime;   /* seconds since boot */
    uint16_t   heap;     /* free heap bytes    */
    /* firmware version omitted to save space — receiver identifies by seq */
} LoRaHeartbeat;

/* Data payload (53 bytes) -> total 59 bytes */
typedef struct __attribute__((packed)) {
    LoRaHeader hdr;        /* 6  */
    /* timestamp */
    uint8_t    date;       /* 1  */
    uint8_t    month;      /* 1  */
    uint8_t    year;       /* 1  — last 2 digits (e.g. 26 for 2026) */
    uint8_t    hour;       /* 1  */
    uint8_t    minute;     /* 1  */
    /* sensor data */
    SensorData sensor;     /* 33 */
    /* BLE */
    uint8_t    ble_valid;  /* 1  */
    BleSensorData ble;     /* 14 - only meaningful if ble_valid == 1 */
} LoRaDataPacket;

/* ACK packet (4 bytes) — sent by RX after receiving a valid data packet */
typedef struct __attribute__((packed)) {
    uint8_t  magic;      /* LORA_MAGIC */
    uint8_t  type;       /* LORA_PKT_ACK */
    uint16_t seq;        /* sequence number being acknowledged */
} LoRaAckPacket;

#endif
