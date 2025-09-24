/*
 * crsf_defs.h
 *
 *  Created on: Sep 24, 2025
 *      Author: brzan
 */

#ifndef CRSF_CRSF_DEFS_H_
#define CRSF_CRSF_DEFS_H_

#include <stdint.h>

// ---- CRSF basics (subset sufficient for RX channels + link stats + simple TX) ----
#define CRSF_BAUDRATE                      420000
#define CRSF_FRAME_SIZE_MAX                64
#define CRSF_PAYLOAD_SIZE_MAX              60
#define CRSF_FRAME_LENGTH_ADDRESS          1
#define CRSF_FRAME_LENGTH_FRAMELENGTH      1
#define CRSF_FRAME_LENGTH_TYPE_CRC         2   // Type + CRC
#define CRSF_FRAME_LENGTH_EXT_TYPE_CRC     4   // Type + Dest + Origin + CRC

// Device addresses (subset)
#define CRSF_ADDRESS_FLIGHT_CONTROLLER     0xC8
#define CRSF_ADDRESS_CRSF_RECEIVER         0xEA
#define CRSF_ADDRESS_RADIO_TRANSMITTER     0xEA // used in some replies; not critical here

// Frame types (subset we handle)
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED  0x16
#define CRSF_FRAMETYPE_LINK_STATISTICS     0x14
#define CRSF_FRAMETYPE_BATTERY_SENSOR      0x08
#define CRSF_FRAMETYPE_HEARTBEAT           0x0B

// Payload sizes (used for TX battery example)
#define CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE  (2/*mV x100*/ + 2/*mA x100*/ + 3/*mAh*/ + 1/*%*/)

// Limits / timing
#define CRSF_MAX_CHANNEL                   16
#define CRSF_TIME_NEEDED_PER_FRAME_US      1750   // conservative (Betaflight uses similar) :contentReference[oaicite:1]{index=1}
#define CRSF_LINK_LOSS_TIMEOUT_US          300000 // 300 ms -> failsafe if no RC frame

// Packed RC channels payload (16 * 11-bit = 22 bytes) — same layout as Betaflight. :contentReference[oaicite:2]{index=2}
typedef struct __attribute__((__packed__)) {
    unsigned int chan0  : 11; unsigned int chan1  : 11;
    unsigned int chan2  : 11; unsigned int chan3  : 11;
    unsigned int chan4  : 11; unsigned int chan5  : 11;
    unsigned int chan6  : 11; unsigned int chan7  : 11;
    unsigned int chan8  : 11; unsigned int chan9  : 11;
    unsigned int chan10 : 11; unsigned int chan11 : 11;
    unsigned int chan12 : 11; unsigned int chan13 : 11;
    unsigned int chan14 : 11; unsigned int chan15 : 11;
} crsfPayloadRcChannelsPacked_t;

// A lean CRSF frame container compatible with how we parse
typedef struct crsfFrameDef_s {
    uint8_t deviceAddress;
    uint8_t frameLength;  // length starting at <Type> up to and incl. CRC
    uint8_t type;
    uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + 1]; // +1 for CRC at end of payload
} crsfFrameDef_t;

typedef union {
    uint8_t       bytes[CRSF_FRAME_SIZE_MAX];
    crsfFrameDef_t frame;
} crsfFrame_t;

#endif /* CRSF_CRSF_DEFS_H_ */
