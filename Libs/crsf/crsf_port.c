/*
 * crsf_port.c
 *
 *  Created on: Sep 24, 2025
 *      Author: brzan
 */

#include <crsf/crsf_port.h>
#include <string.h>

// ======= Internal state =======
static UART_HandleTypeDef *s_uart = NULL;
static TIM_HandleTypeDef  *s_tim  = NULL;
static volatile uint8_t    s_rxByte;

static osThreadId_t s_notifyThread = NULL;

static volatile uint16_t s_channels[CRSF_MAX_CHANNEL];
static volatile uint32_t s_lastRcFrameUs = 0;

static volatile CRSF_LinkStats_t s_link = {0};
static volatile uint32_t s_lastByteWindowStartUs = 0;
static uint8_t s_framePos = 0;
static crsfFrame_t s_frame;

// ======= Utilities =======
// CRC-8 DVB-S2 as in Betaflight (poly 0xD5), same algorithm. :contentReference[oaicite:3]{index=3}
static inline uint8_t crc8_dvb_s2(uint8_t crc, uint8_t next_byte) {
    crc ^= next_byte;
    for (int i = 0; i < 8; ++i) {
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
    return crc;
}

// --- TIM1-based µs timebase with overflow counting ---
static volatile uint32_t s_timOverflows = 0;

// Build a stable 32-bit micros() using (overflows << 16) + CNT.
// Works with PSC=71 (1 MHz tick) and ARR=9999 (10 ms period) or any ARR;
static inline uint32_t micros(void) {
    const uint32_t arrp1 = 10000; // e.g., 10000
    uint32_t ovf1 = s_timOverflows;
    uint16_t cnt  = __HAL_TIM_GET_COUNTER(s_tim);
    uint32_t ovf2 = s_timOverflows;

    // If overflow happened during read and CNT is still near the start, bind to new overflow
    if (ovf2 != ovf1 && cnt < (arrp1 >> 1)) {
        ovf1 = ovf2;
    }
    // 1 tick = 1 µs at PSC=71 (1 MHz). Period = arrp1 µs.
    return ovf1 * arrp1 + cnt;
}


static uint8_t compute_frame_crc(const crsfFrame_t *fr) {
    uint8_t crc = crc8_dvb_s2(0, fr->frame.type);
    const int payLen = fr->frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC; // excludes type+CRC
    for (int i = 0; i < payLen; ++i) crc = crc8_dvb_s2(crc, fr->frame.payload[i]);
    return crc;
}

#ifdef CRSF_DEBUG
static volatile uint32_t g_frame_pos_reset         = 0;
static volatile uint32_t g_frame_good         = 0;
static volatile uint32_t g_fullFramesSeen = 0;
static volatile uint32_t g_crcOk          = 0;
#endif

// ======= Parser =======
static void parser_on_byte(uint8_t b) {
    uint32_t now = micros();

    if ((now - s_lastByteWindowStartUs) > CRSF_TIME_NEEDED_PER_FRAME_US) {
        // New frame window
        s_framePos = 0;
#ifdef CRSF_DEBUG
        g_frame_pos_reset++;
#endif
    }
    if (s_framePos == 0) {
#ifdef CRSF_DEBUG
    	g_frame_good++;
#endif
        s_lastByteWindowStartUs = now;
    }

    // Expected full frame length once we know frameLength
    int fullLen = (s_framePos < 3)
        ? 5
        : (int)(s_frame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH);
    if (fullLen > CRSF_FRAME_SIZE_MAX) fullLen = CRSF_FRAME_SIZE_MAX; // clamp defensively

    if (s_framePos < fullLen) {
        s_frame.bytes[s_framePos++] = b;

        if (s_framePos >= fullLen) {
            s_framePos = 0;
#ifdef CRSF_DEBUG
            g_fullFramesSeen++;
#endif
            uint8_t crc = compute_frame_crc(&s_frame);
            uint8_t rx_crc = s_frame.bytes[fullLen - 1];

            if (crc == rx_crc) {
#ifdef CRSF_DEBUG
            	g_crcOk++;
#endif
                switch (s_frame.frame.type) {
                case CRSF_FRAMETYPE_RC_CHANNELS_PACKED: {
                    if (s_frame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
                        const crsfPayloadRcChannelsPacked_t *rc =
                            (const crsfPayloadRcChannelsPacked_t *)&s_frame.frame.payload[0];

                        // store raw (unscaled) 11-bit values, as requested
                        s_channels[0]  = rc->chan0;  s_channels[1]  = rc->chan1;
                        s_channels[2]  = rc->chan2;  s_channels[3]  = rc->chan3;
                        s_channels[4]  = rc->chan4;  s_channels[5]  = rc->chan5;
                        s_channels[6]  = rc->chan6;  s_channels[7]  = rc->chan7;
                        s_channels[8]  = rc->chan8;  s_channels[9]  = rc->chan9;
                        s_channels[10] = rc->chan10; s_channels[11] = rc->chan11;
                        s_channels[12] = rc->chan12; s_channels[13] = rc->chan13;
                        s_channels[14] = rc->chan14; s_channels[15] = rc->chan15;

                        s_lastRcFrameUs = now;
                        if (s_notifyThread) { osThreadFlagsSet(s_notifyThread, CRSF_TFLAG_RX_FRAME); }
                    }
                } break;

                case CRSF_FRAMETYPE_LINK_STATISTICS: {
                	const uint8_t *p = s_frame.frame.payload;
                    // Payload layout:
                    // [0] RSSI 1 (uplink, negated dBm)
                    // [1] RSSI 2 (uplink, negated dBm)
                    // [2] LQ (uplink, %)
                    // [3] SNR (uplink, signed dB)
                    // [4] active antenna (0 or 1)
                    // [5] RF mode (50/150/250/500 Hz)
                    // [6] TX power (0=25mW, 1=100mW, etc.)
                    // [7] RSSI (downlink, negated dBm)
                    // [8] LQ (downlink, %)
                    // [9] SNR (downlink, signed dB)

                    // ---- Uplink stats ----
                    s_link.uplink_rssi_1_dbm  = -(int8_t)p[0];
                    s_link.uplink_rssi_2_dbm  = -(int8_t)p[1];
                    s_link.uplink_lq_percent  = p[2];
                    s_link.uplink_snr_db      = (int8_t)p[3];

                    // p[4] = active antenna
                    // p[5] = rf mode
                    // p[6] = tx power index

                    // ---- Downlink stats ----
                    s_link.downlink_rssi_dbm  = -(int8_t)p[7];
                    s_link.downlink_lq_percent= p[8];
                    s_link.downlink_snr_db    = (int8_t)p[9];

                    s_link.lastUpdateUs = now;
                    if (s_notifyThread) { osThreadFlagsSet(s_notifyThread, CRSF_TFLAG_LINK_STATS); }
                } break;

                default: break;
                }
            }
        }
    }
}

// ======= Public API =======
bool CRSF_Init(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim_us) {
    s_uart = huart;
    s_tim  = htim_us;

    HAL_TIM_Base_Start(s_tim);

    // ---- Timebase: ensure TIM is running and update IRQ is enabled ----
    // Do NOT change PSC/ARR here (PWM uses this timer). We only start base + enable update IT.
    __HAL_TIM_ENABLE_IT(s_tim, TIM_IT_UPDATE); // count overflows in CRSF_TIM_UpdateIRQ()
    __HAL_TIM_ENABLE(s_tim);                   // make sure the counter runs even if all PWM channels are stopped

    // ---- UART RX: arm byte-by-byte IT (UART already configured by CubeMX) ----
    if (HAL_UART_Receive_IT(s_uart, (uint8_t *)&s_rxByte, 1) != HAL_OK) {
        return false;
    }

    return true;
}

void CRSF_SetNotifyThread(osThreadId_t thread_id) {
    s_notifyThread = thread_id;
}

bool CRSF_GetChannels(uint16_t out[CRSF_MAX_CHANNEL], uint32_t *age_us) {
    if (!out) return false;
    uint32_t now = micros();
    for (int i = 0; i < CRSF_MAX_CHANNEL; ++i) out[i] = s_channels[i];
    if (age_us) *age_us = now - s_lastRcFrameUs;
    return (s_lastRcFrameUs != 0);
}

bool CRSF_GetLinkStats(CRSF_LinkStats_t *out) {
    if (!out) return false;
    *out = s_link;
    return (s_link.lastUpdateUs != 0);
}

bool CRSF_IsLinkLost(void) {
    uint32_t now = micros();
    return (now - s_lastRcFrameUs) > CRSF_LINK_LOSS_TIMEOUT_US;
}

bool CRSF_TxSend(uint8_t type, const uint8_t *payload, uint8_t payload_len) {
    if (payload_len > CRSF_PAYLOAD_SIZE_MAX) return false;

    uint8_t buf[CRSF_FRAME_SIZE_MAX];
    int pos = 0;
//    buf[pos++] = CRSF_ADDRESS_CRSF_RECEIVER;   wrong?  // Device address (to RX)
    buf[pos++] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    buf[pos++] = (uint8_t)(payload_len + CRSF_FRAME_LENGTH_TYPE_CRC);
    buf[pos++] = type;
    memcpy(&buf[pos], payload, payload_len); pos += payload_len;

    // CRC over <Type> + <Payload>
    uint8_t crc = 0;
    crc = crc8_dvb_s2(crc, type);                  // same CRC as RX side. :contentReference[oaicite:5]{index=5}
    for (int i = 0; i < payload_len; ++i) crc = crc8_dvb_s2(crc, payload[i]);
    buf[pos++] = crc;

    if (HAL_UART_Transmit(s_uart, buf, pos, 5) != HAL_OK) return false;
    return true;
}

// ======= HAL hooks =======
void CRSF_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == s_uart) {
        parser_on_byte(s_rxByte);
        HAL_UART_Receive_IT(s_uart, (uint8_t *)&s_rxByte, 1); // re-arm
    }
}
void CRSF_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == s_uart) {
        __HAL_UART_CLEAR_OREFLAG(s_uart);
        __HAL_UART_CLEAR_NEFLAG(s_uart);
        __HAL_UART_CLEAR_FEFLAG(s_uart);
        // Re-arm reception
        (void)HAL_UART_Receive_IT(s_uart, (uint8_t *)&s_rxByte, 1);
    }
}
void CRSF_TIM_UpdateIRQ(TIM_HandleTypeDef *htim) {
    if (htim == s_tim) {
        s_timOverflows++;
    }
}
