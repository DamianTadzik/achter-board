/*
 * crsf_port.h
 *
 *  Created on: Sep 24, 2025
 *      Author: brzan
 */

#ifndef CRSF_CRSF_PORT_H_
#define CRSF_CRSF_PORT_H_

#include <crsf/crsf_defs.h>
#include "stm32f1xx_hal.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <stdint.h>

// ---- User-facing API ----

// Initialize CRSF. Assumes UART was already configured in CubeMX (420k 8N1).
// Starts the provided TIM (TIM1 in your case) as a 1 MHz timebase with overflow counting,
// and arms UART Receive IT for byte-by-byte parsing.
bool CRSF_Init(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim_us);

// Latest raw (unscaled) channel value [0..15], thread-safe snapshot access
bool CRSF_GetChannels(uint16_t out[CRSF_MAX_CHANNEL], uint32_t *age_us);

// Link statistics
typedef struct {
    int8_t  uplink_rssi_1_dbm;
    int8_t  uplink_rssi_2_dbm;
    uint8_t uplink_lq_percent;
    int8_t  uplink_snr_db;

    int8_t  downlink_rssi_dbm;
    uint8_t downlink_lq_percent;
    int8_t  downlink_snr_db;

    uint32_t lastUpdateUs;
} CRSF_LinkStats_t;

bool CRSF_GetLinkStats(CRSF_LinkStats_t *out);

// Failsafe: true if no RC frame within CRSF_LINK_LOSS_TIMEOUT_US
bool CRSF_IsLinkLost(void);

// Telemetry TX — write already-assembled CRSF payload (<Type> + payload, we append CRC & header)
bool CRSF_TxSend(uint8_t type, const uint8_t *payload, uint8_t payload_len);

// ---- Hooks to be called from HAL callbacks ----
// -- From HAL_UART_RxCpltCallback --
void CRSF_UART_RxCpltCallback(UART_HandleTypeDef *huart);
// -- From HAL_UART_ErrorCallback --
void CRSF_UART_ErrorCallback(UART_HandleTypeDef *huart);
// Call this from HAL_TIM_PeriodElapsedCallback when htim == your CRSF timebase (TIM1)
void CRSF_TIM_UpdateIRQ(TIM_HandleTypeDef *htim);

// ---- OS objects (created internally) you might wait on ----
#define CRSF_TFLAG_RX_FRAME   (1u << 0)   // set when fresh RC channels parsed
#define CRSF_TFLAG_LINK_STATS (1u << 1)   // set when link stats parsed

// Register the thread to be notified (call once from your receiver task)
void CRSF_SetNotifyThread(osThreadId_t thread_id);


#endif /* CRSF_CRSF_PORT_H_ */
