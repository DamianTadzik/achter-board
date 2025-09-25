/*
 * task_crsf_transmitter.c
 *
 *  Created on: Sep 24, 2025
 *      Author: brzan
 */
#include <crsf/crsf_port.h>
#include "task_crsf_transmitter.h"
#include "main.h"
#include "cmsis_os2.h"
#include "achter_board.h"

#include "usart.h"

//// Global dummy telemetry value
//volatile uint16_t g_dummy_millivolts_x100 = 1234; // 12.34 V demo
//
//static void pack_battery_payload(uint8_t *p) {
//    // 0x08 Battery sensor (mV*100, mA*100, mAh (24-bit), %, see BF tx_crsf.c) :contentReference[oaicite:6]{index=6}
//    uint16_t mvx100 = g_dummy_millivolts_x100;
//    uint16_t maxx100 = 0;      // current (mA*100) dummy
//    uint32_t mah = 0;          // fuel
//    uint8_t  pct = 0;          // remaining
//
//    p[0] = (mvx100 >> 8) & 0xFF; p[1] = mvx100 & 0xFF;
//    p[2] = (maxx100 >> 8) & 0xFF; p[3] = maxx100 & 0xFF;
//    p[4] = (mah >> 16) & 0xFF; p[5] = (mah >> 8) & 0xFF; p[6] = mah & 0xFF;
//    p[7] = pct;
//}

// ---- Transmitter: send telemetry every ~1s (or faster) ----
extern volatile uint32_t task_crsf_transmitter_alive;
void task_crsf_transmitter(void *argument)
{
    (void)argument;
//    uint8_t payload[CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE];

    for (;;)
    {
    	task_crsf_transmitter_alive++;
//        pack_battery_payload(payload);
//
//        (void)CRSF_TxSend(CRSF_FRAMETYPE_BATTERY_SENSOR, payload, sizeof(payload));

		// --- Send CRSF Heartbeat (0x0B) ---
		// Payload = 1 byte: Origin Device Address (use Flight Controller address 0xC8)
		// Ref: CRSF heartbeat payload spec. :contentReference[oaicite:1]{index=1}
    	if (0)
		{
			uint8_t hb_payload = CRSF_ADDRESS_FLIGHT_CONTROLLER; // 0xC8
			(void)CRSF_TxSend(CRSF_FRAMETYPE_HEARTBEAT, &hb_payload, 1);
		}

        osDelay(1000); // 1 Hz for now — you can reduce to e.g. 100-200ms if needed
    }
}
