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
static void pack_battery_payload(uint8_t *p,
		float voltage_v, float current_a)
{
    // Convert
    uint16_t voltage_mv = (uint16_t)(voltage_v * 10.0f);
    uint16_t current_ma = (uint16_t)(current_a * 10.0f);
    uint32_t fuel_mah   = 0;  // set if you track mAh
    uint8_t  pct        = 0;  // percentage (optional)

    // Voltage (MSB first)
    p[0] = (voltage_mv >> 8) & 0xFF;
    p[1] = (voltage_mv >> 0) & 0xFF;

    // Current (MSB first)
    p[2] = (current_ma >> 8) & 0xFF;
    p[3] = (current_ma >> 0) & 0xFF;

    // Fuel (mAh), 24-bit
    p[4] = (fuel_mah >> 0) & 0xFF;
    p[5] = (fuel_mah >> 8) & 0xFF;
    p[6] = (fuel_mah >> 16) & 0xFF;

    // Battery percentage
    p[7] = pct;
}

// ---- Transmitter: send telemetry every ~1s (or faster) ----
extern volatile uint32_t task_crsf_transmitter_alive;
void task_crsf_transmitter(void *argument)
{
	achter_board_t* ab_ptr = achter_board_get_ptr();
//	ab_ptr->odesc.bus_voltage = 16.8;
//	ab_ptr->odesc.bus_current = 2.5;

    static uint32_t last_hb = 0;
    static uint32_t last_bat = 0;

    while (1)
    {
    	uint32_t now = osKernelGetTickCount();

        // ---- Battery telemetry every ~200 ms ----
        if ((now - last_bat) >= 200)
        {
            last_bat = now;

            uint8_t bat_payload[8]= { 0 };
            float bus_v = ab_ptr->odesc.bus_voltage;
            float bus_a = ab_ptr->odesc.bus_current;
            pack_battery_payload(bat_payload, bus_v, bus_a);

            CRSF_TxSend(CRSF_FRAMETYPE_BATTERY_SENSOR, bat_payload, sizeof(bat_payload));
        }

        // ---- Heartbeat every ~1000 ms ----
        if ((now - last_hb) >= 1000)
        {
            last_hb = now;

            uint8_t hb_payload = CRSF_ADDRESS_FLIGHT_CONTROLLER; // 0xC8
            CRSF_TxSend(CRSF_FRAMETYPE_HEARTBEAT, &hb_payload, 1);
        }

        task_crsf_transmitter_alive++;
        osDelay(200);
    }
}
