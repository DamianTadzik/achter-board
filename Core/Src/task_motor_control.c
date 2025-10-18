/*
 * task_motor_control.c
 *
 *  Created on: Sep 25, 2025
 *      Author: brzan
 */
#include "task_motor_control.h"
#include "main.h"
#include "cmsis_os2.h"
#include "achter_board.h"

#include "can-not/can_not.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ======= USER TUNABLES =======
#define TASK_PERIOD_MS 			50
#define ODRV_NODE_ID            0x00    // <axis>.config.can.node_id (0..0x3F) - your ODrive's node ID.
#define VEL_MAX_TURNS_S         76.0f   // max speed at 1000 permile throttle [turns/s] (example)
#define THROTTLE_DEADBAND_PERMILE   45    // +/- permile deadband around zero
#define HYSTERESIS_MS_TO_CLOSED_LOOP_CONTROL	500     // must persist past threshold this long
#define HYSTERESIS_MS_TO_IDLE              		800    // must persist past threshold this long
static inline uint16_t odrv_can_id(uint8_t node_id, uint8_t cmd_id)
{
    // Pack into standard 11-bit identifier
    return (uint16_t)(((node_id & 0x3F) << 5) | (cmd_id & 0x1F));
}

static inline void put_f32_le(uint8_t *dst, float v)
{
    uint32_t u;
    memcpy(&u, &v, sizeof(u)); // strict-alias safe enough for embedded
    dst[0] = (uint8_t)(u & 0xFF);
    dst[1] = (uint8_t)((u >> 8) & 0xFF);
    dst[2] = (uint8_t)((u >> 16) & 0xFF);
    dst[3] = (uint8_t)((u >> 24) & 0xFF);
}

static void odrv_send_set_axis_state(uint8_t node_id, uint32_t axis_state)
{
    cant_generic_struct_t m = {0};
    m.msg_id  = odrv_can_id(node_id, CMD_SET_AXIS_STATE);
    m.msg_dlc = 4; // 32-bit state
    // Little-endian u32
    m.msg_payload[0] = (uint8_t)(axis_state & 0xFF);
    m.msg_payload[1] = (uint8_t)((axis_state >> 8) & 0xFF);
    m.msg_payload[2] = (uint8_t)((axis_state >> 16) & 0xFF);
    m.msg_payload[3] = (uint8_t)((axis_state >> 24) & 0xFF);
    cant_transmit(&m);
}

static void odrv_send_set_input_vel(uint8_t node_id,
									float vel_turns_s,
									float torque_ff_nm)
{
    cant_generic_struct_t m = {0};
    m.msg_id  = odrv_can_id(node_id, CMD_SET_INPUT_VEL);
    m.msg_dlc = 8; // 4B velocity, 4B torque_ff (floats, LE)
    put_f32_le(&m.msg_payload[0], vel_turns_s);
    put_f32_le(&m.msg_payload[4], torque_ff_nm);
    cant_transmit(&m);
}

static void odrv_send_clear_errors(uint8_t node_id)
{
	cant_generic_struct_t m = {0};
	m.msg_id  = odrv_can_id(node_id, CMD_CLEAR_ERRORS);
	m.msg_dlc = 0;
	cant_transmit(&m);
}


uint8_t previous = 0, current = 0;

volatile uint8_t node_id = ODRV_NODE_ID;
volatile uint8_t mode = AXIS_STATE_UNDEFINED;
volatile float vel_sp = 0.0f;

extern volatile uint32_t task_motor_control_alive;
void task_motor_control(void *argument)
{
	achter_board_t* ab_ptr = achter_board_get_ptr();

	while (1)
	{
		task_motor_control_alive++;

		current = ab_ptr->from_radio.arm_switch;
		if (current != previous)
		{
			// nastapila zmiana na disarmed
			if (current == 0)
			{
				mode = AXIS_STATE_UNDEFINED;
				odrv_send_set_axis_state(node_id, AXIS_STATE_IDLE);
			}
			// nastapila zmiana na armed
			if (current == 1 || current == 2)
			{
			    // Clear errors, feed watchdog with 0 sp, run startup sequence,
			    odrv_send_clear_errors(node_id);
			    odrv_send_set_input_vel(node_id, 0.0f, 0.0f);
			    mode = AXIS_STATE_STARTUP_SEQUENCE;
			    odrv_send_set_axis_state(node_id, AXIS_STATE_STARTUP_SEQUENCE);

			    // Poll until controller reports idle state. Keep feeding watchdog
			    do {
			    	buzz();
			    	osDelay(200);
			    	no_buzz();
			        odrv_send_set_input_vel(node_id, 0.0f, 0.0f);
			    } while (ab_ptr->odesc.axis_current_state != 1);
			    mode = AXIS_STATE_IDLE;
			}
		}


		/* 1) Read throttle and clamp */
		int16_t thr = ab_ptr->from_radio.throttle;   // -1000..1000 (permile)
		if (thr > 1000) thr = 1000;
		if (thr < -400) thr = -400;

		/* 2) Apply deadband */
		if (abs(thr) < THROTTLE_DEADBAND_PERMILE) thr = 0;

		/* 3) Map to turns/s */
		vel_sp = (float)thr * (VEL_MAX_TURNS_S / 1000.0f);

		/* Mode supervision */
		static uint32_t hysteresis_ms = 0;

		if (mode == AXIS_STATE_IDLE && abs(thr) >= THROTTLE_DEADBAND_PERMILE)
		{
			hysteresis_ms += TASK_PERIOD_MS;
			if (hysteresis_ms >= HYSTERESIS_MS_TO_CLOSED_LOOP_CONTROL)
			{
				mode = AXIS_STATE_CLOSED_LOOP_CONTROL;
				odrv_send_set_axis_state(node_id, AXIS_STATE_CLOSED_LOOP_CONTROL);
			}
		}
		else if (mode == AXIS_STATE_CLOSED_LOOP_CONTROL && abs(thr) < THROTTLE_DEADBAND_PERMILE/3)
		{
			hysteresis_ms += TASK_PERIOD_MS;
			if (hysteresis_ms >= HYSTERESIS_MS_TO_IDLE)
			{
				mode = AXIS_STATE_IDLE;
				odrv_send_set_axis_state(node_id, AXIS_STATE_IDLE);
			}
		}

		if ((ab_ptr->from_radio.arm_switch == 1 || ab_ptr->from_radio.arm_switch == 2) && mode == AXIS_STATE_CLOSED_LOOP_CONTROL)
		{
			odrv_send_set_input_vel(node_id, vel_sp, 0.0f);
		}
		else
		{
			odrv_send_set_input_vel(node_id, 0.0f, 0.0f);
		}

		previous = current;

		osDelay(TASK_PERIOD_MS);
	}
}
