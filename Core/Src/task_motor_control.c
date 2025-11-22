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

static inline float map_f(float x,
                         float in_min,  float in_max,
                         float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min)
           + out_min;
}


#define ODRV_NODE_ID            0x00    // <axis>.config.can.node_id (0..0x3F) - your ODrive's node ID.
#define TASK_PERIOD_MS 			50
static uint8_t previous = 0, current = 0;
volatile uint8_t mode = AXIS_STATE_UNDEFINED;
extern volatile uint32_t task_motor_control_alive;
void task_motor_control(void *argument)
{
	achter_board_t* ab_ptr = achter_board_get_ptr();

	while (1)
	{
		current = ab_ptr->from_radio.arm_switch;
		if (current != previous)
		{
			// nastapila zmiana na disarmed
			if (current == 0)
			{
				mode = AXIS_STATE_UNDEFINED;
				odrv_send_set_axis_state(ODRV_NODE_ID, AXIS_STATE_IDLE);
			}
			// nastapila zmiana na armed
			if (current == 1 || current == 2)
			{
			    // Clear errors, feed watchdog with 0 sp, run startup sequence,
			    odrv_send_clear_errors(ODRV_NODE_ID);
			    odrv_send_set_input_vel(ODRV_NODE_ID, 0.0f, 0.0f);
			    mode = AXIS_STATE_STARTUP_SEQUENCE;
			    odrv_send_set_axis_state(ODRV_NODE_ID, AXIS_STATE_STARTUP_SEQUENCE);

			    // Poll until controller reports idle state. Keep feeding watchdog
			    do {
			    	buzz();
			    	osDelay(200);
			    	no_buzz();
			        odrv_send_set_input_vel(ODRV_NODE_ID, 0.0f, 0.0f);
			    } while (ab_ptr->odesc.axis_current_state != 1);
			    mode = AXIS_STATE_IDLE;
			}
		}
		previous = current;


		/* 1) Read throttle, clamp and scale down the reverse throttle */
		volatile int16_t thr = ab_ptr->from_radio.throttle;   // -1000..1000 (permile)
		if (thr > 1000) thr = 1000;
		if (thr < 0 ) thr = map_f(thr, -1000, 0, -300, 0);


		/* 5) Mode supervision with hysteresis */
#define THROTTLE_DEADBAND_PERMILE   25  // +/- permile deadband around zero
#define HYSTERESIS_MS_TO_CLOSED_LOOP_CONTROL	200     // must persist past threshold this long
#define HYSTERESIS_MS_TO_IDLE              		500    // must persist past threshold this long

		static uint32_t hysteresis_ms = 0;

		if (mode == AXIS_STATE_IDLE && abs(thr) >= THROTTLE_DEADBAND_PERMILE)
		{
			hysteresis_ms += TASK_PERIOD_MS;
			if (hysteresis_ms >= HYSTERESIS_MS_TO_CLOSED_LOOP_CONTROL)
			{
				mode = AXIS_STATE_CLOSED_LOOP_CONTROL;
				odrv_send_set_axis_state(ODRV_NODE_ID, AXIS_STATE_CLOSED_LOOP_CONTROL);
			}
		}
		else if (mode == AXIS_STATE_CLOSED_LOOP_CONTROL && abs(thr) < THROTTLE_DEADBAND_PERMILE/3)
		{
			hysteresis_ms += TASK_PERIOD_MS;
			if (hysteresis_ms >= HYSTERESIS_MS_TO_IDLE)
			{
				mode = AXIS_STATE_IDLE;
				odrv_send_set_axis_state(ODRV_NODE_ID, AXIS_STATE_IDLE);
			}
		}


		/* 2) Apply deadband */
		if (abs(thr) < THROTTLE_DEADBAND_PERMILE) thr = 0;


		/* 3) Map to turns/s */
#define VEL_MAX_TURNS_S         50.0f   // max speed at 1000 permile throttle [turns/s] (example)
		float vel_sp = (float)thr * (VEL_MAX_TURNS_S / 1000.0f);


		/* 4) Software rate limiter aka ramp */
		#define INITIAL_RISE_RATE      0.1f
		#define INITIAL_RATE_UP_TO     2.0f

		#define FORWARD_RISE_RATE      0.5f
		#define FORWARD_FALL_RATE      2.0f

		#define BACKWARD_FALL_RATE     0.5f
		#define BACKWARD_RISE_RATE     2.0f

		#define HIGH_SPEED_THRESHOLD   15.0f
		#define HIGH_SPEED_RISE_RATE   1.5f
		#define HIGH_SPEED_FALL_RATE   2.0f

		static float limited_vel_sp = 0.0f;

		float diff = vel_sp - limited_vel_sp;
		float abs_prev = fabsf(limited_vel_sp);
		float abs_req  = fabsf(vel_sp);

		float rise_rate;
		float fall_rate;

		/* ---------- Stage selection ---------- */

		/* Stage 1: TRUE soft start (only when starting to accelerate away from zero) */
		if (abs_prev < INITIAL_RATE_UP_TO && abs_req > abs_prev)
		{
		    rise_rate = INITIAL_RISE_RATE;
		    fall_rate = INITIAL_RISE_RATE;   // soft start only, not soft stop
		}
		/* Stage 3: High-speed region */
		else if (abs_prev > HIGH_SPEED_THRESHOLD)
		{
		    rise_rate = HIGH_SPEED_RISE_RATE;
		    fall_rate = HIGH_SPEED_FALL_RATE;
		}
		/* Stage 2: Normal region */
		else if (limited_vel_sp >= 0.0f)
		{
		    rise_rate = FORWARD_RISE_RATE;
		    fall_rate = FORWARD_FALL_RATE;
		}
		else
		{
		    rise_rate = BACKWARD_RISE_RATE;
		    fall_rate = BACKWARD_FALL_RATE;
		}

		/* ---------- Apply ramp ---------- */
		if (diff > rise_rate)
		    limited_vel_sp += rise_rate;
		else if (diff < -fall_rate)
		    limited_vel_sp -= fall_rate;
		else
		    limited_vel_sp = vel_sp;




		/* 6) Transmit vel setpoint */
		odrv_send_set_input_vel(ODRV_NODE_ID, limited_vel_sp, 0.0f);


		task_motor_control_alive++;
		osDelay(TASK_PERIOD_MS);
	}
}
