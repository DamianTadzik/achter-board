/*
 * task_can_tx.c
 *
 *  Created on: Oct 11, 2025
 *      Author: brzan
 */
#include "task_can_tx.h"
#include "main.h"
#include "cmsis_os2.h"
#include "achter_board.h"

#include "can-not/can_not.h"
#include "can-messages-mini-celka/src/cmmc.h"
#include "string.h"


void send_cmmc_radio_control(achter_board_t* ab_ptr)
{
	struct cmmc_radio_control_t tmp = {
			.throttle = cmmc_radio_control_throttle_encode(ab_ptr->from_radio.throttle),
			.steering = cmmc_radio_control_steering_encode(ab_ptr->from_radio.steering),
			.front_pitch = cmmc_radio_control_front_pitch_encode(ab_ptr->from_radio.front_pitch_sp),
			.front_roll = cmmc_radio_control_front_roll_encode(ab_ptr->from_radio.front_roll_sp),
			.rear_pitch = cmmc_radio_control_rear_pitch_encode(ab_ptr->from_radio.rear_pitch_sp),
			.arm_switch = cmmc_radio_control_arm_switch_encode(ab_ptr->from_radio.arm_switch),
			.mode_switch = cmmc_radio_control_mode_switch_encode(ab_ptr->from_radio.mode_switch),
	};
	cant_generic_struct_t msg = {
			.msg_dlc	= CMMC_RADIO_CONTROL_LENGTH,
			.msg_id		= CMMC_RADIO_CONTROL_FRAME_ID,
			.msg_payload = { 0U },
	};
	cmmc_radio_control_pack(msg.msg_payload, &tmp, msg.msg_dlc);
	cant_transmit(&msg);
}

static void send_cmmc_distance_achter_feedback(achter_board_t* ab_ptr)
{
	struct cmmc_distance_achter_feedback_t tmp = {
			.range_mm_l = cmmc_distance_achter_feedback_range_mm_l_encode(ab_ptr->left_tof.range_mm),
			.signal_rate_mcps_l = ab_ptr->left_tof.signalRate_mcps,	// do not use encode here
			.error_status_l = cmmc_distance_achter_feedback_error_status_l_encode(ab_ptr->left_tof.errorStatus),
			.range_mm_r = cmmc_distance_achter_feedback_range_mm_r_encode(ab_ptr->right_tof.range_mm),
			.signal_rate_mcps_r = ab_ptr->right_tof.signalRate_mcps, // do not use encode here
			.error_status_r = cmmc_distance_achter_feedback_error_status_r_encode(ab_ptr->right_tof.errorStatus),
	};
	cant_generic_struct_t msg = {
			.msg_dlc	= CMMC_DISTANCE_ACHTER_FEEDBACK_LENGTH,
			.msg_id		= CMMC_DISTANCE_ACHTER_FEEDBACK_FRAME_ID,
			.msg_payload = { 0U },
	};
	cmmc_distance_achter_feedback_pack(msg.msg_payload, &tmp, msg.msg_dlc);
	cant_transmit(&msg);
}

static void send_cmmc_actuator_steering_feedback(achter_board_t* ab_ptr)
{
	struct cmmc_actuator_steering_feedback_t tmp = {
			.current = cmmc_actuator_steering_feedback_current_encode(ab_ptr->steering_servo_power.current),
			.voltage = cmmc_actuator_steering_feedback_voltage_encode(ab_ptr->steering_servo_power.voltage),
			.power = cmmc_actuator_steering_feedback_power_encode(ab_ptr->steering_servo_power.power),
			.setpoint_us = cmmc_actuator_steering_feedback_setpoint_us_encode(ab_ptr->steering_servo_feedback.setpoint_us),
			.position_raw = cmmc_actuator_steering_feedback_position_raw_encode(ab_ptr->steering_servo_feedback.raw_adc),
			.mode = cmmc_actuator_steering_feedback_mode_encode(ab_ptr->steering_servo_feedback.mode),
	};
	cant_generic_struct_t msg = {
			.msg_dlc	= CMMC_ACTUATOR_STEERING_FEEDBACK_LENGTH,
			.msg_id		= CMMC_ACTUATOR_STEERING_FEEDBACK_FRAME_ID,
			.msg_payload = { 0U },
	};
	cmmc_actuator_steering_feedback_pack(msg.msg_payload, &tmp, msg.msg_dlc);
	cant_transmit(&msg);
}

static void send_cmmc_actuator_rear_foil_feedback(achter_board_t* ab_ptr)
{
	struct cmmc_actuator_rear_foil_feedback_t tmp = {
			.current = cmmc_actuator_rear_foil_feedback_current_encode(ab_ptr->rear_servo_power.current),
			.voltage = cmmc_actuator_rear_foil_feedback_voltage_encode(ab_ptr->rear_servo_power.voltage),
			.power = cmmc_actuator_rear_foil_feedback_power_encode(ab_ptr->rear_servo_power.power),
			.setpoint_us = cmmc_actuator_rear_foil_feedback_setpoint_us_encode(ab_ptr->rear_servo_feedback.setpoint_us),
			.position_raw = cmmc_actuator_rear_foil_feedback_position_raw_encode(ab_ptr->rear_servo_feedback.raw_adc),
			.mode = cmmc_actuator_rear_foil_feedback_mode_encode(ab_ptr->rear_servo_feedback.mode),
	};
	cant_generic_struct_t msg = {
			.msg_dlc	= CMMC_ACTUATOR_REAR_FOIL_FEEDBACK_LENGTH,
			.msg_id		= CMMC_ACTUATOR_REAR_FOIL_FEEDBACK_FRAME_ID,
			.msg_payload = { 0U },
	};
	cmmc_actuator_rear_foil_feedback_pack(msg.msg_payload, &tmp, msg.msg_dlc);
	cant_transmit(&msg);
}

static uint32_t distance_achter_feedback_message_period = 1;
static uint32_t actuator_steering_feedback_message_period = 1;
static uint32_t actuator_rear_foil_feedback_message_period = 1;

extern volatile uint32_t task_can_tx_alive;
void task_can_tx(void *argument)
{
	achter_board_t* ab_ptr = achter_board_get_ptr();

	uint32_t distance_achter_feedback_message_counter = distance_achter_feedback_message_period;
	uint32_t actuator_steering_feedback_message_counter = actuator_steering_feedback_message_period;
	uint32_t actuator_rear_foil_feedback_message_counter = actuator_rear_foil_feedback_message_period;

	while (1)
	{
		if (distance_achter_feedback_message_counter)
		{
			distance_achter_feedback_message_counter--;
		}
		else
		{
			distance_achter_feedback_message_counter = distance_achter_feedback_message_period;
			send_cmmc_distance_achter_feedback(ab_ptr);
		}


		if (actuator_steering_feedback_message_counter)
		{
			actuator_steering_feedback_message_counter--;
		}
		else
		{
			actuator_steering_feedback_message_counter = actuator_steering_feedback_message_period;
			send_cmmc_actuator_steering_feedback(ab_ptr);
		}


		if (actuator_rear_foil_feedback_message_counter)
		{
			actuator_rear_foil_feedback_message_counter--;
		}
		else
		{
			actuator_rear_foil_feedback_message_counter = actuator_rear_foil_feedback_message_period;
			send_cmmc_actuator_rear_foil_feedback(ab_ptr);
		}

		task_can_tx_alive++;
		osDelay(1000);
	}
}
