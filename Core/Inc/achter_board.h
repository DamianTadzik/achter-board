/*
 * achter_board.h
 *
 *  Created on: Sep 1, 2025
 *      Author: brzan
 */

#ifndef INC_FORE_BOARD_H_
#define INC_FORE_BOARD_H_

#include "main.h"

typedef enum {
	DISARMED,
	ARMED_STEERING_PROPULSION,
	ARMED_ALL,
} arm_switch_t;

typedef enum {
	To,
	Be,
	Done,
} mode_switch_t;

/* All values in following structure as like in the can should be normalized to permile */
typedef struct {
	int16_t throttle;
	int16_t steering;

	int16_t front_pitch_sp;
	int16_t front_roll_sp;
	int16_t rear_pitch_sp;

//	int16_t free_knob;

	arm_switch_t arm_switch;
	mode_switch_t mode_switch;
	uint8_t sync_switch;

	uint8_t is_connected;
} radio_controls_t;

typedef struct {
	uint8_t axis_current_state;

} odesc_feedback_t;

typedef struct {
	float voltage;
	uint16_t raw_adc;
	int16_t setpoint_us;
	uint8_t mode;
} servo_feedback_t;

typedef struct {
	uint8_t range_mm;
	int32_t signalRate_mcps;
	uint8_t errorStatus;
} range_meas_t;

typedef struct {
    float voltage;
    float current;
    float power;
} servo_power_data_t;

typedef struct {
	servo_power_data_t steering_servo_power;
	servo_power_data_t rear_servo_power;

	servo_feedback_t steering_servo_feedback;
	servo_feedback_t rear_servo_feedback;

	range_meas_t left_tof;
	range_meas_t right_tof;

	radio_controls_t from_radio;

	odesc_feedback_t odesc;
} achter_board_t;

void achter_board_init(void);

achter_board_t* achter_board_get_ptr(void);

void buzz(void);

void no_buzz(void);

#endif /* INC_FORE_BOARD_H_ */
