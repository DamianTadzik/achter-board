/*
 * task_motor_control.h
 *
 *  Created on: Sep 25, 2025
 *      Author: brzan
 */

#ifndef INC_TASK_MOTOR_CONTROL_H_
#define INC_TASK_MOTOR_CONTROL_H_

// ======= ODrive Axis States (full) =======
#define AXIS_STATE_UNDEFINED             0
#define AXIS_STATE_IDLE                  1
#define AXIS_STATE_STARTUP_SEQUENCE      2
#define AXIS_STATE_FULL_CALIBRATION      3
#define AXIS_STATE_MOTOR_CALIBRATION     4
#define AXIS_STATE_SENSORLESS_CONTROL    5
#define AXIS_STATE_ENCODER_INDEX_SEARCH  6
#define AXIS_STATE_ENCODER_OFFSET_CALIB  7
#define AXIS_STATE_CLOSED_LOOP_CONTROL   8
#define AXIS_STATE_LOCKIN_SPIN           9
#define AXIS_STATE_ENCODER_DIR_FIND      10
#define AXIS_STATE_HOMING                11
#define AXIS_STATE_ENCODER_HALL_POLARITY 12
#define AXIS_STATE_ENCODER_HALL_PHASE    13

// ======= ODrive Control Modes (full) =======
#define CONTROL_MODE_VOLTAGE_CONTROL     0
#define CONTROL_MODE_TORQUE_CONTROL      1
#define CONTROL_MODE_VELOCITY_CONTROL    2
#define CONTROL_MODE_POSITION_CONTROL    3

// ======= ODrive Input Modes (full) =======
#define INPUT_MODE_INACTIVE              0
#define INPUT_MODE_PASSTHROUGH           1
#define INPUT_MODE_VEL_RAMP              2
#define INPUT_MODE_POS_FILTER            3
#define INPUT_MODE_MIX_CHANNELS          4
#define INPUT_MODE_TRAP_TRAJ             5
#define INPUT_MODE_TORQUE_RAMP           6
#define INPUT_MODE_MIRROR                7
#define INPUT_MODE_TUNING                8

// ======= ODrive CANSimple Command IDs (11-bit std ID) =======
// Message ID = (node_id << 5) | cmd_id  (node_id: 6 bits, cmd_id: 5 bits). See docs.
#define CMD_HEARTBEAT           0x001   // not sent here, but for reference
#define CMD_SET_AXIS_STATE      0x007
#define CMD_SET_CONTROLLER_MODE 0x00B
#define CMD_SET_INPUT_POS       0x00C
#define CMD_SET_INPUT_VEL       0x00D
#define CMD_SET_INPUT_TORQUE    0x00E
#define CMD_SET_LIMITS          0x00F
#define CMD_CLEAR_ERRORS 		0x018

void task_motor_control(void *argument);

#endif /* INC_TASK_MOTOR_CONTROL_H_ */
