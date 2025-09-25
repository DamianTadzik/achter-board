/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "achter_board.h"

#include "can-not/can_not.h"

#include "task_can_rx.h"
#include "task_adc.h"
#include "task_servo_control.h"
#include "task_servo_power_monitor.h"
#include "task_range_meas.h"

#include "task_crsf_receiver.h"
#include "task_crsf_transmitter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t task_can_rx_handle;
const osThreadAttr_t task_can_rx_attributes = {
  .name = "task_can_rx",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t task_adc_handle;
const osThreadAttr_t task_adc_attributes = {
  .name = "task_adc",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t task_servo_control_handle;
const osThreadAttr_t task_servo_control_attributes = {
  .name = "task_servo_control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t task_servo_power_monitor_handle;
const osThreadAttr_t task_servo_power_monitor_attributes = {
  .name = "task_servo_power_monitor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t task_range_meas_handle;
const osThreadAttr_t task_range_meas_attributes = {
  .name = "task_range_meas",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityNormal,
};


osThreadId_t task_crsf_receiver_handle;
const osThreadAttr_t task_crsf_receiver_attributes = {
  .name = "task_crsf_receiver",
  .stack_size = 128 * 2,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t task_crsf_transmitter_handle;
const osThreadAttr_t task_crsf_transmitter_attributes = {
  .name = "task_crsf_transmitter",
  .stack_size = 128 * 2,
  .priority = (osPriority_t) osPriorityNormal,
};


/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	volatile size_t res = xPortGetFreeHeapSize();
	UNUSED(res);

	cant_freertos_init();
	res = xPortGetFreeHeapSize();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  res = xPortGetFreeHeapSize();

  task_can_rx_handle = osThreadNew(task_can_rx, NULL, &task_can_rx_attributes);
  res = xPortGetFreeHeapSize();

  task_adc_handle = osThreadNew(task_adc, NULL, &task_adc_attributes);
  res = xPortGetFreeHeapSize();

  task_servo_control_handle = osThreadNew(task_servo_control, NULL, &task_servo_control_attributes);
  res = xPortGetFreeHeapSize();

  task_servo_power_monitor_handle = osThreadNew(task_servo_power_monitor, NULL, &task_servo_power_monitor_attributes);
  res = xPortGetFreeHeapSize();

  task_range_meas_handle = osThreadNew(task_range_meas, NULL, &task_range_meas_attributes);
  res = xPortGetFreeHeapSize();

  task_crsf_receiver_handle = osThreadNew(task_crsf_receiver, NULL, &task_crsf_receiver_attributes);
  res = xPortGetFreeHeapSize();

  task_crsf_transmitter_handle = osThreadNew(task_crsf_transmitter, NULL, &task_crsf_transmitter_attributes);
  res = xPortGetFreeHeapSize();

 UNUSED(res);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */


volatile uint32_t task_can_rx_alive;
volatile uint32_t task_adc_alive;
volatile uint32_t task_servo_control_alive;
volatile uint32_t task_servo_power_monitor_alive;
volatile uint32_t task_range_meas_alive;
volatile uint32_t task_crsf_receiver_alive;
volatile uint32_t task_crsf_transmitter_alive;

volatile UBaseType_t task_can_rx_high_watermark;
volatile UBaseType_t task_adc_high_watermark;
volatile UBaseType_t task_servo_control_high_watermark;
volatile UBaseType_t task_servo_power_monitor_high_watermark;
volatile UBaseType_t task_range_meas_high_watermark;
volatile UBaseType_t task_crsf_receiver_high_watermark;
volatile UBaseType_t task_crsf_transmitter_high_watermark;

// ======= USER TUNABLES =======
#define ODRV_NODE_ID            0x00    // <axis>.config.can.node_id (0..0x3F). Set this to your ODrive's node ID.
#define VEL_MAX_TURNS_S         80.0f   // max speed at 100% throttle [turns/s] (example)
#define ACCEL_SLEW_TURNS_SS     100.0f  // simple slew [turns/s^2]; 0=disabled
#define CURRENT_LIMIT_A         25.0f   // motor current limit [A] (example)
#define VEL_LIMIT_TURNS_S       50.0f   // controller velocity limit [turns/s] (example)
#define TASK_PERIOD_MS          10      // control update period [ms]
#define THROTTLE_DEADBAND_PCT   4       // +/- percent deadband around zero

// ======= ODrive CANSimple Command IDs (11-bit std ID) =======
// Message ID = (node_id << 5) | cmd_id  (node_id: 6 bits, cmd_id: 5 bits). See docs.
#define CMD_HEARTBEAT           0x001   // not sent here, but for reference
#define CMD_SET_AXIS_STATE      0x007
#define CMD_SET_CONTROLLER_MODE 0x00B
#define CMD_SET_INPUT_POS       0x00C
#define CMD_SET_INPUT_VEL       0x00D
#define CMD_SET_INPUT_TORQUE    0x00E
#define CMD_SET_LIMITS          0x00F

// ======= ODrive Axis States (subset) =======
#define AXIS_STATE_IDLE                 1
#define AXIS_STATE_CLOSED_LOOP_CONTROL  8

// ======= ODrive Control/Input Modes (see docs) =======
#define CONTROL_MODE_VELOCITY_CONTROL  2
#define INPUT_MODE_PASSTHROUGH         1

static inline uint16_t odrv_can_id(uint8_t node_id, uint8_t cmd_id) {
    // Pack into standard 11-bit identifier
    return (uint16_t)(((node_id & 0x3F) << 5) | (cmd_id & 0x1F));
}

static inline void put_f32_le(uint8_t *dst, float v) {
    uint32_t u;
    memcpy(&u, &v, sizeof(u)); // strict-alias safe enough for embedded
    dst[0] = (uint8_t)(u & 0xFF);
    dst[1] = (uint8_t)((u >> 8) & 0xFF);
    dst[2] = (uint8_t)((u >> 16) & 0xFF);
    dst[3] = (uint8_t)((u >> 24) & 0xFF);
}

static void odrv_send_set_controller_mode(uint8_t node_id, uint8_t control_mode, uint8_t input_mode) {
    cant_generic_struct_t m = {0};
    m.msg_id  = odrv_can_id(node_id, CMD_SET_CONTROLLER_MODE);
    m.msg_dlc = 2; // 2 bytes: control_mode (u8), input_mode (u8)
    m.msg_payload[0] = control_mode;
    m.msg_payload[1] = input_mode;
    cant_transmit(&m);
}

static void odrv_send_set_limits(uint8_t node_id, float vel_limit_turns_s, float current_limit_a) {
    cant_generic_struct_t m = {0};
    m.msg_id  = odrv_can_id(node_id, CMD_SET_LIMITS);
    m.msg_dlc = 8; // 4B vel_limit, 4B current_limit (floats, LE)
    put_f32_le(&m.msg_payload[0], vel_limit_turns_s);
    put_f32_le(&m.msg_payload[4], current_limit_a);
    cant_transmit(&m);
}

static void odrv_send_set_axis_state(uint8_t node_id, uint32_t axis_state) {
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

static void odrv_send_set_input_vel(uint8_t node_id, float vel_turns_s, float torque_ff_nm) {
    cant_generic_struct_t m = {0};
    m.msg_id  = odrv_can_id(node_id, CMD_SET_INPUT_VEL);
    m.msg_dlc = 8; // 4B velocity, 4B torque_ff (floats, LE)
    put_f32_le(&m.msg_payload[0], vel_turns_s);
    put_f32_le(&m.msg_payload[4], torque_ff_nm);
    cant_transmit(&m);
}

#define CMD_CLEAR_ERRORS 0x018
static void odrv_send_clear_errors(uint8_t node_id)
{
	cant_generic_struct_t m = {0};
	m.msg_id  = odrv_can_id(node_id, CMD_CLEAR_ERRORS);
	m.msg_dlc = 0;
	cant_transmit(&m);
}

volatile test_mode_xd = 0;
volatile uint8_t node_id = ODRV_NODE_ID;
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	achter_board_t* ab_ptr = achter_board_get_ptr();
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin);

	  task_can_rx_high_watermark           = uxTaskGetStackHighWaterMark((TaskHandle_t)task_can_rx_handle);
	  task_adc_high_watermark              = uxTaskGetStackHighWaterMark((TaskHandle_t)task_adc_handle);
	  task_servo_control_high_watermark    = uxTaskGetStackHighWaterMark((TaskHandle_t)task_servo_control_handle);
	  task_servo_power_monitor_high_watermark = uxTaskGetStackHighWaterMark((TaskHandle_t)task_servo_power_monitor_handle);
	  task_range_meas_high_watermark       = uxTaskGetStackHighWaterMark((TaskHandle_t)task_range_meas_handle);

	  task_crsf_receiver_high_watermark       = uxTaskGetStackHighWaterMark((TaskHandle_t)task_crsf_receiver_handle);
	  task_crsf_transmitter_high_watermark    = uxTaskGetStackHighWaterMark((TaskHandle_t)task_crsf_transmitter_handle);

	  /* startup */
	  if (test_mode_xd == -1)
	  {
		  /* clear all errors and start sending the setpoint over can so watchdog does not trip */
		  odrv_send_clear_errors(node_id);
		  odrv_send_set_input_vel(node_id, 0.0f, 0.0f);
		  /* set the axis requested state to STARTUP_SEQUENCE=2 */
		  odrv_send_set_axis_state(node_id, 2);
		  /* poll the current_state til it reaches 1, then set to 8 */
		  osDelay(200);
		  do
		  {
			  odrv_send_set_input_vel(node_id, 0.0f, 0.0f);
			  osDelay(200);
		  }
		  while (ab_ptr->odesc.axis_current_state == 4);
		  odrv_send_set_axis_state(node_id, 8);
		  /* startup complete */
		  test_mode_xd = 2;
	  }

	  if (test_mode_xd == 1)
	  {
		odrv_send_set_controller_mode(node_id, CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH); // Velocity/Passthrough
		odrv_send_set_limits(node_id, VEL_LIMIT_TURNS_S, CURRENT_LIMIT_A);                               // Limits
		odrv_send_set_axis_state(node_id, AXIS_STATE_CLOSED_LOOP_CONTROL);
	    test_mode_xd = 2;
	  }
	  if (test_mode_xd == 2)
	  {
		// 1) Read throttle and clamp
		int16_t thr = ab_ptr->radio.throttle;           // -100..100 (%)
		if (thr > 100) thr = 100;
		if (thr < -100) thr = -100;

		// 2) Deadband
		if (abs(thr) <= THROTTLE_DEADBAND_PCT) thr = 0;

		// 3) Map to turns/s
		const float target_vel = (float)thr * (VEL_MAX_TURNS_S / 100.0f);

		// 5) Send Set_Input_Vel (also feeds watchdog)
		odrv_send_set_input_vel(node_id, target_vel, 0.0f);
	  }

	  osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

