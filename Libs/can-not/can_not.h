/*
 * can_not.h
 *
 *  Created on: Nov 14, 2024
 *      Author: brzan
 */

/**** EXAMPLE USAGE OF cant_transmit ****************************************************
cant_generic_struct_t tmp = {
	.msg_id = YOUR_FRAME_ID,
	.msg_dlc = YOUR_MESSAGE_DLC,
	.msg_payload = { 0 }, <- You can copy payload to this field in many many ways...
};
memcpy(tmp.msg_payload, sourcePtr, size); <- memcpy for example..
cant_transmit(&tmp);
 ****************************************************************************************/

/**** EXAMPLE USAGE OF cant_receive *****************************************************
cant_generic_struct_t tmp = { 0 };
while (1)
{
	if (MESSAGE_RECEIVED == cant_receive(&tmp))
	{
		Process your received tmp message here...
	}
	osThreadYield();	<- preffered no delay inside task loop in freeRTOS
}
 ****************************************************************************************/

#ifndef INC_CAN_NOT_H_
#define INC_CAN_NOT_H_

typedef struct {	/* Only standard 11 bit ID's. Up to 8 bytes of payload. */
	uint16_t msg_id : 11;	/* 11 bytes wide field for identifier [0 0x7FF] */
	uint8_t msg_dlc : 4;	/* 4 byte wide field for data length code [0 8] */
	uint8_t padding : 1;
	uint8_t msg_payload[8];
} cant_generic_struct_t;	/* This structure is 10 bytes in size total */

typedef enum {
	MESSAGE_NOT_RECEIVED = 0U,
	MESSAGE_RECEIVED = 1U,
} cant_status_t;


/**
 * Example CAN MX configuration should be as following (SYSCLK = 144 Mhz):
 *	Prescaler = 18 (it is determined by core frequency, and it determines the Time Quantum)
 *	Time Quantum = 500 ns
 *	Time Quanta in Bit Segment 1 = 12 Times
 *	Time Quanta in Bit Segment 1 = 3 Times
 *	Time for one Bit = 8000 ns
 *	Baud Rate = 125000 bit/s
 *	ReSynchronization Jump Width = 1 Time
 *
 *	Time Triggered Communication Mode = Disable
 *	Automatic Bus-Off Management = Enable
 *	Automatic Wake-Up Mode = Disable
 *	Automatic Retransmission = Enable
 *	Receive Fifo Locked Mode = Disable
 *	Transmit Fifo Priority = Disable
 *
 *	Operating Mode = Normal
 *
 * Following interrupts should be enabled:
 * 	CAN TX interrupt
 * 	CAN RX0 interrupt
 * 	CAN RX1 interrupt
 *
 * 	Another important thing is AGREEMENT:
 * 	- that we are not using extended IDs in the system
 * 	- system runs on 125000 kbit/s
 * 	- max dlc is 8 bytes
 */

/**
 * You have to define can instance that you are using. You should do it in main.h.
 ****	for STM32F446RE you can choose between CANT1 and CANT2
 ****	for STM32Fxxxx	you have only CANT
 ****	example: #define CANT2
 *
 * You also could activate debugging counters by defining a macro in main.h.
 ****	example: #define CANT_DEBUG
 */

/**
 * 	This function has to be called in main.c
 * 		Inside the main(void) function
 * 			Just after the USER CODE BEGIN 2 (that's after MX_*_Init() function calls
 *
 * 	It initalizes filters and then enables the CAN peripherial (params of this peripherial are set in MX)
 *
 * 	Please pay attention to CAN instance that you are using.
 */
void cant_main_init(void);

/**
 * 	This function has to be called in freertos.c,
 * 		Inside the MX_FREERTOS_Init(void) function
 * 			Between USER CODE BEGIN RTOS_QUEUES and USER CODE END RTOS_QUEUES
 *
 * 	It creates the necessary FreeRTOS queues - user has to check if there is enough memory on the heap :)))
 */
void cant_freertos_init(void);

/**
 * This function can be called whenever you want to transmit message on the CAN bus
 */
void cant_transmit(cant_generic_struct_t* msg);

/**
 * This function can be polled in loop in order to get messages
 * if returned status is 1 that means the message has been received
 */
cant_status_t cant_receive(cant_generic_struct_t* msg);

#endif /* INC_CAN_NOT_H_ */
