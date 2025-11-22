/*
 * task_can_rx.c
 *
 *  Created on: Aug 31, 2025
 *      Author: brzan
 */
#include "task_can_rx.h"
#include "main.h"
#include "cmsis_os2.h"
#include "achter_board.h"

#include "can-not/can_not.h"
#include <string.h>
#include "can-messages-mini-celka/src/cmmc.h"

volatile uint32_t g_can_messages_arrived = 0;
volatile uint32_t g_can_messages_decoded = 0;


cant_generic_struct_t g_tmp = { 0 };
static void debug_g_tmp(cant_generic_struct_t *src)
{
	g_tmp.msg_dlc = src->msg_dlc;
	g_tmp.msg_id = src->msg_id;
	memcpy(g_tmp.msg_payload, src->msg_payload, 8);
}


extern volatile uint32_t task_can_rx_alive;
void task_can_rx(void *argument)
{
	achter_board_t* ab_ptr = achter_board_get_ptr();

	cant_generic_struct_t l_tmp = { 0 };

	while (1)
	{
		if (MESSAGE_RECEIVED == cant_receive(&l_tmp))
		{
			//Process your received g_tmp message here...
			switch (l_tmp.msg_id)
			{
			case 0x001:
				// TODO FIXME decode this properly via dbc and add everything whats needed to ab structure xd
				ab_ptr->odesc.axis_current_state = l_tmp.msg_payload[4];

				g_can_messages_decoded++;
				break;

			case CMMC_ODRIVE_GET_BUS_VOLTAGE_CURRENT_FRAME_ID:
				// Do not use CANTOOLS to decode IEEE Floats
//				struct cmmc_odrive_get_bus_voltage_current_t tmp = { 0 };
//
//				cmmc_odrive_get_bus_voltage_current_unpack(&tmp, l_tmp.msg_payload,
//						CMMC_ODRIVE_GET_BUS_VOLTAGE_CURRENT_LENGTH);
//
//				ab_ptr->odesc.bus_voltage =
//						cmmc_odrive_get_bus_voltage_current_bus_voltage_decode(
//								tmp.bus_voltage);
//				ab_ptr->odesc.bus_current =
//						cmmc_odrive_get_bus_voltage_current_bus_current_decode(
//								tmp.bus_current);

				// Decode by simply copying bytes instead :>
				memcpy(&ab_ptr->odesc.bus_voltage, &l_tmp.msg_payload[0], 4);
				memcpy(&ab_ptr->odesc.bus_current, &l_tmp.msg_payload[4], 4);

				g_can_messages_decoded++;
				break;

			default:
				break;
			}
			g_can_messages_arrived++;
		}
		task_can_rx_alive++;
		osThreadYield();	//<- preffered no delay inside this task loop in freeRTOS
	}
}


