/*
 * can_not.c
 *
 *  Created on: Nov 14, 2024
 *      Author: brzan
 */
#include "can.h"
#include "main.h"
#include "can_not.h"
#include "cmsis_os2.h"

/* ################################################################################### */
/* ############################## CAN instance selector ############################## */
/* ################################################################################### */

#ifdef CANT1	/* hcan1 STM32F446RE */
CAN_HandleTypeDef *hcanPtr = &hcan1;
#endif

#ifdef CANT2	/* hcan2 STM32F446RE */
CAN_HandleTypeDef *hcanPtr = &hcan2;
#endif

#ifdef CANT		/* hcan STM32Fxxxx */
CAN_HandleTypeDef *hcanPtr = &hcan;
#endif

/* ################################################################################### */
/* ######################## CAN debug counters and variables  ######################## */
/* ################################################################################### */

#ifdef CANT_DEBUG		/* "can_not" global volatile debug counters */
volatile uint32_t debug_TxMailbox0Complete_callback_execution_counter = 0;
volatile uint32_t debug_TxMailbox1Complete_callback_execution_counter = 0;
volatile uint32_t debug_TxMailbox2Complete_callback_execution_counter = 0;

volatile uint32_t debug_hal_add_tx_message_error_counter = 0; //
volatile uint32_t debug_tx_fallback_queue_is_full_counter = 0;
volatile uint32_t debug_tx_fallback_queue_messages_put_counter = 0;
volatile uint32_t debug_tx_fallback_queue_messages_get_counter = 0;

volatile uint32_t debug_RxFifo0MsgPending_callback_execution_counter = 0;
volatile uint32_t debug_RxFifo1MsgPending_callback_execution_counter = 0;

volatile uint32_t debug_hal_get_rx_message_error_counter = 0;
volatile uint32_t debug_rx_general_queue_is_full_counter = 0;
volatile uint32_t debug_rx_general_queue_messages_put_counter = 0;
volatile uint32_t debug_rx_general_queue_messages_get_counter = 0;
#endif

/* Private "can_not" freeRTOS queues */
static osMessageQueueId_t cant_tx_fallback_queue;
static osMessageQueueId_t cant_rx_general_queue;

/* ################################################################################### */
/* ################################ Exported functions ############################### */
/* ################################################################################### */

void cant_main_init(void)// FINE TO RENAME cant_main_init
{
	/*
	 * This function configures filters, this is very important because message rx works that way:
	 * message is received on RX pin -> goes to filter -> if msg passed filter it goes to fifo
	 * That means: missconfiguration of the filters will probbably stop any messages receive.
	 * PAY ATTENTION TO CAN instance used CAN1 or CAN2 some controllers have only one CAN instance
	 *
	 * on STM32F446RE there are two instances
	 * CAN1 - master - lower filter banks numbers
	 * CAN2 - slave - higher filter banks numbers - needs master clock to be enabled in order to work bc
	 * it does not have its own memory
	 *
	 * Luckily HAL manages it all for us :)) we just have to configure things properly in this function
	 */
#ifdef CANT1
	CAN_FilterTypeDef sFilterConfig0 = {
			.FilterIdHigh = 0,
			.FilterIdLow = 0,
			.FilterMaskIdHigh = 0,
			.FilterMaskIdLow = 0,

			.FilterFIFOAssignment = CAN_FILTER_FIFO0,
			.FilterBank = 0,

			.FilterMode = CAN_FILTERMODE_IDMASK,
			.FilterScale = CAN_FILTERSCALE_16BIT,
			.FilterActivation = CAN_FILTER_ENABLE,
			.SlaveStartFilterBank = 13,
	};
	HAL_CAN_ConfigFilter(hcanPtr, &sFilterConfig0);

	CAN_FilterTypeDef sFilterConfig1 = {
			.FilterIdHigh = 0,
			.FilterIdLow = 0,
			.FilterMaskIdHigh = 0,
			.FilterMaskIdLow = 0,

			.FilterFIFOAssignment = CAN_FILTER_FIFO1,
			.FilterBank = 1,

			.FilterMode = CAN_FILTERMODE_IDMASK,
			.FilterScale = CAN_FILTERSCALE_16BIT,
			.FilterActivation = CAN_FILTER_ENABLE,
			.SlaveStartFilterBank = 13,
	};
	HAL_CAN_ConfigFilter(hcanPtr, &sFilterConfig1);
#endif

#ifdef CANT2
	CAN_FilterTypeDef sFilterConfig0 = {
			.FilterIdHigh = 0,
			.FilterIdLow = 0,
			.FilterMaskIdHigh = 0,
			.FilterMaskIdLow = 0,

			.FilterFIFOAssignment = CAN_FILTER_FIFO0,
			.FilterBank = 14,

			.FilterMode = CAN_FILTERMODE_IDMASK,
			.FilterScale = CAN_FILTERSCALE_16BIT,
			.FilterActivation = CAN_FILTER_ENABLE,
			.SlaveStartFilterBank = 13,
	};
	HAL_CAN_ConfigFilter(hcanPtr, &sFilterConfig0);

	CAN_FilterTypeDef sFilterConfig1 = {
			.FilterIdHigh = 0,
			.FilterIdLow = 0,
			.FilterMaskIdHigh = 0,
			.FilterMaskIdLow = 0,

			.FilterFIFOAssignment = CAN_FILTER_FIFO1,
			.FilterBank = 15,

			.FilterMode = CAN_FILTERMODE_IDMASK,
			.FilterScale = CAN_FILTERSCALE_16BIT,
			.FilterActivation = CAN_FILTER_ENABLE,
			.SlaveStartFilterBank = 13,
	};
	HAL_CAN_ConfigFilter(hcanPtr, &sFilterConfig1);
#endif

#ifdef CANT
	CAN_FilterTypeDef sFilterConfig0 = {
			.FilterIdHigh = 0,
			.FilterIdLow = 0,
			.FilterMaskIdHigh = 0,
			.FilterMaskIdLow = 0,

			.FilterFIFOAssignment = CAN_FILTER_FIFO0,
			.FilterBank = 0,

			.FilterMode = CAN_FILTERMODE_IDMASK,
			.FilterScale = CAN_FILTERSCALE_16BIT,
			.FilterActivation = CAN_FILTER_ENABLE,
			.SlaveStartFilterBank = 13,
	};
	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig0);

	CAN_FilterTypeDef sFilterConfig1 = {
			.FilterIdHigh = 0,
			.FilterIdLow = 0,
			.FilterMaskIdHigh = 0,
			.FilterMaskIdLow = 0,

			.FilterFIFOAssignment = CAN_FILTER_FIFO1,
			.FilterBank = 1,

			.FilterMode = CAN_FILTERMODE_IDMASK,
			.FilterScale = CAN_FILTERSCALE_16BIT,
			.FilterActivation = CAN_FILTER_ENABLE,
			.SlaveStartFilterBank = 13,
	};
	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig1);
#endif

	HAL_CAN_Start(hcanPtr);

	HAL_CAN_ActivateNotification(hcanPtr, CAN_IT_TX_MAILBOX_EMPTY |
			CAN_IT_RX_FIFO0_MSG_PENDING |
			CAN_IT_RX_FIFO1_MSG_PENDING);
}

void cant_freertos_init(void)
{
	/* 8 is very nice number so 3 tx mailboxes + 5 queue places = 8 messages capacity */
	uint32_t tx_msg_count = 5;
	const osMessageQueueAttr_t cant_tx_fallback_queue_attributes = {
			.name = "cantTxQueue",
	};
	cant_tx_fallback_queue = osMessageQueueNew(tx_msg_count, sizeof(cant_generic_struct_t), &cant_tx_fallback_queue_attributes);

	/* If you are willing to poll cant_receive less frequently, you should probably increase the size of the following. */
	uint32_t rx_msg_count = 8;
	const osMessageQueueAttr_t cant_rx_general_queue_attributes = {
			.name = "cantRxQueue",
	};
	cant_rx_general_queue = osMessageQueueNew(rx_msg_count, sizeof(cant_generic_struct_t), &cant_rx_general_queue_attributes);

	/* If queues were not created */
	if (cant_rx_general_queue == NULL || cant_tx_fallback_queue == NULL)
	{
		Error_Handler();
	}
}

void cant_transmit(cant_generic_struct_t* msg)
{
	/* Enter a critical section */
	__disable_irq();

	/* Get level of TX mailboxes utilization */
	uint32_t tx_mailboxes_free_level = HAL_CAN_GetTxMailboxesFreeLevel(hcanPtr);

	/* if there is zero free TX mailboxes */
	if (tx_mailboxes_free_level == 0)
	{
		/* Add the message to TX queue as there are no free mailboxes right now */
		osStatus_t result = osMessageQueuePut(cant_tx_fallback_queue, msg, 0U, 0U);	/* mq_id, msg_ptr, msg_prio, timeout */

		/* We are aware that if this TX queue is full the message is dropped */
#ifdef CANT_DEBUG
		if (result != osOK) debug_tx_fallback_queue_is_full_counter++;
		else debug_tx_fallback_queue_messages_put_counter++;
#else
		UNUSED(result);
#endif
	}
	else
	{
		/* Add message to TX mailbox */
		uint32_t tx_mailbox_used = 0;
		CAN_TxHeaderTypeDef header = {
				.StdId = msg->msg_id,	/* Standard 11-bit identifier */
				.ExtId = 0,				/* Pass zero as we use standard ID not extended ID */
				.IDE = CAN_ID_STD,		/* Selects the standard ID length */
				.RTR = CAN_RTR_DATA,	/* Selects that this frame is standard data frame with payload */
				.DLC = msg->msg_dlc,	/* Must be a number between Min_Data = 0 and Max_Data = 8 */
				.TransmitGlobalTime = DISABLE, 	/* Disabled - refer to documentation */
		};
		if (HAL_OK != HAL_CAN_AddTxMessage(hcanPtr, &header, msg->msg_payload, &tx_mailbox_used))
		{
#ifdef CANT_DEBUG
			debug_hal_add_tx_message_error_counter++;
#endif
		}
		UNUSED(tx_mailbox_used);
	}
	/* Exit the critical section */
	__enable_irq();
}

cant_status_t cant_receive(cant_generic_struct_t* msg)
{
	if (osOK == osMessageQueueGet(cant_rx_general_queue, msg, 0U, 0U))
	{
#ifdef CANT_DEBUG
		debug_rx_general_queue_messages_get_counter++;
#endif
		return MESSAGE_RECEIVED;
	}
	else return MESSAGE_NOT_RECEIVED;
}


/* ################################################################################### */
/* ########################## HAL Callback implementations ########################### */
/* ################################################################################### */

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
#ifdef CANT_DEBUG
	debug_TxMailbox0Complete_callback_execution_counter++;
#endif

	cant_generic_struct_t tmp;
	/* If there is message in fallback queue then send it */
	if (osOK == osMessageQueueGet(cant_tx_fallback_queue, &tmp, 0U, 0U))
	{
#ifdef CANT_DEBUG
		debug_tx_fallback_queue_messages_get_counter++;
#endif
		cant_transmit(&tmp);
	}
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
#ifdef CANT_DEBUG
	debug_TxMailbox1Complete_callback_execution_counter++;
#endif

	cant_generic_struct_t tmp;
	/* If there is message in fallback queue then send it */
	if (osOK == osMessageQueueGet(cant_tx_fallback_queue, &tmp, 0U, 0U))
	{
#ifdef CANT_DEBUG
		debug_tx_fallback_queue_messages_get_counter++;
#endif
		cant_transmit(&tmp);
	}
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
#ifdef CANT_DEBUG
	debug_TxMailbox2Complete_callback_execution_counter++;
#endif

	cant_generic_struct_t tmp;
	/* If there is message in fallback queue then send it */
	if (osOK == osMessageQueueGet(cant_tx_fallback_queue, &tmp, 0U, 0U))
	{
#ifdef CANT_DEBUG
		debug_tx_fallback_queue_messages_get_counter++;
#endif
		cant_transmit(&tmp);
	}
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
#ifdef CANT_DEBUG
	debug_RxFifo0MsgPending_callback_execution_counter++;
#endif

	/* Create temporary variables for storing header and payload */
	CAN_RxHeaderTypeDef tmp_header = { 0 };
	cant_generic_struct_t tmp = { 0 };

	/* Obtain the message and the header via HAL */
	if (HAL_OK != HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &tmp_header, tmp.msg_payload))
	{
#ifdef CANT_DEBUG
		debug_hal_get_rx_message_error_counter++;
#endif
	}

	/* Obtain the standard ID and DLC from header */
	tmp.msg_id = tmp_header.StdId;
	UNUSED(tmp_header.ExtId);
	UNUSED(tmp_header.IDE);	// MAYBE Might add error handling if extended id is recieved?
	UNUSED(tmp_header.RTR);
	tmp.msg_dlc = tmp_header.DLC;
	UNUSED(tmp_header.Timestamp);
	UNUSED(tmp_header.FilterMatchIndex);

	/* Add the message to RX queue */
	osStatus_t result = osMessageQueuePut(cant_rx_general_queue, &tmp, 0U, 0U);

	/* We are aware that if RX general queue is full the message is dropped */
#ifdef CANT_DEBUG
	if (result != osOK) debug_rx_general_queue_is_full_counter++;
	else debug_rx_general_queue_messages_put_counter++;
#else
	UNUSED(result);
#endif
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
#ifdef CANT_DEBUG
	debug_RxFifo1MsgPending_callback_execution_counter++;
#endif

	/* Create temporary variables for storing header and payload */
	CAN_RxHeaderTypeDef tmp_header = { 0 };
	cant_generic_struct_t tmp = { 0 };

	/* Obtain the message and the header via HAL */
	if (HAL_OK != HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &tmp_header, tmp.msg_payload))
	{
#ifdef CANT_DEBUG
		debug_hal_get_rx_message_error_counter++;
#endif
	}

	/* Obtain the standard ID and DLC from header */
	tmp.msg_id = tmp_header.StdId;
	UNUSED(tmp_header.ExtId);
	UNUSED(tmp_header.IDE);	// MAYBE Might add error handling if extended id is received?
	UNUSED(tmp_header.RTR);
	tmp.msg_dlc = tmp_header.DLC;
	UNUSED(tmp_header.Timestamp);
	UNUSED(tmp_header.FilterMatchIndex);

	/* Add the message to RX queue */
	osStatus_t result = osMessageQueuePut(cant_rx_general_queue, &tmp, 0U, 0U);

	/* We are aware that if RX general queue is full the message is dropped */
#ifdef CANT_DEBUG
	if (result != osOK) debug_rx_general_queue_is_full_counter++;
	else debug_rx_general_queue_messages_put_counter++;
#else
	UNUSED(result);
#endif
}

// MAYBE might add void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan); for monitoring the dropped variables with debugger?
