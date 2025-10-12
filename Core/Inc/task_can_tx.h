/*
 * task_can_tx.h
 *
 *  Created on: Oct 11, 2025
 *      Author: brzan
 */

#ifndef INC_TASK_CAN_TX_H_
#define INC_TASK_CAN_TX_H_

#include "achter_board.h"

void send_cmmc_radio_control(achter_board_t* ab_ptr);

void task_can_tx(void *argument);

#endif /* INC_TASK_CAN_TX_H_ */
