/*
 * achter_board.c
 *
 *  Created on: Sep 1, 2025
 *      Author: brzan
 */
#include "achter_board.h"

static achter_board_t ab;

void achter_board_init(void)
{

}

achter_board_t* achter_board_get_ptr(void)
{
	return &ab;
}

void buzz(void)
{
	HAL_GPIO_WritePin(GPIO_BUZZER_GPIO_Port, GPIO_BUZZER_Pin, GPIO_PIN_RESET);
}

void no_buzz(void)
{
	HAL_GPIO_WritePin(GPIO_BUZZER_GPIO_Port, GPIO_BUZZER_Pin, GPIO_PIN_SET);
}
