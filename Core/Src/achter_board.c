/*
 * achter_board.c
 *
 *  Created on: Sep 1, 2025
 *      Author: brzan
 */
#include "achter_board.h"

static achter_board_t fb;

void achter_board_init(void)
{

}

achter_board_t* achter_board_get_ptr(void)
{
	return &fb;
}
