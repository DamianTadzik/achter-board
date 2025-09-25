/*
 * task_crsf_receiver.c
 *
 *  Created on: Sep 24, 2025
 *      Author: brzan
 */
#include <crsf/crsf_port.h>
#include "task_crsf_receiver.h"
#include "main.h"
#include "cmsis_os2.h"
#include "achter_board.h"


static inline uint16_t map_u16(uint16_t x,
                               uint16_t in_min,  uint16_t in_max,
                               uint16_t out_min, uint16_t out_max)
{
    // Avoid divide by zero
    if (in_max == in_min) return out_min;

    uint32_t num   = (uint32_t)(x - in_min) * (out_max - out_min);
    uint32_t denom = (uint32_t)(in_max - in_min);
    return (uint16_t)(out_min + (num / denom));
}

static inline int16_t map_i16(int16_t x,
                              int16_t in_min,  int16_t in_max,
                              int16_t out_min, int16_t out_max)
{
    if (in_max == in_min) return out_min;

    int32_t num   = (int32_t)(x - in_min) * (out_max - out_min);
    int32_t denom = (int32_t)(in_max - in_min);
    return (int16_t)(out_min + (num / denom));
}



volatile uint16_t ch[CRSF_MAX_CHANNEL];
volatile uint32_t age_us;

volatile CRSF_LinkStats_t stats;

volatile uint32_t g_link_losses_cnt;
volatile uint32_t g_flags;
// ---- Receiver: wait for frames, push messages, check failsafe ----
extern volatile uint32_t task_crsf_receiver_alive;
void task_crsf_receiver(void *argument)
{
	achter_board_t* ab_ptr = achter_board_get_ptr();

    CRSF_SetNotifyThread(osThreadGetId());

    for (;;)
    {
    	task_crsf_receiver_alive++;
    	g_flags = osThreadFlagsWait(CRSF_TFLAG_RX_FRAME | CRSF_TFLAG_LINK_STATS,
                                          osFlagsWaitAny, 100);
        if (g_flags == CRSF_TFLAG_RX_FRAME)
        {
            if (CRSF_GetChannels(ch, &age_us))
            {
            	ab_ptr->radio.throttle = map_i16(ch[0],
            									 172, 1811,
												 -100, 100);
            	ab_ptr->radio.steering = map_u16(ch[1],
												 172, 1811,
												 1000, 2000);
            }
        }

        if (g_flags == CRSF_TFLAG_LINK_STATS)
        {
            if (CRSF_GetLinkStats(&stats))
            {

            }
        }

        // Failsafe guard: if link lost, you can trigger your shutdown logic here
        if (CRSF_IsLinkLost())
        {
        	g_link_losses_cnt++;
        	ab_ptr->radio.is_connected = 0;
        	// TODO: call your safe-shutdown / disarm routine
        	ab_ptr->radio.throttle = 0;
        	ab_ptr->radio.steering = 1500;
        }
        else
        {
        	ab_ptr->radio.is_connected = 1;
        }
    }
}
