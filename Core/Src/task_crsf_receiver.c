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
            	ab_ptr->radio.throttle = ch[0];
            	ab_ptr->radio.steering = ch[1];
            }
        }

        if (g_flags == CRSF_TFLAG_LINK_STATS)
        {
            if (CRSF_GetLinkStats(&stats))
            {
//                if (g_crsfLinkQ) osMessageQueuePut(g_crsfLinkQ, &lmsg, 0, 0);
            }
            int tmp = 0;
        }

        // Failsafe guard: if link lost, you can trigger your shutdown logic here
        if (CRSF_IsLinkLost()) {
        	g_link_losses_cnt++;
            // TODO: call your safe-shutdown / disarm routine
            // e.g., set a global flag, cut PWM, etc.

        	ab_ptr->radio.throttle = 992;
        	ab_ptr->radio.steering = 992;
        }
    }
}
