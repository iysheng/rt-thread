/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2021-10-01     iysheng           TCD1209 driver
 */

#include <rtconfig.h>
#include <rtdevice.h>
#include "gd32f20x_timer.h"
#include "drv_tcd1209.h"

#define DBG_LVL    DBG_INFO
#define DBG_TAG    "tcd1209"
#include <rtdbg.h>

/*
 * AHB = 120M
 * APB2 = 120M
 * APB1 = 60M
 *
 * APB1 的分频作为 TIMER1/2/3/4/5/6 11/12/13 的时钟, 最大为 60MHz 并且如果分频为 1 ,那么 x1 否则 x2
 * APB2 的分频作为 TIMER0/7/8/9/10 的时钟, 最大为 120MHz 并且如果分频为 1 ,那么 x1 否则 x2
 * */

int tcd1209_hw_init(void)
{
    int ret;
    timer_parameter_struct timer4f1;

    timer4f1.prescaler         = 0U;
    timer4f1.alignedmode       = TIMER_COUNTER_EDGE;
    timer4f1.counterdirection  = TIMER_COUNTER_UP;
    timer4f1.period            = 0U;
    timer4f1.clockdivision     = TIMER_CKDIV_DIV1;
    timer4f1.repetitioncounter = 0U;

    LOG_I("Hello TCD1209");
    rcu_periph_clock_enable(RCU_TIMER3);
    timer_init(TIMER3, &timer4f1);
    timer_channel_output_mode_config(TIMER3, TIMER_CH_1, TIMER_OC_MODE_PWM1);
    timer_autoreload_value_config(TIMER3, 119);
    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_1, 9);
    timer_channel_output_state_config(TIMER3, TIMER_CH_1, ENABLE);
    timer_interrupt_enable(TIMER3, TIMER_INT_CH1);
    timer_enable(TIMER3);

    return ret;
}
 INIT_PREV_EXPORT(tcd1209_hw_init);

