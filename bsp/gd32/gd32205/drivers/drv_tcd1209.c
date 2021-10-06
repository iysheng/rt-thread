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

/**
  * @brief AD9945 初始化
  * @param void: 
  * retval .
  */
static void ad9945_device_init(void)
{
    /* timer 4 AD9945 device */
    timer_parameter_struct timer4shp, timer4shd, timer4dataclk, timer4clpob, timer4pblk;

    timer4shp.prescaler         = 0U;
    timer4shp.alignedmode       = TIMER_COUNTER_EDGE;
    timer4shp.counterdirection  = TIMER_COUNTER_UP;
    timer4shp.period            = 0U;
    timer4shp.clockdivision     = TIMER_CKDIV_DIV1;
    timer4shp.repetitioncounter = 0U;
}

int tcd1209_hw_init(void)
{
    int ret;
    /* timer 4 TCD1209 device */
    timer_parameter_struct timer4f1, timer4f2, timer4cp, timer4rs, timer4sh;

    timer4f1.prescaler         = 5U;
    timer4f1.alignedmode       = TIMER_COUNTER_EDGE;
    timer4f1.counterdirection  = TIMER_COUNTER_UP;
    timer4f1.period            = 0U;
    timer4f1.clockdivision     = TIMER_CKDIV_DIV1;
    timer4f1.repetitioncounter = 0U;

    timer4f2.prescaler         = 5U;
    timer4f2.alignedmode       = TIMER_COUNTER_EDGE;
    timer4f2.counterdirection  = TIMER_COUNTER_UP;
    timer4f2.period            = 0U;
    timer4f2.clockdivision     = TIMER_CKDIV_DIV1;
    timer4f2.repetitioncounter = 0U;

    timer4cp.prescaler         = 5U;
    timer4cp.alignedmode       = TIMER_COUNTER_EDGE;
    timer4cp.counterdirection  = TIMER_COUNTER_UP;
    timer4cp.period            = 0U;
    timer4cp.clockdivision     = TIMER_CKDIV_DIV1;
    timer4cp.repetitioncounter = 0U;

    timer4rs.prescaler         = 5U;
    timer4rs.alignedmode       = TIMER_COUNTER_EDGE;
    timer4rs.counterdirection  = TIMER_COUNTER_UP;
    timer4rs.period            = 0U;
    timer4rs.clockdivision     = TIMER_CKDIV_DIV1;
    timer4rs.repetitioncounter = 0U;

    timer4sh.prescaler         = 119U;
    timer4sh.alignedmode       = TIMER_COUNTER_EDGE;
    timer4sh.counterdirection  = TIMER_COUNTER_UP;
    timer4sh.period            = 0U;
    timer4sh.clockdivision     = TIMER_CKDIV_DIV1;
    timer4sh.repetitioncounter = 0U;

    LOG_I("Hello TCD1209");

    rcu_periph_clock_enable(RCU_TIMER1);
    timer_init(TIMER1, &timer4sh);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_autoreload_value_config(TIMER1, 2500);
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, 1);
    timer_channel_output_state_config(TIMER1, TIMER_CH_1, ENABLE);
    timer_interrupt_disable(TIMER1, TIMER_INT_CH1);
    timer_enable(TIMER1);
    //rt_thread_mdelay(1);

    rcu_periph_clock_enable(RCU_TIMER3);
    timer_init(TIMER3, &timer4f1);
    timer_channel_output_mode_config(TIMER3, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_autoreload_value_config(TIMER3, 19);
    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_1, 10);
    timer_channel_output_state_config(TIMER3, TIMER_CH_1, ENABLE);
    timer_interrupt_disable(TIMER3, TIMER_INT_CH1);
    timer_enable(TIMER3);

    rcu_periph_clock_enable(RCU_TIMER2);
    timer_init(TIMER2, &timer4f2);
    timer_channel_output_mode_config(TIMER2, TIMER_CH_1, TIMER_OC_MODE_PWM1);
    timer_autoreload_value_config(TIMER2, 19);
    timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_1, 10);
    timer_channel_output_state_config(TIMER2, TIMER_CH_1, ENABLE);
    timer_interrupt_disable(TIMER2, TIMER_INT_CH1);
    timer_enable(TIMER2);

    rcu_periph_clock_enable(RCU_TIMER9);
    timer_init(TIMER9, &timer4cp);
    timer_channel_output_mode_config(TIMER9, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_autoreload_value_config(TIMER9, 19);
    timer_channel_output_pulse_value_config(TIMER9, TIMER_CH_0, 4);
    timer_channel_output_state_config(TIMER9, TIMER_CH_0, ENABLE);
    timer_interrupt_disable(TIMER9, TIMER_INT_CH0);
    timer_enable(TIMER9);

    rcu_periph_clock_enable(RCU_TIMER10);
    timer_init(TIMER10, &timer4rs);
    timer_channel_output_mode_config(TIMER10, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_autoreload_value_config(TIMER10, 19);
    timer_channel_output_pulse_value_config(TIMER10, TIMER_CH_0, 6);
    timer_channel_output_state_config(TIMER10, TIMER_CH_0, ENABLE);
    timer_interrupt_disable(TIMER10, TIMER_INT_CH0);
    timer_enable(TIMER10);

    return ret;
}
INIT_PREV_EXPORT(tcd1209_hw_init);

