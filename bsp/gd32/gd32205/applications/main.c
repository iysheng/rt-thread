/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-09-21     iysheng      the first version
 */

#include <stdio.h>
#include <rtthread.h>
#include "rtdevice.h"

#define DBG_LVL               DBG_INFO
#define DBG_TAG               "app.MAIN"
#include <rtdbg.h>

/* HEART PIN is GPIOB_15 */
#define HEART_PIN    31
/* PWM PIN is GPIOB_5 */
#define PWM_PIN    21

static rt_thread_t gs_can_thread;
extern void can_backend_entry(void * arg);

int main(void)
{
    gs_can_thread = rt_thread_create("canBack", can_backend_entry, RT_NULL, 0x800, 5, 10);
    if (!gs_can_thread)
    {
        LOG_E("Failed create can backend thread.");
        return -1;
    }
    else if (RT_EOK != rt_thread_startup(gs_can_thread))
    {
        LOG_E("Failed startup can backend thread.");
        return -2;
    }

    rt_pin_mode(PWM_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(PWM_PIN, PIN_LOW);
    while(1)
    {
        rt_pin_write(HEART_PIN, PIN_LOW);
        rt_thread_mdelay(60000);
        rt_pin_write(HEART_PIN, PIN_HIGH);
        rt_thread_mdelay(1000);
    }

    return 0;
}
