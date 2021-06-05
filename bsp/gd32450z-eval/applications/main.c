/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-05-18     tanek        first implementation
 */

#include <stdio.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <drv_gpio.h>

#define HEART_LED_PIN    GET_PIN(A, 3)

#define DBG_LED0_PIN     GET_PIN(B, 0)
#define DBG_LED1_PIN     GET_PIN(B, 1)
#define DBG_LED2_PIN     GET_PIN(B, 2)

int main(void)
{
    rt_pin_mode(HEART_LED_PIN, PIN_MODE_OUTPUT);

    while(1)
    {
        rt_pin_write(HEART_LED_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(HEART_LED_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }

    return 0;
}
