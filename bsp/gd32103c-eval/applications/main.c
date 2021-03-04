/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2021-01-04     iysheng           first version
 */

#include <board.h>
#include <drivers/adc.h>
#include <rtdbg.h>
#include <stdio.h>
#include <rtthread.h>
#include "board.h"

int main(void)
{
#if 0
#define SH  GET_PIN(A, 7)
#define FM  GET_PIN(A, 1)
#define ICG  GET_PIN(B, 8)
    rt_pin_mode(SH, PIN_MODE_OUTPUT);
    rt_pin_mode(FM, PIN_MODE_OUTPUT);
    rt_pin_mode(ICG, PIN_MODE_OUTPUT);
#endif

    while (1)
    {
#if 0
        rt_pin_write(SH, PIN_LOW);
        rt_pin_write(FM, PIN_LOW);
        rt_pin_write(ICG, PIN_LOW);
#endif
        rt_thread_mdelay(1000);
    }

    return 0;
}
