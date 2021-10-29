/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-27     iysheng      first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>


/*
 *  LED_PIN -> PA6
 * */

int main(void)
{

    /* TODO set pa6 to 0 */

    *(volatile uint32_t *)0x40021018 |= 0x1 << 2;
    *(volatile rt_uint32_t *)0x40010800 = 0x43444444;
    *(volatile rt_uint32_t *)0x4001080c &= (~(1 << 6));

    while(1)
    {

    *(volatile rt_uint32_t *)0x4001080c &= 0xbf;

    rt_thread_mdelay(1000);
    *(volatile rt_uint32_t *)0x4001080c |= 0x40;

    rt_thread_mdelay(1000);
    }

    return RT_EOK;
}
