/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-23     tyustli      first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"

/**
  * @brief 点灯程序
  * 
  * @param void: 
  * retval N/A.
  */
static void blink_test(void)
{
    static int led_status;

    if (!led_status)
    {
        gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, \
            GPIO_PIN_9);
    }
    gpio_bit_write(GPIOC, GPIO_PIN_9, (led_status++ % 2));
    rt_kprintf("hello china.\n");
    rt_thread_mdelay(500);
}

int main(int argc, char *argv[]) {

    while(1)
    {
        blink_test();
    }

    return RT_EOK;
}

/******************** end of file *******************/

