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
#include "drivers/drv_infra.h"

#define DRV_DEBUG
#define DRV_TAG    "drv.infra"
#include <rtdbg.h>

#define INFRA_RELAY0_PIN     4
#define INFRA_RELAY1_PIN     2

#ifndef INFRA_SEND_DEVICE
static rt_device_t gs_seg_dev_ptr;
#endif
/**
  * @brief 点灯程序
  *
  * @param void:
  * retval N/A.
  */
static void blink_test(void)
{
    static unsigned char led_status;
    int ret;

    if (!led_status)
    {
        gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, \
            GPIO_PIN_9);
    }
    gpio_bit_write(GPIOC, GPIO_PIN_9, led_status % 2);
}

/*
 * test infra pin level
 * */
/**
  * @brief 测试红外数据读取
  *
  * @param void:
  * retval .
  */
static void test_infra_pin_level(void)
{
    static int s_pin_inited_done;
    static unsigned char pin_level, pin_channel, scan_cycle;

    if (!s_pin_inited_done)
    {
        s_pin_inited_done = 1;
    }

#if 0 /* 需要恢复 */
	if (pin_level == 1)
	{
        rt_device_write(gs_seg_dev_ptr, 0, &pin_channel, 1);
        rt_pin_write(INFRA_RELAY0_PIN, PIN_HIGH);
        rt_pin_write(INFRA_RELAY1_PIN, PIN_HIGH);
        scan_cycle = 1;
	}
    rt_device_write(gs_seg_dev_ptr, 0, &pin_level, 1);
#endif
#if 0
    pin_channel++;
	if (pin_channel == 16)
	{
		pin_channel = 0;
		if (!scan_cycle)
		{
			rt_pin_write(INFRA_RELAY0_PIN, PIN_LOW);
			rt_pin_write(INFRA_RELAY1_PIN, PIN_LOW);
		}
        scan_cycle = 0;
	}
#endif
}

int main(int argc, char *argv[])
{
    rt_device_t infra_dev_ptr = RT_NULL;
    int ret;
    unsigned char channel;

    infra_dev_ptr = rt_device_find("Infra");
    if (!infra_dev_ptr)
    {
        LOG_E("no found such device");
        return -ENODEV;
    }
    ret = rt_device_open(infra_dev_ptr, RT_DEVICE_FLAG_RDWR);
    if (ret)
    {
        LOG_E("failed open device err=%d.", ret);
        return ret;
    }
#ifndef INFRA_SEND_DEVICE
    gs_seg_dev_ptr = rt_device_find("redSeg");
    if (!gs_seg_dev_ptr)
    {
        LOG_E("no found such device");
        return -ENODEV;
    }

    ret = rt_device_open(gs_seg_dev_ptr, RT_DEVICE_OFLAG_WRONLY);
    if (ret)
    {
        LOG_E("failed open device err=%d.", ret);
        return ret;
    }

    rt_pin_mode(INFRA_RELAY0_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(INFRA_RELAY0_PIN, PIN_LOW);
    rt_pin_mode(INFRA_RELAY1_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(INFRA_RELAY1_PIN, PIN_LOW);
#else
    /* wait for recv */
    rt_thread_mdelay(3000);
    timer_enable(TIMER2);
#endif

    while(1)
    {
        blink_test();
        rt_thread_mdelay(50);
#ifndef INFRA_SEND_DEVICE
        /* read channel num */
        if (!rt_device_control(infra_dev_ptr, RED_INFRA_GET_CHANNEL, &channel))
        {
            rt_pin_write(INFRA_RELAY0_PIN, PIN_HIGH);
            rt_device_write(gs_seg_dev_ptr, 0, &channel, sizeof channel);
        }
        else
        {
            rt_pin_write(INFRA_RELAY0_PIN, PIN_LOW);
        }
#endif
    }

    return RT_EOK;
}

/******************** end of file *******************/

