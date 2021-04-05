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

#if 0
/* 发送端设备 */
#define INFRA_SEND_DEVICE
#endif

#define INFRA_SINGAL_PIN     6

static rt_device_t s_seg_dev_ptr;
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
        s_seg_dev_ptr = rt_device_find("redSeg");
        if (s_seg_dev_ptr)
        {
            ret = rt_device_open(s_seg_dev_ptr, RT_DEVICE_OFLAG_WRONLY);
            rt_kprintf("ret=%d\n", ret);
        }
    }
    rt_kprintf("hello china. %d\n", led_status++);
    gpio_bit_write(GPIOC, GPIO_PIN_9, led_status % 2);
#if 0
    if (s_seg_dev_ptr)
    {
        rt_device_write(s_seg_dev_ptr, 0, &led_status, 1);
    }
#endif
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
	static unsigned char pin_level;

    if (!s_pin_inited_done)
    {
#ifdef INFRA_SEND_DEVICE
        rt_pin_mode(INFRA_SINGAL_PIN, PIN_MODE_OUTPUT);
#else
        rt_pin_mode(INFRA_SINGAL_PIN, PIN_MODE_INPUT);
#endif
    }

#ifdef INFRA_SEND_DEVICE
	rt_pin_write(INFRA_SINGAL_PIN, 0 % 2);
	rt_kprintf("set infra signal is:%u\n", pin_level);
#else
	pin_level = (unsigned char)rt_pin_read(INFRA_SINGAL_PIN);
	rt_kprintf("pin_level of infra signal is:%u\n", pin_level);

    rt_device_write(s_seg_dev_ptr, 0, &pin_level, 1);
#endif
}

int main(int argc, char *argv[]) {

    while(1)
    {
        blink_test();
        test_infra_pin_level();
        /* TODO read pin level */
        rt_thread_mdelay(500);
    }

    return RT_EOK;
}

/******************** end of file *******************/

