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
#define INFRA_CONTROL_PIN    7

#define INFRA_RELAY0_PIN     4
#define INFRA_RELAY1_PIN     2

static int _set_pin_channel(int channel)
{
    if (channel > 15)
    {
        rt_kprintf("invalid channel");
        return -E2BIG;
    }
    else
    {
        rt_kprintf("set channel=%d\n", channel);
    }

    /* BIT[12:15] */
    GPIO_OCTL(GPIOB) &= ~(uint32_t)0xf000;
    GPIO_OCTL(GPIOB) |= (uint32_t)(channel << 12);
    return 0;
}

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
#ifdef INFRA_SEND_DEVICE
        rt_pin_mode(INFRA_SINGAL_PIN, PIN_MODE_OUTPUT);
#else
        rt_pin_mode(INFRA_SINGAL_PIN, PIN_MODE_INPUT);
#endif
        /* enable chip select */
        rt_pin_mode(INFRA_CONTROL_PIN, PIN_MODE_OUTPUT);
        rt_pin_write(INFRA_CONTROL_PIN, PIN_LOW);

        rt_pin_mode(INFRA_RELAY0_PIN, PIN_MODE_OUTPUT);
        rt_pin_write(INFRA_RELAY0_PIN, PIN_LOW);
        rt_pin_mode(INFRA_RELAY1_PIN, PIN_MODE_OUTPUT);
        rt_pin_write(INFRA_RELAY1_PIN, PIN_LOW);
        /* set addr pin mode output */
        gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);
        gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
        gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_14);
        gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_15);
        s_pin_inited_done = 1;
    }

	_set_pin_channel(pin_channel);
#ifdef INFRA_SEND_DEVICE
    rt_pin_write(INFRA_CONTROL_PIN, PIN_HIGH);
	//rt_pin_write(INFRA_SINGAL_PIN, 0);
	rt_kprintf("set infra signal channel is:%u\n", pin_channel);
#else
	pin_level = (unsigned char)rt_pin_read(INFRA_SINGAL_PIN);
	rt_kprintf("pin_level of infra signal is:%u\n", pin_level);
	if (pin_level == 1)
	{
        rt_device_write(s_seg_dev_ptr, 0, &pin_channel, 1);
        rt_pin_write(INFRA_RELAY0_PIN, PIN_HIGH);
        rt_pin_write(INFRA_RELAY1_PIN, PIN_HIGH);
        scan_cycle = 1;
	}
#endif
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
}

int main(int argc, char *argv[]) {

    while(1)
    {
        blink_test();
        test_infra_pin_level();
        /* TODO read pin level */
#ifdef INFRA_SEND_DEVICE
        rt_thread_mdelay(50);
#else
        rt_thread_mdelay(5);
#endif
    }

    return RT_EOK;
}

/******************** end of file *******************/

