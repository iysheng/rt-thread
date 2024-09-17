/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 * Copyright (c) 2019-2020, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-10-24     Magicoe      first version
 * 2020-01-10     Kevin/Karl   Add PS demo
 * 2020-09-21     supperthomas fix the main.c
 *
 */

#include <rtdevice.h>
#include <rtthread.h>
#include "drv_pin.h"

#define LEDB_PIN        ((1*32)+2)
#define BUTTON_PIN      ((0*32)+23)

static void sw_pin_cb(void *args);
extern void usb_vcom_main(void);

#define MK_THREAD_ASSETS(name, stack_size) \
	static rt_uint8_t name##_stack[stack_size]; \
	static struct rt_thread gs_##name##_thread; \
	extern void name##_thread_entry(void *);

#define CREATE_THREAD(name, parg, pri, tick) \
	rt_thread_init(&gs_##name##_thread, #name, name##_thread_entry, parg, \
		name##_stack, sizeof(name##_stack), pri, tick)

#define DEFINE_THREAD_ENTRY(name) \
		void name##_thread_entry(void * parg)

#define MAKE_THREAD_START(name) \
	rt_thread_startup(&gs_##name##_thread)

MK_THREAD_ASSETS(bmp, 0X1000)

int main(void)
{
	rt_err_t result;
#if defined(__CC_ARM)
    rt_kprintf("using armcc, version: %d\n", __ARMCC_VERSION);
#elif defined(__clang__)
    rt_kprintf("using armclang, version: %d\n", __ARMCC_VERSION);
#elif defined(__ICCARM__)
    rt_kprintf("using iccarm, version: %d\n", __VER__);
#elif defined(__GNUC__)
    rt_kprintf("using gcc, version: %d.%d\n", __GNUC__, __GNUC_MINOR__);
#endif

    rt_pin_mode(LEDB_PIN, PIN_MODE_OUTPUT);  /* Set GPIO as Output */

    rt_pin_mode(BUTTON_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(BUTTON_PIN, PIN_IRQ_MODE_FALLING, sw_pin_cb, RT_NULL);
    rt_pin_irq_enable(BUTTON_PIN, 1);

    rt_kprintf("MCXN947 HelloWorld\r\n");
    usb_vcom_main();

	  result = CREATE_THREAD(bmp, RT_NULL, RT_THREAD_PRIORITY_MAX / 5, 20);
    RT_ASSERT(result == RT_EOK);
    MAKE_THREAD_START(bmp);
	
#ifdef RT_USING_SDIO
    rt_thread_mdelay(2000);
    if (dfs_mount("sd", "/", "elm", 0, NULL) == 0)
    {
        rt_kprintf("sd mounted to /\n");
    }
    else
    {
        rt_kprintf("sd mount to / failed\n");
    }
#endif

    while (1)
    {
        rt_pin_write(LEDB_PIN, PIN_HIGH);    /* Set GPIO output 1 */
        rt_thread_mdelay(500);               /* Delay 500mS */
        rt_pin_write(LEDB_PIN, PIN_LOW);     /* Set GPIO output 0 */
        rt_thread_mdelay(500);               /* Delay 500mS */
    }
}

static void sw_pin_cb(void *args)
{
    rt_kprintf("sw pressed\r\n");
}

// end file
