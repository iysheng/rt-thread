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
#define RPA12  GET_PIN(A, 12)
    rt_pin_mode(RPA12, PIN_MODE_OUTPUT);
#endif

    while (1)
    {
#if 0
        rt_pin_write(RPA12, PIN_LOW);
        rt_thread_mdelay(500);
        rt_pin_write(RPA12, PIN_HIGH);
#endif
        rt_thread_mdelay(500);
    }

    return 0;
}


/*
 * File      : can_thread.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author           Notes
 * 2020-10-10     Yangyongsheng    first implementation
 */
#include <rtthread.h>
#include <board.h>
#include "drv_can.h"

#define DBG_TAG               "thread.CAN"
#define DBG_LVL               LOG_LVL_DBG
#include <rtdbg.h>

#define CAN_DEV_NAME       "can0"      /* CAN 设备名称 */

static rt_device_t g_can_dev;          /* CAN 设备句柄 */

int can_thread_entry(void)
{
    rt_err_t ret;
    size_t size;
    struct rt_can_status status;
    struct rt_can_msg msg = {0};

    LOG_D("hello %s\n", __func__);
    /* 查找 CAN 设备 */
    g_can_dev = rt_device_find(CAN_DEV_NAME);

    if (!g_can_dev)
    {
        LOG_E("no found %s\n", CAN_DEV_NAME);
        return -ENODEV;
    }

    ret = rt_device_open(g_can_dev, \
        RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX);
#define CAN_BAUD CAN500kBaud
    /* 设置 CAN 通信的波特率为 100kbit/s*/
    ret = rt_device_control(g_can_dev, RT_CAN_CMD_SET_BAUD, \
        (void *)CAN_BAUD);
	LOG_I("baud=%d\n", CAN_BAUD);
#if 0
    ret = rt_device_control(g_can_dev, RT_CAN_CMD_SET_MODE, \
        (void *)RT_CAN_MODE_LOOPBACK);
#else
    ret = rt_device_control(g_can_dev, RT_CAN_CMD_SET_MODE, \
        (void *)RT_CAN_MODE_NORMAL);
#endif
    /* 只有使能滤波才可以正常收发数据 */
    ret = rt_device_control(g_can_dev, RT_CAN_CMD_SET_FILTER, NULL);
#if 0
    /* 获取 CAN 总线设备的状态 */
    res = rt_device_control(can_dev, RT_CAN_CMD_GET_STATUS, &status);
    rcu_periph_clock_disable(RCU_USART0);
#endif


int i = 0;
    msg.id = 0x78;              /* ID 为 0x78 */
    msg.ide = RT_CAN_STDID;     /* 标准格式 */
    msg.rtr = RT_CAN_DTR;       /* 数据帧 */
    msg.len = 8;                /* 数据长度为 8 */
    /* 待发送的 8 字节数据 */
    msg.data[0] = 0x00;
    msg.data[1] = 0x11;
    msg.data[2] = 0x22;
    msg.data[3] = 0x33;
    msg.data[4] = 0x44;
    msg.data[5] = 0x55;
    msg.data[6] = 0x66;
    msg.data[7] = 0x77;
    /* 发送一帧 CAN 数据 */
    size = rt_device_write(g_can_dev, 0, &msg, sizeof(msg));
	LOG_I("send %d time.", i);
	rt_thread_mdelay(500);
    while (1)
    {
        rt_memset(&msg, 0, sizeof msg);
        size = rt_device_read(g_can_dev, 0, &msg, sizeof(msg));
        if (size)
        {
            LOG_I("id=%x ide=%d time=%d", msg.id, msg.ide, i++);
            LOG_HEX("can_fram", 8, msg.data, msg.len);
        }
    }

	while (1)
	{
    msg.id = 0x78;              /* ID 为 0x78 */
    msg.ide = RT_CAN_STDID;     /* 标准格式 */
    msg.rtr = RT_CAN_DTR;       /* 数据帧 */
    msg.len = 8;                /* 数据长度为 8 */
    /* 待发送的 8 字节数据 */
    msg.data[0] = 0x00;
    msg.data[1] = 0x11;
    msg.data[2] = 0x22;
    msg.data[3] = 0x33;
    msg.data[4] = 0x44;
    msg.data[5] = 0x55;
    msg.data[6] = 0x66;
    msg.data[7] = 0x77;
    /* 发送一帧 CAN 数据 */
    size = rt_device_write(g_can_dev, 0, &msg, sizeof(msg));
		LOG_I("send %d time.", i);
	rt_thread_mdelay(500);
	if (i++ > 100)
	{
		break;
	}
	}

    return 0;
}

INIT_APP_EXPORT(can_thread_entry);
