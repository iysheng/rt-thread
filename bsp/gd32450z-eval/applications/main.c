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
#include <stdlib.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <drv_gpio.h>
#include "can_comm.h"
#include "ccd_main_base.h"

#define DBG_LVL               DBG_LOG
#define DBG_TAG               "app.MAIN"
#include <rtdbg.h>

#define HEART_LED_PIN    GET_PIN(A, 3)
#define TEST_CALIBRATE_PIN    GET_PIN(B, 15)

#define DBG_LED0_PIN     GET_PIN(B, 0)
#define DBG_LED1_PIN     GET_PIN(B, 1)
#define DBG_LED2_PIN     GET_PIN(B, 2)

extern int tc(int argc, char *argv[]);
extern void encoder_comm_backend_init(void *arg);
extern void screen_backend_entry(void * arg);
static rt_thread_t gs_encoder_backend, gs_screen_backend;
extern void display_calibrate_value(unsigned char * level, unsigned char len);
extern void display_check_value(unsigned char * level, unsigned char len);
extern int get_ccd_calibrate_info(unsigned char addr, unsigned char *value, unsigned char len);
extern int ccd_main_db_init(void);

int main(void)
{
    int ret = 0;
    unsigned char calibrate_info_buffer[6] = {0};

    ccd_main_db_init();
    rt_pin_mode(HEART_LED_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(TEST_CALIBRATE_PIN, PIN_MODE_INPUT);

    LOG_I("tc start.");
    tc(0, NULL);
    LOG_I("tc end.");
    if (!get_ccd_calibrate_info(0x01, calibrate_info_buffer, 6))
    {
        display_calibrate_value(calibrate_info_buffer, 6);
        LOG_I("Get calibrate info success");
    }
    else
    {
        LOG_E("Failed Get Calibrate info");
    }

    gs_encoder_backend = rt_thread_create("encoderB", encoder_comm_backend_init, RT_NULL, 0x1000, 5, 10);
    if (gs_encoder_backend)
    {
        ret = rt_thread_startup(gs_encoder_backend);
        if (ret)
        {
            LOG_E("Failed startup encoder backend thread, err=%d", ret);
        }
    }
    gs_screen_backend = rt_thread_create("screenB", screen_backend_entry, RT_NULL, 0x1000, 5, 10);
    if (gs_screen_backend)
    {
        ret = rt_thread_startup(gs_screen_backend);
        if (ret)
        {
            LOG_E("Failed startup screen backend thread, err=%d", ret);
        }
    }
    while(1)
    {
        rt_pin_write(HEART_LED_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(HEART_LED_PIN, PIN_LOW);
        rt_thread_mdelay(500);
        if (PIN_LOW == rt_pin_read(TEST_CALIBRATE_PIN))
        {
            /* 触发校准 */
            tc(0, NULL);
            if (!get_ccd_calibrate_info(0x01, calibrate_info_buffer, 6))
            {
                display_calibrate_value(calibrate_info_buffer, 6);
                LOG_I("Get calibrate info success");
            }
        }
    }

    return 0;
}
///////////
/*
 * 程序清单：这是一个 CAN 设备使用例程
 * 例程导出了 tc 命令到控制终端
 * 命令调用格式：tc can1
 * 命令解释：命令第二个参数是要使用的 CAN 设备名称，为空则使用默认的 CAN 设备
 * 程序功能：通过 CAN 设备发送一帧，并创建一个线程接收数据然后打印输出。
*/

#if 0
#include <rtthread.h>
#include "rtdevice.h"

#define CAN_DEV_NAME       "can0"      /* CAN 设备名称 */

static rt_device_t can_dev;            /* CAN 设备句柄 */


static void can_rx_thread(void *parameter)
{
    int i;
    rt_err_t res;
    struct rt_can_msg rxmsg = {0};

#ifdef RT_CAN_USING_HDR
    struct rt_can_filter_item items[5] =
    {
        RT_CAN_FILTER_ITEM_INIT(0x100, 0, 0, 1, 0x700, RT_NULL, RT_NULL), /* std,match ID:0x100~0x1ff，hdr 为 - 1，设置默认过滤表 */
        RT_CAN_FILTER_ITEM_INIT(0x300, 0, 0, 1, 0x700, RT_NULL, RT_NULL), /* std,match ID:0x300~0x3ff，hdr 为 - 1 */
        RT_CAN_FILTER_ITEM_INIT(0x211, 0, 0, 1, 0x7ff, RT_NULL, RT_NULL), /* std,match ID:0x211，hdr 为 - 1 */
        RT_CAN_FILTER_STD_INIT(0x486, RT_NULL, RT_NULL),                  /* std,match ID:0x486，hdr 为 - 1 */
        {0x555, 0, 0, 1, 0x7ff, 7,}                                       /* std,match ID:0x555，hdr 为 7，指定设置 7 号过滤表 */
    };
    struct rt_can_filter_config cfg = {5, 1, items}; /* 一共有 5 个过滤表 */
    /* 设置硬件过滤表 */
    res = rt_device_control(can_dev, RT_CAN_CMD_SET_FILTER, &cfg);
    RT_ASSERT(res == RT_EOK);
#endif
    while (1)
    {
        /* hdr 值为 - 1，表示直接从 uselist 链表读取数据 */
        rxmsg.hdr = -1;
        /* 阻塞等待接收信号量 */
        rt_sem_take(&rx_sem, rt_waiting_forever);
        /* 从 can 读取一帧数据 */
        rt_device_read(can_dev, 0, &rxmsg, sizeof(rxmsg));
        /* 打印数据 id 及内容 */
        rt_kprintf("id:%x", rxmsg.id);
        for (i = 0; i < 8; i++)
        {
            rt_kprintf("%2x", rxmsg.data[i]);
        }

        rt_kprintf("\n");
    }
}

#endif
int tc(int argc, char *argv[])
{
    unsigned char cmd;
    unsigned int cmd_args = 1;

    if (argc < 2)
    {
        set_ccd_calibrate(0x01, 3);
    }
    else if (argc == 2)
    {
        switch(atoi(argv[1]))
        {
            case 1:
                LOG_I("set ccd calibrate");
                set_ccd_calibrate(0x01, 3);
                break;
            case 2:
                LOG_I("set ccd check");
                set_ccd_check(0x01, 0x12345678);
                break;
        }
    }
    else if (argc == 3)
    {
        switch(atoi(argv[1]))
        {
            case 1:
                LOG_I("set ccd calibrate");
                set_ccd_calibrate(0x01, (unsigned char)atoi(argv[2]));
                break;
            case 2:
                LOG_I("set ccd check");
                set_ccd_check(0x01, (unsigned int)atoi(argv[2]));
                break;
        }
    }

    return 0;
}
/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(tc, can device sample);

