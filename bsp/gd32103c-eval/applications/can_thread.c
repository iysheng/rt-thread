/******************************************************************************
* File:             can_thread.c
*
* Author:           iysheng@163.com
*****************************************************************************/

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <drivers/can.h>

enum {
    CCD_CHECK = 0x01,
    CCD_CHECK_RESPON = 0x02,
    CCD_CALIBRATE = 0x03,
    CCD_CALIBRATE_RESPON = 0x04,
} can_comm_cmd_E;

#define DBG_LVL               DBG_LOG
#define DBG_TAG               "thread.CAN"
#include <rtdbg.h>

#define REMOTE_CCD_MAIN_ADDR  0x01
#define CAN_DEV_NAME       "can0"               /* CAN 设备名称 */
#define CAN_BAUD           CAN1MBaud            /* 使用 1Mb 的速率通讯 */
#define CAN_RECV_MAX_DELAY (RT_TICK_PER_SECOND / 50)   /* 最长等待时间为 1s */

static struct rt_semaphore gs_can_rx_sem;
static rt_device_t gs_can_dev;

extern int set_tcd1304_device_marktimes(int data);
extern int set_tcd1304_device_data(int data);
extern int get_ccd_check_ans(void);
/**
  * @brief 控制 CCD 设备进行标定
  *
  * @param rt_device_t dev:
  * @param rt_can_msg *msg: CAN 数据帧
  * retval errno/Linux.
  */
static int _set_ccd_calibrate(rt_device_t dev, rt_can_msg_t msg)
{
    int ret = -1;

    RT_ASSERT(dev);
    /* 限制最大校准次数为 10 */
    if (msg->data[1] < 10)
    {
        /* TODO 采样标定 */
        LOG_I("times=%d", msg->data[1]);
        set_tcd1304_device_marktimes(msg->data[1]);
        NVIC_EnableIRQ(TIMER3_IRQn);
        /* 设置标定成功 */
        msg->data[2] = 0;
        LOG_HEX("ccdCal", 8, msg->data, 8);
        ret = 0;
    }

    return ret;
}

/**
  * @brief 控制 CCD 设备进行检测
  *
  * @param rt_device_t dev:
  * @param struct rt_can_msg *msg: CAN 数据帧
  * retval errno/Linux.
  */
static int _set_ccd_check(rt_device_t dev, rt_can_msg_t msg)
{
    int ret;

    RT_ASSERT(dev);
    /* TODO 采样标定 */
    set_tcd1304_device_data(1);
    NVIC_EnableIRQ(TIMER3_IRQn);
#if 0
    /* test code need delete  */
    static int times;
    ret = times++ % 2;
#else
    ret = get_ccd_check_ans();
    LOG_I("<<<<<<< ret=%d.", ret);
#endif
    /* 设置检测结果 */
    msg->data[5] = (unsigned char)ret;
    LOG_HEX("ccdCheck", 8, msg->data, 8);

    return ret;
}

/**
  * @brief 控制 CCD 进行标定
  *
  * @param unsigned char addr:
  * @param unsigned char times:
  * retval errno/Linux.
  *      0 表示校准成功
  */
int set_ccd_calibrate(rt_can_msg_t msg)
{
    return _set_ccd_calibrate(gs_can_dev, msg);
}

/**
  * @brief 控制 CCD 进行检测并返回检测结果
  *
  * @param unsigned char addr:
  * param unsigned int id:
  * retval errno/Linux.
  *     0 表示匹配
  *     1 表示不匹配
  */
int set_ccd_check(rt_can_msg_t msg)
{
    return _set_ccd_check(gs_can_dev, msg);
}

/* 接收数据回调函数 */
static rt_err_t can_rx_call(rt_device_t dev, rt_size_t ret)
{
    /* CAN 接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    rt_sem_release(&gs_can_rx_sem);

    return RT_EOK;
}

void can_backend_entry(void * arg)
{
    int ret;
    struct rt_can_msg msg = {0};
    gs_can_dev = rt_device_find(CAN_DEV_NAME);

    LOG_I("hello can");
    if (!gs_can_dev)
    {
        LOG_D("Find %s failed!\n", CAN_DEV_NAME);
        return;
    }

    /* 初始化 CAN 接收信号量 */
    rt_sem_init(&gs_can_rx_sem, "can_rx_sem", 0, RT_IPC_FLAG_FIFO);

    /* 以中断接收及发送方式打开 CAN 设备 */
    ret = rt_device_open(gs_can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    RT_ASSERT(ret == RT_EOK);
    /* 设置 CAN 通信的波特率为 1Mbit/s*/
    ret = rt_device_control(gs_can_dev, RT_CAN_CMD_SET_BAUD, \
        (void *)CAN_BAUD);
#if 0
    ret = rt_device_control(gs_can_dev, RT_CAN_CMD_SET_MODE, \
        (void *)RT_CAN_MODE_LOOPBACK);
#else
    ret = rt_device_control(gs_can_dev, RT_CAN_CMD_SET_MODE, \
        (void *)RT_CAN_MODE_NORMAL);
#endif
    rt_device_set_rx_indicate(gs_can_dev, can_rx_call);
    /* 只有使能滤波才可以正常接收数据 */
#if 1
    ret = rt_device_control(gs_can_dev, RT_CAN_CMD_SET_FILTER, NULL);
#else
    struct rt_can_filter_item items[1] = {
        RT_CAN_FILTER_STD_INIT(0x01)
    };
    struct rt_can_filter_config cfg = {1, 1, items}; /* 一共有 5 个过滤表 */
    /* 设置硬件过滤表 */
    ret = rt_device_control(gs_can_dev, RT_CAN_CMD_SET_FILTER, &cfg);
#endif
    if (ret)
    {
        LOG_D("Init can commiuncation@%d failed.", CAN_BAUD);
        return;
    }
    else
    {
        LOG_D("Init can commiuncation@%d success.", CAN_BAUD);
    }

    msg.id = 0x01;              /* ID 为 0x01 */
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
    ret = rt_device_write(gs_can_dev, 0, &msg, sizeof(msg));

    while (1)
    {
        rt_memset(&msg, 0, sizeof msg);
        ret = rt_device_read(gs_can_dev, 0, &msg, sizeof(msg));
        if (ret)
        {
            LOG_I("id=%x ide=%d", msg.id, msg.ide);
            LOG_HEX("can_fram", 8, msg.data, msg.len);
            switch(msg.data[0])
            {
                case CCD_CHECK:
                    /* TODO calibrate */
                    set_ccd_check(&msg);
                    msg.id = REMOTE_CCD_MAIN_ADDR;
                    msg.data[0] = CCD_CHECK_RESPON;
                    /* TODO respon to remote */
                    rt_device_write(gs_can_dev, 0, &msg, sizeof(msg));
                    break;
                case CCD_CALIBRATE:
                    /* TODO check wether match */
                    set_ccd_calibrate(&msg);
                    msg.id = REMOTE_CCD_MAIN_ADDR;
                    msg.data[0] = CCD_CALIBRATE_RESPON;
                    /* TODO respon to remote */
                    rt_device_write(gs_can_dev, 0, &msg, sizeof(msg));
                    break;
                default:
                    LOG_E("Invalid cmd:%x", msg.data[0]);
            }
        }
        rt_thread_mdelay(100);
    }
}
