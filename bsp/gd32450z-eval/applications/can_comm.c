/******************************************************************************
* File:             can_comm.c
*
* Author:           iysheng@163.com
* Created:          06/12/21
*                   CAN 通讯协议
*****************************************************************************/

#include <rtthread.h>
#include <rtdevice.h>

#define DBG_LVL               DBG_LOG
#define DBG_TAG               "com.CAN"
#include <rtdbg.h>

#define CAN_DEV_NAME       "can0"      /* CAN 设备名称 */
#define CAN_BAUD           CAN1MBaud   /* 使用 1Mb 的速率通讯 */
static struct rt_semaphore gs_can_rx_sem;
static rt_device_t gs_can_dev;

/**
  * @brief 控制 CCD 设备进行标定
  *
  * @param rt_device_t dev:
  * @param unsigned char addr:
  * @param unsigned char times:
  * retval errno/Linux.
  */
static int _set_ccd_calibrate(rt_device_t dev, unsigned char addr, unsigned char times)
{
    int ret;
    struct rt_can_msg msg = {0};

    RT_ASSERT(dev);
    msg.id = addr;
    msg.ide = RT_CAN_STDID;     /* 标准格式 */
    msg.rtr = RT_CAN_DTR;       /* 数据帧 */
    msg.len = 8;                /* 数据长度为 8 */
    /* 发送校验次数 */
    msg.data[0] = times;
    /* 异或校验结果 */
    msg.data[7] = times;

    if (sizeof(msg) == rt_device_write(dev, 0, &msg, sizeof(msg)))
    {
        /* TODO 检测返回值 */
    }

    return ret;
}

/**
  * @brief 控制 CCD 设备进行检测
  *
  * @param rt_device_t dev:
  * @param unsigned char addr:
  * @param unsigned int id: 检测对应的 ID 信息
  * retval errno/Linux.
  */
static int _set_ccd_check(rt_device_t dev, unsigned char addr, unsigned int id)
{
    int ret;
    struct rt_can_msg msg = {0};

    RT_ASSERT(dev);
    msg.id = addr;
    msg.ide = RT_CAN_STDID;     /* 标准格式 */
    msg.rtr = RT_CAN_DTR;       /* 数据帧 */
    msg.len = 8;                /* 数据长度为 8 */
    /* 发送校验次数 */
    msg.data[0] = id >> 24 & 0xff;
    msg.data[1] = id >> 16 & 0xff;
    msg.data[2] = id >> 8 & 0xff;
    msg.data[3] = id & 0xff;
    /* 异或校验结果 */
    msg.data[7] = msg.data[0] && msg.data[1] && msg.data[2] && msg.data[3];
    if (sizeof(msg) == rt_device_write(dev, 0, &msg, sizeof(msg)))
    {
        /* TODO 检测返回值 */
    }

    return ret;
}

/**
  * @brief 控制 CCD 进行标定
  * 
  * @param unsigned char addr: 
  * @param unsigned char times: 
  * retval errno/Linux.
  */
int set_ccd_calibrate(unsigned char addr, unsigned char times)
{
    return _set_ccd_calibrate(gs_can_dev, addr, times);
}

/**
  * @brief 控制 CCD 进行检测并返回检测结果
  * 
  * @param unsigned char addr: 
  * @param unsigned int id: 
  * retval errno/Linux.
  */
int set_ccd_check(unsigned char addr, unsigned int id)
{
    return _set_ccd_check(gs_can_dev, addr,id);
}

/* 接收数据回调函数 */
static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size)
{
    /* CAN 接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    rt_sem_release(&gs_can_rx_sem);

    return RT_EOK;
}

static int can_comm_init(void)
{
    int ret;
    gs_can_dev = rt_device_find(CAN_DEV_NAME);
    if (!gs_can_dev)
    {
        LOG_E("find %s failed!\n", CAN_DEV_NAME);
        return RT_ERROR;
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
    /* 只有使能滤波才可以正常收发数据 */
    ret = rt_device_control(gs_can_dev, RT_CAN_CMD_SET_FILTER, NULL);
    if (ret)
    {
        LOG_E("Init can commiuncation@%d failed.", CAN_BAUD);
    }
    else
    {
        LOG_I("Init can commiuncation@%d success.", CAN_BAUD);
    }

    return ret;
}
INIT_APP_EXPORT(can_comm_init);
