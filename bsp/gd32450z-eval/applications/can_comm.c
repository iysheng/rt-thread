/******************************************************************************
* File:             can_comm.c
*
* Author:           iysheng@163.com
* Created:          06/12/21
*                   CAN 通讯协议
*****************************************************************************/

#include <rtthread.h>
#include <rtdevice.h>

enum {
    CCD_CHECK = 0x01,
    CCD_CHECK_RESPON = 0x02,
    CCD_CALIBRATE = 0x03,
    CCD_CALIBRATE_RESPON = 0x04,
} can_comm_cmd_E;

#define DBG_LVL               DBG_INFO
#define DBG_TAG               "com.CAN"
#include <rtdbg.h>

#define CAN_DEV_NAME       "can0"               /* CAN 设备名称 */
#define CAN_BAUD           CAN1MBaud            /* 使用 1Mb 的速率通讯 */
#define CAN_RECV_MAX_DELAY (RT_TICK_PER_SECOND / 50)   /* 最长等待时间为 1s */

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
    int ret = -1;
    struct rt_can_msg msg = {0};
    struct rt_can_msg rx_msg = {0};

    RT_ASSERT(dev);
    msg.id = addr;
    msg.ide = RT_CAN_STDID;     /* 标准格式 */
    msg.rtr = RT_CAN_DTR;       /* 数据帧 */
    msg.len = 8;                /* 数据长度为 8 */
    /* 发送校验次数 */
    msg.data[0] = CCD_CALIBRATE;
    msg.data[1] = times;
    /* 异或校验结果 */
    msg.data[7] = times ^ CCD_CALIBRATE;

    if (sizeof(msg) == rt_device_write(dev, 0, &msg, sizeof(msg)))
    {
        /* 阻塞等待接收信号量 */
        ret = rt_sem_take(&gs_can_rx_sem, CAN_RECV_MAX_DELAY);
        if (ret != RT_EOK)
        {
            LOG_D("Failed take gs can sem. err=%d", ret);
            return ret;
        }
        /* 从 can 读取一帧数据 */
        ret = rt_device_read(gs_can_dev, 0, &rx_msg, sizeof(rx_msg));
        if (ret != sizeof(rx_msg))
        {
            LOG_D("Failed get respon of calibrate");
            return -EINVAL;
        }
        else if ((unsigned char)rx_msg.id != addr)
        {
            LOG_D("Respon no match addr");
            return -ENODEV;
        }
        else if (rx_msg.data[0] != CCD_CALIBRATE_RESPON || rx_msg.data[1] != times)
        {
            LOG_D("Respon no match calibrate respon or times");
            return -EACCES;
        }
        else
        {
            ret = rx_msg.data[1];
        }
        /* 打印数据 id 及内容 */
        LOG_D("id:%x", rx_msg.id);
        LOG_HEX("ccdCal", 8, rx_msg.data, 8);
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
    struct rt_can_msg rx_msg = {0};

    RT_ASSERT(dev);
    msg.id = addr;
    msg.ide = RT_CAN_STDID;     /* 标准格式 */
    msg.rtr = RT_CAN_DTR;       /* 数据帧 */
    msg.len = 8;                /* 数据长度为 8 */
    /* 发送校验次数 */
    msg.data[0] = CCD_CHECK;
    msg.data[1] = id >> 24 & 0xff;
    msg.data[2] = id >> 16 & 0xff;
    msg.data[3] = id >> 8 & 0xff;
    msg.data[4] = id & 0xff;
    /* 异或校验结果 */
    msg.data[7] = CCD_CHECK ^ msg.data[1] ^ msg.data[2] ^ msg.data[3] ^ msg.data[4];
    if (sizeof(msg) == rt_device_write(dev, 0, &msg, sizeof(msg)))
    {
        /* 阻塞等待接收信号量 */
        ret = rt_sem_take(&gs_can_rx_sem, CAN_RECV_MAX_DELAY);
        if (ret != RT_EOK)
        {
            LOG_D("Failed take gs can sem. err=%d", ret);
            return ret;
        }
        /* 从 can 读取一帧数据 */
        ret = rt_device_read(gs_can_dev, 0, &rx_msg, sizeof(rx_msg));
        if (ret != sizeof(rx_msg))
        {
            LOG_D("Failed get respon of check");
            return -EINVAL;
        }
        else if ((unsigned char)rx_msg.id != addr)
        {
            LOG_D("Respon no match addr");
            return -ENODEV;
        }
        else if ((CCD_CHECK_RESPON != rx_msg.data[0]) || (rx_msg.data[1] << 24 | rx_msg.data[2] << 16 |
            rx_msg.data[3] << 8 | rx_msg.data[4] != id))
        {
            LOG_D("Respon no check cmd or id");
            return -EACCES;
        }
        else
        {
            ret = rx_msg.data[5];
        }
        /* 打印数据 id 及内容 */
        LOG_D("id:%x", rx_msg.id);
        LOG_HEX("ccdCheck", 8, rx_msg.data, 8);
    }

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
int set_ccd_calibrate(unsigned char addr, unsigned char times)
{
    return _set_ccd_calibrate(gs_can_dev, addr, times);
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
        LOG_D("find %s failed!\n", CAN_DEV_NAME);
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
    rt_device_set_rx_indicate(gs_can_dev, can_rx_call);
    /* 只有使能滤波才可以正常接收数据 */
    ret = rt_device_control(gs_can_dev, RT_CAN_CMD_SET_FILTER, NULL);
    if (ret)
    {
        LOG_D("Init can commiuncation@%d failed.", CAN_BAUD);
    }
    else
    {
        LOG_D("Init can commiuncation@%d success.", CAN_BAUD);
    }

    return ret;
}
INIT_APP_EXPORT(can_comm_init);
