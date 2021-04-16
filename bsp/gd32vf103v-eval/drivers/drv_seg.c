/******************************************************************************
* File:             drv_seg.c
*
* Author:           iysheng@163.com  
* Created:          04/04/21 
* Description:      数码管驱动
*****************************************************************************/

#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"

static struct rt_device g_seg_device;
#define SEG8_MAP_COUNTS    17

static unsigned char gs_seg8_data_map[SEG8_MAP_COUNTS] ={
    0x3f,
    0x30,
    0x5b,
    0x4f,
    0x66,
    0x6d,
    0x7d,
    0x07,
    0x7f,
    0x6f,
    0x77,
    0x7c,
    0x39,
    0x5e,
    0x79,
    0x71,
    0x80,
};


static rt_size_t seg_dev_write  (rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    unsigned char data = *(unsigned char *)buffer;
    /* TODO check data valid */
    GPIO_OCTL(GPIOC) &= ~(uint32_t) 0xff;
    GPIO_OCTL(GPIOC) |= (uint32_t) gs_seg8_data_map[data % SEG8_MAP_COUNTS];

    return size;
}

static rt_err_t  seg_dev_init   (rt_device_t dev)
{
    rcu_periph_clock_enable(RCU_GPIOC);

    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5);
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);

    return 0;
}

#ifdef RT_USING_DEVICE_OPS
const struct rt_device_ops g_seg_device_ops = {
    .init = seg_dev_init,
    .write = seg_dev_write,
};
#endif

int rt_seg_init(void)
{
    g_seg_device.type         = RT_Device_Class_Char;
    g_seg_device.rx_indicate  = RT_NULL;
    g_seg_device.tx_complete  = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
    g_seg_device.ops          = &g_seg_device_ops;
#else
    g_seg_device.init         = seg_dev_init;
    g_seg_device.open         = RT_NULL;
    g_seg_device.close        = RT_NULL;
    g_seg_device.read         = RT_NULL;
    g_seg_device.write        = seg_dev_write;
    g_seg_device.control      = RT_NULL;
#endif

    g_seg_device.user_data    = RT_NULL;

    /* register a character device */
    rt_device_register(&g_seg_device, "redSeg", RT_DEVICE_FLAG_WRONLY);
}
INIT_BOARD_EXPORT(rt_seg_init);
