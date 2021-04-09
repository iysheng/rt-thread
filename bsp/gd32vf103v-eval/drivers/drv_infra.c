/******************************************************************************
* File:             drv_infra.c
*
* Author:           iysheng@163.com  
* Created:          04/09/21 
*                   红外光幕驱动文件
*****************************************************************************/

#include <rtthread.h>
#include <rtthread.h>
#include "board.h"
#include "drv_infra.h"

static struct rt_device g_infra_device;

static rt_err_t  infra_dev_init   (rt_device_t dev)
{
    rcu_periph_clock_enable(RCU_TIMER0);
    /* TODO init timer0 */

    return 0;
}

static rt_err_t infra_dev_control(rt_device_t dev, int cmd, void *args)
{
    int ret = 0;

    switch (cmd)
    {
    case RED_INFRA_INIT_CHANNEL:
        /* TODO start timer */
        break;
    case RED_INFRA_GET_CHANNEL:
        /* TODO get target channel */
        break;
    case RED_INFRA_DEINIT_CHANNEL:
        /* TODO stop timer */
        break;
    default:
        ret = -EINVAL;
        break;
    }

    return ret;
}

#ifdef RT_USING_DEVICE_OPS
const struct rt_device_ops g_infra_device_ops = {
    .init = infra_dev_init,
    .control = infra_dev_control,
};
#endif

int rt_infra_init(void)
{
    g_infra_device.type         = RT_Device_Class_Char;
    g_infra_device.rx_indicate  = RT_NULL;
    g_infra_device.tx_complete  = RT_NULL;

#ifdef RT_USING_DEVICE_OPS
    g_infra_device.ops          = &g_infra_device_ops;
#else
    g_infra_device.init         = infra_dev_init;
    g_infra_device.open         = RT_NULL;
    g_infra_device.close        = RT_NULL;
    g_infra_device.read         = RT_NULL;
    g_infra_device.write        = RT_NULL;
    g_infra_device.control      = infra_dev_control;
#endif

    g_infra_device.user_data    = RT_NULL;

    /* register a character device */
    rt_device_register(&g_infra_device, "Infra", RT_DEVICE_FLAG_RDWR);
}
INIT_BOARD_EXPORT(rt_infra_init);
