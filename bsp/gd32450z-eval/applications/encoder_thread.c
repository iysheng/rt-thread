/******************************************************************************
* File:             encoder_thread.c
*
* Author:           iysheng@163.com  
* Created:          06/13/21 
*                   编码器后端服务线程
*****************************************************************************/

#include <rtthread.h>
#include <rtdevice.h>
#include "can_comm.h"
#include "drv_gpio.h"

#define DBG_LVL               DBG_LOG
#define DBG_TAG               "app.encoder"
#include <rtdbg.h>

#define RELAY_PIN0    GET_PIN(E, 2)
#define RELAY_PIN1    GET_PIN(E, 3)

extern void *get_sync_obj_encoder(void);
int encoder_comm_backend_init(void)
{
    unsigned int id = 0;
    int ret;
    rt_sem_t drv_encoder_sem = get_sync_obj_encoder();

    if (!drv_encoder_sem)
    {
        LOG_E("Failed get encoder sem.");
        return -ENODEV;
    }

    /* TODO Init Relay Pin */
    rt_pin_mode(RELAY_PIN0, PIN_MODE_OUTPUT);
    rt_pin_mode(RELAY_PIN1, PIN_MODE_OUTPUT);

    LOG_D("Hello encoder");

    while(1)
    {
        rt_thread_mdelay(1);
        if (!rt_sem_take(drv_encoder_sem, RT_TICK_PER_SECOND))
        {
            ret = set_ccd_check(1, id++);
            LOG_I("ret=%d", ret);
            if (!ret)
            {
                LOG_I("No match");
                rt_pin_write(RELAY_PIN0, PIN_HIGH);
                rt_pin_write(RELAY_PIN1, PIN_HIGH);
            }
            else if (1 == ret)
            {
                LOG_I("Match");
                rt_pin_write(RELAY_PIN0, PIN_LOW);
                rt_pin_write(RELAY_PIN1, PIN_LOW);
            }
            else
            {
                LOG_E("Errors in check, err=%d", ret);
            }
        }
#if 0
        else
        {
            LOG_E("No ok");
        }
#endif
    }

    return 0;
}
