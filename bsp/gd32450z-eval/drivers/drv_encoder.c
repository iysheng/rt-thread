/******************************************************************************
* File:             drv_encoder.c
*
* Author:           iysheng@163.com  
* Created:          06/10/21 
* Description:      编码器驱动
*****************************************************************************/

#include <rtthread.h>
#include <rtdevice.h>
#include "gd32f4xx.h"
#include <drv_gpio.h>
#include "can_comm.h"

#define DBG_LEVEL  DBG_LOG
#define DBG_TAG    "drv.encoder"
#include <rtdbg.h>

#define ENCODER_A GET_PIN(E, 5)
#define ENCODER_B GET_PIN(E, 6)
#define ENCODER_Z GET_PIN(E, 9)

static struct rt_semaphore gs_encoder4backend_sem;

void *get_sync_obj_encoder(void)
{
    return &gs_encoder4backend_sem;
}

static void catch_encoder(void *args)
{
    static int times;
    LOG_I("just catch encoder interrupt:%d.\n", times++);
    if (times % 5 == 1)
    {
        rt_sem_release(&gs_encoder4backend_sem);
    }
}

static int rt_hw_encoder_init(void)
{
    LOG_I("Init ok ....");

    /* 初始化 CAN 接收信号量 */
    rt_sem_init(&gs_encoder4backend_sem, "encoder_sem", 0, RT_IPC_FLAG_FIFO);
    rcu_periph_clock_enable(RCU_GPIOE);
    rt_pin_mode(ENCODER_A, PIN_MODE_INPUT_PULLDOWN);
    rt_pin_mode(ENCODER_A, PIN_MODE_INPUT);
#if 1
    rcu_periph_clock_enable(RCU_SYSCFG);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOE, EXTI_SOURCE_PIN5);
    exti_init(EXTI_5, \
           EXTI_INTERRUPT, \
           EXTI_TRIG_RISING);
    exti_interrupt_enable(EXTI_5);
#endif
    rt_pin_attach_irq(ENCODER_A, PIN_IRQ_MODE_RISING, catch_encoder, RT_NULL);
    rt_pin_irq_enable(ENCODER_A, PIN_IRQ_ENABLE);

    return 0;
}
INIT_DEVICE_EXPORT(rt_hw_encoder_init);
