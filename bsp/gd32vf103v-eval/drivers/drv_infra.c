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
#include "drv_gpio.h"
#include "drv_infra.h"

#define DBG_LEVEL   DBG_INFO
#define DRV_TAG    "drv.infra"
#include <rtdbg.h>

#define INFRA_CHANNEL_MAX 16

static struct rt_device g_infra_device;
static int sg_infra_channel;
static unsigned char sg_target_channel = 0xff;

static int _set_pin_channel(int channel)
{
    if (channel == INFRA_CHANNEL_MAX)
    {
        return -E2BIG;
    }

    /* BIT[12:15] */
    GPIO_OCTL(GPIOB) &= ~(uint32_t)0xf000;
    GPIO_OCTL(GPIOB) |= (uint32_t)(channel << 12);

    return 0;
}

void TIMER2_IRQHandler(void)
{
    rt_interrupt_enter();

#ifdef INFRA_SEND_DEVICE
    static int timer2_irq_counts;
    if (timer_interrupt_flag_get(TIMER2, TIMER_INT_FLAG_UP))
    {
        timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);
        if (++timer2_irq_counts % 2 == 0)
        {
            _set_pin_channel(sg_infra_channel++);
            if (sg_infra_channel == INFRA_CHANNEL_MAX)
            {
                sg_infra_channel = 0;
            }
        }
    }
#else
    if (timer_interrupt_flag_get(TIMER2, TIMER_INT_FLAG_CH3))
    {
        timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_CH3);
        _set_pin_channel(sg_infra_channel);
#if 1 /* test code */
		static int aa;
        gpio_bit_write(GPIOC, GPIO_PIN_9, (aa++ % 2) ? RESET : SET);
#endif
        if (PIN_HIGH == gpio_input_bit_get(GPIOA, GPIO_PIN_6))
        {
            /* get channel num */
            sg_target_channel = sg_infra_channel;
        }
        else if (sg_target_channel == sg_infra_channel)
        {
            /* mark the channel is invalid */
            sg_target_channel = 0xff;
        }
        else if ((sg_target_channel != 0xff) && \
            (sg_target_channel != sg_infra_channel))
        {
            /* not in a cycle just ignore */
        }

        sg_infra_channel++;
        if (sg_infra_channel == INFRA_CHANNEL_MAX)
        {
            sg_infra_channel = 0;
        }
    }
#endif

    rt_interrupt_leave();
}
#if 0
static void infra4recv_handler(void *args)
{
    _set_pin_channel(sg_infra_channel);
    if (PIN_HIGH == gpio_input_bit_get(GPIOA, GPIO_PIN_6))
    {
        /* get channel num */
        sg_target_channel = sg_infra_channel;
    }
    else if (sg_target_channel == sg_infra_channel)
    {
        /* mark the channel is invalid */
        sg_target_channel = 0xff;
    }
    else if ((sg_target_channel != 0xff) && \
        (sg_target_channel != sg_infra_channel))
    {
        /* not in a cycle just ignore */
    }

    sg_infra_channel++;
    if (sg_infra_channel == INFRA_CHANNEL_MAX)
    {
        sg_infra_channel = 0;
    }
}
#endif

static rt_err_t  infra_dev_init(rt_device_t dev)
{
#ifdef INFRA_SEND_DEVICE
    timer_parameter_struct para = {
        .prescaler = (10800 - 1), /* APB1 clk = 54MHz Cycle = 10KHz */
        .alignedmode = TIMER_COUNTER_EDGE,
        .counterdirection = TIMER_COUNTER_UP,
        .period = 10, /* Cycle 10KHz / 10 = 1Khz */
        .clockdivision = TIMER_CKDIV_DIV1,
        .repetitioncounter = 0,
    };
    timer_oc_parameter_struct ocpara = {
        .outputstate = TIMER_CCX_ENABLE,
        .outputnstate = TIMER_CCXN_DISABLE,
        .ocpolarity = TIMER_OC_POLARITY_HIGH,
        .ocnpolarity = TIMER_OCN_POLARITY_HIGH,
        .ocidlestate = TIMER_OC_IDLE_STATE_LOW,
        .ocnidlestate = TIMER_OCN_IDLE_STATE_LOW,
    };

    rcu_periph_clock_enable(RCU_TIMER2);
    /* TODO init timer2 */
    timer_init(TIMER2, &para);
    timer_internal_clock_config(TIMER2);
    timer_channel_output_config(TIMER2, TIMER_CH_3, &ocpara);
    timer_channel_output_mode_config(TIMER2, TIMER_CH_3, TIMER_OC_MODE_PWM0);
    timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_3, 5);
    /* remap pin PB1 as timer2_ch3 output */
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1);

    /* init infra signal pin to low */
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
    gpio_bit_write(GPIOA, GPIO_PIN_6, RESET);

    eclic_irq_enable(TIMER2_IRQn, 0, 0);
    timer_interrupt_enable(TIMER2, TIMER_INT_UP);
#else /* receive device */
    /* gpio init of PA6 as input */
    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
#if 0
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING/*GPIO_MODE_IPD*/, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOB, GPIO_PIN_SOURCE_1);
    /* band PB1 interrupt */
    rt_pin_attach_irq(17, PIN_IRQ_MODE_RISING, infra4recv_handler, RT_NULL);
    rt_pin_irq_enable(17, PIN_IRQ_ENABLE);
#else
    timer_parameter_struct para = {
        .prescaler = (216 - 1), /* APB1 clk = 54MHz Cycle = 0.5MHz */
        .alignedmode = TIMER_COUNTER_EDGE,
        .counterdirection = TIMER_COUNTER_UP,
        .period = 65535, /* Max cycle 1 / 0.5MHz * 65535 ~ 120ms */
        .clockdivision = TIMER_CKDIV_DIV1,
        .repetitioncounter = 0,
    };
    timer_ic_parameter_struct icpara = {
    };

    rcu_periph_clock_enable(RCU_TIMER2);
    /* TODO init timer2 */
    timer_init(TIMER2, &para);
    timer_channel_input_struct_para_init(&icpara);
    icpara.icpolarity  = TIMER_IC_POLARITY_FALLING;
    icpara.icfilter = 0;
    timer_input_capture_config(TIMER2, TIMER_CH_3, &icpara);
    timer_auto_reload_shadow_enable(TIMER2);
    /* remap pin PB1 as timer2_ch3 channel */
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_1);

    timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_CH3);
    eclic_irq_enable(TIMER2_IRQn, 0, 0);
    timer_interrupt_enable(TIMER2, TIMER_INT_CH3);
    timer_enable(TIMER2);
#endif
#endif
    /* init PA7 as port select control pin */
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
    gpio_bit_write(GPIOA, GPIO_PIN_7, RESET);

    /* set addr pin mode output */
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_14);
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_15);
    _set_pin_channel(0);

    return 0;
}

static rt_err_t infra_dev_control(rt_device_t dev, int cmd, void *args)
{
    int ret = -EINVAL;

    switch (cmd)
    {
    case RED_INFRA_INIT_CHANNEL:
        /* TODO start timer */
        break;
    case RED_INFRA_GET_CHANNEL:
        /* TODO get target channel */
        if (sg_target_channel != 0xff)
        {
            *(unsigned char *)args = sg_target_channel;
            ret = 0;
        }
        else
        {
            ret = -EINVAL;
        }
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
