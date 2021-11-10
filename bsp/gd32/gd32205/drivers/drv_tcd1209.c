/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2021-10-01     iysheng           TCD1209 driver
 */

#include <rtconfig.h>
#include <rtdevice.h>
#include "gd32f20x_timer.h"
#include "gd32f20x_dma.h"
#include "drv_gpio.h"
#include "drv_tcd1209.h"

#define DBG_LVL    DBG_INFO
#define DBG_TAG    "tcd1209"
#include <rtdbg.h>

static uint16_t gs_ad9945_data[AD9945_DATA_COUNTS], gs_ad9945_data4portc[AD9945_DATA_COUNTS];
/*
 * AHB = 120M
 * APB2 = 120M
 * APB1 = 60M
 *
 * APB1 的分频作为 TIMER1/2/3/4/5/6 11/12/13 的时钟, 最大为 60MHz 并且如果分频为 1 ,那么 x1 否则 x2
 * APB2 的分频作为 TIMER0/7/8/9/10 的时钟, 最大为 120MHz 并且如果分频为 1 ,那么 x1 否则 x2
 * */
#if 0
/**
  * @brief 获取指定寄存器地址的数据
  * @param uint8_t reg_addr: 
  * retval 寄存器地址的数据.
  */
static uint16_t _get_ad9945_reg_value(uint8_t reg_addr)
{
    int addr_index = 0;
    uint16_t reg_value = 0;

    rt_pin_mode(GD32_AD9945_SDA_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(GD32_AD9945_SL_PIN, PIN_LOW);

    for (; addr_index < 4; addr_index ++)
    {
        rt_pin_write(GD32_AD9945_SDA_PIN, reg_addr >> addr_index & 0x01);
        rt_pin_write(GD32_AD9945_SCK_PIN, PIN_LOW);
        rt_thread_mdelay(1);
        rt_pin_write(GD32_AD9945_SCK_PIN, PIN_HIGH);
        rt_thread_mdelay(1);
    }
    rt_pin_mode(GD32_AD9945_SDA_PIN, PIN_MODE_INPUT);
    for (addr_index = 0; addr_index < 12; addr_index ++)
    {
        rt_pin_write(GD32_AD9945_SCK_PIN, PIN_LOW);
        rt_thread_mdelay(1);
        reg_value |= (rt_pin_read(GD32_AD9945_SDA_PIN) & 0x01) << addr_index;
        rt_pin_write(GD32_AD9945_SCK_PIN, PIN_HIGH);
        rt_thread_mdelay(1);
    }

    return reg_value;
}
#endif

/**
  * @brief 设置指定寄存器地址的数据
  * @param uint8_t reg_addr: 
  * @param uint16_t reg_data: 
  * retval N/A.
  */
static void _set_ad9945_reg_value(uint8_t reg_addr, uint16_t reg_data)
{
    int addr_index = 0;

    rt_pin_write(GD32_AD9945_SL_PIN, PIN_LOW);
    rt_thread_mdelay(1);

    for (; addr_index < 4; addr_index ++)
    {
        rt_pin_write(GD32_AD9945_SCK_PIN, PIN_LOW);
        rt_pin_write(GD32_AD9945_SDA_PIN, reg_addr >> addr_index & 0x01);
        rt_thread_mdelay(1);
        rt_pin_write(GD32_AD9945_SCK_PIN, PIN_HIGH);
        rt_thread_mdelay(1);
    }
    for (addr_index = 0; addr_index < 12; addr_index ++)
    {
        rt_pin_write(GD32_AD9945_SCK_PIN, PIN_LOW);
        rt_pin_write(GD32_AD9945_SDA_PIN, reg_data >> addr_index & 0x01);
        rt_thread_mdelay(1);
        rt_pin_write(GD32_AD9945_SCK_PIN, PIN_HIGH);
        rt_thread_mdelay(1);
    }
    rt_pin_write(GD32_AD9945_SL_PIN, PIN_HIGH);
}

/**
  * @brief 获取指定寄存器地址的数据
  * @param uint8_t reg_addr: 
  * retval 寄存器地址的数据.
  */
static uint16_t _get_ad9945_ad_value(void)
{
    rt_uint16_t port_b_value, port_c_value;

    port_b_value = gpio_input_port_get(GPIOB);
    port_c_value = gpio_input_port_get(GPIOC);

    port_b_value &= 0xc7f;
    port_c_value &= 0x1c00;
    port_c_value >>= 10;
    port_b_value |= port_c_value << 7;

    return port_b_value;
}

static int s_index;
static int s_start_sample;
void TIMER4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    if (s_index == AD9945_DATA_COUNTS)
    {
        timer_interrupt_disable(TIMER4, TIMER_INT_CH3);
    }
#if 0
    else if (s_index < 400)
    {
        rt_pin_write(GD32_AD9945_CLPOB_PIN, RESET);
    }
    else
    {
        rt_pin_write(GD32_AD9945_CLPOB_PIN, SET);
    }
#endif
    if (SET == timer_interrupt_flag_get(TIMER4, TIMER_INT_FLAG_CH3))
    {
        timer_interrupt_flag_clear(TIMER4, TIMER_INT_FLAG_CH3);
     //   gs_ad9945_data[s_index++ % AD9945_DATA_COUNTS] = _get_ad9945_ad_value();
    }
    /* leave interrupt */
    rt_interrupt_leave();
}

void TIMER1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    if (timer_flag_get(TIMER1, TIMER_FLAG_CH1))
    {

    if (0 == s_start_sample)
    {
     //   timer_interrupt_enable(TIMER4, TIMER_INT_CH3);
        timer_enable(TIMER4);
        s_start_sample = 1;
    }
        timer_interrupt_flag_clear(TIMER1, TIMER_FLAG_CH1);
    }
    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel0_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    if (SET == dma_flag_get(DMA1, DMA_CH0, DMA_FLAG_FTF))
    {
        dma_flag_clear(DMA1, DMA_CH0, DMA_FLAG_FTF);
        timer_interrupt_disable(TIMER1, TIMER_INT_CH1);
        timer_disable(TIMER4);
    }
#if 0
    if (SET == dma_flag_get(DMA1, DMA_CH0, DMA_FLAG_HTF))
    {
        dma_flag_clear(DMA1, DMA_CH0, DMA_FLAG_HTF);
        LOG_I("WOW DMA1 CHANNEL0 HTF***********************");
    }
    if (SET == dma_flag_get(DMA1, DMA_CH0, DMA_FLAG_G))
    {
        dma_flag_clear(DMA1, DMA_CH0, DMA_FLAG_G);
        LOG_I("WOW DMA1 CHANNEL0 GLOBAL***********************");
    }
#endif
    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    if (SET == dma_flag_get(DMA1, DMA_CH1, DMA_FLAG_FTF))
    {
        dma_flag_clear(DMA1, DMA_CH1, DMA_FLAG_FTF);
        timer_interrupt_disable(TIMER1, TIMER_INT_CH1);
        timer_disable(TIMER4);
        s_index = AD9945_DATA_COUNTS;
    }
    /* leave interrupt */
    rt_interrupt_leave();
}

/**
  * @brief 初始化 DMA 完成 AD9945 输出的 AD 数据搬移
  * @param void: 
  * retval N/A.
  */
static void dma_init4ad9945(void)
{
    dma_parameter_struct dma_param4dataclk_portb, dma_param4dataclk_portc;

    rcu_periph_clock_enable(RCU_DMA1);
    dma_param4dataclk_portb.periph_addr  = GPIOB + 0x08U;
    dma_param4dataclk_portb.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_param4dataclk_portb.periph_inc   = (uint8_t)DMA_PERIPH_INCREASE_DISABLE;
    dma_param4dataclk_portb.memory_addr  = (uint32_t)&gs_ad9945_data[0];
    dma_param4dataclk_portb.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_param4dataclk_portb.memory_inc   = (uint8_t)DMA_MEMORY_INCREASE_ENABLE;
    dma_param4dataclk_portb.number       = AD9945_DATA_COUNTS;
    dma_param4dataclk_portb.direction    = (uint8_t)DMA_PERIPHERAL_TO_MEMORY;
    dma_param4dataclk_portb.priority     = DMA_PRIORITY_HIGH;

    dma_param4dataclk_portc.periph_addr  = GPIOC + 0x08U;
    dma_param4dataclk_portc.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_param4dataclk_portc.periph_inc   = (uint8_t)DMA_PERIPH_INCREASE_DISABLE;
    dma_param4dataclk_portc.memory_addr  = (uint32_t)&gs_ad9945_data4portc[0];
    dma_param4dataclk_portc.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_param4dataclk_portc.memory_inc   = (uint8_t)DMA_MEMORY_INCREASE_ENABLE;
    dma_param4dataclk_portc.number       = AD9945_DATA_COUNTS;
    dma_param4dataclk_portc.direction    = (uint8_t)DMA_PERIPHERAL_TO_MEMORY;
    dma_param4dataclk_portc.priority     = DMA_PRIORITY_HIGH;

    dma_init(DMA1, DMA_CH0, &dma_param4dataclk_portb);
    dma_init(DMA1, DMA_CH1, &dma_param4dataclk_portc);
    NVIC_SetPriority(DMA1_Channel0_IRQn, 0);
    NVIC_EnableIRQ(DMA1_Channel0_IRQn);
    NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    dma_interrupt_enable(DMA1, DMA_CH0, DMA_INT_FTF | DMA_INT_ERR);
    dma_interrupt_enable(DMA1, DMA_CH1, DMA_INT_FTF | DMA_INT_ERR);
    LOG_I("DMA init ok");
}

/**
  * @brief 初始化调节 pwm 补光灯
  * @param void: 
  * retval N/A.
  */
static void pwm_adj4led_init(void)
{
    timer_parameter_struct timer4led;

    timer4led.prescaler         = 119U;
    timer4led.alignedmode       = TIMER_COUNTER_EDGE;
    timer4led.counterdirection  = TIMER_COUNTER_UP;
    timer4led.period            = 0U;
    timer4led.clockdivision     = TIMER_CKDIV_DIV1;
    timer4led.repetitioncounter = 0U;

    rcu_periph_clock_enable(RCU_TIMER11);
    timer_init(TIMER11, &timer4led);
    timer_channel_output_mode_config(TIMER11, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_autoreload_value_config(TIMER11, 1000);
    timer_channel_output_pulse_value_config(TIMER11, TIMER_CH_0, 300);
    timer_channel_output_state_config(TIMER11, TIMER_CH_0, ENABLE);
    timer_interrupt_disable(TIMER11, TIMER_INT_CH0);
    timer_enable(TIMER11);

    LOG_I("pwm for adj led init");
}

/**
  * @brief AD9945 初始化
  * @param void: 
  * retval .
  */
static void ad9945_device_init(void)
{
    /* timer 4 AD9945 device */
    timer_parameter_struct timer4shp, timer4shd, timer4dataclk, timer4clpob, timer4pblk;
    timer_oc_parameter_struct timer_oc4shd;

    timer4shp.prescaler         = 5U;
    timer4shp.alignedmode       = TIMER_COUNTER_EDGE;
    timer4shp.counterdirection  = TIMER_COUNTER_UP;
    timer4shp.period            = 0U;
    timer4shp.clockdivision     = TIMER_CKDIV_DIV1;
    timer4shp.repetitioncounter = 0U;

    timer4shd.prescaler         = 5U;
    timer4shd.alignedmode       = TIMER_COUNTER_EDGE;
    timer4shd.counterdirection  = TIMER_COUNTER_UP;
    timer4shd.period            = 0U;
    timer4shd.clockdivision     = TIMER_CKDIV_DIV1;
    timer4shd.repetitioncounter = 0U;

    timer_oc4shd.outputstate  = TIMER_CCX_ENABLE;
    timer_oc4shd.outputnstate = TIMER_CCXN_DISABLE;
    timer_oc4shd.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_oc4shd.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_oc4shd.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_oc4shd.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer4dataclk.prescaler         = 5U;
    timer4dataclk.alignedmode       = TIMER_COUNTER_EDGE;
    timer4dataclk.counterdirection  = TIMER_COUNTER_UP;
    timer4dataclk.period            = 0U;
    timer4dataclk.clockdivision     = TIMER_CKDIV_DIV1;
    timer4dataclk.repetitioncounter = 0U;

    rcu_periph_clock_enable(RCU_TIMER8);
    timer_init(TIMER8, &timer4shp);
    timer_channel_output_mode_config(TIMER8, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_autoreload_value_config(TIMER8, 19);
    timer_channel_output_pulse_value_config(TIMER8, TIMER_CH_0, 15);
    timer_channel_output_state_config(TIMER8, TIMER_CH_0, ENABLE);
    timer_interrupt_disable(TIMER8, TIMER_INT_CH0);

    rcu_periph_clock_enable(RCU_TIMER7);
    timer_init(TIMER7, &timer4shd);
    timer_channel_output_mode_config(TIMER7, TIMER_CH_0, TIMER_OC_MODE_PWM1);
    timer_autoreload_value_config(TIMER7, 19);
    timer_channel_output_pulse_value_config(TIMER7, TIMER_CH_0, 5);
    timer_channel_output_state_config(TIMER7, TIMER_CH_0, ENABLE);
    timer_interrupt_disable(TIMER7, TIMER_INT_CH0);
    timer_channel_output_config(TIMER7, TIMER_CH_0, &timer_oc4shd);
    timer_primary_output_config(TIMER7, ENABLE);
    timer_channel_output_fast_config(TIMER7, TIMER_CH_0, TIMER_OC_FAST_ENABLE);
    timer_enable(TIMER8);
    timer_enable(TIMER7);

    rcu_periph_clock_enable(RCU_TIMER4);
    timer_init(TIMER4, &timer4dataclk);
    timer_channel_output_mode_config(TIMER4, TIMER_CH_3, TIMER_OC_MODE_PWM1);
    timer_autoreload_value_config(TIMER4, 19);
    timer_channel_output_pulse_value_config(TIMER4, TIMER_CH_3, 10);
    timer_channel_output_state_config(TIMER4, TIMER_CH_3, ENABLE);
    timer_interrupt_disable(TIMER4, TIMER_INT_CH3);

    /* just for another dma for portc */
    timer_channel_output_mode_config(TIMER4, TIMER_CH_2, TIMER_OC_MODE_PWM1);
    timer_autoreload_value_config(TIMER4, 19);
    timer_channel_output_pulse_value_config(TIMER4, TIMER_CH_2, 10);
    timer_channel_output_state_config(TIMER4, TIMER_CH_2, ENABLE);
    timer_interrupt_disable(TIMER4, TIMER_INT_CH2);

    timer_channel_dma_request_source_select(TIMER4, TIMER_DMAREQUEST_CHANNELEVENT);
    timer_dma_enable(TIMER4, TIMER_DMA_CH3D);
    timer_dma_enable(TIMER4, TIMER_DMA_CH2D);

#if 0
    NVIC_SetPriority(TIMER4_IRQn, 0);
    NVIC_EnableIRQ(TIMER4_IRQn);
#endif

    rt_pin_write(GD32_AD9945_PBLK_PIN, SET);
    rt_pin_write(GD32_AD9945_CLPOB_PIN, SET);
    _set_ad9945_reg_value(0x00, 0x08);
    _set_ad9945_reg_value(0x01, 0x00);
    _set_ad9945_reg_value(0x02, 0x80);
    _set_ad9945_reg_value(0x03, 0x01);
    _set_ad9945_reg_value(0x0d, 0x838);
    dma_init4ad9945();
    LOG_I("AD9945 START");
}

int tcd1209_hw_init(void)
{
    int ret;
    /* timer 4 TCD1209 device */
    timer_parameter_struct timer4f1, timer4f2, timer4cp, timer4rs, timer4sh;

    timer4f1.prescaler         = 5U;
    timer4f1.alignedmode       = TIMER_COUNTER_EDGE;
    timer4f1.counterdirection  = TIMER_COUNTER_UP;
    timer4f1.period            = 0U;
    timer4f1.clockdivision     = TIMER_CKDIV_DIV1;
    timer4f1.repetitioncounter = 0U;

    timer4f2.prescaler         = 5U;
    timer4f2.alignedmode       = TIMER_COUNTER_EDGE;
    timer4f2.counterdirection  = TIMER_COUNTER_UP;
    timer4f2.period            = 0U;
    timer4f2.clockdivision     = TIMER_CKDIV_DIV1;
    timer4f2.repetitioncounter = 0U;

    timer4cp.prescaler         = 5U;
    timer4cp.alignedmode       = TIMER_COUNTER_EDGE;
    timer4cp.counterdirection  = TIMER_COUNTER_UP;
    timer4cp.period            = 0U;
    timer4cp.clockdivision     = TIMER_CKDIV_DIV1;
    timer4cp.repetitioncounter = 0U;

    timer4rs.prescaler         = 5U;
    timer4rs.alignedmode       = TIMER_COUNTER_EDGE;
    timer4rs.counterdirection  = TIMER_COUNTER_UP;
    timer4rs.period            = 0U;
    timer4rs.clockdivision     = TIMER_CKDIV_DIV1;
    timer4rs.repetitioncounter = 0U;

    timer4sh.prescaler         = 119U;
    timer4sh.alignedmode       = TIMER_COUNTER_EDGE;
    timer4sh.counterdirection  = TIMER_COUNTER_UP;
    timer4sh.period            = 0U;
    timer4sh.clockdivision     = TIMER_CKDIV_DIV1;
    timer4sh.repetitioncounter = 0U;

    LOG_I("Hello TCD1209");

    rcu_periph_clock_enable(RCU_TIMER1);
    timer_init(TIMER1, &timer4sh);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_autoreload_value_config(TIMER1, AD9945_DATA_COUNTS);
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_1, 1);
    timer_channel_output_state_config(TIMER1, TIMER_CH_1, ENABLE);
    timer_interrupt_disable(TIMER1, TIMER_INT_CH1);
    timer_enable(TIMER1);
    NVIC_SetPriority(TIMER1_IRQn, 10);
    NVIC_EnableIRQ(TIMER1_IRQn);
    //rt_thread_mdelay(1);

    rcu_periph_clock_enable(RCU_TIMER3);
    timer_init(TIMER3, &timer4f1);
    timer_channel_output_mode_config(TIMER3, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_autoreload_value_config(TIMER3, 19);
    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_1, 10);
    timer_channel_output_state_config(TIMER3, TIMER_CH_1, ENABLE);
    timer_interrupt_disable(TIMER3, TIMER_INT_CH1);
    timer_enable(TIMER3);

    rcu_periph_clock_enable(RCU_TIMER2);
    timer_init(TIMER2, &timer4f2);
    timer_channel_output_mode_config(TIMER2, TIMER_CH_1, TIMER_OC_MODE_PWM1);
    timer_autoreload_value_config(TIMER2, 19);
    timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_1, 10);
    timer_channel_output_state_config(TIMER2, TIMER_CH_1, ENABLE);
    timer_interrupt_disable(TIMER2, TIMER_INT_CH1);
    timer_enable(TIMER2);

    rcu_periph_clock_enable(RCU_TIMER9);
    timer_init(TIMER9, &timer4cp);
    timer_channel_output_mode_config(TIMER9, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_autoreload_value_config(TIMER9, 19);
    timer_channel_output_pulse_value_config(TIMER9, TIMER_CH_0, 4);
    timer_channel_output_state_config(TIMER9, TIMER_CH_0, ENABLE);
    timer_interrupt_disable(TIMER9, TIMER_INT_CH0);
    timer_enable(TIMER9);

    rcu_periph_clock_enable(RCU_TIMER10);
    timer_init(TIMER10, &timer4rs);
    timer_channel_output_mode_config(TIMER10, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_autoreload_value_config(TIMER10, 19);
    timer_channel_output_pulse_value_config(TIMER10, TIMER_CH_0, 6);
    timer_channel_output_state_config(TIMER10, TIMER_CH_0, ENABLE);
    timer_interrupt_disable(TIMER10, TIMER_INT_CH0);
    timer_enable(TIMER10);

    ad9945_device_init();
    pwm_adj4led_init();
    return ret;
}
INIT_PREV_EXPORT(tcd1209_hw_init);

long show_ad9945(void)
{
    int i = 0;

    s_start_sample = 0;

    dma_channel_enable(DMA1, DMA_CH0);
    dma_channel_enable(DMA1, DMA_CH1);
    timer_interrupt_flag_clear(TIMER1, TIMER_FLAG_CH1);
    timer_interrupt_enable(TIMER1, TIMER_INT_CH1);
    while(s_index < AD9945_DATA_COUNTS);
    dma_channel_disable(DMA1, DMA_CH0);
    dma_channel_disable(DMA1, DMA_CH1);
    dma_transfer_number_config(DMA1, DMA_CH0, AD9945_DATA_COUNTS);
    dma_transfer_number_config(DMA1, DMA_CH1, AD9945_DATA_COUNTS);
    s_index = 0;
#if 1
    for (; i < AD9945_DATA_COUNTS; i++)
    {
        gs_ad9945_data4portc[i] &= 0x1c00;
        gs_ad9945_data4portc[i] >>= 10;
        rt_kprintf("%u,", (gs_ad9945_data[i] & 0xc7f) | gs_ad9945_data4portc[i] << 7);
    }
#endif
}
MSH_CMD_EXPORT(show_ad9945, list device in system);

long adj_led(int argc, char *argv[])
{
    timer_channel_output_pulse_value_config(TIMER11, TIMER_CH_0, atoi(argv[1]));
}
MSH_CMD_EXPORT(adj_led, adjust led level);
