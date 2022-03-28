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
#include "tcd_abs.h"
#include <rthw.h>

#define DBG_LVL    DBG_INFO
#define DBG_TAG    "tcd1209"
#include <rtdbg.h>

static uint16_t _gs_ad9945_data[AD9945_DATA_COUNTS], _gs_ad9945_data4portc[AD9945_DATA_COUNTS];
static uint16_t gs_ccd_raw_value[AD9945_DATA_COUNTS];
static uint16_t gs_index;
static int16_t _gs_data_delta = 10;
static uint8_t gs_sync4dma_flag = 0x80;
static uint8_t gs_type4scan_mode = SCAN_TYPE_ONESHOT;
static uint8_t _gs_catch_command = 0;

static uint8_t gs_just_test_led;

void mark_catch_command(uint8_t status)
{
    _gs_catch_command = status;
}

/*
 * CCD 标定位置以及参数配置
 * */
static ccd_data_map_t gs_sample_test = {
    .position = {
        600,900,600,
    },
};

static ccd_data_t gs_calibrate_data;

static inline uint16_t get_ccd_value2index(uint16_t index)
{
    uint16_t ccd_value = 0;

    /*
     * bit [11:0]
     * C4:C5:B0:B1:B2:B10:B11:B7:B8:B9:C0:C1
     * */

    ccd_value = 0;
    ccd_value = (_gs_ad9945_data4portc[index] & 0x10) << 7;
    ccd_value |= (_gs_ad9945_data4portc[index] & 0x20) << 5;
    ccd_value |= (_gs_ad9945_data[index] & 0x01) << 9;
    ccd_value |= (_gs_ad9945_data[index] & 0x02) << 7;
    ccd_value |= (_gs_ad9945_data[index] & 0x04) << 5;
    ccd_value |= (_gs_ad9945_data[index] & 0x400) >> 4;
    ccd_value |= (_gs_ad9945_data[index] & 0x800) >> 6;
    ccd_value |= (_gs_ad9945_data[index] & 0x80) >> 3;
    ccd_value |= (_gs_ad9945_data[index] & 0x100) >> 5;
    ccd_value |= (_gs_ad9945_data[index] & 0x200) >> 7;
    ccd_value |= (_gs_ad9945_data4portc[index] & 0x01) << 1;
    ccd_value |= (_gs_ad9945_data4portc[index] & 0x02) >> 1;

    return ccd_value;
}

static inline void raw_data_sync(void)
{
    int i = 0;
    uint16_t raw_value_tmp;

    for (; i < AD9945_DATA_COUNTS; i++)
    {
        raw_value_tmp = get_ccd_value2index(i);
        //if (i < 50)
        //    rt_kprintf("%u]", raw_value_tmp);
        //gs_ccd_raw_value[i] = raw_value_tmp > gs_ccd_raw_value[i] ? raw_value_tmp : gs_ccd_raw_value[i];
        gs_ccd_raw_value[i] = raw_value_tmp;
    }
    //rt_kprintf("\n");
}

static inline void raw_data_clear(void)
{
    int i = 0;
    uint32_t * ptr = (uint32_t *)&gs_ccd_raw_value[0];

    for (; i < AD9945_DATA_COUNTS / 2; i++)
    {
        *ptr++ = 0x00;
    }
    gs_sample_test.value.left = 0;
    gs_sample_test.value.middle = 0;
    gs_sample_test.value.right = 0;
}

static inline void ccd_scan_restart(void)
{
    gs_index = 0;
    dma_transfer_number_config(DMA0, DMA_CH0, AD9945_DATA_COUNTS);
    dma_transfer_number_config(DMA0, DMA_CH4, AD9945_DATA_COUNTS);
    dma_channel_enable(DMA0, DMA_CH0);
    dma_channel_enable(DMA0, DMA_CH4);
    timer_interrupt_flag_clear(TIMER7, TIMER_INT_FLAG_CH3);
    timer_interrupt_enable(TIMER7, TIMER_INT_CH3);
}

void ccd_scan_recovery(void)
{
    if (likely(_gs_catch_command && SCAN_TYPE_CONTINUOUS == gs_type4scan_mode))
    {
        ccd_scan_restart();
        _gs_catch_command = 0;
    }
}

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
        rt_pin_write(GD33_AD9945_SDA_PIN, reg_addr >> addr_index & 0x01);
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

#if 0
void TIMER1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    if (gs_index == AD9945_DATA_COUNTS)
    {
        timer_interrupt_disable(TIMER1, TIMER_INT_CH0);
    }
#if 0
    else if (gs_index < 100)
    {
        rt_pin_write(GD32_AD9945_CLPOB_PIN, RESET);
    }
    else
    {
        rt_pin_write(GD32_AD9945_CLPOB_PIN, SET);
    }
#endif
    if (SET == timer_interrupt_flag_get(TIMER1, TIMER_INT_FLAG_CH0))
    {
        timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_CH0);
     //   gs_ad9915_data[gs_index++ % AD9945_DATA_COUNTS] = _get_ad9915_ad_value();
    }
    /* leave interrupt */
    rt_interrupt_leave();
}
#endif


void TIMER7_Channel_IRQHandler(void)
{
    rt_base_t level;
    /* enter interrupt */
    rt_interrupt_enter();

#if 1
    if (SET == timer_flag_get(TIMER7, TIMER_FLAG_CH3))
    {
        if ((gs_index == 0) && (gs_sync4dma_flag & 0x80))
        {
            //timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_0, 10);
            //timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_2, 10);
            /* DATACLK 定时器开启 */
            timer_enable(TIMER1);
            gs_index = 1;
        }
        else if (!(gs_sync4dma_flag & 0x80))
        {
            gs_sync4dma_flag |= 0x80;
        }
        timer_interrupt_flag_clear(TIMER7, TIMER_INT_FLAG_CH3);
    }
#else
    if (SET == timer_flag_get(TIMER7, TIMER_FLAG_CH3))
    {
        timer_interrupt_flag_clear(TIMER7, TIMER_INT_FLAG_CH3);
        rt_pin_write(21, gs_just_test_led++%2);
    }
#endif
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

    rcu_periph_clock_enable(RCU_DMA0);
    dma_param4dataclk_portb.periph_addr  = GPIOB + 0x08U;
    dma_param4dataclk_portb.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_param4dataclk_portb.periph_inc   = (uint8_t)DMA_PERIPH_INCREASE_DISABLE;
    dma_param4dataclk_portb.memory_addr  = (uint32_t)&_gs_ad9945_data[0];
    dma_param4dataclk_portb.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_param4dataclk_portb.memory_inc   = (uint8_t)DMA_MEMORY_INCREASE_ENABLE;
    dma_param4dataclk_portb.number       = AD9945_DATA_COUNTS;
    dma_param4dataclk_portb.direction    = (uint8_t)DMA_PERIPHERAL_TO_MEMORY;
    dma_param4dataclk_portb.priority     = DMA_PRIORITY_HIGH;

    dma_param4dataclk_portc.periph_addr  = GPIOC + 0x08U;
    dma_param4dataclk_portc.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_param4dataclk_portc.periph_inc   = (uint8_t)DMA_PERIPH_INCREASE_DISABLE;
    dma_param4dataclk_portc.memory_addr  = (uint32_t)&_gs_ad9945_data4portc[0];
    dma_param4dataclk_portc.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_param4dataclk_portc.memory_inc   = (uint8_t)DMA_MEMORY_INCREASE_ENABLE;
    dma_param4dataclk_portc.number       = AD9945_DATA_COUNTS;
    dma_param4dataclk_portc.direction    = (uint8_t)DMA_PERIPHERAL_TO_MEMORY;
    dma_param4dataclk_portc.priority     = DMA_PRIORITY_HIGH;

    dma_init(DMA0, DMA_CH0, &dma_param4dataclk_portb);
    dma_init(DMA0, DMA_CH4, &dma_param4dataclk_portc);
    NVIC_SetPriority(DMA0_Channel0_IRQn, 0x40);
    NVIC_EnableIRQ(DMA0_Channel0_IRQn);
    NVIC_SetPriority(DMA0_Channel4_IRQn, 0x40);
    NVIC_EnableIRQ(DMA0_Channel4_IRQn);
    dma_interrupt_enable(DMA0, DMA_CH0, DMA_INT_FTF | DMA_INT_ERR);
    dma_interrupt_enable(DMA0, DMA_CH4, DMA_INT_FTF | DMA_INT_ERR);
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
  * param void:
  * retval .
  */
static void ad9945_device_init(void)
{
    /* timer 4 AD9945 device */
    timer_parameter_struct timer4shp, timer4shd, timer4dataclk, timer4clpob, timer4pblk;
    timer_oc_parameter_struct timer_oc4clpob;
    uint32_t init_value4shp = 6, init_value4shd = 12, init_value4dataclk = 10;
    uint32_t init_value4clpob = 0, init_value4pblk = 0;

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

    timer4dataclk.prescaler         = 5U;
    timer4dataclk.alignedmode       = TIMER_COUNTER_EDGE;
    timer4dataclk.counterdirection  = TIMER_COUNTER_UP;
    timer4dataclk.period            = 0U;
    timer4dataclk.clockdivision     = TIMER_CKDIV_DIV1;
    timer4dataclk.repetitioncounter = 0U;

    rcu_periph_clock_enable(RCU_TIMER8);
    timer_init(TIMER8, &timer4shp);
    timer_channel_output_mode_config(TIMER8, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_autoreload_value_config(TIMER8, 19);
    timer_channel_output_pulse_value_config(TIMER8, TIMER_CH_1, 18);
    timer_channel_output_state_config(TIMER8, TIMER_CH_1, ENABLE);
    timer_interrupt_disable(TIMER8, TIMER_INT_CH1);

    rcu_periph_clock_enable(RCU_TIMER4);
    timer_init(TIMER4, &timer4shd);
    timer_channel_output_mode_config(TIMER4, TIMER_CH_2, TIMER_OC_MODE_PWM0);
    timer_autoreload_value_config(TIMER4, 19);
    timer_channel_output_pulse_value_config(TIMER4, TIMER_CH_2, 18);
    timer_channel_output_state_config(TIMER4, TIMER_CH_2, ENABLE);
    timer_interrupt_disable(TIMER4, TIMER_INT_CH2);

    rcu_periph_clock_enable(RCU_TIMER1);
    timer_init(TIMER1, &timer4dataclk);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_autoreload_value_config(TIMER1, 19);
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_0, 10);
    timer_channel_output_state_config(TIMER1, TIMER_CH_0, ENABLE);
    timer_interrupt_disable(TIMER1, TIMER_INT_CH0);
    timer_counter_value_config(TIMER1, init_value4dataclk);
    timer_counter_value_config(TIMER8, init_value4shp);
    timer_counter_value_config(TIMER4, init_value4shd);
    timer_enable(TIMER4);
    timer_enable(TIMER8);
    //timer_enable(TIMER1);

    timer4clpob.prescaler         = 119U;
    timer4clpob.alignedmode       = TIMER_COUNTER_EDGE;
    timer4clpob.counterdirection  = TIMER_COUNTER_UP;
    timer4clpob.period            = 0U;
    timer4clpob.clockdivision     = TIMER_CKDIV_DIV1;
    timer4clpob.repetitioncounter = 0U;

#if 0
    timer_oc4clpob.outputstate  = TIMER_CCX_DISABLE;
    timer_oc4clpob.outputnstate = TIMER_CCXN_ENABLE;
    timer_oc4clpob.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_oc4clpob.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_oc4clpob.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_oc4clpob.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
#endif

    rcu_periph_clock_enable(RCU_TIMER12);
    timer_init(TIMER12, &timer4clpob);
    timer_channel_output_mode_config(TIMER12, TIMER_CH_0, TIMER_OC_MODE_PWM1);
    timer_autoreload_value_config(TIMER12, AD9945_DATA_COUNTS);
    timer_channel_output_pulse_value_config(TIMER12, TIMER_CH_0, 32);
    timer_channel_output_state_config(TIMER12, TIMER_CH_0, ENABLE);
    timer_interrupt_disable(TIMER12, TIMER_INT_CH0);
    timer_counter_value_config(TIMER12, 23);
    timer_enable(TIMER12);

    timer4pblk.prescaler         = 119U;
    timer4pblk.alignedmode       = TIMER_COUNTER_EDGE;
    timer4pblk.counterdirection  = TIMER_COUNTER_UP;
    timer4pblk.period            = 0U;
    timer4pblk.clockdivision     = TIMER_CKDIV_DIV1;
    timer4pblk.repetitioncounter = 0U;

    rcu_periph_clock_enable(RCU_TIMER13);
    timer_init(TIMER13, &timer4pblk);
    timer_channel_output_mode_config(TIMER13, TIMER_CH_0, TIMER_OC_MODE_PWM1);
    timer_autoreload_value_config(TIMER13, AD9945_DATA_COUNTS);
    timer_channel_output_pulse_value_config(TIMER13, TIMER_CH_0, 30);
    timer_channel_output_state_config(TIMER13, TIMER_CH_0, ENABLE);
    timer_interrupt_disable(TIMER13, TIMER_INT_CH0);

#if 1
    timer_enable(TIMER13);
#else
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);
    gpio_bit_set(GPIOA, GPIO_PIN_7);
    //gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
    //gpio_bit_set(GPIOA, GPIO_PIN_6);
#endif

#if 1
    /* just for another dma for portc */
    timer_channel_output_mode_config(TIMER1, TIMER_CH_2, TIMER_OC_MODE_PWM0);
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_2, 10);
    timer_channel_output_state_config(TIMER1, TIMER_CH_2, DISABLE);
    timer_interrupt_disable(TIMER1, TIMER_INT_CH2);
    timer_channel_dma_request_source_select(TIMER1, TIMER_DMAREQUEST_CHANNELEVENT);
    timer_dma_enable(TIMER1, TIMER_DMA_CH0D);
    timer_dma_enable(TIMER1, TIMER_DMA_CH2D);
#endif

#if 0
    NVIC_SetPriority(TIMER1_IRQn, 0x0a);
    NVIC_EnableIRQ(TIMER1_IRQn);
#endif

#if 1
    //rt_pin_write(GD32_AD9945_PBLK_PIN, SET);
    /* 放开钳位 */
    _set_ad9945_reg_value(0x00, 0x08);
    _set_ad9945_reg_value(0x01, 0x00);
    /* 修改钳位 */
    _set_ad9945_reg_value(0x02, 00);
    /* 设置增益为 6dB */
    _set_ad9945_reg_value(0x03, 0x00);
    _set_ad9945_reg_value(0x0d, 0x838);
#endif
    dma_init4ad9945();
    gpio_bit_set(GPIOB, GPIO_PIN_5);
    LOG_I("AD9945 START");
}

int tcd1209_hw_init(void)
{
    int ret;
    /* timer 4 TCD1209 device */
    timer_parameter_struct timer4f1, timer4cp, timer4rs, timer4sh;
    timer_oc_parameter_struct timer_oc4f;
    uint32_t init_value4f = 4, init_value4cp = 4, init_value4rs = 0, init_value4sh = 0;

    timer4f1.prescaler         = 5U;
    timer4f1.alignedmode       = TIMER_COUNTER_EDGE;
    timer4f1.counterdirection  = TIMER_COUNTER_UP;
    timer4f1.period            = 0U;
    timer4f1.clockdivision     = TIMER_CKDIV_DIV1;
    timer4f1.repetitioncounter = 0U;

    timer_oc4f.outputstate  = TIMER_CCX_ENABLE;
    timer_oc4f.outputnstate = TIMER_CCXN_ENABLE;
    timer_oc4f.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_oc4f.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_oc4f.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_oc4f.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

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

    rcu_periph_clock_enable(RCU_TIMER7);
    timer_init(TIMER7, &timer4sh);
    timer_channel_output_mode_config(TIMER7, TIMER_CH_3, TIMER_OC_MODE_PWM0);
    timer_autoreload_value_config(TIMER7, AD9945_DATA_COUNTS);
    timer_channel_output_pulse_value_config(TIMER7, TIMER_CH_3, 1);
    timer_channel_output_state_config(TIMER7, TIMER_CH_3, ENABLE);
    timer_primary_output_config(TIMER7, ENABLE);
    timer_interrupt_disable(TIMER7, TIMER_INT_CH3);
    timer_enable(TIMER7);
    NVIC_SetPriority(TIMER7_Channel_IRQn, 0x0);
    NVIC_EnableIRQ(TIMER7_Channel_IRQn);
    //rt_thread_mdelay(1);
    rcu_periph_clock_enable(RCU_TIMER0);
    timer_init(TIMER0, &timer4f1);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM1);
    timer_autoreload_value_config(TIMER0, 19);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, 10);
    timer_channel_output_state_config(TIMER0, TIMER_CH_0, ENABLE);
    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_oc4f);
    timer_primary_output_config(TIMER0, ENABLE);
    timer_interrupt_disable(TIMER0, TIMER_INT_CH0);

    rcu_periph_clock_enable(RCU_TIMER2);
    timer_init(TIMER2, &timer4cp);
    timer_channel_output_mode_config(TIMER2, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_autoreload_value_config(TIMER2, 19);
    timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_0, 2);
    timer_channel_output_state_config(TIMER2, TIMER_CH_0, ENABLE);
    timer_interrupt_disable(TIMER2, TIMER_INT_CH0);

    rcu_periph_clock_enable(RCU_TIMER3);
    timer_init(TIMER3, &timer4rs);
    timer_channel_output_mode_config(TIMER3, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_autoreload_value_config(TIMER3, 19);
    timer_channel_output_pulse_value_config(TIMER3, TIMER_CH_0, 2);
    timer_channel_output_state_config(TIMER3, TIMER_CH_0, ENABLE);
    timer_interrupt_disable(TIMER3, TIMER_INT_CH0);

    timer_counter_value_config(TIMER2, init_value4cp);
    timer_counter_value_config(TIMER0, init_value4f);
    timer_counter_value_config(TIMER3, init_value4rs);
    timer_enable(TIMER2);
    timer_enable(TIMER0);
    timer_enable(TIMER3);
    ad9945_device_init();
    pwm_adj4led_init();

    return ret;
}
INIT_PREV_EXPORT(tcd1209_hw_init);

int convert_calibrate_ans(uint16_t *data, uint16_t data_len)
#if 1
{
    uint32_t sum = 0;
    uint16_t i = 0;

    for (; i < data_len; i++)
    {
        sum += data[i];
    }
    sum /= data_len;
    //rt_kprintf("sum:%u, left=%u\n", sum, gs_sample_test.position.left);
    gs_sample_test.value.left = 0;
    gs_sample_test.value.middle = 0;
    gs_sample_test.value.right = 0;

      for (i = 0; i < gs_sample_test.position.left; i++)
      {
          gs_sample_test.value.left += data[i] > sum ? 1 : 0;
      }
      for (; i < gs_sample_test.position.left + gs_sample_test.position.middle; i++)
      {
          gs_sample_test.value.middle += data[i] > sum ? 1 : 0;
      }
      for (; i < gs_sample_test.position.left + gs_sample_test.position.middle + gs_sample_test.position.right; i++)
      {
          gs_sample_test.value.right += data[i] > sum ? 1 : 0;
      }

    //rt_kprintf("scan ans:%hu,%hu,%hu\n", gs_sample_test.value.left, gs_sample_test.value.middle, gs_sample_test.value.right);
    return 0;
}
#else
{
    uint32_t sum = 0;
    uint16_t i = 0;

    for (; i < data_len; i++)
    {
        sum += data[i];
    }
    sum /= data_len;
    rt_kprintf("sum:%u, left=%u\n", sum, gs_sample_test.position.left);

    if (likely(gs_sample_test.value.left + gs_sample_test.value.middle + gs_sample_test.value.right != 0))
    {
        for (i = 0; i < gs_sample_test.position.left; i++)
        {
            gs_sample_test.value.left += data[i] > sum ? 1 : 0;
        }
        for (; i < gs_sample_test.position.left + gs_sample_test.position.middle; i++)
        {
            gs_sample_test.value.middle += data[i] > sum ? 1 : 0;
        }
        for (; i < gs_sample_test.position.left + gs_sample_test.position.middle + gs_sample_test.position.right; i++)
        {
            gs_sample_test.value.right += data[i] > sum ? 1 : 0;
        }
        gs_sample_test.value.left >>= 1;
        gs_sample_test.value.middle >>= 1;
        gs_sample_test.value.right >>= 1;
    }
    else
    {
        for (i = 0; i < gs_sample_test.position.left; i++)
        {
            gs_sample_test.value.left += data[i] > sum ? 1 : 0;
        }
        for (; i < gs_sample_test.position.left + gs_sample_test.position.middle; i++)
        {
            gs_sample_test.value.middle += data[i] > sum ? 1 : 0;
        }
        for (; i < gs_sample_test.position.left + gs_sample_test.position.middle + gs_sample_test.position.right; i++)
        {
            gs_sample_test.value.right += data[i] > sum ? 1 : 0;
        }
    }

    rt_kprintf("scan ans:%hu,%hu,%hu\n", gs_sample_test.value.left, gs_sample_test.value.middle, gs_sample_test.value.right);
    return 0;
}
#endif

static inline void convert_raw2data_sync(void)
{
    convert_calibrate_ans(gs_ccd_raw_value, AD9945_DATA_COUNTS);
}

static inline void save_calibrate_data(void)
{
    gs_calibrate_data = gs_sample_test.value;
    rt_kprintf("calibrate%u.%u.%u\n", gs_calibrate_data.left, gs_calibrate_data.middle, gs_calibrate_data.right);
}

static inline int compare_with_calibrate(void)
{
    int16_t delta;

    delta = gs_calibrate_data.left - gs_sample_test.value.left;
    if ((delta > _gs_data_delta) || delta + _gs_data_delta < 0)
    {
        return 1;
    }

    delta = gs_calibrate_data.middle - gs_sample_test.value.middle;
    if ((delta > _gs_data_delta) || delta + _gs_data_delta < 0)
    {
        return 1;
    }

    delta = gs_calibrate_data.right - gs_sample_test.value.right;
    if ((delta > _gs_data_delta) || delta + _gs_data_delta < 0)
    {
        return 1;
    }

    return 0;
}

long calibrate_ans(void)
{
    rt_kprintf("int16_t=%u ans:%hu,%hu,%hu\n", sizeof(int16_t), gs_calibrate_data.left, gs_calibrate_data.middle, gs_calibrate_data.right);
}
MSH_CMD_EXPORT(calibrate_ans, show calibrate info now);

int tcd1209_calibrate_triger(int times)
{
    if (likely(SCAN_TYPE_CONTINUOUS == gs_type4scan_mode))
    {
        if (likely(CHECK_END_TYPE == times))
        {
            /* TODO get calibrate ans and save */
            convert_raw2data_sync();
            save_calibrate_data();
            /* TODO clear original data */
            raw_data_clear();
        }
        else if (CHECK_START_TYPE == times)
        {

        }
        else
        {
            /* just ignore */
        }
    }
    else
    {
        if (CHECK_START_TYPE == times)
        {
            gs_type4scan_mode = SCAN_TYPE_CONTINUOUS;
        }
        else if (unlikely(CHECK_END_TYPE == times))
        {
            /* just ignore */
        }
        ccd_scan_restart();
        while(gs_index < AD9945_DATA_COUNTS);
        convert_raw2data_sync();
        save_calibrate_data();
    }

    return 0;
}

int tcd1209_get_delimiters_info(unsigned char *value, unsigned char len)
{
    value[0] = gs_sample_test.position.left >> 8;
    value[1] = gs_sample_test.position.left;
    value[2] = gs_sample_test.position.middle >> 8;
    value[3] = gs_sample_test.position.middle;
    value[4] = gs_sample_test.position.right >> 8;
    value[5] = gs_sample_test.position.right;

    return 0;
}

int tcd1209_set_calibrate_delta_info(unsigned char *value, unsigned char len)
{
    /* TODO check parameter valid */
    if ((value[0] << 8 | value[1]) > 4095)
    {
        return -1;
    }
    _gs_data_delta = value[0] << 8 | value[1];

    return 0;
}

int tcd1209_get_calibrate_delta_info(unsigned char *value, unsigned char len)
{
    /* TODO check parameter valid */
    value[0] = _gs_data_delta >> 8;
    value[1] = _gs_data_delta;

    return 0;
}
int tcd1209_set_delimiters_info(unsigned char *value, unsigned char len)
{
    /* TODO check parameter valid */
    if ((gs_sample_test.position.left > AD9945_DATA_COUNTS) ||
        (gs_sample_test.position.middle > AD9945_DATA_COUNTS) ||
        (gs_sample_test.position.right > AD9945_DATA_COUNTS) ||
        (gs_sample_test.position.left + gs_sample_test.position.middle + \
         gs_sample_test.position.right > AD9945_DATA_COUNTS))
    {
        return -1;
    }

    gs_sample_test.position.left   = value[0] << 8 | value[1];
    gs_sample_test.position.middle = value[2] << 8 | value[3];
    gs_sample_test.position.right  = value[4] << 8 | value[5];
    ef_set_abs_ccd_info(&gs_sample_test);
}

int tcd1209_calibrate_get_info(unsigned char *value, unsigned char len)
{
    if (likely(gs_sample_test.value.left + gs_sample_test.value.middle + gs_sample_test.value.right != 0))
    {
        value[0] = gs_sample_test.value.left >> 8 & 0xff;
        value[1] = gs_sample_test.value.left & 0xff;
        value[2] = gs_sample_test.value.middle >> 8 & 0xff;
        value[3] = gs_sample_test.value.middle & 0xff;
        value[4] = gs_sample_test.value.right >> 8 & 0xff;
        value[5] = gs_sample_test.value.right & 0xff;
        return 0;
    }
    else
    {
        return -1;
    }
}

int tcd1209_check_triger(int times)
{
    int ans = -1;
    if (likely(SCAN_TYPE_CONTINUOUS == gs_type4scan_mode))
    {
        if (likely(CHECK_END_TYPE == times))
        {
            /* TODO get check ans and compare */
            convert_raw2data_sync();
            ans = compare_with_calibrate();
            /* TODO clear original data */
            rt_kprintf("check(%d):%hd.%hd.%hd\n", ans, gs_sample_test.value.left,
                gs_sample_test.value.middle, gs_sample_test.value.right);
            rt_kprintf("calibrate check(%d):%hd.%hd.%hd\n", ans, gs_calibrate_data.left,
                gs_calibrate_data.middle, gs_calibrate_data.right);
            raw_data_clear();

        }
        else if (CHECK_START_TYPE == times)
        {

        }
        else
        {
            /* just ignore */
            //gs_type4scan_mode = SCAN_TYPE_ONESHOT;
        }
    }
    else
    {
        if (CHECK_START_TYPE == times)
        {
            gs_type4scan_mode = SCAN_TYPE_CONTINUOUS;
        }
        else if (unlikely(CHECK_END_TYPE == times))
        {
            /* TODO no support just */
        }
        ccd_scan_restart();
        while(gs_index < AD9945_DATA_COUNTS);
        convert_raw2data_sync();
    }

    return ans;
}

void DMA0_Channel0_IRQHandler(void)
{
    rt_interrupt_enter();
    if (SET == dma_flag_get(DMA0, DMA_CH0, DMA_FLAG_FTF))
    {
        dma_flag_clear(DMA0, DMA_CH0, DMA_FLAG_FTF);
        dma_channel_disable(DMA0, DMA_CH0);
        if (gs_sync4dma_flag & 0x10)
        {
            /* mark DMA transmit done */
            gs_index = AD9945_DATA_COUNTS;
            timer_disable(TIMER1);
            timer_interrupt_disable(TIMER7, TIMER_INT_CH3);
            raw_data_sync();
            rt_pin_write(21, gs_just_test_led++%2);
            if ((SCAN_TYPE_CONTINUOUS == gs_type4scan_mode) && (!_gs_catch_command))
            {
                convert_raw2data_sync();
                gs_sync4dma_flag = 0;
                ccd_scan_restart();
            }
        }
        else
        {
            /* mark DMA channel0 done */
            gs_sync4dma_flag |= 1;
        }
    }
    rt_interrupt_leave();
}

void DMA0_Channel4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    if (SET == dma_flag_get(DMA0, DMA_CH4, DMA_FLAG_FTF))
    {
        dma_flag_clear(DMA0, DMA_CH4, DMA_FLAG_FTF);
        dma_channel_disable(DMA0, DMA_CH4);
        if (gs_sync4dma_flag & 0x01)
        {
            /* mark DMA transmit done */
            gs_index = AD9945_DATA_COUNTS;
            timer_disable(TIMER1);
            timer_interrupt_disable(TIMER7, TIMER_INT_CH3);
            raw_data_sync();
            rt_pin_write(21, gs_just_test_led++%2);
            if ((SCAN_TYPE_CONTINUOUS == gs_type4scan_mode) && (!_gs_catch_command))
            {
                convert_raw2data_sync();
                ccd_scan_restart();
                gs_sync4dma_flag = 0;
            }
        }
        else
        {
            /* mark DMA channel4 done */
            gs_sync4dma_flag |= 0x10;
        }
    }
    /* leave interrupt */
    rt_interrupt_leave();
}

long show_ad9945(void)
{
    tcd1209_calibrate_triger(1);
    calibrate_ans();
    return 0;
}
MSH_CMD_EXPORT(show_ad9945, list device in system);

long adj_led(int argc, char *argv[])
{
    timer_channel_output_pulse_value_config(TIMER11, TIMER_CH_0, atoi(argv[1]));
}
MSH_CMD_EXPORT(adj_led, adjust led level);

/* 设置钳位 */
long adj_ob(int argc, char *argv[])
{
    if (argc > 1)
    {
        _set_ad9945_reg_value(0x02, 0xff & atoi(argv[1]));
        _set_ad9945_reg_value(0x0d, 0x838);
    }

    return 0;
}
MSH_CMD_EXPORT(adj_ob, adjust ob clamp level);
