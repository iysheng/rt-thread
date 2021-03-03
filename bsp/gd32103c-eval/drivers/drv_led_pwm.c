/******************************************************************************
* File:             drv_tcd1304.c
*
* Author:           iysheng
* Created:          12/26/20
* Description:      TCD1304 CCD driver
*****************************************************************************/

#include <gd32f10x.h>
#include <board.h>
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG    "drv.led"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

#ifndef LED_PWM_MAX
#define LED_PWM_MAX 10000
#endif
static rt_uint32_t g_led_pwm_frq_max = LED_PWM_MAX;
static rt_uint32_t g_led_pwm_pin_value;
#define LED_PWM_PIN_NUM    GET_PIN(B, 11)

/*
 * static int init_timer1_4led
 * 配置 TIM4 時鍾產生指定頻率的 PWM 波形
 *
 * @ unsigned int freq:
 * return: errno/Linux
 */
static int init_timer1_4led(unsigned int freq)
{
    TIMER_BaseInitPara TIMER_Init = {0};
    GPIO_InitPara GPIO_InitStructure = {0};
    TIMER_OCInitPara TIMER_OCInit = {0};
    RCC_ClocksPara RCC_ClocksState = {0};

    LOG_I("init timer1 with led");

    if (!freq)
    {
        /* 如果频率为 0 直接关闭定时器 */
        rcu_periph_clock_disable(RCU_TIMER1);
        return 0;
    }
    rcu_periph_clock_enable(RCU_TIMER1);
    rcu_periph_clock_enable(RCU_GPIOB);
    RCC_GetClocksFreq(&RCC_ClocksState);

    TIMER_InternalClockConfig(TIMER1);
    /* 根据 usr manual
     * 因为 APB1 的时钟分频是 2, 所以这里定时器的时钟源频率实际是 2 * fapb1  */
    TIMER_Init.TIMER_Period                = \
        500000 / freq - 1;
    /*
     * 强制将定时器的时钟分频到 500KHz 来满足 [20 ~ 20KHz] 的 PWM 频率输出
     * */
    TIMER_Init.TIMER_Prescaler             = (RCC_ClocksState.APB1_Frequency << 2) / 1000000 - 1;
    TIMER_Init.TIMER_ClockDivision         = TIMER_CDIV_DIV1;
    TIMER_Init.TIMER_CounterMode           = TIMER_COUNTER_UP;
    TIMER_BaseInit(TIMER1, &TIMER_Init);

    /* remap PB11 to TIMER1_CH3 */
    rcu_periph_clock_enable(RCU_AF);
    GPIO_PinRemapConfig(GPIO_FULL_REMAP_TIMER2, ENABLE);
    /* 初始化 PB11 */
    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIMER_OCInit.TIMER_OCMode = TIMER_OC_MODE_PWM1;
    TIMER_OCInit.TIMER_OutputState = TIMER_OUTPUT_STATE_ENABLE;
    TIMER_OCInit.TIMER_Pulse = \
        TIMER_Init.TIMER_Period >> 1;
    TIMER_OC4_Init(TIMER1, &TIMER_OCInit);
    /* 開啓計數 */
    TIMER_Enable(TIMER1, ENABLE);
}

/*
 * static int drv_led_pwm_init
 * TCD1304 CCD chip 初始化入口
 *
 * @ void:
 * return: errno/Linux
 */
static int drv_led_pwm_init(void)
{
    /* 上电设置亮度最大 */
    init_timer1_4led(LED_PWM_MAX);
}
/* 降低加載的優先級可以正常調試打印 */
INIT_DEVICE_EXPORT(drv_led_pwm_init);

#ifdef RT_USING_FINSH

#include "finsh.h"

/*
 * static int do_with_led_light
 * 控制補光相關的命令
 *
 * @ int light_level:
 * return: errno/Linux
 */
static int do_with_led_light(int light_level)
{
    int ret = 0;

    if (light_level > 100)
    {
        light_level = 100;
    }
    else if (light_level < 0)
    {
        /* TODO with light level */
        light_level = 0;
    }

    if (light_level)
    {
        /* TODO just init peroid and duty */
        init_timer1_4led(LED_PWM_MAX * light_level / 100);
    }
    else
    {
        init_timer1_4led(0);
    }

    return ret;
}

static long tcd_pwm_light(int argc, char * argv[])
{
    int light_level;
    rt_kprintf("Hello led pwm light!\n");

    if (argc > 1)
    {
        light_level = atoi(argv[1]);
        do_with_led_light(light_level);
    }
    else
    {
        rt_kprintf("usage: tcd_light level[0..100]\n");
        return -1;
    }

    return 0;
}
MSH_CMD_EXPORT(tcd_pwm_light, control light 4 tcd1304);
#endif
