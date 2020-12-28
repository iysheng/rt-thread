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

#define DBG_TAG    "drv.CCD"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

/* test code */
#if 1
#define BEEP_PIN_NUM    GET_PIN(C, 0)
void test_func(void)
{
    /* 蜂鸣器引脚为输出模式 */
    rt_pin_mode(BEEP_PIN_NUM, PIN_MODE_OUTPUT);
    /* 默认低电平 */
    rt_pin_write(BEEP_PIN_NUM, PIN_HIGH);
}

/*
 * static int init_timer1_4fm
 * 配置 TIM1 時鍾產生指定頻率的 PWM 波形
 *
 * @ unsigned int fm_freq:
 * return: errno/Linux
 */
static int init_timer1_4fm(unsigned int fm_freq)
{
    TIMER_BaseInitPara TIMER_Init = {0};
    GPIO_InitPara GPIO_InitStructure = {0};
    TIMER_OCInitPara TIMER_OCInit = {0};

    LOG_I("init timer1 with fm");

    rcu_periph_clock_enable(RCU_TIMER1);
    TIMER_Init.TIMER_Period                = 31;
    TIMER_Init.TIMER_Prescaler             = 1;
    TIMER_Init.TIMER_ClockDivision         = TIMER_CDIV_DIV1;
    TIMER_Init.TIMER_CounterMode           = TIMER_COUNTER_UP;
    TIMER_Init.TIMER_RepetitionCounter     = 0x0000;
    TIMER_BaseInit(TIMER1, &TIMER_Init);

    /* 初始化 PA1 */
    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

#if 1
    TIMER_OCInit.TIMER_OCMode = TIMER_OC_MODE_PWM1;
    TIMER_OCInit.TIMER_OutputState = TIMER_OUTPUT_STATE_ENABLE;
    //TIMER_OCInit.TIMER_OutputNState = TIMER_OUTPUTN_STATE_ENABLE;
    TIMER_OCInit.TIMER_Pulse = 15;
    TIMER_OCInit.TIMER_OCPolarity = TIMER_OC_POLARITY_HIGH;
    TIMER_OCInit.TIMER_OCNPolarity = TIMER_OCN_POLARITY_LOW;
    TIMER_OC2_Init(TIMER1, &TIMER_OCInit);
#endif
    TIMER_Enable(TIMER1, ENABLE);
    TIMER_CtrlPWMOutputs(TIMER1, ENABLE);
    /* SMC[2:0]=3'b000 */

    /* 使用分頻 CEN=1  PSC 寄存器設置分頻 */

    /* CHxCOMCTL bits to 3’b110 (PWM mode0) or to 3’b111(PWM mode1) */
}
#endif
/*
 * static int drv_tcd1304_init
 * TCD1304 CCD chip 初始化入口
 *
 * @ void:
 * return: errno/Linux
 */
static int drv_tcd1304_init(void)
{
    test_func();
    /* 測試產生 2MHz 的波形 */
    init_timer1_4fm(2000000);
    LOG_I("hello red");
}
/* 降低加載的優先級可以正常調試打印 */
INIT_DEVICE_EXPORT(drv_tcd1304_init);
