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

#define CCD_FM_FREQ    1000000

/* test code */
#if 1
#define BEEP_PIN_NUM    GET_PIN(C, 0)

#define LED4_PIN_NUM    GET_PIN(B, 1)
#define LED3_PIN_NUM    GET_PIN(B, 2)
#define LED2_PIN_NUM    GET_PIN(B, 10)
#define LED1_PIN_NUM    GET_PIN(B, 11)

#define FM_GND0         GET_PIN(A, 0)
#define FM_GND1         GET_PIN(A, 2)
#define SH_GND0         GET_PIN(A, 6)
#define SH_GND1         GET_PIN(C, 4)
#define ICG_GND0        GET_PIN(B, 9)

void test_func(void)
{
    /* 蜂鸣器引脚为输出模式 */
    rt_pin_mode(FM_GND0, PIN_MODE_OUTPUT);
    rt_pin_mode(FM_GND1, PIN_MODE_OUTPUT);
    rt_pin_mode(SH_GND0, PIN_MODE_OUTPUT);
    rt_pin_mode(SH_GND1, PIN_MODE_OUTPUT);
    rt_pin_mode(ICG_GND0, PIN_MODE_OUTPUT);
    /* 默认低电平 */
    rt_pin_write(FM_GND0, PIN_LOW);
    rt_pin_write(FM_GND1, PIN_LOW);
    rt_pin_write(SH_GND0, PIN_LOW);
    rt_pin_write(SH_GND1, PIN_LOW);
    rt_pin_write(ICG_GND0, PIN_LOW);

    /* 開啓補光 */
    rt_pin_mode(LED4_PIN_NUM, PIN_MODE_OUTPUT);
    rt_pin_mode(LED3_PIN_NUM, PIN_MODE_OUTPUT);
    rt_pin_mode(LED2_PIN_NUM, PIN_MODE_OUTPUT);
    rt_pin_mode(LED1_PIN_NUM, PIN_MODE_OUTPUT);
#if 1
    rt_pin_write(LED4_PIN_NUM, PIN_LOW);
    rt_pin_write(LED3_PIN_NUM, PIN_LOW);
    rt_pin_write(LED2_PIN_NUM, PIN_HIGH);
    rt_pin_write(LED1_PIN_NUM, PIN_HIGH);
#endif
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
    RCC_ClocksPara RCC_ClocksState = {0};

    LOG_I("init timer1 with fm");

    RCC_GetClocksFreq(&RCC_ClocksState);
    rcu_periph_clock_enable(RCU_TIMER1);
    TIMER_InternalClockConfig(RCU_TIMER1);
    TIMER_Init.TIMER_Period                = \
		(RCC_ClocksState.APB1_Frequency  << 1 ) / fm_freq - 1;
	LOG_I("apb1=%u", RCC_ClocksState.APB1_Frequency);
	LOG_I("apb2=%u", RCC_ClocksState.APB2_Frequency);
	LOG_I("ahb=%u", RCC_ClocksState.AHB_Frequency);
    TIMER_Init.TIMER_Prescaler             = 0;
    TIMER_Init.TIMER_ClockDivision         = TIMER_CDIV_DIV1;
    TIMER_Init.TIMER_CounterMode           = TIMER_COUNTER_UP;
    TIMER_BaseInit(TIMER1, &TIMER_Init);

    /* 初始化 PA1 */
    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

#if 1
    TIMER_OCInit.TIMER_OCMode = TIMER_OC_MODE_PWM1;
    TIMER_OCInit.TIMER_OutputState = TIMER_OUTPUT_STATE_ENABLE;
    TIMER_OCInit.TIMER_Pulse = \
        RCC_ClocksState.APB1_Frequency  / fm_freq;
    TIMER_OC2_Init(TIMER1, &TIMER_OCInit);
#endif
    /* 開啓計數 */
    TIMER_Enable(TIMER1, ENABLE);
#if 0
    TIMER_CtrlPWMOutputs(TIMER1, ENABLE);
#endif
    /* SMC[2:0]=3'b000 */

    /* 使用分頻 CEN=1  PSC 寄存器設置分頻 */

    /* CHxCOMCTL bits to 3’b110 (PWM mode0) or to 3’b111(PWM mode1) */
}

static int init_timer3_4icg(unsigned int icg_freq)
{
    TIMER_BaseInitPara TIMER_Init = {0};
    GPIO_InitPara GPIO_InitStructure = {0};
    TIMER_OCInitPara TIMER_OCInit = {0};
    RCC_ClocksPara RCC_ClocksState = {0};

    LOG_I("init timer3 with icg");

    RCC_GetClocksFreq(&RCC_ClocksState);
    rcu_periph_clock_enable(RCU_TIMER3);
    TIMER_InternalClockConfig(RCU_TIMER3);
    TIMER_Init.TIMER_Period                = icg_freq - 1;
    TIMER_Init.TIMER_Prescaler             = \
        RCC_ClocksState.APB2_Frequency / CCD_FM_FREQ - 1;
    TIMER_Init.TIMER_ClockDivision         = TIMER_CDIV_DIV1;
    TIMER_Init.TIMER_CounterMode           = TIMER_COUNTER_UP;
    TIMER_Init.TIMER_RepetitionCounter     = 0x0000;
    TIMER_BaseInit(TIMER3, &TIMER_Init);

    /* 初始化 PB8 */
    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIMER_OCInit.TIMER_OCMode = TIMER_OC_MODE_PWM1;
    TIMER_OCInit.TIMER_OutputState = TIMER_OUTPUT_STATE_ENABLE;
    TIMER_OCInit.TIMER_Pulse = \
        (5 * CCD_FM_FREQ) / 1000000;
#if 0
    TIMER_OCInit.TIMER_Pulse = (icg_freq >> 1);
#else
    TIMER_OCInit.TIMER_Pulse = \
        ((10 * CCD_FM_FREQ) / 1000000);
#endif
    TIMER_OCInit.TIMER_OCIdleState = TIMER_OC_IDLE_STATE_RESET;
    TIMER_OC3_Init(TIMER3, &TIMER_OCInit);
    TIMER_Enable(TIMER3, ENABLE);
    TIMER_Enable(TIMER2, ENABLE);
#if 0
    TIMER_CtrlPWMOutputs(TIMER3, ENABLE);
#endif
}

static int init_timer2_4sh(unsigned int sh_freq)
{
    TIMER_BaseInitPara TIMER_Init = {0};
    GPIO_InitPara GPIO_InitStructure = {0};
    TIMER_OCInitPara TIMER_OCInit = {0};
    RCC_ClocksPara RCC_ClocksState = {0};

    LOG_I("init timer2 with sh");

    RCC_GetClocksFreq(&RCC_ClocksState);
    rcu_periph_clock_enable(RCU_TIMER2);
    TIMER_InternalClockConfig(RCU_TIMER2);
    TIMER_Init.TIMER_Prescaler             = \
        RCC_ClocksState.APB2_Frequency / CCD_FM_FREQ - 1;
    TIMER_Init.TIMER_Period                = sh_freq - 1;
    TIMER_Init.TIMER_ClockDivision         = TIMER_CDIV_DIV1;
    TIMER_Init.TIMER_CounterMode           = TIMER_COUNTER_UP;
    TIMER_Init.TIMER_RepetitionCounter     = 0x0000;
    TIMER_BaseInit(TIMER2, &TIMER_Init);

    /* 初始化 PA7 */
    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIMER_OCInit.TIMER_OCMode = TIMER_OC_MODE_PWM2;
    TIMER_OCInit.TIMER_OutputState = TIMER_OUTPUT_STATE_ENABLE;
    TIMER_OCInit.TIMER_Pulse = (3 * CCD_FM_FREQ) / 1000000;
    TIMER_OCInit.TIMER_OCPolarity = TIMER_OC_POLARITY_HIGH;
    TIMER_OCInit.TIMER_OCIdleState = TIMER_OC_IDLE_STATE_RESET;
    TIMER_OC2_Init(TIMER2, &TIMER_OCInit);
#if 0
    TIMER_Enable(TIMER2, ENABLE);
    TIMER_CtrlPWMOutputs(TIMER2, ENABLE);
#endif
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
    init_timer1_4fm(CCD_FM_FREQ);
	/* 2M/30 = 66kHz  */
    init_timer2_4sh(10);
	/* 2M/12000 = 166Hz  */
    init_timer3_4icg(36900);
    LOG_I("hello red");
}
/* 降低加載的優先級可以正常調試打印 */
INIT_DEVICE_EXPORT(drv_tcd1304_init);
