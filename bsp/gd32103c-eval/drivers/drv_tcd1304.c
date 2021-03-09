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
#define CCD_DATA_LEN    3694

#define FM_GND0         GET_PIN(A, 0)
#define FM_GND1         GET_PIN(A, 2)
#define SH_GND0         GET_PIN(A, 6)
#define SH_GND1         GET_PIN(C, 4)
#define ICG_GND0        GET_PIN(B, 9)

static uint16_t g_tcd_convert_data[CCD_DATA_LEN];

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
}

/*
 * static int init_timer4_4fm
 * 配置 TIM4 時鍾產生指定頻率的 PWM 波形
 *
 * @ unsigned int fm_freq:
 * return: errno/Linux
 */
static int init_timer4_4fm(unsigned int fm_freq)
{
    TIMER_BaseInitPara TIMER_Init = {0};
    GPIO_InitPara GPIO_InitStructure = {0};
    TIMER_OCInitPara TIMER_OCInit = {0};
    RCC_ClocksPara RCC_ClocksState = {0};

    LOG_I("init timer4 with fm");

    RCC_GetClocksFreq(&RCC_ClocksState);
    rcu_periph_clock_enable(RCU_TIMER4);
    TIMER_InternalClockConfig(TIMER4);
    TIMER_Init.TIMER_Period                = \
		(RCC_ClocksState.APB1_Frequency  << 1 ) / fm_freq - 1;
	LOG_I("apb1=%u", RCC_ClocksState.APB1_Frequency);
	LOG_I("apb2=%u", RCC_ClocksState.APB2_Frequency);
	LOG_I("ahb=%u", RCC_ClocksState.AHB_Frequency);
    TIMER_Init.TIMER_Prescaler             = 0;
    TIMER_Init.TIMER_ClockDivision         = TIMER_CDIV_DIV1;
    TIMER_Init.TIMER_CounterMode           = TIMER_COUNTER_UP;
    TIMER_BaseInit(TIMER4, &TIMER_Init);

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
    TIMER_OC2_Init(TIMER4, &TIMER_OCInit);
#endif
    /* 開啓計數 */
    TIMER_Enable(TIMER4, ENABLE);
#if 0
    TIMER_CtrlPWMOutputs(TIMER4, ENABLE);
#endif
    /* SMC[2:0]=3'b000 */

    /* 使用分頻 CEN=1  PSC 寄存器設置分頻 */

    /* CHxCOMCTL bits to 3’b110 (PWM mode0) or to 3’b111(PWM mode1) */
}

typedef struct {
    int should_scan;
} drv_tcd1304_data_t;

static drv_tcd1304_data_t g_tcd1304_device_data = {
	.should_scan = 0,
};

/*
 * int set_tcd1304_device_data
 * 设置 tcd1304 设备的控制数据
 *
 * @ data:
 * return: errno/Linux
 */
int set_tcd1304_device_data(int data)
{
	g_tcd1304_device_data.should_scan = data;
	return 0;
}

/*
 * int get_tcd1304_device_data
 * 获取 1304 设备的控制数据
 *
 * @ void:
 * return: errno/Linux
 */
int get_tcd1304_device_data(void)
{
	return g_tcd1304_device_data.should_scan;
}

void TIMER3_IRQHandler(void)
{
    rt_interrupt_enter();
    TIMER_ClearIntBitState(TIMER3, TIMER_INT_UPDATE);
	if (g_tcd1304_device_data.should_scan)
	{
		rt_kprintf("scan now.\n");
        TIMER_Enable(TIMER0, ENABLE);
		g_tcd1304_device_data.should_scan--;
	}
	else
	{
#if 1
        TIMER_Enable(TIMER0, DISABLE);
#endif
	}
    rt_interrupt_leave();
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
    TIMER_InternalClockConfig(TIMER3);
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
    /* TODO 开启定时器的更新中断，在中断处理函数中需要做一些事情 */
    TIMER_INTConfig(TIMER3, TIMER_INT_UPDATE, ENABLE);
    NVIC_SetPriority(TIMER3_IRQn, 0);
#if 0
    NVIC_EnableIRQ(TIMER3_IRQn);
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
    TIMER_InternalClockConfig(TIMER2);
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

/*
 * static int init_timer0_4adc
 * 配置 TIM0 時鍾產生指定頻率的时钟，作为 ADC 的时钟源
 *
 * @ unsigned int freq:
 * return: errno/Linux
 */
static int init_timer0_4adc(unsigned int freq)
{
    TIMER_BaseInitPara TIMER_Init = {0};
    GPIO_InitPara GPIO_InitStructure = {0};
    TIMER_OCInitPara TIMER_OCInit = {0};
    RCC_ClocksPara RCC_ClocksState = {0};

    LOG_I("init timer0 with adc");

    RCC_GetClocksFreq(&RCC_ClocksState);
    rcu_periph_clock_enable(RCU_TIMER0);
    TIMER_InternalClockConfig(TIMER0);
    TIMER_Init.TIMER_Period                = \
        RCC_ClocksState.APB2_Frequency / freq - 1;
    LOG_I("timer0 apb2=%u", RCC_ClocksState.APB2_Frequency);
    TIMER_Init.TIMER_Prescaler             = 0;
    TIMER_Init.TIMER_ClockDivision         = TIMER_CDIV_DIV1;
    TIMER_Init.TIMER_CounterMode           = TIMER_COUNTER_UP;
    TIMER_BaseInit(TIMER0, &TIMER_Init);

#if 1
    /* 初始化 PA8 */
    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
#else
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_GPIOE);
    GPIO_PinRemapConfig(GPIO_FULL_REMAP_TIMER1, ENABLE);
    /* 初始化 PE9 */
    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
#endif

    TIMER_OCInit.TIMER_OCMode = TIMER_OC_MODE_PWM1;
    TIMER_OCInit.TIMER_OutputState = TIMER_OUTPUT_STATE_ENABLE;
    TIMER_OCInit.TIMER_Pulse = TIMER_Init.TIMER_Period >> 1;
    TIMER_OC1_Init(TIMER0, &TIMER_OCInit);
#if 0
    /* 開啓計數 */
    TIMER_Enable(TIMER0, ENABLE);
#endif
    /* 高级定时器 0 和定时器 7 需要开启这个才会输出 PWM */
    TIMER_CtrlPWMOutputs(TIMER0, ENABLE);

    return 0;
}

void DMA0_Channel0_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();
    DMA_ClearIntBitState(DMA1_INT_GL1 | DMA1_INT_TC1 | DMA1_INT_HT1 | DMA1_INT_ERR1 );
    rt_kprintf("DMA handler catch ad_value[0]=%d.\n", 0xfff & g_tcd_convert_data[50]);
    rt_kprintf("DMA handler catch ad_value[1]=%d.\n", 0xfff & g_tcd_convert_data[51]);
    rt_kprintf("DMA handler catch ad_value[2]=%d.\n", 0xfff & g_tcd_convert_data[52]);
    /* leave interrupt */
    rt_interrupt_leave();
}

/*
 * static int init_adc_4tcd
 * 以指定的周期初始化定时器关联 AD 采样
 *
 * @ unsigned int freq:
 * return: errno/Linux
 */
static int init_adc_4tcd(unsigned int freq)
{
    int ret;
    GPIO_InitPara GPIO_InitStructure = {0};
    ADC_InitPara ADC_InitParaStruct;
    DMA_InitPara DMA_InitParaStruct;

    ret = init_timer0_4adc(freq);
    if (ret)
    {
        LOG_E("failed init timer0 for adc.");
        return ret;
    }

#if 1
    /* DMA 配置 */
    rcu_periph_clock_enable(RCU_DMA0);
    DMA_InitParaStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC0->RDTR;
    DMA_InitParaStruct.DMA_MemoryBaseAddr = (uint32_t)&g_tcd_convert_data[0];
    DMA_InitParaStruct.DMA_DIR = DMA_DIR_PERIPHERALSRC;
    DMA_InitParaStruct.DMA_BufferSize = CCD_DATA_LEN;
    DMA_InitParaStruct.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
    DMA_InitParaStruct.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
    DMA_InitParaStruct.DMA_PeripheralDataSize = DMA_PERIPHERALDATASIZE_HALFWORD;
    DMA_InitParaStruct.DMA_MemoryDataSize = DMA_PERIPHERALDATASIZE_HALFWORD;
    DMA_InitParaStruct.DMA_Mode = DMA_MODE_CIRCULAR;
    DMA_InitParaStruct.DMA_Priority = DMA_PRIORITY_HIGH;
    DMA_InitParaStruct.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
    /* 使用 DMA0 的通道1 */
    DMA_Init(DMA0_CHANNEL1, &DMA_InitParaStruct);
    DMA_INTConfig(DMA0_CHANNEL1, DMA_INT_TC | DMA_INT_HT | DMA_INT_ERR, ENABLE);
    NVIC_SetPriority(DMA0_Channel1_IRQn, 0);
    NVIC_EnableIRQ(DMA0_Channel1_IRQn);
    DMA_Enable(DMA0_CHANNEL1, ENABLE);

    /* ADC 配置 */
    rcu_periph_clock_enable(RCU_GPIOC);
    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    rcu_periph_clock_enable(RCU_ADC0);
    ADC_InitParaStruct.ADC_Trig_External = ADC_EXTERNAL_TRIGGER_MODE_T1_CC1;
    ADC_InitParaStruct.ADC_Channel_Number = 16;
    ADC_InitParaStruct.ADC_Data_Align = ADC_DATAALIGN_RIGHT;
    ADC_InitParaStruct.ADC_Mode_Scan = DISABLE;
    ADC_InitParaStruct.ADC_Mode = ADC_MODE_INDEPENDENT;
    ADC_InitParaStruct.ADC_Mode_Continuous = DISABLE;
    ADC_Init(ADC0, &ADC_InitParaStruct);
    ADC_Enable(ADC0, ENABLE);
    ADC_RegularChannel_Config(ADC0, ADC_CHANNEL_15, 1, ADC_SAMPLETIME_55POINT5);
    ADC_ExternalTrigConv_Enable(ADC0, ENABLE);
    LOG_I("enable external clock for adc.");

    /* AD 转换完成就会出发 DMA 请求 */
    ADC_DMA_Enable(ADC0, ENABLE);
#endif
}

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
    init_timer4_4fm(CCD_FM_FREQ);
    /* 2M/30 = 66kHz  */
    init_timer2_4sh(10);
    /* 2M/12000 = 166Hz  */
    init_timer3_4icg(36900);
    /* init_timer0_4adc */
    init_adc_4tcd(CCD_FM_FREQ / 4);
    LOG_I("hello red");
}
/* 降低加載的優先級可以正常調試打印 */
INIT_DEVICE_EXPORT(drv_tcd1304_init);
