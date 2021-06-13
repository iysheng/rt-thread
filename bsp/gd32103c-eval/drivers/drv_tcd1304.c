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
#include <math.h>

#define DBG_TAG    "drv.CCD"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

#define CCD_FM_FREQ    2000000
#define CCD_DATA_LEN    3694
#define VALID_CCD_DATA_LEN    3648  /* valid data is [33, 3680] */

#define FM_GND0         GET_PIN(A, 0)
#define FM_GND1         GET_PIN(A, 2)
#define SH_GND0         GET_PIN(A, 6)
#define SH_GND1         GET_PIN(C, 4)
#define ICG_GND0        GET_PIN(B, 9)

static uint16_t g_tcd_convert_data[CCD_DATA_LEN];
static uint16_t g_tcd_convert_data_filter[VALID_CCD_DATA_LEN];
static uint16_t g_tcd_convert_data_filter4mark[VALID_CCD_DATA_LEN];
static uint16_t g_tcd_convert_data_filter4cmp[VALID_CCD_DATA_LEN];

typedef struct {
    struct rt_semaphore sem;
    int ans;
} ccd_check_sync_t;

static ccd_check_sync_t gs_ccd_sync;


typedef struct {
    uint16_t ans_zone0; /* [0...1000) */
    uint16_t ans_zone1; /* [1000...2647] */
    uint16_t ans_zone2; /* (2647,3647] */
} drv_tcd1304_target_ans_t;

/* 存储标记的区域像素信息 */
drv_tcd1304_target_ans_t gs_tcd_mark_target_ans;
/* 存储标记的区域像素信息 */
static drv_tcd1304_target_ans_t gs_tcd_cmp_target_ans;

static uint16_t _get_stand_value(uint16_t *data, int data_len)
{
    /* 初始化参考的标准值 */
    double stand_value = 0;
    double value_counts = 0, value4tmp = 0;
    int i = 0;

    for (; i < data_len; i++)
    {
        value4tmp = (double)data[i];
        value_counts += value4tmp * value4tmp;
    }
    value_counts /= data_len;
    stand_value = (uint16_t)sqrt(value_counts);

    return stand_value;
}

static int do_format_data(uint16_t *data, int data_len)
{
    int i = 0;
    uint16_t stand_value = _get_stand_value(data, data_len);
#if 0
    rt_kprintf("stand=%d", stand_value);
#endif

    for (; i < data_len; i++)
    {
        data[i] = (data[i] > stand_value) ? 1 : 0;
    }

    return 0;
}

static int do_xor_with_data(uint16_t *target, uint16_t *data, int data_len)
{
    int i = 0;

    for (; i < data_len; i++)
    {
        target[i] ^= data[i];
    }

    return 0;
}

/* 统计 0 / 1 数量 */
static int do_get_target_ans(uint16_t *target, int data_len, drv_tcd1304_target_ans_t *ans_data)
{
    int i = 0;
    static const int cmp_index_array[2] = {1000, 2647};

    for (; i < data_len; i++)
    {
        if (i < cmp_index_array[0])
        {
            ans_data->ans_zone0 += !!target[i];
        }
        else if (i < cmp_index_array[1])
        {
            ans_data->ans_zone1 += !!target[i];
        }
        else
        {
            ans_data->ans_zone2 += !!target[i];
        }
    }

    LOG_I("target_ans=[%u,%u,%u]", ans_data->ans_zone0, ans_data->ans_zone1, ans_data->ans_zone2);
    return 0;
}

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
    int mark_times;
} drv_tcd1304_data_t;

static drv_tcd1304_data_t g_tcd1304_device_data = {
    .should_scan = 0,
    .mark_times = 0, /* 默认标定的时候采样次数为 0 */
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
    rt_memset(&g_tcd_convert_data_filter4cmp[0], 0, sizeof(uint16_t) * VALID_CCD_DATA_LEN);
    g_tcd1304_device_data.should_scan = data;
    return 0;
}

/*
 * int set_tcd1304_device_marktimes
 * 设置 tcd1304 设备的控制数据
 *
 * @ data:
 * return: errno/Linux
 */
int set_tcd1304_device_marktimes(int data)
{
    rt_memset(&g_tcd_convert_data_filter4mark[0], 0, sizeof(uint16_t) * VALID_CCD_DATA_LEN);
    rt_memset(&g_tcd_convert_data_filter4cmp[0], 0, sizeof(uint16_t) * VALID_CCD_DATA_LEN);
    g_tcd1304_device_data.mark_times = data;
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
    /* 如果需要对采样的数据 AD 转换 */
    if (g_tcd1304_device_data.should_scan > 0)
    {
#if 0
        rt_kprintf("should_scan=%d\n", g_tcd1304_device_data.should_scan);
        if ((g_tcd1304_device_data.mark_times != 0 && g_tcd1304_device_data.should_scan == g_tcd1304_device_data.mark_times) || g_tcd1304_device_data.mark_times == 0)
        {
#endif
            g_tcd1304_device_data.should_scan--;
            DMA_SetCurrDataCounter(DMA0_CHANNEL1, 0);
            TIMER_Enable(TIMER0, ENABLE);
#if 0
        }
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
        (RCC_ClocksState.APB1_Frequency << 1) / CCD_FM_FREQ - 1;
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
    /* make the scale to 1/2 * 1/CCD_FM_FREQ
     * when CCD_FM_FREQ == 2Mhz
     * then = 0.5us / 2 = 0.25us  */
    TIMER_Init.TIMER_Prescaler             = \
        (RCC_ClocksState.APB1_Frequency << 1) / CCD_FM_FREQ / 2 - 1;
    TIMER_Init.TIMER_Period                = sh_freq * 2 - 1;
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
    /* 0.25 * 6 us */
    TIMER_OCInit.TIMER_Pulse = 5;
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

static void show_voltage(char * title, uint16_t * adc_value, int counts)
{
	int i = 0;

    rt_kprintf("%s begin\n",title);
	for (; i < counts; i++)
	{
		rt_kprintf("%d\t%u\n", i, adc_value[i]);
	}
    rt_kprintf("%s end\n", title);
}

/**
  * @brief 
  * 
  * @param uint16_t * src: 
  * @param uint16_t *dst: 
  * @param int len: 
  * @param float alpha: 
  * retval .
  */
void do_filter_with_lowpass(uint16_t * src, uint16_t *dst, int len, float alpha)
{
    int i = 1;
    float tmp_data_raw, tmp_data_filter;

    /* 提取有效的数据 [33...3680]*/
    rt_memmove(src, src + 33, len);

    dst[0] = src[0];
    for (; i < len; i++)
    {
        tmp_data_filter = dst[i - 1];
        tmp_data_raw = src[i];
        tmp_data_filter = tmp_data_filter + (alpha * (tmp_data_raw - tmp_data_filter));
        dst[i] = tmp_data_filter;
    }
}

static int check_whether_match_target(drv_tcd1304_target_ans_t *mark, drv_tcd1304_target_ans_t *cmp)
{
#define COMPARE_THREOLD    150
    int match = 0;

    if (mark->ans_zone0 > cmp->ans_zone0)
    {
        match += (mark->ans_zone0 - cmp->ans_zone0 > COMPARE_THREOLD);
    }
    else
    {
        match += (cmp->ans_zone0 - mark->ans_zone0 > COMPARE_THREOLD);
    }

    if (mark->ans_zone1 > cmp->ans_zone1)
    {
        match += (mark->ans_zone1 - cmp->ans_zone1 > COMPARE_THREOLD);
    }
    else
    {
        match += (cmp->ans_zone1 - mark->ans_zone1 > COMPARE_THREOLD);
    }

    if (mark->ans_zone2 > cmp->ans_zone2)
    {
        match += (mark->ans_zone2 - cmp->ans_zone2 > COMPARE_THREOLD);
    }
    else
    {
        match += (cmp->ans_zone2 - mark->ans_zone2 > COMPARE_THREOLD);
    }

    LOG_I(">--------------no match counts=%d.", match);

    return match;
}

/*
 * 获取检测的结果
 * 1 : 匹配
 * 0 : 不匹配
 * < 0 : 错误
 * */
int get_ccd_check_ans(void)
{
    if (RT_EOK == rt_sem_take(&gs_ccd_sync.sem, RT_TICK_PER_SECOND))
    {
        return gs_ccd_sync.ans;
    }
    else
    {
        return -RT_ERROR;
    }
}

void DMA0_Channel0_IRQHandler(void)
{
    unsigned int voltage = 0;
    /* enter interrupt */
    rt_interrupt_enter();
    TIMER_Enable(TIMER0, DISABLE);
    DMA_ClearIntBitState(DMA1_INT_GL1 | DMA1_INT_TC1 | DMA1_INT_ERR1);
#if 0
    show_voltage("raw", g_tcd_convert_data, CCD_DATA_LEN);
#endif
    do_filter_with_lowpass(g_tcd_convert_data, g_tcd_convert_data_filter, VALID_CCD_DATA_LEN, 0.1);
#if 0
    show_voltage("filter", g_tcd_convert_data_filter, VALID_CCD_DATA_LEN);
#endif
    do_format_data(g_tcd_convert_data_filter, VALID_CCD_DATA_LEN);
#if 1
    /* TODO check whether use filtered data mark stand */
    if (g_tcd1304_device_data.mark_times > 0)
    {
        do_xor_with_data(g_tcd_convert_data_filter4mark, g_tcd_convert_data_filter, VALID_CCD_DATA_LEN);
        if (--g_tcd1304_device_data.mark_times == 0)
        {
            rt_memset(&gs_tcd_mark_target_ans, 0, sizeof(gs_tcd_mark_target_ans));
            do_get_target_ans(g_tcd_convert_data_filter4mark, VALID_CCD_DATA_LEN, &gs_tcd_mark_target_ans);
        }
    }
    else
    {
        do_xor_with_data(g_tcd_convert_data_filter4cmp, g_tcd_convert_data_filter, VALID_CCD_DATA_LEN);
        if (g_tcd1304_device_data.should_scan == 0)
        {
            rt_memset(&gs_tcd_cmp_target_ans, 0, sizeof(gs_tcd_cmp_target_ans));
            do_get_target_ans(g_tcd_convert_data_filter4cmp, VALID_CCD_DATA_LEN, &gs_tcd_cmp_target_ans);
            /* check whether match */
            gs_ccd_sync.ans = check_whether_match_target(&gs_tcd_mark_target_ans, &gs_tcd_cmp_target_ans);
            if (gs_ccd_sync.ans)
            {
                LOG_I("no match");
            }
            else
            {
                LOG_W("match to alarm");
            }
            /* 释放信号量 */
            rt_sem_release(&gs_ccd_sync.sem);
        }
    }
#endif
#if 0
    show_voltage("filter", g_tcd_convert_data_filter, CCD_DATA_LEN);
#endif
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
    DMA_InitParaStruct.DMA_MemoryDataSize = DMA_MEMORYDATASIZE_HALFWORD;
    DMA_InitParaStruct.DMA_Mode = DMA_MODE_CIRCULAR;
    DMA_InitParaStruct.DMA_Priority = DMA_PRIORITY_HIGH;
    DMA_InitParaStruct.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
    /* 使用 DMA0 的通道1 */
    DMA_Init(DMA0_CHANNEL1, &DMA_InitParaStruct);
    DMA_INTConfig(DMA0_CHANNEL1, DMA_INT_TC | DMA_INT_ERR, ENABLE);
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

    ADC_Calibration(ADC0);
    ADC_RegularChannel_Config(ADC0, ADC_CHANNEL_15, 1, ADC_SAMPLETIME_1POINT5);
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
    rt_sem_init(&gs_ccd_sync.sem, "check_sem", 0, RT_IPC_FLAG_FIFO);
    test_func();
    /* 測試產生 2MHz 的波形 */
    init_timer4_4fm(CCD_FM_FREQ);
    /* 2M/4 = 500kHz  */
    init_timer2_4sh(4);
    /* 2M/4/3694 = 135Hz  */
    init_timer3_4icg(4*3694);
    /* init_timer0_4adc */
    init_adc_4tcd(CCD_FM_FREQ / 4);
    LOG_I("hello red");
}
/* 降低加載的優先級可以正常調試打印 */
INIT_DEVICE_EXPORT(drv_tcd1304_init);
