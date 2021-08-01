/******************************************************************************
* File:             drv_keyboard.c
*
* Author:           iysheng@163.com
* Created:          08/01/21
*                   按键板驱动
*****************************************************************************/

#include <rtdevice.h>
#include <drv_gpio.h>
#include "drv_keyboard.h"

#define DBG_LEVEL DBG_WARNING
#define DBG_TAG   "drv.key"
#include <rtdbg.h>

#define KEYBOARD_E     GET_PIN(E, 11)
#define KEYBOARD_COM   GET_PIN(E, 10)
#define KEYBOARD_S0    GET_PIN(E, 12)
#define KEYBOARD_S1    GET_PIN(E, 13)
#define KEYBOARD_S2    GET_PIN(E, 14)
#define KEYBOARD_S3    GET_PIN(E, 15)


typedef struct {
    unsigned short key_value;
    unsigned short key_value_index;
    unsigned char key_value_buffer[16];
} keyboard_key_t;
static keyboard_key_t gs_keyboard_value;

static rt_timer_t gs_timer4keyboard;

/**
  * @brief
  * @param uint32_t *value:
  * retval .
  */
int get_keyboard_keydown(uint32_t *value)
{
    /* TODO check value whether valid */
    if (gs_keyboard_value.key_value_index)
    {
        *value = gs_keyboard_value.key_value_buffer[--gs_keyboard_value.key_value_index];
        return 0;
    }

    return -1;
}

/**
  * @brief 设置按键的数值
  * @param uint32_t value:
  * retval .
  */
static void set_key_marix_value(uint32_t value)
{
    GPIO_OCTL(GPIOE) &= ~(uint32_t)(0xf << 12);
    GPIO_OCTL(GPIOE) |= (uint32_t)value << 12;
}

static void scan_func4keybaord(void *parameter)
{
    if (PIN_HIGH == rt_pin_read(KEYBOARD_COM))
    {
        LOG_I("Wow key=%hu", gs_keyboard_value.key_value);
        gs_keyboard_value.key_value_buffer[gs_keyboard_value.key_value_index++] = gs_keyboard_value.key_value;
    }
    gs_keyboard_value.key_value++;
    gs_keyboard_value.key_value &= 0xf;
    set_key_marix_value(gs_keyboard_value.key_value);
    //LOG_I("scan key value");
}

int rt_hw_keyboard_init(void)
{

    LOG_I("Hello Keyboard driver");
    rt_pin_mode(KEYBOARD_E, PIN_MODE_OUTPUT);
    rt_pin_mode(KEYBOARD_S0, PIN_MODE_OUTPUT);
    rt_pin_mode(KEYBOARD_S1, PIN_MODE_OUTPUT);
    rt_pin_mode(KEYBOARD_S2, PIN_MODE_OUTPUT);
    rt_pin_mode(KEYBOARD_S3, PIN_MODE_OUTPUT);
    rt_pin_mode(KEYBOARD_COM, PIN_MODE_INPUT);

    rt_pin_write(KEYBOARD_E, PIN_LOW);
    LOG_I("Set KEYBOARD_E TO LOW");
    gs_timer4keyboard = rt_timer_create("Tkey", scan_func4keybaord,
                             RT_NULL, RT_TICK_PER_SECOND / 70,
                             RT_TIMER_FLAG_PERIODIC);

    set_key_marix_value(gs_keyboard_value.key_value);
    /* 启动定时器 1 */
    if (gs_timer4keyboard != RT_NULL) rt_timer_start(gs_timer4keyboard);

    return 0;
}
INIT_DEVICE_EXPORT(rt_hw_keyboard_init);
