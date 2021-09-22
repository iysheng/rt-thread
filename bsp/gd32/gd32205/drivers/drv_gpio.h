/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2021-06-20     RiceChen          the first version
 * 2021-09-21     iysheng           modify to suite with gd32f2xx chip
 */

#ifndef __DRV_GPIO_H__
#define __DRV_GPIO_H__

#include "gd32f20x.h"
#include "gd32f20x_exti.h"

#define GD32_PIN(index, port, pin) {index, RCU_GPIO##port,      \
                                    GPIO##port, GPIO_PIN_##pin, \
                                    GPIO_PORT_SOURCE_GPIO##port,     \
                                    GPIO_PIN_SOURCE_##pin}
#define GD32_PIN_DEFAULT            {-1, (rcu_periph_enum)0, 0, 0, 0, 0}

struct pin_index
{
    rt_int16_t index;
    rcu_periph_enum clk;
    rt_uint32_t gpio_periph;
    rt_uint32_t pin;
    rt_uint8_t port_src;
    rt_uint8_t pin_src;
};

struct pin_irq_map
{
    rt_uint16_t pinbit;
    IRQn_Type irqno;
};

#endif
