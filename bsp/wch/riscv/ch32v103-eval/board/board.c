/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2021-10-26     iysheng           first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>
#include "board.h"
#include "ch32v10x_rcc.h"

rt_uint32_t ch32_get_sysclock_frequency(void)
{
    RCC_ClocksTypeDef RCC_Clocks;

    RCC_GetClocksFreq(&RCC_Clocks);

    return RCC_Clocks.SYSCLK_Frequency;
}

#if 0
void ch32f1_usart_clock_and_io_init(USART_TypeDef *usartx)
{

    GPIO_InitTypeDef GPIO_InitStructure;

    if (usartx == USART1)
    {

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

        /* USART1 TX-->A.9   RX-->A.10 */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    }

    if (usartx == USART2)
    {

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

        /* USART2 TX-->A.2   RX-->A.3 */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    }

    if (usartx == USART3)
    {

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

        /* USART3 TX-->C.10   RX-->C.11 */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
        GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
    }
}
#endif
