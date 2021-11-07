/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2021-08-10     charlown          first version
 * 2021-11-05     iysheng           port to ch32v10x
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include "riscv-ops.h"
#include "drv_uart.h"
#include "ch32v10x_rcc.h"
#include "ch32v10x_usart.h"
#include "ch32v10x_misc.h"

#ifdef BSP_USING_UART
#ifndef ITEM_NUM
#define ITEM_NUM(items) sizeof(items) / sizeof(items[0])
#endif

struct usart_device
{
    struct rt_serial_device parent;
    char *name;
    USART_TypeDef *periph;
    IRQn_Type irqn;
};

#ifdef BSP_USING_UART1
static struct usart_device usart_device1 =
    {
        .name = "uart1",
        .periph = USART1,
        .irqn = USART1_IRQn,
};
#endif

#ifdef BSP_USING_UART2
static struct usart_device usart_device2 =
    {
        .name = "uart2",
        .periph = USART2,
        .irqn = USART2_IRQn,
};
#endif
#ifdef BSP_USING_UART3
static struct usart_device usart_device3 =
    {
        .name = "uart3",
        .periph = USART3,
        .irqn = USART3_IRQn,
};
#endif

static rt_err_t ch32v1_usart_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct usart_device *usart_dev;
    USART_InitTypeDef USART_InitStructure;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    usart_dev = (struct usart_device *)serial;
    RT_ASSERT(usart_dev != RT_NULL);

    USART_StructInit(&USART_InitStructure);

    USART_InitStructure.USART_BaudRate = cfg->baud_rate;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    switch (cfg->data_bits)
    {
    case DATA_BITS_8:
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        break;
    case DATA_BITS_9:
        USART_InitStructure.USART_WordLength = USART_WordLength_9b;
        break;
    default:
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        break;
    }

    switch (cfg->stop_bits)
    {
    case STOP_BITS_1:
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        break;
    case STOP_BITS_2:
        USART_InitStructure.USART_StopBits = USART_StopBits_2;
        break;
    default:
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        break;
    }

    switch (cfg->parity)
    {
    case PARITY_NONE:
        USART_InitStructure.USART_Parity = USART_Parity_No;
        break;
    case PARITY_ODD:
        USART_InitStructure.USART_Parity = USART_Parity_Odd;
        break;
    case PARITY_EVEN:
        USART_InitStructure.USART_Parity = USART_Parity_Even;
        break;
    default:
        USART_InitStructure.USART_Parity = USART_Parity_No;
        break;
    }
    USART_Init(usart_dev->periph, &USART_InitStructure);
    USART_Cmd(usart_dev->periph, ENABLE);

    return RT_EOK;
}

static rt_err_t ch32v1_usart_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct usart_device *usart_dev;

    NVIC_InitTypeDef NVIC_InitStruct;

//    return 0;
    RT_ASSERT(serial != RT_NULL);

    usart_dev = (struct usart_device *)serial;

    RT_ASSERT(usart_dev != RT_NULL);

    NVIC_InitStruct.NVIC_IRQChannel = usart_dev->irqn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        NVIC_InitStruct.NVIC_IRQChannelCmd = DISABLE;
        NVIC_Init(&NVIC_InitStruct);
        USART_ITConfig(usart_dev->periph, USART_IT_RXNE, DISABLE);
        break;

    case RT_DEVICE_CTRL_SET_INT:
        NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStruct);
        USART_ITConfig(usart_dev->periph, USART_IT_RXNE, ENABLE);
        USART_ITConfig(usart_dev->periph, USART_IT_TC, ENABLE);
        break;

    case RT_DEVICE_CHECK_OPTMODE:
        return RT_SERIAL_TX_BLOCKING_BUFFER;
    }

    return RT_EOK;
}

static int ch32v1_usart_putc(struct rt_serial_device *serial, char ch)
{
    struct usart_device *usart_dev;

    RT_ASSERT(serial != RT_NULL);

    usart_dev = (struct usart_device *)serial;

    RT_ASSERT(usart_dev != RT_NULL);

    USART_SendData(usart_dev->periph, (uint8_t)ch);
    while (USART_GetFlagStatus(usart_dev->periph, USART_FLAG_TXE) == RESET)
        ;

    return 1;
}

static int ch32v1_usart_getc(struct rt_serial_device *serial)
{
    struct usart_device *usart_dev;
    int ch;

    RT_ASSERT(serial != RT_NULL);

    usart_dev = (struct usart_device *)serial;

    RT_ASSERT(usart_dev != RT_NULL);

    ch = -1;
    if (RESET != USART_GetFlagStatus(usart_dev->periph, USART_FLAG_RXNE))
    {
        ch = USART_ReceiveData(usart_dev->periph) & 0xff;
    }

    return ch;
}


rt_size_t ch32v1_usart_transmit(struct rt_serial_device       *serial,
                             rt_uint8_t             *buf,
                             rt_size_t               size,
                             rt_uint32_t             tx_flag)
{
    rt_size_t index = 0;
    struct usart_device *usart_dev;

    RT_ASSERT(serial != RT_NULL);

    usart_dev = (struct usart_device *)serial;

    RT_ASSERT(usart_dev != RT_NULL);

    for (; index < size; index ++)
    {
        USART_SendData(usart_dev->periph, (uint8_t)buf[index]);
        while (USART_GetFlagStatus(usart_dev->periph, USART_FLAG_TXE) == RESET);
    }

    return size;
}

static const struct rt_uart_ops usart_ops = {
    .configure = ch32v1_usart_configure,
    .control = ch32v1_usart_control,
    .putc = ch32v1_usart_putc,
    .getc = ch32v1_usart_getc,
    .transmit = ch32v1_usart_transmit,
};

static void usart_isr(struct usart_device *usart_dev)
{

    RT_ASSERT(usart_dev != RT_NULL);

    if ((USART_GetITStatus(usart_dev->periph, USART_IT_RXNE) != RESET) && (RESET != USART_GetFlagStatus(usart_dev->periph, USART_FLAG_RXNE)))
    {
        rt_hw_serial_isr(&usart_dev->parent, RT_SERIAL_EVENT_RX_IND);
        USART_ClearITPendingBit(usart_dev->periph, USART_IT_RXNE);
        USART_ClearFlag(usart_dev->periph, USART_FLAG_RXNE);
    }
    else
    {
        if (USART_GetFlagStatus(usart_dev->periph, USART_FLAG_CTS) != RESET)
        {
            USART_ClearFlag(usart_dev->periph, USART_FLAG_CTS);
        }

        if (USART_GetFlagStatus(usart_dev->periph, USART_FLAG_LBD) != RESET)
        {
            USART_ClearFlag(usart_dev->periph, USART_FLAG_LBD);
        }

        if (USART_GetFlagStatus(usart_dev->periph, USART_FLAG_TC) != RESET)
        {
            USART_ClearFlag(usart_dev->periph, USART_FLAG_TC);
        }
    }
}

#ifdef BSP_USING_UART1
void USART1_IRQHandler(void)
{
    //switch_to_interrupt_stack();
    rt_interrupt_enter();

    usart_isr(&usart_device1);

    rt_interrupt_leave();
    //switch_from_interrupt_stack();
}
#endif

#ifdef BSP_USING_UART2
__attribute__((interrupt()))
void USART2_IRQHandler(void)
{
    switch_to_interrupt_stack();
    rt_interrupt_enter();

    usart_isr(&usart_device2);

    rt_interrupt_leave();
    switch_from_interrupt_stack();
}
#endif
#ifdef BSP_USING_UART3
void USART3_IRQHandler(void)
{
    switch_to_interrupt_stack();
    rt_interrupt_enter();

    usart_isr(&usart_device3);

    rt_interrupt_leave();
    switch_from_interrupt_stack();
}
#endif

int rt_hw_uart_init(void)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

#ifdef BSP_USING_UART1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    usart_device1.parent.ops = &usart_ops;
    usart_device1.parent.config = config;
    rt_hw_serial_register(&usart_device1.parent, usart_device1.name,
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX,
                          RT_NULL);
#endif

#ifdef BSP_USING_UART2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    usart_device2.parent.ops = &usart_ops;
    usart_device2.parent.config = config;
    rt_hw_serial_register(&usart_device2.parent, usart_device2.name,
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX,
                          RT_NULL);
#endif

#ifdef BSP_USING_UART3
    usart_device3.parent.ops = &usart_ops;
    usart_device3.parent.config = config;
    rt_hw_serial_register(&usart_device3.parent, usart_device3.name,
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX,
                          RT_NULL);
#endif

    return RT_EOK;
}

#endif /* BSP_USING_UART */
