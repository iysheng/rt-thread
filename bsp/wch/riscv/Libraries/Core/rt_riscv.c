/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2021-10-27     iysheng           first version
 */

#include <rtthread.h>
#include "type.h"
#include "ch32v10x.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

int def;
__attribute__((interrupt()))
void SysTick_Handler(void)
{
      def++;
#if 0
      if (def % 100 == 1)
      {

            if (def / 100 % 2)
            {
            *(volatile rt_uint32_t *)0x4001080c |= 0x40;
            }
            else
            {
            *(volatile rt_uint32_t *)0x4001080c &= 0xbf;
            }
      }
#endif
      rt_interrupt_enter();

#if 1
      SysTick->CNTL0 = 0;
      SysTick->CNTL1 = 0;
      SysTick->CNTL2 = 0;
      SysTick->CNTL3 = 0;
      SysTick->CNTH0 = 0;
      SysTick->CNTH1 = 0;
      SysTick->CNTH2 = 0;
      SysTick->CNTH3 = 0;

      rt_tick_increase();
#endif
      NVIC_ClearPendingIRQ(SysTicK_IRQn);

      rt_interrupt_leave();
}
