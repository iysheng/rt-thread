/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2020-10-09     Yangyongsheng  the first version
 */

#ifndef __DRV_CAN_H__
#define __DRV_CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <board.h>
#include <rtdevice.h>
#include <rtthread.h>

#include "jari_base.h"

#define BS1SHIFT        16
#define BS2SHIFT        20
#define RRESCLSHIFT     0
#define SJWSHIFT        24
#define BS1MASK         ((0x0F) << BS1SHIFT )
#define BS2MASK         ((0x07) << BS2SHIFT )
#define RRESCLMASK      (0x3FF << RRESCLSHIFT )
#define SJWMASK         (0x3 << SJWSHIFT )

#define IS_BIT_SET(REG, BIT)         (((REG) & (BIT)) != 0U)
#define IS_BIT_CLR(REG, BIT)         (((REG) & (BIT)) == 0U)

#define CAN_ID_STD                  0x00000000U  /*!< Standard Id */
#define CAN_ID_EXT                  0x00000004U  /*!< Extended Id */

/** @defgroup CAN_remote_transmission_request CAN Remote Transmission Request
  * @{
  */
#define CAN_RTR_DATA                0x00000000U  /*!< Data frame */
#define CAN_RTR_REMOTE              0x00000002U  /*!< Remote frame */

/** @defgroup CAN_receive_FIFO_number CAN Receive FIFO Number
  * @{
  */
#define CAN_RX_FIFO0                (0x00000000U)  /*!< CAN receive FIFO 0 */
#define CAN_RX_FIFO1                (0x00000001U)  /*!< CAN receive FIFO 1 */

#define IS_CAN_STDID(STDID)   ((STDID) <= 0x00007FFU)
#define IS_CAN_EXTID(EXTID)   ((EXTID) <= 0x1FFFFFFFU)

struct gd32_baud_rate_tab
{
    rt_uint32_t baud_rate;
    rt_uint32_t config_data;
};

#define BAUD_DATA(TYPE,NO)       ((can_baud_rate_tab[NO].config_data & TYPE##MASK) \
	>> TYPE##SHIFT)
#define BAUD_DATA_SET(DATA,TYPE)       ((DATA << TYPE##SHIFT))

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

/* can interrupt config */
#define CAN_IT_TX_MAILBOX_EMPTY         (1 << 0)

#define CAN_IT_RX_FIFO0_MSG_PENDING     (1 << 1)
#define CAN_IT_RX_FIFO0_FULL            (1 << 2)
#define CAN_IT_RX_FIFO0_OVERRUN         (1 << 3)
#define CAN_IT_RX_FIFO1_MSG_PENDING     (1 << 4)
#define CAN_IT_RX_FIFO1_FULL            (1 << 5)
#define CAN_IT_RX_FIFO1_OVERRUN         (1 << 6)

#define CAN_IT_ERROR_WARNING            (1 << 8)
#define CAN_IT_ERROR_PASSIVE            (1 << 9)
#define CAN_IT_BUSOFF                   (1 << 10)
#define CAN_IT_LAST_ERROR_CODE          (1 << 11)
#define CAN_IT_ERROR                    (1 << 15)

#define CAN_IT_MTF0_FINISH              (1 << 0)
#define CAN_IT_MTFNERR0_FINISH          (1 << 1)
#define CAN_IT_MTF1_FINISH              (1 << 8)
#define CAN_IT_MTFNERR1_FINISH          (1 << 9)
#define CAN_IT_MTF2_FINISH              (1 << 16)
#define CAN_IT_MTFNERR2_FINISH          (1 << 17)

#define CAN_RX_RFF                      (1 << 3)
#define CAN_RX_RFO                      (1 << 4)
#define CAN_RX_RFD                      (1 << 5)

/**
  * @brief  Enable the specified CAN interrupts
  * @param  __HANDLE__: CAN handle.
  * @param  __INTERRUPT__: CAN Interrupt.
  *         This parameter can be one of the following values:
  *            @arg CAN_IT_TME: Transmit mailbox empty interrupt enable
  *            @arg CAN_IT_FMP0: FIFO 0 message pending interrupt
  *            @arg CAN_IT_FF0 : FIFO 0 full interrupt
  *            @arg CAN_IT_FOV0: FIFO 0 overrun interrupt
  *            @arg CAN_IT_FMP1: FIFO 1 message pending interrupt
  *            @arg CAN_IT_FF1 : FIFO 1 full interrupt
  *            @arg CAN_IT_FOV1: FIFO 1 overrun interrupt
  *            @arg CAN_IT_WKU : Wake-up interrupt
  *            @arg CAN_IT_SLK : Sleep acknowledge interrupt
  *            @arg CAN_IT_EWG : Error warning interrupt
  *            @arg CAN_IT_EPV : Error passive interrupt
  *            @arg CAN_IT_BOF : Bus-off interrupt
  *            @arg CAN_IT_LEC : Last error code interrupt
  *            @arg CAN_IT_ERR : Error Interrupt
  * @retval None.
  */
#define __HAL_CAN_ENABLE_IT(__HANDLE__, __INTERRUPT__) \
	(((__HANDLE__)->IER) |= (__INTERRUPT__))

/**
  * @brief  Disable the specified CAN interrupts
  * @param  __HANDLE__: CAN handle.
  * @param  __INTERRUPT__: CAN Interrupt.
  *         This parameter can be one of the following values:
  *            @arg CAN_IT_TME: Transmit mailbox empty interrupt enable
  *            @arg CAN_IT_FMP0: FIFO 0 message pending interrupt
  *            @arg CAN_IT_FF0 : FIFO 0 full interrupt
  *            @arg CAN_IT_FOV0: FIFO 0 overrun interrupt
  *            @arg CAN_IT_FMP1: FIFO 1 message pending interrupt
  *            @arg CAN_IT_FF1 : FIFO 1 full interrupt
  *            @arg CAN_IT_FOV1: FIFO 1 overrun interrupt
  *            @arg CAN_IT_WKU : Wake-up interrupt
  *            @arg CAN_IT_SLK : Sleep acknowledge interrupt
  *            @arg CAN_IT_EWG : Error warning interrupt
  *            @arg CAN_IT_EPV : Error passive interrupt
  *            @arg CAN_IT_BOF : Bus-off interrupt
  *            @arg CAN_IT_LEC : Last error code interrupt
  *            @arg CAN_IT_ERR : Error Interrupt
  * @retval None.
  */
#define __HAL_CAN_DISABLE_IT(__HANDLE__, __INTERRUPT__) \
	(((__HANDLE__)->Instance->IER) &= ~(__INTERRUPT__))

/* gd32 can device */
struct gd32_can
{
    char *name;
    CAN_InitPara CanParam;
    CAN_FilterInitPara CanFilter;
    CAN_TypeDef *CanHandle;
    struct rt_can_device device;     /* inherit from can device */
};

int rt_hw_can_init(void);

#ifdef __cplusplus
}
#endif

#endif /*__DRV_CAN_H__ */

/************************** end of file ******************/
