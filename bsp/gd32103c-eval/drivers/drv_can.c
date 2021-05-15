/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author         Notes
 * 2020-10-09     Yangyongsheng  the first version
 */

#include "drv_can.h"
#include <rtthread.h>
#include "gd32f10x.h"

#ifdef RT_USING_CAN

#include <rtdevice.h>

#ifdef RT_CAN_DEBUG
#define DBG_LVL               DBG_LOG
#else
#define DBG_LVL               DBG_INFO
#endif
#define DBG_TAG               "drv.CAN"
#include <rtdbg.h>

/*
 *  开启对应中断号的中断
 */
void nvic_irq_enable(IRQn_Type irqn, uint32_t priority_pad, uint32_t priority)
{
    NVIC_SetPriority(irqn, priority);
    NVIC_EnableIRQ(irqn);
}
/* attention !!! baud calculation example: Tclk / ((ss + bs1 + bs2) * brp)
 * 54 / ((1 + 9 + 8) * 30) = 100kHz */
static const struct gd32_baud_rate_tab can_baud_rate_tab[] =
{
    {CAN100kBaud, (BAUD_DATA_SET(CAN_SJW_1TQ, SJW) \
        | BAUD_DATA_SET(CAN_BS1_9TQ, BS1) | BAUD_DATA_SET(CAN_BS2_8TQ, BS2) \
        | 30)},
};

#ifdef RT_USING_CAN0
static struct gd32_can drv_can0 =
{
    .name = "can0",
    .CanHandle = CAN1,
};
#endif

#ifdef RT_USING_CAN1
static struct gd32_can drv_can1 =
{
    "can1",
    .CanHandle = CAN2,
};
#endif

static rt_uint32_t get_can_baud_index(rt_uint32_t baud)
{
    rt_uint32_t len, index;

    len = sizeof(can_baud_rate_tab) / sizeof(can_baud_rate_tab[0]);
    for (index = 0; index < len; index++)
    {
        if (can_baud_rate_tab[index].baud_rate == baud)
            return index;
    }

    return 0; /* default baud is CAN100kBaud */
}

static rt_err_t _can_config(struct rt_can_device *can, struct can_configure *cfg)
{
    struct gd32_can *drv_can;
    rt_uint32_t baud_index;

    RT_ASSERT(can);
    RT_ASSERT(cfg);
    drv_can = (struct gd32_can *)can->parent.user_data;
    RT_ASSERT(drv_can);

    drv_can->CanParam.CAN_TTC = DISABLE;
    drv_can->CanParam.CAN_AWK = DISABLE;
    drv_can->CanParam.CAN_ABOR = DISABLE;
    drv_can->CanParam.CAN_ARD = ENABLE;
    drv_can->CanParam.CAN_RFOD = DISABLE;
    drv_can->CanParam.CAN_TFO = DISABLE;

    switch (cfg->mode)
    {
    case RT_CAN_MODE_NORMAL:
        drv_can->CanParam.CAN_Mode = CAN_MODE_NORMAL;
        break;
    case RT_CAN_MODE_LISEN:
        drv_can->CanParam.CAN_Mode = CAN_MODE_SILENT;
        break;
    case RT_CAN_MODE_LOOPBACK:
        drv_can->CanParam.CAN_Mode = CAN_MODE_LOOPBACK;
        break;
    case RT_CAN_MODE_LOOPBACKANLISEN:
        drv_can->CanParam.CAN_Mode = CAN_MODE_SILENT_LOOPBACK;
        break;
    }

    baud_index = get_can_baud_index(cfg->baud_rate);

    drv_can->CanParam.CAN_Prescaler = BAUD_DATA(RRESCL, baud_index);
    drv_can->CanParam.CAN_SJW = BAUD_DATA(SJW, baud_index);
    drv_can->CanParam.CAN_BS1 = BAUD_DATA(BS1, baud_index);
    drv_can->CanParam.CAN_BS2 = BAUD_DATA(BS2, baud_index);

    /* init can */
    if (CAN_Init(drv_can->CanHandle, &drv_can->CanParam) != \
        CAN_INITSTATE_SUCCESS)
    {
        return -RT_ERROR;
    }

    /* default filter config */
    //CAN_FilterInit(drv_can->CanHandle, &drv_can->CanFilter);

    return RT_EOK;
}

static rt_err_t _can_control(struct rt_can_device *can, int cmd, void *arg)
{
    rt_uint32_t argval;
    struct gd32_can *drv_can;
    struct rt_can_filter_config *filter_cfg;

    RT_ASSERT(can != RT_NULL);
    drv_can = (struct gd32_can *)can->parent.user_data;
    RT_ASSERT(drv_can != RT_NULL);
    switch (cmd)
    {
#if 0
    case RT_DEVICE_CTRL_CLR_INT:
        argval = (rt_uint32_t) arg;
        if (argval == RT_DEVICE_FLAG_INT_RX)
        {
            if (CAN1 == drv_can->CanHandle)
            {
                nvic_irq_disable(CAN1_RX0_IRQn);
                nvic_irq_disable(CAN1_RX1_IRQn);
            }
#ifdef RT_USING_CAN1
            if (CAN2 == drv_can->CanHandle)
            {
                nvic_irq_disable(CAN2_RX0_IRQn);
                nvic_irq_disable(CAN2_RX1_IRQn);
            }
#endif
            __HAL_CAN_DISABLE_IT(&drv_can->CanHandle, CAN_IT_RX_FIFO0_MSG_PENDING);
            __HAL_CAN_DISABLE_IT(&drv_can->CanHandle, CAN_IT_RX_FIFO0_FULL);
            __HAL_CAN_DISABLE_IT(&drv_can->CanHandle, CAN_IT_RX_FIFO0_OVERRUN);
            __HAL_CAN_DISABLE_IT(&drv_can->CanHandle, CAN_IT_RX_FIFO1_MSG_PENDING);
            __HAL_CAN_DISABLE_IT(&drv_can->CanHandle, CAN_IT_RX_FIFO1_FULL);
            __HAL_CAN_DISABLE_IT(&drv_can->CanHandle, CAN_IT_RX_FIFO1_OVERRUN);
        }
        else if (argval == RT_DEVICE_FLAG_INT_TX)
        {
            if (CAN1 == drv_can->CanHandle)
            {
                nvic_irq_disable(CAN1_TX_IRQn);
            }
#ifdef RT_USING_CAN1
            if (CAN2 == drv_can->CanHandle)
            {
                nvic_irq_disable(CAN2_TX_IRQn);
            }
#endif
            __HAL_CAN_DISABLE_IT(&drv_can->CanHandle, CAN_IT_TX_MAILBOX_EMPTY);
        }
        else if (argval == RT_DEVICE_CAN_INT_ERR)
        {
            if (CAN1 == drv_can->CanHandle)
            {
                nvic_irq_disable(CAN1_SCE_IRQn);
            }
#ifdef RT_USING_CAN1
            if (CAN2 == drv_can->CanHandle)
            {
                nvic_irq_disable(CAN2_SCE_IRQn);
            }
#endif
            __HAL_CAN_DISABLE_IT(&drv_can->CanHandle, CAN_IT_ERROR_WARNING);
            __HAL_CAN_DISABLE_IT(&drv_can->CanHandle, CAN_IT_ERROR_PASSIVE);
            __HAL_CAN_DISABLE_IT(&drv_can->CanHandle, CAN_IT_BUSOFF);
            __HAL_CAN_DISABLE_IT(&drv_can->CanHandle, CAN_IT_LAST_ERROR_CODE);
            __HAL_CAN_DISABLE_IT(&drv_can->CanHandle, CAN_IT_ERROR);
        }
        break;
#endif
    case RT_DEVICE_CTRL_SET_INT:
        argval = (rt_uint32_t) arg;
        if (argval == RT_DEVICE_FLAG_INT_RX)
        {
            __HAL_CAN_ENABLE_IT(drv_can->CanHandle, CAN_IT_RX_FIFO0_MSG_PENDING);
            __HAL_CAN_ENABLE_IT(drv_can->CanHandle, CAN_IT_RX_FIFO0_FULL);
            __HAL_CAN_ENABLE_IT(drv_can->CanHandle, CAN_IT_RX_FIFO0_OVERRUN);
            __HAL_CAN_ENABLE_IT(drv_can->CanHandle, CAN_IT_RX_FIFO1_MSG_PENDING);
            __HAL_CAN_ENABLE_IT(drv_can->CanHandle, CAN_IT_RX_FIFO1_FULL);
            __HAL_CAN_ENABLE_IT(drv_can->CanHandle, CAN_IT_RX_FIFO1_OVERRUN);

            if (CAN1 == drv_can->CanHandle)
            {
#ifdef GD32F10X_HD
                nvic_irq_enable(USB_LP_CAN1_RX0_IRQn, 1, 0);
                nvic_irq_enable(CAN1_RX1_IRQn, 1, 0);
#elif defined(GD32F10X_CL)
                nvic_irq_enable(CAN1_RX0_IRQn, 1, 0);
                nvic_irq_enable(CAN1_RX1_IRQn, 1, 0);
#endif
            }
#ifdef RT_USING_CAN1
            if (CAN2 == drv_can->CanHandle)
            {
                nvic_irq_enable(CAN2_RX0_IRQn, 1, 0);
                nvic_irq_enable(CAN2_RX1_IRQn, 1, 0);
            }
#endif
        }
        else if (argval == RT_DEVICE_FLAG_INT_TX)
        {
            __HAL_CAN_ENABLE_IT(drv_can->CanHandle, CAN_IT_TX_MAILBOX_EMPTY);

            if (CAN1 == drv_can->CanHandle)
            {
#ifdef GD32F10X_HD
                nvic_irq_enable(USB_HP_CAN1_TX_IRQn, 1, 0);
#elif defined(GD32F10X_CL)
                nvic_irq_enable(CAN1_TX_IRQn, 1, 0);
#endif
            }
#ifdef RT_USING_CAN1
            if (CAN2 == drv_can->CanHandle)
            {
                nvic_irq_enable(CAN2_TX_IRQn, 1, 0);
            }
#endif
        }
        else if (argval == RT_DEVICE_CAN_INT_ERR)
        {
            __HAL_CAN_ENABLE_IT(drv_can->CanHandle, CAN_IT_ERROR_WARNING);
            __HAL_CAN_ENABLE_IT(drv_can->CanHandle, CAN_IT_ERROR_PASSIVE);
            __HAL_CAN_ENABLE_IT(drv_can->CanHandle, CAN_IT_BUSOFF);
            __HAL_CAN_ENABLE_IT(drv_can->CanHandle, CAN_IT_LAST_ERROR_CODE);
            __HAL_CAN_ENABLE_IT(drv_can->CanHandle, CAN_IT_ERROR);

            if (CAN1 == drv_can->CanHandle)
            {
                nvic_irq_enable(CAN1_SCE_IRQn, 1, 0);
            }
#ifdef RT_USING_CAN1
            if (CAN2 == drv_can->CanHandle)
            {
                nvic_irq_enable(CAN2_SCE_IRQn, 1, 0);
            }
#endif
        }
        break;
    case RT_CAN_CMD_SET_FILTER:
        if (RT_NULL == arg)
        {
            /* default filter config */
            CAN_FilterInit(&drv_can->CanFilter);
        }
        else
        {
            filter_cfg = (struct rt_can_filter_config *)arg;
            /* get default filter */
            for (int i = 0; i < filter_cfg->count; i++)
            {
                drv_can->CanFilter.CAN_FilterNumber = filter_cfg->items[i].hdr;
                drv_can->CanFilter.CAN_FilterFIFOAssociation = 0;
                drv_can->CanFilter.CAN_FilterMode = filter_cfg->items[i].mode;
                /* just use 32bit filter */
                drv_can->CanFilter.CAN_FilterScale = CAN_FILTERSCALE_32BIT;

                if (drv_can->CanFilter.CAN_FilterFIFOAssociation == 1)
                {
                    rt_kprintf("111111111 filter\n");
                }
                if (drv_can->CanFilter.CAN_FilterScale == \
                    CAN_FILTERSCALE_16BIT)
                {
                    drv_can->CanFilter.CAN_FilterListHigh = \
                        ((filter_cfg->items[i].id << 5 & 0xFFE0) | \
                        (filter_cfg->items[i].rtr << 4) | \
                        (filter_cfg->items[i].ide << 3) | \
                        (filter_cfg->items[i].id >> 25 & 0x07));
                    drv_can->CanFilter.CAN_FilterListLow = \
                        ((filter_cfg->items[i].id << 5 & 0xFFE0) | \
                        (filter_cfg->items[i].rtr << 4) | \
                        (filter_cfg->items[i].ide << 3) | \
                        (filter_cfg->items[i].id >> 25 & 0x07));
                    if (drv_can->CanFilter.CAN_FilterMode == CAN_FILTERMODE_MASK)
                    {
                        drv_can->CanFilter.CAN_FilterMaskListHigh = (filter_cfg->items[i].mask >> 16) & 0xFFFF;
                        drv_can->CanFilter.CAN_FilterMaskListLow  = filter_cfg->items[i].mask & 0xFFFF;
                    }
                }
                else if (drv_can->CanFilter.CAN_FilterScale == \
                    CAN_FILTERSCALE_32BIT)
                {
                    drv_can->CanFilter.CAN_FilterListHigh = \
                        ((filter_cfg->items[i].id << 5 & 0xFFE0) | \
                        (filter_cfg->items[i].id >> 23 & 0x1f));
                    drv_can->CanFilter.CAN_FilterListLow = \
                        ((filter_cfg->items[i].id >> 10 << 3 & 0xFFF8) | \
                        (filter_cfg->items[i].rtr << 1) | \
                        (filter_cfg->items[i].ide << 2));
                    if (drv_can->CanFilter.CAN_FilterMode == CAN_FILTERMODE_LIST)
                    {
                        drv_can->CanFilter.CAN_FilterMaskListHigh = \
                            drv_can->CanFilter.CAN_FilterListHigh;
                        drv_can->CanFilter.CAN_FilterMaskListLow = \
                            drv_can->CanFilter.CAN_FilterListLow;
                    }
                    else
                    {
                        drv_can->CanFilter.CAN_FilterMaskListHigh = (filter_cfg->items[i].mask >> 16) & 0xFFFF;
                        drv_can->CanFilter.CAN_FilterMaskListLow  = filter_cfg->items[i].mask & 0xFFFF;
                    }
                }
                drv_can->CanFilter.CAN_FilterWork = ENABLE;
                /* Filter conf */
                CAN_FilterInit(&drv_can->CanFilter);
            }
        }
        break;
    case RT_CAN_CMD_SET_MODE:
        argval = (rt_uint32_t) arg;
        if (argval != RT_CAN_MODE_NORMAL &&
                argval != RT_CAN_MODE_LISEN &&
                argval != RT_CAN_MODE_LOOPBACK &&
                argval != RT_CAN_MODE_LOOPBACKANLISEN)
        {
            return -RT_ERROR;
        }
        if (argval != drv_can->device.config.mode)
        {
            drv_can->device.config.mode = argval;
            return _can_config(&drv_can->device, &drv_can->device.config);
        }
        break;
    case RT_CAN_CMD_SET_BAUD:
        argval = (rt_uint32_t) arg;
        if (argval != CAN1MBaud &&
                argval != CAN800kBaud &&
                argval != CAN500kBaud &&
                argval != CAN250kBaud &&
                argval != CAN125kBaud &&
                argval != CAN100kBaud &&
                argval != CAN50kBaud  &&
                argval != CAN20kBaud  &&
                argval != CAN10kBaud)
        {
            return -RT_ERROR;
        }
        if (argval != drv_can->device.config.baud_rate)
        {
            drv_can->device.config.baud_rate = argval;
            return _can_config(&drv_can->device, &drv_can->device.config);
        }
        break;
#if 0
    case RT_CAN_CMD_SET_PRIV:
        argval = (rt_uint32_t) arg;
        if (argval != RT_CAN_MODE_PRIV &&
                argval != RT_CAN_MODE_NOPRIV)
        {
            return -RT_ERROR;
        }
        if (argval != drv_can->device.config.privmode)
        {
            drv_can->device.config.privmode = argval;
            return _can_config(&drv_can->device, &drv_can->device.config);
        }
        break;
    case RT_CAN_CMD_GET_STATUS:
    {
        rt_uint32_t errtype;
        errtype = drv_can->CanHandle->ESR;
        drv_can->device.status.rcverrcnt = errtype >> 24;
        drv_can->device.status.snderrcnt = (errtype >> 16 & 0xFF);
        drv_can->device.status.lasterrtype = errtype & 0x70;
        drv_can->device.status.errcode = errtype & 0x07;

        rt_memcpy(arg, &drv_can->device.status, sizeof(drv_can->device.status));
    }
    break;
#endif
    }

    return RT_EOK;
}

static int _can_sendmsg(struct rt_can_device *can, const void *buf, rt_uint32_t box_num)
{
    CAN_TypeDef *hcan;
    hcan = ((struct gd32_can *) can->parent.user_data)->CanHandle;
    struct rt_can_msg *pmsg = (struct rt_can_msg *) buf;
    CanTxMessage txheader = {0};
    /* Check the parameters */
    RT_ASSERT(pmsg->len <= 0x08);

    /* check select mailbox is empty */
    switch (1 << box_num)
    {
    case 1:
        if (IS_BIT_SET(hcan->TSTR, CAN_TSTR_TME0) != SET)
        {
            /* Return function status */
            return -RT_ERROR;
        }
        break;
    case 2:
        if (IS_BIT_SET(hcan->TSTR, CAN_TSTR_TME1) != SET)
        {
            /* Change CAN state */
            /* Return function status */
            return -RT_ERROR;
        }
        break;
    case 4:
        if (IS_BIT_SET(hcan->TSTR, CAN_TSTR_TME2) != SET)
        {
            /* Change CAN state */
            /* Return function status */
            return -RT_ERROR;
        }
        break;
    default:
        RT_ASSERT(0);
        break;
    }

    if (RT_CAN_STDID == pmsg->ide)
    {
        txheader.FF = CAN_ID_STD;
        RT_ASSERT(IS_CAN_STDID(pmsg->id));
        txheader.StdId = pmsg->id;
    }
    else
    {
        txheader.FF = CAN_ID_EXT;
        RT_ASSERT(IS_CAN_EXTID(pmsg->id));
        txheader.ExtId = pmsg->id;
    }

    if (RT_CAN_DTR == pmsg->rtr)
    {
        txheader.FT = CAN_RTR_DATA;
    }
    else
    {
        txheader.FT = CAN_RTR_REMOTE;
    }
    /* Set up the Id */
    if (RT_CAN_STDID == pmsg->ide)
    {
        hcan->TxMailBox[box_num].TMIR &= \
            ~(CAN_TMIR_StdId_Mask << CAN_TMIR_STID_POS);
        hcan->TxMailBox[box_num].TMIR &= \
            ~(1 << CAN_TMIR_FT_POS);
        hcan->TxMailBox[box_num].TMIR &= \
            ~(1 << CAN_TMIR_FF_POS);
        hcan->TxMailBox[box_num].TMIR |= (txheader.StdId << CAN_TMIR_STID_POS);
        hcan->TxMailBox[box_num].TMIR |= txheader.FT;
        hcan->TxMailBox[box_num].TMIR |= txheader.FF;
    }
    else
    {
        hcan->TxMailBox[box_num].TMIR &= \
            ~(CAN_TMIR_ExtId_Mask << CAN_TMIR_EXTID_POS);
        hcan->TxMailBox[box_num].TMIR &= \
            ~(1 << CAN_TMIR_FT_POS);
        hcan->TxMailBox[box_num].TMIR &= \
            ~(1 << CAN_TMIR_FF_POS);
        hcan->TxMailBox[box_num].TMIR |= (txheader.ExtId << CAN_TMIR_EXTID_POS) | txheader.FF | txheader.FT;
    }
    /* Set up the DLC */
    hcan->TxMailBox[box_num].TMPR = pmsg->len & 0x0FU;
    /* Set up the data field */
    WRITE_REG(hcan->TxMailBox[box_num].TMD1R,
              ((uint32_t)pmsg->data[7] << 24) |
              ((uint32_t)pmsg->data[6] << 16) |
              ((uint32_t)pmsg->data[5] << 8) |
              ((uint32_t)pmsg->data[4]));
    WRITE_REG(hcan->TxMailBox[box_num].TMD0R,
              ((uint32_t)pmsg->data[3] << 24) |
              ((uint32_t)pmsg->data[2] << 16) |
              ((uint32_t)pmsg->data[1] << 8) |
              ((uint32_t)pmsg->data[0]));
    /* Request transmission */
    SET_BIT(hcan->TxMailBox[box_num].TMIR, CAN_TMIR0_TE);

    return RT_EOK;
}

static int _can_recvmsg(struct rt_can_device *can, void *buf, rt_uint32_t fifo)
{
    uint8_t len;
    CAN_TypeDef *hcan;
    struct rt_can_msg *pmsg;

#if 1
    CanRxMessage rxmsg = {0};
    RT_ASSERT(can);

    hcan = ((struct gd32_can *)can->parent.user_data)->CanHandle;
    pmsg = (struct rt_can_msg *) buf;

    /* get data */
    len = CAN_Receive(hcan, fifo, &rxmsg);
    if (0 == len)
        return -RT_ERROR;
    /* get id */
    if (CAN_ID_STD == rxmsg.FF)
    {
        pmsg->ide = RT_CAN_STDID;
        pmsg->id = rxmsg.StdId;
    }
    else
    {
        pmsg->ide = RT_CAN_EXTID;
        pmsg->id = rxmsg.ExtId;
    }
    /* get type */
    if (CAN_RTR_DATA == rxmsg.FT)
    {
        pmsg->rtr = RT_CAN_DTR;
    }
    else
    {
        pmsg->rtr = RT_CAN_RTR;
    }
    /* get len */
    pmsg->len = rxmsg.DLC;
    rt_memcpy(pmsg->data, rxmsg.Data, rxmsg.DLC);
    /* get hdr */
    if (hcan == CAN1)
    {
        pmsg->hdr = (rxmsg.FI + 1) >> 1;
    }
#ifdef RT_USING_CAN1
    else if (hcan == CAN2)
    {
       pmsg->hdr = (rxmsg.FI>> 1) + 14;
    }
#endif
#endif

    return RT_EOK;
}


static const struct rt_can_ops _can_ops =
{
    _can_config,
    _can_control,
    _can_sendmsg,
    _can_recvmsg,
};

static void _can_rx_isr(struct rt_can_device *can, rt_uint32_t fifo)
{
    CAN_TypeDef *hcan;
    RT_ASSERT(can);
    hcan = ((struct gd32_can *) can->parent.user_data)->CanHandle;

    switch (fifo)
    {
    case CAN_RX_FIFO0:
        /* Check Overrun flag for FIFO0 */
        if (hcan->RFR0 & CAN_RX_RFO)
        {
            /* Clear FIFO0 Overrun Flag */
            SET_BIT(hcan->RFR0, CAN_RX_RFO);
            rt_hw_can_isr(can, RT_CAN_EVENT_RXOF_IND | fifo << 8);
        }

        /* save to user list */
        if (CAN_MessageLength(hcan, CAN_RX_FIFO0))
        {
            rt_hw_can_isr(can, RT_CAN_EVENT_RX_IND | fifo << 8);
        }

        /* Check FULL flag for FIFO0 */
        if (hcan->RFR0 & CAN_RX_RFF)
        {
            /* Clear FIFO0 FULL Flag */
            SET_BIT(hcan->RFR0, CAN_RX_RFF);
        }

        break;
    case CAN_RX_FIFO1:
        /* Check Overrun flag for FIFO0 */
        if (hcan->RFR1 & CAN_RX_RFO)
        {
            /* Clear FIFO0 Overrun Flag */
            SET_BIT(hcan->RFR1, CAN_RX_RFO);
            rt_hw_can_isr(can, RT_CAN_EVENT_RXOF_IND | fifo << 8);
        }

        /* save to user list */
        if (CAN_MessageLength(hcan, CAN_RX_FIFO1))
        {
            rt_hw_can_isr(can, RT_CAN_EVENT_RX_IND | fifo << 8);
        }

        /* Check FULL flag for FIFO0 */
        if (hcan->RFR1 & CAN_RX_RFF)
        {
            /* Clear FIFO0 FULL Flag */
            SET_BIT(hcan->RFR1, CAN_RX_RFF);
        }
        break;
    }
}

#ifdef RT_USING_CAN0
/**
 * @brief This function handles CAN1 TX interrupts. transmit fifo0/1/2 is empty can trigger this interrupt
 */
void USBD_HP_CAN0_TX_IRQHandler(void)
{
    rt_interrupt_enter();
    CAN_TypeDef *hcan;
    hcan = drv_can0.CanHandle;

    if (IS_BIT_SET(hcan->TSTR, CAN_IT_MTF0_FINISH))
    {
        if (IS_BIT_SET(hcan->TSTR, CAN_IT_MTFNERR0_FINISH))
        {
            rt_hw_can_isr(&drv_can0.device, RT_CAN_EVENT_TX_DONE | 0 << 8);
        }
        else
        {
            rt_hw_can_isr(&drv_can0.device, RT_CAN_EVENT_TX_FAIL | 0 << 8);
        }
        /* Write 0 to Clear transmission status flag RQCPx */
        SET_BIT(hcan->TSTR, CAN_IT_MTF0_FINISH);
    }
    else if (IS_BIT_SET(hcan->TSTR, CAN_IT_MTF1_FINISH))
    {
        if (IS_BIT_SET(hcan->TSTR, CAN_IT_MTFNERR1_FINISH))
        {
            rt_hw_can_isr(&drv_can0.device, RT_CAN_EVENT_TX_DONE | 1 << 8);
        }
        else
        {
            rt_hw_can_isr(&drv_can0.device, RT_CAN_EVENT_TX_FAIL | 1 << 8);
        }
        /* Write 0 to Clear transmission status flag RQCPx */
        SET_BIT(hcan->TSTR, CAN_IT_MTF1_FINISH);
    }
    else if (IS_BIT_SET(hcan->TSTR, CAN_IT_MTF2_FINISH))
    {
        if (IS_BIT_SET(hcan->TSTR, CAN_IT_MTFNERR2_FINISH))
        {
            rt_hw_can_isr(&drv_can0.device, RT_CAN_EVENT_TX_DONE | 2 << 8);
        }
        else
        {
            rt_hw_can_isr(&drv_can0.device, RT_CAN_EVENT_TX_FAIL | 2 << 8);
        }
        /* Write 0 to Clear transmission status flag RQCPx */
        SET_BIT(hcan->TSTR, CAN_IT_MTF2_FINISH);
    }
    rt_interrupt_leave();
}

/**
 * @brief This function handles CAN1 RX0 interrupts.
 */
void USBD_LP_CAN0_RX0_IRQHandler(void)
{
    rt_interrupt_enter();
    _can_rx_isr(&drv_can0.device, CAN_RX_FIFO0);
    rt_interrupt_leave();
}

/**
 * @brief This function handles CAN1 RX1 interrupts.
 */
void CAN0_RX1_IRQHandler(void)
{
    rt_interrupt_enter();
    _can_rx_isr(&drv_can0.device, CAN_RX_FIFO1);
    rt_interrupt_leave();
}

/**
 * @brief This function handles CAN0 EWMC interrupts.
 */
void CAN0_EWMC_IRQHandler(void)
{
    rt_uint32_t errtype;
    CAN_TypeDef *hcan;

    hcan = drv_can0.CanHandle;
    errtype = hcan->ER;

	rt_kprintf("err--------, errtype=%x\n", errtype);
    rt_interrupt_enter();
#if 0
    HAL_CAN_IRQHandler(hcan);

    switch ((errtype & 0x70) >> 4)
    {
    case RT_CAN_BUS_BIT_PAD_ERR:
        drv_can0.device.status.bitpaderrcnt++;
        break;
    case RT_CAN_BUS_FORMAT_ERR:
        drv_can0.device.status.formaterrcnt++;
        break;
    case RT_CAN_BUS_ACK_ERR:/* attention !!! test ack err's unit is transmit unit */
        drv_can0.device.status.ackerrcnt++;
        if (!READ_BIT(drv_can0.CanHandle->TSTR, CAN_FLAG_TXOK0))
            rt_hw_can_isr(&drv_can0.device, RT_CAN_EVENT_TX_FAIL | 0 << 8);
        else if (!READ_BIT(drv_can0.CanHandle->TSTR, CAN_FLAG_TXOK1))
            rt_hw_can_isr(&drv_can0.device, RT_CAN_EVENT_TX_FAIL | 1 << 8);
        else if (!READ_BIT(drv_can0.CanHandle->TSTR, CAN_FLAG_TXOK2))
            rt_hw_can_isr(&drv_can0.device, RT_CAN_EVENT_TX_FAIL | 2 << 8);
        break;
    case RT_CAN_BUS_IMPLICIT_BIT_ERR:
    case RT_CAN_BUS_EXPLICIT_BIT_ERR:
        drv_can0.device.status.biterrcnt++;
        break;
    case RT_CAN_BUS_CRC_ERR:
        drv_can0.device.status.crcerrcnt++;
        break;
    }

    drv_can0.device.status.lasterrtype = errtype & 0x70;
    drv_can0.device.status.rcverrcnt = errtype >> 24;
    drv_can0.device.status.snderrcnt = (errtype >> 16 & 0xFF);
    drv_can0.device.status.errcode = errtype & 0x07;
#endif
    hcan->STR |= CAN_MSR_ERRI;
    rt_interrupt_leave();
}
#endif /* RT_USING_CAN0 */

#ifdef RT_USING_CAN1
/**
 * @brief This function handles CAN2 TX interrupts.
 */
void CAN1_TX_IRQHandler(void)
{
    rt_interrupt_enter();
    CAN_TypeDef *hcan;
    hcan = drv_can1.CanHandle;
    if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_RQCP0))
    {
        if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_TXOK0))
        {
            rt_hw_can_isr(&drv_can1.device, RT_CAN_EVENT_TX_DONE | 0 << 8);
        }
        else
        {
            rt_hw_can_isr(&drv_can1.device, RT_CAN_EVENT_TX_FAIL | 0 << 8);
        }
        /* Write 0 to Clear transmission status flag RQCPx */
        SET_BIT(hcan->TSTR, CAN_TSTR_RQCP0);
    }
    else if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_RQCP1))
    {
        if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_TXOK1))
        {
            rt_hw_can_isr(&drv_can1.device, RT_CAN_EVENT_TX_DONE | 1 << 8);
        }
        else
        {
            rt_hw_can_isr(&drv_can1.device, RT_CAN_EVENT_TX_FAIL | 1 << 8);
        }
        /* Write 0 to Clear transmission status flag RQCPx */
        SET_BIT(hcan->TSTR, CAN_TSTR_RQCP1);
    }
    else if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_RQCP2))
    {
        if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_TXOK2))
        {
            rt_hw_can_isr(&drv_can1.device, RT_CAN_EVENT_TX_DONE | 2 << 8);
        }
        else
        {
            rt_hw_can_isr(&drv_can1.device, RT_CAN_EVENT_TX_FAIL | 2 << 8);
        }
        /* Write 0 to Clear transmission status flag RQCPx */
        SET_BIT(hcan->TSTR, CAN_TSTR_RQCP2);
    }
    rt_interrupt_leave();
}

/**
 * @brief This function handles CAN2 RX0 interrupts.
 */
void CAN1_RX0_IRQHandler(void)
{
    rt_interrupt_enter();
    /* 使用的 RX FIFO0 */
    _can_rx_isr(&drv_can1.device, CAN_RX_FIFO0);
    rt_interrupt_leave();
}

/**
 * @brief This function handles CAN2 RX1 interrupts.
 */
void CAN1_RX1_IRQHandler(void)
{
    rt_interrupt_enter();
    _can_rx_isr(&drv_can1.device, CAN_RX_FIFO1);
    rt_interrupt_leave();
}

/**
 * @brief This function handles CAN1 SCE interrupts.
 */
void CAN1_EWMC_IRQHandler(void)
{
    rt_uint32_t errtype;
    CAN_TypeDef *hcan;

    hcan = &drv_can1.CanHandle;
    errtype = hcan->ESR;

    rt_interrupt_enter();
    HAL_CAN_IRQHandler(hcan);

    switch ((errtype & 0x70) >> 4)
    {
    case RT_CAN_BUS_BIT_PAD_ERR:
        drv_can1.device.status.bitpaderrcnt++;
        break;
    case RT_CAN_BUS_FORMAT_ERR:
        drv_can1.device.status.formaterrcnt++;
        break;
    case RT_CAN_BUS_ACK_ERR:
        drv_can1.device.status.ackerrcnt++;
        if (!READ_BIT(drv_can1.CanHandle->TSTR, CAN_FLAG_TXOK0))
            rt_hw_can_isr(&drv_can1.device, RT_CAN_EVENT_TX_FAIL | 0 << 8);
        else if (!READ_BIT(drv_can1.CanHandle->TSTR, CAN_FLAG_TXOK1))
            rt_hw_can_isr(&drv_can1.device, RT_CAN_EVENT_TX_FAIL | 1 << 8);
        else if (!READ_BIT(drv_can1.CanHandle->TSTR, CAN_FLAG_TXOK2))
            rt_hw_can_isr(&drv_can1.device, RT_CAN_EVENT_TX_FAIL | 2 << 8);
        break;
    case RT_CAN_BUS_IMPLICIT_BIT_ERR:
    case RT_CAN_BUS_EXPLICIT_BIT_ERR:
        drv_can1.device.status.biterrcnt++;
        break;
    case RT_CAN_BUS_CRC_ERR:
        drv_can1.device.status.crcerrcnt++;
        break;
    }

    drv_can1.device.status.lasterrtype = errtype & 0x70;
    drv_can1.device.status.rcverrcnt = errtype >> 24;
    drv_can1.device.status.snderrcnt = (errtype >> 16 & 0xFF);
    drv_can1.device.status.errcode = errtype & 0x07;
    hcan->MSR |= CAN_MSR_ERRI;
    rt_interrupt_leave();
}
#endif /* RT_USING_CAN1 */

int rt_hw_can_init(void)
{
    struct can_configure config = CANDEFAULTCONFIG;
    GPIO_InitPara GPIO_InitStruct = {0};
    CAN_FilterInitPara can_filter_default = {
        .CAN_FilterListHigh = 0x00,
        .CAN_FilterListLow = 0x00,
        .CAN_FilterMaskListHigh = 0x00,
        .CAN_FilterMaskListLow = 0x00,
        .CAN_FilterFIFOAssociation = CAN_FILTER_FIFO0,
        .CAN_FilterNumber = 0,
        .CAN_FilterMode = CAN_FILTERMODE_MASK,
        .CAN_FilterScale = CAN_FILTERSCALE_32BIT,
        .CAN_FilterWork = ENABLE,
    };

    config.baud_rate = CAN100kBaud;
    config.privmode = RT_CAN_MODE_NOPRIV;
    config.ticks = 50;
#ifdef RT_CAN_USING_HDR
    config.maxhdr = 14;
#ifdef CAN1
    config.maxhdr = 28;
#endif
#endif
    /* pin config */
    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);

    /* CAN0_TX(PA12), CAM0_RX(PA11) GPIO pin configuration */
    GPIO_InitStruct.GPIO_Pin  = GPIO_PIN_11;
    GPIO_InitStruct.GPIO_Speed = GPIO_SPEED_50MHZ;
    GPIO_InitStruct.GPIO_Mode = GPIO_MODE_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin  = GPIO_PIN_12;
    GPIO_InitStruct.GPIO_Speed = GPIO_SPEED_50MHZ;
    GPIO_InitStruct.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
#if 0
    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);
    //gpio_init(GPIOD, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
#endif

    /* TODO config default filter */

#ifdef RT_USING_CAN0
    drv_can0.CanFilter = can_filter_default;
    drv_can0.device.config = config;
    /* register CAN0 device */
    rt_hw_can_register(&drv_can0.device,
                       drv_can0.name,
                       &_can_ops,
                       &drv_can0);
#endif /* RT_USING_CAN0 */

#ifdef RT_USING_CAN1
    drv_can1.device.config = config;
    drv_can0.CanFilter = can_filter_default;
    /* register CAN1 device */
    rt_hw_can_register(&drv_can1.device,
                       drv_can1.name,
                       &_can_ops,
                       &drv_can1);
#endif /* RT_USING_CAN1 */

    return 0;
}

INIT_BOARD_EXPORT(rt_hw_can_init);


#endif /* BSP_USING_CAN */

/************************** end of file ******************/
