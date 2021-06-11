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
#include "gd32f4xx.h"

#ifdef RT_USING_CAN

#include <rtdevice.h>
#define RT_CAN_DEBUG
#ifdef RT_CAN_DEBUG
#define DBG_LVL               DBG_LOG
#else
#define DBG_LVL               DBG_INFO
#endif
#define DBG_TAG               "drv.CAN"
#include <rtdbg.h>

/* attention !!! baud calculation example: Tclk / ((ss + bs1 + bs2) * brp)
 * 54 / ((1 + 9 + 8) * 30) = 100kHz */
static const struct gd32_baud_rate_tab can_baud_rate_tab[] =
{
    {CAN100kBaud, (BAUD_DATA_SET(CAN_BT_SJW_1TQ, SJW) \
        | BAUD_DATA_SET(CAN_BT_BS1_11TQ, BS1) | BAUD_DATA_SET(CAN_BT_BS2_8TQ, BS2) \
        | 25)},
    {CAN500kBaud, (BAUD_DATA_SET(CAN_BT_SJW_1TQ, SJW) \
        | BAUD_DATA_SET(CAN_BT_BS1_11TQ, BS1) | BAUD_DATA_SET(CAN_BT_BS2_8TQ, BS2) \
        | 5)},
    {CAN1MBaud, (BAUD_DATA_SET(CAN_BT_SJW_1TQ, SJW) \
        | BAUD_DATA_SET(CAN_BT_BS1_5TQ, BS1) | BAUD_DATA_SET(CAN_BT_BS2_4TQ, BS2) \
        | 5)},
};

#ifdef RT_USING_CAN0
static struct gd32_can drv_can0 =
{
    .name = "can0",
    .CanHandle = CAN0,
};
#endif

#ifdef RT_USING_CAN1
static struct gd32_can drv_can1 =
{
    "can1",
    .CanHandle = CAN1,
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
    int ret = 0;

    RT_ASSERT(can);
    RT_ASSERT(cfg);
    drv_can = (struct gd32_can *)can->parent.user_data;
    RT_ASSERT(drv_can);

    drv_can->CanParam.time_triggered = DISABLE;
    drv_can->CanParam.auto_wake_up = DISABLE;
    drv_can->CanParam.auto_bus_off_recovery = DISABLE;
    drv_can->CanParam.no_auto_retrans = DISABLE;
    drv_can->CanParam.rec_fifo_overwrite = DISABLE;
    drv_can->CanParam.trans_fifo_order = DISABLE;

    switch (cfg->mode)
    {
    case RT_CAN_MODE_NORMAL:
        drv_can->CanParam.working_mode = CAN_NORMAL_MODE;
        break;
    case RT_CAN_MODE_LISEN:
        drv_can->CanParam.working_mode = CAN_SILENT_MODE;
        break;
    case RT_CAN_MODE_LOOPBACK:
        drv_can->CanParam.working_mode = CAN_LOOPBACK_MODE;
        break;
    case RT_CAN_MODE_LOOPBACKANLISEN:
        drv_can->CanParam.working_mode = CAN_SILENT_LOOPBACK_MODE;
        break;
    }

    baud_index = get_can_baud_index(cfg->baud_rate);

    drv_can->CanParam.prescaler = BAUD_DATA(RRESCL, baud_index);
    drv_can->CanParam.resync_jump_width = BAUD_DATA(SJW, baud_index);
    drv_can->CanParam.time_segment_1 = BAUD_DATA(BS1, baud_index);
    drv_can->CanParam.time_segment_2 = BAUD_DATA(BS2, baud_index);

    /* init can */
    ret = can_init(drv_can->CanHandle, &drv_can->CanParam);
    if (ret != \
        SUCCESS)
    {
        LOG_I("failed init can, ret=%d", ret);
        return -RT_ERROR;
    }

    /* default filter config */
    //can_filter_init(drv_can->CanHandle, &drv_can->CanFilter);

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
            if (CAN0 == drv_can->CanHandle)
            {
                nvic_irq_disable(CAN1_RX0_IRQn);
                nvic_irq_disable(CAN1_RX1_IRQn);
            }
#ifdef RT_USING_CAN1
            if (CAN1 == drv_can->CanHandle)
            {
                nvic_irq_disable(CAN2_RX0_IRQn);
                nvic_irq_disable(CAN2_RX1_IRQn);
            }
#endif
            can_interrupt_disable(&drv_can->CanHandle, CAN_IT_RX_FIFO0_MSG_PENDING);
            can_interrupt_disable(&drv_can->CanHandle, CAN_IT_RX_FIFO0_FULL);
            can_interrupt_disable(&drv_can->CanHandle, CAN_IT_RX_FIFO0_OVERRUN);
            can_interrupt_disable(&drv_can->CanHandle, CAN_IT_RX_FIFO1_MSG_PENDING);
            can_interrupt_disable(&drv_can->CanHandle, CAN_IT_RX_FIFO1_FULL);
            can_interrupt_disable(&drv_can->CanHandle, CAN_IT_RX_FIFO1_OVERRUN);
        }
        else if (argval == RT_DEVICE_FLAG_INT_TX)
        {
            if (CAN0 == drv_can->CanHandle)
            {
                nvic_irq_disable(CAN1_TX_IRQn);
            }
#ifdef RT_USING_CAN1
            if (CAN1 == drv_can->CanHandle)
            {
                nvic_irq_disable(CAN2_TX_IRQn);
            }
#endif
            can_interrupt_disable(&drv_can->CanHandle, CAN_IT_TX_MAILBOX_EMPTY);
        }
        else if (argval == RT_DEVICE_CAN_INT_ERR)
        {
            if (CAN0 == drv_can->CanHandle)
            {
                nvic_irq_disable(CAN0_EWMC_IRQn);
            }
#ifdef RT_USING_CAN1
            if (CAN1 == drv_can->CanHandle)
            {
                nvic_irq_disable(CAN1_EWMC_IRQn);
            }
#endif
            can_interrupt_disable(&drv_can->CanHandle, CAN_IT_ERROR_WARNING);
            can_interrupt_disable(&drv_can->CanHandle, CAN_IT_ERROR_PASSIVE);
            can_interrupt_disable(&drv_can->CanHandle, CAN_IT_BUSOFF);
            can_interrupt_disable(&drv_can->CanHandle, CAN_IT_LAST_ERROR_CODE);
            can_interrupt_disable(&drv_can->CanHandle, CAN_IT_ERROR);
        }
        break;
#endif
    case RT_DEVICE_CTRL_SET_INT:
        argval = (rt_uint32_t) arg;
        if (argval == RT_DEVICE_FLAG_INT_RX)
        {
            can_interrupt_enable(drv_can->CanHandle, CAN_IT_RX_FIFO0_MSG_PENDING);
            can_interrupt_enable(drv_can->CanHandle, CAN_IT_RX_FIFO0_FULL);
            can_interrupt_enable(drv_can->CanHandle, CAN_IT_RX_FIFO0_OVERRUN);
            can_interrupt_enable(drv_can->CanHandle, CAN_IT_RX_FIFO1_MSG_PENDING);
            can_interrupt_enable(drv_can->CanHandle, CAN_IT_RX_FIFO1_FULL);
            can_interrupt_enable(drv_can->CanHandle, CAN_IT_RX_FIFO1_OVERRUN);

            if (CAN0 == drv_can->CanHandle)
            {
                nvic_irq_enable(CAN0_RX0_IRQn, 1, 0);
                nvic_irq_enable(CAN0_RX1_IRQn, 1, 0);
            }
#ifdef RT_USING_CAN1
            if (CAN1 == drv_can->CanHandle)
            {
                nvic_irq_enable(CAN1_RX0_IRQn, 1, 0);
                nvic_irq_enable(CAN1_RX1_IRQn, 1, 0);
            }
#endif
        }
        else if (argval == RT_DEVICE_FLAG_INT_TX)
        {
            can_interrupt_enable(drv_can->CanHandle, CAN_IT_TX_MAILBOX_EMPTY);

            if (CAN0 == drv_can->CanHandle)
            {
                nvic_irq_enable(CAN0_TX_IRQn, 1, 0);
            }
#ifdef RT_USING_CAN1
            if (CAN1 == drv_can->CanHandle)
            {
                nvic_irq_enable(CAN1_TX_IRQn, 1, 0);
            }
#endif
        }
        else if (argval == RT_DEVICE_CAN_INT_ERR)
        {
            can_interrupt_enable(drv_can->CanHandle, CAN_IT_ERROR_WARNING);
            can_interrupt_enable(drv_can->CanHandle, CAN_IT_ERROR_PASSIVE);
            can_interrupt_enable(drv_can->CanHandle, CAN_IT_BUSOFF);
            can_interrupt_enable(drv_can->CanHandle, CAN_IT_LAST_ERROR_CODE);
            can_interrupt_enable(drv_can->CanHandle, CAN_IT_ERROR);

            if (CAN0 == drv_can->CanHandle)
            {
                nvic_irq_enable(CAN0_EWMC_IRQn, 1, 0);
            }
#ifdef RT_USING_CAN1
            if (CAN1 == drv_can->CanHandle)
            {
                nvic_irq_enable(CAN1_EWMC_IRQn, 1, 0);
            }
#endif
        }
        break;
    case RT_CAN_CMD_SET_FILTER:
        if (RT_NULL == arg)
        {
            /* default filter config */
            can_filter_init(&drv_can->CanFilter);
        }
        else
        {
            filter_cfg = (struct rt_can_filter_config *)arg;
            /* get default filter */
            for (int i = 0; i < filter_cfg->count; i++)
            {
                drv_can->CanFilter.filter_number = filter_cfg->items[i].hdr;
                drv_can->CanFilter.filter_fifo_number = 0;
                drv_can->CanFilter.filter_mode = filter_cfg->items[i].mode;
                /* just use 32bit filter */
                drv_can->CanFilter.filter_bits = CAN_FILTERBITS_32BIT;

                if (drv_can->CanFilter.filter_fifo_number == 1)
                {
                    rt_kprintf("111111111 filter\n");
                }
                if (drv_can->CanFilter.filter_bits == \
                    CAN_FILTERBITS_16BIT)
                {
                    drv_can->CanFilter.filter_list_high = \
                        ((filter_cfg->items[i].id << 5 & 0xFFE0) | \
                        (filter_cfg->items[i].rtr << 4) | \
                        (filter_cfg->items[i].ide << 3) | \
                        (filter_cfg->items[i].id >> 25 & 0x07));
                    drv_can->CanFilter.filter_list_low = \
                        ((filter_cfg->items[i].id << 5 & 0xFFE0) | \
                        (filter_cfg->items[i].rtr << 4) | \
                        (filter_cfg->items[i].ide << 3) | \
                        (filter_cfg->items[i].id >> 25 & 0x07));
                    if (drv_can->CanFilter.filter_mode == CAN_FILTERMODE_MASK)
                    {
                        drv_can->CanFilter.filter_mask_high = (filter_cfg->items[i].mask >> 16) & 0xFFFF;
                        drv_can->CanFilter.filter_mask_low = filter_cfg->items[i].mask & 0xFFFF;
                    }
                }
                else if (drv_can->CanFilter.filter_bits == \
                    CAN_FILTERBITS_32BIT)
                {
                    drv_can->CanFilter.filter_list_high = \
                        ((filter_cfg->items[i].id << 5 & 0xFFE0) | \
                        (filter_cfg->items[i].id >> 23 & 0x1f));
                    drv_can->CanFilter.filter_list_low = \
                        ((filter_cfg->items[i].id >> 10 << 3 & 0xFFF8) | \
                        (filter_cfg->items[i].rtr << 1) | \
                        (filter_cfg->items[i].ide << 2));
                    if (drv_can->CanFilter.filter_mode == CAN_FILTERMODE_LIST)
                    {
                        drv_can->CanFilter.filter_mask_high = \
                            drv_can->CanFilter.filter_list_high;
                        drv_can->CanFilter.filter_mask_low = \
                            drv_can->CanFilter.filter_list_low;
                    }
                    else
                    {
                        drv_can->CanFilter.filter_mask_high = (filter_cfg->items[i].mask >> 16) & 0xFFFF;
                        drv_can->CanFilter.filter_mask_low  = filter_cfg->items[i].mask & 0xFFFF;
                    }
                }
                drv_can->CanFilter.filter_enable = ENABLE;
                /* Filter conf */
                can_filter_init(&drv_can->CanFilter);
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
    uint32_t hcan;
    hcan = ((struct gd32_can *) can->parent.user_data)->CanHandle;
    struct rt_can_msg *pmsg = (struct rt_can_msg *) buf;
    can_trasnmit_message_struct txheader = {0};
    /* Check the parameters */
    RT_ASSERT(pmsg->len <= 0x08);

    /* check select mailbox is empty */
    switch (1 << box_num)
    {
    case 1:
        if ((CAN_TSTAT(hcan) & CAN_TSTAT_TME0) == RESET)
        {
            /* Return function status */
            return -RT_ERROR;
        }
        break;
    case 2:
        if ((CAN_TSTAT(hcan) & CAN_TSTAT_TME1) == RESET)
        {
            /* Change CAN state */
            /* Return function status */
            return -RT_ERROR;
        }
        break;
    case 4:
        if ((CAN_TSTAT(hcan) & CAN_TSTAT_TME2) == RESET)
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
        txheader.tx_ff = CAN_ID_STD;
        RT_ASSERT(IS_CAN_STDID(pmsg->id));
        txheader.tx_sfid = pmsg->id;
    }
    else
    {
        txheader.tx_ff = CAN_ID_EXT;
        RT_ASSERT(IS_CAN_EXTID(pmsg->id));
        txheader.tx_efid = pmsg->id;
    }

    if (RT_CAN_DTR == pmsg->rtr)
    {
        txheader.tx_ft = CAN_RTR_DATA;
    }
    else
    {
        txheader.tx_ft = CAN_RTR_REMOTE;
    }
    txheader.tx_dlen = pmsg->len;
    LOG_I("len=%d\n", txheader.tx_dlen);
    rt_memcpy(txheader.tx_data, pmsg->data, txheader.tx_dlen);
    can_message_transmit(hcan, &txheader);

    return RT_EOK;
}

static int _can_recvmsg(struct rt_can_device *can, void *buf, rt_uint32_t fifo)
{
    uint8_t len;
    uint32_t hcan;
    struct rt_can_msg *pmsg;

#if 1
    can_receive_message_struct rxmsg = {0};
    RT_ASSERT(can);

    hcan = ((struct gd32_can *)can->parent.user_data)->CanHandle;
    pmsg = (struct rt_can_msg *) buf;

    /* get data */
    can_message_receive(hcan, fifo, &rxmsg);
    if (0 == rxmsg.rx_dlen)
        return -RT_ERROR;
    /* get id */
    if (CAN_ID_STD == rxmsg.rx_ff)
    {
        pmsg->ide = RT_CAN_STDID;
        pmsg->id = rxmsg.rx_sfid;
    }
    else
    {
        pmsg->ide = RT_CAN_EXTID;
        pmsg->id = rxmsg.rx_efid;
    }
    /* get type */
    if (CAN_RTR_DATA == rxmsg.rx_ft)
    {
        pmsg->rtr = RT_CAN_DTR;
    }
    else
    {
        pmsg->rtr = RT_CAN_RTR;
    }
    /* get len */
    pmsg->len = rxmsg.rx_dlen;
    rt_memcpy(pmsg->data, rxmsg.rx_data, rxmsg.rx_dlen);
    /* get hdr */
    if (hcan == CAN0)
    {
        /* TODO set 硬件过滤表号 */
        //pmsg->hdr = (rxmsg.FI + 1) >> 1;
    }
#ifdef RT_USING_CAN1
    else if (hcan == CAN1)
    {
        /* TODO set 硬件过滤表号 */
       //pmsg->hdr = (rxmsg.FI>> 1) + 14;
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
    uint32_t hcan;
    RT_ASSERT(can);
    hcan = ((struct gd32_can *) can->parent.user_data)->CanHandle;

    switch (fifo)
    {
    case CAN_RX_FIFO0:
        /* Check Overrun flag for FIFO0 */
        if (CAN_RFIFO0(hcan) & CAN_RFIFO0_RFO0)
        {
            /* Clear FIFO0 Overrun Flag */
            can_flag_clear(hcan, CAN_FLAG_RFO0);
            rt_hw_can_isr(can, RT_CAN_EVENT_RXOF_IND | fifo << 8);
        }

        /* save to user list */
        if (CAN_RFIFO0(hcan) & CAN_RFIFO0_RFL0)
        {
            rt_hw_can_isr(can, RT_CAN_EVENT_RX_IND | fifo << 8);
        }

        /* Check FULL flag for FIFO0 */
        if (CAN_RFIFO0(hcan) & CAN_RFIFO0_RFF0)
        {
            /* Clear FIFO0 FULL Flag */
            can_flag_clear(hcan, CAN_FLAG_RFF0);
        }
        break;
    case CAN_RX_FIFO1:
        /* Check Overrun flag for FIFO0 */
        if (CAN_RFIFO1(hcan) & CAN_RFIFO1_RFO1)
        {
            /* Clear FIFO0 Overrun Flag */
            can_flag_clear(hcan, CAN_FLAG_RFO1);
            rt_hw_can_isr(can, RT_CAN_EVENT_RXOF_IND | fifo << 8);
        }
        /* save to user list */
        if (CAN_RFIFO1(hcan) & CAN_RFIFO1_RFL1)
        {
            rt_hw_can_isr(can, RT_CAN_EVENT_RX_IND | fifo << 8);
        }
        /* Check FULL flag for FIFO0 */
        if (CAN_RFIFO1(hcan) & CAN_RFIFO1_RFF1)
        {
            /* Clear FIFO0 FULL Flag */
            can_flag_clear(hcan, CAN_FLAG_RFF1);
        }
        break;
    }
}

#ifdef RT_USING_CAN0
/**
 * @brief This function handles CAN0 TX interrupts. transmit fifo0/1/2 is empty can trigger this interrupt
 */
void CAN0_TX_IRQHandler(void)
{
    rt_interrupt_enter();
    uint32_t hcan;
    hcan = drv_can0.CanHandle;

    if (CAN_TSTAT(hcan) & CAN_TSTAT_MTF0)
    {
        if (CAN_TSTAT(hcan) & CAN_TSTAT_MTFNERR0)
        {
            rt_hw_can_isr(&drv_can0.device, RT_CAN_EVENT_TX_DONE | 0 << 8);
        }
        else
        {
            rt_hw_can_isr(&drv_can0.device, RT_CAN_EVENT_TX_FAIL | 0 << 8);
        }
        /* Write 0 to Clear transmission status flag RQCPx */
        can_flag_clear(hcan, CAN_FLAG_MTF0);
    }
    else if (CAN_TSTAT(hcan) & CAN_TSTAT_MTF1)
    {
        if (CAN_TSTAT(hcan) & CAN_TSTAT_MTFNERR1)
        {
            rt_hw_can_isr(&drv_can0.device, RT_CAN_EVENT_TX_DONE | 1 << 8);
        }
        else
        {
            rt_hw_can_isr(&drv_can0.device, RT_CAN_EVENT_TX_FAIL | 1 << 8);
        }
        /* Write 0 to Clear transmission status flag RQCPx */
        can_flag_clear(hcan, CAN_FLAG_MTF1);
    }
    else if (CAN_TSTAT(hcan) & CAN_TSTAT_MTF2)
    {
        if (CAN_TSTAT(hcan) & CAN_TSTAT_MTFNERR2)
        {
            rt_hw_can_isr(&drv_can0.device, RT_CAN_EVENT_TX_DONE | 2 << 8);
        }
        else
        {
            rt_hw_can_isr(&drv_can0.device, RT_CAN_EVENT_TX_FAIL | 2 << 8);
        }
        /* Write 0 to Clear transmission status flag RQCPx */
        can_flag_clear(hcan, CAN_FLAG_MTF2);
    }
    rt_interrupt_leave();
}

/**
 * @brief This function handles CAN0 RX0 interrupts.
 */
void CAN0_RX0_IRQHandler(void)
{
    rt_interrupt_enter();
    _can_rx_isr(&drv_can0.device, CAN_RX_FIFO0);
    rt_interrupt_leave();
}

/**
 * @brief This function handles CAN0 RX1 interrupts.
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
    uint32_t hcan;

    hcan = drv_can0.CanHandle;
    errtype = CAN_ERR(hcan);

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
    CAN_STAT(hcan) |= CAN_STAT_ERRIF;
    rt_interrupt_leave();
}
#endif /* RT_USING_CAN0 */

#ifdef RT_USING_CAN1
/**
 * @brief This function handles CAN1 TX interrupts.
 */
void CAN1_TX_IRQHandler(void)
{
    rt_interrupt_enter();
    uint32_t hcan;
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
 * @brief This function handles CAN1 RX0 interrupts.
 */
void CAN1_RX0_IRQHandler(void)
{
    rt_interrupt_enter();
    /* 使用的 RX FIFO0 */
    _can_rx_isr(&drv_can1.device, CAN_RX_FIFO0);
    rt_interrupt_leave();
}

/**
 * @brief This function handles CAN1 RX1 interrupts.
 */
void CAN1_RX1_IRQHandler(void)
{
    rt_interrupt_enter();
    _can_rx_isr(&drv_can1.device, CAN_RX_FIFO1);
    rt_interrupt_leave();
}

/**
 * @brief This function handles CAN0 SCE interrupts.
 */
void CAN1_EWMC_IRQHandler(void)
{
    rt_uint32_t errtype;
    uint32_t hcan;

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
    can_filter_parameter_struct can_filter_default = {
        .filter_list_high = 0x00,
        .filter_list_low = 0x00,
        .filter_mask_high = 0x00,
        .filter_mask_low = 0x00,
        .filter_fifo_number = CAN_FIFO0,
        .filter_number = 0,
        .filter_mode = CAN_FILTERMODE_MASK,
        .filter_bits = CAN_FILTERBITS_32BIT,
        .filter_enable = ENABLE,
    };

    config.baud_rate = CAN100kBaud;
    config.privmode = RT_CAN_MODE_NOPRIV;
    config.ticks = 50;
#ifdef RT_CAN_USING_HDR
    config.maxhdr = 14;
#ifdef CAN0
    config.maxhdr = 28;
#endif
#endif
    /* pin config */
    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_GPIOB);

    /* CAN0_TX(PB9), CAM0_RX(PB8) GPIO pin configuration */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_af_set(GPIOB, GPIO_AF_9, GPIO_PIN_8);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_af_set(GPIOB, GPIO_AF_9, GPIO_PIN_9);
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
    drv_can1.CanFilter = can_filter_default;
    /* register CAN0 device */
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
