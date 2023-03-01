/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-12-7      Vandoul      first version.
 */
#include "rtthread.h"
#include "drivers/pin.h"
#include "hal_data.h"
#include "drivers/spi.h"
#ifdef __cplusplus
extern "C"{
#endif
#include "bsp_api.h"
#include "spi_msd.h"
#ifdef __cplusplus
}
#endif

static struct rt_spi_device sd_device;

void hal_entry(void)
{
    rt_kprintf("hal_entry run.\r\n");
    rt_pin_mode(BSP_IO_PORT_04_PIN_04, PIN_MODE_OUTPUT);
    rt_pin_mode(BSP_IO_PORT_04_PIN_05, PIN_MODE_OUTPUT);
    rt_pin_write(BSP_IO_PORT_04_PIN_05, PIN_HIGH);
    while (1)
    {
        rt_pin_write(BSP_IO_PORT_04_PIN_04, !rt_pin_read(BSP_IO_PORT_04_PIN_04));
        rt_thread_mdelay(100);
        rt_kprintf("hello renesas!\r\n");
    }
}

