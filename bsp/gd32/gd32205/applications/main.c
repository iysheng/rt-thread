/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-09-21     iysheng      the first version
 */

#include <stdio.h>
#include <rtthread.h>
#include "rtdevice.h"
#include <easyflash.h>
#include "tcd_abs.h"

#define DBG_LVL               DBG_INFO
#define DBG_TAG               "app.MAIN"
#include <rtdbg.h>

/* HEART PIN is GPIOB_15 */
#define HEART_PIN    31
/* PWM PIN is GPIOB_5 */
#define PWM_PIN    21

static rt_thread_t gs_can_thread;
extern void can_backend_entry(void * arg);

#if 1
/**
 * Env demo.
 */
void test_env(void) {
    uint32_t i_boot_times = 0;
    char *c_old_boot_times, c_new_boot_times[11] = {0};

    /* get the boot count number from Env */
    c_old_boot_times = ef_get_env("boot_times");
    if(c_old_boot_times)
        i_boot_times = atoi(c_old_boot_times);
    /* boot count +1 */
    i_boot_times ++;
    rt_kprintf("The system now boot %d times\n", i_boot_times);
    /* interger to string */
    rt_sprintf(c_new_boot_times, "%d", i_boot_times);
    /* set and store the boot count number to Env */
    ef_set_env("boot_times", c_new_boot_times);
    ef_save_env();
}

/* flash erase test command */
long erase_flash(int argc, char *argv[])
{
    int addr, len;
    if (argc > 2)
    {
        addr = atoi(argv[1]);
        len = atoi(argv[2]);
        ef_port_erase(addr, len);
    }
    else
    {
        ef_port_erase(EF_START_ADDR, ENV_AREA_SIZE);
    }

    return 0;
}
MSH_CMD_EXPORT(erase_flash, erase flashs areas);
#endif
int main(void)
{
    ccd_data_map_t ccd_data = {0};

    if (easyflash_init() != EF_NO_ERR)
    {
        LOG_E("Failed init easyflash.");
    }
    else if (ef_get_abs_ccd_info(&ccd_data))
    {
        LOG_I("tcd_abs:%u,%u,%u,%u,%u,%u.", ccd_data.position.left,\
            ccd_data.position.middle,\
            ccd_data.position.right,\
            ccd_data.value.left,\
            ccd_data.value.middle,\
            ccd_data.value.right);
    }

    gs_can_thread = rt_thread_create("canBack", can_backend_entry, RT_NULL, 0x800, 5, 10);
    if (!gs_can_thread)
    {
        LOG_E("Failed create can backend thread.");
        return -1;
    }
    else if (RT_EOK != rt_thread_startup(gs_can_thread))
    {
        LOG_E("Failed startup can backend thread.");
        return -2;
    }

    rt_pin_mode(PWM_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(PWM_PIN, PIN_LOW);
    while(1)
    {
        rt_pin_write(HEART_PIN, PIN_LOW);
        rt_thread_mdelay(60000);
        rt_pin_write(HEART_PIN, PIN_HIGH);
        rt_thread_mdelay(1000);
    }

    return 0;
}
