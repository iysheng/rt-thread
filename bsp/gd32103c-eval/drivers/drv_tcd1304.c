/******************************************************************************
* File:             drv_tcd1304.c
*
* Author:           iysheng
* Created:          12/26/20
* Description:      TCD1304 CCD driver
*****************************************************************************/

#include <gd32f10x.h>
#include <board.h>
#include <rtthread.h>

#define DBG_TAG    "drv.CCD"
#define DBG_LVL    DBG_INFO
#include <rtdbg.h>

/*
 * static int drv_tcd1304_init
 * TCD1304 CCD chip 初始化入口
 *
 * @ void:
 * return: errno/Linux
 */
static int drv_tcd1304_init(void)
{
    LOG_I("hello red");
}
/* 降低加載的優先級可以正常調試打印 */
INIT_DEVICE_EXPORT(drv_tcd1304_init);
