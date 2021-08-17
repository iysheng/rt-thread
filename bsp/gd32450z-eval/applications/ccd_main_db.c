/******************************************************************************
* File:             ccd_main_db.c
*
* Author:           iysheng@163.com  
* Created:          08/16/21 
* Description:      配置系统相关
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include "ccd_main_base.h"
#include <flashdb.h>

#define DBG_LVL               DBG_LOG
#define DBG_TAG               "app.db"
#include <rtdbg.h>


static struct fdb_kvdb gs_flashdb4sys;
static uint32_t gs_bootcounts;

ccd_main_system_t gs_ccd_main_sysinfo = {
    .ccd_main_config = {
        .left = 1000,
        .middle = 1400,
        .right = 1000,
    },
};

static struct fdb_default_kv_node gs_default_kvnode4sys[] = {
    {.key = "bootcounts", .value = &gs_bootcounts, .value_len = 4},
    {.key = "ccd_main_info", .value = &gs_ccd_main_sysinfo, .value_len = sizeof(ccd_main_system_t)},
};

struct fdb_default_kv gs_defautlkv4sys = {
    .kvs = gs_default_kvnode4sys,
    .num = 2,
};

/**
  * @brief 设置 ccd 的检测配置
  * @param ccd_main_config_t ccd_main_config: 
  * retval .
  */
int set_ccd_main_config(ccd_main_config_t *ccd_main_config)
{
    int ret = 0;
    struct fdb_blob blob;

    rt_memcpy(&gs_ccd_main_sysinfo.ccd_main_config, ccd_main_config, sizeof(ccd_main_config_t));
    ret = fdb_kv_set_blob(&gs_flashdb4sys, "ccd_main_info", fdb_blob_make(&blob, &gs_ccd_main_sysinfo, sizeof(gs_ccd_main_sysinfo)));

    return ret;
}

static void up_boot_progress(fdb_kvdb_t kvdb)
{
    struct fdb_blob blob;
    int bootcounts = 0;

    /* get the "boot_count" KV value */
    fdb_kv_get_blob(kvdb, "bootcounts", fdb_blob_make(&blob, &bootcounts, sizeof(bootcounts)));
    /* the blob.saved.len is more than 0 when get the value successful */
    if (blob.saved.len > 0) {
        LOG_I("get the 'bootcounts' value is %d", bootcounts);
    } else {
        LOG_I("get the 'bootcounts' failed");
    }

    bootcounts ++;
    /* change the "bootcounts" KV's value */
    fdb_kv_set_blob(kvdb, "bootcounts", fdb_blob_make(&blob, &bootcounts, sizeof(bootcounts)));
}

extern fdb_err_t fdb_kvdb_init(fdb_kvdb_t db, const char *name, const char *part_name, struct fdb_default_kv *default_kv,
        void *user_data);

int ccd_main_db_init(void)
{
    int ret;
    struct fdb_blob blob;

    ret = fdb_kvdb_init(&gs_flashdb4sys, "syscfg", "sys", &gs_defautlkv4sys, NULL);
    if (!ret)
    {
        up_boot_progress(&gs_flashdb4sys);
        fdb_kv_get_blob(&gs_flashdb4sys, "ccd_main_info", fdb_blob_make(&blob, &gs_ccd_main_sysinfo, sizeof(gs_ccd_main_sysinfo)));
        LOG_I("ccd_main_config[%u,%u,%u]", gs_ccd_main_sysinfo.ccd_main_config.left, gs_ccd_main_sysinfo.ccd_main_config.middle, gs_ccd_main_sysinfo.ccd_main_config.right);
        set_ccd_main_config(&gs_ccd_main_sysinfo.ccd_main_config);
    }

    return ret;
}
