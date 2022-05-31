/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2021-12-26     iysheng           tcd abstract layer
 */

#include "drv_tcd1209.h"
#include "tcd_abs.h"
#include <easyflash.h>

int set_abs_ccd_device_marktimes(int data)
{
    return tcd1209_calibrate_triger(data);
};

int set_abs_ccd_device_data(int data){
    return tcd1209_check_triger(data);
};

int get_ccd_check_ans(void){};

int get_abs_ccd_calibrate_info(unsigned char *value, unsigned char len)
{
    return tcd1209_calibrate_get_info(value, len);
};

int set_abs_ccd_calibrate_info(unsigned char *value, unsigned char len)
{
    return tcd1209_calibrate_set_info(value, len);
};

int get_abs_ccd_check_info(unsigned char *value, unsigned char len)
{
    return tcd1209_get_check_info(value, len);
};

int set_abs_ccd_delimiters_info(unsigned char *value, unsigned char len)
{
    return tcd1209_set_delimiters_info(value, len);
};

int get_abs_ccd_delimiters_info(unsigned char *value, unsigned char len)
{
    return tcd1209_get_delimiters_info(value, len);
};

int get_abs_ccd_calibrate_delta_info(unsigned char *value, unsigned char len)
{
    return tcd1209_get_calibrate_delta_info(value, len);
}

int set_abs_ccd_calibrate_delta_info(unsigned char *value, unsigned char len)
{
    return tcd1209_set_calibrate_delta_info(value, len);
};

int get_abs_ccd_calibrate_threshold_info(unsigned char *value, unsigned char len)
{
    return tcd1209_get_calibrate_threshold_info(value, len);
}

int set_abs_ccd_calibrate_threshold_info(unsigned char *value, unsigned char len)
{
    return tcd1209_set_calibrate_threshold_info(value, len);
};

int ef_set_abs_ccd_info(ccd_data_map_t *ccd_data)
{
    ef_set_env_blob(TCD_ABS_EF_NAME, ccd_data, sizeof(ccd_data_map_t));
}

int ef_get_abs_ccd_info(ccd_data_map_t *ccd_data)
{
    size_t read_len = 0;

    return ef_get_env_blob(TCD_ABS_EF_NAME, ccd_data, sizeof(ccd_data_map_t), &read_len);
}

void register_abs_tcd_info(ccd_data_map_t *data)
{
    tcd1209_register_abs_tcd_info(data);
}
