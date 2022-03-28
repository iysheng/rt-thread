/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2022-03-23     iysheng           tcd abstract layer
 */

#ifndef __TCD_ABS_H__
#define __TCD_ABS_H__

typedef struct {
    int16_t left;
    int16_t middle;
    int16_t right;
} ccd_data_t;

typedef struct {
    int16_t delta;
    ccd_data_t position;
    ccd_data_t value;
} ccd_data_map_t;

#define TCD_ABS_EF_NAME    "tcd_abs"

int set_abs_ccd_device_marktimes(int data);
int set_abs_ccd_device_data(int data);
int get_ccd_check_ans(void);
int set_abs_ccd_delimiters_info(unsigned char *value, unsigned char len);
int get_abs_ccd_delimiters_info(unsigned char *value, unsigned char len);
int get_abs_ccd_check_info(unsigned char *value, unsigned char len);
int set_abs_ccd_calibrate_delta_info(unsigned char *value, unsigned char len);
int get_abs_ccd_calibrate_delta_info(unsigned char *value, unsigned char len);
int get_abs_ccd_calibrate_info(unsigned char *value, unsigned char len);
int set_abs_ccd_calibrate_info(unsigned char *value, unsigned char len);
int ef_get_abs_ccd_info(ccd_data_map_t *ccd_data);
int ef_set_abs_ccd_info(ccd_data_map_t *ccd_data);
void register_abs_tcd_info(ccd_data_map_t *data);

#endif /* ifndef __TCD_ABS_H__ */
