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

#endif /* ifndef __TCD_ABS_H__ */
