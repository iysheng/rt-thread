/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2021-12-26     iysheng           tcd abstract layer
 */

int set_tcd1304_device_marktimes(int data){};

int set_tcd1304_device_data(int data){};

int get_ccd_check_ans(void){};

int get_tcd1304_calibrate_info(unsigned char *value, unsigned char len){};

int set_tcd1304_calibrate_info(unsigned char *value, unsigned char len){};

int get_tcd1304_check_info(unsigned char *value, unsigned char len){};

int set_tcd1304_duanluo_info(unsigned char *value, unsigned char len){};

int get_tcd1304_duanluo_info(unsigned char *value, unsigned char len){};

int get_tcd1304_calibrate_delta_info(unsigned char *value, unsigned char len){};

int set_tcd1304_calibrate_delta_info(unsigned char *value, unsigned char len){};
