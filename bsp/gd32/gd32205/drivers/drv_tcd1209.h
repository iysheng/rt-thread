/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2021-10-01     iysheng           TCD1209 driver head file
 */

#ifndef __TCD_1209_H__
#define __TCD_1209_H__

#include <stdint.h>

#define GD32_F1_PIN                    GET_PIN(B, 13)   /* TIMER0_CH0_ON R*/
#define GD32_F2_PIN                    GET_PIN(A, 8)    /* TIMER0_CH0 R*/
#define GD32_CP_PIN                    GET_PIN(B, 4)    /* TIMER2_CH0 R*/
#define GD32_RS_PIN                    GET_PIN(B, 6)    /* TIMER3_CH0 R*/
#define GD32_SH_PIN                    GET_PIN(C, 9)    /* TIMER7_CH3 R*/

/* AD9945 涉及到的管脚 */
#define GD32_AD9945_SHP_PIN            GET_PIN(A, 3)    /* TIMER8_CH1 R*/
#define GD32_AD9945_SHD_PIN            GET_PIN(A, 2)    /* TIMER4_CH2 R*/
#define GD32_AD9945_DATACLK_PIN        GET_PIN(A, 5)    /* TIMER1_CH0 R*/
#define GD32_AD9945_CLPOB_PIN          GET_PIN(A, 6)    /* TIMER12_CH0 R*/
#define GD32_AD9945_PBLK_PIN           GET_PIN(A, 7)    /* TIMER13_CH0 R*/

#define GD32_AD9945_SCK_PIN            GET_PIN(C, 2)
#define GD32_AD9945_SDA_PIN            GET_PIN(C, 3)
#define GD32_AD9945_SL_PIN             GET_PIN(A, 1)

#define GD32_AD9945_D0_PIN             GET_PIN(C, 1)
#define GD32_AD9945_D1_PIN             GET_PIN(C, 0)
#define GD32_AD9945_D2_PIN             GET_PIN(B, 9)
#define GD32_AD9945_D3_PIN             GET_PIN(B, 8)
#define GD32_AD9945_D4_PIN             GET_PIN(B, 7)
#define GD32_AD9945_D5_PIN             GET_PIN(B, 11)
#define GD32_AD9945_D6_PIN             GET_PIN(B, 10)
#define GD32_AD9945_D7_PIN             GET_PIN(B, 2)
#define GD32_AD9945_D8_PIN             GET_PIN(B, 1)
#define GD32_AD9945_D9_PIN             GET_PIN(B, 0)
#define GD32_AD9945_D10_PIN            GET_PIN(C, 5)
#define GD32_AD9945_D11_PIN            GET_PIN(C, 4)

#define AD9945_DATA_COUNTS             2100

typedef struct {
    int16_t left;
    int16_t middle;
    int16_t right;
} ccd_data_t;

typedef struct {
    ccd_data_t position;
    ccd_data_t value;
} ccd_data_map_t;

#define CHECK_START_TYPE    0xfe
#define CHECK_END_TYPE      0xff

typedef enum {
    SCAN_TYPE_ONESHOT,
    SCAN_TYPE_CONTINUOUS,
} scan_type_E;

int tcd1209_calibrate_triger(int times);

int tcd1209_calibrate_get_info(unsigned char *value, unsigned char len);

int tcd1209_check_triger(int times);

#endif /* ifndef __TCD_1209_H__ */

