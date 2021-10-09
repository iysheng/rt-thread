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

#define GD32_F1_PIN                    GET_PIN(B, 7)    /* TIMER3_CH1 */
#define GD32_F2_PIN                    GET_PIN(A, 7)    /* TIMER2_CH1 */
#define GD32_CP_PIN                    GET_PIN(B, 8)    /* TIMER9_CH0 */
#define GD32_RS_PIN                    GET_PIN(B, 9)    /* TIMER10_CH0 */
#define GD32_SH_PIN                    GET_PIN(A, 1)    /* TIMER1_CH1 */

/* AD9945 涉及到的管脚 */
#define GD32_AD9945_SHP_PIN            GET_PIN(A, 2)    /* TIMER8_CH0 */
#define GD32_AD9945_SHD_PIN            GET_PIN(C, 6)    /* TIMER7_CH0 */
#define GD32_AD9945_DATACLK_PIN        GET_PIN(A, 3)    /* TIMER4_CH3 */
#define GD32_AD9945_CLPOB_PIN          GET_PIN(B, 13)   /* TIMER0_CH0_ON */
#define GD32_AD9945_PBLK_PIN           GET_PIN(A, 8)    /* TIMER0_CH0 这个是可选的 */

#define GD32_AD9945_SCK_PIN            GET_PIN(C, 7)
#define GD32_AD9945_SDA_PIN            GET_PIN(C, 8)
#define GD32_AD9945_SL_PIN             GET_PIN(C, 9)

#define GD32_AD9945_D0_PIN             GET_PIN(B, 0)
#define GD32_AD9945_D1_PIN             GET_PIN(B, 1)
#define GD32_AD9945_D2_PIN             GET_PIN(B, 2)
#define GD32_AD9945_D3_PIN             GET_PIN(B, 3)
#define GD32_AD9945_D4_PIN             GET_PIN(B, 4)
#define GD32_AD9945_D5_PIN             GET_PIN(B, 5)
#define GD32_AD9945_D6_PIN             GET_PIN(B, 6)
#define GD32_AD9945_D7_PIN             GET_PIN(C, 10)
#define GD32_AD9945_D8_PIN             GET_PIN(C, 11)
#define GD32_AD9945_D9_PIN             GET_PIN(C, 12)
#define GD32_AD9945_D10_PIN            GET_PIN(B, 10)
#define GD32_AD9945_D11_PIN            GET_PIN(B, 11)

#define AD9945_DATA_COUNTS             2500
#endif /* ifndef __TCD_1209_H__ */

