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
#define GD32_AD9945_SHP_PIN            GET_PIN(A, 2)    /* TIMER4_CH2 */
#define GD32_AD9945_SHD_PIN            GET_PIN(C, 6)    /* TIMER7_CH0 */
#define GD32_AD9945_DATACLK_PIN        GET_PIN(A, 3)    /* TIMER8_CH1 */
#define GD32_AD9945_CLPOB_PIN          GET_PIN(B, 13)   /* TIMER0_CH0_ON */
#define GD32_AD9945_PBLK_PIN           GET_PIN(A, 8)    /* TIMER0_CH0 这个是可选的 */

#endif /* ifndef __TCD_1209_H__ */

