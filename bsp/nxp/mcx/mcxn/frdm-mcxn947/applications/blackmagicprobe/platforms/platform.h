/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file provides the platform specific declarations for the native implementation. */

#ifndef PLATFORMS_NXP_PLATFORM_H
#define PLATFORMS_NXP_PLATFORM_H

#define PLATFORM_IDENT   "NXP"

//running_status = (state)
#define SET_RUN_STATE(state)
#define SET_IDLE_STATE(state)
#define SET_ERROR_STATE(state)

/* Hardware definitions... */
#define JTAG_PORT    3


#define TMS_PORT     JTAG_PORT
#define TCK_PORT     JTAG_PORT
#define TMS_PIN      20
#define TCK_PIN      21

#define TMS_DIR_PORT 1 // P1_0 备用
#define TMS_DIR_PIN  0 // 0 表示没用
#define TCK_DIR_PORT 1 // P1_1 备用
#define TCK_DIR_PIN  1

#define TDI_PORT     1
#define TDO_PORT     1
#define TDI_PIN      16 // P1_16 备用
#define TDO_PIN      17 // P1_17 备用

#define SWDIO_DIR_PORT JTAG_PORT
#define SWDIO_PORT     JTAG_PORT
#define SWCLK_PORT     JTAG_PORT
#define SWDIO_DIR_PIN  TMS_DIR_PIN
#define SWDIO_PIN      TMS_PIN  // P3_19
#define SWCLK_PIN      TCK_PIN  // P3_20

#endif /* PLATFORMS_NATIVE_PLATFORM_H */
