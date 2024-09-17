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

/* This file implements the platform specific functions for the native implementation. */

#include "general.h"
#include "platform.h"

uint32_t target_clk_divider = 0;

uint32_t platform_time_ms(void)
{
	return (uint32_t)rt_tick_get_millisecond();
}

void platform_target_clk_output_enable(bool enable)
{
}

int platform_hwversion(void)
{
	return 0X123;
}

 const char *platform_target_voltage(void)
 {
	return "3.2V";
 }
 
 void platform_nrst_set_val(bool assert)
 {
	return;
 }
 
 bool platform_nrst_get_val(void) 
 {
	 return true;
 }
	 
 void platform_max_frequency_set(const uint32_t frequency)
 {
 
 }
 
 void platform_delay(uint32_t ms)
 {
	rt_thread_mdelay(ms);
 }

 
 uint32_t platform_max_frequency_get(void)
 {
	return 100000;
 }
 
// TODO insert this func
unsigned int swdptap_bit_in(void)
{
	unsigned int ret = 0;
	ret = rt_pin_read(NXP_SWDIO_PIN) == PIN_HIGH ? 1 : 0;
    return ret;
}

// TODO insert this func
void swdptap_bit_out(unsigned char value)
{
    if (value)
			rt_pin_write(NXP_SWDIO_PIN, PIN_HIGH);
		else
			rt_pin_write(NXP_SWDIO_PIN, PIN_LOW);
}

void nxp4bmp_platform_init(void)
{
	rt_pin_mode(NXP_SWDIO_PIN, PIN_MODE_OUTPUT);
	rt_pin_mode(NXP_SWCLK_PIN, PIN_MODE_OUTPUT);
	rt_pin_write(NXP_SWCLK_PIN, PIN_LOW);
	rt_pin_write(NXP_SWDIO_PIN, PIN_LOW);
	rt_thread_mdelay(100);
				rt_pin_write(NXP_SWCLK_PIN, PIN_HIGH);
	rt_pin_write(NXP_SWDIO_PIN, PIN_HIGH);
		rt_thread_mdelay(100);
	rt_pin_write(NXP_SWDIO_PIN, PIN_LOW);
			rt_pin_write(NXP_SWCLK_PIN, PIN_LOW);
	rt_thread_mdelay(100);
}

extern void vcom_putchar(const char c, const int flush);
// TODO insert this func later
//  send c to in ep
// flush == 1, and when full, just flush, or will just save to buffer_send here
void gdb_if_putchar(const char c, const int flush)
{
	vcom_putchar(c, flush);
}

extern char vcom_getchar(void);
// when no data get return \x04
// if get data then return one char
char gdb_if_getchar(void)
{
	return vcom_getchar();
}

extern char vcom_getchar_to(const uint32_t timeout);
//  超时返回 -1
// 正常读到数据返回数据
// 否则返回 \x04
char gdb_if_getchar_to(const uint32_t timeout) 
{
	return vcom_getchar_to(timeout);
}

void platform_init(void)
{

}

void debug_serial_send_stdout(const uint8_t *const data, const size_t len)
{

}