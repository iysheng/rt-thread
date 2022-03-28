/*
 * This file is part of the EasyFlash Library.
 *
 * Copyright (c) 2015-2019, Armink, <armink.ztl@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: Portable interface for each platform.
 * Created on: 2015-01-16
 */

#include <easyflash.h>
#include <stdarg.h>

static char rt_log_buf[128];
int rt_vsnprintf(char *buf, int size, const char *fmt, va_list args);
int rt_snprintf(char *buf, int size, const char *fmt, ...);
void rt_kprintf(const char *fmt, ...);
extern void fmc_lock(void);
extern void fmc_unlock(void);
/* default environment variables set for user */
static const ef_env default_env_set[] = {
    {"boot_times", "0", 2}
};

/**
 * Flash port for hardware initialize.
 *
 * @param default_env default ENV set for user
 * @param default_env_size default ENV size
 *
 * @return result
 */
EfErrCode ef_port_init(ef_env const **default_env, size_t *default_env_size) {
    EfErrCode result = EF_NO_ERR;

    *default_env = default_env_set;
    *default_env_size = sizeof(default_env_set) / sizeof(default_env_set[0]);

    return result;
}

/**
 * Read data from flash.
 * @note This operation's units is word.
 *
 * @param addr flash address
 * @param buf buffer to store read data
 * @param size read bytes size
 *
 * @return result
 */
EfErrCode ef_port_read(uint32_t addr, uint32_t *buf, size_t size) {
    EfErrCode result = EF_NO_ERR;
    int i = 0;
    uint8_t * buffer_ptr = (uint8_t *)buf;

    for (; i < size; i++)
    {
        *buffer_ptr++ = *((uint8_t *)addr++);
    }

    return result;
}

/**
 * Erase data on flash.
 * @note This operation is irreversible.
 * @note This operation's units is different which on many chips.
 *
 * @param addr flash address
 * @param size erase bytes size
 *
 * @return result
 */
EfErrCode ef_port_erase(uint32_t addr, size_t size) {
extern int fmc_page_erase(uint32_t page_address);
    int i = 0;
    EfErrCode result = EF_NO_ERR;

    /* make sure the start address is a multiple of EF_ERASE_MIN_SIZE */
    EF_ASSERT(addr % EF_ERASE_MIN_SIZE == 0);

    fmc_unlock();
    /* You can add your code under here. */
    for (; i < size / EF_ERASE_MIN_SIZE; i++)
    {
        fmc_page_erase(addr + i * EF_ERASE_MIN_SIZE);
    }
    fmc_lock();

    return result;
}

/**
 * Write data to flash.
 * @note This operation's units is word.
 * @note This operation must after erase. @see flash_erase.
 *
 * @param addr flash address
 * @param buf the write data buffer
 * @param size write bytes size
 *
 * @return result
 */
EfErrCode ef_port_write(uint32_t addr, const uint32_t *buf, size_t size) {
    EfErrCode result = EF_NO_ERR;
    int i = 0;
extern int fmc_word_program(uint32_t address, uint32_t data);
    fmc_unlock();
    /* You can add your code under here. */
    for (; i < size; i += 4)
        fmc_word_program(addr + i, *buf++);
    fmc_lock();

    return result;
}

/**
 * lock the ENV ram cache
 */
void ef_port_env_lock(void) {
}

/**
 * unlock the ENV ram cache
 */
void ef_port_env_unlock(void) {
    fmc_unlock();
}


/**
 * This function is print flash debug info.
 *
 * @param file the file which has call this function
 * @param line the line number which has call this function
 * @param format output format
 * @param ... args
 *
 */
void ef_log_debug(const char *file, const long line, const char *format, ...) {
#ifdef PRINT_DEBUG
    int length;
    va_list args;
    va_start(args, format);
    length = rt_snprintf(rt_log_buf, sizeof(rt_log_buf) - 1, "%s@%d ", file, line);
    length += rt_vsnprintf(rt_log_buf + length, sizeof(rt_log_buf) - 1 - length, format, args);
    rt_kprintf("%s", rt_log_buf);
    va_end(args);
#endif

}

/**
 * This function is print flash routine info.
 *
 * @param format output format
 * @param ... args
 */
void ef_log_info(const char *format, ...) {
    va_list args;
    int length;

    /* args point to the first variable parameter */
    va_start(args, format);
    length = rt_vsnprintf(rt_log_buf, sizeof(rt_log_buf) - 1, format, args);
    rt_kprintf("%s\n", rt_log_buf);
    va_end(args);
}
/**
 * This function is print flash non-package info.
 *
 * @param format output format
 * @param ... args
 */
void ef_print(const char *format, ...) {
    int length;
    va_list args;

    /* args point to the first variable parameter */
    va_start(args, format);
    length = rt_vsnprintf(rt_log_buf, sizeof(rt_log_buf) - 1, format, args);
    rt_kprintf("%s\n", rt_log_buf);
    va_end(args);
}
