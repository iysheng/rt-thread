/******************************************************************************
* File:             screen_thread.c
*
* Author:           iysheng@163.com  
* Created:          07/28/21 
* Description:      显示屏线程
*****************************************************************************/
#include <rtthread.h>
#include <rtdevice.h>
#include "drv_keyboard.h"
#include "ccd_main_base.h"

#define DBG_LVL               DBG_LOG
#define DBG_TAG               "app.screen"
#include <rtdbg.h>

extern void st7525_draw_point(int x, int y, unsigned int rgb);

static struct EXTERNAL_GFX_OP
{
    void (*draw_pixel)(int x, int y, unsigned int rgb);
    void (*fill_rect)(int x0, int y0, int x1, int y1, unsigned int rgb);
} gs_gui_ops = {
    .draw_pixel = st7525_draw_point,
};

extern void startHelloStar(void* phy_fb, int width, int height, int color_bytes, struct EXTERNAL_GFX_OP* gfx_op);

extern int get_keyboard_keydown(uint32_t *value);

typedef struct {
    unsigned char duanluo_mode;
    ccd_main_config_t duanluo_config_value;
} duanluo_config4screen_t;

static duanluo_config4screen_t gs_duanluo_mode = {
    .duanluo_mode = DUANLUO_LEFT_INDEX, /* 初始表示空 * 模式 */
};

extern ccd_main_system_t gs_ccd_main_sysinfo;
/**
  * @brief 显示段落信息
  * @param duanluo_config4screen_t *duanluo: 
  * retval .
  */
extern void display_modify_duanluo(unsigned char duanluo_index, unsigned short int level);
extern void display_duanluozhi(ccd_main_config_t *ccd_main_config);
extern void display_calibrate_value(unsigned char * level, unsigned char len);
extern int set_ccd_duanluo(unsigned char addr, ccd_main_config_t *config);
extern int set_ccd_duanluo_cankao(unsigned char addr, ccd_main_config_t *config);
extern int set_ccd_duanluo_cankao_delta(unsigned char addr, ccd_main_config_t *config);
extern int get_ccd_calibrate_info(unsigned char addr, unsigned char *value, unsigned char len);

static void do_display_with_duanluo(duanluo_config4screen_t *duanluo)
{
    /* TODO check duanluo mode valid */
    switch (duanluo->duanluo_mode)
    {
        case DUANLUO_LEFT_INDEX:
            display_modify_duanluo(DUANLUO_LEFT_INDEX, duanluo->duanluo_config_value.duanluo_cfg.ccd_duanluo_pos_value.left);
            break;
        case DUANLUO_MIDDLE_INDEX:
            display_modify_duanluo(DUANLUO_MIDDLE_INDEX, duanluo->duanluo_config_value.duanluo_cfg.ccd_duanluo_pos_value.middle);
            break;
        case DUANLUO_RIGHT_INDEX:
            display_modify_duanluo(DUANLUO_RIGHT_INDEX, duanluo->duanluo_config_value.duanluo_cfg.ccd_duanluo_pos_value.right);
            break;
        case DUANLUO_CANKAO_LEFT_INDEX:
            display_modify_duanluo(DUANLUO_CANKAO_LEFT_INDEX, duanluo->duanluo_config_value.duanluo_cfg.ccd_duanluo_pos_value.cankao_left);
            break;
        case DUANLUO_CANKAO_MIDDLE_INDEX:
            display_modify_duanluo(DUANLUO_CANKAO_MIDDLE_INDEX, duanluo->duanluo_config_value.duanluo_cfg.ccd_duanluo_pos_value.cankao_middle);
            break;
        case DUANLUO_CANKAO_RIGHT_INDEX:
            display_modify_duanluo(DUANLUO_CANKAO_RIGHT_INDEX, duanluo->duanluo_config_value.duanluo_cfg.ccd_duanluo_pos_value.cankao_right);
            break;
        case DUANLUO_CANKAO_DELTA_INDEX:
            display_modify_duanluo(DUANLUO_CANKAO_DELTA_INDEX, duanluo->duanluo_config_value.duanluo_cfg.ccd_duanluo_pos_value.cankao_delta[0]);
            break;
        default:
            break;
    }
}

void screen_backend_entry(void * arg)
{
    unsigned char value[6] = {0};
    unsigned char calibrate_info_buffer[6] = {0};
    uint32_t key_value;

    LOG_I("Hello screen");
    startHelloStar(NULL, 256, 160, 2, &gs_gui_ops);
    rt_memcpy(&gs_duanluo_mode.duanluo_config_value, &gs_ccd_main_sysinfo.ccd_main_config, sizeof(ccd_main_config_t));
    LOG_I("DuanLuo[%u,%u,%u]", gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_pos_value.left, gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_pos_value.middle, gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_pos_value.right);
    /* pre init display */
    display_duanluozhi(&gs_duanluo_mode.duanluo_config_value);
    do_display_with_duanluo(&gs_duanluo_mode);
    LOG_I("CanKao[%u,%u,%u@%hu]", gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_pos_value.cankao_left, gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_pos_value.cankao_middle, gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_pos_value.cankao_right, gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_pos_value.cankao_delta[0]);

    while (1)
    {
        rt_thread_mdelay(1000);

        if (!get_keyboard_keydown(&key_value))
        {
            switch(key_value)
            {
                case KEYBOARD_XING:
                    gs_duanluo_mode.duanluo_mode++;
                    gs_duanluo_mode.duanluo_mode %= DUANLUO_NULL_INDEX;
                    do_display_with_duanluo(&gs_duanluo_mode);
                    break;
                case KEYBOARD_JING:
                    if (gs_duanluo_mode.duanluo_mode < DUANLUO_CANKAO_LEFT_INDEX)
                    {
                        display_duanluozhi(&gs_duanluo_mode.duanluo_config_value);
                        set_ccd_duanluo(0x01, &gs_duanluo_mode.duanluo_config_value);
                    }
                    else if (gs_duanluo_mode.duanluo_mode < DUANLUO_CANKAO_DELTA_INDEX)
                    {
                        calibrate_info_buffer[0] = gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_pos_value.cankao_left >> 8;
                        calibrate_info_buffer[1] = gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_pos_value.cankao_left;
                        calibrate_info_buffer[2] = gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_pos_value.cankao_middle >> 8;
                        calibrate_info_buffer[3] = gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_pos_value.cankao_middle;
                        calibrate_info_buffer[4] = gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_pos_value.cankao_right >> 8;
                        calibrate_info_buffer[5] = gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_pos_value.cankao_right;
                        display_calibrate_value(calibrate_info_buffer, 6);
                        set_ccd_duanluo_cankao(0x01, &gs_duanluo_mode.duanluo_config_value);
                    }
                    else if (gs_duanluo_mode.duanluo_mode < DUANLUO_NULL_INDEX)
                    {
                        set_ccd_duanluo_cankao_delta(0x01, &gs_duanluo_mode.duanluo_config_value);
                    }
                    set_ccd_main_config(&gs_duanluo_mode.duanluo_config_value);
                    break;
                case KEYBOARD_UP:
                    /* 触发校准 */
                    tc(0, NULL);
                    if (!get_ccd_calibrate_info(0x01, calibrate_info_buffer, 6))
                    {
                        display_calibrate_value(calibrate_info_buffer, 6);
                        LOG_I("Get calibrate info success");
                    }
                case KEYBOARD_DOWN:
                    gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_value[gs_duanluo_mode.duanluo_mode] = 0;
                    break;
                case KEYBOARD_0:
                    gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_value[gs_duanluo_mode.duanluo_mode] *= 10;
                    break;
                case KEYBOARD_1:
                    gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_value[gs_duanluo_mode.duanluo_mode] *= 10;
                    gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_value[gs_duanluo_mode.duanluo_mode] += 1;
                    break;
                case KEYBOARD_2:
                    gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_value[gs_duanluo_mode.duanluo_mode] *= 10;
                    gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_value[gs_duanluo_mode.duanluo_mode] += 2;
                    break;
                case KEYBOARD_3:
                    gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_value[gs_duanluo_mode.duanluo_mode] *= 10;
                    gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_value[gs_duanluo_mode.duanluo_mode] += 3;
                    break;
                case KEYBOARD_4:
                    gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_value[gs_duanluo_mode.duanluo_mode] *= 10;
                    gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_value[gs_duanluo_mode.duanluo_mode] += 4;
                    break;
                case KEYBOARD_5:
                    gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_value[gs_duanluo_mode.duanluo_mode] *= 10;
                    gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_value[gs_duanluo_mode.duanluo_mode] += 5;
                    break;
                case KEYBOARD_6:
                    gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_value[gs_duanluo_mode.duanluo_mode] *= 10;
                    gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_value[gs_duanluo_mode.duanluo_mode] += 6;
                    break;
                case KEYBOARD_7:
                    gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_value[gs_duanluo_mode.duanluo_mode] *= 10;
                    gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_value[gs_duanluo_mode.duanluo_mode] += 7;
                    break;
                case KEYBOARD_8:
                    gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_value[gs_duanluo_mode.duanluo_mode] *= 10;
                    gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_value[gs_duanluo_mode.duanluo_mode] += 8;
                    break;
                case KEYBOARD_9:
                    gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_value[gs_duanluo_mode.duanluo_mode] *= 10;
                    gs_duanluo_mode.duanluo_config_value.duanluo_cfg.ccd_duanluo_value[gs_duanluo_mode.duanluo_mode] += 9;
                    break;
                default:
                    break;
            }
            do_display_with_duanluo(&gs_duanluo_mode);
        }
    }
}
