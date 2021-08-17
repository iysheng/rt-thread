/******************************************************************************
* File:             screen_thread.c
*
* Author:           iysheng@163.com  
* Created:          07/28/21 
* Description:      显示屏线程
*****************************************************************************/
#include <rtthread.h>
#include <rtdevice.h>

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
void screen_backend_entry(void * arg)
{
    unsigned char value[6] = {0};
    uint32_t key_value;
    LOG_I("Hello screen");
    startHelloStar(NULL, 256, 160, 2, &gs_gui_ops);

    while (1)
    {
        rt_thread_mdelay(1000);

        if (!get_keyboard_keydown(&key_value))
        {
            value[1] = key_value & 0xff;
            display_check_value(value, 6);
        }
    }
}
