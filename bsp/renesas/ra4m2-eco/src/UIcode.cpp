#define GUILITE_ON  //Do not define this macro once more!!!
#include "GuiLite.h"
#include <stdlib.h>

#define UI_WIDTH 128
#define UI_HEIGHT 64

static c_surface* s_surface;
static c_display* s_display;

extern const LATTICE_FONT_INFO youshehaoshenti_10;
extern const LATTICE_FONT_INFO youshehaoshenti_10;

class c_star {
public:
};

//////////////////////// start UI ////////////////////////

extern "C" {
    void display_can_frame(unsigned char *data, int len, unsigned int id)
    {
        char data_display[16] = {0};
        /* 为了演示,只打印 8 字节长度的数据帧 */
        if (len != 8)
            return;
        s_surface->fill_rect(0, 0, UI_WIDTH - 1, UI_HEIGHT - 1, 0, Z_ORDER_LEVEL_0);
        snprintf(data_display, 16, "%#02X %#02X %#02X", data[0], data[1], data[2]);
        c_word::draw_string(s_surface, Z_ORDER_LEVEL_0, data_display, 0, 0, c_theme::get_font(FONT_DEFAULT), GL_RGB(1, 0, 0), GL_ARGB(0, 0, 0, 0));
        memset(data_display, 0, sizeof data);
        snprintf(data_display, 16, "%#02X %#02X %#02X", data[3], data[4], data[5]);
        c_word::draw_string(s_surface, Z_ORDER_LEVEL_0, data_display, 0, 18, c_theme::get_font(FONT_DEFAULT), GL_RGB(1, 0, 0), GL_ARGB(0, 0, 0, 0));
        memset(data_display, 0, sizeof data);
        snprintf(data_display, 16, "%#02X %#02X :%#04X", data[6], data[7], id);
        c_word::draw_string(s_surface, Z_ORDER_LEVEL_0, data_display, 0, 36, c_theme::get_font(FONT_DEFAULT), GL_RGB(1, 0, 0), GL_ARGB(0, 0, 0, 0));
    }
}

c_star stars[100];
void create_ui(void* phy_fb, int screen_width, int screen_height, int color_bytes, struct DISPLAY_DRIVER* driver) {
	c_theme::add_font(FONT_DEFAULT, &youshehaoshenti_10);
	static c_surface surface(UI_WIDTH, UI_HEIGHT, color_bytes, Z_ORDER_LEVEL_0);
	static c_display display(phy_fb, screen_width, screen_height, &surface, driver);
	s_surface = &surface;
	s_display = &display;

	s_surface->fill_rect(0, 0, UI_WIDTH - 1, UI_HEIGHT - 1, 0, Z_ORDER_LEVEL_0);
	char data[11] = {0X0};

	int num = 1;
	while(1) {
#if 0
		snprintf(data, 10, "%d", num++);
	    s_surface->fill_rect(0, 0, UI_WIDTH - 1, 20, 0, Z_ORDER_LEVEL_0);
    	c_word::draw_string(s_surface, Z_ORDER_LEVEL_0, data, 0, 0, c_theme::get_font(FONT_DEFAULT), GL_RGB(1, 0, 0), GL_ARGB(0, 0, 0, 0));
#endif
		thread_sleep(1);
	}
}

//////////////////////// interface for all platform ////////////////////////
extern "C" void startHelloStar(void* phy_fb, int width, int height, int color_bytes, struct DISPLAY_DRIVER* driver) {
	create_ui(phy_fb, width, height, color_bytes, driver);
}

void* getUiOfHelloStar(int* width, int* height, bool force_update)
{
	if (s_display)
	{
		return s_display->get_updated_fb(width, height, force_update);
	}
	return NULL;
}
