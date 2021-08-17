#define GUILITE_ON  //Do not define this macro once more!!!
#include "GuiLite.h"
#include <stdlib.h>

#define UI_WIDTH 256
#define UI_HEIGHT 160

static c_surface* s_surface;
static c_display* s_display;

enum WND_ID
{
    ID_ROOT = 1,
    ID_LABEL_CANKAO,
    ID_LABEL_ZONGBAOJING,
    ID_LABEL_ZONGJIANCE,
    ID_LABEL_JIANCE,
    ID_LABEL_MODIFYDUANLUO,
    ID_LABEL_DUANLUO,
    ID_LABEL_CANKAOZHI,
    ID_LABEL_ZONGBAOJINGZHI,
    ID_LABEL_ZONGJIANCEZHI,
    ID_LABEL_JIANCEZHI,
    ID_LABEL_MODIFYDUANLUOZHI,
    ID_LABEL_DUANLUOZHI,
    ID_LABEL_MAX
};

class c_my_ui : public c_wnd
{
	virtual void on_init_children()
	{
	}
	virtual void on_paint(void)
	{
		c_label * c_label_ptr;

extern const BITMAP_INFO biaopan0_bmp;
extern const BITMAP_INFO jari_logo_40_bmp;
#if 0
        c_bitmap::draw_bitmap(m_surface, Z_ORDER_LEVEL_0, &biaopan0_bmp, 156, 60);

        c_label_ptr = (c_label *)get_wnd_ptr(ID_CIRCLE);
        c_label_ptr->set_align_type(ALIGN_HCENTER | ALIGN_VCENTER);
        c_label_ptr = (c_label *)get_wnd_ptr(ID_CIRCLE_HAN);
        c_label_ptr->set_align_type(ALIGN_HCENTER | ALIGN_VCENTER);
#endif
    }
public:
    static int s_init_my_ui_flag;
	GL_DECLARE_MESSAGE_MAP()//delcare message
};

int c_my_ui::s_init_my_ui_flag = 0;

GL_BEGIN_MESSAGE_MAP(c_my_ui)
GL_END_MESSAGE_MAP()

// Layout Widgets
static c_my_ui * gs_my_ui;
static c_surface_no_fb * gs_surface_no_fb;

#define EAST_CODE   "\xe4\xb8\x9c"
#define NORTH_CODE  "\xe5\x8c\x97"
#define SOURCE_CODE "\xe5\x8d\x97"
#define WEST_CODE   "\xE8\xA5\xBF"

WND_TREE s_main_widgets[] =
{
    /* 参考 */
    { NULL,        ID_LABEL_CANKAO,    "\xe5\x8f\x82\xe8\x80\x83\x3a",    5, 6, 40, 20},
    /* 总报警 */
    { NULL,        ID_LABEL_ZONGBAOJING,    "\xe6\x80\xbb\xe6\x8a\xa5\xe8\xad\xa6\x3a",    5, 32, 60, 20},
    /* 总检测 */
    { NULL,        ID_LABEL_ZONGJIANCE,    "\xe6\x80\xbb\xe6\xa3\x80\xe6\xb5\x8b\x3a",    5, 58, 60, 20},
    /* 检测 */
    { NULL,        ID_LABEL_JIANCE,    "\xe6\xa3\x80\xe6\xb5\x8b\x3a",    5, 84, 40, 20},
    /* 左 */
    { NULL,        ID_LABEL_MODIFYDUANLUO,    "\xe5\xb7\xa6",    12, 112, 20, 20},
    /* 左中右 */
    { NULL,        ID_LABEL_DUANLUO,    "\xe5\xb7\xa6\xE4\xB8\xAD\xe5\x8f\xb3\x3a",    45, 138, 65, 20},

    /* 参考值 */
    { NULL,        ID_LABEL_CANKAOZHI,    ",,",    55, 6, 175, 20},
    /* 总报警次数 */
    { NULL,        ID_LABEL_ZONGBAOJINGZHI,    "0",    75, 32, 150, 20},
    /* 总检测次数 */
    { NULL,        ID_LABEL_ZONGJIANCEZHI,    "0",    75, 58, 150, 20},
    /* 详细检测 */
    { NULL,        ID_LABEL_JIANCEZHI,     ",,",   55, 84, 175, 20},
    /* 左/中/右 分段修改 */
    { NULL,        ID_LABEL_MODIFYDUANLUO,    "9999",    0, 138, 44, 20},
    /* 左中右分段 */
    { NULL,        ID_LABEL_DUANLUO,    "1000,2000,1000",    115, 138, 130, 20},

    { NULL, 0 , 0, 0, 0, 0, 0}
};

// Create GUI
extern const FONT_INFO yahei_22;
extern const FONT_INFO yahei_16;
extern const FONT_INFO songti_15;
void load_resource()
{
	c_theme::add_font(FONT_DEFAULT, &songti_15);
	c_theme::add_font(FONT_CUSTOM1, &yahei_16);
	c_theme::add_font(FONT_CUSTOM2, &yahei_22);
	c_theme::add_color(COLOR_WND_FONT, 1);
	c_theme::add_color(COLOR_WND_NORMAL, 1);
}

static c_label gs_label4dispaly[ID_LABEL_MAX];

static inline void widgets_pre_init(WND_TREE *tree)
{
    int i = 0;

#if 0
    gs_label4dispaly[ID_LABEL_CANKAOZHI-2].set_font_type(&yahei_16);
    gs_label4dispaly[ID_LABEL_ZONGBAOJINGZHI-2].set_font_type(&yahei_16);
    gs_label4dispaly[ID_LABEL_ZONGJIANCEZHI-2].set_font_type(&yahei_16);
    gs_label4dispaly[ID_LABEL_JIANCEZHI-2].set_font_type(&yahei_16);
#endif
    while (tree[i].resource_id)
    {
        tree[i].p_wnd = &gs_label4dispaly[i];
        i++;
    }
}
//////////////////////// start UI ////////////////////////

void create_ui(void* phy_fb, int screen_width, int screen_height, int color_bytes, struct EXTERNAL_GFX_OP* gfx_op) {
    gs_my_ui = new c_my_ui();
    gs_my_ui->set_bg_color(0);
    gs_my_ui->set_font_color(1);
    load_resource();
    widgets_pre_init(s_main_widgets);

    gs_surface_no_fb = new c_surface_no_fb(UI_WIDTH, UI_HEIGHT, color_bytes, gfx_op, Z_ORDER_LEVEL_0);

    s_display = new c_display(phy_fb, screen_width, screen_height, gs_surface_no_fb);

    /* 设置当前 c_wnd 的 m_surface 成员 */
    gs_my_ui->set_surface(gs_surface_no_fb);
    gs_my_ui->connect(NULL, ID_ROOT, 0, 0, 0, UI_WIDTH, UI_HEIGHT, s_main_widgets);
    gs_my_ui->show_window();

    gs_surface_no_fb->fill_rect(0, 109, 44, 109, GL_RGB(0, 0, 0), Z_ORDER_LEVEL_0);
    gs_surface_no_fb->fill_rect(44, 135, 256, 135, GL_RGB(0, 0, 0), Z_ORDER_LEVEL_0);
    gs_surface_no_fb->fill_rect(44, 109, 44, 160, GL_RGB(0, 0, 0), Z_ORDER_LEVEL_0);
    /* 標記 GuiLite 初始化完成 */
    c_my_ui::s_init_my_ui_flag = 1;
}

//////////////////////// interface for all platform ////////////////////////
extern "C" void startHelloStar(void* phy_fb, int width, int height, int color_bytes, struct EXTERNAL_GFX_OP* gfx_op) {
	create_ui(phy_fb, width, height, color_bytes, gfx_op);
}

extern const BITMAP_INFO alarm_bmp;
extern "C" void display_check_ans(int alarm, unsigned int check_value)
{
    static unsigned int s_total_alram_counts;
    static unsigned int s_total_check_counts;
    c_label * wind_alarm_label = (c_label *)gs_my_ui->get_wnd_ptr(ID_LABEL_ZONGBAOJINGZHI);
    c_label * wind_check_label = (c_label *)gs_my_ui->get_wnd_ptr(ID_LABEL_ZONGJIANCEZHI);
    char alarm_buffer[16] = {0};
    if (!c_my_ui::s_init_my_ui_flag)
    {
        rt_kprintf("GuiLite is not ready\n");
        return;
    }

    s_total_check_counts++;
    if (alarm == 1)
    {
        s_total_alram_counts++;
        c_bitmap::draw_bitmap(gs_surface_no_fb , Z_ORDER_LEVEL_0, &alarm_bmp, 216, 0);
    }
    else
    {
        c_bitmap::hide_bitmap(gs_surface_no_fb , Z_ORDER_LEVEL_0, &alarm_bmp, 216, 0);
    }

    if (wind_alarm_label)
    {
        if (alarm > 0)
        {
            snprintf(alarm_buffer, sizeof(alarm_buffer), "%u", s_total_alram_counts);
            wind_alarm_label->set_str(alarm_buffer);
            wind_alarm_label->show_window();
        }
    }
    if (wind_check_label)
    {
        memset(alarm_buffer, 0, sizeof alarm_buffer);
        snprintf(alarm_buffer, sizeof(alarm_buffer), "%u", s_total_check_counts);
        wind_check_label->set_str(alarm_buffer);
        wind_check_label->show_window();
    }
}

extern "C" void display_calibrate_value(unsigned char * level, unsigned char len)
{
    c_label * wind_level_label = (c_label *)gs_my_ui->get_wnd_ptr(ID_LABEL_CANKAOZHI);
    char level_buffer[16] = {0};

    if (!c_my_ui::s_init_my_ui_flag)
    {
        rt_kprintf("GuiLite is not ready\n");
        return;
    }

    if (wind_level_label)
    {
        if (level >= 0)
        {
            snprintf(level_buffer, sizeof(level_buffer), "%hu,%hu,%hu", level[0] << 8 | level[1], \
                level[2] << 8 | level[3], \
                level[4] << 8 | level[5]);
        }
        else
        {
            snprintf(level_buffer, sizeof(level_buffer), "*,*,*");
        }
        wind_level_label->set_str(level_buffer);
        wind_level_label->show_window();
    }
}
extern "C" void display_check_value(unsigned char * level, unsigned char len)
{
    c_label * wind_level_label = (c_label *)gs_my_ui->get_wnd_ptr(ID_LABEL_JIANCEZHI);
    char level_buffer[16] = {0};

    if (!c_my_ui::s_init_my_ui_flag)
    {
        rt_kprintf("GuiLite is not ready\n");
        return;
    }

    if (wind_level_label)
    {
        if (level >= 0)
        {
            snprintf(level_buffer, sizeof(level_buffer), "%hu,%hu,%hu", level[0] << 8 | level[1], \
                level[2] << 8 | level[3], \
                level[4] << 8 | level[5]);
        }
        else
        {
            snprintf(level_buffer, sizeof(level_buffer), "*,*,*");
        }
        wind_level_label->set_str(level_buffer);
        wind_level_label->show_window();
    }
}

void* getUiOfHelloStar(int* width, int* height, bool force_update)
{
	if (s_display)
	{
		return s_display->get_updated_fb(width, height, force_update);
	}
	return NULL;
}
