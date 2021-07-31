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
    ID_TITLE,
    ID_LABEL_CANKAO,
    ID_LABEL_ZONGBAOJING,
    ID_LABEL_SHIFOUBAOJING,
    ID_LABEL_ZONGJIANCE,
    ID_LABEL_JIANCE,
    ID_LABEL_CANKAOZHI,
    ID_LABEL_ZONGBAOJINGZHI,
    ID_LABEL_SHIFOUBAOJINGZHI,
    ID_LABEL_ZONGJIANCEZHI,
    ID_LABEL_JIANCEZHI,
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
	{ NULL,		ID_LABEL_CANKAO,	"\xe5\x8f\x82\xe8\x80\x83\x3a",	0, 0, 75, 38},
	/* 总报警 */
	{ NULL,		ID_LABEL_ZONGBAOJING,	"\xe6\x80\xbb\xe6\x8a\xa5\xe8\xad\xa6\x3a",	0, 40, 115, 38},
	/* 总检测 */
	{ NULL,		ID_LABEL_ZONGJIANCE,	"\xe6\x80\xbb\xe6\xa3\x80\xe6\xb5\x8b\x3a",	0, 80, 115, 38},
	/* 检测 */
	{ NULL,		ID_LABEL_JIANCE,	"\xe6\xa3\x80\xe6\xb5\x8b\x3a",	0, 120, 75, 38},

	/* 参考值 */
	{ NULL,		ID_LABEL_CANKAOZHI,	"1.2.3",	75, 0, 60, 38},
	/* 总报警次数 */
	{ NULL,		ID_LABEL_ZONGBAOJINGZHI,	"0",	115, 40, 150, 38},
	/* 总检测次数 */
	{ NULL,		ID_LABEL_ZONGJIANCEZHI,	"0",	115, 80, 150, 38},
	/* 检测详细 */
	{ NULL,		ID_LABEL_JIANCEZHI,	"4.5.6",	75, 120, 175, 38},

	{ NULL, 0 , 0, 0, 0, 0, 0}
};

// Create GUI
extern const FONT_INFO yahei_22;
extern const FONT_INFO yahei_16;
void load_resource()
{
	c_theme::add_font(FONT_DEFAULT, &yahei_22);
	c_theme::add_font(FONT_CUSTOM1, &yahei_16);
	c_theme::add_color(COLOR_WND_FONT, 1);
	c_theme::add_color(COLOR_WND_NORMAL, 1);
}

static c_label gs_label4dispaly[ID_LABEL_MAX];

static inline void widgets_pre_init(WND_TREE *tree)
{
    int i = 0;

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

extern "C" void display_check_value(unsigned int level)
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
        memset(level_buffer, 0, sizeof level_buffer);
        if (level >= 0)
        {
            snprintf(level_buffer, sizeof(level_buffer), "%d", level);
        }
        else
        {
            snprintf(level_buffer, sizeof(level_buffer), "**");
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
