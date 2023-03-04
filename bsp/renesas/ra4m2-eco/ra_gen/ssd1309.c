/******************************************************************************
* File:             ssd1309.c
*
* Author:           iysheng@163.com
* Created:          11/17/22
* Description:      ssd1309 驱动
*****************************************************************************/

#include "rtthread.h"
#include "drivers/pin.h"
#include <string.h>
#include "hal_data.h"

typedef unsigned char uint8_t;

#define TAG    "oled"

/* 通过别名获取 "oled_dc" 设备树 node id */
#define OLED_DC_NODE BSP_IO_PORT_04_PIN_05

/* 通过别名获取 "oled_cs" 设备树 node id */
#define OLED_CS_NODE BSP_IO_PORT_00_PIN_04

/* 通过别名获取 "oled_rst" 设备树 node id */
#define OLED_RST_NODE BSP_IO_PORT_00_PIN_02

/* 通过别名获取 "oled_clk" 设备树 node id */
#define OLED_CLK_NODE BSP_IO_PORT_03_PIN_01
/* 通过别名获取 "oled_mosi" 设备树 node id */
#define OLED_MOSI_NODE BSP_IO_PORT_03_PIN_02

static void oled_gpio_init(void)
{
    rt_pin_mode(OLED_DC_NODE, PIN_MODE_OUTPUT);
    rt_pin_mode(OLED_RST_NODE, PIN_MODE_OUTPUT);
    rt_pin_mode(OLED_CS_NODE, PIN_MODE_OUTPUT);
    rt_pin_mode(OLED_CLK_NODE, PIN_MODE_OUTPUT);
    rt_pin_mode(OLED_MOSI_NODE, PIN_MODE_OUTPUT);

    rt_pin_write(OLED_CS_NODE, PIN_LOW);
}

static void lcd_cs(uint8_t d)
{
#if 0
    if (d == 1) {
        rt_pin_write(OLED_CS_NODE, PIN_HIGH);
    } else {
        rt_pin_write(OLED_CS_NODE, PIN_LOW);
    }
#endif
}

static void lcd_dc(uint8_t d)
{
    if (d == 1) {
        rt_pin_write(OLED_DC_NODE, PIN_HIGH);
    } else {
        rt_pin_write(OLED_DC_NODE, PIN_LOW);
    }
}

static int oled_spi_write_data(unsigned char *data, int data_len)
{
    int i = 0;
    unsigned char wdata = data[0];

    for (; i < 8; i++)
    {
        rt_pin_write(OLED_CLK_NODE, PIN_LOW);
        rt_pin_write(OLED_MOSI_NODE, (wdata & 0X80) ? PIN_HIGH : PIN_LOW);
        wdata = wdata << 1;
        rt_pin_write(OLED_CLK_NODE, PIN_HIGH);
    }

    return data_len;
}

void Write_Command(unsigned char Data)
{
    int ret;

    lcd_cs(0);
    lcd_dc(0);
    ret = oled_spi_write_data(&Data, 1);
    if (0 > ret) {
        rt_kprintf("failed send cmd to oled\r\n");
    }
    lcd_dc(1);
    lcd_cs(1);
}

void Write_Data(unsigned char Data)
{
    int ret;

    lcd_cs(0);
    lcd_dc(1);
    ret = oled_spi_write_data(&Data, 1);
    if (0 > ret) {
        rt_kprintf("failed send data to oled\r\n");
    }
    lcd_dc(1);
    lcd_cs(1);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Instruction Setting
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Set_Start_Column(unsigned char d)
{
    Write_Command(0x00 + d % 16); // Set Lower Column Start Address for Page Addressing Mode
    //   Default => 0x00
    Write_Command(0x10 + d / 16); // Set Higher Column Start Address for Page Addressing Mode
    //   Default => 0x10
}

void Set_Addressing_Mode(unsigned char d)
{
    Write_Command(0x20); // Set Memory Addressing Mode
    Write_Command(d);    //   Default => 0x02
    //     0x00 => Horizontal Addressing Mode
    //     0x01 => Vertical Addressing Mode
    //     0x02 => Page Addressing Mode
}

void Set_Column_Address(unsigned char a, unsigned char b)
{
    Write_Command(0x21); // Set Column Address
    Write_Command(a);    //   Default => 0x00 (Column Start Address)
    Write_Command(b);    //   Default => 0x7F (Column End Address)
}

void Set_Page_Address(unsigned char a, unsigned char b)
{
    Write_Command(0x22); // Set Page Address
    Write_Command(a);    //   Default => 0x00 (Page Start Address)
    Write_Command(b);    //   Default => 0x07 (Page End Address)
}

void Set_Start_Line(unsigned char d)
{
    Write_Command(0x40 | d); // Set Display Start Line
    //   Default => 0x40 (0x00)
}

void Set_Contrast_Control(unsigned char d)
{
    Write_Command(0x81); // Set Contrast Control for Bank 0
    Write_Command(d);    //   Default => 0x7F
}

void Set_Segment_Remap(unsigned char d)
{
    Write_Command(d); // Set Segment Re-Map
    //   Default => 0xA0
    //     0xA0 => Column Address 0 Mapped to SEG0
    //     0xA1 => Column Address 0 Mapped to SEG127
}

void Set_Entire_Display(unsigned char d)
{
    Write_Command(d); // Set Entire Display On / Off
    //   Default => 0xA4
    //     0xA4 => Normal Display
    //     0xA5 => Entire Display On
}

void Set_Inverse_Display(unsigned char d)
{
    Write_Command(d); // Set Inverse Display On/Off
    //   Default => 0xA6
    //     0xA6 => Normal Display
    //     0xA7 => Inverse Display On
}

void Set_Multiplex_Ratio(unsigned char d)
{
    Write_Command(0xA8); // Set Multiplex Ratio
    Write_Command(d);    //   Default => 0x3F (1/64 Duty)
}

void Set_Display_On_Off(unsigned char d)
{
    Write_Command(d); // Set Display On/Off
    //   Default => 0xAE
    //     0xAE => Display Off
    //     0xAF => Display On
}

void Set_Start_Page(unsigned char d)
{
    Write_Command(0xB0 | d); // Set Page Start Address for Page Addressing Mode
    //   Default => 0xB0 (0x00)
}

void Set_Common_Remap(unsigned char d)
{
    Write_Command(d); // Set COM Output Scan Direction
    //   Default => 0xC0
    //     0xC0 => Scan from COM0 to 63
    //     0xC8 => Scan from COM63 to 0
}

void Set_Display_Offset(unsigned char d)
{
    Write_Command(0xD3); // Set Display Offset
    Write_Command(d);    //   Default => 0x00
}

void Set_Display_Clock(unsigned char d)
{
    Write_Command(0xD5); // Set Display Clock Divide Ratio / Oscillator Frequency
    Write_Command(d);    //   Default => 0x70
    //     D[3:0] => Display Clock Divider
    //     D[7:4] => Oscillator Frequency
}

void Set_Low_Power(unsigned char d)
{
    Write_Command(0xD8); // Set Low Power Display Mode
    Write_Command(d);    //   Default => 0x04 (Normal Power Mode)
}

void Set_Precharge_Period(unsigned char d)
{
    Write_Command(0xD9); // Set Pre-Charge Period
    Write_Command(d); //   Default => 0x22 (2 Display Clocks [Phase 2] / 2 Display Clocks [Phase 1])
    //     D[3:0] => Phase 1 Period in 1~15 Display Clocks
    //     D[7:4] => Phase 2 Period in 1~15 Display Clocks
}

void Set_Common_Config(unsigned char d)
{
    Write_Command(0xDA); // Set COM Pins Hardware Configuration
    Write_Command(d);    //   Default => 0x12
    //     Alternative COM Pin Configuration
    //     Disable COM Left/Right Re-Map
}

void Set_VCOMH(unsigned char d)
{
    Write_Command(0xDB); // Set VCOMH Deselect Level
    Write_Command(d);    //   Default => 0x34 (0.78*VCC)
}

void Set_NOP()
{
    Write_Command(0xE3); // Command for No Operation
}

void Set_Command_Lock(unsigned char d)
{
    Write_Command(0xFD); // Set Command Lock
    Write_Command(d);    //   Default => 0x12
    //     0x12 => Driver IC interface is unlocked from entering command.
    //     0x16 => All Commands are locked except 0xFD.
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Global Variables
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define XLevelL 0x00
#define XLevelH 0x10
#define XLevel ((XLevelH & 0x0F) * 16 + XLevelL)
#define Max_Column 128
#define Max_Row 64
#define Brightness 0xBF

#define SET_BIT(b, n) b |= (1 << n)
#define CLR_BIT(b, n) b &= ~(uint8_t)(1 << n)

uint8_t g_oled_ram[8][128];

/* 画点函数 */
void oled_draw_point(uint8_t c, uint8_t r, uint8_t t)
{
    if (t) {
        SET_BIT(g_oled_ram[r / 8][c], ((r % 8)));
    } else {
        CLR_BIT(g_oled_ram[r / 8][c], (r % 8));
    }

    Set_Start_Page(r / 8);
    Set_Start_Column(c);
    Write_Data(g_oled_ram[r/8][c]);
}

void oled_draw_frame(uint8_t p[Max_Row][Max_Column])
{
    unsigned char i, j;

    for (i = 0; i < Max_Row; i++) {
        for (j = 0; j < Max_Column; j++) {
            oled_draw_point(j, i, p[i][j]);
        }
    }
}

/* 将缓存中的数据刷新显示 */
void oled_reflesh()
{
    unsigned char i, j;
    for (i = 0; i < 8; i++) {
        Set_Start_Page(i);
        Set_Start_Column(0x00);

        for (j = 0; j < 128; j++) {
            Write_Data(g_oled_ram[i][j]);
        }
    }
}

void Fill_RAM(unsigned char Data)
{
    unsigned char i, j;

    for (i = 0; i < 8; i++) {
        Set_Start_Page(i);
        Set_Start_Column(0x00);

        for (j = 0; j < 128; j++) {
            Write_Data(Data);
        }
    }
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Initialization
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
static void oled_initialize()
{
    rt_pin_write(OLED_RST_NODE, PIN_LOW);
    rt_thread_mdelay(100);
    rt_pin_write(OLED_RST_NODE, PIN_HIGH);
    rt_thread_mdelay(150);

    /* oled 初始化 */
    Set_Command_Lock(0x12);           // Unlock Driver IC (0x12/0x16)
    Set_Display_On_Off(0xAE);         // Display Off (0xAE/0xAF)
    Set_Display_Clock(0xA0);          // Set Clock as 116 Frames/Sec
    Set_Multiplex_Ratio(0x3F);        // 1/64 Duty (0x0F~0x3F)
    Set_Display_Offset(0x00);         // Shift Mapping RAM Counter (0x00~0x3F)
    Set_Start_Line(0x00);             // Set Mapping RAM Display Start Line (0x00~0x3F)
    Set_Low_Power(0x04);              // Set Normal Power Mode (0x04/0x05)
    Set_Addressing_Mode(0x02);        // Set Page Addressing Mode (0x00/0x01/0x02)
    Set_Segment_Remap(0xA1);          // Set SEG/Column Mapping (0xA0/0xA1)
    Set_Common_Remap(0xC8);           // Set COM/Row Scan Direction (0xC0/0xC8)
    Set_Common_Config(0x12);          // Set Alternative Configuration (0x02/0x12)
    Set_Contrast_Control(Brightness); // Set SEG Output Current
    Set_Precharge_Period(0x82);       // Set Pre-Charge as 8 Clocks & Discharge as 2 Clocks
    Set_VCOMH(0x34);                  // Set VCOM Deselect Level
    Set_Entire_Display(0xA4);         // Disable Entire Display On (0xA4/0xA5)
    Set_Inverse_Display(0xA6);        // Disable Inverse Display On (0xA6/0xA7)

    /* 清屏 */
    Fill_RAM(0x00); // Clear Screen

    Set_Display_On_Off(0xAF); // Display On (0xAE/0xAF)
}

uint8_t g_bm_rv[64][128] = {{0x55,0X55,0X55}};

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Main Program
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void ssd1309_init()
{
    /* 硬件 oled 初始化 */
    oled_gpio_init();
    /* oled 初始化 */
    oled_initialize();

    int i,j;
    /* 显示欢迎界面 */
    for (i = 0; i < 64; i++)
        for (j = 0; j < 128; j++)
            g_bm_rv[i][j] = (i+j) % 2;
    /* 将数据写到缓存 */
    oled_draw_frame(g_bm_rv);
    /* 刷新屏幕显示 */
    //oled_reflesh();
    rt_kprintf("SSD1309 init done.\n");
}

