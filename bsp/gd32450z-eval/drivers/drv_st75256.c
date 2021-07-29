/******************************************************************************
* File:             drv_st75256.c
*
* Author:           iysheng@163.com  
* Created:          06/26/21 
* Description:      ST75256 显示屏幕驱动程序
*****************************************************************************/

#include "drv_gpio.h"
#include<rtthread.h>
#include<rtdevice.h>

#define WHITE_RGB_CODE      0x00
#define BLACK_RGB_CODE      0xff
unsigned char gs_display_buffer[256][20];

/* PD[0..7] LCD_AD[0..7]
 * PD8      LCD_CS
 * PD9      LCD_A0
 * PD10     LCD_ERD
 * PD11     LCD_RWR
 * PD12     LCD_RSTB
 * */

#define DBG_LEVEL  DBG_LOG
#define DBG_TAG    "drv.encoder"
#include <rtdbg.h>

#define LCD_CS_PIN      GPIO_PIN_8
#define LCD_A0_PIN      GPIO_PIN_9
#define LCD_ERD_PIN     GPIO_PIN_10
#define LCD_PWR_PIN     GPIO_PIN_11
#define LCD_RSTB_PIN    GPIO_PIN_12

/******************************************************************************
* Function:         static void write_cmd
* Description:      写命令
* Where:
*                   uint8_t cmd - TODO
* Return:           
* Error:            
*****************************************************************************/
static void write_cmd(uint8_t cmd)
{
    uint8_t bit = 0;
    /* TODO A0 = 0 ERD = 1 R/W = 0 */
    gpio_bit_write(GPIOD, LCD_A0_PIN, RESET);
    gpio_bit_write(GPIOD, LCD_ERD_PIN, SET);
    gpio_bit_write(GPIOD, LCD_PWR_PIN, RESET);
    for (; bit < 8; bit++)
    {
        gpio_bit_write(GPIOD, 1 << bit, cmd & 0x01 ? SET : RESET);
        cmd >>= 1;
    }
    gpio_bit_write(GPIOD, LCD_ERD_PIN, RESET);
}
/******************************************************************************
* Function:         static uint8_t read_cmd
*                   读命令
* Where:
*                   uint8_t addr - TODO
* Return:           
* Error:            
*****************************************************************************/
static uint8_t read_cmd(uint8_t addr)
{
    /* TODO A0 = 0 ERD = 1 R/W = 1 */
    gpio_bit_write(GPIOD, LCD_A0_PIN, RESET);
    gpio_bit_write(GPIOD, LCD_ERD_PIN, SET);
    gpio_bit_write(GPIOD, LCD_PWR_PIN, SET);
    GPIO_OCTL(GPIOD) &= 0xffffff00;
    GPIO_OCTL(GPIOD) |= addr;
}

/******************************************************************************
* Function:         static void write_data
* Description:      写命令
* Where:
*                   uint8_t data - TODO
* Return:           
* Error:            
*****************************************************************************/
static void write_data(uint8_t data)
{
    uint8_t bit = 0;
    /* TODO A0 = 1 ERD = 1 R/W = 0 */
    gpio_bit_write(GPIOD, LCD_A0_PIN, SET);
    gpio_bit_write(GPIOD, LCD_ERD_PIN, SET);
    gpio_bit_write(GPIOD, LCD_PWR_PIN, RESET);
    for (; bit < 8; bit++)
    {
        gpio_bit_write(GPIOD, 1 << bit, data & 0x01 ? SET : RESET);
        data >>= 1;
    }
    gpio_bit_write(GPIOD, LCD_ERD_PIN, RESET);
}
/******************************************************************************
* Function:         static uint8_t read_data
*                   读命令
* Where:
*                   uint8_t addr - TODO
* Return:           
* Error:            
*****************************************************************************/
static uint8_t read_data(uint8_t addr)
{
    /* TODO A0 = 1 ERD = 1 R/W = 1 */
    gpio_bit_write(GPIOD, LCD_A0_PIN, SET);
    gpio_bit_write(GPIOD, LCD_ERD_PIN, SET);
    gpio_bit_write(GPIOD, LCD_PWR_PIN, SET);
    GPIO_OCTL(GPIOD) &= 0xffffff00;
    GPIO_OCTL(GPIOD) |= addr;
    return 0;
}


static void _st75256_pin_init(void)
{
#if 0
    rt_pin_mode(LCD_CS_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LCD_A0_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LCD_ERD_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LCD_PWR_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LCD_RSTB_PIN, PIN_MODE_OUTPUT);
#endif
    gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_0);
    gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_1);
    gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_2);
    gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_3);
    gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_4);
    gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_5);
    gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_6);
    gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_7);
    gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_8);
    gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_10);
    gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_11);
    gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_12);
#if 0
    gpio_bit_write(GPIOD, LCD_CS_PIN, RESET);
    gpio_bit_write(GPIOD, LCD_A0_PIN, RESET);
    gpio_bit_write(GPIOD, LCD_ERD_PIN, RESET);
    gpio_bit_write(GPIOD, LCD_PWR_PIN, RESET);
    gpio_bit_write(GPIOD, LCD_RSTB_PIN, RESET);

    while(1);
#endif
}

void clear_dram()
{
    int i,j;
    write_cmd(0x30);
    write_cmd(0x15);    // Column Address Setting
    write_data(0x00);    // SEG0 -> SEG255
    write_data(0xFF);
    write_cmd(0x75);    // Page Address Setting
    write_data(0x00);    // COM0 -> COM159
    write_data(0x9f);
    write_cmd(0x5c);
    for(i=0;i<20;i++)
    {
        for(j=0;j<256;j++)
        {
            write_data(WHITE_RGB_CODE);
        }
    }
}

void clear_area(uint8_t x_zone, uint8_t y_zone)
{
    int i,j;
    write_cmd(0x30);
    write_cmd(0x15);    // Column Address Setting
    write_data(0x0);
    write_data(x_zone - 1);
    write_cmd(0x75);    // Page Address Setting
    write_data(0x00);
    write_data(y_zone - 1);
    write_cmd(0x5c);
    for(i=0;i<y_zone / 8;i++)
    {
        for(j=0;j<x_zone;j++)
        {
            write_data(0x00);
        }
    }
}

void show_partial_zone(uint8_t y_zone_min, uint8_t y_zone_max)
{
    /* TODO check zone valid */
    write_cmd(0x30);
    write_cmd(0xa8);
    write_data(y_zone_min);
    write_data(y_zone_max);
}

/**
  * @brief 在指定的位置显示内容
  * @param int x: 
  * @param int y: 
  * @param unsigned char wb_code: 
  * retval .
  */
static void _st75256_display_at_pos(int x, int y, unsigned char wb_code)
{
    /* TODO check x and y valid
     * x must [0, 159]
     * y must mod 8 ---> y % 8 == 0
     * */
    write_cmd(0x30);
    write_cmd(0x15);    // Column Address Setting
    write_data(x);
    write_data(x);
    write_cmd(0x75);    // Page Address Setting
    write_data(y / 8);
    write_data(y / 8);
    write_cmd(0x5c);
    write_data(wb_code);
}

/**
  * @brief 在指定坐标打点
  * @param int x: 
  * @param int y: 
  * @param unsigned int rgb: 
  * retval None.
  */
void st7525_draw_point(int x, int y, unsigned int rgb)
{
    int x_index, y_index, y_index_mod;

    x_index = x;
    y_index = y / 8;
    y_index_mod = y % 8;
    if (!!rgb)
    {
        if (0 == (gs_display_buffer[x_index][y_index] & 1 << y_index_mod))
        {
            gs_display_buffer[x_index][y_index] |= 1 << y_index_mod;
            _st75256_display_at_pos(x_index, y_index * 8, gs_display_buffer[x_index][y_index]);
        }
    }
    else
    {
        if (0 != (gs_display_buffer[x_index][y_index] & 1 << y_index_mod))
        {
            gs_display_buffer[x_index][y_index] &= ~(1 << y_index_mod);
            _st75256_display_at_pos(x_index, y_index * 8, gs_display_buffer[x_index][y_index]);
        }
    }

}
#if 0
/**
  * @brief 填充在指定区域
  * @param int x0: 
  * @param int y0: 
  * @param int x1: 
  * @param int y2: 
  * @param unsigned int rgb: 
  * retval .
  */
void st7525_fill_rect(int x0, int y0, int x1, int y1, unsigned int rgb)
{
    int i,j;
    /* TODO check x0 < x1 && y0 < y1 */
    write_cmd(0x30);
    write_cmd(0x15);    // Column Address Setting
    write_data(x0 & 0xff);
    write_data(x1 & 0xff);
    write_cmd(0x75);    // Page Address Setting
    write_data((y0 / 8 & 0xff));
    write_data((y1 / 8 & 0xff));
    /* write data to display ram */
    if (y0 % 8)
    {

    }
    if (y1 % 8)
    {

    }
    write_cmd(0x5c);
    for(i = 0; i < 1 + (y1 - y0) / 8; i++)
    {
        for(j = 0; j < x1 - x0 + 1; j++)
        {
            write_data(rgb ? BLACK_RGB_CODE : WHITE_RGB_CODE);
        }
    }
#if 0
    write_cmd(0xa8);
    write_data(y0);
    write_data(y1);
    write_cmd(0xa9);
#endif
}
#endif

void _st75256_chip_init(void)
{
    gpio_bit_write(GPIOD, LCD_RSTB_PIN, SET);
    rt_thread_mdelay(20);
    gpio_bit_write(GPIOD, LCD_RSTB_PIN, RESET);
    rt_thread_mdelay(20);
    gpio_bit_write(GPIOD, LCD_RSTB_PIN, SET);
    rt_thread_mdelay(200);
            
    write_cmd(0x30);   //Extension Command1
    write_cmd(0x6e);   //Enable Master
        
    write_cmd(0x31);   //Extension Command2
    write_cmd(0xd7);   //Disable Auto Read
    write_data(0x9f);  //Extension Command
    write_cmd(0xe0);   //Enable OTP Read
    write_data(0x00);  //Extension Command
    rt_thread_mdelay(20);
    write_cmd(0xe3);   //OTP Up-Load
    rt_thread_mdelay(20);
    write_cmd(0xe1);   //OTP Control Out
        
    write_cmd(0x30);   //Extension Command1
    write_cmd(0x94);   //Sleep Out
    write_cmd(0xae);   //Display off
    rt_thread_mdelay(50);
    
    write_cmd(0x20);   //Power Control
    write_data(0x0b);  //VB,VR,VF AllON

    write_cmd(0x0C);       // D0=0 LSB on top

    write_cmd(0x81);   //Set VOP
    write_data(0x21);  //0x03/0x3f=13.8V 0x2b=13.0 ;0x1d/0x04=15.0V
    //write_data(0x2f);  //0x03/0x3f=13.8V 0x2b=13.0 ;0x1d/0x04=15.0V
    write_data(0x04);  //    
    
    write_cmd(0x31);       // Extension Command 2    
    write_cmd(0x32);   //Analog Circuit Set    
    write_data(0x00);  //
    write_data(0x01);  //Booster Efficiency-6KHz
    write_data(0x02);  //0x04=Bias=1/10    0x02=1/12
    
    write_cmd(0x51);   //Booster Level     
    write_data(0xfb);  //*10    
    
    write_cmd(0x30);   //Extension Command1    
    
    write_cmd(0xf0);   //Display Mode
    write_data(0x10);  //Mono Mode
    
    write_cmd(0xca);   //Display Control
    write_data(0x00);  //CL Dividing Ratio---Not Divide
    write_data(0x9f);  //Duty Set---- 1/160    
    write_data(0x00);  //Frame Inversion
    
    write_cmd(0x31);       // Extension Command 2
    
    write_cmd(0xf2);   //Temperature Range
    write_data(0x1E);  // TA=-10
    write_data(0x28);  // TB=0
    write_data(0x32);  // TC=10
    
    write_cmd(0xf0);   //Frame rate
    write_data(0x16);  //
    write_data(0x16);  //
    write_data(0x16);  // 0X18
    write_data(0x16);  //102Hz
    
    write_cmd(0x30);   //Extension Command1    
    write_cmd(0xbc);   //Data Scan Direction
    write_data(0x02);  //Address direction
    
    write_cmd(0xa6);  //Normal display
    
    write_cmd(0x31);   //Extension Command2
    write_cmd(0x40);   //Internal Power Supply
    
    clear_dram();
    write_cmd(0xaf);  //Display ON
}

static int rt_hw_st75256_init(void)
{
    _st75256_pin_init();
    gpio_bit_write(GPIOD, LCD_CS_PIN, RESET);

    _st75256_chip_init();
    write_cmd(0xa9);
    LOG_I("Hello screen");

    return 0;
}
INIT_DEVICE_EXPORT(rt_hw_st75256_init);
