/******************************************************************************
* File:             ccd_base.h
*
* Author:           iysheng@163.com  
* Created:          08/16/21 
*                   CCD_MAIN 项目基本头文件
*****************************************************************************/

#ifndef __CCD_MAIN_BASE_H__
#define __CCD_MAIN_BASE_H__

typedef struct {
    uint16_t left;
    uint16_t middle;
    uint16_t right;
    uint16_t pad[1];
} ccd_main_config_t;

typedef struct {
    ccd_main_config_t ccd_main_config;
} ccd_main_system_t;

#endif
