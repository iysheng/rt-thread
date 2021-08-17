/******************************************************************************
* File:             ccd_base.h
*
* Author:           iysheng@163.com  
* Created:          08/16/21 
*                   CCD_MAIN 项目基本头文件
*****************************************************************************/

#ifndef __CCD_MAIN_BASE_H__
#define __CCD_MAIN_BASE_H__

typedef enum {
    DUANLUO_LEFT_INDEX,
    DUANLUO_MIDDLE_INDEX,
    DUANLUO_RIGHT_INDEX,
    DUANLUO_NULL_INDEX,
    DUANLUO_MAX_INDEX,
} duanluo_index_E;

typedef struct {
    union {
        struct {
            uint16_t left;
            uint16_t middle;
            uint16_t right;
            uint16_t pad[1];
        } ccd_duanluo_pos_value;
        uint16_t ccd_duanluo_value[DUANLUO_MAX_INDEX];
    } duanluo_cfg;
} ccd_main_config_t;

typedef struct {
    ccd_main_config_t ccd_main_config;
} ccd_main_system_t;

extern int set_ccd_main_config(ccd_main_config_t *ccd_main_config);
#endif
