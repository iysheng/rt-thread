/******************************************************************************
* File:             can_comm.h
*
* Author:           iysheng@163.com  
* Created:          06/13/21 
*                   CAN 通讯的头文件
*****************************************************************************/

#ifndef __CAN_COMM_H__
#define __CAN_COMM_H__

#include "ccd_main_base.h"

/**
  * @brief 控制 CCD 进行标定
  * 
  * @param unsigned char addr: 
  * @param unsigned char times: 
  * retval errno/Linux.
  */
int set_ccd_calibrate(unsigned char addr, unsigned char times);

/**
  * @brief 控制 CCD 进行检测并返回检测结果
  * 
  * @param unsigned char addr: 
  * @param unsigned int id: 
  * retval errno/Linux.
  */
int set_ccd_check(unsigned char addr, unsigned int id);

/**
  * @brief 控制 CCD 段落配置
  *
  * @param unsigned char addr:
  * param unsigned int id:
  * retval errno/Linux.
  *     0 表示匹配
  *     1 表示不匹配
  */
int set_ccd_duanluo(unsigned char addr, ccd_main_config_t *config);
#endif /* ifndef __CAN_COMM_H__ */
