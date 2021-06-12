/******************************************************************************
* File:             can_comm.h
*
* Author:           iysheng@163.com  
* Created:          06/13/21 
*                   CAN 通讯的头文件
*****************************************************************************/

#ifndef __CAN_COMM_H__
#define __CAN_COMM_H__

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
#endif /* ifndef __CAN_COMM_H__ */
