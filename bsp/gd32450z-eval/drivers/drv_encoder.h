/******************************************************************************
* File:             drv_encoder.h
*
* Author:           iysheng@163.com  
* Created:          08/19/21 
*                   编码器驱动头文件
*****************************************************************************/
#ifndef __ENCODER_H__
#define __ENCODER_H__ 0

typedef struct {
    struct rt_semaphore sem4sync;
    int sync_counts;
} encoder_sync_t;

#define ENCODER_START_CHECK    20
#define ENCODER_END_CHECK      200

#endif
