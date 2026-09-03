/**
 ****************************************************************************************************
 * @file        steering_engine.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       舵机 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F407电机开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211014
 * 第一次发布
 *
 ****************************************************************************************************
 */
#ifndef __STEERING_ENGINE_H
#define __STEERING_ENGINE_H

#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* 舵机参数宏定义 */
#define SERVO_MIN_ANGLE         0.0f        /* 舵机最小角度 0° */
#define SERVO_MAX_ANGLE         180.0f      /* 舵机最大角度 180° */
#define SERVO_MIN_PULSE_US      500         /* 0° 对应的比较值/脉宽 (0.5ms = 500us) */
#define SERVO_MAX_PULSE_US      2500        /* 180° 对应的比较值/脉宽 (2.5ms = 2500us) */

/******************************************************************************************/
/* 外部接口函数 */
uint16_t angle_to_tim_val(float angle);                     /* 角度转比较值 */
uint8_t servo_angle_set(uint8_t id, float angle);           /* 设置角度 */
#endif
