/**
 ****************************************************************************************************
 * @file        pid.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       PID代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 F407电机开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com/forum.php
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211014
 * 第一次发布
 *
 ****************************************************************************************************
 */
#ifndef __PID_H
#define __PID_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* PID相关参数 */

#define  INCR_LOCT_SELECT  1         /* 0：位置式，1：增量式 */

#if INCR_LOCT_SELECT

/* 注意：双环控制的时候，外环PID参数调节幅度不要太大，这对于整个曲线的影响很大 */

/* 定义位置环（外环）PID参数相关宏 */
#define  L_KP      0.18f             /* P参数 */
#define  L_KI      0.00f             /* I参数 */
#define  L_KD      0.08f             /* D参数 */

/* 定义速度环（内环）PID参数相关宏 */
#define  S_KP      20.0f             /* P参数 */
#define  S_KI      10.00f            /* I参数 */
#define  S_KD      0.02f             /* D参数 */
#define  SMAPLSE_PID_SPEED  50       /* 采样周期 单位ms */

#else

/* 定义位置环PID参数相关宏 */
#define  L_KP      0.18f             /* P参数 */
#define  L_KI      0.00f             /* I参数 */
#define  L_KD      0.08f             /* D参数 */

/* 定义速度PID参数相关宏 */
#define  S_KP      20.0f             /* P参数 */
#define  S_KI      10.00f            /* I参数 */
#define  S_KD      0.02f             /* D参数 */
#define  SMAPLSE_PID_SPEED  50       /* 采样周期 单位ms */

#endif

/* PID结构体 */
typedef struct
{
    __IO float  SetPoint;            /* 目标值 */
    __IO float  ActualValue;         /* 期望输出值 */
    __IO float  SumError;            /* 误差累计 */
    __IO float  Proportion;          /* 比例常数 P */
    __IO float  Integral;            /* 积分常数 I */
    __IO float  Derivative;          /* 微分常数 D */
    __IO float  Error;               /* Error[1] */
    __IO float  LastError;           /* Error[-1] */
    __IO float  PrevError;           /* Error[-2] */
} PID_TypeDef;

extern PID_TypeDef  g_location_pid;  /* 位置环PID参数结构体 */
extern PID_TypeDef  g_speed_pid;     /* 速度环PID参数结构体 */

/******************************************************************************************/

void pid_init(void);                 /* pid初始化 */
int32_t increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value);      /* pid闭环控制 */

#endif
