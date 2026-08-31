/**
 ****************************************************************************************************
 * @file        pid.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       PID算法代码
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

#include "./BSP/PID/pid.h"
#include "./BSP/DC_MOTOR/dc_motor.h"

PID_TypeDef  g_location_pid;             /* 位置环PID参数结构体 */
PID_TypeDef  g_current_pid;              /* 电流环PID参数结构体 */

/**
 * @brief       pid初始化
 * @param       无
 * @retval      无
 */
void pid_init(void)
{
    /* 初始化位置环PID参数 */
    g_location_pid.SetPoint = 0.0;       /* 目标值 */
    g_location_pid.ActualValue = 0.0;    /* 期望输出值 */
    g_location_pid.SumError = 0.0;       /* 积分值 */
    g_location_pid.Error = 0.0;          /* Error[1] */
    g_location_pid.LastError = 0.0;      /* Error[-1] */
    g_location_pid.PrevError = 0.0;      /* Error[-2] */
    g_location_pid.Proportion = L_KP;    /* 比例常数 Proportional Const */
    g_location_pid.Integral = L_KI;      /* 积分常数 Integral Const */
    g_location_pid.Derivative = L_KD;    /* 微分常数 Derivative Const */ 
    
    /* 初始化电流环PID参数 */
    g_current_pid.SetPoint = 0.0;        /* 目标值 */
    g_current_pid.ActualValue = 0.0;     /* 期望输出值 */
    g_current_pid.SumError = 0.0;        /* 积分值*/
    g_current_pid.Error = 0.0;           /* Error[1]*/
    g_current_pid.LastError = 0.0;       /* Error[-1]*/
    g_current_pid.PrevError = 0.0;       /* Error[-2]*/
    g_current_pid.Proportion = C_KP;     /* 比例常数 Proportional Const */
    g_current_pid.Integral = C_KI;       /* 积分常数 Integral Const */
    g_current_pid.Derivative = C_KD;     /* 微分常数 Derivative Const */
}

/**
 * @brief       pid闭环控制
 * @param       *PID：PID结构体变量地址
 * @param       Feedback_value：当前实际值
 * @retval      期望输出值
 */
int32_t increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value)
{
    PID->Error = (float)(PID->SetPoint - Feedback_value);                   /* 计算偏差 */
    
#if  INCR_LOCT_SELECT                                                       /* 增量式PID */
    
    PID->ActualValue += (PID->Proportion * (PID->Error - PID->LastError))                          /* 比例环节 */
                        + (PID->Integral * PID->Error)                                             /* 积分环节 */
                        + (PID->Derivative * (PID->Error - 2 * PID->LastError + PID->PrevError));  /* 微分环节 */
    
    PID->PrevError = PID->LastError;                                        /* 存储偏差，用于下次计算 */
    PID->LastError = PID->Error;
    
#else                                                                       /* 位置式PID */
    
    PID->SumError += PID->Error;
    PID->ActualValue = (PID->Proportion * PID->Error)                       /* 比例环节 */
                       + (PID->Integral * PID->SumError)                    /* 积分环节 */
                       + (PID->Derivative * (PID->Error - PID->LastError)); /* 微分环节 */
    PID->LastError = PID->Error;
    
#endif
    return ((int32_t)(PID->ActualValue));                                   /* 返回计算后输出的数值 */
}

