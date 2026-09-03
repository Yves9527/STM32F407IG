/**
 ****************************************************************************************************
 * @file        steering_engine.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       舵机 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 F407电机开发板
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
#include "./BSP/STEERING_ENGINE/steering_engine.h"
#include "./BSP/TIMER/atim.h"

/***************************************************************************************************/
extern TIM_HandleTypeDef g_atimx_pwm_chy_handle;      /* 定时器x句柄 */

/**
 * @brief       将舵机目标角度转换为定时器比较值 (CCR)
 * @param       angle: 目标角度 (0.0° ~ 180.0°)
 * @retval      定时器比较值 (500 ~ 2500 us)，若角度越界则返回 0
 */
uint16_t angle_to_tim_val(float angle)
{
    /* 1. 角度越界校验（卫语句提前退出） */
    if ((angle < SERVO_MIN_ANGLE) || (angle > SERVO_MAX_ANGLE))
    {
        return 0;
    }

    /* 2. 线性插值：CCR = 基础脉宽(0°) + (角度 / 180.0°) * 脉宽有效跨度(2000us)
     *    +0.5f 实现四舍五入转换
     */
    float pulse_us = SERVO_MIN_PULSE_US + 
                     (angle / SERVO_MAX_ANGLE) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);

    return (uint16_t)(pulse_us + 0.5f);
}

/**
 * @brief       舵机角度设定
 * @param       id:舵机编号对应舵机的接口：1~3
 * @param       angle:角度 
 * @retval      0:成功
 */
uint8_t servo_angle_set(uint8_t id,float angle)
{
    uint16_t val;
    switch(id)
    {
        case 1:
            val = angle_to_tim_val(angle);                                          /* 得到角度转换的比较值 */
            if(val != 0)
            {
                __HAL_TIM_SetCompare(&g_atimx_pwm_chy_handle,TIM_CHANNEL_1,val);    /* 设置比较值 */
            }
            break;
        case 2:
            val = angle_to_tim_val(angle);
            if(val != 0)
            {
                __HAL_TIM_SetCompare(&g_atimx_pwm_chy_handle,TIM_CHANNEL_2,val);
            }
            break;
        case 3:
            val = angle_to_tim_val(angle);
            if(val != 0)
            {
                __HAL_TIM_SetCompare(&g_atimx_pwm_chy_handle,TIM_CHANNEL_3,val);
            }
            break;
        default:
            break;
    }
    return 0;
}
