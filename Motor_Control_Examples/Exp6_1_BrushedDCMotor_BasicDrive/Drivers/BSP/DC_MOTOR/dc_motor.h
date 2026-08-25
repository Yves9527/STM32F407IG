/**
 ****************************************************************************************************
 * @file        dc_motor.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       直流有刷电机控制代码
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

#ifndef __DCMOTOR_H
#define __DCMOTOR_H

#include "./SYSTEM/sys/sys.h"

/*************************************    基本驱动    *****************************************************/

/* 停止引脚操作宏定义 
 * 此引脚控制H桥是否生效以达到开启和关闭电机的效果
 */
#define SHUTDOWN1_Pin                 GPIO_PIN_10
#define SHUTDOWN1_GPIO_Port           GPIOF

#define SHUTDOWN2_Pin                 GPIO_PIN_2
#define SHUTDOWN2_GPIO_Port           GPIOF
#define SHUTDOWN_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)   /* PF口时钟使能 */

/* 电机停止引脚定义 这里默认是接口1 */
#define ENABLE_MOTOR    HAL_GPIO_WritePin(SHUTDOWN1_GPIO_Port,SHUTDOWN1_Pin,GPIO_PIN_SET)
#define DISABLE_MOTOR   HAL_GPIO_WritePin(SHUTDOWN1_GPIO_Port,SHUTDOWN1_Pin,GPIO_PIN_RESET)

/******************************************************************************************/

void dcmotor_init(void);                /* 直流有刷电机初始化 */
void dcmotor_start(void);               /* 开启电机 */
void dcmotor_stop(void);                /* 关闭电机 */  
void dcmotor_dir(uint8_t para);         /* 设置电机方向 */
void dcmotor_speed(uint16_t para);      /* 设置电机速度 */
void motor_pwm_set(float para);         /* 电机控制 */
    
#endif


















