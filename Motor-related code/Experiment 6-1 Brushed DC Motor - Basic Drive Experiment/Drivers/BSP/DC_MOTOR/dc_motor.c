/**
 ****************************************************************************************************
 * @file        dc_motor.c
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

#include "./BSP/DC_MOTOR/dc_motor.h"
#include "./SYSTEM/delay/delay.h"


/*************************************    基本驱动    *****************************************************/

extern TIM_HandleTypeDef g_atimx_cplm_pwm_handle;                              /* 定时器x句柄 */

/**
 * @brief       电机初始化
 * @param       无
 * @retval      无
 */
void dcmotor_init(void)
{
    SHUTDOWN_GPIO_CLK_ENABLE();
    GPIO_InitTypeDef gpio_init_struct;
    
    /* SD引脚设置，设置为推挽输出 */
    gpio_init_struct.Pin = SHUTDOWN1_Pin|SHUTDOWN2_Pin;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SHUTDOWN1_GPIO_Port, &gpio_init_struct);
    
    HAL_GPIO_WritePin(GPIOF, SHUTDOWN1_Pin|SHUTDOWN2_Pin, GPIO_PIN_RESET);      /* SD拉低，关闭输出 */
    
    dcmotor_stop();                 /* 停止电机 */
    dcmotor_dir(0);                 /* 设置正转 */
    dcmotor_speed(0);               /* 速度设置为0 */
    dcmotor_start();                /* 开启电机 */
}

/**
 * @brief       电机开启
 * @param       无
 * @retval      无
 */
void dcmotor_start(void)
{
    ENABLE_MOTOR;                                                       /* 拉高SD引脚，开启电机 */
}

/**
 * @brief       电机停止
 * @param       无
 * @retval      无
 */
void dcmotor_stop(void)
{
    HAL_TIM_PWM_Stop(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1);          /* 关闭主通道输出 */
    HAL_TIMEx_PWMN_Stop(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1);       /* 关闭互补通道输出 */
    DISABLE_MOTOR;                                                      /* 拉低SD引脚，停止电机 */
}

/**
 * @brief       电机旋转方向设置
 * @param       para:方向 0正转，1反转
 * @note        以电机正面，顺时针方向旋转为正转
 * @retval      无
 */
void dcmotor_dir(uint8_t para)
{
    HAL_TIM_PWM_Stop(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1);          /* 关闭主通道输出 */
    HAL_TIMEx_PWMN_Stop(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1);       /* 关闭互补通道输出 */

    if (para == 0)                /* 正转 */
    {
        HAL_TIM_PWM_Start(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1);     /* 开启主通道输出 */
    } 
    else if (para == 1)           /* 反转 */
    {
        HAL_TIMEx_PWMN_Start(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1);  /* 开启互补通道输出 */
    }
}

/**
 * @brief       电机速度设置
 * @param       para:比较寄存器值
 * @retval      无
 */
void dcmotor_speed(uint16_t para)
{
    if (para < (__HAL_TIM_GetAutoreload(&g_atimx_cplm_pwm_handle) - 0x0F))  /* 限速 */
    {  
        __HAL_TIM_SetCompare(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1, para);
    }
}

/**
 * @brief       电机控制
 * @param       para: pwm比较值 ,正数电机为正转，负数为反转
 * @note        根据传入的参数控制电机的转向和速度
 * @retval      无
 */
void motor_pwm_set(float para)
{
    int val = (int)para;

    if (val >= 0) 
    {
        dcmotor_dir(0);           /* 正转 */
        dcmotor_speed(val);
    } 
    else 
    {
        dcmotor_dir(1);           /* 反转 */
        dcmotor_speed(-val);
    }
}



