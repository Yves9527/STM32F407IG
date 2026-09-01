/**
 ****************************************************************************************************
 * @file        dcmotor_tim.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-19
 * @brief       定时器 驱动代码
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
 * V1.0 20211019
 * 第一次发布
 *
 ****************************************************************************************************
 */

#ifndef __DCMOTOR_TIM_H
#define __DCMOTOR_TIM_H

#include "./SYSTEM/sys/sys.h"

/******************************* 第一部分  电机基本驱动 互补输出带死区控制 **************************************/

/* TIMX 互补输出模式定义 */

/* 输出通道引脚 */
#define ATIM_TIMX_CPLM_CHY_GPIO_PORT            GPIOA
#define ATIM_TIMX_CPLM_CHY_GPIO_PIN             GPIO_PIN_8
#define ATIM_TIMX_CPLM_CHY_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PA口时钟使能 */

/* 互补输出通道引脚 */
#define ATIM_TIMX_CPLM_CHYN_GPIO_PORT           GPIOB
#define ATIM_TIMX_CPLM_CHYN_GPIO_PIN            GPIO_PIN_13
#define ATIM_TIMX_CPLM_CHYN_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PB口时钟使能 */

/* TIMX 引脚复用设置
 * 因为PA8/PB13, 默认并不是TIM1的功能脚, 必须开启复用, 才可以: TIM1_CH1->PA8; TIM1_CH1N->PB13;
 * 这样,PA8/PB13才能用作TIM1的CH1/CH1N功能.
 */
#define ATIM_TIMX_CPLM_CHY_GPIO_AF              GPIO_AF1_TIM1

/* 互补输出使用的定时器 */
#define ATIM_TIMX_CPLM                          TIM1
#define ATIM_TIMX_CPLM_CHY                      TIM_CHANNEL_1
#define ATIM_TIMX_CPLM_CHY_CCRY                 ATIM_TIMX_CPLM->CCR1
#define ATIM_TIMX_CPLM_CLK_ENABLE()             do{ __HAL_RCC_TIM1_CLK_ENABLE(); }while(0)    /* TIM1 时钟使能 */


/******************************* 第二部分  电机编码器测速 **************************************/

/* 通用定时器 定义 
 * 
 */
#define GTIM_TIMX_ENCODER_CH1_GPIO_PORT         GPIOC
#define GTIM_TIMX_ENCODER_CH1_GPIO_PIN          GPIO_PIN_6
#define GTIM_TIMX_ENCODER_CH1_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)  /* PC口时钟使能 */

#define GTIM_TIMX_ENCODER_CH2_GPIO_PORT         GPIOC
#define GTIM_TIMX_ENCODER_CH2_GPIO_PIN          GPIO_PIN_7
#define GTIM_TIMX_ENCODER_CH2_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)  /* PC口时钟使能 */

/* TIMX 引脚复用设置
 * 因为PC6/PC7, 默认并不是TIM3的功能脚, 必须开启复用, 才可以: TIM3_CH1->PC6; TIM3_CH2->PC7;
 * 这样,PC6/PC7才能用作TIM3的CH1/CH2功能.
 */
#define GTIM_TIMX_ENCODERCH1_GPIO_AF            GPIO_AF2_TIM3                                /* 端口复用到TIM3 */
#define GTIM_TIMX_ENCODERCH2_GPIO_AF            GPIO_AF2_TIM3                                /* 端口复用到TIM3 */

#define GTIM_TIMX_ENCODER                       TIM3                                         /* TIM3 */
#define GTIM_TIMX_ENCODER_INT_IRQn              TIM3_IRQn
#define GTIM_TIMX_ENCODER_INT_IRQHandler        TIM3_IRQHandler

#define GTIM_TIMX_ENCODER_CH1                   TIM_CHANNEL_1                                /* 通道1 */
#define GTIM_TIMX_ENCODER_CH1_CLK_ENABLE()      do{ __HAL_RCC_TIM3_CLK_ENABLE(); }while(0)   /* TIM3 时钟使能 */

#define GTIM_TIMX_ENCODER_CH2                   TIM_CHANNEL_2                                /* 通道2 */
#define GTIM_TIMX_ENCODER_CH2_CLK_ENABLE()      do{ __HAL_RCC_TIM3_CLK_ENABLE(); }while(0)   /* TIM3 时钟使能 */


/* 基本定时器 定义 
 * 捕获编码器值，用于计算速度
 */
#define BTIM_TIMX_INT                           TIM6
#define BTIM_TIMX_INT_IRQn                      TIM6_DAC_IRQn
#define BTIM_TIMX_INT_IRQHandler                TIM6_DAC_IRQHandler
#define BTIM_TIMX_INT_CLK_ENABLE()              do{ __HAL_RCC_TIM6_CLK_ENABLE(); }while(0)    /* TIM6 时钟使能 */

/* 编码器参数结构体 */
typedef struct 
{
  int encode_old;                  /* 上一次计数值 */
  int encode_now;                  /* 当前计数值 */
  float speed;                     /* 编码器速度 */
} ENCODE_TypeDef;

extern ENCODE_TypeDef g_encode;    /* 编码器参数变量 */

/******************************************************************************************/

void atim_timx_cplm_pwm_init(uint16_t arr, uint16_t psc);           /* 高级定时器 互补输出 初始化函数 */
void gtim_timx_encoder_chy_init(uint16_t arr, uint16_t psc);        /* 通用定时器 定时中断初始化函数 */
void btim_timx_int_init(uint16_t arr, uint16_t psc);                /* 基本定时器 定时中断初始化函数 */
int gtim_get_encode(void);

#endif




