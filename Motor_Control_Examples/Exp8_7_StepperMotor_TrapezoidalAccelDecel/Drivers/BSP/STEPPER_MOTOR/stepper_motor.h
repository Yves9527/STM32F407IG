/**
 ****************************************************************************************************
 * @file        stepper_motor.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       步进电机 驱动代码
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
#ifndef __STEPPER_MOTOR_H
#define __STEPPER_MOTOR_H

#include "./SYSTEM/sys/sys.h"
/******************************************************************************************/

#define TIM_FREQ            168000000U                      /* 定时器主频 */
#define MAX_STEP_ANGLE      0.225                           /* 最小步距(1.8/MICRO_STEP) */
#define PAI                 3.1415926                       /* 圆周率*/
#define FSPR                200                             /* 步进电机单圈步数 */
#define MICRO_STEP          8                               /* 步进电机驱动器细分数 */
#define T1_FREQ             (TIM_FREQ/84)                   /* 频率ft值 */
#define SPR                 (FSPR*MICRO_STEP)               /* 旋转一圈需要的脉冲数 */

/* 数学常数 */

#define ALPHA               ((float)(2*PAI/SPR))            /* α = 2*pi/spr */
#define A_T_x10             ((float)(10*ALPHA*T1_FREQ))
#define T1_FREQ_148         ((float)((T1_FREQ*0.69)/10))    /* 0.69为误差修正值 */
#define A_SQ                ((float)(2*100000*ALPHA))
#define A_x200              ((float)(200*ALPHA))            /* 2*10*10*a/10 */

typedef struct
{
    __IO uint8_t  run_state;                                /* 电机旋转状态 */
    __IO uint8_t  dir;                                      /* 电机旋转方向 */
    __IO int32_t  step_delay;                               /* 下个脉冲周期（时间间隔），启动时为加速度 */
    __IO uint32_t decel_start;                              /* 开始减速位置 */
    __IO int32_t  decel_val;                                /* 减速阶段步数 */
    __IO int32_t  min_delay;                                /* 速度最快，计数值最小的值(最大速度，即匀速段速度) */
    __IO int32_t  accel_count;                              /* 加减速阶段计数值 */
} speedRampData;

enum STA
{
    STOP = 0,                                               /* 加减速曲线状态：停止*/
    ACCEL,                                                  /* 加减速曲线状态：加速阶段*/
    DECEL,                                                  /* 加减速曲线状态：减速阶段*/
    RUN                                                     /* 加减速曲线状态：匀速阶段*/
};

enum DIR
{
 CCW = 0,                                                   /* 逆时针 */ 
 CW                                                         /* 顺时针 */
};

enum EN
{
 EN_ON = 0,                                                 /* 失能脱机引脚 */
 EN_OFF                                                     /* 使能脱机引脚 使能后电机停止旋转 */
};

/******************************************************************************************/
/* 步进电机引脚定义*/

#define STEPPER_MOTOR_1       1                             /* 步进电机接口序号 */
#define STEPPER_MOTOR_2       2
#define STEPPER_MOTOR_3       3
#define STEPPER_MOTOR_4       4
/* 步进电机方向引脚定义 */

#define STEPPER_DIR1_GPIO_PIN                  GPIO_PIN_14
#define STEPPER_DIR1_GPIO_PORT                 GPIOF
#define STEPPER_DIR1_GPIO_CLK_ENABLE()         do{  __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)    /* PF口时钟使能 */

#define STEPPER_DIR2_GPIO_PIN                  GPIO_PIN_12
#define STEPPER_DIR2_GPIO_PORT                 GPIOF
#define STEPPER_DIR2_GPIO_CLK_ENABLE()         do{  __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)    /* PF口时钟使能 */

#define STEPPER_DIR3_GPIO_PIN                  GPIO_PIN_2
#define STEPPER_DIR3_GPIO_PORT                 GPIOB
#define STEPPER_DIR3_GPIO_CLK_ENABLE()         do{  __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)    /* PB口时钟使能 */

#define STEPPER_DIR4_GPIO_PIN                  GPIO_PIN_2
#define STEPPER_DIR4_GPIO_PORT                 GPIOH
#define STEPPER_DIR4_GPIO_CLK_ENABLE()         do{  __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)    /* PH口时钟使能 */

/* 步进电机脱机引脚定义 */

#define STEPPER_EN1_GPIO_PIN                   GPIO_PIN_15
#define STEPPER_EN1_GPIO_PORT                  GPIOF
#define STEPPER_EN1_GPIO_CLK_ENABLE()          do{  __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)    /* PF口时钟使能 */

#define STEPPER_EN2_GPIO_PIN                   GPIO_PIN_13
#define STEPPER_EN2_GPIO_PORT                  GPIOF
#define STEPPER_EN2_GPIO_CLK_ENABLE()          do{  __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)    /* PF口时钟使能 */

#define STEPPER_EN3_GPIO_PIN                   GPIO_PIN_11
#define STEPPER_EN3_GPIO_PORT                  GPIOF
#define STEPPER_EN3_GPIO_CLK_ENABLE()          do{  __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)    /* PF口时钟使能 */
 
#define STEPPER_EN4_GPIO_PIN                   GPIO_PIN_3
#define STEPPER_EN4_GPIO_PORT                  GPIOH
#define STEPPER_EN4_GPIO_CLK_ENABLE()          do{  __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)    /* PH口时钟使能 */

/*----------------------- 方向引脚控制 -----------------------------------*/
/* 由于我们使用的是共阳极解法，并且硬件对电平做了取反操作，所以当 x = 1 有效，x = 0时无效*/  
#define ST1_DIR(x)    do{ x ? \
                              HAL_GPIO_WritePin(STEPPER_DIR1_GPIO_PORT, STEPPER_DIR1_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(STEPPER_DIR1_GPIO_PORT, STEPPER_DIR1_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)  

#define ST2_DIR(x)    do{ x ? \
                              HAL_GPIO_WritePin(STEPPER_DIR2_GPIO_PORT, STEPPER_DIR2_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(STEPPER_DIR2_GPIO_PORT, STEPPER_DIR2_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)  

#define ST3_DIR(x)    do{ x ? \
                              HAL_GPIO_WritePin(STEPPER_DIR3_GPIO_PORT, STEPPER_DIR3_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(STEPPER_DIR3_GPIO_PORT, STEPPER_DIR3_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)  

#define ST4_DIR(x)    do{ x ? \
                              HAL_GPIO_WritePin(STEPPER_DIR4_GPIO_PORT, STEPPER_DIR4_GPIO_PIN, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(STEPPER_DIR4_GPIO_PORT, STEPPER_DIR4_GPIO_PIN, GPIO_PIN_RESET); \
                          }while(0)  

/*----------------------- 脱机引脚控制 -----------------------------------*/
/* 由于我们使用的是共阳极解法，并且硬件对电平做了取反操作，所以当 x = 1 有效，x = 0时无效*/                          
#define ST1_EN(x)      do{ x ? \
                          HAL_GPIO_WritePin(STEPPER_EN1_GPIO_PORT, STEPPER_EN1_GPIO_PIN, GPIO_PIN_SET) : \
                          HAL_GPIO_WritePin(STEPPER_EN1_GPIO_PORT, STEPPER_EN1_GPIO_PIN, GPIO_PIN_RESET); \
                        }while(0)    

#define ST2_EN(x)      do{ x ? \
                          HAL_GPIO_WritePin(STEPPER_EN2_GPIO_PORT, STEPPER_EN2_GPIO_PIN, GPIO_PIN_SET) : \
                          HAL_GPIO_WritePin(STEPPER_EN2_GPIO_PORT, STEPPER_EN2_GPIO_PIN, GPIO_PIN_RESET); \
                        }while(0)    

#define ST3_EN(x)      do{ x ? \
                          HAL_GPIO_WritePin(STEPPER_EN3_GPIO_PORT, STEPPER_EN3_GPIO_PIN, GPIO_PIN_SET) : \
                          HAL_GPIO_WritePin(STEPPER_EN3_GPIO_PORT, STEPPER_EN3_GPIO_PIN, GPIO_PIN_RESET); \
                        }while(0)    

#define ST4_EN(x)      do{ x ? \
                          HAL_GPIO_WritePin(STEPPER_EN4_GPIO_PORT, STEPPER_EN4_GPIO_PIN, GPIO_PIN_SET) : \
                          HAL_GPIO_WritePin(STEPPER_EN4_GPIO_PORT, STEPPER_EN4_GPIO_PIN, GPIO_PIN_RESET); \
                        }while(0)                             
                        
/******************************************************************************************/
/* 外部接口函数*/
void stepper_init(uint16_t arr, uint16_t psc);                                          /* 步进电机接口初始化 */
void stepper_star(uint8_t motor_num);                                                   /* 开启步进电机 */
void stepper_stop(uint8_t motor_num);                                                   /* 关闭步进电机 */   

void create_t_ctrl_param(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed); /* 梯形加减速控制函数 */                        
#endif
