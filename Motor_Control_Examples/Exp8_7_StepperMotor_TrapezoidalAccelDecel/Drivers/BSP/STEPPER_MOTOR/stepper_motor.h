/**
 ****************************************************************************************************
 * @file        stepper_motor.h
 * @author      正点原子团队(ALIENTEK) / 优化重构版
 * @version     V2.0
 * @date        2024-05-10
 * @brief       步进电机 梯形加减速控制驱动 (基于 AVR446 / David Austin 算法)
 * @encoding    GBK / GB2312
 ****************************************************************************************************
 */

#ifndef __STEPPER_MOTOR_H
#define __STEPPER_MOTOR_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* 基础物理与定时器参数配置 */

#define TIM_CLK_FREQ        168000000U              /* 主定时器时钟源频率: 168MHz */
#define TIM_PSC_DIV         84U                     /* 预分频系数: 84分频 */
#define TIM_CNT_FREQ        (TIM_CLK_FREQ / TIM_PSC_DIV) /* 计数器频率 ft = 2MHz (1个tick = 0.5us) */

#define FSPR                200                     /* 步进电机基本整步步数 (1.8度步距角: 360/1.8 = 200) */
#define MICRO_STEP          8                       /* 驱动器细分倍数 (当前为8细分) */
#define SPR                 (FSPR * MICRO_STEP)     /* 电机旋转一整圈所需的总脉冲数: 1600 */
#define PI                  3.14159265358979f

/* 每走一步电机转过的物理弧度 alpha = 2*PI / SPR */
#define STEP_RADIAN         ((float)(2.0f * PI / SPR))

/******************************************************************************************/
/* AVR446 算法推导辅助参数
 * 说明：函数入参 speed 单位为 0.1 rad/s，accel/decel 单位为 0.1 rad/s^2。
 * 为避免在中断与规划中进行重复的浮点开销，将常数提前合并。
 */

/* 1. 计算最高速度最小周期: min_delay = (alpha * ft) / omega_max = (10 * alpha * ft) / speed */
#define COEFF_MIN_DELAY     ((float)(10.0f * STEP_RADIAN * TIM_CNT_FREQ))

/* 2. 计算第一步启动初周期: c0 = 0.676 * ft * sqrt(2 * alpha / a)
 * 0.676 为 David Austin 算法针对第1步近似误差的经典数学补偿系数
 */
#define COEFF_C0_FREQ       ((float)(TIM_CNT_FREQ * 0.676f / 10.0f))
#define COEFF_C0_RADIAN     ((float)(2.0f * 100000.0f * STEP_RADIAN))

/* 3. 计算达到最高转速所需步数: n_max = omega^2 / (2 * alpha * a) */
#define COEFF_MAX_STEPS     ((float)(20.0f * STEP_RADIAN)) /* 2 * 0.1 * alpha 的放大系数 */

/******************************************************************************************/
/* 运动状态与加减速控制数据结构 */

typedef enum
{
    STOP = 0,   /* 停止状态 (未使能或运行结束) */
    ACCEL,      /* 加速阶段 */
    RUN,        /* 匀速巡航阶段 */
    DECEL       /* 减速阶段 */
} MotorRunState_t;

typedef enum
{
    CCW = 0,    /* 逆时针 */
    CW  = 1     /* 顺时针 */
} MotorDir_t;

typedef enum
{
    EN_ON  = 0, /* 使能电机驱动输出 */
    EN_OFF = 1  /* 脱机释放电机 (掉电可手动转动) */
} MotorEnable_t;

/* 加减速轨迹运行数据结构体 */
typedef struct
{
    __IO uint8_t  run_state;    /* 当前加减速状态 (STOP / ACCEL / RUN / DECEL) */
    __IO uint8_t  dir;          /* 旋转方向 (CW / CCW) */
    __IO int32_t  step_delay;   /* 当前脉冲定时器周期计数 (值越小，脉冲频率越高、转速越快) */
    __IO uint32_t decel_start;  /* 开始减速的步数位置 (从第几步开始刹车) */
    __IO int32_t  decel_val;    /* 减速段步数(负数，用于递推公式自然变号以减速) */
    __IO int32_t  min_delay;    /* 最高设定转速对应的最小定时器周期 */
    __IO int32_t  accel_count;  /* 当前加/减速步进计数 (加速时从0递增；减速时从负值向0逼近) */
} speedRampData;

/******************************************************************************************/
/* 硬件引脚定义 (保持与开发板接线完全一致) */

#define STEPPER_MOTOR_1       1
#define STEPPER_MOTOR_2       2
#define STEPPER_MOTOR_3       3
#define STEPPER_MOTOR_4       4

/* 方向引脚 (DIR) */
#define STEPPER_DIR1_GPIO_PIN                  GPIO_PIN_14
#define STEPPER_DIR1_GPIO_PORT                 GPIOF
#define STEPPER_DIR1_GPIO_CLK_ENABLE()         do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)

#define STEPPER_DIR2_GPIO_PIN                  GPIO_PIN_12
#define STEPPER_DIR2_GPIO_PORT                 GPIOF
#define STEPPER_DIR2_GPIO_CLK_ENABLE()         do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)

#define STEPPER_DIR3_GPIO_PIN                  GPIO_PIN_2
#define STEPPER_DIR3_GPIO_PORT                 GPIOB
#define STEPPER_DIR3_GPIO_CLK_ENABLE()         do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

#define STEPPER_DIR4_GPIO_PIN                  GPIO_PIN_2
#define STEPPER_DIR4_GPIO_PORT                 GPIOH
#define STEPPER_DIR4_GPIO_CLK_ENABLE()         do{ __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)

/* 使能引脚 (EN) */
#define STEPPER_EN1_GPIO_PIN                   GPIO_PIN_15
#define STEPPER_EN1_GPIO_PORT                  GPIOF
#define STEPPER_EN1_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)

#define STEPPER_EN2_GPIO_PIN                   GPIO_PIN_13
#define STEPPER_EN2_GPIO_PORT                  GPIOF
#define STEPPER_EN2_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)

#define STEPPER_EN3_GPIO_PIN                   GPIO_PIN_11
#define STEPPER_EN3_GPIO_PORT                  GPIOF
#define STEPPER_EN3_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)

#define STEPPER_EN4_GPIO_PIN                   GPIO_PIN_3
#define STEPPER_EN4_GPIO_PORT                  GPIOH
#define STEPPER_EN4_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)

/* 方向控制宏 */
#define ST1_DIR(x)    HAL_GPIO_WritePin(STEPPER_DIR1_GPIO_PORT, STEPPER_DIR1_GPIO_PIN, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define ST2_DIR(x)    HAL_GPIO_WritePin(STEPPER_DIR2_GPIO_PORT, STEPPER_DIR2_GPIO_PIN, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define ST3_DIR(x)    HAL_GPIO_WritePin(STEPPER_DIR3_GPIO_PORT, STEPPER_DIR3_GPIO_PIN, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define ST4_DIR(x)    HAL_GPIO_WritePin(STEPPER_DIR4_GPIO_PORT, STEPPER_DIR4_GPIO_PIN, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)

/* 使能控制宏 */
#define ST1_EN(x)     HAL_GPIO_WritePin(STEPPER_EN1_GPIO_PORT, STEPPER_EN1_GPIO_PIN, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define ST2_EN(x)     HAL_GPIO_WritePin(STEPPER_EN2_GPIO_PORT, STEPPER_EN2_GPIO_PIN, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define ST3_EN(x)     HAL_GPIO_WritePin(STEPPER_EN3_GPIO_PORT, STEPPER_EN3_GPIO_PIN, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define ST4_EN(x)     HAL_GPIO_WritePin(STEPPER_EN4_GPIO_PORT, STEPPER_EN4_GPIO_PIN, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)

/******************************************************************************************/
/* 导出函数与全局状态变量 */

extern speedRampData g_srd;               /* 加减速全局参数结构体 */
extern __IO int32_t  g_step_position;     /* 当前电机绝对脉冲位置 */
extern __IO uint8_t  g_motion_sta;        /* 运动状态：0-空闲/已停止，1-正在运动 */
extern __IO uint32_t g_add_pulse_count;   /* 累计已输出的脉冲计数 */

void stepper_init(uint16_t arr, uint16_t psc);
void stepper_star(uint8_t motor_num);
void stepper_stop(uint8_t motor_num);
void create_t_ctrl_param(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed);

#endif /* __STEPPER_MOTOR_H */