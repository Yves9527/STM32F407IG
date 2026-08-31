/**
 ****************************************************************************************************
 * @file        dcmotor_time.c
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

#include "./BSP/TIMER/dcmotor_tim.h"
#include "./BSP/LED/led.h"
#include "./BSP/DC_MOTOR/dc_motor.h"
#include "./BSP/PID/pid.h"
#include "./DEBUG/debug.h"

/******************************* 第一部分  电机基本驱动 互补输出带死区控制 **************************************/
/* 电机控制参数限制与周期定义 */
#define MOTOR_SPEED_CALC_PERIOD_MS    5       /* 速度计算采样周期: 5ms */
#define MOTOR_MAX_TARGET_CURRENT        120.0f  /* 位置环输出限幅（目标电流最大值） */
#define MOTOR_MAX_PWM_DUTY            8200.0f /* 速度环输出限幅（PWM占空比最大值） */


/* 定时器配置句柄 定义 */
TIM_HandleTypeDef g_atimx_cplm_pwm_handle;                              /* 定时器x句柄 */

/* 互补输出带死区控制 */
TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;                    /* 死区时间设置 */

/**
 * @brief       高级定时器TIMX 互补输出 初始化函数（使用PWM模式1）
 * @note
 *              配置高级定时器TIMX 互补输出, 一路OCy 一路OCyN, 并且可以设置死区时间
 *
 *              高级定时器的时钟来自APB2, 而PCLK2 = 168Mhz, 我们设置PPRE2不分频, 因此
 *              高级定时器时钟 = 168Mhz
 *              定时器溢出时间计算方法: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=定时器工作频率, 单位 : Mhz
 *
 * @param       arr: 自动重装值。
 * @param       psc: 时钟预分频数
 * @retval      无
 */

void atim_timx_cplm_pwm_init(uint16_t arr, uint16_t psc)
{
    TIM_OC_InitTypeDef sConfigOC ;

    g_atimx_cplm_pwm_handle.Instance = ATIM_TIMX_CPLM;                      /* 定时器x */
    g_atimx_cplm_pwm_handle.Init.Prescaler = psc;                           /* 定时器预分频系数 */
    g_atimx_cplm_pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;          /* 向上计数模式 */
    g_atimx_cplm_pwm_handle.Init.Period = arr;                              /* 自动重装载值 */
    g_atimx_cplm_pwm_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;    /* 时钟分频因子 */
    g_atimx_cplm_pwm_handle.Init.RepetitionCounter = 0;                     /* 重复计数器寄存器为0 */
    g_atimx_cplm_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;        /* 使能影子寄存器TIMx_ARR */
    HAL_TIM_PWM_Init(&g_atimx_cplm_pwm_handle) ;

    /* 设置PWM输出 */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;                                     /* PWM模式1 */
    sConfigOC.Pulse = 0;                                                    /* 比较值为0 */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;                              /* OCy 低电平有效 */
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;                            /* OCyN 低电平有效 */
    sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;                               /* 不使用快速模式 */
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;                          /* 主通道的空闲状态 */
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;                        /* 互补通道的空闲状态 */
    HAL_TIM_PWM_ConfigChannel(&g_atimx_cplm_pwm_handle, &sConfigOC, ATIM_TIMX_CPLM_CHY);    /* 配置后默认清CCER的互补输出位 */   
    
    /* 设置死区参数，开启死区中断 */
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;                 /* OSSR设置为1 */
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;               /* OSSI设置为0 */
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;                     /* 上电只能写一次，需要更新死区时间时只能用此值 */
    sBreakDeadTimeConfig.DeadTime = 0X0F;                                   /* 死区时间 */
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;                    /* BKE = 0, 关闭BKIN检测 */
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;             /* BKP = 1, BKIN低电平有效 */
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;     /* 使能AOE位，允许刹车后自动恢复输出 */
    HAL_TIMEx_ConfigBreakDeadTime(&g_atimx_cplm_pwm_handle, &sBreakDeadTimeConfig);         /* 设置BDTR寄存器 */

}

/**
 * @brief       定时器底层驱动，时钟使能，引脚配置
                此函数会被HAL_TIM_PWM_Init()调用
 * @param       htim:定时器句柄
 * @retval      无
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == ATIM_TIMX_CPLM)
    {
        GPIO_InitTypeDef gpio_init_struct;
        
        ATIM_TIMX_CPLM_CHY_GPIO_CLK_ENABLE();   /* 通道X对应IO口时钟使能 */
        ATIM_TIMX_CPLM_CHYN_GPIO_CLK_ENABLE();  /* 通道X互补通道对应IO口时钟使能 */
        ATIM_TIMX_CPLM_CLK_ENABLE();            /* 定时器x时钟使能 */

        /* 配置PWM主通道引脚 */
        gpio_init_struct.Pin = ATIM_TIMX_CPLM_CHY_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH ;
        gpio_init_struct.Alternate = ATIM_TIMX_CPLM_CHY_GPIO_AF;                /* 端口复用 */   
        HAL_GPIO_Init(ATIM_TIMX_CPLM_CHY_GPIO_PORT, &gpio_init_struct);

        /* 配置PWM互补通道引脚 */
        gpio_init_struct.Pin = ATIM_TIMX_CPLM_CHYN_GPIO_PIN;
        HAL_GPIO_Init(ATIM_TIMX_CPLM_CHYN_GPIO_PORT, &gpio_init_struct);
    }
}


/******************************* 第二部分  电机编码器测速 ****************************************************/

/********************************* 1 通用定时器 编码器程序 *************************************/

TIM_HandleTypeDef g_timx_encode_chy_handle;         /* 定时器x句柄 */
TIM_Encoder_InitTypeDef g_timx_encoder_chy_handle;  /* 定时器编码器句柄 */

/**
 * @brief       通用定时器TIMX 通道Y 编码器接口模式 初始化函数
 * @note
 *              通用定时器的时钟来自APB1,当PPRE1 ≥ 2分频的时候
 *              通用定时器的时钟为APB1时钟的2倍, 而APB1为42M, 所以定时器时钟 = 84Mhz
 *              定时器溢出时间计算方法: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=定时器工作频率,单位:Mhz
 *
 * @param       arr: 自动重装值。
 * @param       psc: 时钟预分频数
 * @retval      无
 */
void gtim_timx_encoder_chy_init(uint16_t arr, uint16_t psc)
{
    /* 定时器x配置 */
    g_timx_encode_chy_handle.Instance = GTIM_TIMX_ENCODER;                      /* 定时器x */
    g_timx_encode_chy_handle.Init.Prescaler = psc;                              /* 定时器分频 */
    g_timx_encode_chy_handle.Init.Period = arr;                                 /* 自动重装载值 */
    g_timx_encode_chy_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;       /* 时钟分频因子 */
    
    /* 定时器x编码器配置 */
    g_timx_encoder_chy_handle.EncoderMode = TIM_ENCODERMODE_TI12;               /* TI1、TI2都检测，4倍频 */
    g_timx_encoder_chy_handle.IC1Polarity = TIM_ICPOLARITY_RISING;              /* 输入极性，非反向 */
    g_timx_encoder_chy_handle.IC1Selection = TIM_ICSELECTION_DIRECTTI;          /* 输入通道选择 */
    g_timx_encoder_chy_handle.IC1Prescaler = TIM_ICPSC_DIV1;                    /* 不分频 */
    g_timx_encoder_chy_handle.IC1Filter = 10;                                   /* 滤波器设置 */
    g_timx_encoder_chy_handle.IC2Polarity = TIM_ICPOLARITY_RISING;              /* 输入极性，非反向 */
    g_timx_encoder_chy_handle.IC2Selection = TIM_ICSELECTION_DIRECTTI;          /* 输入通道选择 */
    g_timx_encoder_chy_handle.IC2Prescaler = TIM_ICPSC_DIV1;                    /* 不分频 */
    g_timx_encoder_chy_handle.IC2Filter = 10;                                   /* 滤波器设置 */
    HAL_TIM_Encoder_Init(&g_timx_encode_chy_handle, &g_timx_encoder_chy_handle);/* 初始化定时器x编码器 */
     
    HAL_TIM_Encoder_Start(&g_timx_encode_chy_handle,GTIM_TIMX_ENCODER_CH1);     /* 使能编码器通道1 */
    HAL_TIM_Encoder_Start(&g_timx_encode_chy_handle,GTIM_TIMX_ENCODER_CH2);     /* 使能编码器通道2 */
    __HAL_TIM_ENABLE_IT(&g_timx_encode_chy_handle,TIM_IT_UPDATE);               /* 使能更新中断 */
    __HAL_TIM_CLEAR_FLAG(&g_timx_encode_chy_handle,TIM_IT_UPDATE);              /* 清除更新中断标志位 */
    
}

/**
 * @brief       定时器底层驱动，时钟使能，引脚配置
                此函数会被HAL_TIM_Encoder_Init()调用
 * @param       htim:定时器句柄
 * @retval      无
 */
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == GTIM_TIMX_ENCODER)
    {
        GPIO_InitTypeDef gpio_init_struct;
        GTIM_TIMX_ENCODER_CH1_GPIO_CLK_ENABLE();                                 /* 开启通道x的GPIO时钟 */
        GTIM_TIMX_ENCODER_CH2_GPIO_CLK_ENABLE();
        GTIM_TIMX_ENCODER_CH1_CLK_ENABLE();                                      /* 开启定时器时钟 */
        GTIM_TIMX_ENCODER_CH2_CLK_ENABLE();

        gpio_init_struct.Pin = GTIM_TIMX_ENCODER_CH1_GPIO_PIN;                   /* 通道x的GPIO口 */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                                 /* 复用推挽输出 */
        gpio_init_struct.Pull = GPIO_NOPULL;                                     /* 上拉 */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;                           /* 高速 */
        gpio_init_struct.Alternate = GTIM_TIMX_ENCODERCH1_GPIO_AF;               /* 端口复用 */
        HAL_GPIO_Init(GTIM_TIMX_ENCODER_CH1_GPIO_PORT, &gpio_init_struct);  
        
        gpio_init_struct.Pin = GTIM_TIMX_ENCODER_CH2_GPIO_PIN;                   /* 通道x的GPIO口 */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                                 /* 复用推挽输出 */
        gpio_init_struct.Pull = GPIO_NOPULL;                                     /* 上拉 */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;                           /* 高速 */
        gpio_init_struct.Alternate = GTIM_TIMX_ENCODERCH2_GPIO_AF;               /* 端口复用 */
        HAL_GPIO_Init(GTIM_TIMX_ENCODER_CH2_GPIO_PORT, &gpio_init_struct);         
       
        HAL_NVIC_SetPriority(GTIM_TIMX_ENCODER_INT_IRQn, 2, 0);                  /* 中断优先级设置 */
        HAL_NVIC_EnableIRQ(GTIM_TIMX_ENCODER_INT_IRQn);                          /* 开启中断 */
    
    }
}

/**
 * @brief       定时器中断服务函数
 * @param       无
 * @retval      无
 */
void GTIM_TIMX_ENCODER_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_timx_encode_chy_handle);
}

/******************************** 2 基本定时器 编码器程序 ************************************/

TIM_HandleTypeDef timx_handler;         /* 定时器参数句柄 */

/**
 * @brief       基本定时器TIMX定时中断初始化函数
 * @note
 *              基本定时器的时钟来自APB1,当PPRE1 ≥ 2分频的时候
 *              基本定时器的时钟为APB1时钟的2倍, 而APB1为42M, 所以定时器时钟 = 84Mhz
 *              定时器溢出时间计算方法: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=定时器工作频率,单位:Mhz
 *
 * @param       arr: 自动重装值。
 * @param       psc: 时钟预分频数
 * @retval      无
 */
void btim_timx_int_init(uint16_t arr, uint16_t psc)
{
    timx_handler.Instance = BTIM_TIMX_INT;                              /* 基本定时器X */
    timx_handler.Init.Prescaler = psc;                                  /* 设置预分频器  */
    timx_handler.Init.CounterMode = TIM_COUNTERMODE_UP;                 /* 向上计数器 */
    timx_handler.Init.Period = arr;                                     /* 自动装载值 */
    timx_handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;           /* 时钟分频因子 */
    HAL_TIM_Base_Init(&timx_handler);
    
    HAL_TIM_Base_Start_IT(&timx_handler);                               /* 使能基本定时器x和及其更新中断：TIM_IT_UPDATE */
    __HAL_TIM_CLEAR_IT(&timx_handler,TIM_IT_UPDATE);                    /* 清除更新中断标志位 */
}

/**
 * @brief       定时器底册驱动，开启时钟，设置中断优先级
                此函数会被HAL_TIM_Base_Init()函数调用
 * @param       无
 * @retval      无
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == BTIM_TIMX_INT)
    {
        BTIM_TIMX_INT_CLK_ENABLE();                                     /*使能TIM时钟*/
        HAL_NVIC_SetPriority(BTIM_TIMX_INT_IRQn, 1, 3);                 /* 抢占1，子优先级3，组2 */
        HAL_NVIC_EnableIRQ(BTIM_TIMX_INT_IRQn);                         /*开启ITM3中断*/
    }
}

/**
 * @brief       基本定时器TIMX中断服务函数
 * @param       无
 * @retval      无
 */
void BTIM_TIMX_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&timx_handler);                                  /*定时器回调函数*/
}

/******************************** 3 公用部分 编码器程序 ************************************/

volatile int g_timx_encode_count = 0;                                   /* 溢出次数 */
uint8_t  g_run_flag = 0;                                                /* 电机启停状态标志，0：停止，1：启动 */

/**
 * @brief       数值限幅辅助函数
 * @param       val: 待限幅数值
 * @param       min: 最小值
 * @param       max: 最大值
 * @retval      限幅后结果
 */
static inline float math_clamp(float val, float min, float max)
{
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

/**
 * @brief       根据外环输出确定转向并提取单极性目标电流
 * @param       loc_out: 位置环原始输出 (含正负号)
 * @retval      单极性目标电流幅值 (>= 0)
 */
static inline float motor_update_dir_and_get_target_current(float loc_out)
{
    if (loc_out >= 0.0f)
    {
        dcmotor_dir(0);                     /* 目标为正，设置电机正转 */
        return loc_out;
    }
    else
    {
        dcmotor_dir(1);                     /* 目标为负，设置电机反转 */
        return -loc_out;                    /* 取绝对值作为电流目标幅值 */
    }
}

/**
 * @brief       位置+速度双环 PID 控制计算
 * @param       无
 * @retval      无
 */
static void motor_dual_loop_control_step(void)
{
    if (g_run_flag)                                             /* 判断电机是否启动了 */
    { 
        float raw_current_target;
        float current_target_clamped;

        /* 1. 位置环PID控制（外环） */
        raw_current_target = increment_pid_ctrl(&g_location_pid, g_motor_data.location);
        
        /* 2. 判断正负号设置电机转向，并提取目标电流绝对值 */
        current_target_clamped = motor_update_dir_and_get_target_current(raw_current_target);

        /* 3. 限制外环输出（单极性电流上限限幅: 0 ~ 120） */
        current_target_clamped = math_clamp(current_target_clamped, 0.0f, MOTOR_MAX_TARGET_CURRENT);
        g_current_pid.SetPoint = current_target_clamped;
        
        /* 4. 电流环PID控制（内环） */
        g_motor_data.motor_pwm = increment_pid_ctrl(&g_current_pid, g_motor_data.current);
        
        /* 5. 限制PWM占空比输出（单极性占空比限幅: 0 ~ 8200） */
        g_motor_data.motor_pwm = math_clamp(g_motor_data.motor_pwm, 0.0f, MOTOR_MAX_PWM_DUTY);

#if DEBUG_ENABLE  /* 发送基本参数 */
        debug_send_wave_data(1, g_motor_data.location);         /* 选择通道1，发送实际位置（波形显示） */
        debug_send_wave_data(2, g_location_pid.SetPoint);       /* 选择通道2，发送目标位置（波形显示） */
#endif
        /* 6. 输出占空比到硬件驱动 */
        dcmotor_speed((uint16_t)g_motor_data.motor_pwm);
    }
    else
    {
        dcmotor_speed(0);                                       /* 停止输出 */
    }
}
/**
 * @brief       定时器更新中断回调函数
 * @param        htim:定时器句柄指针
 * @note        此函数会被定时器中断函数共同调用的
 * @retval      无
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
     static uint8_t val = 0;

     if (htim->Instance == TIM3)
     {
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&g_timx_encode_chy_handle))   /* 判断CR1的DIR位 */
        {
            g_timx_encode_count--;                                      /* DIR位为1，也就是递减计数 */
        }
        else
        {
            g_timx_encode_count++;                                      /* DIR位为0，也就是递增计数 */
        }
     }
     else if (htim->Instance == TIM6)
     {
        static uint8_t val = 0;
        int encode_now = gtim_get_encode(); /* 获取当前编码器计数值 */

        /* 编码器滤波与速度计算 */
        speed_computer(encode_now, MOTOR_SPEED_CALC_PERIOD_MS);
        g_motor_data.location = (float)encode_now;

        /* 分频触发双环 PID 计算 */
        if (++val >= SMAPLSE_PID_SPEED)
        {
            val = 0;
            motor_dual_loop_control_step(); /* 执行位置+电流双环控制 */
        }
    }
}

/**
 * @brief       获取编码器的值
 * @param       无
 * @retval      编码器值
 */
int gtim_get_encode(void)
{
    return ( int32_t )__HAL_TIM_GET_COUNTER(&g_timx_encode_chy_handle) + g_timx_encode_count * 65536;   /* 总的编码器值 = 当前计数值 + 之前累计编码器的值 */
}

