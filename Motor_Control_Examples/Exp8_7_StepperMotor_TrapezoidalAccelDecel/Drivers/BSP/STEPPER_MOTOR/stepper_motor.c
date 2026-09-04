/**
 ****************************************************************************************************
 * @file        stepper_motor.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       步进电机 驱动代码
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
 
#include "./BSP/STEPPER_MOTOR/stepper_motor.h"
#include "./BSP/TIMER/stepper_tim.h"
#include "math.h"
/**
 * @brief       初始化步进电机相关IO口, 并使能时钟
 * @param       arr: 自动重装值
 * @param       psc: 时钟预分频数
 * @retval      无
 */
void stepper_init(uint16_t arr, uint16_t psc)
{
    GPIO_InitTypeDef gpio_init_struct;

    STEPPER_DIR1_GPIO_CLK_ENABLE();                                 /* DIR1时钟使能 */
    STEPPER_DIR2_GPIO_CLK_ENABLE();                                 /* DIR2时钟使能 */
    STEPPER_DIR3_GPIO_CLK_ENABLE();                                 /* DIR3时钟使能 */
    STEPPER_DIR4_GPIO_CLK_ENABLE();                                 /* DIR4时钟使能 */
            
    STEPPER_EN1_GPIO_CLK_ENABLE();                                  /* EN1时钟使能 */
    STEPPER_EN2_GPIO_CLK_ENABLE();                                  /* EN2时钟使能 */
    STEPPER_EN3_GPIO_CLK_ENABLE();                                  /* EN3时钟使能 */
    STEPPER_EN4_GPIO_CLK_ENABLE();                                  /* EN4时钟使能 */
    

    gpio_init_struct.Pin = STEPPER_DIR1_GPIO_PIN;                   /* DIR1引脚 */
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;                    /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLDOWN;                          /* 下拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;                   /* 低速 */
    HAL_GPIO_Init(STEPPER_DIR1_GPIO_PORT, &gpio_init_struct);       /* 初始化DIR1引脚 */

    gpio_init_struct.Pin = STEPPER_DIR2_GPIO_PIN;                   /* DIR2引脚 */
    HAL_GPIO_Init(STEPPER_DIR2_GPIO_PORT, &gpio_init_struct);       /* 初始化DIR2引脚 */

    gpio_init_struct.Pin = STEPPER_DIR3_GPIO_PIN;                   /* DIR3引脚 */
    HAL_GPIO_Init(STEPPER_DIR3_GPIO_PORT, &gpio_init_struct);       /* 初始化DIR3引脚 */

    gpio_init_struct.Pin = STEPPER_DIR4_GPIO_PIN;                   /* DIR4引脚 */
    HAL_GPIO_Init(STEPPER_DIR4_GPIO_PORT, &gpio_init_struct);       /* 初始化DIR4引脚 */
    
    /*   脱机引脚初始化   */
    
    gpio_init_struct.Pin = STEPPER_EN1_GPIO_PIN;                    /* EN1引脚 */
    HAL_GPIO_Init(STEPPER_EN1_GPIO_PORT, &gpio_init_struct);        /* 初始化EN1引脚 */
    
    gpio_init_struct.Pin = STEPPER_EN2_GPIO_PIN;                    /* EN2引脚 */
    HAL_GPIO_Init(STEPPER_EN2_GPIO_PORT, &gpio_init_struct);        /* 初始化EN2引脚 */
    
    gpio_init_struct.Pin = STEPPER_EN3_GPIO_PIN;                    /* EN3引脚 */
    HAL_GPIO_Init(STEPPER_EN3_GPIO_PORT, &gpio_init_struct);        /* 初始化EN3引脚 */
    
    gpio_init_struct.Pin = STEPPER_EN4_GPIO_PIN;                    /* EN4引脚 */
    HAL_GPIO_Init(STEPPER_EN4_GPIO_PORT, &gpio_init_struct);        /* 初始化EN4引脚 */
    
    atim_timx_oc_chy_init(arr, psc);                                /* 初始化PUL引脚，以及脉冲模式等 */
}

/**
 * @brief       开启步进电机
 * @param       motor_num: 步进电机接口序号
 * @retval      无
 */
void stepper_star(uint8_t motor_num)
{
    /* 开启对应PWM通道 */
    switch(motor_num)
    {
        case STEPPER_MOTOR_1 :
        {
            if(g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_PWM1||g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_PWM2) 
            {
                HAL_TIM_PWM_Start(&g_atimx_handle, ATIM_TIMX_PWM_CH1);       
            }
            if(g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_TOGGLE)
            {
                HAL_TIM_OC_Start_IT(&g_atimx_handle, ATIM_TIMX_PWM_CH1);    
            }         
            break;
        }
        case STEPPER_MOTOR_2 :
        {
            if(g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_PWM1||g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_PWM2) 
            {
                HAL_TIM_PWM_Start(&g_atimx_handle, ATIM_TIMX_PWM_CH2);      
            }
            if(g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_TOGGLE)
            {
                HAL_TIM_OC_Start_IT(&g_atimx_handle, ATIM_TIMX_PWM_CH2);     
            }
            break;
        }
        case STEPPER_MOTOR_3 :
        {
            if(g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_PWM1||g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_PWM2) 
            {
                HAL_TIM_PWM_Start(&g_atimx_handle, ATIM_TIMX_PWM_CH3);     
            }
            if(g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_TOGGLE)
            {
                HAL_TIM_OC_Start_IT(&g_atimx_handle, ATIM_TIMX_PWM_CH3);    
            }
            break;  
        }
        case STEPPER_MOTOR_4 :
        {
            if(g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_PWM1||g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_PWM2) 
            {
                HAL_TIM_PWM_Start(&g_atimx_handle, ATIM_TIMX_PWM_CH4);      
            }
            if(g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_TOGGLE)
            {
                HAL_TIM_OC_Start_IT(&g_atimx_handle, ATIM_TIMX_PWM_CH4);    
            }
            break;
        }
        default : break;
    }
}

/**
 * @brief       关闭步进电机
 * @param       motor_num: 步进电机接口序号
 * @retval      无
 */
void stepper_stop(uint8_t motor_num)
{
    /* 关闭对应PWM通道 */
    switch(motor_num)
    {
        case STEPPER_MOTOR_1 :
        {
            if(g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_PWM1||g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_PWM2) 
            {
                HAL_TIM_PWM_Stop(&g_atimx_handle, ATIM_TIMX_PWM_CH1);       
            }
            if(g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_TOGGLE)
            {
                HAL_TIM_OC_Stop_IT(&g_atimx_handle, ATIM_TIMX_PWM_CH1);    
            }
            break;
        }
        case STEPPER_MOTOR_2 :
        {
            if(g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_PWM1||g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_PWM2) 
            {
                HAL_TIM_PWM_Stop(&g_atimx_handle, ATIM_TIMX_PWM_CH2);       
            }
            if(g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_TOGGLE)
            {
                HAL_TIM_OC_Stop_IT(&g_atimx_handle, ATIM_TIMX_PWM_CH2);    
            }
            break;
        }
        case STEPPER_MOTOR_3 :
        {
            if(g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_PWM1||g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_PWM2) 
            {
                HAL_TIM_PWM_Stop(&g_atimx_handle, ATIM_TIMX_PWM_CH3);      
            }
            if(g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_TOGGLE)
            {
                HAL_TIM_OC_Stop_IT(&g_atimx_handle, ATIM_TIMX_PWM_CH3);   
            }
            break;  
        }
        case STEPPER_MOTOR_4 :
        {
            if(g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_PWM1||g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_PWM2) 
            {
                HAL_TIM_PWM_Stop(&g_atimx_handle, ATIM_TIMX_PWM_CH4);       
            }
            if(g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_TOGGLE)
            {
                HAL_TIM_OC_Stop_IT(&g_atimx_handle, ATIM_TIMX_PWM_CH4);   
            }
            break;
        }
        default : break;
    }
}


/********************************************梯形加减速***********************************************/
speedRampData g_srd               = {STOP,CW,0,0,0,0,0};  /* 加减速变量 */
__IO int32_t  g_step_position     = 0;                    /* 当前位置 */
__IO uint8_t  g_motion_sta        = 0;                    /* 是否在运动？0：停止，1：运动 */
__IO uint32_t g_add_pulse_count   = 0;                    /* 脉冲个数累计 */

/*
 * @brief       生成梯形运动控制参数
 * @param       step：移动的步数 (正数为顺时针，负数为逆时针).
 * @param       accel  加速度,实际值为accel*0.1*rad/sec^2  10倍并且2个脉冲算一个完整的周期
 * @param       decel  减速度,实际值为decel*0.1*rad/sec^2
 * @param       speed  最大速度,实际值为speed*0.1*rad/sec
 * @retval      无
 */
void create_t_ctrl_param(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{
    __IO uint16_t tim_count;        /* 达到最大速度时的步数*/
    __IO uint32_t max_s_lim;        /* 必须要开始减速的步数（如果加速没有达到最大速度）*/
    __IO uint32_t accel_lim;
    if(g_motion_sta != STOP)        /* 只允许步进电机在停止的时候才继续*/
        return;
    if(step < 0)                    /* 步数为负数 */
    {   
        g_srd.dir = CCW;            /* 逆时针方向旋转 */
        ST3_DIR(CCW);
        step = -step;               /* 获取步数绝对值 */
    }
    else
    {
        g_srd.dir = CW;             /* 顺时针方向旋转 */
        ST3_DIR(CW);
    }

    if(step == 1)                   /* 步数为1 */
    {
        g_srd.accel_count = -1;     /* 只移动一步 */
        g_srd.run_state = DECEL;    /* 减速状态. */
        g_srd.step_delay = 1000;    /* 默认速度 */
    }
    else if(step != 0)              /* 如果目标运动步数不为0*/
    {
        /*设置最大速度极限, 计算得到min_delay用于定时器的计数器的值 min_delay = (alpha / t)/ w*/
        g_srd.min_delay = (int32_t)(A_T_x10 /speed); //匀速运行时的计数值

        /* 通过计算第一个(c0) 的步进延时来设定加速度，其中accel单位为0.1rad/sec^2
         step_delay = 1/tt * sqrt(2*alpha/accel)
         step_delay = ( tfreq*0.69/10 )*10 * sqrt( (2*alpha*100000) / (accel*10) )/100 */
        
        g_srd.step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel))/10); /* c0 */

        max_s_lim = (uint32_t)(speed*speed / (A_x200*accel/10));/* 计算多少步之后达到最大速度的限制 max_s_lim = speed^2 / (2*alpha*accel) */

        if(max_s_lim == 0)                                      /* 如果达到最大速度小于0.5步，我们将四舍五入为0,但实际我们必须移动至少一步才能达到想要的速度 */
        {
            max_s_lim = 1;
        }
        accel_lim = (uint32_t)(step*decel/(accel+decel));       /* 这里不限制最大速度 计算多少步之后我们必须开始减速 n1 = (n1+n2)decel / (accel + decel) */

        if(accel_lim == 0)                                      /* 不足一步 按一步处理*/
        {
            accel_lim = 1;
        }
        if(accel_lim <= max_s_lim)                              /* 加速阶段到不了最大速度就得减速。。。使用限制条件我们可以计算出减速阶段步数 */
        {
            g_srd.decel_val = accel_lim - step;                 /* 减速段的步数 */
        }
        else
        {
            g_srd.decel_val = -(max_s_lim*accel/decel);         /* 减速段的步数 */
        }
        if(g_srd.decel_val == 0)                                /* 不足一步 按一步处理 */
        {
            g_srd.decel_val = -1;
        }
        g_srd.decel_start = step + g_srd.decel_val;             /* 计算开始减速时的步数 */
        
        
        if(g_srd.step_delay <= g_srd.min_delay)                 /* 如果一开始c0的速度比匀速段速度还大，就不需要进行加速运动，直接进入匀速 */
        {
            g_srd.step_delay = g_srd.min_delay;
            g_srd.run_state = RUN;
        }
        else  
        {
            g_srd.run_state = ACCEL;
        }
        g_srd.accel_count = 0;                                  /* 复位加减速计数值 */
    }
    g_motion_sta = 1;                                           /* 电机为运动状态 */
    ST3_EN(EN_ON);
    tim_count=__HAL_TIM_GET_COUNTER(&g_atimx_handle);
    __HAL_TIM_SET_COMPARE(&g_atimx_handle,ATIM_TIMX_PWM_CH3,tim_count+g_srd.step_delay/2);  /* 设置定时器比较值 */
    HAL_TIM_OC_Start_IT(&g_atimx_handle,ATIM_TIMX_PWM_CH3);                                 /* 使能定时器通道 */
}

/**
  * @brief  定时器比较中断
  * @param  htim：定时器句柄指针
  * @note   无
  * @retval 无
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
 
    __IO uint32_t tim_count = 0;
    __IO uint32_t tmp = 0;
    uint16_t new_step_delay = 0;                            /* 保存新（下）一个延时周期 */
    __IO static uint16_t last_accel_delay = 0;              /* 加速过程中最后一次延时（脉冲周期） */
    __IO static uint32_t step_count = 0;                    /* 总移动步数计数器*/
    __IO static int32_t rest = 0;                           /* 记录new_step_delay中的余数，提高下一步计算的精度 */
    __IO static uint8_t i = 0;                              /* 定时器使用翻转模式，需要进入两次中断才输出一个完整脉冲 */

    if(htim->Instance==TIM8)
    {
       
        tim_count = __HAL_TIM_GET_COUNTER(&g_atimx_handle);
        tmp = tim_count + g_srd.step_delay/2;               /* 整个C值里边是需要翻转两次的所以需要除以2 */
        __HAL_TIM_SET_COMPARE(&g_atimx_handle,ATIM_TIMX_PWM_CH3,tmp);

        i++;                                                /* 定时器中断次数计数值 */
        if(i == 2)                                          /* 2次，说明已经输出一个完整脉冲 */
        {
            i = 0;                                          /* 清零定时器中断次数计数值 */
            switch(g_srd.run_state)                         /* 加减速曲线阶段 */
            {
            case STOP:
                step_count = 0;                             /* 清零步数计数器 */
                rest = 0;                                   /* 清零余值 */
                /* 关闭通道*/
                HAL_TIM_OC_Stop_IT(&g_atimx_handle, ATIM_TIMX_PWM_CH3);
                ST3_EN(EN_OFF);
                g_motion_sta = 0;                           /* 电机为停止状态  */
                break;

            case ACCEL:
                g_add_pulse_count++;                        /* 只用于记录相对位置转动了多少度 */
                step_count++;                               /* 步数加1*/
                if(g_srd.dir == CW)
                {
                    g_step_position++;                      /* 绝对位置加1  记录绝对位置转动多少度*/
                }
                else
                {
                    g_step_position--;                      /* 绝对位置减1*/
                }
                g_srd.accel_count++;                        /* 加速计数值加1*/
                new_step_delay = g_srd.step_delay - (((2 *g_srd.step_delay) + rest)/(4 * g_srd.accel_count + 1));/* 计算新(下)一步脉冲周期(时间间隔) */
                rest = ((2 * g_srd.step_delay)+rest)%(4 * g_srd.accel_count + 1);                                /* 计算余数，下次计算补上余数，减少误差 */
                if(step_count >= g_srd.decel_start)         /* 检查是否到了需要减速的步数 */
                {
                    g_srd.accel_count = g_srd.decel_val;    /* 加速计数值为减速阶段计数值的初始值 */
                    g_srd.run_state = DECEL;                /* 下个脉冲进入减速阶段 */
                }
                else if(new_step_delay <= g_srd.min_delay)  /* 检查是否到达期望的最大速度 计数值越小速度越快，当你的速度和最大速度相等或更快就进入匀速*/
                {
                    last_accel_delay = new_step_delay;      /* 保存加速过程中最后一次延时（脉冲周期）*/
                    new_step_delay = g_srd.min_delay;       /* 使用min_delay（对应最大速度speed）*/
                    rest = 0;                               /* 清零余值 */
                    g_srd.run_state = RUN;                  /* 设置为匀速运行状态 */
                }
                break;

            case RUN:
                g_add_pulse_count++;
                step_count++;                               /* 步数加1 */
                if(g_srd.dir == CW)
                {
                    g_step_position++;                      /* 绝对位置加1 */
                }
                else
                {
                    g_step_position--;                      /* 绝对位置减1*/
                }
                new_step_delay = g_srd.min_delay;           /* 使用min_delay（对应最大速度speed）*/
                if(step_count >= g_srd.decel_start)         /* 需要开始减速 */
                {
                    g_srd.accel_count = g_srd.decel_val;    /* 减速步数做为加速计数值 */
                    new_step_delay = last_accel_delay;      /* 加阶段最后的延时做为减速阶段的起始延时(脉冲周期) */
                    g_srd.run_state = DECEL;                /* 状态改变为减速 */
                }
                break;

            case DECEL:
                step_count++;                               /* 步数加1 */
                g_add_pulse_count++;
                if(g_srd.dir == CW)
                {
                    g_step_position++;                      /* 绝对位置加1 */
                }
                else
                {
                    g_step_position--;                      /* 绝对位置减1 */
                }
                g_srd.accel_count++;
                new_step_delay = g_srd.step_delay - (((2 * g_srd.step_delay) + rest)/(4 * g_srd.accel_count + 1));  /* 计算新(下)一步脉冲周期(时间间隔) */
                rest = ((2 * g_srd.step_delay)+rest)%(4 * g_srd.accel_count + 1);                                   /* 计算余数，下次计算补上余数，减少误差 */

                /* 检查是否为最后一步 */
                if(g_srd.accel_count >= 0)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {
                    g_srd.run_state = STOP;
                }
                break;
            }
            g_srd.step_delay = new_step_delay;              /* 为下个(新的)延时(脉冲周期)赋值 */
        }
    }
}
