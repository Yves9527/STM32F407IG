/**
 ****************************************************************************************************
 * @file        stepper_motor.c
 * @author      正点原子团队(ALIENTEK) / 优化重构版
 * @version     V2.0
 * @date        2024-05-10
 * @brief       步进电机 梯形加减速控制实现
 * @encoding    GBK / GB2312
 ****************************************************************************************************
 */

#include "./BSP/STEPPER_MOTOR/stepper_motor.h"
#include "./BSP/TIMER/stepper_tim.h"
#include <math.h>

/* 全局运动状态及统计 */
speedRampData g_srd             = {STOP, CW, 0, 0, 0, 0, 0};
__IO int32_t  g_step_position   = 0;
__IO uint8_t  g_motion_sta      = 0;
__IO uint32_t g_add_pulse_count = 0;

/**
 * @brief       初始化步进电机GPIO与定时器输出比较通道
 * @param       arr: 定时器自动重装载值
 * @param       psc: 预分频系数
 * @retval      无
 */
void stepper_init(uint16_t arr, uint16_t psc)
{
    GPIO_InitTypeDef gpio_init_struct;

    STEPPER_DIR1_GPIO_CLK_ENABLE();
    STEPPER_DIR2_GPIO_CLK_ENABLE();
    STEPPER_DIR3_GPIO_CLK_ENABLE();
    STEPPER_DIR4_GPIO_CLK_ENABLE();

    STEPPER_EN1_GPIO_CLK_ENABLE();
    STEPPER_EN2_GPIO_CLK_ENABLE();
    STEPPER_EN3_GPIO_CLK_ENABLE();
    STEPPER_EN4_GPIO_CLK_ENABLE();

    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull  = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;

    /* 初始化方向引脚 */
    gpio_init_struct.Pin = STEPPER_DIR1_GPIO_PIN;
    HAL_GPIO_Init(STEPPER_DIR1_GPIO_PORT, &gpio_init_struct);
    gpio_init_struct.Pin = STEPPER_DIR2_GPIO_PIN;
    HAL_GPIO_Init(STEPPER_DIR2_GPIO_PORT, &gpio_init_struct);
    gpio_init_struct.Pin = STEPPER_DIR3_GPIO_PIN;
    HAL_GPIO_Init(STEPPER_DIR3_GPIO_PORT, &gpio_init_struct);
    gpio_init_struct.Pin = STEPPER_DIR4_GPIO_PIN;
    HAL_GPIO_Init(STEPPER_DIR4_GPIO_PORT, &gpio_init_struct);

    /* 初始化使能引脚 */
    gpio_init_struct.Pin = STEPPER_EN1_GPIO_PIN;
    HAL_GPIO_Init(STEPPER_EN1_GPIO_PORT, &gpio_init_struct);
    gpio_init_struct.Pin = STEPPER_EN2_GPIO_PIN;
    HAL_GPIO_Init(STEPPER_EN2_GPIO_PORT, &gpio_init_struct);
    gpio_init_struct.Pin = STEPPER_EN3_GPIO_PIN;
    HAL_GPIO_Init(STEPPER_EN3_GPIO_PORT, &gpio_init_struct);
    gpio_init_struct.Pin = STEPPER_EN4_GPIO_PIN;
    HAL_GPIO_Init(STEPPER_EN4_GPIO_PORT, &gpio_init_struct);

    /* 初始化定时器脉冲输出比较通道 */
    atim_timx_oc_chy_init(arr, psc);
}

/**
 * @brief       启动指定通道脉冲
 */
void stepper_star(uint8_t motor_num)
{
    uint32_t channel;
    switch(motor_num)
    {
        case STEPPER_MOTOR_1: channel = ATIM_TIMX_PWM_CH1; break;
        case STEPPER_MOTOR_2: channel = ATIM_TIMX_PWM_CH2; break;
        case STEPPER_MOTOR_3: channel = ATIM_TIMX_PWM_CH3; break;
        case STEPPER_MOTOR_4: channel = ATIM_TIMX_PWM_CH4; break;
        default: return;
    }

    if (g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_PWM1 || g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_PWM2)
    {
        HAL_TIM_PWM_Start(&g_atimx_handle, channel);
    }
    else if (g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_TOGGLE)
    {
        HAL_TIM_OC_Start_IT(&g_atimx_handle, channel);
    }
}

/**
 * @brief       停止指定通道脉冲
 */
void stepper_stop(uint8_t motor_num)
{
    uint32_t channel;
    switch(motor_num)
    {
        case STEPPER_MOTOR_1: channel = ATIM_TIMX_PWM_CH1; break;
        case STEPPER_MOTOR_2: channel = ATIM_TIMX_PWM_CH2; break;
        case STEPPER_MOTOR_3: channel = ATIM_TIMX_PWM_CH3; break;
        case STEPPER_MOTOR_4: channel = ATIM_TIMX_PWM_CH4; break;
        default: return;
    }

    if (g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_PWM1 || g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_PWM2)
    {
        HAL_TIM_PWM_Stop(&g_atimx_handle, channel);
    }
    else if (g_atimx_oc_chy_handle.OCMode == TIM_OCMODE_TOGGLE)
    {
        HAL_TIM_OC_Stop_IT(&g_atimx_handle, channel);
    }
}

/**
 * @brief       梯形/三角形加减速轨迹规划器并启动运行
 * @param       step:  目标总步数 (正数: 顺时针 CW, 负数: 逆时针 CCW)
 * @param       accel: 加速度 (单位: 0.1 rad/s^2)
 * @param       decel: 减速度 (单位: 0.1 rad/s^2)
 * @param       speed: 最大巡航速度 (单位: 0.1 rad/s)
 * @retval      无
 */
void create_t_ctrl_param(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{
    uint32_t max_speed_steps; /* 加速到最大期望速度所需的理论步数 */
    uint32_t accel_limit;     /* 总步数约束下允许加速的最大步数 (三角形交点) */
    uint16_t tim_count;

    /* 若电机当前正在运动中，则忽略新指令 */
    if (g_motion_sta != STOP)
    {
        return;
    }

    /* 1. 确定方向并设置方向引脚电平 */
    if (step < 0)
    {
        g_srd.dir = CCW;
        ST3_DIR(CCW);
        step = -step; /* 转为正数进行统一的几何曲线规划 */
    }
    else
    {
        g_srd.dir = CW;
        ST3_DIR(CW);
    }

    /* 2. 边界情况处理 */
    if (step == 0)
    {
        return;
    }
    else if (step == 1)
    {
        /* 仅走 1 步：直接以单步脉冲处理 */
        g_srd.accel_count = -1;
        g_srd.run_state   = DECEL;
        g_srd.step_delay  = 1000;
        g_srd.decel_val   = -1;
        g_srd.decel_start = 1;
    }
    else
    {
        /* 3. 计算设定最高速度对应的最小定时器周期 min_delay
         * min_delay = (alpha * ft) / omega_max
         */
        g_srd.min_delay = (int32_t)(COEFF_MIN_DELAY / speed);

        /* 4. 计算起步第 0 步的定时器周期 c0 (初速度)
         * c0 = 0.676 * ft * sqrt(2 * alpha / a)
         */
        g_srd.step_delay = (int32_t)((COEFF_C0_FREQ * sqrt(COEFF_C0_RADIAN / accel)) / 10.0f);

        /* 5. 计算加速到设定最高速度所需要的步数 n_max
         * 运动学公式: omega^2 = 2 * a * theta = 2 * a * (n * alpha)
         * 推出: n_max = omega^2 / (2 * alpha * a)
         */
        max_speed_steps = (uint32_t)((speed * speed) / (COEFF_MAX_STEPS * accel / 10.0f));
        if (max_speed_steps == 0)
        {
            max_speed_steps = 1;
        }

        /* 6. 在有限总步数 step 的限制下，计算能加速的最大步数 accel_limit (三角形加减速交点)
         * 依据 加速距离 : 减速距离 = 减速度 : 加速度
         * n_accel = step * (decel / (accel + decel))
         */
        accel_limit = (uint32_t)(step * decel / (accel + decel));
        if (accel_limit == 0)
        {
            accel_limit = 1;
        }

        /* 7. 轨迹形态判决：是【梯形】还是【三角形】？ */
        if (accel_limit <= max_speed_steps)
        {
            /* 【三角形加减速】：步数不够，未达到最高速就必须刹车
             * 减速段步数 = step - accel_limit
             * 存入 decel_val 的值为负数: -(step - accel_limit) = accel_limit - step
             */
            g_srd.decel_val = (int32_t)(accel_limit - step);
        }
        else
        {
            /* 【梯形加减速】：能加速到最高速度，有一段匀速巡航
             * 减速段步数 = max_speed_steps * (accel / decel)
             * 同样存为负数形式
             */
            g_srd.decel_val = -(int32_t)(max_speed_steps * accel / decel);
        }

        if (g_srd.decel_val == 0)
        {
            g_srd.decel_val = -1;
        }

        /* 计算刹车减速的起点位置 (注意 decel_val 本身是负数):
         * decel_start = 总步数 - 减速步数 = step + decel_val
         */
        g_srd.decel_start = step + g_srd.decel_val;

        /* 8. 检查初速度是否已经大于等于最高速度设定 */
        if (g_srd.step_delay <= g_srd.min_delay)
        {
            g_srd.step_delay = g_srd.min_delay;
            g_srd.run_state  = RUN;   /* 周期已达极小值，直接进入匀速 */
        }
        else
        {
            g_srd.run_state  = ACCEL; /* 进入加速状态 */
        }

        g_srd.accel_count = 0;       /* 加速计数值复位从 0 开始 */
    }

    /* 9. 开启驱动器使能，装载定时器翻转比较值，启动中断 */
    g_motion_sta = 1;
    ST3_EN(EN_ON);

    tim_count = __HAL_TIM_GET_COUNTER(&g_atimx_handle);
    /* 定时器使用 Toggle 模式，翻转两次为 1 个完整周期，因此半周期比较值为 step_delay / 2 */
    __HAL_TIM_SET_COMPARE(&g_atimx_handle, ATIM_TIMX_PWM_CH3, tim_count + g_srd.step_delay / 2);
    HAL_TIM_OC_Start_IT(&g_atimx_handle, ATIM_TIMX_PWM_CH3);
}

/**
 * @brief       定时器输出比较中断回调 (加减速算法的实时执行引擎)
 * @param       htim: 定时器句柄
 * @retval      无
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint32_t tim_count;
    uint32_t next_compare_val;
    uint16_t new_step_delay;
    static uint16_t last_accel_delay  = 0; /* 保存加速结束时的周期，用于减速段首步衔接 */
    static uint32_t step_count        = 0; /* 当前总已走步数累计器 */
    static int32_t  rest              = 0; /* AVR446 整数除法余数累加器 (消除累积时基截断误差) */
    static uint8_t  toggle_half_pulse = 0; /* 翻转计数器：0-前半周期，1-后半周期 */

    if (htim->Instance == TIM8)
    {
        /* 1. 动态装载下一个半周期的定时器比较值 */
        tim_count = __HAL_TIM_GET_COUNTER(&g_atimx_handle);
        next_compare_val = tim_count + (g_srd.step_delay / 2);
        __HAL_TIM_SET_COMPARE(&g_atimx_handle, ATIM_TIMX_PWM_CH3, next_compare_val);

        /* 2. Toggle 翻转模式下：每 2 次中断才构成 1 个完整步进脉冲 */
        toggle_half_pulse++;
        if (toggle_half_pulse < 2)
        {
            return; /* 仅完成了半周期的电平翻转，返回等待下一个边沿 */
        }
        toggle_half_pulse = 0; /* 走完一个完整步进脉冲，重置半周期标志 */

        /* 3. 加减速状态机处理 */
        switch (g_srd.run_state)
        {
            case STOP:
            {
                /* 运动到站完成，关闭中断与使能 */
                step_count = 0;
                rest       = 0;
                HAL_TIM_OC_Stop_IT(&g_atimx_handle, ATIM_TIMX_PWM_CH3);
                ST3_EN(EN_OFF);
                g_motion_sta = 0;
                return;
            }

            case ACCEL:
            {
                step_count++;
                g_add_pulse_count++;
                g_step_position += (g_srd.dir == CW) ? 1 : -1;
                g_srd.accel_count++;

                /* David Austin 一阶泰勒逼近递推公式:
                 * c_n = c_(n-1) - (2 * c_(n-1) + rest) / (4n + 1)
                 */
                new_step_delay = g_srd.step_delay - (((2 * g_srd.step_delay) + rest) / (4 * g_srd.accel_count + 1));
                rest           = ((2 * g_srd.step_delay) + rest) % (4 * g_srd.accel_count + 1);

                /* 判定 1：是否到达减速起点 (适用于三角形曲线：步数不足以达到最高速提前刹车) */
                if (step_count >= g_srd.decel_start)
                {
                    g_srd.accel_count = g_srd.decel_val; /* 置为负数，进入对称减速阶段 */
                    g_srd.run_state   = DECEL;
                }
                /* 判定 2：是否达到最高速度 */
                else if (new_step_delay <= g_srd.min_delay)
                {
                    last_accel_delay = new_step_delay;
                    new_step_delay   = g_srd.min_delay; /* 钳位在最大转速 */
                    rest             = 0;
                    g_srd.run_state  = RUN;             /* 切换至匀速巡航段 */
                }
                break;
            }

            case RUN:
            {
                step_count++;
                g_add_pulse_count++;
                g_step_position += (g_srd.dir == CW) ? 1 : -1;

                new_step_delay = g_srd.min_delay;       /* 匀速段脉冲周期恒定 */

                /* 判定：到达减速刹车起点 */
                if (step_count >= g_srd.decel_start)
                {
                    g_srd.accel_count = g_srd.decel_val; /* 置为负数，开始减速 */
                    new_step_delay    = last_accel_delay;
                    g_srd.run_state   = DECEL;
                }
                break;
            }

            case DECEL:
            {
                step_count++;
                g_add_pulse_count++;
                g_step_position += (g_srd.dir == CW) ? 1 : -1;
                g_srd.accel_count++;

                /* 关键数学技巧：
                 * 此时 accel_count 是负数，因此分母 (4n+1) 为负数！
                 * 减去一个负数等于加上一个正数，使得 new_step_delay 逐级递增，电机实现平滑减速！
                 */
                new_step_delay = g_srd.step_delay - (((2 * g_srd.step_delay) + rest) / (4 * g_srd.accel_count + 1));
                rest           = ((2 * g_srd.step_delay) + rest) % (4 * g_srd.accel_count + 1);

                /* 当 accel_count 递增到 0 时，说明减速步数正好耗尽，精准到达目标终点 */
                if (g_srd.accel_count >= 0)
                {
                    g_srd.run_state = STOP;
                }
                break;
            }

            default:
                break;
        }

        /* 4. 更新下一个步进脉冲的周期基准 */
        g_srd.step_delay = new_step_delay;
    }
}