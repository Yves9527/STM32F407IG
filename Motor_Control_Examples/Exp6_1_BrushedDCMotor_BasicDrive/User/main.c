/**
 ****************************************************************************************************
 * @file        main.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-16
 * @brief       直流有刷电机基础驱动 实验
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
 ****************************************************************************************************
 */

#include "SYSTEM/sys/sys.h"
#include "SYSTEM/usart/usart.h"
#include "SYSTEM/delay/delay.h"
#include "BSP/LED/led.h"
#include "BSP/KEY/key.h"
#include "BSP/LCD/lcd.h"
#include "BSP/TIMER/dcmotor_tim.h"
#include "BSP/DC_MOTOR/dc_motor.h"

/*-------------------------------  电机状态机定义  -------------------------------*/

/* 电机运行状态 */
typedef enum
{
    MOTOR_STOP     = 0,    /* 停止 */
    MOTOR_FORWARD,         /* 正转 */
    MOTOR_BACKWARD         /* 反转 */
} motor_state_t;

/* 状态机事件(由按键触发) */
typedef enum
{
    EVT_NONE = 0,          /* 无事件 */
    EVT_KEY0,              /* KEY0:增加比较值(正转/加速) */
    EVT_KEY1,              /* KEY1:减小比较值(反转/加速) */
    EVT_KEY2               /* KEY2:停止电机 */
} motor_event_t;

#define MOTOR_PWM_MAX   8400    /* PWM比较值限幅 */
#define MOTOR_PWM_STEP  400     /* 每次按键比较值变化量 */

static motor_state_t g_motor_state = MOTOR_STOP;    /* 当前运行状态 */
static int32_t       g_motor_pwm   = 0;             /* 当前PWM比较值 */

/**
 * @brief       由PWM比较值推导电机运行状态
 * @param       pwm: PWM比较值(有符号)
 * @retval      电机运行状态
 */
static motor_state_t motor_state_of(int32_t pwm)
{
    if (pwm > 0)
    {
        return MOTOR_FORWARD;
    }
    else if (pwm < 0)
    {
        return MOTOR_BACKWARD;
    }
    return MOTOR_STOP;
}

/**
 * @brief       应用目标PWM:限幅->推导状态->启/停电机->设置占空比
 * @param       pwm: 目标PWM比较值
 * @retval      无
 */
static void motor_apply_pwm(int32_t pwm)
{
    if (pwm >  MOTOR_PWM_MAX)
    {
        pwm =  MOTOR_PWM_MAX;
    }
    if (pwm < -MOTOR_PWM_MAX)
    {
        pwm = -MOTOR_PWM_MAX;
    }

    g_motor_pwm = pwm;
    g_motor_state = motor_state_of(pwm);

    if (g_motor_state == MOTOR_STOP)
    {
        dcmotor_stop();               /* 停止则立刻响应 */
    }
    else
    {
        dcmotor_start();              /* 开启电机 */
    }

    motor_pwm_set(g_motor_pwm);       /* 设置电机PWM占空比 */
}

/**
 * @brief       状态机事件处理:根据当前状态+事件 计算新PWM并转移状态
 * @param       evt: 按键事件
 * @retval      无
 */
static void motor_fsm_event(motor_event_t evt)
{
    int32_t pwm = g_motor_pwm;

    if (evt == EVT_KEY2)              /* 停止事件:翻转LED并停止电机 */
    {
        LED1_TOGGLE();
        motor_apply_pwm(0);
        return;
    }

    switch (g_motor_state)
    {
    case MOTOR_STOP:
        if (evt == EVT_KEY0)          /* 触发正转启动 */
        {
            pwm = g_motor_pwm + MOTOR_PWM_STEP;
        }
        else if (evt == EVT_KEY1)     /* 触发反转启动 */
        {
            pwm = g_motor_pwm - MOTOR_PWM_STEP;
        }
        break;

    case MOTOR_FORWARD:
        if (evt == EVT_KEY0)          /* 正转加速 */
        {
            pwm = g_motor_pwm + MOTOR_PWM_STEP;
        }
        else if (evt == EVT_KEY1)     /* 正转减速,跨过0点后进入反转 */
        {
            pwm = g_motor_pwm - MOTOR_PWM_STEP;
        }
        break;

    case MOTOR_BACKWARD:
        if (evt == EVT_KEY0)          /* 反转减速,跨过0点后进入正转 */
        {
            pwm = g_motor_pwm + MOTOR_PWM_STEP;
        }
        else if (evt == EVT_KEY1)     /* 反转加速 */
        {
            pwm = g_motor_pwm - MOTOR_PWM_STEP;
        }
        break;

    default:
        break;
    }

    if ((evt == EVT_KEY0) || (evt == EVT_KEY1))
    {
        motor_apply_pwm(pwm);
    }
}

int main(void)
{
    uint8_t t;

    HAL_Init();                           /* 初始化HAL库 */
    sys_stm32_clock_init(336, 8, 2, 7);   /* 设置时钟,168Mhz */
    delay_init(168);                      /* 延时初始化 */
    usart_init(115200);                   /* 串口初始化为115200 */
    led_init();                           /* 初始化LED */
    lcd_init();                           /* 初始化LCD */
    key_init();                           /* 初始化按键 */
    atim_timx_cplm_pwm_init(8400 - 1, 0); /* 168 000 000 / 1 = 168 000 000 168Mhz的计数频率，计数8400次为50us */
    dcmotor_init();                       /* 初始化电机 */

    g_point_color = WHITE;
    g_back_color = BLACK;
    lcd_show_string(10, 10, 200, 16, 16, "DcMotor Test", g_point_color);
    lcd_show_string(10, 30, 200, 16, 16, "KEY0:Start forward", g_point_color);
    lcd_show_string(10, 50, 200, 16, 16, "KEY1:Start backward", g_point_color);
    lcd_show_string(10, 70, 200, 16, 16, "KEY2:Stop", g_point_color);

    printf("KEY0：增加比较值，KEY1：减小比较值，KEY2：停止电机\r\n");

    while (1)
    {
        uint8_t key = key_scan(0);          /* 按键扫描 */
        motor_event_t evt = EVT_NONE;       /* 当前事件 */

        /* 把按键映射成状态机事件 */
        if (key == KEY0_PRES)
        {
            evt = EVT_KEY0;
        }
        else if (key == KEY1_PRES)
        {
            evt = EVT_KEY1;
        }
        else if (key == KEY2_PRES)
        {
            evt = EVT_KEY2;
        }

        motor_fsm_event(evt);               /* 交给状态机处理 */

        delay_ms(10);
        t++;
        if (t % 20 == 0)
        {
            LED0_TOGGLE(); /*LED0(红灯) 翻转*/
        }
    }
}
