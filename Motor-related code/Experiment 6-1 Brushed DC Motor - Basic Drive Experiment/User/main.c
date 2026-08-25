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

int main(void)
{
    uint8_t key, t;
    int32_t motor_pwm = 0;

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
        key = key_scan(0);    /* 按键扫描 */
        if (key == KEY0_PRES) /* 当key0按下 */
        {
            motor_pwm += 400; /* 因为不同的电机最小启动电压不同，可能在第一次增加的时候电机还不能转起来 */

            if (motor_pwm == 0)
            {
                dcmotor_stop(); /* 停止则立刻响应 */
                motor_pwm = 0;
            }
            else
            {
                dcmotor_start();       /* 开启电机 */
                if (motor_pwm >= 8400) /* 限速 */
                {
                    motor_pwm = 8400;
                }
            }
            motor_pwm_set(motor_pwm); /* 设置电机PWM的占空比 */
        }

        else if (key == KEY1_PRES) /* 当key1按下 */
        {
            motor_pwm -= 400;
            if (motor_pwm == 0)
            {
                dcmotor_stop(); /* 停止则立刻响应 */
                motor_pwm = 0;
            }
            else
            {
                dcmotor_start();        /* 开启电机 */
                if (motor_pwm <= -8400) /* 限速 */
                {
                    motor_pwm = -8400;
                }
            }
            motor_pwm_set(motor_pwm); /* 设置电机PWM的占空比 */
        }

        else if (key == KEY2_PRES) /* 当key2按下 */
        {
            LED1_TOGGLE();
            dcmotor_stop(); /* 关闭电机 */
            motor_pwm = 0;
            motor_pwm_set(motor_pwm); /* 设置电机PWM的占空比 */
        }

        delay_ms(10);
        t++;
        if (t % 20 == 0)
        {
            LED0_TOGGLE(); /*LED0(红灯) 翻转*/
        }
    }
}
