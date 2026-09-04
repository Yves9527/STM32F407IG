/**
 ****************************************************************************************************
 * @file        main.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-16
 * @brief       步进电机梯形加减速 实验
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
 ****************************************************************************************************
 */

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/TIMER/stepper_tim.h"
#include "./BSP/KEY/key.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/STEPPER_MOTOR/stepper_motor.h"

/*  加速度和减速度选取一般根据实际需要，值越大速度变化越快，加减速阶段比较抖动
    所以加速度和减速度值一般是在实际应用中多尝试出来的结果 */
    
__IO uint32_t g_set_speed  = 1000;          /* 最大速度 单位为0.1rad/sec */
__IO uint32_t g_step_accel = 25;            /* 加速度 单位为0.1rad/sec^2 */
__IO uint32_t g_step_decel = 20;            /* 减速度 单位为0.1rad/sec^2 */
__IO uint16_t g_step_angle = 1;             /* 设置的步数*/
extern __IO uint32_t g_add_pulse_count;     /* 脉冲个数累计*/

int main(void)
{     
    uint8_t key,t;
    char buf[32];
    
    HAL_Init();                             /* 初始化HAL库 */
    sys_stm32_clock_init(336, 8, 2, 7);     /* 设置时钟,168Mhz */
    delay_init(168);                        /* 延时初始化 */
    usart_init(115200);                     /* 串口初始化为115200 */
    led_init();                             /* 初始化LED */
    key_init();                             /* 初始化按键 */
    lcd_init();                             /* 初始化LCD */
    stepper_init(0xFFFF, 84 - 1);           /* 168 000 000 / 84 = 2000 000 2M的计数频率 */
   
    g_point_color = WHITE;
    g_back_color  = BLACK;
    lcd_show_string(10,10,200,16,16,"Stepper Motor Test",g_point_color);/* 显示提示信息 */
    lcd_show_string(10,30,200,16,16,"KEY0:Start", g_point_color);       /* 显示提示信息 */
    lcd_show_string(10,50,200,16,16,"KEY1:Set++",g_point_color);        /* 显示提示信息 */
    lcd_show_string(10,70,200,16,16,"KEY2:Set--",g_point_color);        /* 显示提示信息 */
    printf("KEY0开启梯形加减速\r\n");
    printf("KEY1增加步数\r\n");
    printf("KEY2减少步数\r\n");
    
    while (1)
    {     
        t++;
        if(t % 20 == 0)
        {            
            sprintf(buf,"Set_Aangle:%d     ",g_step_angle);             /* 设置的旋转位置（角度）*/
            lcd_show_string(10,90,200,16,16,buf,g_point_color);
            sprintf(buf,"Add_Aangle:%.2f    ",g_add_pulse_count*0.225); /* 累计旋转的角度 */
            lcd_show_string(10,110,200,16,16,buf,g_point_color);
            LED0_TOGGLE();                                              /* LED0(红灯) 翻转 */        
        }
        delay_ms(10);
        key = key_scan(0);
        if(key == KEY0_PRES)                                            /* 开启梯形加减速 */
        {
            create_t_ctrl_param(SPR*g_step_angle, g_step_accel, g_step_decel, g_set_speed);
            g_add_pulse_count=0;
        }
        else if(key == KEY1_PRES)                                       /* 增加步数 */
        {
            g_step_angle++;
            if(g_step_angle>100)  g_step_angle=1;
        }
        else if(key == KEY2_PRES)                                       /* 减少步数 */
        {
            g_step_angle--;
            if(g_step_angle<1)  g_step_angle=100;
        }
    }
}
