/**
 ****************************************************************************************************
 * @file        main.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-16
 * @brief       舵机控制 实验
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 电机开发板F407开发板
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
#include "./BSP/TIMER/atim.h"
#include "./BSP/KEY/key.h"
#include "./BSP/STEERING_ENGINE/steering_engine.h"
#include "./BSP/LCD/lcd.h"

int main(void)
{     
    uint8_t key,t,id = 1;
    char buf[32];
    float angle[3] = {0};                       /* 初始化角度0° */
    
    HAL_Init();                                 /* 初始化HAL库 */
    sys_stm32_clock_init(336, 8, 2, 7);         /* 设置时钟,168Mhz */
    delay_init(168);                            /* 延时初始化 */
    usart_init(115200);                         /* 串口初始化为115200 */
    led_init();                                 /* 初始化LED */
    key_init();                                 /* 初始化按键 */
    lcd_init();                                 /* 初始化LCD */
    atim_timx_pwm_chy_init(20000 - 1, 168 - 1); /* 168 000 000 / 168 = 1000 000 10Khz的计数频率，计数5K次为500ms */
   
    g_point_color = WHITE;
    g_back_color  = BLACK;
    lcd_show_string(10,10,200,16,16,"Servo Test",g_point_color);        /* 显示提示信息 */
    lcd_show_string(10,30,200,16,16,"KEY0:ID + +",g_point_color);
    lcd_show_string(10,50,200,16,16,"KEY1:IAngle +   ",g_point_color);
    lcd_show_string(10,70,200,16,16,"KEY2:Angle -",g_point_color);
    lcd_show_string(10,90,200,16,16,"Servo ID: 1",g_point_color);
    printf("KEY0 选择控制的舵机接口\r\n");
    printf("KEY1 舵机旋转角度+45°\r\n");
    printf("KEY2 舵机旋转角度-45°\r\n");
    
    while (1)
    {      
        if(t % 10 == 1)
        {
            sprintf(buf,"Servo 1: %.1f",angle[0]);
            lcd_show_string(10,110,200,16,16,buf,g_point_color);
            sprintf(buf,"Servo 2: %.1f",angle[1]);
            lcd_show_string(10,130,200,16,16,buf,g_point_color);
            sprintf(buf,"Servo 3: %.1f",angle[2]);
            lcd_show_string(10,150,200,16,16,buf,g_point_color);
        }
        
        key = key_scan(0);     
        if(key == KEY0_PRES)
        {
            id++;
            if(id == 4)
            {
                id = 1;
            }
            sprintf(buf,"Servo ID: %1d",id);        /* 按下KEY0：选择控制哪个舵机，并显示当前ID */
            lcd_show_string(10,90,200,16,16,buf,g_point_color);
            printf("所选舵机接口为%d\r\n",id);
        }
        else if(key == KEY1_PRES)
        {
            angle[id-1] += 45;
            if(angle[id-1] > 180)
            {
                angle[id-1] = 180;
            }
            servo_angle_set(id,angle[id-1]);        /* 控制该ID的舵机，并设置角度值 */
            printf("Servo 1: %.1f\r\n",angle[0]);
            printf("Servo 2: %.1f\r\n",angle[1]);
            printf("Servo 3: %.1f\r\n",angle[2]);
            printf("\r\n");
        }
        else if(key == KEY2_PRES)
        {
            angle[id-1] -= 45;
            if(angle[id-1] < 0)
            {
                angle[id-1] = 0;
            }
            servo_angle_set(id,angle[id-1]);        /* 控制该ID的舵机，并设置角度值 */
            printf("Servo 1: %.1f\r\n",angle[0]);
            printf("Servo 2: %.1f\r\n",angle[1]);
            printf("Servo 3: %.1f\r\n",angle[2]);
            printf("\r\n");
        }   
        
        t++;
        if(t % 20 == 0)
        {
            LED0_TOGGLE();                          /* LED0(红灯) 翻转 */        
        }
        delay_ms(10);
    }
}
