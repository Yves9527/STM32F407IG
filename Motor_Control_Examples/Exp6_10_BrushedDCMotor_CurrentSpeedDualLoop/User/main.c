/**
 ****************************************************************************************************
 * @file        main.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-16
 * @brief       直流有刷电机电流+速度双环PID 实验
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

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/KEY/key.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/DC_MOTOR/dc_motor.h"
#include "./BSP/ADC/adc.h"
#include "./BSP/TIMER/dcmotor_tim.h"
#include "./BSP/PID/pid.h"
#include "./DEBUG/debug.h"

extern uint8_t  g_run_flag;
volatile int32_t s_sum_error = 0;
volatile int32_t c_sum_error = 0;

void lcd_dis(void);

int main(void)
{
    uint8_t key;
    uint16_t t;
    uint8_t debug_cmd = 0;

    
    HAL_Init();                             /* 初始化HAL库 */
    sys_stm32_clock_init(336, 8, 2, 7);     /* 设置时钟,168Mhz */
    delay_init(168);                        /* 延时初始化 */
    usart_init(115200);                     /* 串口1初始化，用于上位机调试 */
    led_init();                             /* 初始化LED */
    lcd_init();                             /* 初始化LCD */
    key_init();                             /* 初始化按键 */
    pid_init();                             /* 初始化PID参数 */
    atim_timx_cplm_pwm_init(8400 - 1 , 0);  /* 168 000 000 / 1 = 168 000 000 168Mhz的计数频率，计数8400次为50us */
    dcmotor_init();                         /* 初始化电机 */
    gtim_timx_encoder_chy_init(0XFFFF, 0);  /* 初始化编码器定时器，不分频直接84M的计数频率 */
    btim_timx_int_init(1000 - 1 , 84 - 1);  /* 初始化基本定时器，1ms计数周期 */
    adc_nch_dma_init();                     /* 初始化ADC、DMA */

#if DEBUG_ENABLE                            /* 开启调试 */
    
    debug_init();                           /* 初始化调试 */
    debug_send_motorcode(DC_MOTOR);         /* 上传电机类型（直流有刷电机）*/
    debug_send_motorstate(IDLE_STATE);      /* 上传电机状态（空闲）*/
    
    /* 同步数据PID参数到上位机 ，无论同步哪一组PID，目标值地址只能是外环PID的 */
    debug_send_initdata(TYPE_PID1, (float *)(&g_speed_pid.SetPoint), S_KP, S_KI, S_KD); /* 速度环PID参数（PID1）*/
    debug_send_initdata(TYPE_PID2, (float *)(&g_speed_pid.SetPoint), C_KP, C_KI, C_KD); /* 电流环PID参数（PID2）*/

#endif

    g_point_color = WHITE;
    g_back_color  = BLACK;
    lcd_show_string(10, 10, 200, 16, 16, "DcMotor Test", g_point_color);
    lcd_show_string(10, 30, 200, 16, 16, "KEY0:Start forward", g_point_color);
    lcd_show_string(10, 50, 200, 16, 16, "KEY1:Start backward", g_point_color);
    lcd_show_string(10, 70, 200, 16, 16, "KEY2:Stop", g_point_color);

    while (1)
    {
        key = key_scan(0);                                  /* 按键扫描 */
        if(key == KEY0_PRES)                                /* 当key0按下 */
        {
            g_run_flag = 1;                                 /* 标记电机启动 */
            g_speed_pid.SetPoint += 50;
            dcmotor_start();                                /* 开启电机 */
            
            if (g_speed_pid.SetPoint >= 300)                /* 限速 */
            {
                g_speed_pid.SetPoint = 300;
            }
#if DEBUG_ENABLE
            debug_send_motorstate(RUN_STATE);               /* 上传电机状态（运行） */
#endif
        }

        else if(key == KEY1_PRES)                           /* 当key1按下 */
        {
            g_run_flag = 1;                                 /* 标记电机启动 */
            g_speed_pid.SetPoint -= 50;
            dcmotor_start();                                /* 开启电机 */
            
            if (g_speed_pid.SetPoint <= -300)               /* 限速 */
            {
                g_speed_pid.SetPoint = -300;
            }
#if DEBUG_ENABLE
            debug_send_motorstate(RUN_STATE);               /* 上传电机状态（运行） */
#endif
        }

        else if(key == KEY2_PRES)                           /* 当key2按下 */
        {
            dcmotor_stop();                                 /* 停止电机 */
            pid_init();                                     /* 重置pid参数 */
            g_run_flag = 0;                                 /* 标记电机停止 */
            g_motor_data.motor_pwm = 0;
            motor_pwm_set(g_motor_data.motor_pwm);          /* 设置电机转向、速度 */
            
#if DEBUG_ENABLE
            debug_send_motorstate(BREAKED_STATE);           /* 上传电机状态（刹车） */
            debug_send_initdata(TYPE_PID1, (float *)(&g_speed_pid.SetPoint), S_KP, S_KI, S_KD);           /* 同步PID参数到上位机 */
            debug_send_initdata(TYPE_PID2, (float *)(&g_speed_pid.SetPoint), C_KP, C_KI, C_KD);
#endif
        }
        
#if DEBUG_ENABLE
        
        /* 接收PID助手设置的速度环PID参数 */
        debug_receive_pid(TYPE_PID1, (float *)&g_speed_pid.Proportion,(float *)&g_speed_pid.Integral,(float *)&g_speed_pid.Derivative);
        
        /* 接收PID助手设置的电流环PID参数 */
        debug_receive_pid(TYPE_PID2, (float *)&g_current_pid.Proportion,(float *)&g_current_pid.Integral, (float *)&g_current_pid.Derivative);
        
        debug_set_point_range(300, -300, 300);              /* 设置目标调节范围，有电流环的系统目标值突变不要太大 */
        debug_cmd = debug_receive_ctrl_code();              /* 读取上位机指令 */

        if (debug_cmd == BREAKED)                           /* 电机刹车 */
        {
            dcmotor_stop();                                 /* 停止电机 */
            pid_init();                                     /* 重置pid参数 */
            g_run_flag = 0;                                 /* 标记电机停止 */
            g_motor_data.motor_pwm = 0;
            motor_pwm_set(g_motor_data.motor_pwm);          /* 设置电机转向、速度 */
            debug_send_motorstate(BREAKED_STATE);           /* 上传电机状态（刹车） */
            debug_send_initdata(TYPE_PID1, (float *)(&g_speed_pid.SetPoint), S_KP, S_KI, S_KD);           /* 同步PID参数到上位机 */
            debug_send_initdata(TYPE_PID2, (float *)(&g_speed_pid.SetPoint), C_KP, C_KI, C_KD);
        } 
        else if (debug_cmd == RUN_CODE)                     /* 电机运行 */
        {  
            dcmotor_start();                                /* 开启电机 */
            g_speed_pid.SetPoint = 50;                      /* 设置目标速度 */
            g_run_flag = 1;                                 /* 标记电机启动 */
            debug_send_motorstate(RUN_STATE);               /* 上传电机状态（运行） */
        }
#endif
        t++;
        if(t % 20 == 0)
        {
            c_sum_error = (int)g_current_pid.SumError;      /* 此变量用于调试时查看电流环累计积分 */
            s_sum_error = (int)g_speed_pid.SumError;        /* 此变量用于调试时查看速度环累计积分 */
            
            lcd_dis();                                      /* 显示数据 */
            LED0_TOGGLE();                                  /* LED0(红灯) 翻转 */
#if DEBUG_ENABLE
            debug_send_speed(g_motor_data.speed);           /* 发送速度 */
#endif
        }
        delay_ms(10);
    }
}

/**
 * @brief       数据显示函数
 * @param       无
 * @retval      无
 */
void lcd_dis(void)
{
    char buf[32];
    
    /* 显示占空比 */
    sprintf(buf, "PWM_Duty:%.1f%c  ", (float)(g_motor_data.motor_pwm * 100 / 8400), '%');
    lcd_show_string(10, 150, 200, 16, 16, buf, g_point_color);

    /* 显示目标速度 */
    sprintf(buf, "SpeedSet:%3d RPM ", (int16_t)g_speed_pid.SetPoint);
    lcd_show_string(10, 170, 200, 16, 16, buf, g_point_color);

    /* 显示实际速度 */
    sprintf(buf, "Speed:%3d RPM", (int16_t)g_motor_data.speed);
    lcd_show_string(10, 190, 200, 16, 16, buf, g_point_color);
    
    /* 显示实际电流 */
    sprintf(buf, "Current:%.3fmA ", g_motor_data.current);
    lcd_show_string(10, 210, 200, 16, 16, buf, g_point_color);
}


