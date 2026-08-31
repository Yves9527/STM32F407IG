/**
 ****************************************************************************************************
 * @file        dc_motor.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       直流有刷电机控制代码
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
 * V1.0 20211014
 * 第一次发布
 *
 ****************************************************************************************************
 */

#include "math.h"
#include "./BSP/DC_MOTOR/dc_motor.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/TIMER/dcmotor_tim.h"
#include "./BSP/ADC/adc.h"
#include "./SYSTEM/usart/usart.h"

/*************************************    第一部分    基本驱动    *****************************************************/

extern TIM_HandleTypeDef g_atimx_cplm_pwm_handle;                              /* 定时器x句柄 */

/**
 * @brief       电机初始化
 * @param       无
 * @retval      无
 */
void dcmotor_init(void)
{
    SHUTDOWN_GPIO_CLK_ENABLE();
    GPIO_InitTypeDef gpio_init_struct;
    
    /* SD引脚设置，设置为推挽输出 */
    gpio_init_struct.Pin = SHUTDOWN1_Pin|SHUTDOWN2_Pin;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SHUTDOWN1_GPIO_Port, &gpio_init_struct);
    
    HAL_GPIO_WritePin(GPIOF, SHUTDOWN1_Pin|SHUTDOWN2_Pin, GPIO_PIN_RESET);      /* SD拉低，关闭输出 */
    
    dcmotor_stop();                 /* 停止电机 */
    dcmotor_dir(0);                 /* 设置正转 */
    dcmotor_speed(0);               /* 速度设置为0 */
    dcmotor_start();                /* 开启电机 */
}

/**
 * @brief       电机开启
 * @param       无
 * @retval      无
 */
void dcmotor_start(void)
{
    ENABLE_MOTOR;                                                       /* 拉高SD引脚，开启电机 */
}

/**
 * @brief       电机停止
 * @param       无
 * @retval      无
 */
void dcmotor_stop(void)
{
    HAL_TIM_PWM_Stop(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1);          /* 关闭主通道输出 */
    HAL_TIMEx_PWMN_Stop(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1);       /* 关闭互补通道输出 */
    DISABLE_MOTOR;                                                      /* 拉低SD引脚，停止电机 */
}

/**
 * @brief       电机旋转方向设置
 * @param       para:方向 0正转，1反转
 * @note        以电机正面，顺时针方向旋转为正转
 * @retval      无
 */
void dcmotor_dir(uint8_t para)
{
    HAL_TIM_PWM_Stop(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1);          /* 关闭主通道输出 */
    HAL_TIMEx_PWMN_Stop(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1);       /* 关闭互补通道输出 */

    if (para == 0)                /* 正转 */
    {
        HAL_TIM_PWM_Start(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1);     /* 开启主通道输出 */
    } 
    else if (para == 1)           /* 反转 */
    {
        HAL_TIMEx_PWMN_Start(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1);  /* 开启互补通道输出 */
    }
}

/**
 * @brief       电机速度设置
 * @param       para:比较寄存器值
 * @retval      无
 */
void dcmotor_speed(uint16_t para)
{
    if (para < (__HAL_TIM_GetAutoreload(&g_atimx_cplm_pwm_handle) - 0x0F))  /* 限速 */
    {  
        __HAL_TIM_SetCompare(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1, para);
    }
}

/**
 * @brief       电机控制
 * @param       para: pwm比较值 ,正数电机为正转，负数为反转
 * @note        根据传入的参数控制电机的转向和速度
 * @retval      无
 */
void motor_pwm_set(float para)
{
    int val = (int)para;

    if (val >= 0) 
    {
        dcmotor_dir(0);           /* 正转 */
        dcmotor_speed(val);
    } 
    else 
    {
        dcmotor_dir(1);           /* 反转 */
        dcmotor_speed(-val);
    }
}


/*************************************    第二部分    电压电流温度采集    **********************************************/

/*
    Rt = Rp *exp(B*(1/T1-1/T2))

    Rt 是热敏电阻在T1温度下的阻值；
    Rp是热敏电阻在T2常温下的标称阻值；
    exp是e的n次方，e是自然常数，就是自然对数的底数，近似等于 2.7182818；
    B值是热敏电阻的重要参数，教程中用到的热敏电阻B值为3380；
    这里T1和T2指的是开尔文温度，T2是常温25℃，即(273.15+25)K
    T1就是所求的温度
*/

const float Rp = 10000.0f;          /* 10K */
const float T2 = (273.15f + 25.0f); /* T2 */
const float Bx = 3380.0f;           /* B */
const float Ka = 273.15f;

/**
 * @brief       计算温度值
 * @param       para: 温度采集对应ADC通道的值（已滤波）
 * @note        计算温度分为两步：
                1.根据ADC采集到的值计算当前对应的Rt
                2.根据Rt计算对应的温度值
 * @retval      无
 */
float get_temp(uint16_t para)
{
    float Rt;
    float temp;
    
    /* 
        第一步：
        Rt = 3.3 * 4700 / VTEMP - 4700 ,其中VTEMP就是温度检测通道采集回来的电压值,VTEMP = ADC值* 3.3/4096
        由此我们可以计算出当前Rt的值：Rt = 3.3f * 4700.0f / (para * 3.3f / 4096.0f ) - 4700.0f; 
    */
    
    Rt = 3.3f * 4700.0f / (para * 3.3f / 4096.0f ) - 4700.0f;       /* 根据当前ADC值计算出Rt的值 */

    /* 
        第二步：
        根据当前Rt的值来计算对应温度值：Rt = Rp *exp(B*(1/T1-1/T2)) 
    */
    
    temp = Rt / Rp;                 /* 解出exp(B*(1/T1-1/T2)) ，即temp = exp(B*(1/T1-1/T2)) */
    temp = log(temp);               /* 解出B*(1/T1-1/T2) ，即temp = B*(1/T1-1/T2) */
    temp /= Bx;                     /* 解出1/T1-1/T2 ，即temp = 1/T1-1/T2 */
    temp += (1.0f / T2);            /* 解出1/T1 ，即temp = 1/T1 */
    temp = 1.0f / (temp);           /* 解出T1 ，即temp = T1 */
    temp -= Ka;                     /* 计算T1对应的摄氏度 */
    return temp;
}

extern uint16_t g_adc_value[ADC_CH_NUM * ADC_COLL];

/**
 * @brief       计算ADC的平均值（滤波）
 * @param       * p ：存放ADC值的指针地址
 * @note        此函数对电压、温度、电流对应的ADC值进行滤波，
 *              p[0]-p[2]对应的分别是电压、温度和电流
 * @retval      无
 */
void calc_adc_val(uint16_t * p)
{
    uint32_t temp[3] = {0,0,0};
    int i;
    for(i=0;i<ADC_COLL;i++)         /* 循环ADC_COLL次取值，累加 */
    {
        temp[0] += g_adc_value[0+i*ADC_CH_NUM];
        temp[1] += g_adc_value[1+i*ADC_CH_NUM];
        temp[2] += g_adc_value[2+i*ADC_CH_NUM];
    }
    temp[0] /= ADC_COLL;            /* 取平均值 */
    temp[1] /= ADC_COLL;
    temp[2] /= ADC_COLL;
    p[0] = temp[0];                 /* 存入电压ADC通道平均值 */
    p[1] = temp[1];                 /* 存入温度ADC通道平均值 */
    p[2] = temp[2];                 /* 存入电流ADC通道平均值 */
}


/*************************************    第三部分    编码器测速    ****************************************************/

Motor_TypeDef g_motor_data;  /*电机参数变量*/
ENCODE_TypeDef g_encode;     /*编码器参数变量*/

/**
 * @brief       电机速度计算
 * @param       encode_now：当前编码器总的计数值
 *              ms：计算速度的间隔，中断1ms进入一次，例如ms = 5即5ms计算一次速度
 * @retval      无
 */
void speed_computer(int32_t encode_now, uint8_t ms)
{
    uint8_t i = 0, j = 0;
    float temp = 0.0;
    static uint8_t sp_count = 0, k = 0;
    static float speed_arr[10] = {0.0};                     /* 存储速度进行滤波运算 */

    if (sp_count == ms)                                     /* 计算一次速度 */
    {
        /* 计算电机转速 
           第一步 ：计算ms毫秒内计数变化量
           第二步 ；计算1min内计数变化量：g_encode.speed * ((1000 / ms) * 60 ，
           第三步 ：除以编码器旋转一圈的计数次数（倍频倍数 * 编码器分辨率）
           第四步 ：除以减速比即可得出电机转速
        */
        g_encode.encode_now = encode_now;                                /* 取出编码器当前计数值 */
        g_encode.speed = (g_encode.encode_now - g_encode.encode_old);    /* 计算编码器计数值的变化量 */
        
        speed_arr[k++] = (float)(g_encode.speed * ((1000 / ms) * 60.0) / REDUCTION_RATIO / ROTO_RATIO );    /* 保存电机转速 */
        
        g_encode.encode_old = g_encode.encode_now;          /* 保存当前编码器的值 */

        /* 累计10次速度值，后续进行滤波*/
        if (k == 10)
        {
            for (i = 10; i >= 1; i--)                       /* 冒泡排序*/
            {
                for (j = 0; j < (i - 1); j++) 
                {
                    if (speed_arr[j] > speed_arr[j + 1])    /* 数值比较 */
                    { 
                        temp = speed_arr[j];                /* 数值换位 */
                        speed_arr[j] = speed_arr[j + 1];
                        speed_arr[j + 1] = temp;
                    }
                }
            }
            
            temp = 0.0;
            
            for (i = 2; i < 8; i++)                         /* 去除两边高低数据 */
            {
                temp += speed_arr[i];                       /* 将中间数值累加 */
            }
            
            temp = (float)(temp / 6);                       /*求速度平均值*/
            
            /* 一阶低通滤波
             * 公式为：Y(n)= qX(n) + (1-q)Y(n-1)
             * 其中X(n)为本次采样值；Y(n-1)为上次滤波输出值；Y(n)为本次滤波输出值，q为滤波系数
             * q值越小则上一次输出对本次输出影响越大，整体曲线越平稳，但是对于速度变化的响应也会越慢
             */
            g_motor_data.speed = (float)( ((float)0.48 * temp) + (g_motor_data.speed * (float)0.52) );
            k = 0;
        }
        sp_count = 0;
    }
    sp_count ++;
}



