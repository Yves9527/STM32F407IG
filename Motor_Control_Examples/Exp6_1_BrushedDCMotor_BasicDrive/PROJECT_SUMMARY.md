# 项目总结：直流有刷电机基础驱动（Exp6_1_BrushedDCMotor_BasicDrive）

## 1. 项目概述

| 项目 | 说明 |
| --- | --- |
| 实验名称 | 直流有刷电机基础驱动 实验 |
| 实验平台 | 正点原子 F407 电机开发板（STM32F407IG） |
| 实验目的 | 学习直流有刷电机的基础驱动，掌握定时器互补 PWM + 死区驱动 H 桥、正反转切换及调速 |
| 开发环境 | STM32 HAL 库 + STM32Cube 风格工程，主时钟 168 MHz |

本工程演示如何用一路高级定时器（TIM1）输出互补 PWM，配合停止引脚（SHDN）驱动直流有刷电机，并通过按键完成「正转 / 反转 / 停止 / 加减速」控制。

## 2. 硬件资源与引脚分配

| 功能 | 外设 / 引脚 | 说明 |
| --- | --- | --- |
| 运行指示灯 | LED0 (DS0, 红色) - PE0 | 指示程序在运行，约 200 ms 翻转一次 |
| 电机 PWM 主通道 | TIM1_CH1 - PA8 | 驱动板 IN1，PWM 主输出 |
| 电机 PWM 互补通道 | TIM1_CH1N - PB13 | 驱动板 IN2，PWM 互补输出 |
| 电机停止引脚 | PF10 | 驱动板 SHDN，关闭/开启输出 |
| LCD | 2.8/3.5/4.3/7/10 寸 TFTLCD（16 位 8080 并口） | 显示按键功能信息 |
| 电机端子 | 驱动板 M1 / M2 | 连接电机线 1 / 2 |

### 电源要求

- 驱动板供电：12 ~ 60 V，10 A 稳压（或可调）电源。
- 开发板供电：12 ~ 24 V，1 A 直流电源（DC 接口）。
- 开发板与驱动板之间用 24P 排线连接，默认使用**接口 1**。

> 实际上本实验只用到 3 个关键引脚：**PA8**（IN1）、**PB13**（IN2）、**PF10**（SHDN）。

## 3. 软件架构

工程使用 HAL 库 + 正点原子 SYSTEM/BSP 分层结构，核心目录如下（`User/` 与 `Drivers/` 分离）：

```
Exp6_1_BrushedDCMotor_BasicDrive/
├── User/                        # 用户代码
│   └── main.c                   # 主程序（含电机状态机）
├── Drivers/BSP/                 # 板级外设驱动
│   ├── LED/                     # LED 驱动（PE0/PE1）
│   ├── KEY/                     # 按键驱动（PE2/PE3/PE4）
│   ├── LCD/                     # LCD 显示驱动（16 位并口）
│   ├── TIMER/dcmotor_tim.c/h    # TIM1 互补 PWM 初始化
│   └── DC_MOTOR/dc_motor.c/h    # 直流有刷电机控制
└── Drivers/SYSTEM/              # 基础驱动
    ├── sys/                     # 时钟、系统初始化
    ├── usart/                   # 串口（printf 输出）
    └── delay/                   # 延时
```

### 各 BSP 模块职责

| 模块 | 职责 |
| --- | --- |
| `dcmotor_tim` | 配置 TIM1 互补 PWM 输出（PWM1 模式、互补通道、死区 0x0F），并完成 PA8/PB13 引脚复用 |
| `dc_motor` | 电机启停、方向、速度设置，封装为 `motor_pwm_set()` 接口 |
| `KEY` | 扫描 KEY0/KEY1/KEY2，返回对应按下事件 |
| `LED` | LED 点亮/熄灭/翻转 |
| `LCD` | 屏显按键功能说明 |

## 4. 核心控制原理

### 4.1 互补 PWM + 死区

- 使用高级定时器 **TIM1**（时钟经 APB2，计数频率 168 MHz）。
- 初始化参数：`atim_timx_cplm_pwm_init(8400 - 1, 0)`，即 ARR = 8399，预分频 = 0。
- PWM 周期：`Tout = ((arr+1) * (psc+1)) / Ft` = 8400 / 168 MHz ≈ **50 µs**。
- 主通道 CH1（PA8）与互补通道 CH1N（PB13）均配置为 **PWM 模式 1**、低电平有效，并设置**死区时间**（`DeadTime = 0x0F`），避免 H 桥上下管同时导通。

### 4.2 正反转控制

- 通过选择输出通道实现方向切换（H 桥换向）：
  - **正转**：`dcmotor_dir(0)` → 开启主通道 CH1（PA8）。
  - **反转**：`dcmotor_dir(1)` → 开启互补通道 CH1N（PB13）。
- 启动/关闭：通过拉高/拉低 **PF10（SHDN）** 使能或关断驱动输出（`dcmotor_start()` / `dcmotor_stop()`）。

### 4.3 调速

- 速度由 PWM 比较值（占空比）决定。`motor_pwm_set(float para)` 是统一入口：
  - `para > 0`：正转，`dcmotor_speed(val)`。
  - `para < 0`：反转，`dcmotor_speed(-val)`。
  - 内部限制了比较值上限（`dcmotor_speed` 中 `para < (ARR - 0x0F)`）。
- 每次按键增减固定步长：`MOTOR_PWM_STEP = 400`；上限 `MOTOR_PWM_MAX = 8400`。

## 5. 程序流程与状态机

原来主循环用多重 `if/else` 直接根据 `motor_pwm` 的正负号推断运行状态；本项目将其重构为**有限状态机（FSM）**，把「运行状态」显式化，按键作为事件驱动。

### 5.1 状态与事件定义

```c
typedef enum { MOTOR_STOP = 0, MOTOR_FORWARD, MOTOR_BACKWARD } motor_state_t;
typedef enum { EVT_NONE = 0, EVT_KEY0, EVT_KEY1, EVT_KEY2 }     motor_event_t;
```

| 状态 | 含义 | KEY0 | KEY1 | KEY2 |
| --- | --- | --- | --- | --- |
| `MOTOR_STOP` | 停止 | 正转启动（+=400） | 反转启动（-=400） | 维持停止 + LED1 翻转 |
| `MOTOR_FORWARD` | 正转 | 加速 | 减速，跨过 0 点后进反转 | 停止 + LED1 翻转 |
| `MOTOR_BACKWARD` | 反转 | 减速，跨过 0 点后进正转 | 加速 | 停止 + LED1 翻转 |

### 5.2 主循环（事件分发）

```c
while (1)
{
    uint8_t key = key_scan(0);
    motor_event_t evt = EVT_NONE;
    if (key == KEY0_PRES)      evt = EVT_KEY0;
    else if (key == KEY1_PRES) evt = EVT_KEY1;
    else if (key == KEY2_PRES) evt = EVT_KEY2;

    motor_fsm_event(evt);      /* 交给状态机处理 */

    delay_ms(10);
    t++;
    if (t % 20 == 0) LED0_TOGGLE();   /* 心跳灯 */
}
```

### 5.3 状态机核心函数

- `motor_state_of(pwm)`：由 PWM 比较值符号推导状态（正 → 正转，负 → 反转，零 → 停止）。
- `motor_apply_pwm(pwm)`：限幅 → 推导状态 → 启/停电机 → 设置占空比，是控制动作的汇聚点。
- `motor_fsm_event(evt)`：根据「当前状态 + 事件」计算新的比较值并转移状态。

采用状态机的优势：状态显式可读、方便扩展（如加刹车/故障/闭环）、限幅逻辑集中、主循环稳定、便于调试与测试。

## 6. 实验现象

1. LED0 指示程序运行，约 200 ms 翻转一次。
2. 按键 K0：增大 PWM 比较值，电机**正转并加速**。
3. 按键 K1：减小 PWM 比较值，电机**减速**；比较值为正时正转，为负时反转。
4. 按键 K2：**停止电机**，同时翻转 LED1。
5. LCD 屏显示按键功能信息。

## 7. 关键接口

| 接口 | 作用 |
| --- | --- |
| `atim_timx_cplm_pwm_init(arr, psc)` | 初始化 TIM1 互补 PWM 输出 |
| `motor_pwm_set(float para)` | 统一设置电机转向与速度（正/负号决定方向） |
| `dcmotor_dir(uint8_t)` | 设置方向（0 正转，1 反转） |
| `dcmotor_speed(uint16_t)` | 设置速度（比较值） |
| `dcmotor_start()` / `dcmotor_stop()` | 使能/关断电机输出 |
| `dcmotor_init()` | 电机初始化（配置 SHDN 引脚、恢复停止态） |

## 8. 注意事项

- 通电前务必检查电源接线，电源电压须按上述要求连接。
- 注意电机旋转/停止可能产生的震动与安全风险。
- 驱动板供电为 12~60 V（10 A），开发板供电为 12~24 V（1 A），不可接反。
- 开发板与驱动板通过 24P 排线连接，默认使用接口 1。

---

> 资料参考：正点原子（ALIENTEK）F407 电机开发板教程，实验 6.1 直流有刷电机基础驱动。
