# 项目总结：直流有刷电机速度+位置双环PID控制（Exp6_8）

## 1. 项目概述

| 项目 | 说明 |
| --- | --- |
| 实验名称 | 直流有刷电机 速度+位置双环PID 控制实验 |
| 实验平台 | 正点原子 F407 电机开发板（STM32F407IG） |
| 实验目的 | 学习直流有刷电机的**速度环（内环）+ 位置环（外环）双闭环PID控制** |
| 开发环境 | STM32 HAL 库 + 正点原子 SYSTEM/BSP 分层工程，主时钟 168 MHz |

本工程演示如何用一路高档定时器（TIM1）输出互补 PWM 驱动直流有刷电机，TIM3 编码器采集电机位置反馈，TIM6 定时采样做测速与双环 PID 运算，最终让电机稳定运行在设定的位置。按键与上位机 PID 调试助手均可修改目标位置。

## 2. 硬件资源与引脚分配

| 功能 | 外设 / 引脚 | 说明 |
| --- | --- | --- |
| 运行指示灯 | LED0 (DS0, 红色) - PE0 | 指示程序运行，约 200 ms 翻转一次 |
| 电机 PWM 主通道 | TIM1_CH1 - PA8 | 驱动板 IN1，PWM 主输出 |
| 电机 PWM 互补通道 | TIM1_CH1N - PB13 | 驱动板 IN2，PWM 互补输出（实现转向） |
| 电机停止引脚 | PF10 | 驱动板 SHDN，关闭/开启输出 |
| 编码器 A 相 | TIM3_CH1 - PC6 | 驱动板 ENCA |
| 编码器 B 相 | TIM3_CH2 - PC7 | 驱动板 ENCB |
| LCD | 2.8/3.5/4.3/7/10 寸 TFTLCD（16 位 8080 并口） | 显示占空比、目标位置、实际位置、速度 |
| 串口 | USART1（PB6/PB7，板载 CH340），115200 bps | 上位机 PID 调试助手通信 |
| 按键 | KEY0 / KEY1 / KEY2（PE2 / PE3 / PE4） | 启动正转角 / 启动反转角 / 回零 |
| ADC | 电压 / 电流 / 温度采集 | 监测母线电压、电机电流与温度 |

### 电源要求

- 驱动板供电：12 ~ 60 V，10 A 稳压（或可调）电源。
- 开发板供电：12 ~ 24 V，1 A 直流电源（DC 接口）。
- 开发板与驱动板通过 24P 排线连接，默认使用**接口 1**。

## 3. 软件架构

工程使用 HAL 库 + 正点原子 SYSTEM/BSP 分层结构，核心目录如下：

```
Exp6_8_BrushedDCMotor_SpeedPositionDualLoop/
├── User/
│   └── main.c                   # 主程序（含电机状态机）
├── Middlewares/DEBUG/           # PID 上位机调试（串口协议解析/上报）
├── Drivers/BSP/                 # 板级外设驱动
│   ├── TIMER/dcmotor_tim.c/h    # TIM1 互补PWM、TIM3 编码器、TIM6 定时采样
│   ├── DC_MOTOR/dc_motor.c/h    # 直流有刷电机控制、速度计算
│   ├── PID/pid.c/h              # 双环 PID 算法
│   ├── KEY/                     # 按键扫描
│   ├── LED/                     # LED 控制
│   ├── LCD/                     # LCD 显示
│   └── ADC/                     # 电压/电流/温度采集
├── Drivers/SYSTEM/              # sys 时钟、usart、delay
└── Drivers/STM32F4xx_HAL_Driver/ # HAL 库
```

### 各 BSP 模块职责

| 模块 | 职责 |
| --- | --- |
| `dcmotor_tim` | 配置 TIM1 互补 PWM（PWM1 模式、死区 0x0F）、TIM3 编码器接口、TIM6 定时中断；在 `HAL_TIM_PeriodElapsedCallback` 中完成测速与双环 PID |
| `dc_motor` | 电机启停、方向、速度设置；`motor_pwm_set()` 统一封装转向与调速；`speed_computer()` 中值滤波 + 一阶低通测速 |
| `pid` | 增量式/位置式 PID，双环各一个 `PID_TypeDef` 实例 |
| `KEY` | 扫描 KEY0/KEY1/KEY2，返回按键事件 |
| `LED` | LED 点亮/熄灭/翻转 |
| `LCD` | 屏显运行数据 |
| `DEBUG` | 与上位机 PID 助手通信：接收 PID 参数与运行指令，上传状态/速度/编码器/波形 |

## 4. 核心控制原理

### 4.1 双环 PID 结构

```
目标位置 ─▶[位置环 PID]─▶ 目标速度 ─▶[速度环 PID]─▶ PWM占空比 ─▶ 电机 ─▶ 编码器 ─▶ 实际位置
                                     ▲                                          │
                                     └──────────── 实际速度（编码器测速）─────────┘
```

- **外环（位置环 PID）**：输入目标位置与实际位置，输出目标速度，参数 `L_KP / L_KI / L_KD`。
- **内环（速度环 PID）**：输入目标速度与实际速度，输出 PWM 占空比，参数 `S_KP / S_KI / S_KD`。
- 算法采用**增量式 PID**（`pid.h` 中 `INCR_LOCT_SELECT = 1`）。

### 4.2 定时器分工

| 定时器 | 模式 | 作用 |
| --- | --- | --- |
| TIM1 | 互补 PWM 带死区 | 输出 PA8 / PB13 互补 PWM，驱动 H 桥；`arr=8399, psc=0`，周期约 50 µs |
| TIM3 | 编码器接口 (TI12) | 4 倍频计数，采集编码器送位置反馈；更新中断处理溢出计数 |
| TIM6 | 定时中断 (1 ms) | 每隔 5 ms 测速、每 50 ms 计算一次双环 PID |

### 4.3 转速计算

编码器一圈计数 = 线数 × 倍频 = `ROTO_RATIO(44)`，减速比 `REDUCTION_RATIO(30)`。

```
转速(RPM) = 编码器计数值变化量 × (1000 / ms) × 60 / 减速比 / 线数
```

测速数据经 **10 次中值滤波**（去高低）+ **一阶低通滤波**（0.48 / 0.52 系数）平滑。

## 5. 程序流程与状态机

主循环已重构为**无限状态机（FSM）**，把原来平铺的 `if/else` 按键与命令处理显式化，逻辑更清晰、易于扩展。

### 5.1 状态定义

```c
typedef enum
{
    MOTOR_FSM_IDLE = 0,   /* 空闲：PID闭环不使能，等待启动 */
    MOTOR_FSM_RUN,        /* 运行：位置环+速度环双闭环使能 */
} motor_fsm_state_t;

static motor_fsm_state_t g_motor_fsm = MOTOR_FSM_IDLE;
```

### 5.2 状态转移

| 当前状态 | 事件 | 动作 | 下一状态 |
| --- | --- | --- | --- |
| IDLE | KEY0 | 目标位置 +1320（正转一圈），使能闭环 | RUN |
| IDLE | KEY1 | 目标位置 -1320（反转一圈），使能闭环 | RUN |
| IDLE | KEY2 | 目标位置置 0 | IDLE |
| RUN | KEY0 / KEY1 | 目标位置 ±1320（限幅 ±6600） | RUN |
| RUN | KEY2 / 上位机 HALT | 目标位置置 0，闭环回零 | RUN |
| IDLE/RUN | 上位机 BREAKED | 关PID、PWM清零、停电机、目标复位 | IDLE |
| IDLE/RUN | 上位机 RUN_CODE | 目标位置置 1320，使能闭环 | RUN |

> 目标位置限幅：`LOC_STEP_MAX = 6600`（正转 5 圈），`LOC_STEP_MIN = -6600`（反转 5 圈），每按一次步进 `LOC_STEP_ONCE = 1320`（一圈）。

### 5.3 主循环（事件分发）

```c
while (1)
{
    uint8_t key = key_scan(0);

    switch (g_motor_fsm)          /* 状态机：按键输入处理 */
    {
        case MOTOR_FSM_IDLE: ...  break;
        case MOTOR_FSM_RUN:   ...  break;
    }

    /* 上位机指令处理（PID参数、运行/停机/刹车） */
    ...

    /* 周期任务：tick++，每 20 次（约 200ms）刷LCD、翻转LED、上报数据 */
    delay_ms(10);
}
```

### 5.4 中断：双环 PID 运算

TIM6 定时（1 ms）进入 `HAL_TIM_PeriodElapsedCallback`：每 5 ms 测速；当 `g_run_flag = 1` 时，每 50 ms 执行：

1. 读取编码器实际位置 -> 位置环 PID 输出目标速度（限幅 ±150）；
2. 速度环 PID 输出 PWM 占空比（限幅 ±8200）；
3. `motor_pwm_set()` 设置占空比与转向。

## 6. 实验现象

1. LED0 指示程序运行，约 200 ms 翻转一次。
2. 按下 KEY0：目标位置 +1320，电机正转到设定位置；按下 KEY1：目标位置 -1320，电机反转到设定位置；按下 KEY2：目标位置回 0，电机回到初始位置。
3. LCD 显示 PWM 占空比、目标位置、实际位置及实际速度。
4. 上位机 PID 调试助手可同步/修改双环 PID 参数，下发运行、停机、刹车指令，并实时显示速度与编码器波形。

## 7. 关键接口

| 接口 | 作用 |
| --- | --- |
| `atim_timx_cplm_pwm_init(arr, psc)` | 初始化 TIM1 互补 PWM 输出 |
| `gtim_timx_encoder_chy_init(arr, psc)` | 初始化 TIM3 编码器接口 |
| `btim_timx_int_init(arr, psc)` | 初始化 TIM6 定时中断（采样周期） |
| `dcmotor_init()` | 电机初始化（SHDN 引脚、方向、速度） |
| `motor_pwm_set(float para)` | 统一设置转向与速度（正/负号决定方向） |
| `dcmotor_start()` / `dcmotor_stop()` | 使能/关闭电机输出 |
| `increment_pid_ctrl(PID, feedback)` | 增量式 PID 闭环计算 |
| `speed_computer(encode_now, ms)` | 编码器测速 + 滤波 |
| `debug_receive_pid()` / `debug_receive_ctrl_code()` | 上位机 PID 参数与控制指令接收 |
| `debug_send_motorstate()` / `debug_send_speed()` | 状态与速度上报 |

## 8. 注意事项

1. 电源电压需按照上文要求连接，当心电机旋转和停止造成的震动。
2. 串口 1 波特率固定为 115200 bps，PID 调试助手需匹配。
3. 双环控制时，**外环 PID 参数调节幅度不要太大**，对整条曲线影响很大。
4. 本例程仅支持 MCU 屏（16 位 8080 并口），不支持 RGB 屏。
5. 目标位置受 ±6600（约 5 圈）限幅，避免超出机械行程。

---

> 参考资料：正点原子（ALIENTEK）F407 电机开发板教程，实验 6.8 直流有刷电机速度+位置双环控制。
