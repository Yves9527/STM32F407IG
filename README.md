# ATK-DMF407 (STM32F407IGT6) VS Code 开发工程

本项目是针对 **正点原子 ATK-DMF407 工业 & 运动控制电机开发板**（主控：`STM32F407IGT6`） 适配的 **VS Code + CMake + Ninja + GCC** 嵌入式开发工程。

摆脱 Keil MDK / IAR 商业环境限制，基于现代开源工具链实现极速编译、代码静态分析与在线单步调试。

---

## 🛠️ 硬件平台特性

* **主控芯片**：STM32F407IGT6（Cortex-M4F 内核，主频 168MHz，1024KB Flash，192KB SRAM）[cite: 1]
* **电机接口**：
  * 2 路全功能直流有刷 / 无刷驱动器接口（支持双路 FOC）[cite: 1]
  * 4 路光耦隔离步进电机驱动器接口[cite: 1]
  * 3 路带电平转换舵机接口[cite: 1]
  * 2 路独立编码器接口[cite: 1]
* **工业通信**：CAN 总线、RS485、RS232、以太网（LAN8720A / RMII）[cite: 1]
* **存储外设**：16MB SPI Flash、2Kb EEPROM、TFT-LCD 接口（FSMC）[cite: 1]

---

## 🧰 工具链与构建环境

* **Compiler**：`arm-gnu-toolchain` (`arm-none-eabi-gcc`)
* **Build System**：`CMake` + `Ninja`（极速多线程构建）
* **Debugger**：`xPack OpenOCD` + VS Code `Cortex-Debug`

---
