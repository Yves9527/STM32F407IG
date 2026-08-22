cmake_minimum_required(VERSION 3.20)

# 设置交叉编译工具链
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_SIZE arm-none-eabi-size)

project(atk_f407 C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 14)

# MCU 架构与 FPU 编译选项
set(CPU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the build type" FORCE)
endif()

add_compile_options(
    ${CPU_FLAGS}
    -Wall -Wextra
    -fdata-sections -ffunction-sections
    $<$<CONFIG:Debug>:-Og>
    $<$<CONFIG:Debug>:-g>
    $<$<CONFIG:Debug>:-gdwarf-2>
    $<$<CONFIG:Release>:-Os>
    $<$<CONFIG:RelWithDebInfo>:-Os>
    $<$<CONFIG:RelWithDebInfo>:-g>
    $<$<CONFIG:RelWithDebInfo>:-gdwarf-2>
)

add_link_options(
    ${CPU_FLAGS}
    -T${CMAKE_SOURCE_DIR}/STM32F407IGTx_FLASH.ld
    -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map,--cref
    -Wl,--gc-sections
    --specs=nano.specs
    --specs=nosys.specs
)

# 宏定义 
add_compile_definitions(
    USE_HAL_DRIVER
    STM32F407xx
)

# 头文件包含路径 (对齐 Drivers 根搜索)
include_directories(
    User
    Drivers
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/SYSTEM/delay
    Drivers/SYSTEM/sys
    Drivers/SYSTEM/usart
    Drivers/BSP/LED
    Drivers/BSP/LCD
)

# 1. User 源文件
set(USER_SRCS
    User/main.c
    User/stm32f4xx_it.c
)

# 2. SYSTEM 源文件
set(SYSTEM_SRCS
    Drivers/SYSTEM/delay/delay.c
    Drivers/SYSTEM/sys/sys.c
    Drivers/SYSTEM/usart/usart.c
)

# 3. BSP 源文件
set(BSP_SRCS
    Drivers/BSP/LED/led.c
    Drivers/BSP/LCD/lcd.c
)

# 4. 精确对齐原 Keil 工程引用的 HAL/LL 驱动文件
set(HAL_SRCS
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_usart.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sram.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_fsmc.c
)

# 5. 启动文件与系统初始化
set(STARTUP_SRCS
    Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407xx.s
    Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c
)

# 确保汇编文件使用 C 预处理器
set_source_files_properties(
    Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f407xx.s
    PROPERTIES LANGUAGE ASM
    COMPILE_FLAGS "-x assembler-with-cpp"
)

add_executable(${PROJECT_NAME}.elf
    ${USER_SRCS}
    ${HAL_SRCS}
    ${SYSTEM_SRCS}
    ${BSP_SRCS}
    ${STARTUP_SRCS}
)

# 编译完成后输出大小并生成 hex/bin
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin
)