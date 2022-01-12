cmake_minimum_required(VERSION 3.12)

set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/toolchain.cmake")

project(console_out-threadx.elf C ASM)

include_directories(include
                    ${twilio-microvisor-hal-stm32u5_INCLUDE_DIRS})

set(INCLUDED_HAL_FILES
  Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal.c
  Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cortex.c
  Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma.c
  Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma_ex.c
  Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_exti.c
  Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash.c
  Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash_ex.c
  Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gpio.c
  Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gtzc.c
  Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_i2c.c
  Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_i2c_ex.c
  Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_icache.c
  Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr.c
  Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr_ex.c
  Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc.c
  Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc_ex.c
  Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tim.c
  Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tim_ex.c
  Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_uart.c
  Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_uart_ex.c
  Drivers/CMSIS/Device/ST/STM32U5xx/Source/Templates/system_stm32u5xx_ns.c
  Drivers/CMSIS/Device/ST/STM32U5xx/Source/Templates/gcc/startup_stm32u585xx.s
)
set(CONFIG_DIRECTORY
  "${CMAKE_SOURCE_DIR}/Config"
)
add_subdirectory(twilio-microvisor-hal-stm32u5)

add_library(ST_Code STATIC
  ST_Code/Core/Src/syscalls.c ST_Code/Core/Src/sysmem.c
  ST_Code/Core/Src/stm32u5xx_hal_msp.c  
)

target_include_directories(ST_Code PUBLIC  
  ST_Code/Core/Inc
)


set(THREADX_ARCH "cortex_m33")
set(THREADX_TOOLCHAIN "gnu")

add_subdirectory(threadx)

target_link_libraries(ST_Code LINK_PUBLIC twilio-microvisor-hal-stm32u5 threadx)

add_subdirectory(Qube)

unset(CONFIG_DIRECTORY)
