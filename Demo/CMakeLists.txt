cmake_minimum_required(VERSION 3.12)

add_executable(gpio_toggle_demo-threadx.elf
  Src/main.c
  Src/app_threadx.c
  Src/app_azure_rtos.c
  Src/logging.c
  Src/stm32u5xx_hal_timebase_tim_template.c
)

target_include_directories(gpio_toggle_demo-threadx.elf PUBLIC
  Inc/
)

target_link_libraries(gpio_toggle_demo-threadx.elf LINK_PUBLIC ST_Code twilio-microvisor-hal-stm32u5 threadx)

# Optional informational and additional format generation
add_custom_command(OUTPUT EXTRAS
  DEPENDS gpio_toggle_demo-threadx.elf
  COMMAND ${CMAKE_SIZE} --format=berkeley "gpio_toggle_demo-threadx.elf"
  COMMAND ${CMAKE_OBJDUMP} -h -S "gpio_toggle_demo-threadx.elf" > "gpio_toggle_demo-threadx.list"
  COMMAND ${CMAKE_OBJCOPY} -O binary "gpio_toggle_demo-threadx.elf" "gpio_toggle_demo-threadx.bin"
)

add_custom_target(extras ALL
  ${CMAKE_COMMAND} -E echo "Outputting additional artifacts"
  DEPENDS EXTRAS
)
