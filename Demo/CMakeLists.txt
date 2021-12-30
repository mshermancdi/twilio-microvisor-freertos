cmake_minimum_required(VERSION 3.12)

add_executable(gpio_toggle_demo-nortos.elf
  Src/main.c
  Src/logging.c
  Src/stm32u5xx_hal_timebase_tim_template.c
)

target_include_directories(gpio_toggle_demo-nortos.elf PUBLIC
  Inc/
)

target_link_libraries(gpio_toggle_demo-nortos.elf LINK_PUBLIC ST_Code twilio-microvisor-hal-stm32u5)

# Optional informational and additional format generation
add_custom_command(OUTPUT EXTRAS
  DEPENDS gpio_toggle_demo-nortos.elf
  COMMAND ${CMAKE_SIZE} --format=berkeley "gpio_toggle_demo-nortos.elf"
  COMMAND ${CMAKE_OBJDUMP} -h -S "gpio_toggle_demo-nortos.elf" > "gpio_toggle_demo-nortos.list"
  COMMAND ${CMAKE_OBJCOPY} -O binary "gpio_toggle_demo-nortos.elf" "gpio_toggle_demo-nortos.bin"
)

add_custom_target(extras ALL
  ${CMAKE_COMMAND} -E echo "Outputting additional artifacts"
  DEPENDS EXTRAS
)
