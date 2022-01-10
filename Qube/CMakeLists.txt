cmake_minimum_required(VERSION 3.12)

add_executable(console_out-threadx.elf
  Src/main.c
  Src/app_threadx.c
  Src/app_azure_rtos.c
  #Src/logging.c
  Src/stm32u5xx_hal_timebase_tim_template.c
  Src/usart.c
  Src/console.c
  Src/stm32u5xx_it.c
)

target_include_directories(console_out-threadx.elf PUBLIC
  Inc/
)

target_link_libraries(console_out-threadx.elf LINK_PUBLIC ST_Code twilio-microvisor-hal-stm32u5 threadx)

# Optional informational and additional format generation
add_custom_command(OUTPUT EXTRAS
  DEPENDS console_out-threadx.elf
  COMMAND ${CMAKE_SIZE} --format=berkeley "console_out-threadx.elf"
  COMMAND ${CMAKE_OBJDUMP} -h -S "console_out-threadx.elf" > "console_out-threadx.list"
  COMMAND ${CMAKE_OBJCOPY} -O binary "console_out-threadx.elf" "console_out-threadx.bin"
)

add_custom_target(extras ALL
  ${CMAKE_COMMAND} -E echo "Outputting additional artifacts"
  DEPENDS EXTRAS
)
