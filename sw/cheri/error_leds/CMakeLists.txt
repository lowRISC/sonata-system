set(NAME error_leds)

add_executable(${NAME} "error.S")
target_include_directories(${NAME} PRIVATE "${CHERIOT_RTOS_SDK}/include")

add_custom_command(
  TARGET ${NAME} POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:${NAME}>" "$<TARGET_FILE:${NAME}>.bin"
  COMMAND srec_cat "$<TARGET_FILE:${NAME}>.bin" -binary -offset 0x0000 -byte-swap 4 -o "$<TARGET_FILE:${NAME}>.vmem" -vmem
  VERBATIM
)
