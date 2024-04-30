add_library(lcd_st7735_lib
${CMAKE_CURRENT_SOURCE_DIR}/../../../../vendor/display_drivers/core/lcd_base.c
${CMAKE_CURRENT_SOURCE_DIR}/../../../../vendor/display_drivers/core/lucida_console_10pt.c
${CMAKE_CURRENT_SOURCE_DIR}/../../../../vendor/display_drivers/core/lucida_console_12pt.c
${CMAKE_CURRENT_SOURCE_DIR}/../../../../vendor/display_drivers/st7735/lcd_st7735.c
)

# add_executable(lcd_st7735 main.c)
add_executable(lcd_st7735 main.c lcd.c fractal_fixed.c fractal_float.c fractal_palette.c)

# pull in core dependencies and additional i2c hardware support
target_link_libraries(lcd_st7735 common lcd_st7735_lib)

target_include_directories(lcd_st7735 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../../../vendor/display_drivers)

add_custom_command(
  TARGET lcd_st7735 POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:lcd_st7735>" "$<TARGET_FILE:lcd_st7735>.bin"
  COMMAND srec_cat "$<TARGET_FILE:lcd_st7735>.bin" -binary -offset 0x0000 -byte-swap 4 -o "$<TARGET_FILE:lcd_st7735>.vmem" -vmem
  VERBATIM)
