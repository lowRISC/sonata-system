add_library(common OBJECT sonata_system.c uart.c timer.c rv_plic.c gpio.c i2c.c pwm.c spi.c crt0.S)
target_include_directories(common INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
