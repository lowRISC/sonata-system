add_executable(hello_usb main.c)

# pull in core dependencies and additional usbdev hardware support
target_link_libraries(hello_usb common)

add_custom_command(
  TARGET hello_usb POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary "$<TARGET_FILE:hello_usb>" "$<TARGET_FILE:hello_usb>.bin"
  COMMAND srec_cat "$<TARGET_FILE:hello_usb>.bin" -binary -offset 0x0000 -byte-swap 4 -o "$<TARGET_FILE:hello_usb>.vmem" -vmem
  VERBATIM)
