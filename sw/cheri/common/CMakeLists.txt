add_library(common OBJECT boot.S end.S)
target_include_directories(common PRIVATE "${CHERIOT_RTOS_SDK}/include")

