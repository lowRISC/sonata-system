set(NAME common)
add_library(${NAME} OBJECT hyperram_exec_test.S boot.S)
target_include_directories(${NAME} PRIVATE ${CHERIOT_SDK_INCLUDES})
