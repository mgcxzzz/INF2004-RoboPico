cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w CACHE STRING "Target board")
if(NOT DEFINED PICO_SDK_PATH)
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif()
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(buddy2_left_motor_bench C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()
add_executable(buddy2_left_motor_bench single_wheel_v3.c)
target_link_libraries(buddy2_left_motor_bench pico_stdlib hardware_pwm hardware_clocks hardware_sync)
target_compile_options(buddy2_left_motor_bench PRIVATE -Wall -Wextra -Werror)
# GP0/GP1 belong to the left encoder. Never enable UART stdio here.
pico_enable_stdio_uart(buddy2_left_motor_bench 0)
pico_enable_stdio_usb(buddy2_left_motor_bench 1)
target_compile_definitions(buddy2_left_motor_bench PRIVATE
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=10000
    PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0)
pico_add_extra_outputs(buddy2_left_motor_bench)
