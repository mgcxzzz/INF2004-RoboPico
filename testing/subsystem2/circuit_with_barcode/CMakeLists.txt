cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w CACHE STRING "Target board")
if(NOT DEFINED PICO_SDK_PATH)
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif()
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(buddy23_circuit_with_barcode_v11 C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()
add_executable(buddy23_circuit_with_barcode_v11 main.c)
target_link_libraries(buddy23_circuit_with_barcode_v11 pico_stdlib hardware_adc hardware_pwm hardware_clocks hardware_sync hardware_flash m)
target_compile_options(buddy23_circuit_with_barcode_v11 PRIVATE -Wall -Wextra -Werror)
pico_enable_stdio_uart(buddy23_circuit_with_barcode_v11 0)
pico_enable_stdio_usb(buddy23_circuit_with_barcode_v11 1)
target_compile_definitions(buddy23_circuit_with_barcode_v11 PRIVATE
    LINE_CAL_MIN_SPAN=50
    CIRCUIT_SWEEP_SEARCH=1
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=5000000 PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0)
pico_add_extra_outputs(buddy23_circuit_with_barcode_v11)
