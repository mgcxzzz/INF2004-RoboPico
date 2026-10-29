cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w CACHE STRING "Target board")
if(NOT DEFINED PICO_SDK_PATH)
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif()
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(buddy123_interface_wifi_v2 C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()
option(BUDDY_WIFI_ENABLED "Build the optional Pico W core1 Wi-Fi/MQTT transport" ON)

add_executable(buddy123_interface_wifi_v2 main.c motion_api.c nav_coordinator.c motion_executor.c nav_codec.c buddy_hooks.c wifi_transport.c wifi_protocol.c wifi_packets.c)
# Include the complete release version in generated ELF/BIN/UF2 filenames.
set_target_properties(buddy123_interface_wifi_v2 PROPERTIES OUTPUT_NAME buddy123_interface_wifi_v2_3_barcode_fine_v0_3)
target_include_directories(buddy123_interface_wifi_v2 PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
target_link_libraries(buddy123_interface_wifi_v2 pico_stdlib hardware_adc hardware_pwm hardware_clocks hardware_sync hardware_flash pico_multicore m)
if(BUDDY_WIFI_ENABLED)
    if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/wifi_config.local.h")
        target_compile_definitions(buddy123_interface_wifi_v2 PRIVATE BUDDY_WIFI_LOCAL_CONFIG=1)
    else()
        target_compile_definitions(buddy123_interface_wifi_v2 PRIVATE BUDDY_WIFI_LOCAL_CONFIG=0)
    endif()
    target_compile_definitions(buddy123_interface_wifi_v2 PRIVATE BUDDY_WIFI_ENABLED=1)
    target_link_libraries(buddy123_interface_wifi_v2 pico_cyw43_arch_lwip_poll pico_lwip_mqtt)
else()
    target_compile_definitions(buddy123_interface_wifi_v2 PRIVATE BUDDY_WIFI_ENABLED=0)
endif()
target_compile_options(buddy123_interface_wifi_v2 PRIVATE -Wall -Wextra -Werror)
pico_enable_stdio_uart(buddy123_interface_wifi_v2 0)
pico_enable_stdio_usb(buddy123_interface_wifi_v2 1)
target_compile_definitions(buddy123_interface_wifi_v2 PRIVATE
    BUDDY_RUNTIME_MAPPING=1 BUDDY_WIFI_LOGGING=1
    LINE_CAL_MIN_SPAN=50
    CIRCUIT_SWEEP_SEARCH=1
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=5000000 PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0)
option(INTERFACE_UNPACKED_REFERENCE "Measurement-only uncompressed interface state" OFF)
if(INTERFACE_UNPACKED_REFERENCE)
    target_compile_definitions(buddy123_interface_wifi_v2 PRIVATE INTERFACE_UNPACKED_REFERENCE=1)
endif()
set(BUDDY_EXTENSION_SOURCE "" CACHE FILEPATH "Optional Buddy5 extension with strong buddy_extension hooks")
if(BUDDY_EXTENSION_SOURCE)
    target_sources(buddy123_interface_wifi_v2 PRIVATE "${BUDDY_EXTENSION_SOURCE}")
endif()
# RP2040: reserve the top 16 KiB of main SRAM for two disjoint 8 KiB stacks.
# SDK default stack regions are too small for the inherited driving call chain.
# Override SDK memory-region symbols so BSS/heap also stop below BOTH stacks.
if(NOT PICO_RP2040)
    message(FATAL_ERROR "This firmware memory layout requires RP2040 / Pico W")
endif()
target_compile_definitions(buddy123_interface_wifi_v2 PRIVATE
    PICO_STACK_SIZE=8192 PICO_CORE1_STACK_SIZE=8192)
target_link_options(buddy123_interface_wifi_v2 PRIVATE
    "LINKER:--defsym=RAM_LENGTH=0x3c000"
    "LINKER:--defsym=SCRATCH_X_ORIGIN=0x2003c000"
    "LINKER:--defsym=SCRATCH_X_LENGTH=8192"
    "LINKER:--defsym=SCRATCH_Y_ORIGIN=0x2003e000"
    "LINKER:--defsym=SCRATCH_Y_LENGTH=8192")
# Retain compiler frame measurements for the actual optimized application.
set_source_files_properties(main.c PROPERTIES COMPILE_OPTIONS "-fstack-usage")
pico_add_extra_outputs(buddy123_interface_wifi_v2)
