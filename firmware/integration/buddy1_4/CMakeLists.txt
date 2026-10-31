cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w CACHE STRING "Target board")
if(NOT DEFINED PICO_SDK_PATH)
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif()
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
project(robopico_integration C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()
option(BUDDY_WIFI_ENABLED "Build the optional Pico W core1 Wi-Fi/MQTT transport" ON)

add_executable(robopico_integration
    src/app/main.c
    src/platform/motor_output.c
    src/navigation/motion_api.c
    src/navigation/nav_coordinator.c
    src/motion/motion_executor.c
    src/navigation/nav_codec.c
    src/app/buddy_hooks.c
    src/network/wifi_transport.c
    src/network/wifi_protocol.c
    src/network/wifi_packets.c
    src/imu/imu_service.c
    src/imu/lsm303_driver.c
    src/imu/imu_proc.c
    src/imu/b4_pack.c
    src/telemetry/b2_pack.c
    src/telemetry/sys_tlm.c)
# Include the complete release version in generated ELF/BIN/UF2 filenames.
# ramp-assist v0.6 (v0.4 + speed hold: min 35 mm/s per wheel, I limit 300, 3x Ki until flat; replaces the v0.5 300 PWM floor) on top of the Buddy4 add-on v0.1: two variants from one tree.
#   -DRAMP_ASSIST_ACTIVE=ON (default, hardware-tested): ramp assist + stall push + speed hold applied
#   -DRAMP_ASSIST_ACTIVE=OFF          : shadow, logic + telemetry only, motors unchanged
option(RAMP_ASSIST_ACTIVE "Apply the ramp assist (OFF = shadow logging only)" ON)
if(RAMP_ASSIST_ACTIVE)
    set(RA_VARIANT active)
    set(RA_ACTIVE_VALUE 1)
else()
    set(RA_VARIANT shadow)
    set(RA_ACTIVE_VALUE 0)
endif()
# Combined build: + barcode peak decoder v0.1 (from interface_wifi_v2_3_barcode_peak_v0_1).
#   -DBARCODE_PEAK_ACTIVE=ON (default, hardware-tested): peak decoder may queue letters the original decoder missed
#   -DBARCODE_PEAK_ACTIVE=OFF          : peak decoder logs only (shadow)
option(BARCODE_PEAK_ACTIVE "Let the barcode peak decoder queue letters (OFF = shadow logging only)" ON)
if(BARCODE_PEAK_ACTIVE)
    set(BP_VARIANT active)
    set(BP_ACTIVE_VALUE 1)
else()
    set(BP_VARIANT shadow)
    set(BP_ACTIVE_VALUE 0)
endif()
set_target_properties(robopico_integration PROPERTIES OUTPUT_NAME robopico_buddy1_4_integration_v0_1)
target_compile_definitions(robopico_integration PRIVATE BUDDY4_ADDON=1 RAMP_ASSIST_ACTIVE=${RA_ACTIVE_VALUE}
    BARCODE_PEAK_ACTIVE=${BP_ACTIVE_VALUE} BARCODE_PEAK_COMBINED=1)
target_include_directories(robopico_integration PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${CMAKE_CURRENT_LIST_DIR}/config"
    "${CMAKE_CURRENT_LIST_DIR}/include/interfaces")
target_link_libraries(robopico_integration pico_stdlib hardware_adc hardware_pwm hardware_clocks hardware_sync hardware_flash pico_multicore m)
if(BUDDY_WIFI_ENABLED)
    if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/config/wifi_config.local.h")
        target_compile_definitions(robopico_integration PRIVATE BUDDY_WIFI_LOCAL_CONFIG=1)
    else()
        target_compile_definitions(robopico_integration PRIVATE BUDDY_WIFI_LOCAL_CONFIG=0)
    endif()
    target_compile_definitions(robopico_integration PRIVATE BUDDY_WIFI_ENABLED=1)
    target_link_libraries(robopico_integration pico_cyw43_arch_lwip_poll pico_lwip_mqtt)
else()
    target_compile_definitions(robopico_integration PRIVATE BUDDY_WIFI_ENABLED=0)
endif()
target_compile_options(robopico_integration PRIVATE -Wall -Wextra -Werror)
pico_enable_stdio_uart(robopico_integration 0)
pico_enable_stdio_usb(robopico_integration 1)
target_compile_definitions(robopico_integration PRIVATE
    BUDDY_RUNTIME_MAPPING=1 BUDDY_WIFI_LOGGING=1 BUDDY_COORDINATED_BARCODE=1
    LINE_CAL_MIN_SPAN=50
    CIRCUIT_SWEEP_SEARCH=1
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=5000000 PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0)
option(INTERFACE_UNPACKED_REFERENCE "Measurement-only uncompressed interface state" OFF)
if(INTERFACE_UNPACKED_REFERENCE)
    target_compile_definitions(robopico_integration PRIVATE INTERFACE_UNPACKED_REFERENCE=1)
endif()
set(BUDDY_EXTENSION_SOURCE "" CACHE FILEPATH "Optional Buddy5 extension with strong buddy_extension hooks")
if(BUDDY_EXTENSION_SOURCE)
    target_sources(robopico_integration PRIVATE "${BUDDY_EXTENSION_SOURCE}")
endif()
# RP2040: reserve the top 16 KiB of main SRAM for two disjoint 8 KiB stacks.
# SDK default stack regions are too small for the inherited driving call chain.
# Override SDK memory-region symbols so BSS/heap also stop below BOTH stacks.
if(NOT PICO_RP2040)
    message(FATAL_ERROR "This firmware memory layout requires RP2040 / Pico W")
endif()
target_compile_definitions(robopico_integration PRIVATE
    PICO_STACK_SIZE=8192 PICO_CORE1_STACK_SIZE=8192)
target_link_options(robopico_integration PRIVATE
    "LINKER:--defsym=RAM_LENGTH=0x3c000"
    "LINKER:--defsym=SCRATCH_X_ORIGIN=0x2003c000"
    "LINKER:--defsym=SCRATCH_X_LENGTH=8192"
    "LINKER:--defsym=SCRATCH_Y_ORIGIN=0x2003e000"
    "LINKER:--defsym=SCRATCH_Y_LENGTH=8192")
# Retain compiler frame measurements for the actual optimized application.
set_source_files_properties(src/app/main.c PROPERTIES COMPILE_OPTIONS "-fstack-usage")
pico_add_extra_outputs(robopico_integration)
