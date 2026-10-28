################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

# app_program/*.c plus one level of subfolders (one per subsystem)
# Isolated integration: the proven controller/coordinator replaces template
# Producer/Consumer and legacy navigation tasks. Only these providers run.
CONTROLLER = ../app_program/subsystem2/controller
TEMP_SRCS = ../app_program/app_main.c ../app_program/system/sys_hal.c \
 ../app_program/subsystem1/drive_bridge.c \
 ../app_program/subsystem2/utk_hal.c ../app_program/subsystem2/utk_hal_kernel.c ../app_program/subsystem2/utk_console_format.c ../app_program/subsystem2/utk_fault.c ../app_program/subsystem2/drive_task.c \
 $(addprefix $(CONTROLLER)/,motion_api.c nav_coordinator.c motion_executor.c nav_codec.c buddy_hooks.c wifi_transport.c wifi_protocol.c wifi_packets.c)
TEMP_OBJS = $(TEMP_SRCS:.c=.o)
TEMP_DEPS = $(TEMP_SRCS:.c=.d)

OBJS += $(subst ../, ./mtkernel_3/, $(TEMP_OBJS))
C_DEPS += $(subst ../, ./mtkernel_3/, $(TEMP_DEPS))

# Pure controller files include libc types; keep T-Kernel headers behind
# the port boundary and put the small compatibility surface first.
mtkernel_3/app_program/subsystem2/drive_task.o mtkernel_3/app_program/subsystem2/utk_hal.o: INCPATH := -I../app_program/subsystem2/controller/utk_compat -I../app_program/subsystem2/controller -I../app_program/subsystem2 $(INCPATH)
mtkernel_3/app_program/subsystem2/drive_task.o: CFLAGS += -fstack-usage -Wall -Wextra -Werror -DBUDDY_UTK_DRIVE=1 -DBUDDY_RUNTIME_MAPPING=1 -DBUDDY_WIFI_LOGGING=1 -DLINE_CAL_MIN_SPAN=50 -DCIRCUIT_SWEEP_SEARCH=1
mtkernel_3/app_program/subsystem2/controller/%.o: CFLAGS += -Wall -Wextra -Werror -DBUDDY_RUNTIME_MAPPING=1 -DBUDDY_WIFI_LOGGING=1 -DLINE_CAL_MIN_SPAN=50 -DCIRCUIT_SWEEP_SEARCH=1

mtkernel_3/app_program/%.o: ../app_program/%.c
	@echo 'Building file: $<'
	@mkdir -p $(dir $@)
	$(GCC) $(INCPATH) -I../app_program $(CFLAGS) -D$(TARGET) -MF"$(@:%.o=%.d)" -MT"$(@)" -x c -c -o "$@" "$$(cygpath -ms "$<")"
	@echo 'Finished building: $<'
	@echo ' '
