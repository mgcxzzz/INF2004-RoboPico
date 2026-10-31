################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

# app_program/*.c plus one level of subfolders (one per subsystem)
ifeq ($(APP),rtos_bringup)
TEMP_SRCS = $(wildcard ../app_program/rtos_bringup/*.c) \
 ../firmware/integration/buddy1_4/src/navigation/motion_api.c \
 ../firmware/integration/buddy1_4/src/navigation/nav_coordinator.c \
 ../firmware/integration/buddy1_4/src/navigation/nav_codec.c \
 ../firmware/integration/buddy1_4/src/motion/motion_executor.c
else
TEMP_SRCS = $(wildcard ../app_program/*.c) $(filter-out ../app_program/rtos_bringup/%, $(wildcard ../app_program/*/*.c))
endif
TEMP_OBJS = $(TEMP_SRCS:.c=.o)
TEMP_DEPS = $(TEMP_SRCS:.c=.d)

OBJS += $(subst ../, ./mtkernel_3/, $(TEMP_OBJS))
C_DEPS += $(subst ../, ./mtkernel_3/, $(TEMP_DEPS))

mtkernel_3/app_program/%.o: ../app_program/%.c
	@echo 'Building file: $<'
	@mkdir -p $(dir $@)
	$(GCC) $(CFLAGS) $(if $(filter rtos_bringup,$(APP)),-Wall -Wextra -Werror,) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

mtkernel_3/firmware/integration/buddy1_4/%.o: ../firmware/integration/buddy1_4/%.c
	@mkdir -p $(dir $@)
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
