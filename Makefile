################################################################################
# 
# Copyright (C) 2022
# 
# Makefile for microros for stm32f103c8t6
#
# by duvallee.lee
# 
################################################################################

# -----------------------------------------------------------------------------
# CubeIDE Ver 1.9 for the windows

# gcc
GCC_PREFIX := arm-none-eabi
CC := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE := arm-none-eabi-size

MKDIR_CMD := mkdir
MKDIR_FROM_CHAR =/
MKDIR_TO_CHAR =\\
MKDIR_OPTION = 2> NUL || echo off

SHELL := cmd

FIRMWARE_MAJOR_VERSION = 0
FIRMWARE_MINOR_VERSION = 1

# -----------------------------------------------------------------------------
TARGET := micro_ros_stm32f401ccu6_${FIRMWARE_MAJOR_VERSION}_${FIRMWARE_MINOR_VERSION}

CFLAGS :=
ASFLAGS :=
LDFLAGS :=
# Link Script
LINK_SCRIPT := "STM32F401CCUX_FLASH.ld"

# -----------------------------------------------------------------------------
ASM_SRC :=
C_SRC :=
INCLUDE_DIR :=

# -----------------------------------------------------------------------------
# Define output directory
OBJECT_DIR = OUTPUT
BIN_DIR = $(OBJECT_DIR)

# -----------------------------------------------------------------------------
# include directory
INCLUDE_DIR += -ICore/Inc
INCLUDE_DIR += -IDrivers/STM32F4xx_HAL_Driver/Inc
INCLUDE_DIR += -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy
INCLUDE_DIR += -IDrivers/CMSIS/Device/ST/STM32F4xx/Include
INCLUDE_DIR += -IDrivers/CMSIS/Include
INCLUDE_DIR += -IMiddlewares/Third_Party/FreeRTOS/Source/include
INCLUDE_DIR += -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
INCLUDE_DIR += -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F

# -----------------------------------------------------------------------------
# include directory for micro-ROS
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/micro_ros_msgs/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/rcl/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/composition_interfaces/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/std_msgs/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/microxrcedds_client/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/builtin_interfaces/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/rosidl_runtime_c/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/rcutils/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/actionlib_msgs/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/rcl_interfaces/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/rosidl_typesupport_interface/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/test_msgs/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/microcdr/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/libyaml_vendor/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/trajectory_msgs/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/shape_msgs/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/sensor_msgs/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/stereo_msgs/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/rcl_action/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/action_msgs/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/rosidl_typesupport_c/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/rcl_lifecycle/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/rclc/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/example_interfaces/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/rclc_parameter/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/geometry_msgs/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/std_srvs/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/diagnostic_msgs/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/statistics_msgs/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/rmw/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/tracetools/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/tinydir_vendor/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/rmw_microxrcedds/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/unique_identifier_msgs/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/lifecycle_msgs/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/rosgraph_msgs/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/rosidl_typesupport_microxrcedds_c/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/rclc_lifecycle/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/nav_msgs/include
INCLUDE_DIR += -IMiddlewares/Third_Party/MicroROS/microros_static_library/microros_include/visualization_msgs/include

# -----------------------------------------------------------------------------
# Assembler Flags
ASFLAGS := -mcpu=cortex-m4
# for debug
ASFLAGS += -g3 -DDEBUG
ASFLAGS += -c
ASFLAGS += -x assembler-with-cpp
ASFLAGS += -MMD
ASFLAGS += -MP
ASFLAGS += --specs=nano.specs

# 
ASFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# ASFLAGS += -mfloat-abi=softfp -mfpu=fpv4-sp-d16
# ASFLAGS += -mfloat-abi=soft -mfpu=fpv4-sp-d16

ASFLAGS += -mthumb

# -----------------------------------------------------------------------------
# C Flags
CFLAGS := -mcpu=cortex-m4
CFLAGS += -std=gnu11
# for debug
CFLAGS += -g3 -DDEBUG
#
CFLAGS += -DUSE_HAL_DRIVER
CFLAGS += -DSTM32F401xC
CFLAGS += -c
CFLAGS +=  $(INCLUDE_DIR)
# O0, O1, O2
CFLAGS += -O0
#
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections
CFLAGS += -Wall
CFLAGS += -fstack-usage
CFLAGS += -MMD
CFLAGS += -MP
CFLAGS += --specs=nano.specs

CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# CFLAGS += -mfloat-abi=softfp -mfpu=fpv4-sp-d16
# CFLAGS += -mfloat-abi=soft -mfpu=fpv4-sp-d16

CFLAGS += -mthumb
CFLAGS += -DFIRMWARE_MAJOR_VERSION=${FIRMWARE_MAJOR_VERSION}
CFLAGS += -DFIRMWARE_MINOR_VERSION=${FIRMWARE_MINOR_VERSION}

# -----------------------------------------------------------------------------
# Linker Flags
LDFLAGS := -mcpu=cortex-m4
LDFLAGS += -T$(LINK_SCRIPT)
LDFLAGS += --specs=nosys.specs
LDFLAGS += -Wl,-Map="$(BIN_DIR)/$(TARGET).map"
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -static
# for library & printf
LDFLAGS += --specs=nano.specs -u _printf_float

LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16 
# LDFLAGS += -mfloat-abi=softfp -mfpu=fpv4-sp-d16
# LDFLAGS += -mfloat-abi=soft -mfpu=fpv4-sp-d16

LDFLAGS += -mthumb
LDFLAGS += -Wl,--start-group -lc -lm -Wl,--end-group
LDFLAGS += Middlewares/Third_Party/MicroROS/microros_static_library/libmicroros.a

# -----------------------------------------------------------------------------
# Assemble Source File
ASM_SRC += Core/Startup/startup_stm32f401ccux.s

# -----------------------------------------------------------------------------
# HAL Library
C_SRC += Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
C_SRC += Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c
C_SRC += Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c
C_SRC += Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
C_SRC += Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
C_SRC += Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
C_SRC += Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
C_SRC += Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
C_SRC += Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
C_SRC += Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
C_SRC += Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
C_SRC += Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
C_SRC += Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
C_SRC += Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
C_SRC += Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
C_SRC += Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
C_SRC += Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
C_SRC += Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
C_SRC += Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
C_SRC += Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
C_SRC += Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_adc.c

# -----------------------------------------------------------------------------
# Middleware : FreeRTOS
C_SRC += Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
C_SRC += Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
C_SRC += Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
C_SRC += Middlewares/Third_Party/FreeRTOS/Source/croutine.c
C_SRC += Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
C_SRC += Middlewares/Third_Party/FreeRTOS/Source/list.c
C_SRC += Middlewares/Third_Party/FreeRTOS/Source/queue.c
C_SRC += Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
C_SRC += Middlewares/Third_Party/FreeRTOS/Source/tasks.c
C_SRC += Middlewares/Third_Party/FreeRTOS/Source/timers.c

# -----------------------------------------------------------------------------
# Middleware : MicroROS

# interface function for MicroROS
C_SRC += Middlewares/Third_Party/MicroROS/extra_sources/custom_memory_manager.c
C_SRC += Middlewares/Third_Party/MicroROS/extra_sources/microros_allocators.c
C_SRC += Middlewares/Third_Party/MicroROS/extra_sources/microros_time.c
C_SRC += Middlewares/Third_Party/MicroROS/extra_sources/microros_transports/dma_transport.c

# -----------------------------------------------------------------------------
# C Source File
C_SRC += Core/Src/freertos.c
C_SRC += Core/Src/main.c
C_SRC += Core/Src/stm32f4xx_hal_msp.c
C_SRC += Core/Src/stm32f4xx_it.c
C_SRC += Core/Src/syscalls.c
C_SRC += Core/Src/sysmem.c
C_SRC += Core/Src/system_stm32f4xx.c
C_SRC += Core/Src/servo_motor_driver.c

# -----------------------------------------------------------------------------
# OBJECT Files
OBJS := $(C_SRC:%.c=$(OBJECT_DIR)/%.o)
OBJS += $(ASM_SRC:%.s=$(OBJECT_DIR)/%.o)

# -----------------------------------------------------------------------------
###############
# Build project
# Major targets
###############
.PHONY: DEBUG_MAKE_BEFORE $(TARGET) clean

all: DEBUG_MAKE_BEFORE $(TARGET)
# all: $(TARGET)

DEBUG_MAKE_BEFORE:
	@echo =======================================================================================
	@echo CFLAGS : $(CFLAGS)
	@echo ASFLAGS : $(ASFLAGS)
	@echo LFLAGS : $(LFLAGS)
	@echo =======================================================================================
	@echo ASM_SRC : $(ASM_SRC)
	@echo C_SRC   : $(C_SRC)
	@echo OBJS    : $(OBJS)
	@echo =======================================================================================


$(TARGET): $(OBJS) 
	$(CC) -o "$(BIN_DIR)/$(TARGET).elf" $(OBJS) $(LDFLAGS)
	$(OBJCOPY) -O ihex "$(BIN_DIR)/$(TARGET).elf" "$(BIN_DIR)/$(TARGET).hex"
	$(OBJCOPY) -O binary "$(BIN_DIR)/$(TARGET).elf" "$(BIN_DIR)/$(TARGET).bin"
	@echo =======================================================================================
	$(SIZE) "$(BIN_DIR)/$(TARGET).elf"
	@echo =======================================================================================
	$(CC) -v

clean:
	$(RM) $(OBJS) "$(BIN_DIR)/$(TARGET).elf" "$(BIN_DIR)/$(TARGET).map" "$(BIN_DIR)/$(TARGET).hex" "$(BIN_DIR)/$(TARGET).bin"


##################
# Implicit targets
##################
$(OBJECT_DIR)/%.o: %.c
	@$(MKDIR_CMD) $(subst $(MKDIR_FROM_CHAR),$(MKDIR_TO_CHAR),$(dir $@)) $(MKDIR_OPTION)
	$(CC) $(CFLAGS) $< -o $@

# $(OBJECT_DIR)/%.o: %.cpp
#	@$(MKDIR_CMD) $(subst $(MKDIR_FROM_CHAR),$(MKDIR_TO_CHAR),$(dir $@)) $(MKDIR_OPTION)
#	$(CXX) $(CXXFLAGS) $< -o $@
	
$(OBJECT_DIR)/%.o: %.s
	@$(MKDIR_CMD) $(subst $(MKDIR_FROM_CHAR),$(MKDIR_TO_CHAR),$(dir $@)) $(MKDIR_OPTION)
	$(CC) $(ASFLAGS) $< -o $@

$(OBJECT_DIR)/%.o: %.S
	@$(MKDIR_CMD) $(subst $(MKDIR_FROM_CHAR),$(MKDIR_TO_CHAR),$(dir $@)) $(MKDIR_OPTION)
	$(CC) $(ASFLAGS) $< -o $@

