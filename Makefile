################################################################################
# 
# Makefile for STM32MP157CAC3 (STM32MP157CACx)
# for CubeIDE Ver 1.3
# 2020
#
################################################################################

# -----------------------------------------------------------------------------
# variable for gcc
# -----------------------------------------------------------------------------
GCC_PREFIX  = arm-none-eabi-
CC          = $(GCC_PREFIX)gcc
AS          = $(GCC_PREFIX)gcc -x assembler-with-cpp
OBJCOPY     = $(GCC_PREFIX)objcopy
AR          = $(GCC_PREFIX)ar
SIZE        = $(GCC_PREFIX)size

HEX	        = $(OBJCOPY) -O ihex
BIN         = $(OBJCOPY) -O binary -S

# -----------------------------------------------------------------------------
# target
# -----------------------------------------------------------------------------
BUILD_DIR   = build
TARGET      = stm32mp_m4_CM4

# -----------------------------------------------------------------------------
# tool
# -----------------------------------------------------------------------------

RMDIR       = RMDIR /S /Q
MKDIR       = mkdir
SLASH_CHAR  =/
BACKSLASH_CHAR =\\
MKDIR_OPT   = 2> NUL || echo off

# -----------------------------------------------------------------------------
# variable for cpu
# -----------------------------------------------------------------------------
# CPU
CPU         = -mcpu=cortex-m4
# fpu
FPU         = -mfpu=fpv4-sp-d16
# floating point - abi
FLOAT-ABI   = -mfloat-abi=hard

# MCU
MCU         = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# -----------------------------------------------------------------------------
# variable for gcc
# -----------------------------------------------------------------------------
# optimization
OPT         = -O0 -g3

# -----------------------------------------------------------------------------
# variable for gcc (c language)
# -----------------------------------------------------------------------------
# compile
C_DEFS      = -c
# C statndard
C_DEFS     += $(MCU) -std=gnu11 $(OPT) 
# use nano library in arm-none-eabi-gcc
C_DEFS     += --specs=nano.specs
# 
C_DEFS     += -ffunction-sections -fdata-sections
#  all error & warning in build
C_DEFS     += -Wall
#
C_DEFS     += -fstack-usage
# Generate dependency information
C_DEFS     += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"
# definition of st
C_DEFS     += -DUSE_HAL_DRIVER -DSTM32MP157Cxx -DCORE_CM4

# -----------------------------------------------------------------------------
# variable for gcc (assebly language)
# -----------------------------------------------------------------------------
# compile
AS_DEFS     = -c -x assembler-with-cpp
# 
AS_DEFS    += $(MCU) --specs=nano.specs $(OPT) 

# -----------------------------------------------------------------------------
# variable for include
# -----------------------------------------------------------------------------
# assembly
AS_INCLUDES = 
# C
C_INCLUDES  = 
C_INCLUDES += -I./Drivers/STM32MP1xx_HAL_Driver/Inc/Legacy
C_INCLUDES += -I./Drivers/CMSIS/Include
C_INCLUDES += -I./Drivers/CMSIS/Device/ST/STM32MP1xx/Include
C_INCLUDES += -I./Drivers/STM32MP1xx_HAL_Driver/Inc
C_INCLUDES += -I./Core/Inc

# -----------------------------------------------------------------------------
# variable for link
# -----------------------------------------------------------------------------
# link script
LDSCRIPT    = link_script/STM32MP157CACX_RAM.ld
# library
LIBS        = -lc -lm

# -----------------------------------------------------------------------------
# variable
# -----------------------------------------------------------------------------
CFLAGS      = $(C_DEFS) $(C_INCLUDES)
ASLAGS      = $(AS_DEFS) $(AS_INCLUDES)

LDLAGS      = $(MCU) -T"$(LDSCRIPT)"
LDLAGS     += --specs=nosys.specs
LDLAGS     += -Wl,-Map="stm32mp_m4_CM4.map"
LDLAGS     += -Wl,--gc-sections -static --specs=nano.specs 
LDLAGS     += -Wl,--start-group
LDLAGS     += $(LIBS)
LDLAGS     += -Wl,--end-group

# -----------------------------------------------------------------------------
# c source
# -----------------------------------------------------------------------------
C_SOURCES   =
# HAL Libary
C_SOURCES  += ./Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal.c
C_SOURCES  += ./Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_cortex.c
C_SOURCES  += ./Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_dma.c
C_SOURCES  += ./Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_dma_ex.c
C_SOURCES  += ./Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_exti.c
C_SOURCES  += ./Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_gpio.c
C_SOURCES  += ./Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_hsem.c
C_SOURCES  += ./Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_mdma.c
C_SOURCES  += ./Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_pwr.c
C_SOURCES  += ./Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_pwr_ex.c
C_SOURCES  += ./Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_rcc.c
C_SOURCES  += ./Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_rcc_ex.c
C_SOURCES  += ./Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_tim.c
C_SOURCES  += ./Drivers/STM32MP1xx_HAL_Driver/Src/stm32mp1xx_hal_tim_ex.c

# common
C_SOURCES  += ./Common/System/system_stm32mp1xx.c

# User Source
C_SOURCES  += ./Core/Src/main.c
C_SOURCES  += ./Core/Src/stm32mp1xx_hal_msp.c
C_SOURCES  += ./Core/Src/stm32mp1xx_it.c
C_SOURCES  += ./Core/Src/syscalls.c
C_SOURCES  += ./Core/Src/sysmem.c

# -----------------------------------------------------------------------------
# ASM source
# -----------------------------------------------------------------------------
ASM_SOURCES = Core/Startup/startup_stm32mp157cacx.s

# -----------------------------------------------------------------------------
# OBJECT Files
OBJS      :=
# OBJS      += $(C_SOURCES:%.c=$(BUILD_DIR)/%.o)
# OBJS      += $(ASM_SOURCES:%.s=$(BUILD_DIR)/%.o)

OBJS      += $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJS      += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# -----------------------------------------------------------------------------
#
# -----------------------------------------------------------------------------
.PHONY: $(TARGET) clean

# =============================================================================
#
# =============================================================================
all: $(TARGET)
	@echo "build success !!!"

$(TARGET): $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@
	$(CC) -o "$(BUILD_DIR)/$(TARGET).elf" $(OBJS) $(LFLAGS)

clean:
	$(RMDIR) $(BUILD_DIR)
	@echo "clean success !!!"

# =============================================================================
# Implicit targets
# =============================================================================
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	@echo "$@"
	$(CC) $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.oo: %.cpp Makefile | $(BUILD_DIR) 
	@echo "$@"
	$(CXX) $(CXXFLAGS) $< -o $@
	
$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR) 
	@echo "$@"
	$(CC) $(AFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.S Makefile | $(BUILD_DIR) 
	@echo "$@"
	$(CC) $(AFLAGS) $< -o $@

$(BUILD_DIR):
	@$(MKDIR) $@		









