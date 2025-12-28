# Makefile for STM32F446 Applications
# make -> build ELF

# Project: stm32f446-baremetal-button-debounce-timer
TARGET := stm32f446-baremetal-button-debounce-timer

# Toolchain
CC := arm-none-eabi-gcc
AS := arm-none-eabi-gcc

# Directories
BUILD_DIR   := build
SRC_DIR     := src
STARTUP_DIR := startup
LINKER_DIR  := linker
CMSIS_CORE  := cmsis/core
CMSIS_DEV   := cmsis/device

# Linker script
LDSCRIPT := $(LINKER_DIR)/stm32f446.ld

# Sources
C_SOURCES := \
  $(SRC_DIR)/app/main.c \
  $(SRC_DIR)/modules/button_tim.c

ASM_SOURCES := \
  $(STARTUP_DIR)/startup_stm32f446.s

# Output ELF file
ELF := $(BUILD_DIR)/$(TARGET).elf

# Objects
C_OBJECTS   := $(patsubst %.c,$(BUILD_DIR)/%.o,$(C_SOURCES))
ASM_OBJECTS := $(patsubst %.s,$(BUILD_DIR)/%.o,$(ASM_SOURCES))
OBJECTS     := $(C_OBJECTS) $(ASM_OBJECTS)

# MCU flags
CPUFLAGS := -mcpu=cortex-m4 -mthumb
FPUFLAGS := -mfpu=fpv4-sp-d16 -mfloat-abi=softfp

# Common flags
DEFINES  := -DSTM32F446xx

INCLUDES := -I$(CMSIS_CORE) -I$(CMSIS_DEV) \
            -I$(SRC_DIR)/modules -I$(SRC_DIR)/app

CFLAGS := $(CPUFLAGS) $(FPUFLAGS) $(DEFINES) $(INCLUDES) \
          -std=c11 -Wall -Wextra -Werror \
          -ffunction-sections -fdata-sections \
          -Og -g3

ASFLAGS := $(CPUFLAGS) $(FPUFLAGS) $(DEFINES) $(INCLUDES) \
           -x assembler-with-cpp -Og -g3

LDFLAGS := $(CPUFLAGS) $(FPUFLAGS) \
           -T$(LDSCRIPT) \
           -Wl,-Map=$(BUILD_DIR)/$(TARGET).map \
           -Wl,--gc-sections \
           -nostartfiles

# Default target
all: $(BUILD_DIR)/$(TARGET).elf

# Compile C
$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# Compile ASM
$(BUILD_DIR)/%.o: %.s
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) -c $< -o $@

# Link
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	@mkdir -p $(BUILD_DIR)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

# Debug with GDB
debug: $(ELF)
	gdb-multiarch $(ELF) \
		-ex "target extended-remote :4242" \
		-ex "monitor reset halt"

# Clean
clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean
