# Project Name
TARGET = midi_oled_monitor

# Sources
CPP_SOURCES = midi_oled_monitor.cpp

# Library Locations
# Optional machine-local paths; command-line overrides still take priority.
-include local.mk
LIBDAISY_DIR ?= ../libDaisy
DAISYSP_DIR ?= ../DaisySP

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

# Always compile the current source before sending firmware to the Seed.
program-dfu: all

# Own USART3 receiver without changing libDaisy or its other UART handlers.
LDFLAGS += -Wl,--defsym=USART3_IRQHandler=KickMidiRxIrq -Wl,--undefined=KickMidiRxIrq
