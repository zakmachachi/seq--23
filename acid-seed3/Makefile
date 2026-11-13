TARGET = midi_acid_monitor
CPP_SOURCES = midi_acid_monitor.cpp
-include local.mk
LIBDAISY_DIR ?= ../libDaisy
DAISYSP_DIR ?= ../DaisySP
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
LDFLAGS += -Wl,--defsym=USART3_IRQHandler=AcidMidiRxIrq -Wl,--undefined=AcidMidiRxIrq
