ESP_OPEN_RTOS_DIR ?= ../esp-open-rtos
PROGRAM = esp-walkie-talkie
EXTRA_COMPONENTS = extras/i2s_dma extras/dhcpserver

include $(ESP_OPEN_RTOS_DIR)/common.mk
