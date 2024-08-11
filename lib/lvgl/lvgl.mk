LVGL_PATH ?= ${shell pwd}/lvgl

LVGL_ASRCS += $(shell find $(LVGL_PATH)/src -type f -name '*.S')
LVGL_CSRCS += $(shell find $(LVGL_PATH)/src -type f -name '*.c')
LVGL_CXXEXT := .cpp
LVGL_CXXSRCS += $(shell find $(LVGL_PATH)/src -type f -name '*${LVGL_CXXEXT}')

LVGL_AFLAGS += "-I$(LVGL_PATH)"
LVGL_CFLAGS += "-I$(LVGL_PATH)"
LVGL_CXXFLAGS += "-I$(LVGL_PATH)"
