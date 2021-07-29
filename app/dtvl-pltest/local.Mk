LOCAL_PATH := $(call local-dir)
TARGET_DO_DIR := $(TARGET_APP_DO_DIR)
include $(CLEAR_MFILE)
LOCAL_VERSION := 1.00.00
LOCAL_MODULE := dtvl_pltest
LOCAL_SRC_FILES := src/main.c\
			src/equipment.c\
			src/utils.c\
			src/gpio_operate.c
LOCAL_C_INCLUDES := $(LOCAL_PATH)/inc
LOCAL_FLAGS := -O2 -g -Wall
LOCAL_FLAGS += -I $(LOCAL_PATH)/inc/
LOCAL_LDFLAGS := -L $(LOCAL_PATH)/lib -lxdjakey-pltest -lrt
LOCAL_LDLIBS := -lpthread

LOCAL_MODULE_PATH := $(TARGET_APP_OUT)/dtvl_pltest
include $(EXECUTABLE_MFILE)
LOCAL_DEPEND_MODULES := dtvl_pltest

include $(CLEAR_MFILE)
LOCAL_MODULE := libxdjakey-pltest.so
LOCAL_SRC_FILES := ./lib/libxdjakey-pltest.so
LOCAL_MODULE_PATH := $(TARGET_APP_OUT)/lib
include $(PREBUILT_MFILE)
#include $(BUILDING2PREBUILT_MFILE)
