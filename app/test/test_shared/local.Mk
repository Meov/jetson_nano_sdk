LOCAL_PATH := $(call local-dir)
TARGET_DO_DIR := $(TARGET_APP_DO_DIR)
include $(CLEAR_MFILE)
LOCAL_VERSION := 1.00.00
LOCAL_MODULE := test_shared
LOCAL_SRC_FILES := src/main.c
LOCAL_C_INCLUDES := $(LOCAL_PATH)/inc
LOCAL_FLAGS := -O2 -g -Wall
LOCAL_FLAGS += -I $(LOCAL_PATH)/inc/
#LOCAL_LDFLAGS := -L $(LOCAL_PATH)/libs -ltest -lrt
LOCAL_LDFLAGS := -ltest_shared -lrt
LOCAL_LDLIBS := -lpthread

LOCAL_MODULE_PATH := $(TARGET_APP_OUT)/apps
include $(EXECUTABLE_MFILE)
LOCAL_DEPEND_MODULES := test_shared

#include $(CLEAR_MFILE)
#LOCAL_MODULE := libtest.so
#LOCAL_SRC_FILES := ./libs/libtest.so
#LOCAL_MODULE_PATH := $(TARGET_APP_OUT)/libs/shared_libs/tset
#LOCAL_DEPEND_MODULES := libtest.so
#include $(PREBUILT_MFILE)
#include $(BUILDING2PREBUILT_MFILE)
