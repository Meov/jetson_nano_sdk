SHELL = /bin/bash
export SDK_VERSION := 0.0.1

export TOP_DIR=$(shell pwd 2>/dev/null)
export START_DATE := $(shell date +%Y_%m_%d_%T 2>/dev/null | awk '{gsub(":","_",$$1);print $$1}' 2>/dev/null)

TARGET := APP

OS_TYPE := $(shell uname -m 2>/dev/null)

BUILD_TOOL_CHAINS :=$(TOP_DIR)/build/tool_chains/gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu

exist = $(shell if [ -d $(BUILD_TOOL_CHAINS) ]; then echo "exist"; else echo "notexist"; fi;)

$(info $(exist) $(BUILD_TOOL_CHAINS))

ifeq ($(strip $(exist)), exist)
export CROSS_COMPILE := $(BUILD_TOOL_CHAINS)/bin/aarch64-linux-gnu-
else 
#$(error CROSS_COMPILER  NOT FOUND in $(TOP_DIR)/build/tool_chains)
$(info BUILD_TOOL_CHAINS does not exist we use gcc as the compiler)
endif

ifeq ($(strip $(OS_TYPE)), x86_64)
#export CROSS_COMPILE :=$(TOP_DIR)/build/tool_chains/gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-
$(info using cross compiler: $(CROSS_COMPILE))
else
$(error unsupported compile platform! only x86_64 supportted!)
endif


#judge APP OR APDD
export APDD_PROJECT_DIR := $(TOP_DIR)/apdd
export APP_PROJECT_DIR := $(TOP_DIR)/app
export MFILES_DIR := $(TOP_DIR)/build/mfiles
export OUT_DIR := $(TOP_DIR)/out

export APP_OUT_DIR := $(OUT_DIR)/app_out
OUT_DIRS_LIST += $(APP_OUT_DIR)

export APDD_OUT_DIR := $(OUT_DIR)/apdd_out
OUT_DIRS_LIST += $(APDD_OUT_DIR)

export TARGET_APP_OUT := $(APP_OUT_DIR)/app_targets
OUT_DIRS_LIST += $(TARGET_APP_OUT)

export TARGET_APDD_OUT := $(APDD_OUT_DIR)/apdd_targets
OUT_DIRS_LIST += $(TARGET_APDD_OUT)

export TARGET_APP_DO_DIR := $(APP_OUT_DIR)/obj
OUT_DIRS_LIST += $(TARGET_APP_DO_DIR)

export TARGET_APDD_DO_DIR := $(APDD_OUT_DIR)/obj
OUT_DIRS_LIST += $(TARGET_APDD_DO_DIR)

export TARGET_COMMON_APP_LDLIBS_DIR := $(APP_OUT_DIR)/obj/libs
OUT_DIRS_LIST += $(TARGET_COMMON_APP_LDLIBS_DIR)

export TARGET_COMMON_APDD_LDLIBS_DIR := $(APDD_OUT_DIR)/obj/libs
OUT_DIRS_LIST += $(TARGET_COMMON_APDD_LDLIBS_DIR)


ifeq ($(strip  $(TARGET)), APDD)
	TARGET_COMMON_LDLIBS_DIR := $(TARGET_COMMON_APDD_LDLIBS_DIR)
	TARGET_BUILD := $(TOP_DIR)/apdd
else ifeq ($(strip $(TARGET)),APP)
	TARGET_COMMON_LDLIBS_DIR := $(TARGET_COMMON_APP_LDLIBS_DIR)
	TARGET_BUILD := $(TOP_DIR)/app
else
	TARGET_BUILD := $(TOP_DIR)/app
endif

export TARGET_BUILD
export APDD_MODULES :=
export APP_MODULES :=

$(info Buding target $(TARGET))
.PHONY :all
.PHONY :all_test

test:
	@echo "start to build $(TARGET)"
	@echo $(TARGET_COMMON_APDD_LDLIBS_DIR)

all:$(APP_SUBDIR) $(APPDD_SUBDIR)#默认的执行规则目标
	@echo "start to build $(TARGET)"
	@for dir in $(APP_SUBDIR); do\
		$(MAKE) -C $$dir || exit 1;\
	done
	
	@for dir in $(APPDD_SUBDIR); do\
		$(MAKE) -C $$dir || exit 1;\
	done

all_test:
	@echo $(CROSS_COMPILE)
	@echo $(OS_TYPE)
	@echo $(OUT_DIRS_LIST)


include $(MFILES_DIR)/help.MK
include $(MFILES_DIR)/build_config.MK
include $(MFILES_DIR)/define.MK

# create target out dirs
$(foreach DIR, $(OUT_DIRS_LIST), $(shell mkdir -p $(DIR)))

# include all the .Mk files in subdirs
# 会将所有子目录下的.Mk文件展开 可以单独编译任何一个模块
subdir_makefiles := $(shell find $(TOP_DIR)/ -name "*.Mk" 2>/dev/null)
$(foreach mk, $(subdir_makefiles), $(info including $(mk) ...)$(eval -include $(mk)))

APP_MODULES := $(sort $(APP_MODULES))
.PHONY :app app.clean app.install

app:$(APP_MODULES)
	$(call color_echo,32,40,"SDK building $@ $(APP_MODULES) done... ")


.PHONY :clean
.PHONY :distclean
#$(RM) 相当于 rm -f
clean:
	@$(RM) -r $(OUT_DIR)
	@for dir in $(APP_SUBDIR); do\
		$(MAKE) -C $$dir clean || exit 1;\
	done

distclean:
	@for dir in $(APP_SUBDIR); do\
		$(MAKE) -C $$dir distclean || exit 1;\
	done
