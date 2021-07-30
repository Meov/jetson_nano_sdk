LACAL_PATH :=$(call local-dir)

#judge wether we use this kernel
ifeq ($(TARGET_KERNEL_DIR),$(LACAL_PATH))
#clear all the FLAGS
include $(CLEAR_MFILE)

LOCAL_FILE := $(LACAL_PATH)/arch/arm/boot/zImage

ARGS := ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE)
MAKE_CMD := make -C $(LACAL_PATH) $(ARGS) $(LOCAL_CFLAGS) -j

.PHONY: kernel
kernel:kernel.install

.PHONY: kernel.install
kernel.install : kernel.img
	$(silent) -rm -rf $(KERNEL_TARGET_OUT_DIR)/kernel

.PHONY : kernel.img
kernel.img:$(LOCAL_FILE)
$(LOCAL_FILE):zImage.clean kernel.config
	$(silent) $(MAKE_CMD) zImage
	$(silent) -rm -rf $(LACAL_PATH)/arch/arm/boot/*.dtbs
	$(silent) $(MAKE_CMD) dtbs
	$(silent) cp -prf $(LACAL_PATH)/arch/arm/boot/*.dtbs $(KERNEL_TARGET_OUT_DIR)

.PHONY: zImage.clean

zImage.clean:
	$(silent) echo "Kernel rm arch/arm/boot/zImage..."
	$(silent) rm -f $(LOCAL_PATH)/arch/arm/boot/zImage

.PHONY : kernel.config
kernel.config : $(LACAL_PATH)/.config
#make defconfig
$(LACAL_PATH)/.config:$(LACAL_PATH)/arch/arm/configs/$(TARGET_KERNEL_CONFIG)
	$(silent) echo "Configure kernel with $(TARGET_KERNEL_CONFIG)..."
	$(silent) $(MAKE_CMD) $(TARGET_KERNEL_CONFIG)

.PHONY : kernel.make
kernel.make:
	@echo "Building kernel..."
	$(silent) $(MAKE_CMD)

.PHONY : kernel.clean
kernel.clean:
	@echo "Cleaning kernel..."
	$(silent) $(MAKE_CMD) clean
.PHONY : kernel.distclean
kernel.distclean:
	@echo "Distcleaning kernel..."
	$(silent) $(MAKE_CMD) distclean

.PHONY:kernel.menuconfig
kernel.menuconfig:
	@echo "kernel menuconfig..."
	$(silent) $(MAKE_CMD) menuconfig

.PHONY:dtbs
dtbs:
	@echo "make dtbs..."
	$(silent) -rm -rf $(LACAL_PATH)/arch/arm/boot/*.dtbs
	$(silent) $(MAKE_CMD) dtbs
	$(silent) cp -prf $(LACAL_PATH)/arch/arm/boot/*.dtbs $(KERNEL_TARGET_OUT_DIR)
endif
