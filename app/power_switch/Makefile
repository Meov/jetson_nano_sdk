#include $(TOPDIR)/rule.mk
LOCAL_PATH=$(shell pwd)

CC := $(CROSS_COMPILE)gcc
TARGET_OUT_DIR :=  $(TARGET_APP_OUT)/power_switch

PRIIVAT_TARGET := power_switch
PRIIVAT_SRC := power_switch.c

.PHONY:TARGET

$(TARGET):$(PRIIVAT_TARGET)
	@echo "buding $(PRIIVAT_TARGET) in $(LOCAL_PATH) "

$(PRIIVAT_TARGET):$(PRIIVAT_SRC)
	$(CC) -c $(PRIIVAT_SRC) -O2 -Wall  -o $(PRIIVAT_TARGET)
	mkdir -p $(TARGET_OUT_DIR)
	cp -prf $(PRIIVAT_TARGET) $(TARGET_OUT_DIR)
	

.PHONY:clean
clean:
	@echo "clean in $(LOCAL_PATH)"
	$(RM) *.o $(PRIIVAT_TARGET)

.PHONY:distclean
distclean:
	@echo "distclean in $(LOCAL_PATH)"
	$(RM) *.o $(PRIIVAT_TARGET)



