ifeq ($(ARCH),)
ARCH ?= arm64
CROSS_COMPILE ?= aarch64-linux-gnu-
else ifeq ($(ARCH),arm)
SYSTEM_TYPE = SYSTEM32
CROSS_COMPILE ?= arm-linux-gnueabihf-
EXTRA_CFLAGS += -DSYSTEM32
else ifeq ($(ARCH),arm64)
SYSTEM_TYPE = SYSTEM64
CROSS_COMPILE ?= aarch64-linux-gnu-
EXTRA_CFLAGS += -DSYSTEM64
endif


ifneq ($(ROKU_PROJECT),)
CONFIG_ROKU=y
CONFIG_CFG80211=y
CONFIG_LINUX_PLATFORM=y
EXTRA_CFLAGS += -DCONFIG_ROKU

-include $(BACKPORTS_VERSIONS)
ifneq ($(BACKPORTED_KERNEL_VERSION),)
export CFG_CFG80211_VERSION ?= $(BACKPORTED_KERNEL_VERSION)
endif

ifneq ($(BACKPORTS_VERSIONS),)
LINUXINCLUDE := $(BACKPORTS_INCLUDE_DIR) $(LINUXINCLUDE)
include $(BACKPORTS_VERSIONS)
EXTRA_CFLAGS += -D__rcu=
EXTRA_CFLAGS += -DCOMPAT_KERNEL_RELEASE=1
EXTRA_CFLAGS += '-include' 'backport/backport.h'
EXTRA_CFLAGS += -DCPTCFG_VERSION=\"$(BACKPORTS_VERSION)\"
EXTRA_CFLAGS += -DCPTCFG_KERNEL_VERSION=\"$(BACKPORTED_KERNEL_VERSION)\"
EXTRA_CFLAGS += -DCPTCFG_KERNEL_NAME=\"$(BACKPORTED_KERNEL_NAME)\"
EXTRA_CFLAGS += -DBACKPORTED_KERNEL_NAME=\"$(BACKPORTED_KERNEL_NAME)\"
EXTRA_CFLAGS += -DBACKPORTS_VERSION=\"$(BACKPORTS_VERSION)\"
EXTRA_CFLAGS += -DBACKPORTED_KERNEL_VERSION=\"$(BACKPORTED_KERNEL_VERSION)\"
EXTRA_CFLAGS += -Wno-error=missing-attributes
EXTRA_CFLAGS += -Wno-error=stringop-truncation
EXTRA_CFLAGS += -Wno-error=unused-variable
endif

endif

ifneq ($(CFG_CFG80211_VERSION),)
VERSION_STR = $(subst $\",,$(subst ., , $(subst -, ,$(subst v,,$(CFG_CFG80211_VERSION)))))
$(info VERSION_STR=$(VERSION_STR))
X = $(firstword $(VERSION_STR))
Y = $(word 2 ,$(VERSION_STR))
Z = $(word 3 ,$(VERSION_STR))
VERSION := $(shell echo "$$(( $(X) * 65536 + $(Y) * 256 + $(Z)))" )
ccflags-y += -DCFG_CFG80211_VERSION=$(VERSION)
$(info DCFG_CFG80211_VERSION=$(VERSION))
endif

ifneq ($(CONFIG_LINUX_PLATFORM),)
EXTRA_CFLAGS += -DLINUX_PLATFORM
endif

ifeq ($(PROJ_NAME),p-amlogic)
SRCTOP ?= /proj/vlsi.wifi/p-amlogic
KERNELDIR ?= $(SRCTOP)/out/target/product/newton/obj/KERNEL_OBJ
EXTRA_CFLAGS += -DSYSTEM64
else ifeq ($(PROJ_NAME),t920l)
SRCTOP ?= /proj/vlsi.wifi/t920l
KERNELDIR ?= $(SRCTOP)/out/target/product/txhd_t920l_r341/obj/KERNEL_OBJ
EXTRA_CFLAGS += -DSYSTEM64
endif

#CC        = $(CROSS_COMPILE)gcc
#STRIP     = $(CROSS_COMPILE)strip

M ?= $(shell pwd)
ifneq ($(KERNEL_SRC),)
CUR_DIR  := $(KERNEL_SRC)/$(M)
else
CUR_DIR  := $(M)
endif
EXT_INCS := $(CUR_DIR)/../common
INCS     := -I$(CUR_DIR) -I$(EXT_INCS)

ccflags-y += -I$(CUR_DIR)
ccflags-y += -I$(CUR_DIR)/aml_bus
ccflags-y += -I$(CUR_DIR)/../common
ccflags-y += -I$(CUR_DIR)/linux


KERNEL_SRC ?= $(KERNELDIR)
#n=for normal mode,y=product mode
#n=not support WIFI_RF when boot up. y=support WIFI_RF when boot up
CONFIG_WIFI_RF_SUPPORT=y
#n=not support access bt rf reg, y=support access bt rf reg
CONFIG_BT_RF_SUPPORT=n
CONFIG_AML_CFG80211=y
CONFIG_AML_P2P=y
CONFIG_SDIO=y
CONFIG_MAC_SUPPORT=y
CONFIG_WIFI_CAPTURE=y
CONFIG_80211_W=y
CONFIG_WPA3=y
ifneq ($(ROKU_PROJECT),)
CONFIG_CHIP_RESET=y
endif
TARGET = w1u

EXTRA_CFLAGS += -Wno-enum-conversion -Wno-strict-prototypes
EXTRA_CFLAGS += -Wno-incompatible-pointer-types
EXTRA_CFLAGS += ${INCS}  -DCONFIG_WAPI
EXTRA_CFLAGS += -Wall -Wno-unused-function #show all warning
EXTRA_CFLAGS += -Werror -Wno-format -Wno-error #convert warning into error
#EXTRA_CFLAGS += -W	#more strict, linux head file cannot pass, so give up
#EXTRA_CFLAGS += -w	#ignore warning
#EXTRA_CFLAGS += -DHALMODULE
EXTRA_CFLAGS += -DPHY_TEST_FUNC
EXTRA_CFLAGS += -DCONFIG_CONCURRENT_MODE
#EXTRA_CFLAGS += -O2	#kernel has
#EXTRA_CFLAGS += -DDRV_SUPPORT_TX_WITHDRAW
EXTRA_CFLAGS += -DHAL_FPGA_VER
EXTRA_CFLAGS += -DPROJECT_W1
EXTRA_CFLAGS += -DINTER_BEACON
EXTRA_CFLAGS += -DSRAM_CONCURRENT
EXTRA_CFLAGS += -DCTS_VERIFIER_GAS
EXTRA_CFLAGS += -DRX_PN_CHECK
#EXTRA_CFLAGS += -DCONFIG_COMPAT
EXTRA_CFLAGS += -DCONFIG_AML_USE_STATIC_BUF
EXTRA_CFLAGS += -DSDIO_BUILD_IN
EXTRA_CFLAGS += -DSU_BF
EXTRA_CFLAGS += -DMU_BF
EXTRA_CFLAGS += -DUSB_BUILD_IN
#EXTRA_CFLAGS += -DDEBUG_MALLOC
#EXTRA_CFLAGS += -DCONFIG_DPD_ENABLE
EXTRA_CFLAGS += -DFW_RF_CALIBRATION
#EXTRA_CFLAGS += -DPOWER_SAVE_NO_SDIO
EXTRA_CFLAGS += -DHOST
EXTRA_CFLAGS += -DDYNAMIC_BW
EXTRA_CFLAGS += -DWFA_P2P_TEST
#EXTRA_CFLAGS += -DPNO_SUPPORT
EXTRA_CFLAGS += -DSDIO_MODE_ON
#EXTRA_CFLAGS += -DAML_CHANLIST_ADJUST
ifneq ($(ROKU_PROJECT),)
EXTRA_CFLAGS += -DNOT_AMLOGIC_PLATFORM
endif

ifeq ($(CONFIG_80211_W),y)
EXTRA_CFLAGS += -DSUPPORT_80211W
ifeq ($(CONFIG_WPA3),y)
EXTRA_CFLAGS += -DAML_WPA3
endif
endif

ifneq ($(CONFIG_CHIP_RESET),)
EXTRA_CFLAGS += -DCHIP_RESET_SUPPORT
endif#CONFIG_CHIP_RESET

ifeq ($(UNOENV),$(UNO_GXL))
EXTRA_CFLAGS += -DCONFIG_GXL
EXTRA_CFLAGS += -DCONFIG_HAS_WAKELOCK
#EXTRA_CFLAGS += -DCONFIG_RTC_ENABLE
#EXTRA_CFLAGS += -DCONFIG_GPIO_RESET
EXTRA_CFLAGS += -DUSE_GPIO_IRQ
#EXTRA_CFLAGS += -DUSE_SDIO_IRQ
endif

ifneq ($(KERNELRELEASE),)

obj-m += w1u_comm.o
w1u_comm-objs:=\
        aml_bus/aml_interface.o\
        aml_bus/usb_common.o
w1u_comm-objs += aml_bus/w1_sdio.o

ifeq ($(CLANG_VERSION), r487747)
EXTRA_CFLAGS += -Wno-unused-but-set-variable
endif

#obj-m += w1u_sdio.o
#w1u_sdio-objs += w1_sdio/w1_sdio.o
#obj-m += w1u_usb.o
#w1u_usb-objs += usb_interface/usb_common.o


obj-m += $(TARGET).o
DISCARD :=
SRC_FILE := $(wildcard $(CUR_DIR)/*.c $(CUR_DIR)/rf_t9026/*.c $(CUR_DIR)/crypto/*.c $(CUR_DIR)/linux/*.c)
TARGET_FILE = $(subst $(CUR_DIR)/,, $(SRC_FILE:%.c=%.o))

ifneq ($(CONFIG_SDIO),)
EXTRA_CFLAGS += -DCONFIG_SDIO_IF
EXTRA_CFLAGS += -DFPGA
#EXTRA_CFLAGS += -DCONFIG_GPIO_WAKEUP

ifeq ($(CONFIG_PT_SUPPORT),y)
EXTRA_CFLAGS += -DDRV_PT_SUPPORT
else
KBUILD_CFLAGS_MODULE += -Wframe-larger-than=200000000
KBUILD_CFLAGS_MODULE += -Wlarger-than=20000000
endif

ifeq ($(CONFIG_WIFI_RF_SUPPORT),y)
EXTRA_CFLAGS += -DWITH_RF
EXTRA_CFLAGS += -DRF_T9026
endif

ifeq ($(CONFIG_MAC_SUPPORT),y)
EXTRA_CFLAGS += -DCONFIG_MAC_SUPPORT
ifeq ($(CONFIG_ROKU),y)
EXTRA_CFLAGS += -DWIFIMAC_PATH=\"/tmp/wifimac.txt\"
else ifeq ($(CONFIG_BUILDROOT),y)
EXTRA_CFLAGS += -DWIFIMAC_PATH=\"/etc/wifi/wifimac.txt\"
EXTRA_CFLAGS += -DLINUX_PLATFORM
CONFIG_CFG80211=y
else
EXTRA_CFLAGS += -DWIFIMAC_PATH=\"/data/vendor/wifi/wifimac.txt\"
endif
endif

ifeq ($(CONFIG_BT_RF_SUPPORT),y)
EXTRA_CFLAGS += -DDRIVER_FOR_BT
endif

ifeq ($(CONFIG_WIFI_CAPTURE),y)
EXTRA_CFLAGS += -DWIFI_CAPTURE
endif

EXTRA_CFLAGS += -DUSE_T902X_RF
else#CONFIG_SDIO_IF
$(DISCARD) += wifi_hif.o
$(DISCARD) += wifi_sdio.o

endif #CONFIG_SDIO_IF

ifneq ($(CONFIG_CFG80211),)
ifneq ($(CONFIG_AML_CFG80211),)
EXTRA_CFLAGS += -DCONFIG_AML_CFG80211

ifeq ($(CONFIG_AML_P2P),y)
EXTRA_CFLAGS += -DCONFIG_P2P
EXTRA_CFLAGS += -DCONFIG_WFD
endif

endif	#CONFIG_AML_CFG80211
endif 	#CONFIG_CFG80211

ifneq ($(CONFIG_USB_CLOSE),y)
EXTRA_CFLAGS += -DCONFIG_USB
endif


TARGET_LIST := $(filter-out $(DISCARD), $(TARGET_FILE))
$(TARGET)-objs = $(TARGET_LIST)

else

modules:
	perl ./create_version_file.pl
	@echo "UNO_GXL_SUB_P200=$(UNO_GXL_SUB_P200)"
	@echo "PROJECT = $(PROJ_NAME)"
	@echo "UNOENV=$(UNOENV) MAC_VER=$(MAC_VER)"
	@echo "EXTRA_CFLAGS=$(EXTRA_CFLAGS)"
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL_SRC) M=$(M) modules
ifneq (${OUT_DIR},)
ifneq (${LLVM},)
	llvm-strip --strip-unneeded ${OUT_DIR}/$(M)/$(TARGET).ko
else
	$(CROSS_COMPILE)strip --strip-unneeded ${OUT_DIR}/$(M)/$(TARGET).ko
endif
else
	$(STRIP) -S $(TARGET).ko
endif

modules_install:
	@$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install
	mkdir -p ${OUT_DIR}/../vendor_lib/modules
	cd ${OUT_DIR}/$(M)/; find -name "*.ko" -exec cp {} ${OUT_DIR}/../vendor_lib/modules/ \;

endif

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions *.order *.symvers *.o.symversions *.mod
	rm -rf ./rf_t9026/*.o ./rf_t9026/.*.cmd ./w1_sdio/*.o ./w1_sdio/.*.cmd ./w1_sdio/.*.o.d
	rm -rf ./usb_interface/*.o ./usb_interface/.*.cmd ./usb_interface/.*.o.d
	rm -rf ./aml_bus/*.o ./aml_bus/.*.cmd ./aml_bus/.*.o.d ./aml_bus/*.o.symversions
	rm -rf ./linux/*.o ./linux/.*.cmd ./linux/.*.o.d ./linux/*.o.symversions
	rm -rf ./crypto/*.o ./crypto/.*.cmd
	rm -rf ./print_version.c

depend .depend dep:
	$(CC) $(EXTRA_CFLAGS) -M *.c > .depend

ifeq (.depend,$(wildcard .depend))
include .depend
endif
