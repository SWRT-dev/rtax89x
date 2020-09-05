-include ../../include/config/auto.conf
-include ../../include/autoconf.mk

FLTYPE=$(CONFIG_FLASH_TYPE)
ifneq ($(CONFIG_MODEL),)
	QCPUARCH=$(CONFIG_MODEL)
else
ifeq ($(CONFIG_ARCH_IPQ807x),y)
	QCPUARCH=ipq807x
else ifeq ($(CONFIG_ARCH_IPQ806x),y)
	QCPUARCH=ipq806x
else ifeq ($(CONFIG_ARCH_IPQ40xx),y)
	QCPUARCH=ipq40xx
endif
endif


all:
	python2 prepareSingleImage.py --arch $(QCPUARCH) --in . $(if $(CONFIG_ARCH_IPQ807x),--genmbn)
	python2 pack_hk.py --arch $(QCPUARCH) --fltype $(FLTYPE) --srcPath . --inImage . --outImage . && \
		rsync -avcH $(FLTYPE)-$(QCPUARCH)-single.img ../../u-boot_$(FLTYPE).img
