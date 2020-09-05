include ../qsdk.config

# Convert asuswrt build environment variable to qca-hk
PKG_BUILD_DIR=$(shell pwd)/source

# Define variables for qca-hk
INSTALLFWDIR:=$(INSTALLDIR)/lib/firmware

all:
	@#Nothing to do

install:
	install -d $(INSTALLFWDIR)/IPQ8074
	cd $(PKG_BUILD_DIR)/PIL_IMAGES && rsync -avcH . $(INSTALLFWDIR)/IPQ8074
	cd $(PKG_BUILD_DIR) && rsync -avcH bdwlan* $(INSTALLFWDIR)/IPQ8074
	install -d $(INSTALLDIR)/vendor/firmware
	[ ! -e $(PKG_BUILD_DIR)/Data.msc ] || install -m 644 $(PKG_BUILD_DIR)/Data.msc $(INSTALLDIR)/vendor/firmware
	[ ! -e $(BUILD_NAME) ] || install -m 644 $(BUILD_NAME)/bdwlan* $(INSTALLFWDIR)/IPQ8074

clean:
	@#Nothing to do
