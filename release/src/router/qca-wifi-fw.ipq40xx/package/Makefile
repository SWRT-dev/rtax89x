# Copyright (c) 2015 Qualcomm Atheros, Inc..
#
# All Rights Reserved.
# Qualcomm Atheros Confidential and Proprietary.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=qca-wifi-fw
PKG_VERSION:=WLAN.BL.3.1-00028-S-1
PKG_FOLDER:=WLAN.BL.3.1
PKG_RELEASE:=1

PKG_SOURCE:=$(PKG_NAME)$(BUILD_VARIANT)-$(PKG_VERSION).tar.bz2
PKG_SOURCE_URL:=http://tritium.qca.qualcomm.com/software/NBU/$(PKG_FOLDER)/$(PKG_VERSION)/src
PKG_BUILD_DIR:=$(BUILD_DIR)/$(PKG_NAME)$(BUILD_VARIANT)-$(PKG_VERSION)
include $(INCLUDE_DIR)/package-version-override.mk
include $(INCLUDE_DIR)/package.mk


PKG_CMN_NAME:=fwcommon
PKG_CMN_SOURCE:=qca-wifi-fw-src-component-cmn-$(PKG_VERSION).tgz

define Download/fwcommon
  FILE:=$(PKG_CMN_SOURCE)
  VERSION:=$(PKG_VERSION)
  URL:=$(PKG_SOURCE_URL)
endef
$(eval $(call Download,fwcommon))

PKG_HALPHY_NAME:=halphy
PKG_HALPHY_SOURCE:=qca-wifi-fw-src-component-halphy_tools-$(PKG_VERSION).tgz

define Download/halphy
  FILE:=$(PKG_HALPHY_SOURCE)
  VERSION:=$(PKG_VERSION)
  URL:=$(PKG_SOURCE_URL)
endef
$(eval $(call Download,halphy))

PKG_FW_CMN:=qca-wifi-fw-src-component-cmn-$(PKG_VERSION).tgz
PKG_HALPHY_SOURCE:=qca-wifi-fw-src-component-halphy_tools-$(PKG_VERSION).tgz

define Build/Prepare
	$(TAR) xzvf $(TOPDIR)/dl/$(PKG_FW_CMN) -C $(TOPDIR)/dl
	$(TAR) xzvf $(TOPDIR)/dl/$(PKG_HALPHY_SOURCE) -C $(TOPDIR)/dl
endef

define WifiFirmware
 define Package/$(PKG_NAME)$(1)
    SECTION:=QCA
    CATEGORY:=QCA Proprietary software
    URL:=http://www.qca.qualcomm.com
    MAINTAINER:=Qualcomm Atheros
    TITLE:=QCA Wifi Firmware$(1)
    VARIANT:=$(2)
    PKGARCH:=all
  endef

  define Package/$(PKG_NAME)$(1)/description
    This package contains the qca-wifi firmware used in 10.x drivers
  endef

  define Package/$(PKG_NAME)$(1)/config
  $(call Package/$(PKG_NAME)/override_source_path,$(PKG_NAME)$(1))
  $(call Package/$(PKG_NAME)/override_version,$(PKG_NAME)$(1),$(PKG_SUPPORTED_VERSION))
  endef


  define Package/$(PKG_NAME)$(1)/install
	$(INSTALL_DIR) $$(1)/lib/firmware/$(3)
	$(HOST_TAR) jxvf $(DL_DIR)/$(PKG_SOURCE) -C $$(1)/lib/firmware/$(3) --strip-components=1
  endef


 $$(eval $$(call BuildPackage,$(PKG_NAME)$(1)))

endef

Build/Compile=true

$(eval $(call WifiFirmware,-hw5-10.4-asic,-IPQ4019_hw_1,IPQ4019/hw.1))
