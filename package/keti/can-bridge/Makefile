include $(TOPDIR)/rules.mk

PKG_NAME:=can-bridge
PKG_VERSION:=1.0
PKG_RELEASE:=1

PKG_LICENSE:=GPL-2.0-or-later
PKG_MAINTAINER:=KETI

include $(INCLUDE_DIR)/package.mk

define Package/can-bridge
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=Bridge a SocketCAN interface onto the network (read-only by default)
  DEPENDS:=+kmod-can +kmod-can-raw +kmod-can-usb-gs +kmod-usb-core \
	+ip-full
endef

define Package/can-bridge/description
 Needs ip-full: BusyBox's ip cannot configure a CAN link at all - it answers
 "type is garbage" - so the default install could bring up a bus only by
 accident. That was found by booting this under QEMU, not by reading it.

 Forwards CAN frames from a SocketCAN interface to a UDP peer and keeps a
 decoded snapshot of selected ids for a dashboard. Injection from the network
 onto the bus is refused unless explicitly enabled, because a wireless hop does
 not belong in a motion control loop. MT7621 has no CAN controller, so this
 expects a USB-CAN adapter on the gs_usb/candleLight driver.
endef

define Package/can-bridge/conffiles
/etc/config/can-bridge
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(TARGET_CC) $(TARGET_CFLAGS) $(TARGET_CPPFLAGS) $(TARGET_LDFLAGS) \
		-o $(PKG_BUILD_DIR)/can-bridge \
		$(PKG_BUILD_DIR)/can-bridge.c $(PKG_BUILD_DIR)/agilex.c
endef

define Package/can-bridge/install
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/can-bridge $(1)/usr/sbin/
	$(INSTALL_BIN) ./files/can-up $(1)/usr/sbin/can-up
	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_BIN) ./files/can-bridge.init $(1)/etc/init.d/can-bridge
	$(INSTALL_DIR) $(1)/etc/config
	$(INSTALL_CONF) ./files/can-bridge.config $(1)/etc/config/can-bridge
	$(INSTALL_DIR) $(1)/etc/hotplug.d/net
	$(INSTALL_BIN) ./files/can-bridge.hotplug \
		$(1)/etc/hotplug.d/net/30-can-bridge
endef

$(eval $(call BuildPackage,can-bridge))
