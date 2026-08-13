include $(TOPDIR)/rules.mk

PKG_NAME:=agx-cmd
PKG_VERSION:=1.0
PKG_RELEASE:=1

PKG_LICENSE:=GPL-2.0-or-later
PKG_MAINTAINER:=KETI

include $(INCLUDE_DIR)/package.mk

define Package/agx-cmd
  SECTION:=utils
  CATEGORY:=Utilities
  TITLE:=Turn teleop intent into AgileX motion commands (off by default)
  DEPENDS:=+can-bridge
endef

define Package/agx-cmd/description
 The last link in the control chain, and the only thing here that can make a
 vehicle move. It does not open a CAN socket: it emits the same inject datagram
 a laptop would send, so can-bridge remains the single thing that touches the
 bus and its read-only default keeps meaning something.

 Off unless enabled, full stick is walking pace rather than the vehicle's
 maximum, step inputs are ramped, and losing the operator commands zero before
 going quiet. See doc/CAN.md.
endef

define Package/agx-cmd/conffiles
/etc/config/agx-cmd
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
	$(CP) ../can-bridge/src/agilex.c ../can-bridge/src/agilex.h $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(TARGET_CC) $(TARGET_CFLAGS) $(TARGET_CPPFLAGS) $(TARGET_LDFLAGS) \
		-I$(PKG_BUILD_DIR) \
		-o $(PKG_BUILD_DIR)/agx-cmd \
		$(PKG_BUILD_DIR)/agx-cmd.c $(PKG_BUILD_DIR)/agilex.c
endef

define Package/agx-cmd/install
	$(INSTALL_DIR) $(1)/usr/sbin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/agx-cmd $(1)/usr/sbin/
	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_BIN) ./files/agx-cmd.init $(1)/etc/init.d/agx-cmd
	$(INSTALL_DIR) $(1)/etc/config
	$(INSTALL_CONF) ./files/agx-cmd.config $(1)/etc/config/agx-cmd
endef

$(eval $(call BuildPackage,agx-cmd))
