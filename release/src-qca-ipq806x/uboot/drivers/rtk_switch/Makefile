
include $(TOPDIR)/config.mk

CFLAGS += -DMDC_MDIO_OPERATION -D_LITTLE_ENDIAN
# -Idrivers/char 
# -Idrivers/char/rtl8370/include

LIB	:= $(obj)librtk_switch.o

COBJS-y	+= rtl8370.o

# obj-y	+= smi.o
# obj-y	+= rtk_api.o
# obj-y	+= rtl8370_asicdrv_acl.o
# obj-y	+= rtl8370_asicdrv.o
# obj-y	+= rtl8370_asicdrv_cputag.o
# obj-y	+= rtl8370_asicdrv_dot1x.o
# obj-y	+= rtl8370_asicdrv_eee.o
# obj-y	+= rtl8370_asicdrv_eeelldp.o
# obj-y	+= rtl8370_asicdrv_fc.o
# obj-y	+= rtl8370_asicdrv_green.o
# obj-y	+= rtl8370_asicdrv_igmp.o
# obj-y	+= rtl8370_asicdrv_inbwctrl.o
# obj-y	+= rtl8370_asicdrv_interrupt.o
# obj-y	+= rtl8370_asicdrv_led.o
# obj-y	+= rtl8370_asicdrv_lut.o
# obj-y	+= rtl8370_asicdrv_meter.o
# obj-y	+= rtl8370_asicdrv_mib.o
# obj-y	+= rtl8370_asicdrv_mirror.o
# obj-y	+= rtl8370_asicdrv_misc.o
# obj-y	+= rtl8370_asicdrv_oam.o
# obj-y	+= rtl8370_asicdrv_phy.o
# obj-y	+= rtl8370_asicdrv_port.o
# obj-y	+= rtl8370_asicdrv_portIsolation.o
# obj-y	+= rtl8370_asicdrv_qos.o
# obj-y	+= rtl8370_asicdrv_rldp.o
# obj-y	+= rtl8370_asicdrv_rma.o
# obj-y	+= rtl8370_asicdrv_rrcp.o
# obj-y	+= rtl8370_asicdrv_scheduling.o
# obj-y	+= rtl8370_asicdrv_specialCongest.o
# obj-y	+= rtl8370_asicdrv_storm.o
# obj-y	+= rtl8370_asicdrv_svlan.o
# obj-y	+= rtl8370_asicdrv_trunking.o
# obj-y	+= rtl8370_asicdrv_unknownMulticast.o
# obj-y	+= rtl8370_asicdrv_vlan.o

COBJS	:= $(COBJS-y)
SRCS 	:= $(COBJS:.o=.c)
OBJS 	:= $(addprefix $(obj),$(COBJS))

all:	$(LIB)

$(LIB):	$(obj).depend $(OBJS)
	$(call cmd_link_o_target, $(OBJS))


#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################
