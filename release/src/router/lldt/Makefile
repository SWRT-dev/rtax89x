VPATH = src
LD= $(CC)
OS_LAYER = osl-linux.c

all: lld2d lld2test

include src/common.mk

CFLAGS += -I$(TOP)/shared
CFLAGS += -I$(SRCBASE)/include
LDFLAGS += -L$(TOP)/shared -lshared
LDFLAGS += -L$(TOP)/nvram -lnvram

include $(SRCBASE)/router/.config

depend:
	$(CC) $(CFLAGS) -M $(DCFILES) >.depend

clean:
	rm -f *.o lld2d lld2test

install: all
	@ echo " [lld2d] Installing..."
	install -D lld2d $(INSTALLDIR)/usr/sbin/lld2d
	install -D lld2d.conf $(INSTALLDIR)/rom/etc/lld2d.conf

	# Non-default .ico
ifeq ($(TUFAX4200),y)
	install -D icon.TUF-AX4200Q.ico $(INSTALLDIR)/rom/etc/icon.TUF-AX4200Q.ico
	install -D icon.large.TUF-AX4200Q.ico $(INSTALLDIR)/rom/etc/icon.large.TUF-AX4200Q.ico
endif

ifeq ($(RTN66U), y)
	install -D icon.RT-N66U.ico $(INSTALLDIR)/rom/etc/icon.ico
	install -D icon.large.RT-N66U.ico $(INSTALLDIR)/rom/etc/icon.large.ico
else ifeq ($(RTN56U), y)
	install -D icon.RT-N56U.ico $(INSTALLDIR)/rom/etc/icon.ico
	install -D icon.large.RT-N56U.ico $(INSTALLDIR)/rom/etc/icon.large.ico
else ifeq ($(DSLN55U), y)
	install -D icon.DSL-N55U.ico $(INSTALLDIR)/rom/etc/icon.ico
	install -D icon.large.DSL-N55U.ico $(INSTALLDIR)/rom/etc/icon.large.ico
else ifeq ($(RTN11PB1), y)
	install -D icon.RT-N300.ico $(INSTALLDIR)/rom/etc/icon.ico
	install -D icon.large.RT-N300.ico $(INSTALLDIR)/rom/etc/icon.large.ico
else
	install -D icon.ico $(INSTALLDIR)/rom/etc/icon.ico
	install -D icon.large.ico $(INSTALLDIR)/rom/etc/icon.large.ico
	[ ! -e icon.$(BUILD_NAME).ico ] || install -D icon.$(BUILD_NAME).ico $(INSTALLDIR)/rom/etc/icon.ico
	[ ! -e icon.large.$(BUILD_NAME).ico ] || install -D icon.large.$(BUILD_NAME).ico $(INSTALLDIR)/rom/etc/icon.large.ico
endif
	$(STRIP) $(INSTALLDIR)/usr/sbin/lld2d
	chmod 0500 $(INSTALLDIR)/usr/sbin/lld2d

#-include .depend
