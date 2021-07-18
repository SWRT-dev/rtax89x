PBBINS:=$(wildcard prebuilt/*)
AQR_FW:=$(filter %.cld,$(PBBINS))
AQR_EXEC:=$(filter-out $(AQR_FW),$(PBBINS))
$(info AQR_FW $(AQR_FW) AQR_EXEC $(AQR_EXEC))

all:

install: all
	@[ -d $(INSTALLDIR)/sbin ] || install -d $(INSTALLDIR)/sbin
	@[ -d $(INSTALLDIR)/lib/firmware ] || install -d $(INSTALLDIR)/lib/firmware
	@for i in $(AQR_EXEC); do install -m 755 -D $$i $(INSTALLDIR)/sbin ; done
	@for i in $(AQR_FW); do install -m 755 -D $$i $(INSTALLDIR)/lib/firmware ; done
	@$(STRIP) $(INSTALLDIR)/sbin/*

clean:
