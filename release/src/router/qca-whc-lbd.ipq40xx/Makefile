PBBINS=$(wildcard prebuild/*)
LIBS=$(filter %.so,$(PBBINS))

all:

install:
	@[ -d $(INSTALLDIR)/usr/lib ] || install -d $(INSTALLDIR)/usr/lib
	@for i in $(LIBS); do install -D $$i $(INSTALLDIR)/usr/lib ; done
	@$(STRIP) $(INSTALLDIR)/usr/lib/*
ifeq ($(QCA_LBD),y)
	[ -d $(INSTALLDIR)/usr/sbin ] || install -d $(INSTALLDIR)/usr/sbin
	@install -m 755 $(PBBINS)/lbd $(INSTALLDIR)/usr/sbin
	@$(STRIP) $(INSTALLDIR)/usr/sbin/lbd
endif

clean:

stage:
