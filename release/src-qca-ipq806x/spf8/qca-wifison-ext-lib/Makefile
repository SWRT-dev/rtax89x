PBBINS=$(wildcard prebuilt/*)
WIFISON_EXT_LIBS=$(filter %.so,$(PBBINS))
WIFISON_EXT_SH=$(filtert %.sh,$(PBBINS))
WIFISON_EXT_EXEC=$(filter-out %.so %.sh,$(PBBINS))

all:

install:
	@[ -d $(INSTALLDIR)/usr/sbin ] || install -d $(INSTALLDIR)/usr/sbin
	@[ -d $(INSTALLDIR)/usr/lib ] || install -d $(INSTALLDIR)/usr/lib
	$(if $(WIFISON_EXT_LIBS),install -m 0755 $(foreach lib,$(WIFISON_EXT_LIBS),$(lib)) $(INSTALLDIR)/usr/lib)
	@for i in $(WIFISON_EXT_EXEC); do install -m 755 -D $$i $(INSTALLDIR)/usr/sbin ; done
	@$(STRIP) $(INSTALLDIR)/usr/sbin/*
	@$(STRIP) $(INSTALLDIR)/usr/lib/*

clean:

stage:
