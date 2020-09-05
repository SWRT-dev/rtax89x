PBBINS=$(wildcard prebuild/*)
LIBS=$(filter %.so,$(PBBINS))

all:

install:
	@[ -d $(INSTALLDIR)/usr/lib ] || install -d $(INSTALLDIR)/usr/lib
	@for i in $(LIBS); do install -D $$i $(INSTALLDIR)/usr/lib ; done
	@$(STRIP) $(INSTALLDIR)/usr/lib/*

clean:

stage:
