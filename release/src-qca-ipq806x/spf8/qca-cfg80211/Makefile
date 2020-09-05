PBBINS=$(wildcard prebuilt/*)
QCA_GPL_SBINS=$(filter %,$(PBBINS))

all:

install: all
	@[ -d $(INSTALLDIR)/usr/sbin ] || install -d $(INSTALLDIR)/usr/sbin
	@for i in $(QCA_GPL_SBINS); do install -m 755 -D $$i $(INSTALLDIR)/usr/sbin ; done
	@$(STRIP) $(INSTALLDIR)/usr/sbin/*

clean:

stage:
