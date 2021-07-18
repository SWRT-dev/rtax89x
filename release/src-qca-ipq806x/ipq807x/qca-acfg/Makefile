PBBINS=$(wildcard prebuilt/*)
QCA_GPL_BINS=$(filter-out %.conf,$(PBBINS))
QCA_GPL_CONF=$(filter %.conf,$(PBBINS))

all:

install:
	@[ -d $(INSTALLDIR)/usr/sbin ] || install -d $(INSTALLDIR)/usr/sbin
	@for i in $(QCA_GPL_BINS); do install -m 755 -D $$i $(INSTALLDIR)/usr/sbin ; done
	@$(STRIP) $(INSTALLDIR)/usr/sbin/*
	@[ -d $(INSTALLDIR)/etc ] || install -d $(INSTALLDIR)/etc
	@for i in $(QCA_GPL_CONF); do install -m 644 -D $$i $(INSTALLDIR)/etc ; done

clean:

stage:
