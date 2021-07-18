PBBINS=$(wildcard prebuilt/*)
QCA_GPL_BINS=$(filter-out %.conf,$(PBBINS))

all:

install:
	@[ -d $(INSTALLDIR)/usr/bin ] || install -d $(INSTALLDIR)/usr/bin
	@for i in $(QCA_GPL_BINS); do install -m 755 -D $$i $(INSTALLDIR)/usr/bin ; done
	@$(STRIP) $(INSTALLDIR)/usr/bin/*
	@mkdir -p $(INSTALLDIR)/data/misc/
	@ln -sf /tmp/wifi $(INSTALLDIR)/data/misc/wifi

clean:

stage:
