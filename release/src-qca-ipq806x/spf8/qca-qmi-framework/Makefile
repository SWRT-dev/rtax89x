PBBINS=$(wildcard prebuilt/*)
QCA_GPL_LIBS=$(filter-out %.conf,$(PBBINS))

all:

install:
	@[ -d $(INSTALLDIR)/usr/lib ] || install -d $(INSTALLDIR)/usr/lib
	@for i in $(QCA_GPL_LIBS); do install -m 755 -D $$i $(INSTALLDIR)/usr/lib ; done

clean:

stage:
