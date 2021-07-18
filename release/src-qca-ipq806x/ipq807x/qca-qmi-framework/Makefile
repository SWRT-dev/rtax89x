PBBINS=$(wildcard prebuilt/*)
QCA_GPL_SBINS=$(filter %/qrtr-ns,$(PBBINS))
QCA_GPL_LIBS=$(filter-out %.conf $(QCA_GPL_SBINS),$(PBBINS))

all:

install:
	@[ -d $(INSTALLDIR)/usr/sbin ] || install -d $(INSTALLDIR)/usr/sbin
	@[ -d $(INSTALLDIR)/usr/lib ] || install -d $(INSTALLDIR)/usr/lib
	@for i in $(QCA_GPL_SBINS); do install -m 755 -D $$i $(INSTALLDIR)/usr/sbin ; done
	@for i in $(QCA_GPL_LIBS); do install -m 755 -D $$i $(INSTALLDIR)/usr/lib ; done

clean:

stage:
