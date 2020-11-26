PBBINS=$(wildcard prebuilt/*)
QCACFG80211_LIBS=$(filter %.so,$(PBBINS))
QCACFG80211_SBINS=$(filter-out $(QCACFG80211_LIBS),$(PBBINS))

all:

install: all
	@mkdir -p $(foreach d,$(INSTALLDIR)/usr/lib $(INSTALLDIR)/usr/sbin,$(d))
	@install $(foreach f,$(QCACFG80211_SBINS),$(f)) $(INSTALLDIR)/usr/sbin
	@install $(foreach f,$(QCACFG80211_LIBS),$(f)) $(INSTALLDIR)/usr/lib
	@$(STRIP) $(INSTALLDIR)/usr/sbin/*

clean:

stage:
