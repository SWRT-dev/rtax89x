INSTALLKMODDIR:=$(INSTALLDIR)/lib/modules/$(LINUX_KERNEL)
PBBINS=$(wildcard prebuilt/*)
NSS_DP_KMOD=$(filter %.ko,$(PBBINS))
NSS_DP_EXEC=$(filter-out %.ko,$(PBBINS))

all:

install:
	@[ -d $(INSTALLKMODDIR) ] || install -d $(INSTALLKMODDIR)
	@for i in $(NSS_DP_KMOD); do install -D $$i $(INSTALLKMODDIR) ; done
	@$(STRIPX) $(INSTALLKMODDIR)/*.ko

clean:

stage:
