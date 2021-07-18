INSTALLKMODDIR:=$(INSTALLDIR)/lib/modules/$(LINUX_KERNEL)
PBBINS=$(wildcard prebuilt/*)
QCAWIGIGDRV_KMOD=$(filter %.ko,$(PBBINS))

all:

install: all
	@mkdir -p $(foreach d,$(INSTALLKMODDIR),$(d))
	@install -D $(foreach f,$(QCAWIGIGDRV_KMOD),$(f)) $(INSTALLKMODDIR)
	@$(STRIPX) $(INSTALLKMODDIR)/*.ko

clean:

