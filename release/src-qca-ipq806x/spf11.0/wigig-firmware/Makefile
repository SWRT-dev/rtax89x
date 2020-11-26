PBBINS=$(wildcard prebuilt/*)
WIL6436=$(filter prebuilt/wil6436%,$(PBBINS))

all:

install: all
	@mkdir -p $(foreach d,$(INSTALLDIR)/lib/firmware,$(d))
	@install $(foreach f,$(WIL6436),$(f)) $(INSTALLDIR)/lib/firmware

clean:
