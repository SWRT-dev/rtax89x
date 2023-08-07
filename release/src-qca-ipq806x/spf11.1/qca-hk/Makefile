include ../qsdk.config

# Convert asuswrt build environment variable to qca-hk
PKG_BUILD_DIR=$(shell pwd)/source

# Define variables for qca-hk
INSTALLFWDIR:=$(INSTALLDIR)/lib/firmware

BIN2TXT:=	bdfUtil/bdfConvert/bin2txt.py
TXT2BIN:=	bdfUtil/bdfConvert/txt2bin.py
STDTXT:=	bdfUtil/bdfConvert/bdwlan.txt
NEWTXT:=	bdfUtil/bdfConvert/bdwlan_new.txt

MODEL_BIN_FILES := $(filter-out %_new.bin,$(wildcard \
	$(BUILD_NAME)/IPQ8074/bdwlan*.bin $(BUILD_NAME)/IPQ8074/REG_CTL/bdwlan*.bin \
	$(BUILD_NAME)/IPQ8074A/bdwlan*.bin $(BUILD_NAME)/IPQ8074A/REG_CTL/bdwlan*.bin))
MODEL_TXT_FILES := $(MODEL_BIN_FILES:.bin=.txt)
MODEL_PATCH_FILES := $(MODEL_BIN_FILES:.bin=.patch)
MODEL_NEW_BIN_FILES := $(MODEL_BIN_FILES:.bin=_new.bin)
MODEL_NEW_TXT_FILES := $(MODEL_NEW_BIN_FILES:.bin=.txt)

all:
	@#Nothing to do

install:
	install -d $(INSTALLDIR)/vendor/firmware
	[ ! -e $(PKG_BUILD_DIR)/Data.msc ] || install -m 644 $(PKG_BUILD_DIR)/Data.msc $(INSTALLDIR)/vendor/firmware
	for socdir in IPQ8074 IPQ8074A; do \
		install -d $(INSTALLFWDIR)/$${socdir} ; \
		cd $(PKG_BUILD_DIR)/$${socdir}/PIL_IMAGES && rsync -avcH . $(INSTALLFWDIR)/$${socdir} && cd - ; \
		cd $(PKG_BUILD_DIR)/$${socdir} && rsync -avcH bdwlan* $(INSTALLFWDIR)/$${socdir} && cd - ; \
		[ ! -e $(BUILD_NAME)/$${socdir} ] || install -m 644 $(BUILD_NAME)/$${socdir}/bdwlan* $(INSTALLFWDIR)/$${socdir} ; \
	done

clean:
	@#Nothing to do

patch_bdwlan_txt: patch_bdwlan_txt.c
	$(CC) -Wall -o $@ $<

# model-specific bdwlan*.bin ==> bdwlan*.txt in same directory.
%.txt: %.bin
	python2 source/$(subst $(BUILD_NAME)/,,$(subst REG_CTL/,,$(dir $^)))$(BIN2TXT) $< \
		source/$(subst $(BUILD_NAME)/,,$(subst REG_CTL/,,$(dir $^)))$(STDTXT) && \
		mv -f source/$(subst $(BUILD_NAME)/,,$(subst REG_CTL/,,$(dir $^)))$(NEWTXT) $@

# generate difference between model-specific bdwlan*.txt and latest standard bdwlan.txt
%.patch: %.txt
	diff --unified=1 -I "COMMON_BDF_HEADER.baseBdfHeader.checksum" -I "COMMON_BDF_HEADER.baseBdfHeader.boardDataRev" \
		source/$(subst $(BUILD_NAME)/,,$(subst REG_CTL/,,$(dir $^)))$(STDTXT) $< > $@ || echo

# copy latest standard bdwlan.txt to model-specific bdwlan*_new.txt,
# # parsing .patch, seperate keyword from "+TYPE NAME VAL", search "TYPE NAME " in new txt and then modify it
%_new.txt: %.txt
	cp -f source/$(subst $(BUILD_NAME)/,,$(subst REG_CTL/,,$(dir $^)))$(STDTXT) $@
	@if [ -e "$(<:.txt=.patch)" ] && [ -n "`cat $(<:.txt=.patch)`" ] ; then \
		echo "$(<:.txt=.patch) remove `grep \"^-[^-]\" $(<:.txt=.patch)|wc -l` lines" \
			", add `grep \"^+[^+]\" $(<:.txt=.patch)|wc -l` lines. Applying ......"; \
		./patch_bdwlan_txt -p $(<:.txt=.patch) -b $@ ; \
	fi

# convert model-specific bdwlan*_new.txt to model-specific bdwlan*_new.bin
%_new.bin: %_new.txt
	python2 source/$(subst $(BUILD_NAME)/,,$(subst REG_CTL/,,$(dir $^)))$(TXT2BIN) $< $@

.PHONY: bin2txt txt2patch patch2newtxt newtxt2newbin apply_new_bin
bin2txt: $(MODEL_TXT_FILES)
	#touch $(MODEL_BIN_FILES)

txt2patch: $(MODEL_PATCH_FILES)
	#touch $(MODEL_TXT_FILES)

patch2newtxt: patch_bdwlan_txt $(MODEL_NEW_TXT_FILES)
	#touch $(MODEL_TXT_FILES)

newtxt2newbin: $(MODEL_NEW_BIN_FILES)
	#touch $(MODEL_NEW_TXT_FILES)

apply_new_bin:
	@for f in `find . -name "*_new.bin"` ; do \
		mv -f $${f} `echo $${f}|sed -e "s,_new,,"` ; \
	done
