#!/bin/sh

if [ ! -f /opt/openwrt-gcc463.arm/bin/arm-openwrt-linux-gcc ];then
	git clone https://github.com/SWRT-dev/qca-toolchains
	sudo mkdir -p /opt/toolchains/
	sudo ln -sf $(pwd)/openwrt-gcc463.arm /opt/toolchains/
	sudo ln -sf /bin/bash /bin/sh
fi
cd release/src-qca-ipq806x
make rt-ax89u

