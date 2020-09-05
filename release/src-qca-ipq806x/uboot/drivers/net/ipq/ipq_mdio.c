/*
 * Copyright (c) 2012 - 2014 The Linux Foundation. All rights reserved.
 */

#include <common.h>
#include <miiphy.h>
#include <phy.h>
#include "ipq_gmac.h"
#include <asm/arch-ipq806x/nss/ipq_mdio.h>
#include <asm/arch-ipq806x/nss/msm_ipq806x_gmac.h>

uint ipq_mdio_read(struct mii_dev *bus, uint phy_addr, uint reg_offset, ushort *data)
{
	uint reg_base = bus->gmac_base;
	uint timeout = MII_MDIO_TIMEOUT;
	uint miiaddr;
	uint start;
	uint ret_val;

	if (reg_base != NSS_GMAC0_BASE && reg_base != NSS_GMAC1_BASE &&
	    reg_base != NSS_GMAC2_BASE && reg_base != NSS_GMAC3_BASE)
	{
		printf("%s: bus %s, invalid gmac_base 0x%08x\n", __func__, bus->name, reg_base);
		reg_base = NSS_GMAC0_BASE;
	}

	miiaddr = (((phy_addr << MIIADDRSHIFT) & MII_ADDRMSK) |
	((reg_offset << MIIREGSHIFT) & MII_REGMSK));

	miiaddr |= (MII_BUSY | MII_CLKRANGE_250_300M);
	writel(miiaddr, (reg_base + MII_ADDR_REG_ADDR));
	udelay(10);

	start = get_timer(0);
	while (get_timer(start) < timeout) {
		if (!(readl(reg_base + MII_ADDR_REG_ADDR) & MII_BUSY)) {
			ret_val = readl(reg_base + MII_DATA_REG_ADDR);
			if (data != NULL)
				*data = ret_val;
			return ret_val;
		}
		udelay(1000);
	}
	return -1;
}

uint ipq_mdio_write(struct mii_dev *bus, uint phy_addr, uint reg_offset, ushort data)
{
	uint reg_base = bus->gmac_base;
	const uint timeout = MII_MDIO_TIMEOUT;
	uint miiaddr;
	uint start;

	if (reg_base != NSS_GMAC0_BASE && reg_base != NSS_GMAC1_BASE &&
	    reg_base != NSS_GMAC2_BASE && reg_base != NSS_GMAC3_BASE)
	{
		printf("%s: bus %s, invalid gmac_base 0x%08x\n", __func__, bus->name, reg_base);
		reg_base = NSS_GMAC0_BASE;
	}

	writel(data, (reg_base + MII_DATA_REG_ADDR));

	miiaddr = (((phy_addr << MIIADDRSHIFT) & MII_ADDRMSK) |
			((reg_offset << MIIREGSHIFT) & MII_REGMSK) |
			(MII_WRITE));

	miiaddr |= (MII_BUSY | MII_CLKRANGE_250_300M);
	writel(miiaddr, (reg_base + MII_ADDR_REG_ADDR));
	udelay(10);

	start = get_timer(0);
	while (get_timer(start) < timeout) {
		if (!(readl(reg_base + MII_ADDR_REG_ADDR) & MII_BUSY)) {
			return 0;
		}
		udelay(1000);
	}
	return -1;
}

int ipq_phy_read(struct mii_dev *bus, int addr, int dev_addr, int regnum)
{
	return ipq_mdio_read(bus, addr, regnum, NULL);
}

int ipq_phy_write(struct mii_dev *bus, int addr, int dev_addr, int regnum,
			ushort value)
{
	return ipq_mdio_write(bus, addr, regnum, value);
}

int ipq_phy_mdio_init(char *name)
{
	struct mii_dev *bus = mdio_alloc();
	int gmac = *(name + strlen("IPQ MDIO")) - '0';

	if(!bus) {
		printf("Failed to allocate IPQ MDIO bus\n");
		return -1;
	}

	bus->read = ipq_phy_read;
	bus->write = ipq_phy_write;
	bus->reset = NULL;
	snprintf(bus->name, sizeof(bus->name), name);
	bus->gmac_base = NSS_GMAC0_BASE;

#if defined(CONFIG_SWITCH_RTL8370M_PHY_QCA8033_X2) || \
    defined(CONFIG_SWITCH_RTL8370MB_PHY_QCA8033_X2)
	if (gmac == CONFIG_PHY0_GMAC || gmac == CONFIG_PHY1_GMAC) {
		/* WAN0/1 PHY use 2-nd MDC/MDIO which belongs to GMAC1. */
		bus->gmac_base = NSS_GMAC1_BASE;
	}
#endif

	return mdio_register(bus);
}
