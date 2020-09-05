#include <common.h>
#include <asm/io.h>
#include <errno.h>
#include <malloc.h>

#define HIGH_ADDR_DFLT  0x200

extern int ipq_mdio_write(int mii_id, int regnum, u16 value);
extern int ipq_mdio_read(int mii_id, int regnum, ushort *data);

static void wait_for_page_switch(void)
{
	mdelay(2);
}

static inline void split_addr(u32 regaddr, u16 * r1, u16 * r2, u16 * page)
{
	regaddr >>= 1;
	*r1 = regaddr & 0x1e;

	regaddr >>= 5;
	*r2 = regaddr & 0x7;

	regaddr >>= 3;
	*page = regaddr & 0x1ff;
}

static u32 ar8xxx_mii_read32(int phy_id, int regnum)
{
	u16 lo, hi;

	lo = ipq_mdio_read(phy_id, regnum, NULL);
	hi = ipq_mdio_read(phy_id, regnum + 1, NULL);

	return (hi << 16) | lo;
}

static void ar8xxx_mii_write32(int phy_id, int regnum, u32 val)
{
	const int mii_lo_first = 0;
	u16 lo, hi;

	lo = val & 0xffff;
	hi = (u16) (val >> 16);

	if (mii_lo_first) {
		ipq_mdio_write(phy_id, regnum, lo);
		ipq_mdio_write(phy_id, regnum + 1, hi);
	} else {
		ipq_mdio_write(phy_id, regnum + 1, hi);
		ipq_mdio_write(phy_id, regnum, lo);
	}
}

u32 ar8xxx_read(int reg)
{
	u16 r1, r2, page;
	u32 val;

	split_addr((u32) reg, &r1, &r2, &page);

	ipq_mdio_write(0x18, 0, page);
	wait_for_page_switch();
	val = ar8xxx_mii_read32(0x10 | r2, r1);
	ipq_mdio_write(0x18, 0, HIGH_ADDR_DFLT);


	return val;
}

void ar8xxx_write(int reg, u32 val)
{
	u16 r1, r2, page;

	split_addr((u32) reg, &r1, &r2, &page);


	ipq_mdio_write(0x18, 0, page);
	wait_for_page_switch();
	ar8xxx_mii_write32(0x10 | r2, r1, val);
	ipq_mdio_write(0x18, 0, HIGH_ADDR_DFLT);

}

static int do_q8337(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	char *cmd;
	uint32_t reg, value;

	if (argc < 2)
		goto usage;

	cmd = argv[1];
	if (*cmd == 'r' || *cmd == 'R') {
		if (argc < 3)
			goto usage;

		reg = simple_strtoul(argv[2], NULL, 16);
		value = ar8xxx_read(reg);
		printf("R: reg 0x%05x = 0x%08x\n", reg, value);
	}
	else if (*cmd == 'w' || *cmd == 'W') {
		if (argc < 3)
			goto usage;

		reg = simple_strtoul(argv[2], NULL, 16);
		value = simple_strtoul(argv[3], NULL, 16);
		ar8xxx_write(reg, value);
		printf("W: reg 0x%05x = 0x%08x\n", reg, value);
	}

	return 0;

usage:
	return CMD_RET_USAGE;
}

#ifdef CONFIG_SYS_LONGHELP
static char q8337_help_text[] =
	"q8337 read - reg\n"
	"q8337 write - reg val\n"
	"";
#endif

U_BOOT_CMD(
	q8337, CONFIG_SYS_MAXARGS, 1, do_q8337,
	"Read/Write QCA8337 register", q8337_help_text
);
