#undef CONFIG_BOOTCOMMAND
#undef CONFIG_IPQ_ATAG_PART_LIST
#ifndef CONFIG_SYS_FLASH_BASE
#define CONFIG_SYS_FLASH_BASE	0xC0000000	/* define fake flash address. */
#endif
#define CONFIG_SYS_BOOTM_LEN	(32 << 20)	/* 32 MB */

#define CONFIG_REF_AP148_030

/*
 * ASUS configuration.
 * All CONFIG_XXX will be copied to include/autoconf.mk automatically.
 */
#define CONFIG_ASUS_PRODUCT
#define CONFIG_MODEL		"RT-AC88S"
#define CONFIG_FLASH_TYPE	"nor"
#define CONFIG_BLS_FIT_IMAGE
#define CONFIG_BLVER		"1003"
#define CONFIG_DUAL_BAND
#define CONFIG_UART_GSBI2
//#define CONFIG_SWITCH_RTL8370M_PHY_QCA8033_X2			/* SR1~SR3 */
#define CONFIG_SWITCH_RTL8370MB_PHY_QCA8033_X2			/* SR4 or above */
#define CONFIG_HAVE_WAN_RED_LED

#if defined(CONFIG_SWITCH_RTL8370MB_PHY_QCA8033_X2)
#define CONFIG_PHY0_GMAC	GMAC_UNIT0
#define CONFIG_PHY1_GMAC	GMAC_UNIT3
#else
#define CONFIG_PHY0_GMAC	GMAC_UNIT2
#define CONFIG_PHY1_GMAC	GMAC_UNIT3
#endif

#define CONFIG_SYS_LOAD_ADDR	0x4B000000
#define CONFIG_SYS_LONGHELP
#define CONFIG_LZMA

#define XMK_STR(x)	#x
#define MK_STR(x)	XMK_STR(x)

/*
 * Environment variables.
 */
#define CONFIG_IPADDR		192.168.1.1
#define CONFIG_SERVERIP		192.168.1.75
#define CONFIG_NETMASK		255.255.255.0
#define CONFIG_BOOTFILE		CONFIG_MODEL ".trx"		/* RT-AC88S.trx */
#define CONFIG_BOOTCOMMAND	"tftp"
#define CONFIG_ETHADDR		00:aa:bb:cc:dd:e0
#define CONFIG_EXTRA_ENV_SETTINGS	\
	"imgaddr="MK_STR(CONFIG_SYS_LOAD_ADDR)"\0" \
	"preferred_nic=eth1\0"

/*
 * Enable commands
 */
#define CONFIG_CMD_LOADB

#ifndef __ASSEMBLY__
/* ipq806x_cdp.c */
extern const char *model;
extern const char *blver;
extern const char *bl_stage;

/* board.c */
extern int modifies;
#endif

/*-----------------------------------------------------------------------
 * Factory
 */
#define RAMAC0_OFFSET	0x1006	/* 2G EEPROM */
#define RAMAC1_OFFSET	0x5006	/* 5G EEPROM */

/*-----------------------------------------------------------------------
 * Bootloader size and Config size definitions
 */
#define CONFIG_MAX_BL_BINARY_SIZE	0x180000
#define CFG_BOOTLOADER_SIZE		CONFIG_MAX_BL_BINARY_SIZE
#define CFG_MAX_BOOTLOADER_BINARY_SIZE	CONFIG_MAX_BL_BINARY_SIZE
#define CONFIG_ENV_SIZE			0x10000
#define CONFIG_ENV_SIZE_MAX		CONFIG_ENV_SIZE
#define CONFIG_SYS_MALLOC_LEN           (4 << 20)

#define MTDIDS				"nand0=nand0"
/* Keep Bootloader size and environment size equal to CFG_BOOTLOADER_SIZE and CONFIG_ENV_SIZE respectively. */
#define MTDPARTS			"mtdparts=nand0:3968k(Bootloader),128k(environment),-(UBI_DEV)"

#define CFG_NVRAM_SIZE			0x10000
#define CFG_FACTORY_SIZE		0x10000

/* Environment address, factory address, and firmware address definitions */
/* Basically, CFG_FACTORY_ADDR and CFG_KERN_ADDR are used to compatible to original code infrastructure.
 * U-Boot environment shares same block with NVRAM.
 */
#define CONFIG_ENV_ADDR			(CONFIG_SYS_FLASH_BASE + CFG_BOOTLOADER_SIZE)
#define CFG_FACTORY_ADDR		(CONFIG_SYS_FLASH_BASE + CFG_BOOTLOADER_SIZE + CFG_NVRAM_SIZE)
#define CFG_KERN_ADDR			(CONFIG_SYS_FLASH_BASE + (CFG_BOOTLOADER_SIZE + CFG_NVRAM_SIZE + CFG_FACTORY_SIZE))
/*-----------------------------------------------------------------------*/


/* Include header files of demo board. */
#include <configs/ipq806x_cdp.h>
