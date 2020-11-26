#undef CONFIG_BOOTCOMMAND
#undef CONFIG_IPQ_ATAG_PART_LIST
#ifndef CONFIG_SYS_FLASH_BASE
#define CONFIG_SYS_FLASH_BASE	0xC0000000	/* define fake flash address. */
#endif
#define CONFIG_SYS_BOOTM_LEN	(70 << 20)	/* 70 MB */

#define CONFIG_EXPECTED_MACHID	(0x8010000)
#define CONFIG_QCA8075_PHYBASE	(0x8)

/*
 * ASUS configuration.
 * All CONFIG_XXX will be copied to include/autoconf.mk automatically.
 */
#define SPFVER			110	/* 8: SPF8, 11 or 111: SPF11.1, 110: SPF11.0 */
#define CONFIG_MODEL		"RT-AX89U"
#define CONFIG_FLASH_TYPE	"nand"
#define CONFIG_BLS_FIT_IMAGE
#define CONFIG_ECC_THRESHOLD	(3)	/* per-page bit-flips threshold. */

#define CONFIG_AQRMODEL		107	/* 113 A1,B0 or 107 */

/* Set 1-st version number in accordance with SPF version. */
#if SPFVER == 11 || SPFVER == 111
#define CONFIG_METATOOLDIR	"build_ipq"
#define KV1C			"2"
#elif SPFVER == 110
#define CONFIG_METATOOLDIR	"build_spf11.0"
#define KV1C			"2"
#elif SPFVER == 8
#define CONFIG_METATOOLDIR	"build_spf8"
#define KV1C			"1"
#else
#define CONFIG_METATOOLDIR	"build_ipq"
#define KV1C			"0"
#endif

/* Set 2-nd version number in accordance with Aquentia PHY chip. */
#if CONFIG_AQRMODEL == 113
#define KV2C			"2"
#elif CONFIG_AQRMODEL == 107
#define KV2C			"1"
#else
#define KV2C			"0"
#endif

#define CONFIG_UBI_SUPPORT
#if SPFVER == 111
#define CONFIG_BLVER		KV1C KV2C "21"
#else
#define CONFIG_BLVER		KV1C KV2C "16"
#endif
#define CONFIG_DUAL_BAND
#define CONFIG_HAVE_WAN_RED_LED
#undef CONFIG_CMD_NFS		        /* NFS support */
#undef CONFIG_CMD_DHCP

#define CONFIG_SYS_LOAD_ADDR	0x4B000000
#define CONFIG_SYS_LONGHELP
#define CONFIG_LZMA
#define CONFIG_CMDLINE_TAG

#define CONFIG_QCA8337
#define CONFIG_AQR_PHYADDR	(7)

/*
 * #define CONFIG_RTAX89U_OLD_SR1	// PCB R1.00
 * #define CONFIG_RTAX89U_OLD_ER1	// PCB R3.00
 * #define CONFIG_RTAX89U_OLD_PR1	// PCB R3.50
 */

#define XMK_STR(x)	#x
#define MK_STR(x)	XMK_STR(x)

/*
 * Environment variables.
 */
#define CONFIG_IPADDR		192.168.50.1
#define CONFIG_SERVERIP		192.168.50.75
#define CONFIG_NETMASK		255.255.255.0
#define CONFIG_BOOTFILE		CONFIG_MODEL ".trx"		/* RT-AX89U.trx */
#define CONFIG_BOOTCOMMAND	"tftp"
#define CONFIG_ETHADDR		00:aa:bb:cc:dd:e0
#define CONFIG_EXTRA_ENV_SETTINGS	\
	"imgaddr="MK_STR(CONFIG_SYS_LOAD_ADDR)"\0"

/*
 * Enable commands
 */

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
#define RAMAC0_OFFSET	0x1014	/* 2G+5G EEPROM, second set of MAC address. */
#define RAMAC1_OFFSET	0x100E	/* 2G+5G EEPROM, first set of MAC address. */
#define FTRY_PARM_SHIFT                 (0x40000)
#define OFFSET_PIN_CODE			(FTRY_PARM_SHIFT + 0xD180)	/* 8 bytes */
#define OFFSET_COUNTRY_CODE		(FTRY_PARM_SHIFT + 0xD188)	/* 2 bytes */
#define OFFSET_BOOT_VER			(FTRY_PARM_SHIFT + 0xD18A)	/* 4 bytes */
#define OFFSET_HWID			(FTRY_PARM_SHIFT + 0x0FF00)	/* 4 bytes */

/*-----------------------------------------------------------------------
 * Bootloader size and Config size definitions
 */
#define CONFIG_MAX_BL_BINARY_SIZE	0x3E0000			/* 4MiB - 128KiB */
#define CFG_BOOTLOADER_SIZE		CONFIG_MAX_BL_BINARY_SIZE
#define CFG_MAX_BOOTLOADER_BINARY_SIZE	CONFIG_MAX_BL_BINARY_SIZE
#define CONFIG_ENV_SIZE			0x20000
#define CONFIG_SYS_MALLOC_LEN		(8 << 20)
#define CONFIG_ALTERNATE_FLASH_SIZE	(128 * 1048576UL)


#define MTDIDS				"nand0=nand0"
/* 1. Keep Bootloader size and environment size equal to CFG_BOOTLOADER_SIZE and CONFIG_ENV_SIZE respectively.
 * 2. Make sure all partitions defined in nand/nor-system-partption.*.bin is not beyond CFG_MAX_BOOTLOADER_BINARY_SIZE + CONFIG_ENV_SIZE.
 */
#define MTDPARTS			"mtdparts=nand0:3968k(Bootloader),128k(environment),-(UBI_DEV)"

/*
 * UBI volume size definitions
 * Don't define size for tailed reserved space due to it's size varies.
 */
#define PEB_SIZE			(128 * 1024)
#define LEB_SIZE			(PEB_SIZE - (2 * 2 * 1024))
#define CFG_UBI_NVRAM_NR_LEB		3
#define CFG_UBI_FACTORY_NR_LEB		3
#define CFG_UBI_FIRMWARE_NR_LEB		826	/* 124KB x 826 = 100.02MB */
#define CFG_UBI_FIRMWARE2_NR_LEB	826
#define CFG_UBI_APP_NR_LEB		1162	/* 124KB x 1162 = 140.7MB. This volume size cannot reach requested size due to UBI's overhead. */
#if defined(CONFIG_ALTERNATE_FLASH_SIZE)
#define CFG_UBI_ALT_FIRMWARE_NR_LEB	397	/* 124KB x 397 = 48.074MB */
#define CFG_UBI_ALT_FIRMWARE2_NR_LEB	397
#define CFG_UBI_ALT_APP_NR_LEB		219	/* 124KB x 666 = 27.37MB. This volume size cannot reach requested size due to UBI's overhead. */
#endif

#define CFG_UBI_NVRAM_SIZE		(LEB_SIZE * CFG_UBI_NVRAM_NR_LEB)
#define CFG_UBI_FACTORY_SIZE		(LEB_SIZE * CFG_UBI_FACTORY_NR_LEB)
#define CFG_UBI_FACTORY2_SIZE		(LEB_SIZE * CFG_UBI_FACTORY_NR_LEB)
#define CFG_UBI_FIRMWARE_SIZE		(LEB_SIZE * CFG_UBI_FIRMWARE_NR_LEB)
#define CFG_UBI_FIRMWARE2_SIZE		(LEB_SIZE * CFG_UBI_FIRMWARE2_NR_LEB)
#define CFG_UBI_APP_SIZE		(LEB_SIZE * CFG_UBI_APP_NR_LEB)
#if defined(CONFIG_ALTERNATE_FLASH_SIZE)
#define CFG_UBI_ALT_FIRMWARE_SIZE	(LEB_SIZE * CFG_UBI_ALT_FIRMWARE_NR_LEB)
#define CFG_UBI_ALT_FIRMWARE2_SIZE	(LEB_SIZE * CFG_UBI_ALT_FIRMWARE2_NR_LEB)
#define CFG_UBI_ALT_APP_SIZE		(LEB_SIZE * CFG_UBI_ALT_APP_NR_LEB)
#endif

#define CFG_NVRAM_SIZE			CFG_UBI_NVRAM_SIZE

#define CFG_FACTORY_SIZE		(CFG_UBI_FACTORY_SIZE + CFG_UBI_FACTORY2_SIZE)

#define CFG_UBI_DEV_OFFSET		(CFG_BOOTLOADER_SIZE + CONFIG_ENV_SIZE)

/* Environment address, factory address, and firmware address definitions */
/* Basically, CFG_FACTORY_ADDR and CFG_KERN_ADDR are used to compatible to original code infrastructure.
 * Real nvram area would be moved into the nvram volume of UBI device.
 * Real Factory area would be moved into the Factory volume of UBI device.
 * Real firmware area would be moved into the linux and linux2 volume of UBI device.
 */
#define CONFIG_ENV_ADDR			(CONFIG_SYS_FLASH_BASE + CFG_BOOTLOADER_SIZE)
#define CFG_FACTORY_ADDR		(CONFIG_SYS_FLASH_BASE + CFG_BOOTLOADER_SIZE + CONFIG_ENV_SIZE + CFG_NVRAM_SIZE)
#define CFG_KERN_ADDR			(CONFIG_SYS_FLASH_BASE + (CFG_BOOTLOADER_SIZE + CONFIG_ENV_SIZE + CFG_NVRAM_SIZE + CFG_FACTORY_SIZE))
#define CFG_KERN2_ADDR			(CONFIG_SYS_FLASH_BASE + (CFG_BOOTLOADER_SIZE + CONFIG_ENV_SIZE + CFG_NVRAM_SIZE + CFG_FACTORY_SIZE + CFG_UBI_FIRMWARE_SIZE))
#if defined(CONFIG_ALTERNATE_FLASH_SIZE)
#define CFG_ALT_KERN2_ADDR		(CONFIG_SYS_FLASH_BASE + (CFG_BOOTLOADER_SIZE + CONFIG_ENV_SIZE + CFG_NVRAM_SIZE + CFG_FACTORY_SIZE + CFG_UBI_ALT_FIRMWARE_SIZE))
#endif
/*-----------------------------------------------------------------------*/


/* Include header files of demo board. */
#include <configs/ipq807x.h>
