/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/spi/spi.h>
#include "giga_spinand.h"

/* Only ecc un-protected fields in the spare area included */
static struct nand_ecclayout winbond_oob_64 = {
	.eccbytes = 32,
	.eccpos = {
		8, 9, 10, 11, 12, 13, 14, 15, 16,
		24, 25, 26, 27, 28, 29, 30, 31, 32,
		40, 41, 42, 43, 44, 45, 46, 47, 48,
		56, 57, 58, 59, 60, 61, 62, 63, 64},
	.oobavail = 8,
	.oobfree = {
		{.offset = 2,  .length = 2},
		{.offset = 18, .length = 2},
		{.offset = 34, .length = 2},
		{.offset = 50, .length = 2},
	}
};

/* Only ecc un-protected fields in the spare area included */
/* ECC parity code stored in the additional hidden spare area */
static struct nand_ecclayout macronix_oob_64 = {
	.eccbytes = 0,
	.eccpos = {},
	.oobfree = {
		{.offset = 2,  .length = 2},
		{.offset = 18, .length = 2},
		{.offset = 34, .length = 2},
		{.offset = 50, .length = 2},
	}
};

void gigadevice_set_defaults(struct spi_device *spi_nand)
{
	struct mtd_info *mtd = (struct mtd_info *)dev_get_drvdata
						(&spi_nand->dev);
	struct nand_chip *chip = (struct nand_chip *)mtd->priv;

	chip->ecc.size	= 0x800;
	chip->ecc.bytes	= 0x0;
	chip->ecc.steps	= 0x0;

	chip->ecc.strength = 1;
	chip->ecc.total	= 0;
	chip->ecc.layout = NULL;
}

void gigadevice_set_defaults_512mb(struct spi_device *spi_nand)
{
	struct mtd_info *mtd = (struct mtd_info *)dev_get_drvdata
						(&spi_nand->dev);
	struct nand_chip *chip = (struct nand_chip *)mtd->priv;

	chip->ecc.size	= 0x1000;
	chip->ecc.bytes	= 0x0;
	chip->ecc.steps	= 0x0;

	chip->ecc.strength = 1;
	chip->ecc.total	= 0;
	chip->ecc.layout = NULL;
}

void winbond_set_defaults(struct spi_device *spi_nand)
{
	struct mtd_info *mtd = dev_get_drvdata(&spi_nand->dev);
	struct nand_chip *chip = (struct nand_chip *)mtd->priv;

	chip->ecc.size	= 0x800;
	chip->ecc.bytes	= 0x0;
	chip->ecc.steps	= 0x0;

	chip->ecc.strength = 1;
	chip->ecc.total	= 0;
	chip->ecc.layout = &winbond_oob_64;
}

void macronix_set_defaults(struct spi_device *spi_nand)
{
	struct mtd_info *mtd = dev_get_drvdata(&spi_nand->dev);
	struct nand_chip *chip = (struct nand_chip *)mtd->priv;

	chip->ecc.size  = 0x800;
	chip->ecc.bytes = 0x0;
	chip->ecc.steps = 0x0;

	chip->ecc.strength = 1;
	chip->ecc.total = 0;
	chip->ecc.layout = &macronix_oob_64;
}

void gigadevice_read_cmd(struct spinand_cmd *cmd, u32 page_id)
{
	cmd->addr[0] = (u8)(page_id >> 16);
	cmd->addr[1] = (u8)(page_id >> 8);
	cmd->addr[2] = (u8)(page_id);
}

void toshiba_read_cmd(struct spinand_cmd *cmd, u32 page_id)
{
	cmd->addr[0] = ((u8)(page_id >> 16) % 2);
	cmd->addr[1] = (u8)(page_id >> 8);
	cmd->addr[2] = (u8)(page_id);
}

void gigadevice_read_data(struct spinand_cmd *cmd, u16 column, u32 page_id)
{
	cmd->addr[0] = 0xff; /*dummy byte*/
	cmd->addr[1] = (u8)(column >> 8);
	cmd->addr[2] = (u8)(column);
}

void macronix_read_data(struct spinand_cmd *cmd, u16 column, u32 page_id)
{
	cmd->addr[0] = ((u8)(column >> 8) & MACRONIX_NORM_RW_MASK);
	cmd->addr[1] = (u8)(column);
}

void winbond_read_data(struct spinand_cmd *cmd, u16 column, u32 page_id)
{
	cmd->addr[0] = (u8)(column >> 8);
	cmd->addr[1] = (u8)(column);
}

void toshiba_read_data(struct spinand_cmd *cmd, u16 column, u32 page_id)
{
	cmd->addr[0] = ((u8)(column >> 8) & TOSHIBA_NORM_RW_MASK);
	cmd->addr[1] = (u8)(column);
}

void gigadevice_write_cmd(struct spinand_cmd *cmd, u32 page_id)
{
	cmd->addr[0] = (u8)(page_id >> 16);
	cmd->addr[1] = (u8)(page_id >> 8);
	cmd->addr[2] = (u8)(page_id);
}

void toshiba_write_cmd(struct spinand_cmd *cmd, u32 page_id)
{
	cmd->addr[0] = ((u8)(page_id >> 16) % 2);
	cmd->addr[1] = (u8)(page_id >> 8);
	cmd->addr[2] = (u8)(page_id);
}

void gigadevice_write_data(struct spinand_cmd *cmd, u16 column, u32 page_id)
{
	cmd->addr[0] = (u8)(column >> 8);
	cmd->addr[1] = (u8)(column);
}

void macronix_write_data(struct spinand_cmd *cmd, u16 column, u32 page_id)
{
	cmd->addr[0] = ((u8)(column >> 8) & MACRONIX_NORM_RW_MASK);
	cmd->addr[1] = (u8)(column);
}

void winbond_write_data(struct spinand_cmd *cmd, u16 column, u32 page_id)
{
	cmd->addr[0] = (u8)(column >> 8);
	cmd->addr[1] = (u8)(column);
}

void toshiba_write_data(struct spinand_cmd *cmd, u16 column, u32 page_id)
{
	cmd->addr[0] = ((u8)(column >> 8) & TOSHIBA_NORM_RW_MASK);
	cmd->addr[1] = (u8)(column);
}

void gigadevice_erase_blk(struct spinand_cmd *cmd, u32 page_id)
{
	cmd->addr[0] = (u8)(page_id >> 16);
	cmd->addr[1] = (u8)(page_id >> 8);
	cmd->addr[2] = (u8)(page_id);
}

void toshiba_erase_blk(struct spinand_cmd *cmd, u32 page_id)
{
	cmd->addr[0] = (u8)((page_id >> 16) % 2);
	cmd->addr[1] = (u8)(page_id >> 8);
	cmd->addr[2] = (u8)(page_id);
}

int gigadevice_verify_ecc(u8 status)
{
	int ecc_status = (status & STATUS_ECC_MASK_GIGA);

	if (ecc_status == STATUS_ECC_ERROR_GIGA)
		return SPINAND_ECC_ERROR;
	else if (ecc_status >= STATUS_ECC_BF_THRESHOLD_GIGA)
		return SPINAND_ECC_CORRECTED;
	else
		return 0;
}

int macronix_verify_ecc(u8 status)
{
	int ecc_status = (status & STATUS_ECC_MASK_MACRONIX);

	if ((ecc_status == STATUS_ECC_ERROR_MACRONIX) ||
	    (ecc_status == STATUS_ECC_MASK_MACRONIX))
		return SPINAND_ECC_ERROR;
	else if (ecc_status)
		return SPINAND_ECC_CORRECTED;
	else
		return 0;
}

int toshiba_verify_ecc(u8 status)
{
	int ecc_status = (status & STATUS_ECC_MASK_TOSHIBA);

	if (ecc_status == STATUS_ECC_ERROR_TOSHIBA)
		return SPINAND_ECC_ERROR;
	else if (ecc_status == STATUS_ECC_BF_THRESHOLD_TOSHIBA)
		return SPINAND_ECC_CORRECTED;
	else
		return 0;
}

int dummy_verify_ecc(u8 status)
{
	return 0;
}

int gigadevice_parse_id(struct spi_device *spi_nand,
			struct spinand_ops *ops, u8 *nand_id, u8 *id)
{
	if (nand_id[0] != NAND_MFR_GIGA && nand_id[0] != NAND_MFR_ATO)
		return -EINVAL;

	if (!(nand_id[0] == NAND_MFR_GIGA && nand_id[1] == ops->dev_id))
		return -EINVAL;

	id[0] = nand_id[0];
	id[1] = nand_id[1];

	return 0;
}

int macronix_parse_id(struct spi_device *spi_nand,
		      struct spinand_ops *ops, u8 *nand_id, u8 *id)
{
	if (nand_id[1] != NAND_MFR_MACRONIX)
		return -EINVAL;

	return 0;
}

int winbond_parse_id(struct spi_device *spi_nand,
		     struct spinand_ops *ops, u8 *nand_id, u8 *id)
{
	if (!(nand_id[1] == NAND_MFR_WINBOND && nand_id[2] == ops->dev_id))
		return -EINVAL;

	return 0;
}

int toshiba_parse_id(struct spi_device *spi_nand,
		     struct spinand_ops *ops, u8 *nand_id, u8 *id)
{
	if (!(nand_id[1] == NAND_MFR_TOSHIBA && nand_id[2] == ops->dev_id))
		return -EINVAL;

	return 0;
}

MODULE_DESCRIPTION("SPI NAND driver for Gigadevice and Macronix");
