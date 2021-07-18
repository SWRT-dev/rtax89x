/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include "commonmhitest.h"
#include<linux/platform_device.h>
#include<linux/device.h>
#include<linux/sched.h>
#define DEFAULT_FW_FILE_NAME	"qcn9000/amss.bin"
#define HST_FW_FILE_NAME	"amssh.bin"/*for test purpose in same folder*/
int mhitest_ss_powerup(const struct subsys_desc *subsys_desc)
{
	int ret;
	struct mhitest_platform *temp;

	pr_mhitest2("-Start\n");
	if (!subsys_desc->dev) {
		pr_mhitest("Error mhitest_ss_power ..\n");
		return -ENODEV;
	}
	temp = (struct mhitest_platform *)subsys_desc->dev->driver_data;
	if (!temp) {
		pr_mhitest("Error not dev data..\n");
		return -EINVAL;
	}
	pr_mhitest2("temp:[%p]\n", temp);

	if (!temp->pci_dev) {
		pr_mhitest2("temp->pci_Dev is NUlll\n");
		return -ENODEV;
	}
	/*
	* Giving HST amss.bin different name so that  HST and other device
	* amss.bin  do not conflict
	* TODO: move it to device specific folder.
	*/
	if (temp->device_id == QCA6390_DEVICE_ID)
		snprintf(temp->fw_name, sizeof(temp->fw_name),
							HST_FW_FILE_NAME);
	else
		snprintf(temp->fw_name, sizeof(temp->fw_name),
							DEFAULT_FW_FILE_NAME);

	ret = mhitest_prepare_pci_mhi_msi(temp);
	if (ret) {
		pr_mhitest("Error prep. pci_mhi_msi  ret:%d\n", ret);
		goto out;
	}

	ret = mhitest_prepare_start_mhi(temp);
	if (ret) {
		pr_mhitest("Error ..preapare start mhi  ret:%d\n", ret);
		goto out;
	}

	pr_mhitest2("-End\n");
	return 0;
out:
/*	mhitest_subsystem_unregister(temp); */

	return ret;
}

int mhitest_ss_shutdown(const struct subsys_desc *desc, bool force_stop)
{

	struct mhitest_platform *mplat = (struct mhitest_platform *)desc->dev->driver_data;
	pr_mhitest2("Going for shutdown...\n");

	mhitest_pci_set_mhi_state(mplat, MHI_POWER_OFF);
//	msleep(1000);
	mhitest_pci_set_mhi_state(mplat, MHI_DEINIT);
	mhitest_pci_remove_all(mplat);
	return 0;
}
void mhitest_ss_crash_shutdown(const struct subsys_desc *mhitest_ss_desc)
{
	struct mhitest_platform *temp;
	int crash_d_instance = -1;
	int ret;

	if (!strcmp(mhitest_ss_desc->name, "mhitest-ss-0"))
		crash_d_instance = 0;
	else if (!strcmp(mhitest_ss_desc->name, "mhitest-ss-1"))
		crash_d_instance = 1;
	temp = get_mhitest_mplat(crash_d_instance);

	pr_mhitest2("Going for shutdown... start temp:%p\n", temp);
	if (!temp)
		return;
	if ((strcmp(temp->mhitest_ss_desc.name, mhitest_ss_desc->name))) {
		pr_mhitest2("Error not a same subsystem\n");
		return;
	}
	ret = mhitest_dump_info(temp, true);
	if (ret) {
		pr_mhitest2("Error :ret:%d\n", ret);
		return;
	}
	pr_mhitest2("Shutdown... End temp:%p\n", temp);

}

int mhitest_ss_ramdump(int enable, const struct subsys_desc *desc)
{
	pr_mhitest2("Going for ss_ramdump...returning 1\n");
	return 1;
}

int mhitest_subsystem_register(struct mhitest_platform *mplat)
{
	int ret = 1;
	struct subsys_desc *mhitest_ss_desc;

	pr_mhitest2("Going for ss_reg.\n");
	mhitest_ss_desc = &mplat->mhitest_ss_desc;

	/*
	* check device id and give subsystem nameif ()
	* pine-0x1104 HST-0x1101
	*/
	if (mplat->d_instance == 0)
		mhitest_ss_desc->name = "mhitest-ss-0";
	else
		mhitest_ss_desc->name = "mhitest-ss-1";

	mhitest_ss_desc->owner = THIS_MODULE;
	mhitest_ss_desc->powerup = mhitest_ss_powerup;
	mhitest_ss_desc->shutdown = mhitest_ss_shutdown;
	mhitest_ss_desc->crash_shutdown = mhitest_ss_crash_shutdown;
	mhitest_ss_desc->ramdump = mhitest_ss_ramdump;
	mhitest_ss_desc->dev = &mplat->plat_dev->dev;
	if (mplat->device_id == QCA6390_DEVICE_ID)
		snprintf(mhitest_ss_desc->fw_name, sizeof(mplat->fw_name),
							HST_FW_FILE_NAME);
	else
		snprintf(mhitest_ss_desc->fw_name, sizeof(mplat->fw_name),
							DEFAULT_FW_FILE_NAME);
	pr_mhitest("SS name :%s and ss_desc->fw_name:%s\n", mplat->mhitest_ss_desc.name, mhitest_ss_desc->fw_name);
	if (!mhitest_ss_desc->dev) {
		pr_mhitest("dev is null\n");
		ret = -ENODEV;
		goto error;
	} else {
		if (!mhitest_ss_desc->dev->of_node) {
			pr_mhitest("of node is null\n");
			ret = -ENODEV;
			goto error;
		}
	}
	mplat->mhitest_ss_device = subsys_register(&mplat->mhitest_ss_desc);
		if (IS_ERR(mplat->mhitest_ss_device)) {
			ret = PTR_ERR(mplat->mhitest_ss_device);
		pr_mhitest2("Error! SS reg ret:%d\n", ret);
		goto error;
	}


	mplat->subsys_handle = subsystem_get(mhitest_ss_desc->name);
	if (!mplat->subsys_handle) {
		pr_mhitest2("Error: ss_handle NULL\n");
		ret = -EINVAL;
		goto error;
	} else if (IS_ERR(mplat->subsys_handle)) {
		ret = PTR_ERR(mplat->subsys_handle);
		pr_mhitest2("Error SS get ret:%d\n", ret);
		goto error;
	}
	pr_mhitest2("Pass SS get ss_handle:[%p]\n", mplat->subsys_handle);

	return 0;
error:
	return ret;
}

void mhitest_subsystem_unregister(struct mhitest_platform *mplat)
{
	if (!mplat) {
		pr_mhitest("mplat is null no subsystem unregister\n");
		return;
	}
	if (!mplat->mhitest_ss_device) {
		pr_mhitest("mplat->mhitest_ss_device--NULL\n");
		return;
	}
	if (mplat->subsys_handle)
		subsystem_put(mplat->subsys_handle);
	mplat->subsys_handle = NULL;
	subsys_unregister(mplat->mhitest_ss_device);
	mplat->mhitest_ss_device = NULL;
}

