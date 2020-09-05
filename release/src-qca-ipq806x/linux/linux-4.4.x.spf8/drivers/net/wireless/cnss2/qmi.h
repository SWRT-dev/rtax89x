/* Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
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

#ifndef _CNSS_QMI_H
#define _CNSS_QMI_H

#include "wlan_firmware_service_v01.h"

#define WLFW_SERVICE_INS_ID_V01_QCA6290		1
#define WLFW_SERVICE_INS_ID_V01_QCA8074		2
#define BDF_MAX_SIZE (256 * 1024)
#define BDF_TYPE_GOLDEN 0
#define BDF_TYPE_CALDATA 2
#define CALDATA_OFFSET(addr) (addr + (128 * 1024))
#define Q6_CALDB_SIZE 0x480000
struct cnss_plat_data;

int cnss_qmi_init(struct cnss_plat_data *plat_priv);
void cnss_qmi_deinit(struct cnss_plat_data *plat_priv);
int cnss_wlfw_server_arrive(struct cnss_plat_data *plat_priv);
int cnss_wlfw_server_exit(struct cnss_plat_data *plat_priv);
int cnss_wlfw_respond_mem_send_sync(struct cnss_plat_data *plat_priv);
int cnss_wlfw_tgt_cap_send_sync(struct cnss_plat_data *plat_priv);
int cnss_wlfw_bdf_dnld_send_sync(struct cnss_plat_data *plat_priv);
int cnss_wlfw_load_bdf(struct wlfw_bdf_download_req_msg_v01 *req,
		struct cnss_plat_data *plat_priv, unsigned int remaining,
		uint8_t is_end, uint8_t bdf_type);
int cnss_wlfw_wlan_mode_send_sync(struct cnss_plat_data *plat_priv,
				  enum wlfw_driver_mode_enum_v01 mode);
int cnss_wlfw_wlan_cfg_send_sync(struct cnss_plat_data *plat_priv,
				 struct wlfw_wlan_cfg_req_msg_v01 *data);
int cnss_wlfw_m3_dnld_send_sync(struct cnss_plat_data *plat_priv);

#endif /* _CNSS_QMI_H */
