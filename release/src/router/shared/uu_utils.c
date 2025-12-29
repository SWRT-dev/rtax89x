/*
* Copyright 2023, ASUSTeK Inc.
* All Rights Reserved.
*
* This is UNPUBLISHED PROPRIETARY SOURCE CODE of ASUSTeK Inc.;
* the contents of this file may not be disclosed to third parties, copied
* or duplicated in any form, in whole or in part, without the prior
* written permission of ASUSTeK Inc..
*/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <ctype.h>
#include <bcmnvram.h>

#include "shutils.h"
#include "shared.h"


#ifdef RTCONFIG_UUPLUGIN
/*
 * Add uu_model_check() API for rc/private.c and libwebapi/priv_webapi.c
 * */
int uu_model_check()
{
	int model = get_model();

	switch(model) {
		case MODEL_RTAC82U:
			if (strcmp(get_productid(), "RT-AC2200"))
				return 0;
			break;
		case MODEL_RTAX58U:
			if (!strcmp(get_productid(), "RT-AX58U"))
				return 0;
			break;
		case MODEL_RTAX82U_V2:
			if (strcmp(get_productid(), "RT-AX82U"))
				return 0;
			break;
		case MODEL_TUFAX5400_V2:
			if (strcmp(get_productid(), "TUF-AX5400"))
				return 0;
			break;
		case MODEL_RTAX5400:
			if (strcmp(get_productid(), "RT-AX5400"))
				return 0;
			break;
		case MODEL_RTAX82_XD6S:
		case MODEL_XD6_V2:
			if (strcmp(get_productid(), "ZenWiFi_XD6"))
				return 0;
			break;
		case MODEL_RTAC68U:
			if (!strcmp(get_productid(), "RP-AC1900"))
				return 0;
			break;
	}

	return 1;
}
#endif

#ifdef RTCONFIG_GEARUPPLUGIN
/*
 * gu_support_status() : check gearup_support to decide the UI hidden or visible
 * */
int gu_support_status()
{
	int support = nvram_get_int("gearup_support");
	int ret = 0;

	switch(support) {
		case GU_HIDDEN:
		case GU_VISIBLE:
			ret = support;
			break;
		default:
			// gu_support is out of range, force to hidden
			ret = GU_HIDDEN;
	}
	GUDBG("ret=%d\n", ret);
	return ret;
}

/*
 * gu_enable_status() : check gearup_enable to decide the feature disable/enable/forbidden
 * */
int gu_enable_status()
{
	int enable = nvram_get_int("gearup_enable");
	int block = nvram_get_int("gearup_block");
	int ret = 0;

	switch(enable) {
		case GU_DISABLED:
		case GU_ENABLED:
			ret = enable;
			break;
		default:
			// gu_enable is out of range, force to disable
			ret = GU_DISABLED;
	}

	// if status is block, we force to remove binary
	if (block) ret = GU_BLOCK;
	GUDBG("enable=%d, block=%d, ret=%d\n", enable, block, ret);
	return ret;
}
#endif
