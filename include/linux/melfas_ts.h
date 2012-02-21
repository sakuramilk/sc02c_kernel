/*
 * include/linux/melfas_ts.h - platform data structure for MCS Series sensor
 *
 * Copyright (C) 2010 Melfas, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_MELFAS_TS_H
#define _LINUX_MELFAS_TS_H

#define MELFAS_TS_NAME "melfas-ts"

struct melfas_version {
	uint8_t tsp_revision;	
	uint8_t hardware;
	uint8_t compatibility;
	uint8_t core;
	uint8_t private;
	uint8_t public;
	uint8_t product_code;
};

struct melfas_tsi_platform_data {
	int x_size;
	int y_size;
	struct melfas_version *version;
	int (*power)(int on);
};


#endif /* _LINUX_MELFAS_TS_H */
