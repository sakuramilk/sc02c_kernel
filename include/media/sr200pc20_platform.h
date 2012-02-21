/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#define DEFAULT_FMT		V4L2_PIX_FMT_YUYV
#define DEFAULT_PREVIEW_WIDTH		800
#define DEFAULT_PREVIEW_HEIGHT		600
#define DEFAULT_CAPTURE_WIDTH		1600 //1280 raj@samsung.com 20110930
#define DEFAULT_CAPTURE_HEIGHT		1200 //960   raj@samsung.com 20110930

struct sr200pc20_platform_data {
	unsigned int default_width;
	unsigned int default_height;
	unsigned int pixelformat;
	int freq;	/* MCLK in KHz */
	/* This SoC supports Parallel & CSI-2 */
	int is_mipi;
};

