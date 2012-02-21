/* linux/drivers/video/samsung/s3cfb_mdnie.c
 *
 * Register interface file for Samsung MDNIE driver
 *
 * Copyright (c) 2009 Samsung Electronics
 * http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/ctype.h>
#include <linux/miscdevice.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fb.h>

#include <linux/io.h>
#include <mach/map.h>
#include <plat/clock.h>
#include <plat/fb.h>
#include <plat/regs-fb.h>
#include <linux/kobject.h>
#include <linux/sysdev.h>
#include "s3cfb.h"
#include "s3cfb_mdnie.h"
#include "s3cfb_ielcd.h"

static struct resource *s3c_mdnie_mem;
static void __iomem *s3c_mdnie_base;

#define s3c_mdnie_readl(addr)             __raw_readl((s3c_mdnie_base + addr))
#define s3c_mdnie_writel(val, addr)        __raw_writel(val, (s3c_mdnie_base + addr))

static char banner[] __initdata = KERN_INFO "S3C MDNIE Driver, (c) 2010 Samsung Electronics\n";

#define 	MDNIE_TUNING

#define TRUE		1
#define FALSE		0

static bool g_mdine_enable;

static char *UI_MODE_FILE;
static char const*const VIDEO_MODE_FILE = VIDEO_MODE_PATH;
static char const*const VIDEO_WARM_MODE_FILE =VIDEO_WARM_MODE_PATH ;
static char const*const VIDEO_WARM_OUTDOOR_MODE_FILE =VIDEO_WARM_OUTDOOR_MODE_PATH ;
static char const*const VIDEO_COLD_MODE_FILE = VIDEO_COLD_MODE_PATH;
static char const*const VIDEO_COLD_OUTDOOR_MODE_FILE = VIDEO_COLD_OUTDOOR_MODE_PATH;
static char const*const CAMERA_MODE_FILE = CAMERA_MODE_PATH;
static char const*const CAMERA_OUTDOOR_MODE_FILE = CAMERA_OUTDOOR_MODE_PATH;
static char const*const GALLERY_MODE_FILE = GALLERY_MODE_PATH;
static char const*const OUTDOOR_MODE_FILE = OUTDOOR_MODE_PATH;
static char const*const STANDARD_MODE_FILE = STANDARD_MODE_PATH;
static char const*const MOVIE_MODE_FILE = MOVIE_MODE_PATH;
static char const*const DYNAMIC_MODE_FILE = DYNAMIC_MODE_PATH;
#ifdef CONFIG_TARGET_LOCALE_KOR
static char const*const DMB_MODE_FILE = DMB_MODE_PATH;
static char const*const  DMB_OUTDOOR_MODE_FILE = DMB_OUTDOOR_MODE_PATH;
static char const*const DMB_WARM_MODE_FILE = DMB_WARM_MODE_PATH;
static char const*const  DMB_WARM_OUTDOOR_MODE_FILE = DMB_WARM_OUTDOOR_MODE_PATH;
static char const*const DMB_COLD_MODE_FILE = DMB_COLD_MODE_PATH;
static char const*const  DMB_COLD_OUTDOOR_MODE_FILE = DMB_COLD_OUTDOOR_MODE_PATH;
#endif	/* CONFIG_TARGET_LOCALE_KOR */

#ifdef CONFIG_TARGET_LOCALE_NTT
static char const*const ISDBT_MODE_FILE = ISDBT_MODE_PATH;
static char const*const ISDBT_OUTDOOR_MODE_FILE = ISDBT_OUTDOOR_MODE_PATH;
static char const*const ISDBT_WARM_MODE_FILE = ISDBT_WARM_MODE_PATH;
static char const*const ISDBT_WARM_OUTDOOR_MODE_FILE = ISDBT_WARM_OUTDOOR_MODE_PATH;
static char const*const ISDBT_COLD_MODE_FILE = ISDBT_COLD_MODE_PATH;
static char const*const ISDBT_COLD_OUTDOOR_MODE_FILE = ISDBT_COLD_OUTDOOR_MODE_PATH;
#endif

int mDNIe_txtbuf_to_parsing(char const*  pFilepath);

static DEFINE_MUTEX(mdnie_use);

struct {
	u16 addr;
	u16 data;
} mDNIe_data_type;

typedef enum {
	mDNIe_UI_MODE,
	mDNIe_VIDEO_MODE,
	mDNIe_VIDEO_WARM_MODE,
	mDNIe_VIDEO_COLD_MODE,
	mDNIe_CAMERA_MODE,
	mDNIe_NAVI,
	mDNIe_GALLERY,
	mDNIe_VT,
#ifdef CONFIG_TARGET_LOCALE_KOR
	mDNIe_DMB_MODE = 20,
	mDNIe_DMB_WARM_MODE,
	mDNIe_DMB_COLD_MODE,
#endif	/* CONFIG_TARGET_LOCALE_KOR */
#ifdef CONFIG_TARGET_LOCALE_NTT
	mDNIe_ISDBT_MODE = 30,
	mDNIe_ISDBT_WARM_MODE,
	mDNIe_ISDBT_COLD_MODE,
#endif
} Lcd_mDNIe_UI;

typedef enum {
	mDNIe_DYNAMIC,
	mDNIe_STANDARD,
	mDNIe_MOVIE,
} Lcd_mDNIe_User_Set;



struct class *mdnieset_ui_class;
struct device *switch_mdnieset_ui_dev;
struct class *mdnieset_outdoor_class;
struct device *switch_mdnieset_outdoor_dev;

Lcd_mDNIe_UI current_mDNIe_Mode = mDNIe_UI_MODE; /* mDNIe Set Status Checking Value.*/
Lcd_mDNIe_User_Set current_mDNIe_user_mode = mDNIe_STANDARD; /*mDNIe_user Set Status Checking Value.*/

u8 current_mDNIe_OutDoor_OnOff = FALSE;

u16 mDNIe_data[100] = {0};

int s3c_mdnie_hw_init(void)
{
	printk(KERN_INFO "MDNIE  INIT ..........\n");

	printk(banner);

	s3c_mdnie_mem = request_mem_region(S3C_MDNIE_PHY_BASE, S3C_MDNIE_MAP_SIZE, "mdnie");
	if (s3c_mdnie_mem == NULL) {
		printk(KERN_ERR "MDNIE: failed to reserved memory region\n");
		return -ENOENT;
	}

	s3c_mdnie_base = ioremap(S3C_MDNIE_PHY_BASE, S3C_MDNIE_MAP_SIZE);
	if (s3c_mdnie_base == NULL) {
		printk(KERN_ERR "MDNIE failed ioremap\n");
		return -ENOENT;
	}

	printk(KERN_INFO "MDNIE  INIT SUCCESS Addr : 0x%p\n", s3c_mdnie_base);

	return 0;
}

int s3c_mdnie_unmask(void)
{
	s3c_mdnie_writel(0x0, S3C_MDNIE_rR40);

	return 0;
}

int s3c_mdnie_set_size(unsigned int hsize, unsigned int vsize)
{
	unsigned int size;

	size = s3c_mdnie_readl(S3C_MDNIE_rR34);
	size &= ~S3C_MDNIE_SIZE_MASK;
	size |= hsize;
	s3c_mdnie_writel(size, S3C_MDNIE_rR34);

	s3c_mdnie_unmask();

	size = s3c_mdnie_readl(S3C_MDNIE_rR35);
	size &= ~S3C_MDNIE_SIZE_MASK;
	size |= vsize;
	s3c_mdnie_writel(size, S3C_MDNIE_rR35);

	s3c_mdnie_unmask();

	return 0;
}

int s3c_mdnie_setup(void)
{
	s3c_mdnie_hw_init();
	s3c_ielcd_hw_init();

	return 0;

}

void mDNIe_Set_Mode(Lcd_mDNIe_UI mode, u8 mDNIe_Outdoor_OnOff)
{
	printk("%s\n", __func__);

	if(!g_mdine_enable) {
		printk(KERN_ERR"[mDNIE WARNING] mDNIE engine is OFF. So you cannot set mDnie Mode correctly.\n");
		return;
	}

	switch(current_mDNIe_user_mode){
		case  mDNIe_DYNAMIC:
			UI_MODE_FILE = UI_DYNAMIC_MODE_PATH;
			break;
		case  mDNIe_MOVIE:
			UI_MODE_FILE = UI_MOVIE_MODE_PATH;
			break;
		case  mDNIe_STANDARD:
			UI_MODE_FILE = UI_STANDARD_MODE_PATH;
			break;
		default:
			UI_MODE_FILE = UI_STANDARD_MODE_PATH;
			printk(KERN_ERR"[mDNIE WARNING] cannot UI_MODE_FILE path change.\n");
			break;
		}
	if (mDNIe_Outdoor_OnOff) {
		switch (mode) {
		case mDNIe_UI_MODE:
		case mDNIe_VT:
			mDNIe_txtbuf_to_parsing(UI_MODE_FILE);
			break;

		case mDNIe_VIDEO_MODE:
			mDNIe_txtbuf_to_parsing(OUTDOOR_MODE_FILE);
			break;

		case mDNIe_VIDEO_WARM_MODE:
			mDNIe_txtbuf_to_parsing(VIDEO_WARM_OUTDOOR_MODE_FILE);
			break;

		case mDNIe_VIDEO_COLD_MODE:
			mDNIe_txtbuf_to_parsing(VIDEO_COLD_OUTDOOR_MODE_FILE);
			break;

		case mDNIe_CAMERA_MODE:
			mDNIe_txtbuf_to_parsing(CAMERA_OUTDOOR_MODE_FILE);
			break;

		case mDNIe_NAVI:
			mDNIe_txtbuf_to_parsing(OUTDOOR_MODE_FILE);
			break;

		case mDNIe_GALLERY:
			mDNIe_txtbuf_to_parsing(GALLERY_MODE_FILE);
			break;

#ifdef CONFIG_TARGET_LOCALE_KOR
		case mDNIe_DMB_MODE:
			mDNIe_txtbuf_to_parsing(DMB_OUTDOOR_MODE_FILE);
			break;

		case mDNIe_DMB_WARM_MODE:
			mDNIe_txtbuf_to_parsing(DMB_WARM_OUTDOOR_MODE_FILE);
			break;

		case mDNIe_DMB_COLD_MODE:
			mDNIe_txtbuf_to_parsing(DMB_COLD_OUTDOOR_MODE_FILE);
			break;
#endif /* CONFIG_TARGET_LOCALE_KOR */
#ifdef CONFIG_TARGET_LOCALE_NTT
		case mDNIe_ISDBT_MODE:
			mDNIe_txtbuf_to_parsing(ISDBT_OUTDOOR_MODE_FILE);
			break;

		case mDNIe_ISDBT_WARM_MODE:
			mDNIe_txtbuf_to_parsing(ISDBT_WARM_OUTDOOR_MODE_FILE);
			break;

		case mDNIe_ISDBT_COLD_MODE:
			mDNIe_txtbuf_to_parsing(ISDBT_COLD_OUTDOOR_MODE_FILE);
			break;
#endif

		}

		current_mDNIe_Mode = mode;

		if (current_mDNIe_Mode == mDNIe_UI_MODE)
			current_mDNIe_OutDoor_OnOff = FALSE;
		else
			current_mDNIe_OutDoor_OnOff = TRUE;
	} else {
		switch (mode) {
		case mDNIe_UI_MODE:
		case mDNIe_VT:
			mDNIe_txtbuf_to_parsing(UI_MODE_FILE);
			break;

		case mDNIe_VIDEO_MODE:
			mDNIe_txtbuf_to_parsing(VIDEO_MODE_FILE);
			break;

		case mDNIe_VIDEO_WARM_MODE:
			mDNIe_txtbuf_to_parsing(VIDEO_WARM_MODE_FILE);
			break;

		case mDNIe_VIDEO_COLD_MODE:
			mDNIe_txtbuf_to_parsing(VIDEO_COLD_MODE_FILE);
			break;

		case mDNIe_CAMERA_MODE:
			mDNIe_txtbuf_to_parsing(CAMERA_MODE_FILE);
			break;

		case mDNIe_NAVI:
			mDNIe_txtbuf_to_parsing(UI_MODE_FILE);
			break;

		case mDNIe_GALLERY:
			mDNIe_txtbuf_to_parsing(GALLERY_MODE_FILE);
			break;

#ifdef CONFIG_TARGET_LOCALE_KOR
		case mDNIe_DMB_MODE:
			mDNIe_txtbuf_to_parsing(DMB_MODE_FILE);
			break;

		case mDNIe_DMB_WARM_MODE:
			mDNIe_txtbuf_to_parsing(DMB_WARM_MODE_FILE);
			break;

		case mDNIe_DMB_COLD_MODE:
			mDNIe_txtbuf_to_parsing(DMB_COLD_MODE_FILE);
			break;
#endif /* CONFIG_TARGET_LOCALE_KOR */
#ifdef CONFIG_TARGET_LOCALE_NTT
		case mDNIe_ISDBT_MODE:
			mDNIe_txtbuf_to_parsing(ISDBT_MODE_FILE);
			break;

		case mDNIe_ISDBT_WARM_MODE:
			mDNIe_txtbuf_to_parsing(ISDBT_WARM_MODE_FILE);
			break;

		case mDNIe_ISDBT_COLD_MODE:
			mDNIe_txtbuf_to_parsing(ISDBT_COLD_MODE_FILE);
			break;
#endif
		}

		current_mDNIe_Mode = mode;
		current_mDNIe_OutDoor_OnOff = FALSE;
	}

	printk(KERN_ERR "[mDNIe] mDNIe_Set_Mode : Current_mDNIe_mode (%d), current_mDNIe_OutDoor_OnOff(%d)\n", current_mDNIe_Mode, current_mDNIe_OutDoor_OnOff);
}

void mDNIe_User_Select_Mode(Lcd_mDNIe_User_Set mode)
{
	printk("%s\n", __func__);

	if(!g_mdine_enable) {
		printk(KERN_ERR"[mDNIE WARNING] mDNIE engine is OFF. So you cannot set mDnie Mode correctly.\n");
		return;
	}

	switch (mode) {
	case mDNIe_DYNAMIC:
		mDNIe_txtbuf_to_parsing(DYNAMIC_MODE_FILE);
		break;

	case mDNIe_STANDARD:
		mDNIe_txtbuf_to_parsing(STANDARD_MODE_FILE);
		break;

	case mDNIe_MOVIE:
		mDNIe_txtbuf_to_parsing(MOVIE_MODE_FILE);
		break;
	}
	current_mDNIe_user_mode = mode;
	printk(KERN_ERR "[mDNIe] mDNIe_user_select_Mode : User_mDNIe_Setting_Mode (%d), Current_mDNIe_mode(%d)\n", current_mDNIe_user_mode,current_mDNIe_Mode);

}

void mDNIe_init_Mode_Set(Lcd_mDNIe_User_Set mode)
{
	printk("%s\n", __func__);

	if(!g_mdine_enable) {
		printk(KERN_ERR" [mDNIE WARNING] mDNIE engine is OFF. So you cannot set mDnie Mode correctly.\n");
		return;
	}
	mDNIe_User_Select_Mode(current_mDNIe_user_mode);
	mDNIe_Set_Mode(current_mDNIe_Mode, current_mDNIe_OutDoor_OnOff);
}


void mDNIe_Init_Set_Mode(void)
{
	mDNIe_init_Mode_Set(current_mDNIe_user_mode);
}

void mDNIe_Mode_Set(void)
{
	mDNIe_Set_Mode(current_mDNIe_Mode, current_mDNIe_OutDoor_OnOff);
}

static ssize_t mdnieset_ui_file_cmd_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int mdnie_ui = 0;

	printk(KERN_INFO "called %s \n", __func__);

	mdnie_ui = current_mDNIe_Mode;

	return sprintf(buf, "%u\n", mdnie_ui);
}

static ssize_t mdnieset_ui_file_cmd_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);

	/*printk(KERN_INFO "[mdnie set] in mdnieset_ui_file_cmd_store, input value = %d \n",value);*/

	current_mDNIe_Mode = value;

	mDNIe_User_Select_Mode(current_mDNIe_user_mode);
	mDNIe_Set_Mode(current_mDNIe_Mode, current_mDNIe_OutDoor_OnOff);

	return size;
}

static DEVICE_ATTR(mdnieset_ui_file_cmd, 0664, mdnieset_ui_file_cmd_show, mdnieset_ui_file_cmd_store);

static ssize_t mdnieset_user_select_file_cmd_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int mdnie_ui = 0;

	printk(KERN_INFO "called %s \n", __func__);

	switch (current_mDNIe_user_mode) {
	case mDNIe_DYNAMIC:
		mdnie_ui = 0;
		break;
	case mDNIe_STANDARD:
		mdnie_ui = 1;
		break;
	case mDNIe_MOVIE:
		mdnie_ui = 2;
		break;
	}
	return sprintf(buf, "%u\n", mdnie_ui);

}

static ssize_t mdnieset_user_select_file_cmd_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);
	/*printk(KERN_INFO "[mdnie set] in mdnieset_ui_file_cmd_store, input value = %d \n",value);*/
	switch (value) {
	case SIG_MDNIE_DYNAMIC:
		current_mDNIe_user_mode = mDNIe_DYNAMIC;
		break;

	case SIG_MDNIE_STANDARD:
		current_mDNIe_user_mode = mDNIe_STANDARD;
		break;

	case SIG_MDNIE_MOVIE:
		current_mDNIe_user_mode = mDNIe_MOVIE;
		break;


	default:
		printk(KERN_ERR "mdnieset_user_select_file_cmd_store value is wrong : value(%d)\n", value);
		break;
	}
	mDNIe_User_Select_Mode(current_mDNIe_user_mode);
	mDNIe_Set_Mode(current_mDNIe_Mode, current_mDNIe_OutDoor_OnOff);

	return size;
}

static DEVICE_ATTR(mdnieset_user_select_file_cmd, 0664, mdnieset_user_select_file_cmd_show, mdnieset_user_select_file_cmd_store);


static ssize_t mdnieset_init_file_cmd_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char temp[15];

	printk(KERN_INFO "called %s \n", __func__);
	sprintf(temp, "mdnieset_init_file_cmd_show \n");
	strcat(buf, temp);

	return strlen(buf);
}

static ssize_t mdnieset_init_file_cmd_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);
	switch (value) {
	case 0:
		current_mDNIe_user_mode = mDNIe_STANDARD;
		current_mDNIe_Mode =mDNIe_UI_MODE;
		break;

	default:
		printk(KERN_ERR "mdnieset_init_file_cmd_store value is wrong : value(%d)\n", value);
		break;
	}
	mDNIe_User_Select_Mode(current_mDNIe_user_mode);
	mDNIe_Set_Mode(current_mDNIe_Mode, current_mDNIe_OutDoor_OnOff);

	return size;
}

static DEVICE_ATTR(mdnieset_init_file_cmd, 0664, mdnieset_init_file_cmd_show, mdnieset_init_file_cmd_store);


static ssize_t mdnieset_outdoor_file_cmd_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "called %s \n", __func__);

	return sprintf(buf, "%u\n", current_mDNIe_OutDoor_OnOff);
}

static ssize_t mdnieset_outdoor_file_cmd_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);

	/*
		printk(KERN_INFO "[mdnie set] in mdnieset_outdoor_file_cmd_store, input value = %d \n",value);
	 */

	if (value)
		current_mDNIe_OutDoor_OnOff = TRUE;
	else
		current_mDNIe_OutDoor_OnOff = FALSE;

	mDNIe_Set_Mode(current_mDNIe_Mode, current_mDNIe_OutDoor_OnOff);

	return size;
}

static DEVICE_ATTR(mdnieset_outdoor_file_cmd, 0664, mdnieset_outdoor_file_cmd_show, mdnieset_outdoor_file_cmd_store);

void init_mdnie_class(void)
{
	mdnieset_ui_class = class_create(THIS_MODULE, "mdnieset_ui");
	if (IS_ERR(mdnieset_ui_class))
		pr_err("Failed to create class(mdnieset_ui_class)!\n");

	switch_mdnieset_ui_dev = device_create(mdnieset_ui_class, NULL, 0, NULL, "switch_mdnieset_ui");
	if (IS_ERR(switch_mdnieset_ui_dev))
		pr_err("Failed to create device(switch_mdnieset_ui_dev)!\n");

	if (device_create_file(switch_mdnieset_ui_dev, &dev_attr_mdnieset_ui_file_cmd) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_mdnieset_ui_file_cmd.attr.name);

	if (device_create_file(switch_mdnieset_ui_dev, &dev_attr_mdnieset_user_select_file_cmd) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_mdnieset_user_select_file_cmd.attr.name);

	if (device_create_file(switch_mdnieset_ui_dev, &dev_attr_mdnieset_init_file_cmd) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_mdnieset_init_file_cmd.attr.name);

	mdnieset_outdoor_class = class_create(THIS_MODULE, "mdnieset_outdoor");
	if (IS_ERR(mdnieset_outdoor_class))
		pr_err("Failed to create class(mdnieset_outdoor_class)!\n");

	switch_mdnieset_outdoor_dev = device_create(mdnieset_outdoor_class, NULL, 0, NULL, "switch_mdnieset_outdoor");
	if (IS_ERR(switch_mdnieset_outdoor_dev))
		pr_err("Failed to create device(switch_mdnieset_outdoor_dev)!\n");

	if (device_create_file(switch_mdnieset_outdoor_dev, &dev_attr_mdnieset_outdoor_file_cmd) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_mdnieset_outdoor_file_cmd.attr.name);
}
EXPORT_SYMBOL(init_mdnie_class);

void mDNIe_tuning_set(void)
{
	u32 i = 0;

	while (mDNIe_data[i] != END_SEQ) {
		s3c_mdnie_writel(mDNIe_data[i+1], mDNIe_data[i]);
		//printk(KERN_INFO "[mDNIe] mDNIe_tuning_initialize: addr(0x%x), data(0x%x)  \n", mDNIe_data[i], mDNIe_data[i+1]);
		i += 2;
	}
	s3c_mdnie_unmask();
}

static int parse_text(char *src, int len)
{
	int i, count, ret;
	int index = 0;
	char *str_line[100];
	char *sstart;
	char *c;
	unsigned int data1, data2;

	c = src;
	count = 0;
	sstart = c;

	for (i = 0; i < len; i++, c++) {
		char a = *c;
		if (a == '\r' || a == '\n') {
			if (c > sstart) {
				str_line[count] = sstart;
				count++;
			}
			*c = '\0';
			sstart = c+1;
		}
	}

	if (c > sstart) {
		str_line[count] = sstart;
		count++;
	}

	//printk(KERN_INFO "----------------------------- Total number of lines:%d\n", count);

	for (i = 0; i < count; i++) {
		//printk(KERN_INFO "line:%d, [start]%s[end]\n", i, str_line[i]);
		ret = sscanf(str_line[i], "0x%x,0x%x\n", &data1, &data2);
		//printk(KERN_INFO "Result => [0x%2x 0x%4x] %s\n", data1, data2, (ret == 2) ? "Ok" : "Not available");
		if (ret == 2) {
			mDNIe_data[index++] = (u16)data1 * 4;
			mDNIe_data[index++]  = (u16)data2;
		}
	}
	return index;
}


int mDNIe_txtbuf_to_parsing(char const*  pFilepath)
{
	struct file *filp;
	char	*dp;
	long	l;
	loff_t  pos;
	int     ret, num;
	mm_segment_t fs;

	mutex_lock(&mdnie_use);

	fs = get_fs();
	set_fs(get_ds());

	if(!pFilepath){
		printk(KERN_ERR "Error : mDNIe_txtbuf_to_parsing has invalid filepath.\n");
		goto parse_err;
	}

	filp = filp_open(pFilepath, O_RDONLY, 0);

	if (IS_ERR(filp)) {
		printk(KERN_ERR "file open error:%d\n", (s32)filp);
		goto parse_err;
	}

	l = filp->f_path.dentry->d_inode->i_size;
	dp = kmalloc(l+10, GFP_KERNEL);		/* add cushion : origianl code is 'dp = kmalloc(l, GFP_KERNEL);' */
	if (dp == NULL) {
		printk(KERN_INFO "Out of Memory!\n");
		filp_close(filp, current->files);
		goto parse_err;
	}
	pos = 0;
	memset(dp, 0, l);
	ret = vfs_read(filp, (char __user *)dp, l, &pos);   /* P1_LSJ : DE08 : died at here  */

	if (ret != l) {
		printk(KERN_INFO "<CMC623> Failed to read file (ret = %d)\n", ret);
		kfree(dp);
		filp_close(filp, current->files);
		goto parse_err;
	}

	filp_close(filp, current->files);
	set_fs(fs);
	num = parse_text(dp, l);
	if (!num) {
		printk(KERN_ERR "Nothing to parse!\n");
		kfree(dp);
		goto parse_err;
	}

	mDNIe_data[num] = END_SEQ;
	mDNIe_tuning_set();

	kfree(dp);

	mutex_unlock(&mdnie_use);
	num = num >> 1;
	return num;

parse_err:
	mutex_unlock(&mdnie_use);
	return -1;
}

int s3c_mdnie_init_global(struct s3cfb_global *s3cfb_ctrl)
{
	s3c_mdnie_set_size(s3cfb_ctrl->lcd->width, s3cfb_ctrl->lcd->height);
	s3c_ielcd_logic_start();
	s3c_ielcd_init_global(s3cfb_ctrl);

	return 0;
}

int s3c_mdnie_start(struct s3cfb_global *ctrl)
{
	/* s3c_ielcd_set_clock(ctrl); */
	s3c_ielcd_start();

	g_mdine_enable = 1;

	return 0;
}

int s3c_mdnie_off(void)
{
	g_mdine_enable = 0;

	s3c_ielcd_logic_stop();

	return 0;
}


int s3c_mdnie_stop(void)
{
	g_mdine_enable = 0;

	return s3c_ielcd_stop();
}


MODULE_AUTHOR("lsi");
MODULE_DESCRIPTION("S3C MDNIE Device Driver");
MODULE_LICENSE("GPL");
