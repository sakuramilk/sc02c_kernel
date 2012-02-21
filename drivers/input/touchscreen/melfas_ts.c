/* drivers/input/touchscreen/melfas_ts.c
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



#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/melfas_ts.h>
#include <linux/gpio.h>
#include <mach/cpufreq.h>


#define TS_MAX_Z_TOUCH			255
#define TS_MAX_W_TOUCH		30


#define TS_MAX_X_COORD 		720
#define TS_MAX_Y_COORD 		1280

#define COMPATIBILITY_VERSION	'B'
#define FW_VERSION				0x66

#define TS_READ_START_ADDR 	0x0F
#define TS_READ_START_ADDR2 	0x10
#define TS_READ_VERSION_ADDR	0xF0

#define TS_READ_REGS_LEN 		66
#define MELFAS_MAX_TOUCH		11

#define DEBUG_PRINT 			0


#if DEBUG_PRINT
#define	tsp_dbg_msg(str, args...)	\
		printk(KERN_ERR "[melfasTSP] %d %s(): " \
				str,__LINE__,  __func__, ##args)
#else
#define	tsp_dbg_msg(str, args...)
#endif

#define SET_DOWNLOAD_BY_GPIO	1


#if SET_DOWNLOAD_BY_GPIO
#include "mcs8000_download.h"
#endif // SET_DOWNLOAD_BY_GPIO


struct muti_touch_info
{
	int strength;
	int width;	
	int posX;
	int posY;
};

struct melfas_ts_data 
{
	uint16_t addr;
	struct i2c_client *client; 
	struct input_dev *input_dev;
	struct melfas_tsi_platform_data *pdata;
	struct melfas_version *version;
	struct work_struct  work;
	uint32_t flags;
	u8	poweron;
	int (*power)(int on);
	struct early_suspend early_suspend;
};

// Merging from Korean Dali - Sysfs related
#define SEC_TSP_SYS_FS_FD

#ifdef SEC_TSP_SYS_FS_FD
#define P5_THRESHOLD 0x05
//#define TS_SYSFS_READ_REGS_LEN		5
#define TS_SYSFS_WRITE_REGS_LEN		16
/*sec_class sysfs*/
extern struct class *sec_class;
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h);
static void melfas_ts_late_resume(struct early_suspend *h);
#endif

static struct muti_touch_info g_Mtouch_info[MELFAS_MAX_TOUCH];
static int melfas_init_panel(struct melfas_ts_data *ts);
static bool touch_cpu_lock_status;

int touch_is_pressed;
EXPORT_SYMBOL(touch_is_pressed);

extern void melfas_power_on(void);
extern void melfas_power_off(void);
// Merging from Korean Dali - Sysfs related
#ifdef SEC_TSP_SYS_FS_FD
static int melfas_ts_resume(struct i2c_client *client);
static int melfas_ts_suspend(struct i2c_client *client, pm_message_t mesg);
static int melfas_i2c_read(struct i2c_client *client, u16 addr, u16 length, u8 *value);
static int melfas_i2c_write(struct i2c_client *client, char *buf, int length);

static bool debug_print = true;
static u16 inspection_data[370] = { 0, };
static u16 lntensity_data[370] = { 0, };

static ssize_t set_tsp_module_on_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	struct melfas_ts_data *ts = (struct melfas_ts_data *)(dev_get_drvdata(dev));

	ret = melfas_ts_resume(ts->client);
	
	if (ret  == 0)
		*buf = '1';
	else
		*buf = '0';

	msleep(500);
	return 0;
}

static ssize_t set_tsp_module_off_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	struct melfas_ts_data *ts = (struct melfas_ts_data *)(dev_get_drvdata(dev));

	ret = melfas_ts_suspend(ts->client, PMSG_SUSPEND);

	if (ret  == 0)
		*buf = '1';
	else
		*buf = '0';

	return 0;
}

static int check_debug_data(struct melfas_ts_data *ts)
{
	u8 write_buffer[6];
	u8 read_buffer[2];
	int sensing_line, exciting_line;
	int ret = 0;
	int gpio = GPIO_TSP_INT;
//	int gpio = ts->client->irq - NR_MSM_IRQS;

	disable_irq(ts->client->irq);

	/* enter the debug mode */

	write_buffer[0] = 0xA0;
	write_buffer[1] = 0x1A;
	write_buffer[2] = 0x0;
	write_buffer[3] = 0x0;
	write_buffer[4] = 0x0;
	write_buffer[5] = 0x01;
	melfas_i2c_write(ts->client, (char *)write_buffer, 6);

	/* wating for the interrupt*/
	while (gpio_get_value(gpio)) { 
		printk(".");
		udelay(100);
	}

	if (debug_print)
		pr_info("[TSP] read dummy\n");

	/* read the dummy data */
	melfas_i2c_read(ts->client, 0xA8, 2, read_buffer);

	if (debug_print)
		pr_info("check_debug_data: [TSP] read inspection data\n");
	write_buffer[5] = 0x03;		// CmABS Test

	for (sensing_line = 0; sensing_line < 14; sensing_line++) {
		tsp_dbg_msg("sensing_line %02d ==> ", sensing_line);
		for (exciting_line =0; exciting_line < 26; exciting_line++) {
			write_buffer[2] = exciting_line;
			write_buffer[3] = sensing_line;
			melfas_i2c_write(ts->client, (char *)write_buffer, 6);
			melfas_i2c_read(ts->client, 0xA8, 2, read_buffer);
			lntensity_data[exciting_line + sensing_line * 26] =
				(read_buffer[1] & 0xf) << 8 | read_buffer[0];
			tsp_dbg_msg("%d.", lntensity_data[exciting_line + sensing_line * 26]);
			if(lntensity_data[exciting_line + sensing_line * 26] < 900 
			|| lntensity_data[exciting_line + sensing_line * 26] > 3000)
				ret = -1;
		}
		printk(" \n");
	}
	pr_info("check_debug_data: [TSP] Reading data end.\n");

	enable_irq(ts->client->irq);

	return ret;
}

static ssize_t set_all_refer_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int status;
	struct melfas_ts_data *ts = (struct melfas_ts_data *)(dev_get_drvdata(dev));

	status = check_debug_data(ts);

	return sprintf(buf, "%u\n", status);
}

static int idx =0;

static int atoi(char *str)
{
	int result = 0;
	int count = 0;
	if( str == NULL ) 
		return -1;
	while( str[count] != '\0' && str[count] >= '0' && str[count] <= '9' )
	{		
		result = result * 10 + str[count] - '0';
		++count;
	}
	return result;
}

ssize_t disp_all_refdata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);

//	printk(KERN_ERR "%s : value %d, IDX = %d\n", __func__, lntensity_data[idx], idx);

	if ( idx >= 350)
	{
	tsp_dbg_msg(KERN_ERR "%s : Resetting refdata IDX now. IDX = %d\n", __func__, idx);
	melfas_power_off();
	mdelay(100);
	melfas_init_panel(ts);
	}
    return sprintf(buf, "%u\n",  lntensity_data[idx]);
}

ssize_t disp_all_refdata_store(struct device *dev, struct device_attribute *attr,
								   const char *buf, size_t size)
{
	idx = atoi((char*) buf);

	tsp_dbg_msg(KERN_ERR "%s : value %d\n", __func__, idx);

  	return size;
}

static int check_delta_data(struct melfas_ts_data *ts)
{
	u8 write_buffer[6];
	u8 read_buffer[2];
	int sensing_line, exciting_line;
	int gpio = GPIO_TSP_INT;

	int ret = 0;
	disable_irq(ts->client->irq);
	/* enter the debug mode */
	write_buffer[0] = 0xA0;
	write_buffer[1] = 0x1A;
	write_buffer[2] = 0x0;
	write_buffer[3] = 0x0;
	write_buffer[4] = 0x0;
	write_buffer[5] = 0x01;
	melfas_i2c_write(ts->client, (char *)write_buffer, 6);

	/* wating for the interrupt*/
	while (gpio_get_value(gpio)) { 
		printk(".");
		udelay(100);
	}

	if (debug_print)
		pr_info("[TSP] read dummy\n");

	/* read the dummy data */
	melfas_i2c_read(ts->client, 0xA8, 2, read_buffer);

	if (debug_print)
		pr_info("[TSP] read inspenction data\n");
	write_buffer[5] = 0x02;		// CmDelta Test
	for (sensing_line = 0; sensing_line < 14; sensing_line++) {
		for (exciting_line =0; exciting_line < 26; exciting_line++) {
			write_buffer[2] = exciting_line;
			write_buffer[3] = sensing_line;
			melfas_i2c_write(ts->client, (char *)write_buffer, 6);
			melfas_i2c_read(ts->client, 0xA8, 2, read_buffer);
			inspection_data[exciting_line + sensing_line * 26] =
				(read_buffer[1] & 0xf) << 8 | read_buffer[0];
			tsp_dbg_msg("%d.", inspection_data[exciting_line + sensing_line * 26]);
			if(inspection_data[exciting_line + sensing_line * 26] < 100 
			|| inspection_data[exciting_line + sensing_line * 26] > 900)
				ret = -1;
		}
		printk(" \n");
	}
	pr_info("[TSP] Reading data end.\n");
		
	msleep(200);
//	release_all_fingers(ts);		// do i need it?
	touch_is_pressed = 0;

//	ts->gpio();					// do i need it?
	melfas_ts_suspend(ts->client, PMSG_SUSPEND);
	msleep(200);
	melfas_ts_resume(ts->client);
		
	

	enable_irq(ts->client->irq);

	return ret;
}

static ssize_t set_all_delta_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int status = 0;
	struct melfas_ts_data *ts = (struct melfas_ts_data *)(dev_get_drvdata(dev));

	status = check_delta_data(ts);

	set_tsp_module_off_show(dev, attr, buf);
	set_tsp_module_on_show(dev, attr, buf);
	printk(KERN_ERR "%s : value %d\n", __func__, status);

	return sprintf(buf, "%u\n", status);
}

ssize_t disp_all_deltadata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);

//	printk(KERN_ERR "%s : value %d, idx=%d\n", __func__, inspection_data[idx], idx);
  
	if ( idx >= 350)
	{
	tsp_dbg_msg(KERN_ERR "%s : Resetting now. IDX = %d\n", __func__, idx);
	melfas_power_off();
	mdelay(100);
	melfas_init_panel(ts);
	}

    return sprintf(buf, "%u\n",  inspection_data[idx]);
}


ssize_t disp_all_deltadata_store(struct device *dev, struct device_attribute *attr,
								   const char *buf, size_t size)
{
	idx = atoi((char*) buf);
//	printk(KERN_ERR "Delta data %d", idx);

  	return size;
}

static void check_intensity_data(struct melfas_ts_data *ts)
{

	u8 write_buffer[6];
	u8 read_buffer[2];
	int sensing_line, exciting_line;
	int gpio = GPIO_TSP_INT;

	disable_irq(ts->client->irq);
	if (0 == inspection_data[0]) {
		/* enter the debug mode */
		write_buffer[0] = 0xA0;
		write_buffer[1] = 0x1A;
		write_buffer[2] = 0x0;
		write_buffer[3] = 0x0;
		write_buffer[4] = 0x0;
		write_buffer[5] = 0x01;
		melfas_i2c_write(ts->client, (char *)write_buffer, 6);

		/* wating for the interrupt*/
		while (gpio_get_value(gpio)) {
			printk(".");
			udelay(100);
		}

		/* read the dummy data */
		melfas_i2c_read(ts->client, 0xA8, 2, read_buffer);

		write_buffer[5] = 0x02;
		for (sensing_line = 0; sensing_line < 14; sensing_line++) {
			for (exciting_line =0; exciting_line < 26; exciting_line++) {
				write_buffer[2] = exciting_line;
				write_buffer[3] = sensing_line;
				melfas_i2c_write(ts->client, (char *)write_buffer, 6);
				melfas_i2c_read(ts->client, 0xA8, 2, read_buffer);
				inspection_data[exciting_line + sensing_line * 26] =
					(read_buffer[1] & 0xf) << 8 | read_buffer[0];
			}
		}
		melfas_ts_suspend(ts->client, PMSG_SUSPEND);
		msleep(200);
		melfas_ts_resume(ts->client);
	}
 
	write_buffer[0] = 0xA0;
	write_buffer[1] = 0x1A;
	write_buffer[4] = 0x0;
	write_buffer[5] = 0x04;
	for (sensing_line = 0; sensing_line < 14; sensing_line++) {
		for (exciting_line =0; exciting_line < 26; exciting_line++) {
			write_buffer[2] = exciting_line;
			write_buffer[3] = sensing_line;
			melfas_i2c_write(ts->client, (char *)write_buffer, 6);
			melfas_i2c_read(ts->client, 0xA8, 2, read_buffer);
			lntensity_data[exciting_line + sensing_line * 26] =
				(read_buffer[1] & 0xf) << 8 | read_buffer[0];
		}
	}
	enable_irq(ts->client->irq);
/*
	pr_info("[TSP] lntensity data");
	int i;
	for (i = 0; i < 14*16; i++) {
		if (0 == i % 26)
			printk("\n");
		printk("%2u, ", lntensity_data[i]);
	}
*/
}

static ssize_t set_refer0_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 refrence = 0;
	struct melfas_ts_data *ts = dev_get_drvdata(dev);

	check_intensity_data(ts);

	refrence = inspection_data[28];
	return sprintf(buf, "%u\n", refrence);
}

static ssize_t set_refer1_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 refrence = 0;
	refrence = inspection_data[288];
	return sprintf(buf, "%u\n", refrence);
}

static ssize_t set_refer2_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 refrence = 0;
	refrence = inspection_data[194];
	return sprintf(buf, "%u\n", refrence);
}

static ssize_t set_refer3_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 refrence = 0;
	refrence = inspection_data[49];
	return sprintf(buf, "%u\n", refrence);
}

static ssize_t set_refer4_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 refrence = 0;
	refrence = inspection_data[309];
	return sprintf(buf, "%u\n", refrence);
}

static ssize_t set_intensity0_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 intensity = 0;
	intensity = lntensity_data[28];
	return sprintf(buf, "%u\n", intensity);
}

static ssize_t set_intensity1_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 intensity = 0;
	intensity = lntensity_data[288];
	return sprintf(buf, "%u\n", intensity);
}

static ssize_t set_intensity2_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 intensity = 0;
	intensity = lntensity_data[194];
	return sprintf(buf, "%u\n", intensity);
}

static ssize_t set_intensity3_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 intensity = 0;
	intensity = lntensity_data[49];
	return sprintf(buf, "%u\n", intensity);
}

static ssize_t set_intensity4_mode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u16 intensity = 0;
	intensity = lntensity_data[309];
	return sprintf(buf, "%u\n", intensity);
}


static DEVICE_ATTR(set_module_on, S_IRUGO | S_IWUSR | S_IWGRP, set_tsp_module_on_show, NULL);
static DEVICE_ATTR(set_module_off, S_IRUGO | S_IWUSR | S_IWGRP, set_tsp_module_off_show, NULL);
static DEVICE_ATTR(set_all_refer, S_IRUGO | S_IWUSR | S_IWGRP, set_all_refer_mode_show, NULL);
static DEVICE_ATTR(disp_all_refdata, S_IRUGO | S_IWUSR | S_IWGRP, disp_all_refdata_show, disp_all_refdata_store);
static DEVICE_ATTR(set_all_delta, S_IRUGO | S_IWUSR | S_IWGRP, set_all_delta_mode_show, NULL);
static DEVICE_ATTR(disp_all_deltadata, S_IRUGO | S_IWUSR | S_IWGRP, disp_all_deltadata_show, disp_all_deltadata_store);
static DEVICE_ATTR(set_refer0, S_IRUGO | S_IWUSR | S_IWGRP, set_refer0_mode_show, NULL);
static DEVICE_ATTR(set_delta0, S_IRUGO | S_IWUSR | S_IWGRP, set_intensity0_mode_show, NULL);
static DEVICE_ATTR(set_refer1, S_IRUGO | S_IWUSR | S_IWGRP, set_refer1_mode_show, NULL);
static DEVICE_ATTR(set_delta1, S_IRUGO | S_IWUSR | S_IWGRP, set_intensity1_mode_show, NULL);
static DEVICE_ATTR(set_refer2, S_IRUGO | S_IWUSR | S_IWGRP, set_refer2_mode_show, NULL);
static DEVICE_ATTR(set_delta2, S_IRUGO | S_IWUSR | S_IWGRP, set_intensity2_mode_show, NULL);
static DEVICE_ATTR(set_refer3, S_IRUGO | S_IWUSR | S_IWGRP, set_refer3_mode_show, NULL);
static DEVICE_ATTR(set_delta3, S_IRUGO | S_IWUSR | S_IWGRP, set_intensity3_mode_show, NULL);
static DEVICE_ATTR(set_refer4, S_IRUGO | S_IWUSR | S_IWGRP, set_refer4_mode_show, NULL);
static DEVICE_ATTR(set_delta4, S_IRUGO | S_IWUSR | S_IWGRP, set_intensity4_mode_show, NULL);

static struct attribute *sec_touch_facotry_attributes[] = {
	&dev_attr_set_module_on.attr,
	&dev_attr_set_module_off.attr,
	&dev_attr_set_all_refer.attr,
	&dev_attr_disp_all_refdata.attr,
	&dev_attr_set_all_delta.attr,
	&dev_attr_disp_all_deltadata.attr,
	&dev_attr_set_refer0.attr,
	&dev_attr_set_delta0.attr,
	&dev_attr_set_refer1.attr,
	&dev_attr_set_delta1.attr,
	&dev_attr_set_refer2.attr,
	&dev_attr_set_delta2.attr,
	&dev_attr_set_refer3.attr,
	&dev_attr_set_delta3.attr,
	&dev_attr_set_refer4.attr,
	&dev_attr_set_delta4.attr,
	NULL,
};

static struct attribute_group sec_touch_factory_attr_group = {
	.attrs = sec_touch_facotry_attributes,
};

static int melfas_i2c_read(struct i2c_client *client, u16 addr, u16 length, u8 *value)
{
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg[2];

	msg[0].addr  = client->addr;
	msg[0].flags = 0x00;
	msg[0].len   = 2;
	msg[0].buf   = (u8 *) &addr;

	msg[1].addr  = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = length;
	msg[1].buf   = (u8 *) value;

	if  (i2c_transfer(adapter, msg, 2) == 2)
		return 0;
	else
		return -EIO;

}


static int melfas_i2c_write(struct i2c_client *client, char *buf, int length)
{
	int i;
	char data[TS_SYSFS_WRITE_REGS_LEN];

	if (length > TS_SYSFS_WRITE_REGS_LEN) {
		printk(KERN_ERR "[TSP] size error - %s\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < length; i++)
		data[i] = *buf++;

	i = i2c_master_send(client, (char *)data, length);

	if (i == length)
		return length;
	else
		return -EIO;
}


static ssize_t set_tsp_firm_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);

	return sprintf(buf, "%#02x, %#02x, %#02x\n", ts->version->core, ts->version->private, ts->version->public);
}

static ssize_t set_tsp_firm_version_read_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	u8 fw_latest_version, privatecustom_version, publiccustom_version;
	int ret;
	uint8_t buff[7] = {0,};

	buff[0] = TS_READ_VERSION_ADDR;
	ret = i2c_master_send(ts->client, &buff, 1);
	if(ret < 0)
	{
		printk(KERN_ERR "%s : i2c_master_send [%d]\n", __func__, ret);
	}

	ret = i2c_master_recv(ts->client, &buff, 7);
	if(ret < 0)
	{
		printk(KERN_ERR "%s : i2c_master_recv [%d]\n", __func__, ret);
	}
	fw_latest_version 		= buff[3];
	privatecustom_version 	= buff[4];
	publiccustom_version 	= buff[5];

	return sprintf(buf, "%#02x, %#02x, %#02x\n", fw_latest_version, privatecustom_version, publiccustom_version);
}

static ssize_t set_tsp_threshold_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = (struct melfas_ts_data *)(dev_get_drvdata(dev));
	u8 threshold;

	melfas_i2c_read(ts->client, P5_THRESHOLD, 1, &threshold);

	return sprintf(buf, "%d\n", threshold);
}

static ssize_t tsp_touchtype_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	char temp[50];

	sprintf(temp, "TSP : MMS144\n");
	strcat(buf, temp);
	return strlen(buf);
}

static DEVICE_ATTR(tsp_firm_version_phone, S_IRUGO | S_IWUSR | S_IWGRP, set_tsp_firm_version_show, NULL);/* PHONE*/	/* firmware version resturn in phone driver version */
static DEVICE_ATTR(tsp_firm_version_panel, S_IRUGO | S_IWUSR | S_IWGRP, set_tsp_firm_version_read_show, NULL);/*PART*/	/* firmware version resturn in TSP panel version */
static DEVICE_ATTR(tsp_threshold, S_IRUGO | S_IWUSR | S_IWGRP, set_tsp_threshold_mode_show, NULL);
static DEVICE_ATTR(melfas_touchtype, S_IRUGO | S_IWUSR | S_IWGRP,	tsp_touchtype_show, NULL);

static struct attribute *sec_touch_attributes[] = {
	&dev_attr_tsp_firm_version_phone.attr,
	&dev_attr_tsp_firm_version_panel.attr,
	&dev_attr_tsp_threshold.attr,
	&dev_attr_melfas_touchtype.attr,
	NULL,
};

static struct attribute_group sec_touch_attr_group = {
	.attrs = sec_touch_attributes,
};
#endif //#ifdef SEC_TSP_SYS_FS_FD

static int melfas_init_panel(struct melfas_ts_data *ts)
{
	u8 buf = 0x00;
	int ret;

	tsp_dbg_msg("\n");

	melfas_power_on();

	ret = i2c_master_send(ts->client, &buf, 1);

	ret = i2c_master_send(ts->client, &buf, 1);

	if(ret <0)
	{
		printk(KERN_ERR "[melfasTSP] melfas_init_panel: i2c_master_send err=%d\n",ret);
		return 0;
	}

	return true;
}

static void melfas_ts_get_data(struct melfas_ts_data *ts)
{
	int ret = 0, i;
	uint8_t buf[TS_READ_REGS_LEN];
	int read_num, FingerID;

	int _touch_is_pressed = 0;

	tsp_dbg_msg("\n");

	if(ts ==NULL)
			printk(KERN_ERR "[melfasTSP]melfas_ts_get_data: ts NULL\n");

	buf[0] = TS_READ_START_ADDR;

	ret = i2c_master_send(ts->client, buf, 1);
	if(ret < 0)
	{
		printk(KERN_ERR "[melfasTSP] melfas_ts_get_data: i2c_master_send 1 err=%d\n",ret);
		return ;	
	}
	ret = i2c_master_recv(ts->client, buf, 1);
	if(ret < 0)
	{
		printk(KERN_ERR "[melfasTSP] melfas_ts_get_data: i2c_master_recv 1 err=%d\n",ret);
		return ;	
	}

	read_num = buf[0];
	
	if(read_num>0)
	{
		buf[0] = TS_READ_START_ADDR2;

		ret = i2c_master_send(ts->client, buf, 1);
		if(ret < 0)
		{
			printk(KERN_ERR "[melfasTSP] melfas_ts_get_data: i2c_master_send 2 err=%d\n",ret);
			return ;	
		}
		ret = i2c_master_recv(ts->client, buf, read_num);
		if(ret < 0)
		{
			printk(KERN_ERR "[melfasTSP] melfas_ts_get_data: i2c_master_recv 2 err=%d\n",ret);
			return ;	
		}
		
		if ((buf[0] & 0x80) && touch_cpu_lock_status == 0)
		{
			printk(KERN_ERR "[melfasTSP] melfas_ts_get_data: Touch Is Pressed\n");
			touch_cpu_lock_status = 1;
		/*	printk(KERN_DEBUG "[melfasTSP] melfas_ts_get_data: Touch ID: %d, State : %d, x: %d, y: %d, z: %d w: %d\n",
			i, (g_Mtouch_info[i].strength>0), g_Mtouch_info[i].posX, 
			g_Mtouch_info[i].posY, g_Mtouch_info[i].strength, 
			g_Mtouch_info[i].width);
		*/

			#ifdef CONFIG_S5PV310_HI_ARMCLK_THAN_1_2GHZ
				s5pv310_cpufreq_lock(DVFS_LOCK_ID_TSP, CPU_L3);
			#else
				s5pv310_cpufreq_lock(DVFS_LOCK_ID_TSP, CPU_L3);
			#endif
		}
		else if (!(buf[0] & 0x80) && touch_cpu_lock_status)
		{
			touch_cpu_lock_status = 0;
		/*	printk(KERN_DEBUG "[melfasTSP] melfas_ts_get_data: Touch ID: %d, State : %d, x: %d, y: %d, z: %d w: %d\n",
			i, (g_Mtouch_info[i].strength>0), g_Mtouch_info[i].posX, 
			g_Mtouch_info[i].posY, g_Mtouch_info[i].strength, 
			g_Mtouch_info[i].width);
		*/
			printk(KERN_ERR "[melfasTSP] melfas_ts_get_data: Touch Is Released\n");
			s5pv310_cpufreq_lock_free(DVFS_LOCK_ID_TSP);
		}

		// ESD protection code added: BEGIN
		if(buf[0] == 0x0F)
		{
			printk(KERN_ERR "[TSP] ESD protection. (%d)\n", __LINE__);
			//PLM 3102 Fix Begin.
			for (i = 0; i < MELFAS_MAX_TOUCH-1 ; i++)
			{
				g_Mtouch_info[i].strength = -1;
				g_Mtouch_info[i].posX = 0;
				g_Mtouch_info[i].posY = 0;
				g_Mtouch_info[i].width = 0;
			}
			// PLM 3102 Fix End
			disable_irq_nosync(ts->client->irq);
			touch_is_pressed = 0;

			s5pv310_cpufreq_lock_free(DVFS_LOCK_ID_TSP);
			touch_cpu_lock_status = 0;

			//ts->gpio();
			melfas_power_off();
			mdelay(200);
			melfas_init_panel(ts);
			enable_irq(ts->client->irq);

			return;
		}

		// ESD protection code: END

		for(i=0; i<read_num; i=i+6)
		{
			FingerID = (buf[i] & 0x0F) -1;

			g_Mtouch_info[FingerID].posX= (uint16_t)(buf[i+1] & 0x0F) << 8 | buf[i+2];
			g_Mtouch_info[FingerID].posY= (uint16_t)(buf[i+1] & 0xF0) << 4 | buf[i+3];	
			
			if((buf[i] & 0x80)==0)
				g_Mtouch_info[FingerID].strength = 0;
			else
				g_Mtouch_info[FingerID].strength = buf[i+5];
			
			g_Mtouch_info[FingerID].width= buf[i+4];		
			
		}
	
	}
	else
	{
		printk(KERN_ERR "[melfasTSP] melfas_ts_get_data: read_num = %d\n",read_num);
	}

	_touch_is_pressed=0;	//Kishore
	for(i=0; i<(MELFAS_MAX_TOUCH-1); i++)
	{
		if(g_Mtouch_info[i].strength== -1)
			continue;

		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[i].posX);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[i].posY);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, g_Mtouch_info[i].strength );
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, g_Mtouch_info[i].width);      				
		input_mt_sync(ts->input_dev);          

#if 1
		tsp_dbg_msg("Touch ID: %d, State : %d, x: %d, y: %d, z: %d w: %d\n",
			i, (g_Mtouch_info[i].strength>0), g_Mtouch_info[i].posX, 
			g_Mtouch_info[i].posY, g_Mtouch_info[i].strength, 
			g_Mtouch_info[i].width);
#else
		printk(KERN_DEBUG "[melfasTSP] melfas_ts_get_data: Touch ID: %d, State : %d, x: %d, y: %d, z: %d w: %d\n",
			i, (g_Mtouch_info[i].strength>0), g_Mtouch_info[i].posX, 
			g_Mtouch_info[i].posY, g_Mtouch_info[i].strength, 
			g_Mtouch_info[i].width);
#endif
		if(g_Mtouch_info[i].strength == 0)
			g_Mtouch_info[i].strength = -1;

		if(g_Mtouch_info[i].strength > 0)		//Kishore
			_touch_is_pressed = 1;
	}
			
	input_sync(ts->input_dev);
	touch_is_pressed = _touch_is_pressed;	//Kishore
}

static irqreturn_t melfas_ts_irq_handler(int irq, void *handle)
{
	struct melfas_ts_data *ts = (struct melfas_ts_data *)handle;
#if 1
	tsp_dbg_msg("\n");
#else
	printk(KERN_DEBUG "[melfasTSP] melfas_ts_irq_handler---------------------\n");
#endif
	if(ts->poweron == false) {
		printk(KERN_ERR " %s : ts->poweron [0x%x]\n",__func__, ts->poweron);
		return IRQ_HANDLED;
	}

	melfas_ts_get_data(ts);
	return IRQ_HANDLED;
}

#define ENXIO_TSP (-6)

static int melfas_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct melfas_ts_data *ts;
	struct melfas_tsi_platform_data *data;
	int ret = 0, i; 
	
	uint8_t buf[4] = {0,};

// Merging from Korean Dali - Sysfs related
#ifdef SEC_TSP_SYS_FS_FD
	struct device *sec_ts;
	struct device *melfas_noise_test;
#endif
// End: Merging from Korean Dali - Sysfs related
	
	tsp_dbg_msg("\n");

	ts = kmalloc(sizeof(struct melfas_ts_data), GFP_KERNEL);
    	if (ts == NULL)
    	{
		printk(KERN_ERR "[melfasTSP]melfas_ts_probe: failed to create a state of melfas-ts\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
    	}	
	melfas_power_on();
	ts->poweron = true;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        printk(KERN_ERR "[melfasTSP]melfas_ts_probe: need I2C_FUNC_I2C\n");
        ret = -ENODEV;
        goto err_check_functionality_failed;
    }


    data = client->dev.platform_data;
    ts->version = data->version;
    ts->client = client;
    i2c_set_clientdata(client, ts);
    ret = i2c_master_send(ts->client, &buf[0], 1);
    if(ret < 0)
    {
        printk(KERN_ERR "[melfasTSP] melfas_ts_probe: i2c_master_send err=%d\n",ret);
    }

	tsp_dbg_msg("i2c_master_send() [%d],Add[%d]\n", ret, ts->client->addr);

#if SET_DOWNLOAD_BY_GPIO
	buf[0] = TS_READ_VERSION_ADDR;
	ret = i2c_master_send(ts->client, &buf[0], 1);
	if(ret < 0)
	{
		printk(KERN_ERR "[melfasTSP] melfas_ts_probe: i2c_master_send err=%d\n",ret);
	}

	ret = i2c_master_recv(ts->client, &buf[0], 4);
	if(ret < 0)
	{
		printk(KERN_ERR "[melfasTSP] melfas_ts_probe: i2c_master_recv err=%d\n",ret);
	}

	tsp_dbg_msg(KERN_ERR "[melfasTSP]melfas tsp version: 0x%x 0x%x 0x%x 0x%x\n", buf[0],buf[1],buf[2],buf[3]);			

	if(buf[3] != FW_VERSION || buf[2] != COMPATIBILITY_VERSION)
	{
		buf[0] = TS_READ_VERSION_ADDR;
		ret = i2c_master_send(ts->client, &buf[0], 1);
		if(ret < 0)
		{
			printk(KERN_ERR "[melfasTSP] melfas_ts_probe: i2c_master_send err=%d\n",ret);
		}

		ret = i2c_master_recv(ts->client, &buf[0], 4);
		if(ret < 0)
		{
			printk(KERN_ERR "[melfasTSP] melfas_ts_probe: i2c_master_recv err=%d\n",ret);
		}

		tsp_dbg_msg(KERN_ERR "[melfasTSP] version check error! retry! melfas tsp version: 0x%x 0x%x 0x%x 0x%x\n", buf[0],buf[1],buf[2],buf[3]);			
	}

	if((buf[3] != FW_VERSION || buf[2] != COMPATIBILITY_VERSION) && ret != ENXIO_TSP)
	{
		ret = mcsdl_download_binary_data();
		if(ret == 0)
		{
			printk(KERN_ERR "[melfasTSP]SET Download Fail - error code [%d]\n", ret);			
		}
		else
		{
			printk(KERN_ERR "[melfasTSP]SET Download OK! GOOD!\n");
		}
	}	
#endif // SET_DOWNLOAD_BY_GPIO
	
	ts->input_dev = input_allocate_device();
    if (!ts->input_dev)
    {
		printk(KERN_ERR "[melfasTSP]melfas_ts_probe: Not enough memory\n");
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	} 

	ts->input_dev->name = "melfas-ts" ;

	ts->input_dev->evbit[0] = BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);
	

	ts->input_dev->keybit[BIT_WORD(KEY_MENU)] |= BIT_MASK(KEY_MENU);
	ts->input_dev->keybit[BIT_WORD(KEY_HOME)] |= BIT_MASK(KEY_HOME);
	ts->input_dev->keybit[BIT_WORD(KEY_BACK)] |= BIT_MASK(KEY_BACK);		
	ts->input_dev->keybit[BIT_WORD(KEY_SEARCH)] |= BIT_MASK(KEY_SEARCH);			


//	__set_bit(BTN_TOUCH, ts->input_dev->keybit);
//	__set_bit(EV_ABS,  ts->input_dev->evbit);
//	ts->input_dev->evbit[0] =  BIT_MASK(EV_SYN) | BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);	

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, TS_MAX_X_COORD, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, TS_MAX_Y_COORD, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, TS_MAX_Z_TOUCH, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, MELFAS_MAX_TOUCH-1, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, TS_MAX_W_TOUCH, 0, 0);
//	__set_bit(EV_SYN, ts->input_dev->evbit); 
//	__set_bit(EV_KEY, ts->input_dev->evbit);	


    ret = input_register_device(ts->input_dev);
    if (ret)
    {
        printk(KERN_ERR "[melfasTSP]melfas_ts_probe: Failed to register device\n");
        ret = -ENOMEM;
        goto err_input_register_device_failed;
    }

// Merging from Korean Dali - Sysfs related 
#ifdef SEC_TSP_SYS_FS_FD
	sec_ts = device_create(sec_class, NULL, 0, ts, "melfas_tsp");
	if (IS_ERR(sec_ts))
		 printk(KERN_ERR "[TSP] Failed to create device for the sysfs\n");

	if (device_create_file(sec_ts, &dev_attr_melfas_touchtype) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_melfas_touchtype.attr.name);
	}
	if (device_create_file(sec_ts, &dev_attr_tsp_firm_version_phone) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_tsp_firm_version_phone.attr.name);
	}
	if (device_create_file(sec_ts, &dev_attr_tsp_firm_version_panel) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_tsp_firm_version_panel.attr.name);
	}
	if (device_create_file(sec_ts, &dev_attr_tsp_threshold) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_tsp_threshold.attr.name);
	}

	/* ssong. test.
	ret = sysfs_create_group(&sec_ts->kobj, &sec_touch_attr_group);
	if (ret)
		printk(KERN_ERR "[TSP] Failed to create sysfs group\n");

	melfas_noise_test = device_create(sec_class, NULL, 0, ts, "melfas_noise_test");
	if (IS_ERR(melfas_noise_test))
	 	printk(KERN_ERR "[TSP] Failed to create melfas_noise_test device for the sysfs\n");
	*/

	if (device_create_file(sec_ts, &dev_attr_set_module_on) < 0) {
			printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_module_on.attr.name);
	}
	if (device_create_file(sec_ts, &dev_attr_set_module_off) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_module_off.attr.name);
	}
	if (device_create_file(sec_ts, &dev_attr_set_all_refer) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_all_refer.attr.name);
	}
	if (device_create_file(sec_ts, &dev_attr_disp_all_refdata) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_disp_all_refdata.attr.name);
	}
	if (device_create_file(sec_ts, &dev_attr_set_all_delta) < 0) {
			printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_all_delta.attr.name);
	}
	if (device_create_file(sec_ts, &dev_attr_disp_all_deltadata) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_tsp_firm_version_phone.attr.name);
	}
	if (device_create_file(sec_ts, &dev_attr_set_refer0) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_refer0.attr.name);
	}
	if (device_create_file(sec_ts, &dev_attr_set_delta0) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_delta0.attr.name);
	}
	if (device_create_file(sec_ts, &dev_attr_set_refer1) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_refer1.attr.name);
	}
	if (device_create_file(sec_ts, &dev_attr_set_delta1) < 0) {
			printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_delta1.attr.name);
	}
	if (device_create_file(sec_ts, &dev_attr_set_refer2) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_refer2.attr.name);
	}
	if (device_create_file(sec_ts, &dev_attr_set_delta2) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_delta2.attr.name);
	}
	if (device_create_file(sec_ts, &dev_attr_set_refer3) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_refer3.attr.name);
	}
	if (device_create_file(sec_ts, &dev_attr_set_delta3) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_delta3.attr.name);
	}
	if (device_create_file(sec_ts, &dev_attr_set_refer4) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_refer4.attr.name);
	}
	if (device_create_file(sec_ts, &dev_attr_set_delta4) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_set_delta4.attr.name);
	}

	/* ssong. test.
	ret = sysfs_create_group(&melfas_noise_test->kobj, &sec_touch_factory_attr_group);
	if (ret)
	 	printk(KERN_ERR "[TSP] Failed to create melfas_noise_test sysfs group\n");
	 */
#endif //#ifdef SEC_TSP_SYS_FS_FD
// End: Merging from Korean Dali - Sysfs related

    if (ts->client->irq)
    {

	    tsp_dbg_msg("trying to request irq: %s-%d\n", 
						ts->client->name, ts->client->irq);

	ret = request_threaded_irq(client->irq, NULL, melfas_ts_irq_handler,IRQF_TRIGGER_FALLING, ts->client->name, ts);

        if (ret > 0)
        {
            printk(KERN_ERR "[melfasTSP]melfas_ts_probe: Can't allocate irq %d, ret %d\n", ts->client->irq, ret);
            ret = -EBUSY;
            goto err_request_irq;
        }
    }

	disable_irq(ts->client->irq);   // 20111128 - To remove the Warning stack - Unbalance enable of IRQ 328.
	for (i = 0; i < (MELFAS_MAX_TOUCH-1) ; i++)  /* _SUPPORT_MULTITOUCH_ */
		g_Mtouch_info[i].strength = -1;	

	tsp_dbg_msg("succeed to register input device\n");

#if CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = melfas_ts_early_suspend;
	ts->early_suspend.resume = melfas_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	enable_irq(client->irq);


	tsp_dbg_msg("Start touchscreen. name: %s, irq: %d\n", 
					ts->client->name, ts->client->irq);
	
	return 0;

err_request_irq:
	printk(KERN_ERR "[melfasTSP]melfas-ts: err_request_irq failed\n");
	free_irq(client->irq, ts);
err_input_register_device_failed:
	printk(KERN_ERR "[melfasTSP]melfas-ts: err_input_register_device failed\n");
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
	printk(KERN_ERR "[melfasTSP]melfas-ts: err_input_dev_alloc failed\n");
err_alloc_data_failed:
	printk(KERN_ERR "[melfasTSP]melfas-ts: err_alloc_data failed_\n");	
	kfree(ts);
err_check_functionality_failed:
	printk(KERN_ERR "[melfasTSP]melfas-ts: err_check_functionality failed_\n");

	return ret;
}

static int melfas_ts_remove(struct i2c_client *client)
{
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

	unregister_early_suspend(&ts->early_suspend);
	free_irq(client->irq, ts);
        ts->power(false);  // modified
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int melfas_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	int i;    
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

	tsp_dbg_msg("\n");

	for (i = 0; i < MELFAS_MAX_TOUCH-1 ; i++)
	{
		g_Mtouch_info[i].strength = -1;
		g_Mtouch_info[i].posX = 0;
		g_Mtouch_info[i].posY = 0;
		g_Mtouch_info[i].width = 0;
	}
    
	disable_irq(client->irq);
	ts->poweron = false;

	tsp_dbg_msg("TSP disable_irq(%d) poweron %x\n",client->irq,ts->poweron);

	ret = i2c_smbus_write_byte_data(client, 0x01, 0x00); /* deep sleep */
	
	if (ret < 0)
		printk(KERN_ERR "[melfasTSP]melfas_ts_suspend: i2c_smbus_write_byte_data failed\n");

	melfas_power_off();

	touch_is_pressed = 0;


//fix for PLM-2056
if(touch_cpu_lock_status == 1)
{
	tsp_dbg_msg(" Sending release event to upper layer\n");
	
	touch_cpu_lock_status = 0;
	s5pv310_cpufreq_lock_free(DVFS_LOCK_ID_TSP);

	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, 0);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, 0);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0 );
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);      				
	input_mt_sync(ts->input_dev);

	input_sync(ts->input_dev);
}

	return 0;
}

static int melfas_ts_resume(struct i2c_client *client)
{
	struct melfas_ts_data *ts = i2c_get_clientdata(client);

	tsp_dbg_msg("client->irq=%d\n",client->irq);

	melfas_init_panel(ts);

	if(ts->poweron == false){
		ts->poweron = true;
		enable_irq(client->irq); // scl wave
		
	}

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h)
{
	struct melfas_ts_data *ts;

	tsp_dbg_msg("\n");

	ts = container_of(h, struct melfas_ts_data, early_suspend);
	melfas_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void melfas_ts_late_resume(struct early_suspend *h)
{
	struct melfas_ts_data *ts;

	tsp_dbg_msg("\n");

	ts = container_of(h, struct melfas_ts_data, early_suspend);
	melfas_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id melfas_ts_id[] =
{
    { MELFAS_TS_NAME, 0 },
    { }
};

static int melfas_pm_suspend(struct device *dev)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct mxt224_data *data = i2c_get_clientdata(client);

	//mxt224_enabled = 0;
	//touch_is_pressed = 0;
	/* Doing_calibration_falg = 0; */
	//return mxt224_internal_suspend(data);

	tsp_dbg_msg("\n");

#ifndef CONFIG_HAS_EARLYSUSPEND
	melfas_power_off();
#endif
	return 0;
}

static int melfas_pm_resume(struct device *dev)
{
	int ret = 0;
#ifndef CONFIG_HAS_EARLYSUSPEND
	struct i2c_client *client = to_i2c_client(dev);
#endif

	tsp_dbg_msg("\n");

#ifndef CONFIG_HAS_EARLYSUSPEND
	ret = melfas_ts_resume(client);
#endif

	return ret;
}

static const struct dev_pm_ops melfas_pm_ops = {
	.suspend = melfas_pm_suspend,
	.resume = melfas_pm_resume,
};

static struct i2c_driver melfas_ts_driver =
{
    .driver = {
    .name = MELFAS_TS_NAME,
	.pm = &melfas_pm_ops,
    },
    .id_table = melfas_ts_id,
    .probe = melfas_ts_probe,
    .remove = __devexit_p(melfas_ts_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend		= melfas_ts_suspend,
	.resume		= melfas_ts_resume,
#endif
};

static int __devinit melfas_ts_init(void)
{
	#if defined(CONFIG_MACH_C1_KDDI_REV00)
	extern int sec_isLpmMode(void);

	if (sec_isLpmMode())
	{
		printk(KERN_ERR "LPM MODE (melfas_ts_init)!\n");
		return 0;
	}
	#endif

	return i2c_add_driver(&melfas_ts_driver);
}

static void __exit melfas_ts_exit(void)
{
	i2c_del_driver(&melfas_ts_driver);
}

MODULE_DESCRIPTION("Driver for Melfas MTSI Touchscreen Controller");
MODULE_AUTHOR("MinSang, Kim <kimms@melfas.com>");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");

module_init(melfas_ts_init);
module_exit(melfas_ts_exit);
