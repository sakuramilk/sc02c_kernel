/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2012 sakuramilk <c.sakuramilk@gmail.com>
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <asm/gpio.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/earlysuspend.h>
#include <asm/io.h>
#ifdef CONFIG_CPU_FREQ
/* #include <mach/cpu-freq-v210.h> */
#endif

#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>

#include "c1-cypress-gpio.h"

#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <plat/gpio-cfg.h>
#include <mach/gpio.h>

#ifdef CONFIG_GENERIC_BLN
#include <linux/bln.h>
#include <linux/spinlock.h>
#endif
#if defined(CONFIG_GENERIC_BLN) || defined(CONFIG_CM_BLN)
#include <linux/wakelock.h>
#endif

/*
 * Melfas touchkey register
 */
#define KEYCODE_REG 0x00
#define FIRMWARE_VERSION 0x01
#define TOUCHKEY_MODULE_VERSION 0x02
#define TOUCHKEY_ADDRESS	0x20

#define UPDOWN_EVENT_BIT 0x08
#define KEYCODE_BIT 0x07
#define ESD_STATE_BIT 0x10

#define I2C_M_WR 0		/* for i2c */

#define DEVICE_NAME "sec_touchkey"
#define TOUCH_FIRMWARE_V04  0x04
#define TOUCH_FIRMWARE_V07  0x07
#define DOOSUNGTECH_TOUCH_V1_2  0x0C

#ifdef CONFIG_MACH_C1_NA_SPR_EPIC2_REV00
#define EPIC2_TK_FIRMWARE_VER  0x03
#define EPIC2_TK_MODULE_VER    0x02
#endif
#ifdef CONFIG_MACH_Q1_REV02
#define Q1_TK_FIRMWARE_VER  0x12
#define Q1_TK_MODULE_VER    0x11
#endif


#if defined(CONFIG_MACH_C1_KDDI_REV00) 
#define ESD_SW_SIMUL_TEST 0
#define RECOVERY_PROTECTION_CODE 0

//ssong110618. reallocate touch key mapping.
static int touchkey_keycode[5] = {NULL, KEY_MENU, KEY_BACK, KEY_HOME, NULL};
#else
#if defined(CONFIG_TARGET_LOCALE_NAATT)
static int touchkey_keycode[5] = { 0, KEY_MENU, KEY_ENTER, KEY_BACK, KEY_END };
#elif defined(CONFIG_TARGET_LOCALE_NA)
static int touchkey_keycode[5] = { NULL, KEY_SEARCH, KEY_BACK, KEY_HOME, KEY_MENU};
#else
static int touchkey_keycode[3] = { 0, KEY_MENU, KEY_BACK };
#endif
#endif

#if defined(CONFIG_TARGET_LOCALE_NAATT) || defined(CONFIG_TARGET_LOCALE_NA) || defined(CONFIG_MACH_Q1_REV02)
static u8 home_sensitivity;
static u8 search_sensitivity;
static u16 raw_data0;
static u16 raw_data1;
static u16 raw_data2;
static u16 raw_data3;
static u8 idac0;
static u8 idac1;
static u8 idac2;
static u8 idac3;
static u8 touchkey_threshold;

int touchkey_autocalibration(void);
int get_touchkey_module_version(void);
#endif

static u8 menu_sensitivity = 0;
static u8 back_sensitivity = 0;

static int touchkey_enable = 0;
static bool touchkey_probe = true;
/*sec_class sysfs*/
extern struct class *sec_class;
struct device *sec_touchkey;


struct i2c_touchkey_driver {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct early_suspend early_suspend;
};
struct i2c_touchkey_driver *touchkey_driver = NULL;
struct work_struct touchkey_work;
struct workqueue_struct *touchkey_wq;

struct work_struct touch_update_work;
struct delayed_work touch_resume_work;


static const struct i2c_device_id melfas_touchkey_id[] = {
	{"melfas_touchkey", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, melfas_touchkey_id);

static void init_hw(void);
static int i2c_touchkey_probe(struct i2c_client *client, const struct i2c_device_id *id);
extern int get_touchkey_firmware(char *version);
static int touchkey_led_status = 0;
static int touchled_cmd_reversed=0;

struct i2c_driver touchkey_i2c_driver = {
	.driver = {
		.name = "melfas_touchkey_driver",
	},
	.id_table = melfas_touchkey_id,
	.probe = i2c_touchkey_probe,
};

static int touchkey_debug_count = 0;
static char touchkey_debug[104];
//static int touch_version = 0;
//static int module_version = 0;
//static int store_module_version = 0;
extern int touch_is_pressed;

extern int ISSP_main(void);
static int touchkey_update_status;

//ssong111207. SMD function test reset issue.
static bool IsTouchKeyExisted = true;

#ifdef CONFIG_GENERIC_BLN
static struct wake_lock bln_wake_lock;
static bool touchkey_suspend = false;
static DECLARE_MUTEX(bln_sem);
#endif

#ifdef CONFIG_CM_BLN
/*
 * Standard CyanogenMod LED Notification functionality.
 */
#define CM_ENABLE_BL		( 1)
#define CM_DISABLE_BL		( 2)
#define CM_BL_ALWAYS_ON		(-1)
#define CM_BL_ALWAYS_OFF	(-2)

static int cm_led_on = 0;
static int cm_screen_on = 1;
static int cm_led_timeout = CM_BL_ALWAYS_OFF; /* never time out */
static int cm_notification_timeout = -1; /* never time out */
static int cm_notification_enabled = -1; /* Disabled by default */
static struct wake_lock cm_led_wake_lock;
static DECLARE_MUTEX(cm_enable_sem);

/* timer related declares */
static struct timer_list cm_led_timer;
static void bl_off(struct work_struct *bl_off_work);
static DECLARE_WORK(bl_off_work, bl_off);
static struct timer_list cm_notification_timer;
static void notification_off(struct work_struct *notification_off_work);
static DECLARE_WORK(notification_off_work, notification_off);
#endif

/* led i2c write value convert helper */
static int inline touchkey_convert_led_value(int data) {
#ifdef CONFIG_MACH_C1_NA_SPR_EPIC2_REV00
	#error
#else
#ifdef CONFIG_TARGET_LOCALE_NA  /* Touchkey LED command changed from F/W Rev 0xC */
  #if defined(CONFIG_MACH_C1_KDDI_REV00) 
	if (data == 1)
		return 0x10;
	else
		return 0x20;
  #else //#if defined(CONFIG_MACH_C1_KDDI_REV00) 
	if(store_module_version >= 8){
		if (data == 1)
			return 0x10;
		else
			return 0x20;
	} else {
		return (data == 1) ? 1 : 2;
	}
  #endif //#if defined(CONFIG_MACH_C1_KDDI_REV00) 
#elif defined(CONFIG_MACH_Q1_REV02)
	if(data == 1)
		return 0x10;
	else
		return 0x20;
#else
	return (data == 1) ? 1 : 2;
#endif
#endif
}

static void touch_forced_release(void)
{
}

int touchkey_led_ldo_on(bool on)
{
	struct regulator *regulator;

#if defined(CONFIG_MACH_C1_KDDI_REV00) 
	printk(KERN_ERR "[TouchKey] touchkey_led_ldo_on() on=%d \n", on);
#endif

	if (on) {
		regulator = regulator_get(NULL, "touch_led");
		if (IS_ERR(regulator)){
			printk(KERN_ERR "[TouchKey] touchkey_led_ldo_on(1): regulator error \n");
			return 0;
		}
		regulator_enable(regulator);
		regulator_put(regulator);
	} else {
		regulator = regulator_get(NULL, "touch_led");
		if (IS_ERR(regulator)) {
			printk(KERN_ERR "[TouchKey] touchkey_led_ldo_on(0): regulator error \n");
			return 0;
		}
		if (regulator_is_enabled(regulator))
			regulator_force_disable(regulator);
		regulator_put(regulator);
	}

	return 0;
}


int touchkey_ldo_on(bool on)
{
	struct regulator *regulator;

#if defined(CONFIG_MACH_C1_KDDI_REV00) 
//	printk(KERN_ERR "[TouchKey] touchkey_ldo_on() on=%d \n", on);
#endif

	if (on) {
		regulator = regulator_get(NULL, "touch");
		if (IS_ERR(regulator)){
			printk(KERN_ERR "[TouchKey] touchkey_ldo_on(1): regulator error \n");
			return 0;
		}
		regulator_enable(regulator);
		regulator_put(regulator);
	} else {
		regulator = regulator_get(NULL, "touch");
		if (IS_ERR(regulator)) {
			printk(KERN_ERR "[TouchKey] touchkey_ldo_on(0): regulator error \n");
			return 0;
		}
		if (regulator_is_enabled(regulator))
			regulator_force_disable(regulator);
		regulator_put(regulator);
	}

	return 1;
}

static void c1_change_touch_key_led_voltage(int vol_mv)
{
	struct regulator *tled_regulator;

	tled_regulator = regulator_get(NULL, "touch_led");
	if (IS_ERR(tled_regulator)) {
		pr_err("%s: failed to get resource %s\n", __func__, "touch_led");
		return;
	}
	regulator_set_voltage(tled_regulator, vol_mv * 1000, vol_mv * 1000);
	regulator_put(tled_regulator);
}

static ssize_t brightness_control(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int data;

	if (sscanf(buf, "%d\n", &data) == 1) {
		printk(KERN_ERR "[TouchKey] touch_led_brightness: %d \n", data);
		c1_change_touch_key_led_voltage(data);
	} else {
		printk(KERN_ERR "[TouchKey] touch_led_brightness Error\n");
	}

	return size;
}

static void set_touchkey_debug(char value)
{
	if (touchkey_debug_count == 100)
		touchkey_debug_count = 0;

	touchkey_debug[touchkey_debug_count] = value;
	touchkey_debug_count++;
}

static int i2c_touchkey_read(u8 reg, u8 *val, unsigned int len)
{
	int err = 0;
	int retry = 2;
	struct i2c_msg msg[1];

	if ((touchkey_driver == NULL)|| !(touchkey_enable == 1) || !touchkey_probe || (IsTouchKeyExisted == false)) { //Kishore
		printk(KERN_ERR "[TouchKey] i2c read err!. touchkey_enable=%d,touchkey_probe=%d,IsTouchKeyExisted=%d\n", touchkey_enable,touchkey_probe,IsTouchKeyExisted);
		return -ENODEV;
	}

	while (retry--) {
		msg->addr = touchkey_driver->client->addr;
		msg->flags = I2C_M_RD;
		msg->len = len;
		msg->buf = val;
		err = i2c_transfer(touchkey_driver->client->adapter, msg, 1);

		if (err >= 0) {
			return 0;
		}
		printk(KERN_ERR "[TouchKey] %s %d i2c transfer error = %d\n", __func__, __LINE__, err);	/* add by inter.park */
		mdelay(10);
	}
	return err;

}

#ifdef CONFIG_MACH_C1_KDDI_REV00
static int i2c_touchkey_read_check_version(u8 reg, u8 *val, unsigned int len)
{
	int err = 0;
	//int retry = 2;
	struct i2c_msg msg[1];

	//if ((touchkey_driver == NULL)|| !(touchkey_enable == 1) || !touchkey_probe || (IsTouchKeyExisted == false)) {
	//	printk(KERN_ERR "[TouchKey] touchkey is not enabled. %d\n", __LINE__);
	//	return -ENODEV;
	//}

	//while (retry--) {
		msg->addr = touchkey_driver->client->addr;
		msg->flags = I2C_M_RD;
		msg->len = len;
		msg->buf = val;
		err = i2c_transfer(touchkey_driver->client->adapter, msg, 1);

		if (err >= 0) {
			return 0;
		}
		printk(KERN_ERR "[TouchKey] %s %d i2c transfer error\n", __func__, __LINE__);	/* add by inter.park */
	//	mdelay(10);
	//}
	return err;

}
#endif

#if defined(CONFIG_TARGET_LOCALE_NAATT)  || defined(CONFIG_TARGET_LOCALE_NA) || defined(CONFIG_MACH_Q1_REV02)
static int i2c_touchkey_write(u8 *val, unsigned int len)
{
	int err = 0;
	struct i2c_msg msg[1];
	int retry = 2;

	if ((touchkey_driver == NULL) || !(touchkey_enable == 1) || !touchkey_probe  || (IsTouchKeyExisted == false)	) {		//Kishore
		printk(KERN_ERR "[TouchKey] touchkey is not enabled. %d\n", __LINE__);
		return -ENODEV;
	}

	while (retry--) {
		msg->addr = touchkey_driver->client->addr;
		msg->flags = I2C_M_WR;
		msg->len = len;
		msg->buf = val;
		err = i2c_transfer(touchkey_driver->client->adapter, msg, 1);
		/*printk("write value %d to address %d\n",*val, msg->addr);*/
		if (err >= 0) {

			return 0;

		}
		printk(KERN_DEBUG "[TouchKey] %s %d i2c transfer error = %d\n",
		       __func__, __LINE__, err);
		mdelay(10);
	}
	return err;
}

#if defined(CONFIG_MACH_C1_KDDI_REV00) 
static void led_i2c_command(int OnOff)
{
	u8 data[2]={0,};
	int count = 0;

	/* Send led on/off Command */
	data[0] = OnOff? 0x10: 0x20;
	
	count = i2c_touchkey_write(data, 1);

//	printk(KERN_ERR"[TouchKey]led_i2c_command(%d) count=%d, data[0]=0x%x\n",OnOff,count,data[0]);

	return;
}
#endif

#else
static int i2c_touchkey_write(u8 *val, unsigned int len)
{
	int err = 0;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retry = 2;

	if ((touchkey_driver == NULL) || !(touchkey_enable == 1)) {
		printk(KERN_ERR "[TouchKey] i2c_touchkey_write: touchkey is not enabled.\n");
		return -ENODEV;
	}

	while (retry--) {
		data[0] = *val;
		msg->addr = touchkey_driver->client->addr;
		msg->flags = I2C_M_WR;
		msg->len = len;
		msg->buf = data;
		err = i2c_transfer(touchkey_driver->client->adapter, msg, 1);
		if (err >= 0) {
			return 0;
		}
		printk(KERN_DEBUG "[TouchKey] %s %d i2c transfer error\n", __func__, __LINE__);
		mdelay(10);
	}
	return err;
}
#endif

#if defined(CONFIG_TARGET_LOCALE_NAATT) || defined(CONFIG_TARGET_LOCALE_NA)
#if defined(CONFIG_MACH_C1_KDDI_REV00) 
int touchkey_autocalibration(void)
{
	u8 data[6]={0,};
	int count = 0;
	int ret = 0;
	unsigned short retry = 0;

	while( retry < 3 )
	{
		ret = i2c_touchkey_read(KEYCODE_REG, data, 4);
		if (ret < 0) {
			printk(KERN_ERR"[TouchKey]i2c read fail.\n");
			return ret;
		}
		printk(KERN_DEBUG "[TouchKey] touchkey_autocalibration :data[0]=%x data[1]=%x data[2]=%x data[3]=%x\n",data[0],data[1],data[2],data[3]);

		/* Send autocal Command */
		data[0] = 0x50;
		data[1] = 0x00;
		data[2] = 0x00;
		data[3] = 0x01;

		count = i2c_touchkey_write(data, 4);

		msleep(100);

		/* Check autocal status*/
		ret = i2c_touchkey_read(KEYCODE_REG, data, 6);

		if((data[5] & 0x80)) {
			printk(KERN_DEBUG "[Touchkey] autocal Enabled\n");
			break;
		}
		else
			printk(KERN_DEBUG "[Touchkey] autocal disabled, retry %d\n", retry);

		retry = retry + 1;
	}

	if( retry == 3 )
		printk(KERN_DEBUG "[Touchkey] autocal failed\n");

	return count;
}
#else //#if defined(CONFIG_MACH_C1_KDDI_REV00) 
int touchkey_autocalibration(void)
{
	u8 data[4]={0,};
	int count = 0;
	int ret = 0;

	ret = i2c_touchkey_read(KEYCODE_REG, data, 4);
	if (ret < 0) {
		printk(KERN_ERR"[TouchKey]i2c read fail.\n");
		return ret;
	}
        printk(KERN_DEBUG "[TouchKey] touchkey_autocalibration :data[0]=%x data[1]=%x data[2]=%x data[3]=%x",data[0],data[1],data[2],data[3]);
	data[0] = 0x50;
	data[3] = 0x01;

	count = i2c_touchkey_write(data, 4);

	return count;
}
#endif //#if defined(CONFIG_MACH_C1_KDDI_REV00) 

#elif defined (CONFIG_MACH_Q1_REV02)

int touchkey_autocalibration(void)
{
	u8 data[6]={0,};
	int count = 0;
	int ret = 0;
	unsigned short retry = 0;

	while( retry < 3 )
	{
		ret = i2c_touchkey_read(KEYCODE_REG, data, 4);
		if (ret < 0) {
			printk(KERN_ERR"[TouchKey]i2c read fail.\n");
			return ret;
		}
		printk(KERN_DEBUG "[TouchKey] touchkey_autocalibration :data[0]=%x data[1]=%x data[2]=%x data[3]=%x\n",data[0],data[1],data[2],data[3]);

		/* Send autocal Command */
		data[0] = 0x50;
		data[3] = 0x01;

		count = i2c_touchkey_write(data, 4);

		msleep(100);

		/* Check autocal status*/
		ret = i2c_touchkey_read(KEYCODE_REG, data, 6);

		if((data[5] & 0x80)) {
			printk(KERN_DEBUG "[Touchkey] autocal Enabled\n");
			break;
		}
		else
			printk(KERN_DEBUG "[Touchkey] autocal disabled, retry %d\n", retry);

		retry = retry + 1;
	}

	if( retry == 3 )
		printk(KERN_DEBUG "[Touchkey] autocal failed\n");

	return count;
}
#endif
#if defined(CONFIG_TARGET_LOCALE_NAATT) || defined(CONFIG_TARGET_LOCALE_NA)
#ifndef CONFIG_TARGET_LOCALE_NA
static ssize_t set_touchkey_autocal_testmode(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int count = 0;
	u8 set_data;
	int on_off;

	if (sscanf(buf, "%d\n", &on_off) == 1) {
		printk(KERN_ERR "[TouchKey] Test Mode : %d \n", on_off);

		if (on_off == 1) {
			set_data = 0x40;
			count = i2c_touchkey_write(&set_data, 1);
		} else {
			touchkey_ldo_on(0);
			msleep(50);
			touchkey_ldo_on(1);
			msleep(50);
			init_hw();
			msleep(50);
			touchkey_autocalibration();
		}
	} else {
		printk(KERN_ERR "[TouchKey] touch_led_brightness Error\n");
	}

	return count;
}
#endif
#if defined(CONFIG_MACH_C1_KDDI_REV00) 

static ssize_t touchkey_raw_data0_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[26] = {0,};
	int ret;

	printk(KERN_DEBUG "called %s \n", __func__);
	ret = i2c_touchkey_read(KEYCODE_REG, data, 26);
	printk(KERN_DEBUG "called %s data[16] =%d,data[17] = %d\n", __func__, data[16], data[17]);
	raw_data0 = ((0x00FF&data[14])<<8)|data[15];
	return sprintf(buf, "%d\n", raw_data0);
}

static ssize_t touchkey_raw_data1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[26] = {0,};
	int ret;

	printk(KERN_DEBUG "called %s \n", __func__);
	ret = i2c_touchkey_read(KEYCODE_REG, data, 26);
	printk(KERN_DEBUG "called %s data[14] =%d,data[15] = %d\n", __func__, data[14], data[15]);
	raw_data1 = ((0x00FF&data[16])<<8)|data[17];
	return sprintf(buf, "%d\n", raw_data1);
}

static ssize_t touchkey_raw_data2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[26] = {0,};
	int ret;

	printk(KERN_DEBUG "called %s \n", __func__);
	ret = i2c_touchkey_read(KEYCODE_REG, data, 26);
	printk(KERN_DEBUG "called %s data[22] =%d,data[23] = %d\n", __func__, data[14], data[15]);
	raw_data2 = ((0x00FF&data[14])<<8)|data[15];
	return sprintf(buf, "%d\n", raw_data2);
}

static ssize_t touchkey_raw_data3_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[26] = {0,};
	int ret;

	printk(KERN_DEBUG "called %s \n", __func__);
	ret = i2c_touchkey_read(KEYCODE_REG, data, 26);
	printk(KERN_DEBUG "called %s data[24] =%d,data[25] = %d\n", __func__, data[16], data[17]);
	raw_data3 = ((0x00FF&data[16])<<8)|data[17];
	return sprintf(buf, "%d\n", raw_data3);
}

static ssize_t touchkey_idac0_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[10];
	int ret;
	printk(KERN_DEBUG "called %s \n", __func__);
	ret = i2c_touchkey_read(KEYCODE_REG, data, 10);
	printk(KERN_DEBUG "called %s data[6] =%d\n", __func__, data[6]);
	idac0 = data[6];
	return sprintf(buf, "%d\n", idac0);
}

static ssize_t touchkey_idac1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[10];
	int ret;
	printk(KERN_DEBUG "called %s \n", __func__);
	ret = i2c_touchkey_read(KEYCODE_REG, data, 10);
	printk(KERN_DEBUG "called %s data[7] = %d\n", __func__, data[7]);
	idac1 = data[7];
	return sprintf(buf, "%d\n", idac1);
}

static ssize_t touchkey_idac2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[10];
	int ret;
	printk(KERN_DEBUG "called %s \n", __func__);
	ret = i2c_touchkey_read(KEYCODE_REG, data, 10);
	printk(KERN_DEBUG "called %s data[8] =%d\n", __func__, data[8]);
	idac2 = data[8];
	return sprintf(buf, "%d\n", idac2);
}

static ssize_t touchkey_idac3_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[10];
	int ret;
	printk(KERN_DEBUG "called %s \n", __func__);
	ret = i2c_touchkey_read(KEYCODE_REG, data, 10);
	printk(KERN_DEBUG "called %s data[9] = %d\n", __func__, data[9]);
	idac3 = data[9];
	return sprintf(buf, "%d\n", idac3);
}
#else //#if defined(CONFIG_MACH_C1_KDDI_REV00) 

static ssize_t touchkey_raw_data0_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[26] = {0,};
	int ret;

	printk(KERN_DEBUG "called %s \n", __func__);
	ret = i2c_touchkey_read(KEYCODE_REG, data, 26);
#if defined(CONFIG_TARGET_LOCALE_NA)
	printk(KERN_DEBUG "called %s data[18] =%d,data[19] = %d\n", __func__, data[18], data[19]);
	raw_data0 = ((0x00FF&data[18])<<8)|data[19];
#elif defined(CONFIG_MACH_Q1_REV02)
	printk(KERN_DEBUG "called %s data[16] =%d,data[17] = %d\n", __func__, data[16], data[17]);
	raw_data0 = ((0x00FF&data[14])<<8)|data[15];
#else
	printk(KERN_DEBUG "called %s data[18] =%d,data[19] = %d\n", __func__, data[10], data[11]);
	raw_data0 = ((0x00FF&data[10])<<8)|data[11];
#endif /* CONFIG_TARGET_LOCALE_NA */
	return sprintf(buf, "%d\n", raw_data0);
}

static ssize_t touchkey_raw_data1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[26] = {0,};
	int ret;

	printk(KERN_DEBUG "called %s \n", __func__);
	ret = i2c_touchkey_read(KEYCODE_REG, data, 26);
#if defined(CONFIG_TARGET_LOCALE_NA)
	printk(KERN_DEBUG "called %s data[20] =%d,data[21] = %d\n", __func__, data[20], data[21]);
	raw_data1 = ((0x00FF&data[20])<<8)|data[21];
#elif defined(CONFIG_MACH_Q1_REV02)
	printk(KERN_DEBUG "called %s data[14] =%d,data[15] = %d\n", __func__, data[14], data[15]);
	raw_data1 = ((0x00FF&data[16])<<8)|data[17];
#else
	printk(KERN_DEBUG "called %s data[20] =%d,data[21] = %d\n", __func__, data[12], data[13]);
	raw_data1 = ((0x00FF&data[12])<<8)|data[13];
#endif /* CONFIG_TARGET_LOCALE_NA */
	return sprintf(buf, "%d\n", raw_data1);
}

static ssize_t touchkey_raw_data2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[26] = {0,};
	int ret;

	printk(KERN_DEBUG "called %s \n", __func__);
	ret = i2c_touchkey_read(KEYCODE_REG, data, 26);
#if defined(CONFIG_TARGET_LOCALE_NA)
	printk(KERN_DEBUG "called %s data[22] =%d,data[23] = %d\n", __func__, data[22], data[23]);
	raw_data2 = ((0x00FF&data[22])<<8)|data[23];
#else
	printk(KERN_DEBUG "called %s data[22] =%d,data[23] = %d\n", __func__, data[14], data[15]);
	raw_data2 = ((0x00FF&data[14])<<8)|data[15];
#endif /* CONFIG_TARGET_LOCALE_NA */
	return sprintf(buf, "%d\n", raw_data2);
}

static ssize_t touchkey_raw_data3_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[26] = {0,};
	int ret;

	printk(KERN_DEBUG "called %s \n", __func__);
	ret = i2c_touchkey_read(KEYCODE_REG, data, 26);
#if defined(CONFIG_TARGET_LOCALE_NA)
	printk(KERN_DEBUG "called %s data[24] =%d,data[25] = %d\n", __func__, data[24], data[25]);
	raw_data3 = ((0x00FF&data[24])<<8)|data[25];
#else
	printk(KERN_DEBUG "called %s data[24] =%d,data[25] = %d\n", __func__, data[16], data[17]);
	raw_data3 = ((0x00FF&data[16])<<8)|data[17];
#endif /* CONFIG_TARGET_LOCALE_NA */
	return sprintf(buf, "%d\n", raw_data3);
}

static ssize_t touchkey_idac0_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[10];
	int ret;
#ifndef CONFIG_MACH_C1_NA_SPR_EPIC2_REV00
#ifdef CONFIG_TARGET_LOCALE_NA
	if (store_module_version < 8) {
	return 0;
	}
#endif
#endif
	printk(KERN_DEBUG "called %s \n", __func__);
	ret = i2c_touchkey_read(KEYCODE_REG, data, 10);
	printk(KERN_DEBUG "called %s data[6] =%d\n", __func__, data[6]);
	idac0 = data[6];
	return sprintf(buf, "%d\n", idac0);
}

static ssize_t touchkey_idac1_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[10];
	int ret;
#ifndef CONFIG_MACH_C1_NA_SPR_EPIC2_REV00
#ifdef CONFIG_TARGET_LOCALE_NA
        if (store_module_version < 8)
        return 0;
#endif
#endif
	printk(KERN_DEBUG "called %s \n", __func__);
	ret = i2c_touchkey_read(KEYCODE_REG, data, 10);
	printk(KERN_DEBUG "called %s data[7] = %d\n", __func__, data[7]);
	idac1 = data[7];
	return sprintf(buf, "%d\n", idac1);
}

static ssize_t touchkey_idac2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[10];
	int ret;
#ifndef CONFIG_MACH_C1_NA_SPR_EPIC2_REV00
#ifdef CONFIG_TARGET_LOCALE_NA
	if (store_module_version < 8)
		return 0;
#endif
#endif
	printk(KERN_DEBUG "called %s \n", __func__);
	ret = i2c_touchkey_read(KEYCODE_REG, data, 10);
	printk(KERN_DEBUG "called %s data[8] =%d\n", __func__, data[8]);
	idac2 = data[8];
	return sprintf(buf, "%d\n", idac2);
}

static ssize_t touchkey_idac3_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[10];
	int ret;
#ifndef CONFIG_MACH_C1_NA_SPR_EPIC2_REV00
#ifdef CONFIG_TARGET_LOCALE_NA
        if(store_module_version < 8)
        return 0;
#endif
#endif
	printk(KERN_DEBUG "called %s \n", __func__);
	ret = i2c_touchkey_read(KEYCODE_REG, data, 10);
	printk(KERN_DEBUG "called %s data[9] = %d\n", __func__, data[9]);
	idac3 = data[9];
	return sprintf(buf, "%d\n", idac3);
}
#endif //#if defined(CONFIG_MACH_C1_KDDI_REV00) 

static ssize_t touchkey_threshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[10];
	int ret;

	printk(KERN_DEBUG "called %s \n", __func__);
	ret = i2c_touchkey_read(KEYCODE_REG, data, 10);
	printk(KERN_DEBUG "called %s data[4] = %d\n", __func__, data[4]);
	touchkey_threshold = data[4];
	return sprintf(buf, "%d\n", touchkey_threshold);
}

#ifndef CONFIG_TARGET_LOCALE_NA
void touchkey_firmware_update(void)
{
	char data[3];
	int retry;
	int ret = 0;

	ret = i2c_touchkey_read(KEYCODE_REG, data, 3);
	if (ret < 0) {
		printk("[TouchKey] i2c read fail. do not excute firm update.\n");
		return;
	}

	printk(KERN_ERR"%s F/W version: 0x%x, Module version:0x%x\n", __FUNCTION__, data[1], data[2]);
	retry = 3;

	touch_version = data[1];
	module_version = data[2];

	if (touch_version < 0x0A) {
		touchkey_update_status = 1;
		while (retry--) {
			if (ISSP_main() == 0) {
				printk(KERN_ERR"[TOUCHKEY]Touchkey_update succeeded\n");
				touchkey_update_status = 0;
				break;
			}
			printk(KERN_ERR"touchkey_update failed... retry...\n");
	   }
		if (retry <= 0) {
			touchkey_ldo_on(0);
			touchkey_update_status = -1;
			msleep(300);
		}

		init_hw();
	} else {
		 if (touch_version >= 0x0A) {
			printk(KERN_ERR "[TouchKey] Not F/W update. Cypess touch-key F/W version is latest. \n");
		} else {
			printk(KERN_ERR "[TouchKey] Not F/W update. Cypess touch-key version(module or F/W) is not valid. \n");
		}
	}
}
#endif /* CONFIG_TARGET_LOCALE_NA */

#ifdef CONFIG_MACH_C1_NA_SPR_EPIC2_REV00
void touchkey_firmware_update_epic2(void)
{
	char data[3];
	int retry = 3;
	int ret = 0;

	ret = i2c_touchkey_read(KEYCODE_REG, data, 3);
	if (ret < 0) {
		printk("[TouchKey] i2c read fail. do not excute firm update.\n");
		return;
	}

	touch_version = data[1];
	module_version = data[2];

	if (system_rev > 6) {
		printk("[TouchKey] not firmup hw(system_rev=%d)\n", system_rev);
		return;
	}

	if ((touch_version < EPIC2_TK_FIRMWARE_VER) &&
			(module_version == EPIC2_TK_MODULE_VER)) {
		/* latest epic2 board firm ver = 0x03, module ver = 0x02 */
		printk(KERN_DEBUG"[TouchKey] firmware auto update excute\n");
		disable_irq(IRQ_TOUCH_INT);
		touchkey_update_status = 1;

		while (retry--) {
			if (ISSP_main() == 0) {
				printk(KERN_DEBUG"[TouchKey]firmware update succeeded\n");
				touchkey_update_status = 0;
				break;
			}
			msleep(100);
			printk(KERN_DEBUG"[TouchKey] firmware update failed. retry\n");
		}
		if (retry <= 0) {
			touchkey_ldo_on(0);
			touchkey_update_status = -1;
			printk(KERN_DEBUG"[TouchKey] firmware update failed.\n");
			msleep(300);
		}
		enable_irq(IRQ_TOUCH_INT);
		init_hw();
	} else {
		printk(KERN_DEBUG"[TouchKey] firmware auto update do not excute\n");
		printk(KERN_DEBUG"[TouchKey] firmware_ver(banary=%d, current=%d)\n",
			EPIC2_TK_FIRMWARE_VER, touch_version);
		printk(KERN_DEBUG"[TouchKey] module_ver(banary=%d, current=%d)\n",
			EPIC2_TK_MODULE_VER, module_version);
		return;
	}
	msleep(100);
	i2c_touchkey_read(KEYCODE_REG, data, 3);
	touch_version = data[1];
	module_version = data[2];
	printk(KERN_DEBUG"[TouchKey] firm ver = %d, module ver = %d\n",
		touch_version, module_version);
}
#elif defined(CONFIG_MACH_Q1_REV02)
extern unsigned char firmware_data_F12M11[];
extern unsigned char firmware_data_F10M09[];
extern unsigned char *firmware_data;
void touchkey_firmware_update_q1(void)
{
	char data[3];
	int retry = 3;
	int ret = 0;
	bool update_firmware = false;

	ret = i2c_touchkey_read(KEYCODE_REG, data, 3);
	if (ret < 0) {
		printk("[TouchKey] i2c read fail. do not excute firm update.\n");
		return;
	}

	touch_version = data[1];
	module_version = data[2];

	if( module_version == 0x09 || module_version == 0x10)	{
		firmware_data = firmware_data_F10M09;
		if (touch_version < 0x10) {
			printk(KERN_DEBUG "[Touchkey] Update for F10M09\n");
			update_firmware = true;
		}
	}
	else if( module_version == Q1_TK_MODULE_VER) {
		firmware_data = firmware_data_F12M11;
		if (touch_version < Q1_TK_FIRMWARE_VER) {
			printk(KERN_DEBUG "[Touchkey] Update for F11M11\n");
			update_firmware = true;
		}
	}

	if( update_firmware ) {
		printk(KERN_DEBUG"[TouchKey] firmware auto update excute\n");
		disable_irq(IRQ_TOUCH_INT);
		touchkey_update_status = 1;

		while (retry--) {
			if (ISSP_main() == 0) {
				printk(KERN_DEBUG"[TouchKey]firmware update succeeded\n");
				touchkey_update_status = 0;
				break;
			}
			msleep(100);
			printk(KERN_DEBUG"[TouchKey] firmware update failed. retry\n");
		}
		if (retry <= 0) {
			touchkey_ldo_on(0);
			touchkey_update_status = -1;
			printk(KERN_DEBUG"[TouchKey] firmware update failed.\n");
			msleep(300);
		}
		enable_irq(IRQ_TOUCH_INT);
		init_hw();
		msleep(100);
		i2c_touchkey_read(KEYCODE_REG, data, 3);
		touch_version = data[1];
		module_version = data[2];
		printk(KERN_DEBUG"[TouchKey] firm ver = %d, module ver = %d\n",
			touch_version, module_version);
	} else {
		printk(KERN_DEBUG"[TouchKey] firmware auto update do not excute\n");
		printk(KERN_DEBUG"[TouchKey] firmware_ver(binary=%d, current=%d)\n",
			Q1_TK_FIRMWARE_VER, touch_version);
		printk(KERN_DEBUG"[TouchKey] module_ver(binary=%d, current=%d)\n",
			Q1_TK_MODULE_VER, module_version);
	}
}
#elif defined(CONFIG_MACH_C1_KDDI_REV00)

extern unsigned char Dali_JPN_UNIT_F04M01_20111026[];
#define K1_FIRMWARE_VERSION 0x04
#define K1_MODULE_VERSION 0x01

/*
extern unsigned char FW_Dali_JPN_UNIT_F03M05_20110906[];
#define K1_FIRMWARE_VERSION 0x03
#define K1_MODULE_VERSION 0x05
*/

extern unsigned char *firmware_data;

int touchkey_firmware_update_k1(int fw_update_condition)
{
	char data[3];
	int retry = 3;
	int ret = 0;
	bool update_firmware = false;

	ret = i2c_touchkey_read_check_version(KEYCODE_REG, data, 3);
	if (ret < 0) 
	{
		printk("[TouchKey FW Download] *** i2c read fail. read again!\n");
		mdelay(1);
		ret = i2c_touchkey_read_check_version(KEYCODE_REG, data, 3);
		if (ret < 0) 
		{
			printk("[TouchKey FW Download] *** i2c read fail. read again!\n");
			mdelay(1);			
			ret = i2c_touchkey_read_check_version(KEYCODE_REG, data, 3);
			if (ret < 0) 
			{
				printk("[TouchKey FW Download] *** i2c read fail. do not excute firm update.\n");
				fw_update_condition = 0x18;
				IsTouchKeyExisted = false;
			}
		}
	}
	else
	{
		printk("[TouchKey FW Download] *** touchkey_firmware_update_k1:Read OK!KEYCODE_REG[0x%x][0x%x][0x%x]\n",data[0],data[1],data[2]);
	}

	touch_version = data[1];
	module_version = data[2];

	printk("[TouchKey FW Download] *** touch_version=0x%x,module_version=0x%x\n",touch_version, module_version);

	if(touch_version < K1_FIRMWARE_VERSION || fw_update_condition == 0x18)
	{
		//firmware_data = FW_Dali_JPN_UNIT_F03M05_20110906;
		//printk(KERN_DEBUG "[Touchkey] *** Update for FW_Dali_JPN_UNIT_F03M05_20110906\n");

			firmware_data = Dali_JPN_UNIT_F04M01_20111026;
			printk(KERN_DEBUG "[TouchKey FW Download] *** Update for version  %d\n", K1_FIRMWARE_VERSION);
		update_firmware = true;
	}

	if(IsTouchKeyExisted == false)
	{
		update_firmware = false;
	}

	if(update_firmware) 
	{
		touchkey_led_ldo_on(0);
		touchkey_ldo_on(0);

		printk(KERN_DEBUG"[TouchKey FW Download] *** firmware auto update excute\n");

		disable_irq(IRQ_TOUCH_INT);
		touchkey_update_status = 1;

		while (retry--) {
			if (ISSP_main() == 0) {
				printk(KERN_DEBUG"[TouchKey FW Download] *** firmware update succeeded\n");
				touchkey_update_status = 0;
				ret = 0;
				break;
			}
			msleep(100);
			printk(KERN_DEBUG"[TouchKey FW Download] *** firmware update failed. retry\n");
			ret = 18;
		}
		if (retry <= 0) {
			touchkey_ldo_on(0);
			touchkey_update_status = -1;
			printk(KERN_DEBUG"[TouchKey FW Download] *** firmware update failed.\n");
			ret = 18;
			msleep(300);
		}
		enable_irq(IRQ_TOUCH_INT);
		init_hw();
		touchkey_led_ldo_on(1);		
		msleep(100);

		i2c_touchkey_read(KEYCODE_REG, data, 3);
		touch_version = data[1];
		module_version = data[2];
		printk(KERN_DEBUG"[TouchKey FW Download] *** firmware version(touch_version)=0x%x, module version=0x%x\n", touch_version, module_version);
	}
	else
	{
		printk(KERN_DEBUG"[TouchKey FW Download] *** firmware auto update do not excute (no need to update!)\n");
		printk(KERN_DEBUG"[TouchKey FW Download] *** touch_version=0x%x,module_version=0x%x\n",touch_version, module_version);
		ret = 0;
	}
	return ret;
}
#endif
#endif

extern void TSP_forced_release(void);
#if defined(CONFIG_MACH_C1_KDDI_REV00) 
void touchkey_work_func(struct work_struct *p)
{
	u8 data[18] = {0, };	
	int ret;
	int retry = 10;

	set_touchkey_debug('a');

	//printk(KERN_DEBUG"[TouchKey] *** touchkey_work_func()!\n");

	ret = i2c_touchkey_read(KEYCODE_REG, data, 3);

	set_touchkey_debug(data[0]);

	if ((data[0] & ESD_STATE_BIT) || (ret != 0)) 
	{
		printk(KERN_DEBUG"[TouchKey] ESD_STATE_BIT set or I2C fail: data: %d, retry: %d, i2c ret=%d\n", data[0], retry, ret);

		/* releae key */
		input_report_key(touchkey_driver->input_dev, touchkey_keycode[1], 0);
		input_report_key(touchkey_driver->input_dev, touchkey_keycode[2], 0);

		retry = 10;
		while (retry--) {
			gpio_direction_output(_3_GPIO_TOUCH_EN, 0);
			touchkey_ldo_on(0);	// touch ldo off.
			mdelay(300);
			init_hw();
			if (i2c_touchkey_read(KEYCODE_REG, data, 3) >= 0) 
			{
				printk(KERN_DEBUG"[TouchKey] %s touchkey init success data[0] = %d\n", __func__, data[0]);
				set_touchkey_debug('O');
				enable_irq(IRQ_TOUCH_INT);
				return;
			}
			printk(KERN_ERR"[TouchKey] %s %d i2c transfer error retry = %d\n", __func__, __LINE__, retry);
		}

		/* touchkey die , do not enable touchkey
		   enable_irq(IRQ_TOUCH_INT); */
		touchkey_enable = -1;
		gpio_direction_output(_3_GPIO_TOUCH_EN, 0);
		touchkey_ldo_on(0);	// touch ldo off.
		gpio_direction_output(_3_TOUCH_SDA_28V, 0);
		gpio_direction_output(_3_TOUCH_SCL_28V, 0);
		printk(KERN_DEBUG "[TouchKey] %s touchkey died\n", __func__);
		set_touchkey_debug('D');
		return;
	}

	if (touchkey_keycode[data[0] & KEYCODE_BIT] != KEY_MENU && touchkey_keycode[data[0] & KEYCODE_BIT] != KEY_BACK) 
	{
		enable_irq(IRQ_TOUCH_INT);
		return ;
	}

	if (data[0] & UPDOWN_EVENT_BIT) 
	{

		input_report_key(touchkey_driver->input_dev, touchkey_keycode[data[0] & KEYCODE_BIT], 0);
		input_sync(touchkey_driver->input_dev);

		//printk(KERN_DEBUG "[TouchKey] release keycode:%d \n", touchkey_keycode[data[0] & KEYCODE_BIT]);
	} 
	else 
	{
		if (touch_is_pressed) 
		{
			printk(KERN_DEBUG "[TouchKey] touchkey pressed but don't send event because touch is pressed. \n");
			set_touchkey_debug('P');
		} 
		else 
		{
			if ((data[0] & KEYCODE_BIT) == 2) 
			{
				/* if back key is pressed, release multitouch */
				/*printk(KERN_DEBUG "[TouchKey] touchkey release tsp input. \n");*/
				touch_forced_release();
			}

			input_report_key(touchkey_driver->input_dev,  touchkey_keycode[data[0] & KEYCODE_BIT], 1);
			input_sync(touchkey_driver->input_dev);

			printk(KERN_DEBUG "[TouchKey] Touchkey pressed \n");
		}
	}

#ifdef CONFIG_CM_BLN
	/* we have timed out or the lights should be on */
	if (cm_led_timer.expires > jiffies || cm_led_timeout != CM_BL_ALWAYS_OFF) {
		int status = 1;
           printk(KERN_ERR "[TouchKey] %d : %s(%d)\n", __LINE__, __func__, status);
		i2c_touchkey_write((u8 *)&status, 1); /* turn on */
	}

	/* restart the timer */
	if (cm_led_timeout > 0) {
		mod_timer(&led_timer, jiffies + msecs_to_jiffies(led_timeout));
	}
#endif

	set_touchkey_debug('A');
	enable_irq(IRQ_TOUCH_INT);
}
#else //#if defined(CONFIG_MACH_C1_KDDI_REV00) 
void touchkey_work_func(struct work_struct *p)
{
#if defined(CONFIG_TARGET_LOCALE_NA) || defined(CONFIG_MACH_Q1_REV02)
	u8 data[18];
#else
	u8 data[10];
#endif
	int ret;
	int retry = 10;

#if 0
	if (gpio_get_value(_3_GPIO_TOUCH_INT)) {
		printk(KERN_DEBUG "[TouchKey] Unknown state.\n", __func__);
		enable_irq(IRQ_TOUCH_INT);
		return;
	}
#endif

	set_touchkey_debug('a');

	ret = i2c_touchkey_read(KEYCODE_REG, data, 3);

	/******************************************************************
	typedef struct I2CReg
	{
		unsigned char	 BtnStatus;							 // 0 :
		unsigned char	 Version;								  // 1 :FW Version
		unsigned char	 PcbStatus;							 // 2 :Module Version
		unsigned char	 Cmd;									  // 3 :
		unsigned char	 Chip_id;								  // 4 :0x55(DEFAULT_CHIP_ID) 0
		unsigned char	 Sens;									   // 5 :sensitivity grade(0x00(slow),0x01(mid),0x02(fast))
		WORD			 DiffData[CSD_TotalSensorCount];   //  6, 7 - 8, 9
		WORD			 RawData[CSD_TotalSensorCount];  // 10,11 - 12,13
		WORD			 Baseline[CSD_TotalSensorCount];   // 14,15 - 16,17
	}I2CReg;
	******************************************************************/
        /******************************************************************
        For Gaudi after adding Auto Calibration feature
        *******************************************************************
        typedef struct I2CReg
        {
                unsigned char   BtnStatus;                      // 00 :button status
                unsigned char   Version;                        // 01 :FW Version
                unsigned char   PcbStatus;                      // 02 :Module Version
                unsigned char   Cmd;                            // 03 :host command
                unsigned char   Threshold;                      // 04 :finger threshold set
                unsigned char   Sens;                           // 05 :sensitivity grade(0x00(slow))
                unsigned char   SetIdac[4];                     // 06,07,08,09 : autocal idac data
                WORD            DiffData[CSD_TotalSensorCount]; // 10,11, 12,13, 14,15, 16,17,
                WORD            RawData[CSD_TotalSensorCount];  // 18,19, 20,21, 22,23, 24,25
                WORD            Baseline[CSD_TotalSensorCount]; // 26,27, 28,29, 30,31, 32,33
        }I2CReg;
        ******************************************************************/
	set_touchkey_debug(data[0]);
	if ((data[0] & ESD_STATE_BIT) || (ret != 0)) {
		printk(KERN_DEBUG "[TouchKey] ESD_STATE_BIT set or I2C fail: data: %d, retry: %d\n", data[0], retry);

		/* release key */
		input_report_key(touchkey_driver->input_dev, touchkey_keycode[1], 0);
		input_report_key(touchkey_driver->input_dev, touchkey_keycode[2], 0);
#ifdef CONFIG_TARGET_LOCALE_NA
		input_report_key(touchkey_driver->input_dev, touchkey_keycode[3], 0);
                input_report_key(touchkey_driver->input_dev, touchkey_keycode[4], 0);
#endif

#if defined(CONFIG_TARGET_LOCALE_NAATT)
		input_report_key(touchkey_driver->input_dev, touchkey_keycode[3], 0);
		input_report_key(touchkey_driver->input_dev, touchkey_keycode[4], 0);
#endif

		retry = 10;
		while (retry--) {
			gpio_direction_output(_3_GPIO_TOUCH_EN, 0);
			mdelay(300);
			init_hw();

			if (i2c_touchkey_read(KEYCODE_REG, data, 3) >= 0) {
				printk(KERN_DEBUG "[TouchKey] %s touchkey init success\n", __func__);
				set_touchkey_debug('O');
				enable_irq(IRQ_TOUCH_INT);
				return;
			}
			printk(KERN_ERR "[TouchKey] %s %d i2c transfer error retry = %d\n", __func__, __LINE__, retry);
		}

		/* touchkey die , do not enable touchkey
		enable_irq(IRQ_TOUCH_INT); */
		touchkey_enable = -1;
		gpio_direction_output(_3_GPIO_TOUCH_EN, 0);
		gpio_direction_output(_3_TOUCH_SDA_28V, 0);
		gpio_direction_output(_3_TOUCH_SCL_28V, 0);
		printk(KERN_DEBUG "[TouchKey] %s touchkey died\n", __func__);
		set_touchkey_debug('D');
		return;
	}

#if defined(CONFIG_TARGET_LOCALE_NAATT)
	if (touchkey_keycode[data[0] & KEYCODE_BIT] != KEY_MENU && touchkey_keycode[data[0] & KEYCODE_BIT] != KEY_BACK
		&&  touchkey_keycode[data[0] & KEYCODE_BIT] != KEY_ENTER && touchkey_keycode[data[0] & KEYCODE_BIT] != KEY_END) {
#elif defined(CONFIG_TARGET_LOCALE_NA)
	if (touchkey_keycode[data[0] & KEYCODE_BIT] != KEY_MENU && touchkey_keycode[data[0] & KEYCODE_BIT] != KEY_BACK &&
		touchkey_keycode[data[0] & KEYCODE_BIT] != KEY_HOME && touchkey_keycode[data[0] & KEYCODE_BIT] != KEY_SEARCH) {
#else
	if (touchkey_keycode[data[0] & KEYCODE_BIT] != KEY_MENU && touchkey_keycode[data[0] & KEYCODE_BIT] != KEY_BACK) {
#endif
		enable_irq(IRQ_TOUCH_INT);
		return ;
	}

	if (data[0] & UPDOWN_EVENT_BIT) {

		input_report_key(touchkey_driver->input_dev, touchkey_keycode[data[0] & KEYCODE_BIT], 0);
		input_sync(touchkey_driver->input_dev);

		//printk(KERN_DEBUG "[TouchKey] release keycode:%d \n", touchkey_keycode[data[0] & KEYCODE_BIT]);
	} else {
		if (touch_is_pressed) {
			printk(KERN_DEBUG
			       "[TouchKey] touchkey pressed but don't send event because touch is pressed. \n");
			set_touchkey_debug('P');
		} else {
			if ((data[0] & KEYCODE_BIT) == 2) {
				/* if back key is pressed, release multitouch */
				/*printk(KERN_DEBUG "[TouchKey] touchkey release tsp input. \n");*/
				touch_forced_release();
			}

			input_report_key(touchkey_driver->input_dev,  touchkey_keycode[data[0] & KEYCODE_BIT], 1);
			input_sync(touchkey_driver->input_dev);

			//printk(KERN_DEBUG "[TouchKey] press keycode:%d \n", touchkey_keycode[data[0] & KEYCODE_BIT]);
		}
	}

#ifdef CONFIG_CM_BLN
	/* we have timed out or the lights should be on */
	if (cm_led_timer.expires > jiffies || cm_led_timeout != CM_BL_ALWAYS_OFF) {
		int status = 1;
		printk(KERN_ERR "[TouchKey] %d : %s(%d)\n", __LINE__, __func__, status);
		i2c_touchkey_write((u8 *)&status, 1); /* turn on */
	}

	/* restart the timer */
	if (cm_led_timeout > 0) {
		mod_timer(&cm_led_timer, jiffies + msecs_to_jiffies(cm_led_timeout));
	}
#endif

	set_touchkey_debug('A');
	enable_irq(IRQ_TOUCH_INT);
}
#endif //#if defined(CONFIG_MACH_C1_KDDI_REV00) 

static irqreturn_t touchkey_interrupt(int irq, void *dummy)
{
	set_touchkey_debug('I');
	disable_irq_nosync(IRQ_TOUCH_INT);
	queue_work(touchkey_wq, &touchkey_work);

	return IRQ_HANDLED;
}

#ifdef CONFIG_GENERIC_BLN
static void touchkey_bln_wakeup(void)
{
	printk(KERN_DEBUG "[TouchKey] touchkey wakeup wake_lock!!!");
	if (wake_lock_active(&bln_wake_lock)) {
		wake_lock(&bln_wake_lock);
	}
	touchkey_ldo_on(1);
	msleep(50);
	touchkey_led_ldo_on(1);
	touchkey_enable = 1;
}

static void touchkey_bln_sleep(void)
{
	printk(KERN_DEBUG "[TouchKey] touchkey sleep wake_unlock!!!");
	touchkey_led_ldo_on(0);
	touchkey_ldo_on(0);
	touchkey_enable = 0;
	wake_unlock(&bln_wake_lock);
}

static void melfas_enable_touchkey_backlights(void) {
	int value;

	printk(KERN_DEBUG "[TouchKey] %s\n", __func__);
	down(&bln_sem);
	if (touchkey_suspend)
	{
		if (touchkey_enable == 0) {
			touchkey_bln_wakeup();
		}
		value = touchkey_convert_led_value(1);
        printk(KERN_ERR "[TouchKey] %d : %s(%d)\n", __LINE__, __func__, value);
		i2c_touchkey_write((u8 *)&value, 1);
		touchkey_led_status = 2;
		touchled_cmd_reversed = 1;
	}
	up(&bln_sem);
}

static void melfas_disable_touchkey_backlights(void) {
	int value;

	printk(KERN_DEBUG "[TouchKey] %s\n", __func__);
	down(&bln_sem);
	if (touchkey_suspend)
	{
		value = touchkey_convert_led_value(2);
        printk(KERN_ERR "[TouchKey] %d : %s(%d)\n", __LINE__, __func__, value);
		i2c_touchkey_write((u8 *)&value, 1);
		if (touchkey_enable == 1) {
			touchkey_bln_sleep();
		}
		touchkey_led_status = 1;
		touchled_cmd_reversed = 0;
	}
	up(&bln_sem);
}

static struct bln_implementation cypress_touchkey_bln = {
	.enable = melfas_enable_touchkey_backlights,
	.disable = melfas_disable_touchkey_backlights,
};
#endif // CONFIG_GENERIC_BLN

#ifdef CONFIG_CM_BLN
/*
 * Start of the main LED Notify code block
 */
static void bl_off(struct work_struct *bl_off_work)
{
	int status;

	/* do nothing if there is an active notification */
	if (cm_led_on == 1 || touchkey_enable != 1)
		return;

	/* we have timed out, turn the lights off */
	status = 2;
    printk(KERN_ERR "[TouchKey] %d : %s(%d)\n", __LINE__, __func__, status);
	i2c_touchkey_write((u8 *)&status, 1);

	return;
}

static void handle_led_timeout(unsigned long data)
{
	/* we cannot call the timeout directly as it causes a kernel spinlock BUG, schedule it instead */
	schedule_work(&bl_off_work);
}

static void notification_off(struct work_struct *notification_off_work)
{
	int status;

	/* do nothing if there is no active notification */
	if (cm_led_on != 1 || touchkey_enable != 1)
		return;

	/* we have timed out, turn the lights off */
	/* disable the regulators */
	touchkey_led_ldo_on(0);	/* "touch_led" regulator */
	touchkey_ldo_on(0);	/* "touch" regulator */

	/* turn off the backlight */
	status = 2; /* light off */
    printk(KERN_ERR "[TouchKey] %d : %s(%d)\n", __LINE__, __func__, status);
	i2c_touchkey_write((u8 *)&status, 1);
	touchkey_enable = 0;
	cm_led_on = 0;

	/* we were using a wakelock, unlock it */
	if (wake_lock_active(&cm_led_wake_lock)) {
		wake_unlock(&cm_led_wake_lock);
	}

	return;
}

static void handle_notification_timeout(unsigned long data)
{
	/* we cannot call the timeout directly as it causes a kernel spinlock BUG, schedule it instead */
	schedule_work(&notification_off_work);
}


static ssize_t led_status_read( struct device *dev, struct device_attribute *attr, char *buf )
{
	return sprintf(buf,"%u\n", cm_led_on);
}

static ssize_t led_status_write( struct device *dev, struct device_attribute *attr, const char *buf, size_t size )
{
	unsigned int data;
	int status;

	if(sscanf(buf,"%u\n", &data ) == 1) {

		switch (data) {
		case CM_ENABLE_BL:
			printk(KERN_DEBUG "[LED] ENABLE_BL\n");
			if (cm_notification_enabled > 0) {
				/* we are using a wakelock, activate it */
				if (!wake_lock_active(&cm_led_wake_lock)) {
					wake_lock(&cm_led_wake_lock);
				}

				if (!cm_screen_on) {
					/* enable regulators */
					touchkey_ldo_on(1);         /* "touch" regulator */
					touchkey_led_ldo_on(1);		/* "touch_led" regulator */
					touchkey_enable = 1;
				}

				/* enable the backlight */
				status = touchkey_convert_led_value(1);
                printk(KERN_ERR "[TouchKey] %d : %s(%d)\n", __LINE__, __func__, status);
				i2c_touchkey_write((u8 *)&status, 1);
				cm_led_on = 1;

				/* See if a timeout value has been set for the notification */
				if (cm_notification_timeout > 0) {
					mod_timer(&cm_notification_timer, jiffies + msecs_to_jiffies(cm_notification_timeout));	/* restart the timer */
				}
			}
			break;

		case CM_DISABLE_BL:
			printk(KERN_DEBUG "[LED] DISABLE_BL\n");

			/* prevent race with late resume*/
			down(&cm_enable_sem);

			/* only do this if a notification is on already, do nothing if not */
			if (cm_led_on == 1) {

				/* turn off the backlight */
				status = touchkey_convert_led_value(2); /* light off */
				printk(KERN_ERR "[TouchKey] %d : %s(%d)\n", __LINE__, __func__, status);
				i2c_touchkey_write((u8 *)&status, 1);
				cm_led_on = 0;

				if (!cm_screen_on) {
					/* disable the regulators */
					touchkey_led_ldo_on(0);	/* "touch_led" regulator */
					touchkey_ldo_on(0);	/* "touch" regulator */
					touchkey_enable = 0;
				}

				/* a notification timeout was set, disable the timer */
				if (cm_notification_timeout > 0) {
					del_timer(&cm_notification_timer);
				}

				/* we were using a wakelock, unlock it */
				if (wake_lock_active(&cm_led_wake_lock)) {
					wake_unlock(&cm_led_wake_lock);
				}
			}

			/* prevent race */
			up(&cm_enable_sem);

			break;
		}
	}

	return size;
}

static ssize_t led_timeout_read( struct device *dev, struct device_attribute *attr, char *buf )
{
	return sprintf(buf,"%d\n", cm_led_timeout);
}

static ssize_t led_timeout_write( struct device *dev, struct device_attribute *attr, const char *buf, size_t size )
{
	sscanf(buf,"%d\n", &cm_led_timeout);
	return size;
}

static ssize_t notification_timeout_read( struct device *dev, struct device_attribute *attr, char *buf )
{
	return sprintf(buf,"%d\n", cm_notification_timeout);
}

static ssize_t notification_timeout_write( struct device *dev, struct device_attribute *attr, const char *buf, size_t size )
{
	sscanf(buf,"%d\n", &cm_notification_timeout);
	return size;
}

static ssize_t notification_enabled_read( struct device *dev, struct device_attribute *attr, char *buf )
{
	return sprintf(buf,"%d\n", cm_notification_enabled);
}

static ssize_t notification_enabled_write( struct device *dev, struct device_attribute *attr, const char *buf, size_t size )
{
	sscanf(buf,"%d\n", &cm_notification_enabled);
	return size;
}

static DEVICE_ATTR(led, S_IRUGO | S_IWUGO, led_status_read, led_status_write );
static DEVICE_ATTR(led_timeout, S_IRUGO | S_IWUGO, led_timeout_read, led_timeout_write );
static DEVICE_ATTR(notification_timeout, S_IRUGO | S_IWUGO, notification_timeout_read, notification_timeout_write );
static DEVICE_ATTR(notification_enabled, S_IRUGO | S_IWUGO, notification_enabled_read, notification_enabled_write );

static struct attribute *cm_bl_led_attributes[] = {
	&dev_attr_led.attr,
	&dev_attr_led_timeout.attr,
	&dev_attr_notification_timeout.attr,
	&dev_attr_notification_enabled.attr,
	NULL
};

static struct attribute_group cm_bln_notification_group = {
	.attrs = cm_bl_led_attributes,
};

static struct miscdevice cm_led_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "notification",
};

/*
 * End of the main LED Notification code block, minor ones below
 */
#endif // CONFIG_CM_BLN


#ifdef CONFIG_HAS_EARLYSUSPEND
#if defined(CONFIG_MACH_C1_KDDI_REV00) 
static int melfas_touchkey_early_suspend(struct early_suspend *h)
{
	int ret;

	down(&bln_sem);

	disable_irq(IRQ_TOUCH_INT);
	ret = cancel_work_sync(&touchkey_work);
	if( ret ) {
		printk(KERN_DEBUG "[Touchkey] melfas_touchkey_early_suspend enable_irq... cancel_work_sync ret=%d\n", ret);
		enable_irq(IRQ_TOUCH_INT);
	}

	input_report_key(touchkey_driver->input_dev, touchkey_keycode[1], 0);
	input_report_key(touchkey_driver->input_dev, touchkey_keycode[2], 0);

	set_touchkey_debug('S');

	printk(KERN_DEBUG "[TouchKey] melfas_touchkey_early_suspend\n");

	if (touchkey_enable < 0) {
		printk(KERN_DEBUG "[TouchKey] ---%s---touchkey_enable: %d Just Return Here!\n", __func__, touchkey_enable);
		up(&bln_sem);
		return 0;
	}

	led_i2c_command(0);

	touchkey_enable = 0;
#ifdef CONFIG_GENERIC_BLN
    touchkey_suspend = true;
#endif

	gpio_direction_input(_3_GPIO_TOUCH_INT);

	/* disable ldo18 */
	touchkey_led_ldo_on(0);

	/* disable ldo11 */
	touchkey_ldo_on(0);

	up(&bln_sem);
	return 0;
}

static int melfas_touchkey_late_resume(struct early_suspend *h)
{
	set_touchkey_debug('R');

	printk(KERN_DEBUG "[TouchKey] melfas_touchkey_late_resume\n");

	down(&bln_sem);

#ifdef CONFIG_CM_BLN
	/* Avoid race condition with LED notification disable */
	down(&cm_enable_sem);
#endif

	/* enable ldo11 */
	touchkey_ldo_on(1);

	if (touchkey_enable < 0) 
	{
		printk(KERN_DEBUG "[TouchKey] ---%s---touchkey_enable: %d Just Return Here!\n", __func__, touchkey_enable);
#ifdef CONFIG_CM_BLN
		up(&cm_enable_sem);
#endif
		up(&bln_sem);
		return 0;
	}

	gpio_direction_output(_3_GPIO_TOUCH_EN, 1);
	gpio_direction_output(_3_TOUCH_SDA_28V, 1);
	gpio_direction_output(_3_TOUCH_SCL_28V, 1);
	gpio_direction_output(_3_GPIO_TOUCH_INT, 1);

	set_irq_type(IRQ_TOUCH_INT, IRQF_TRIGGER_FALLING);
	s3c_gpio_cfgpin(_3_GPIO_TOUCH_INT, _3_GPIO_TOUCH_INT_AF);
	s3c_gpio_setpull(_3_GPIO_TOUCH_INT, S3C_GPIO_PULL_NONE);
	msleep(50);
	touchkey_led_ldo_on(1);

#ifdef CONFIG_CM_BLN
	cm_screen_on = 1;
	/* see if late_resume is running before DISABLE_BL */
	if (led_on) {
		/* if a notification timeout was set, disable the timer */
		if (notification_timeout > 0) {
			del_timer(&cm_notification_timer);
		}

		/* we were using a wakelock, unlock it */
		if (wake_lock_active(&cm_led_wake_lock)) {
			wake_unlock(&cm_led_wake_lock);
		}

		led_on = 0; /* force DISABLE_BL to ignore the led state because we want it left on */
	}

	if (cm_led_timeout != BL_ALWAYS_OFF) {
		/* ensure the light is ON */
		int status = 1;
		printk(KERN_ERR "[TouchKey] %d : %s(%d)\n", __LINE__, __func__, status);
		i2c_touchkey_write((u8 *)&status, 1);
	}

	/* restart the timer if needed */
	if (cm_led_timeout > 0) {
		mod_timer(&cm_led_timer, jiffies + msecs_to_jiffies(led_timeout));
	}
#endif

#ifdef CONFIG_GENERIC_BLN
    touchkey_suspend = false;
	printk(KERN_DEBUG "[TouchKey] bln_wake_unlock\n");
	wake_unlock(&bln_wake_lock);
#endif

	touchkey_enable = 1;

	led_i2c_command(0);

	msleep(50);
	touchkey_autocalibration();
	msleep(200);

#ifdef CONFIG_CM_BLN
	/* Avoid race condition with LED notification disable */
	up(&cm_enable_sem);
#endif

	if (touchled_cmd_reversed) {
		int value = touchkey_convert_led_value(touchkey_led_status);
		touchled_cmd_reversed = 0;
		i2c_touchkey_write(&value, 1);
		printk("LED returned on\n");
	}

	up(&bln_sem);
	return 0;
}
#else //#if defined(CONFIG_MACH_C1_KDDI_REV00) 
static int melfas_touchkey_early_suspend(struct early_suspend *h)
{
	down(&bln_sem);

#if defined(CONFIG_TARGET_LOCALE_NA) || defined(CONFIG_TARGET_LOCALE_NAATT)
	 /* release key */
	input_report_key(touchkey_driver->input_dev, touchkey_keycode[1], 0);
	input_report_key(touchkey_driver->input_dev, touchkey_keycode[2], 0);
	input_report_key(touchkey_driver->input_dev, touchkey_keycode[3], 0);
	input_report_key(touchkey_driver->input_dev, touchkey_keycode[4], 0);
#elif defined(CONFIG_MACH_Q1_REV02)
	input_report_key(touchkey_driver->input_dev, touchkey_keycode[1], 0);
	input_report_key(touchkey_driver->input_dev, touchkey_keycode[2], 0);
#endif

	touchkey_enable = 0;
#ifdef CONFIG_GENERIC_BLN
    touchkey_suspend = true;
#endif

	set_touchkey_debug('S');
	printk(KERN_DEBUG "[TouchKey] melfas_touchkey_early_suspend\n");
	if (touchkey_enable < 0) {
		printk(KERN_DEBUG "[TouchKey] ---%s---touchkey_enable: %d\n", __func__, touchkey_enable);
		up(&bln_sem);
		return 0;
	}

	disable_irq(IRQ_TOUCH_INT);
	gpio_direction_input(_3_GPIO_TOUCH_INT);

#if 0
	gpio_direction_output(_3_GPIO_TOUCH_EN, 0);
	gpio_direction_output(_3_TOUCH_SDA_28V, 0);
	gpio_direction_output(_3_TOUCH_SCL_28V, 0);
	s3c_gpio_setpull(_3_GPIO_TOUCH_INT, S3C_GPIO_PULL_DOWN);
#endif

	/* disable ldo18 */
	touchkey_led_ldo_on(0);

	/* disable ldo11 */
	touchkey_ldo_on(0);

	up(&bln_sem);
	return 0;
}

static int melfas_touchkey_late_resume(struct early_suspend *h)
{
	set_touchkey_debug('R');
	printk(KERN_DEBUG "[TouchKey] melfas_touchkey_late_resume\n");

	down(&bln_sem);

#ifdef CONFIG_CM_BLN
	/* Avoid race condition with LED notification disable */
	down(&cm_enable_sem);
#endif

	/* enable ldo11 */
	touchkey_ldo_on(1);

	if (touchkey_enable < 0) {
		printk(KERN_DEBUG "[TouchKey] ---%s---touchkey_enable: %d\n", __func__, touchkey_enable);
#ifdef CONFIG_CM_BLN
		up(&cm_enable_sem);
#endif
		up(&bln_sem);
		return 0;
	}

	gpio_direction_output(_3_GPIO_TOUCH_EN, 1);
	gpio_direction_output(_3_TOUCH_SDA_28V, 1);
	gpio_direction_output(_3_TOUCH_SCL_28V, 1);

	gpio_direction_output(_3_GPIO_TOUCH_INT, 1);
	set_irq_type(IRQ_TOUCH_INT, IRQF_TRIGGER_FALLING);
	s3c_gpio_cfgpin(_3_GPIO_TOUCH_INT, _3_GPIO_TOUCH_INT_AF);
	s3c_gpio_setpull(_3_GPIO_TOUCH_INT, S3C_GPIO_PULL_NONE);
	msleep(50); // 50msec required for touchkey device to wakeup

	touchkey_led_ldo_on(1);

#ifdef CONFIG_CM_BLN
	cm_screen_on = 1;
	/* see if late_resume is running before DISABLE_BL */
	if (cm_led_on) {
		/* if a notification timeout was set, disable the timer */
		if (cm_notification_timeout > 0) {
			del_timer(&cm_notification_timer);
		}

		/* we were using a wakelock, unlock it */
		if (wake_lock_active(&cm_led_wake_lock)) {
			wake_unlock(&cm_led_wake_lock);
		}

		cm_led_on = 0; /* force DISABLE_BL to ignore the led state because we want it left on */
	}

	if (cm_led_timeout != CM_BL_ALWAYS_OFF) {
		/* ensure the light is ON */
		int status = 1;
		printk(KERN_ERR "[TouchKey] %d : %s(%d)\n", __LINE__, __func__, status);
		i2c_touchkey_write((u8 *)&status, 1);
	}

	/* restart the timer if needed */
	if (cm_led_timeout > 0) {
		mod_timer(&cm_led_timer, jiffies + msecs_to_jiffies(cm_led_timeout));
	}
#endif

#ifdef CONFIG_GENERIC_BLN
	touchkey_suspend = false;
	printk(KERN_DEBUG "[TouchKey] bln_wake_unlock\n");
	wake_unlock(&bln_wake_lock);
#endif

	enable_irq(IRQ_TOUCH_INT);
	touchkey_enable = 1;

#if defined(CONFIG_TARGET_LOCALE_NAATT) || defined(CONFIG_TARGET_LOCALE_NA) || defined(CONFIG_MACH_Q1_REV02)
#ifndef CONFIG_MACH_C1_NA_SPR_EPIC2_REV00
#if defined(CONFIG_TARGET_LOCALE_NA)
	if (store_module_version >= 8) {
#endif
#endif
		msleep(50);
		touchkey_autocalibration();
		msleep(200);
#ifndef CONFIG_MACH_C1_NA_SPR_EPIC2_REV00
#if defined(CONFIG_TARGET_LOCALE_NA)
	}
#endif /* CONFIG_TARGET_LOCALE_NA */
#endif
#endif

#ifdef CONFIG_CM_BLN
	/* Avoid race condition with LED notification disable */
	up(&cm_enable_sem);
#endif

	if (touchled_cmd_reversed) {
		int value = touchkey_convert_led_value(touchkey_led_status);
		touchled_cmd_reversed = 0;
		i2c_touchkey_write((u8 *)&value, 1);
		printk("LED returned on\n");
	}

#ifdef CONFIG_GENERIC_BLN
	up(&bln_sem);
#endif
	return 0;
}

#endif //#if defined(CONFIG_MACH_C1_KDDI_REV00) 
#endif // CONFIG_HAS_EARLYSUSPEND

extern int mcsdl_download_binary_data(void);
static int i2c_touchkey_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct input_dev *input_dev;
	int err = 0;
	unsigned char data;
	/*struct regulator *regulator;*/

	printk(KERN_DEBUG "[TouchKey] melfas i2c_touchkey_probe\n");

	touchkey_driver = kzalloc(sizeof(struct i2c_touchkey_driver), GFP_KERNEL);
	if (touchkey_driver == NULL) {
		dev_err(dev, "failed to create our state\n");
		return -ENOMEM;
	}

	touchkey_driver->client = client;
	touchkey_driver->client->irq = IRQ_TOUCH_INT;
	strlcpy(touchkey_driver->client->name, "melfas-touchkey", I2C_NAME_SIZE);

	input_dev = input_allocate_device();

	if (!input_dev) {
		return -ENOMEM;
	}

	touchkey_driver->input_dev = input_dev;

	input_dev->name = DEVICE_NAME;
	input_dev->phys = "melfas-touchkey/input0";
	input_dev->id.bustype = BUS_HOST;

#if defined(CONFIG_MACH_C1_NA_SPR_EPIC2_REV00)
	 touchkey_keycode[1] = KEY_MENU;
	 touchkey_keycode[2] = KEY_HOME;
	 touchkey_keycode[3] = KEY_SEARCH;
	 touchkey_keycode[4] = KEY_BACK;
#endif

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(touchkey_keycode[1], input_dev->keybit);
	set_bit(touchkey_keycode[2], input_dev->keybit);
#ifdef CONFIG_TARGET_LOCALE_NA
	set_bit(touchkey_keycode[3], input_dev->keybit);
	set_bit(touchkey_keycode[4], input_dev->keybit);
#endif
#if defined(CONFIG_TARGET_LOCALE_NAATT)
	set_bit(touchkey_keycode[3], input_dev->keybit);
	set_bit(touchkey_keycode[4], input_dev->keybit);
#endif

	err = input_register_device(input_dev);
	if (err) {
		input_free_device(input_dev);
		return err;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	touchkey_driver->early_suspend.suspend = (void *) melfas_touchkey_early_suspend;
	touchkey_driver->early_suspend.resume = (void *) melfas_touchkey_late_resume;
	register_early_suspend(&touchkey_driver->early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */

	/* enable ldo11 */
	touchkey_ldo_on(1);
	msleep(50);
	touchkey_enable = 1;
	data = 1;

#if defined (CONFIG_MACH_Q1_REV02)
	if (get_touchkey_module_version() < 0) {
		printk(KERN_ERR"[TouchKey] Probe fail\n");
		input_free_device(input_dev);
		touchkey_probe = false;
		return -ENODEV;
	}
#endif

	if (request_irq(IRQ_TOUCH_INT, touchkey_interrupt, IRQF_TRIGGER_FALLING, DEVICE_NAME, NULL)) {
		printk(KERN_ERR "[TouchKey] %s Can't allocate irq ..\n", __func__);
		return -EBUSY;
	}

	/* enable ldo18 */
	touchkey_led_ldo_on(1);


#if defined(CONFIG_MACH_C1_NA_SPR_EPIC2_REV00)
	/* touch key firmware auto update */
	touchkey_firmware_update_epic2();
#elif defined (CONFIG_MACH_Q1_REV02)
	touchkey_firmware_update_q1();
#elif defined(CONFIG_MACH_C1_KDDI_REV00)	//Kishore
	touchkey_firmware_update_k1(0);
#endif

#if defined(CONFIG_TARGET_LOCALE_NAATT)|| defined(CONFIG_TARGET_LOCALE_NA) || defined(CONFIG_MACH_Q1_REV02)
	/*touchkey_firmware_update();*/
#ifndef CONFIG_MACH_C1_NA_SPR_EPIC2_REV00
#if defined(CONFIG_TARGET_LOCALE_NA)
	if(store_module_version >= 8){
#endif
#endif
		msleep(100);
		err = touchkey_autocalibration();
		if (err < 0) {
			printk(KERN_ERR"[TouchKey] probe autocalibration fail\n");
			return err;
		}
#ifndef CONFIG_MACH_C1_NA_SPR_EPIC2_REV00
#if defined(CONFIG_TARGET_LOCALE_NA)
	}
#endif /* CONFIG_TARGET_LOCALE_NA */
#endif
#endif

#if defined(CONFIG_MACH_C1_KDDI_REV00) 	
	//led_i2c_command(1);
#endif

	set_touchkey_debug('K');

#ifdef CONFIG_CM_BLN
	err = misc_register(&cm_led_device);
	if (err) {
		printk(KERN_ERR "[LED Notify] sysfs misc_register failed.\n");
	} else {
		if( sysfs_create_group( &cm_led_device.this_device->kobj, &cm_bln_notification_group) < 0){
			printk(KERN_ERR "[LED Notify] sysfs create group failed.\n");
		}
	}

	/* Setup the timer for the timeouts */
	setup_timer(&cm_led_timer, handle_led_timeout, 0);
	setup_timer(&cm_notification_timer, handle_notification_timeout, 0);

	/* wake lock for LED Notify */
	wake_lock_init(&cm_led_wake_lock, WAKE_LOCK_SUSPEND, "led_wake_lock");

	/* turn on the LED if it is not supposed to be allways off */
	if (cm_led_timeout != CM_BL_ALWAYS_OFF) {
		int status = 1;
        printk(KERN_ERR "[TouchKey] %d : %s(%d)\n", __LINE__, __func__, status);
		i2c_touchkey_write((u8 *)&status, 1);
	}
#endif

	return 0;
}

static void init_hw(void)
{
	gpio_direction_output(_3_GPIO_TOUCH_EN, 1);
	touchkey_ldo_on(1);	// touch ldo on.
	msleep(200);
	s3c_gpio_setpull(_3_GPIO_TOUCH_INT, S3C_GPIO_PULL_NONE);
	set_irq_type(IRQ_TOUCH_INT, IRQF_TRIGGER_FALLING);
	s3c_gpio_cfgpin(_3_GPIO_TOUCH_INT, _3_GPIO_TOUCH_INT_AF);
}

#if defined(CONFIG_TARGET_LOCALE_NA) || defined(CONFIG_MACH_Q1_REV02)
int get_touchkey_module_version()
{
    char data[3]= { 0, };
	int ret = 0;

	ret = i2c_touchkey_read(KEYCODE_REG, data, 3);
	if (ret < 0) {
		printk(KERN_ERR"[TouchKey] module version read fail\n");
		return ret;
	} else {
    	printk(KERN_DEBUG "[TouchKey] Module Version: %d \n",data[2]);
		return data[2];
	}
}
#endif /* CONFIG_TARGET_LOCALE_NA */

int touchkey_update_open(struct inode *inode, struct file *filp)
{
	return 0;
}

ssize_t touchkey_update_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	char data[3] = { 0, };

	get_touchkey_firmware(data);
	put_user(data[1], buf);

	return 1;
}

int touchkey_update_release(struct inode *inode, struct file *filp)
{
	return 0;
}

struct file_operations touchkey_update_fops = {
	.owner = THIS_MODULE,
	.read = touchkey_update_read,
	.open = touchkey_update_open,
	.release = touchkey_update_release,
};

static struct miscdevice touchkey_update_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "melfas_touchkey",
	.fops = &touchkey_update_fops,
};

static ssize_t touch_version_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	char data[3] = { 0, };
	int count;

	init_hw();

	i2c_touchkey_read(KEYCODE_REG, data, 3);

	count = sprintf(buf, "0x%x\n", data[1]);

	printk(KERN_DEBUG "[TouchKey] touch_version_read 0x%x\n", data[1]);
#ifdef CONFIG_MACH_Q1_REV02
	printk(KERN_DEBUG "[TouchKey] module_version_read 0x%x\n", data[2]);
#endif
	return count;
}

static ssize_t touch_version_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	printk(KERN_DEBUG "[TouchKey] input data --> %s\n", buf);

	return size;
}

void touchkey_update_func(struct work_struct *p)
{
//Kishore.siso
#if defined(CONFIG_MACH_C1_KDDI_REV00)
	touchkey_firmware_update_k1(0x18);
	enable_irq(IRQ_TOUCH_INT);
#else //#if defined(CONFIG_MACH_C1_KDDI_REV00)
	int retry = 10;
#if defined(CONFIG_TARGET_LOCALE_NAATT)
	char data[3];
	i2c_touchkey_read(KEYCODE_REG, data, 3);
	printk(KERN_DEBUG"[%s] F/W version: 0x%x, Module version:0x%x\n", __FUNCTION__, data[1], data[2]);
#endif

	touchkey_update_status = 1;
	printk(KERN_DEBUG "[TouchKey] %s start\n", __func__);
	touchkey_enable = 0;
	while (retry--) {
		if (ISSP_main() == 0) {
			printk(KERN_DEBUG "[TouchKey] touchkey_update succeeded\n");
			init_hw();
			enable_irq(IRQ_TOUCH_INT);
			touchkey_enable = 1;
#if defined(CONFIG_MACH_C1_NA_SPR_EPIC2_REV00) || defined(CONFIG_MACH_Q1_REV02)
			touchkey_autocalibration();
#else
#if defined(CONFIG_TARGET_LOCALE_NA)
			if(store_module_version >= 8){
				touchkey_autocalibration();
			}
#endif
#endif
			touchkey_update_status = 0;
			return;
		}
#if defined(CONFIG_TARGET_LOCALE_NAATT) || defined(CONFIG_TARGET_LOCALE_NA) || defined(CONFIG_MACH_Q1_REV02)
		touchkey_ldo_on(0);
		msleep(300);
		init_hw();
#endif
	}

	touchkey_update_status = -1;
	printk(KERN_DEBUG "[TouchKey] touchkey_update failed\n");
	return;
//Kishore.siso
#endif  //#if defined(CONFIG_MACH_C1_KDDI_REV00)
}

static ssize_t touch_update_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
//Kishore.siso
#if defined(CONFIG_MACH_C1_KDDI_REV00)

		printk(KERN_DEBUG "[TouchKey] touchkey firmware update in touch_update_write()\n");
		if (*buf == 'S') {
			disable_irq(IRQ_TOUCH_INT);
			INIT_WORK(&touch_update_work, touchkey_update_func);
			queue_work(touchkey_wq, &touch_update_work);
		}
#else //#ifdef CONFIG_MACH_C1_KDDI_REV00
#ifndef CONFIG_MACH_C1_NA_SPR_EPIC2_REV00
#ifdef CONFIG_TARGET_LOCALE_NA
	if(store_module_version < 8){
		printk(KERN_DEBUG "[TouchKey] Skipping firmware update for rev05 and old devices: module_version =%d\n",store_module_version);
		touchkey_update_status=0;
                return 1;
	}else {
#endif /* CONFIG_TARGET_LOCALE_NA */
#endif
		printk(KERN_DEBUG "[TouchKey] touchkey firmware update \n");

		if (*buf == 'S') {
			disable_irq(IRQ_TOUCH_INT);
			INIT_WORK(&touch_update_work, touchkey_update_func);
			queue_work(touchkey_wq, &touch_update_work);
		}
		return size;
#ifndef CONFIG_MACH_C1_NA_SPR_EPIC2_REV00
#ifdef CONFIG_TARGET_LOCALE_NA
	}
#endif /* CONFIG_TARGET_LOCALE_NA */
#endif
#endif //#ifdef CONFIG_MACH_C1_KDDI_REV00
}

static ssize_t touch_update_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = 0;

	printk(KERN_DEBUG "[TouchKey] touch_update_read: touchkey_update_status %d\n", touchkey_update_status);

	if (touchkey_update_status == 0) {
		count = sprintf(buf, "PASS\n");
	} else if (touchkey_update_status == 1) {
		count = sprintf(buf, "Downloading\n");
	} else if (touchkey_update_status == -1) {
		count = sprintf(buf, "Fail\n");
	}

	return count;
}

static ssize_t touch_led_control(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int data, value;
	int errnum;
	if (sscanf(buf, "%d\n", &data) == 1) {
#ifdef CONFIG_CM_BLN
		if (cm_led_timeout == CM_BL_ALWAYS_OFF && data == 1) {
			return size;
		}
#endif
		value = touchkey_convert_led_value(data);
		printk("[TouchKey] led %s\n", (value == 0x1 || value == 0x10) ? "on" : "off");
		errnum = i2c_touchkey_write((u8 *)&value, 1);
		if (errnum == -ENODEV) {
			touchled_cmd_reversed = 1;
		}
		touchkey_led_status = data;
	} else {
		printk(KERN_DEBUG "[TouchKey] touch_led_control Error\n");
	}

	return size;
}

static ssize_t touchkey_enable_disable(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}

#if defined(CONFIG_TARGET_LOCALE_NAATT) || defined(CONFIG_TARGET_LOCALE_NA)
#if defined(CONFIG_MACH_C1_KDDI_REV00) 
static ssize_t touchkey_menu_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[14] = {0, };
	int ret;

	ret = i2c_touchkey_read(KEYCODE_REG, data, 14);
	printk("called %s data[13] =%d\n", __func__, data[13]);
	menu_sensitivity = data[13];

	return sprintf(buf,"%d\n",menu_sensitivity);
}

static ssize_t touchkey_back_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[14] = {0, };
	int ret;

	ret = i2c_touchkey_read(KEYCODE_REG, data, 14);
	printk("called %s data[11] =%d\n", __func__, data[11]);
	back_sensitivity = data[11];
	return sprintf(buf, "%d\n", back_sensitivity);
}

static ssize_t touchkey_home_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	home_sensitivity = 0;
	return sprintf(buf, "%d\n", home_sensitivity);
}

static ssize_t touchkey_search_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	search_sensitivity = 0;
	return sprintf(buf, "%d\n", search_sensitivity);
}

#else //#if defined(CONFIG_MACH_C1_KDDI_REV00) 
static ssize_t touchkey_menu_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[18] = {0, };
	int ret;

	printk("called %s \n", __func__);
	ret = i2c_touchkey_read(KEYCODE_REG, data, 18);
#ifdef CONFIG_TARGET_LOCALE_NA
#if defined(CONFIG_MACH_C1_NA_SPR_EPIC2_REV00)
	printk("called %s data[11] =%d\n", __func__, data[11]);
	menu_sensitivity = data[11];
#else
	if(store_module_version < 8){
		printk("called %s data[12] =%d,data[13] = %d\n", __func__, data[12], data[13]);
		menu_sensitivity = ((0x00FF&data[12])<<8) | data[13];
	}else {
		printk("called %s data[17] =%d\n", __func__, data[17]);
		menu_sensitivity = data[17];
	}
#endif
#else
	printk("called %s data[10] =%d,data[11] = %d\n", __func__, data[10], data[11]);
	menu_sensitivity = ((0x00FF&data[10])<<8) | data[11];
#endif /* CONFIG_TARGET_LOCALE_NA */
	return sprintf(buf, "%d\n", menu_sensitivity);
}

static ssize_t touchkey_home_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[18] = {0, };
	int ret;

	printk("called %s \n", __func__);
	ret = i2c_touchkey_read(KEYCODE_REG, data, 18);
#ifdef CONFIG_TARGET_LOCALE_NA
#if defined(CONFIG_MACH_C1_NA_SPR_EPIC2_REV00)
	printk("called %s data[13] =%d\n",__func__, data[13]);
	home_sensitivity = data[13];
#else
	if(store_module_version < 8){
		printk("called %s data[10] =%d,data[11] = %d\n", __func__, data[10], data[11]);
		home_sensitivity = ((0x00FF&data[10])<<8) | data[11];
	}else {
		printk("called %s data[15] =%d\n",__func__, data[15]);
		home_sensitivity = data[15];
	}
#endif
#else
	printk("called %s data[12] =%d,data[13] = %d\n", __func__, data[12], data[13]);
	home_sensitivity = ((0x00FF&data[12])<<8) | data[13];
#endif /* CONFIG_TARGET_LOCALE_NA */
	return sprintf(buf, "%d\n", home_sensitivity);
}

static ssize_t touchkey_back_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[18] = {0, };
	int ret;

	printk("called %s \n", __func__);
	ret = i2c_touchkey_read(KEYCODE_REG, data, 18);
#ifdef CONFIG_TARGET_LOCALE_NA
#if defined(CONFIG_MACH_C1_NA_SPR_EPIC2_REV00)
	printk("called %s data[17] =%d\n", __func__, data[17]);
	back_sensitivity = data[17];
#else
	if (store_module_version < 8) {
		printk("called %s data[8] =%d,data[9] = %d\n", __func__, data[8], data[9]);
		back_sensitivity = ((0x00FF&data[8])<<8)  | data[9];
	} else {
		printk("called %s data[13] =%d\n", __func__, data[13]);
		back_sensitivity = data[13];
	}
#endif
#else
	printk("called %s data[14] =%d,data[15] = %d\n", __func__, data[14], data[15]);
	back_sensitivity = ((0x00FF&data[14])<<8)  | data[15];
#endif /* CONFIG_TARGET_LOCALE_NA */
	return sprintf(buf, "%d\n", back_sensitivity);
}

static ssize_t touchkey_search_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[18] = {0, };
	int ret;

	printk("called %s \n", __func__);
	ret = i2c_touchkey_read(KEYCODE_REG, data, 18);
#ifdef CONFIG_TARGET_LOCALE_NA
#if defined(CONFIG_MACH_C1_NA_SPR_EPIC2_REV00)
	printk("called %s data[15] =%d\n", __func__, data[15]);
	search_sensitivity = data[15];
#else
	if (store_module_version < 8) {
		printk("called %s data[6] =%d,data[7] = %d\n", __func__, data[6], data[7]);
		search_sensitivity = ((0x00FF&data[6])<<8)  | data[7];
	}else {
		printk("called %s data[11] =%d\n", __func__, data[11]);
		search_sensitivity = data[11];
	}
#endif
#else
	printk("called %s data[16] =%d,data[17] = %d\n", __func__, data[16], data[17]);
	search_sensitivity = ((0x00FF&data[16])<<8)  | data[17];
#endif /* CONFIG_TARGET_LOCALE_NA */
	return sprintf(buf, "%d\n", search_sensitivity);
}
#endif //#if defined(CONFIG_MACH_C1_KDDI_REV00) 
#else
static ssize_t touchkey_menu_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#if defined(CONFIG_MACH_Q1_REV02)
	u8 data[14] = {0, };
	int ret;

	ret = i2c_touchkey_read(KEYCODE_REG, data, 14);

	printk("called %s data[13] =%d\n", __func__, data[13]);
	menu_sensitivity = data[13];
#else
	u8 data[10];
	int ret;

	printk(KERN_DEBUG "[TouchKey] called %s \n",__func__);
	ret = i2c_touchkey_read(KEYCODE_REG, data, 10);
	menu_sensitivity = data[7];
#endif
	return sprintf(buf,"%d\n",menu_sensitivity);
}

static ssize_t touchkey_back_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#if defined(CONFIG_MACH_Q1_REV02)
	u8 data[14] = {0, };
	int ret;

	ret = i2c_touchkey_read(KEYCODE_REG, data, 14);

	printk("called %s data[11] =%d\n", __func__, data[11]);
	back_sensitivity = data[11];
#else
	u8 data[10];
	int ret;

	printk(KERN_DEBUG "[TouchKey] called %s \n",__func__);
	ret = i2c_touchkey_read(KEYCODE_REG, data, 10);
	back_sensitivity = data[9];
#endif
	return sprintf(buf, "%d\n", back_sensitivity);
}
#endif

#if defined( CONFIG_TARGET_LOCALE_NA) || defined(CONFIG_MACH_Q1_REV02)
static ssize_t autocalibration_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int data;

	sscanf(buf, "%d\n", &data);

	if(data == 1)
		touchkey_autocalibration();

	return size;
}

static ssize_t autocalibration_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data[6];
	int ret;

	printk("called %s \n",__func__);

	ret = i2c_touchkey_read(KEYCODE_REG, data, 6);
	if((data[5] & 0x80))
		return sprintf(buf,"Enabled\n");
	else
		return sprintf(buf,"Disabled\n");
}
#endif /* CONFIG_TARGET_LOCALE_NA */

static ssize_t touch_sensitivity_control(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned char data = 0x40;
	i2c_touchkey_write(&data, 1);
	return size;
}

static ssize_t set_touchkey_firm_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#if defined(CONFIG_MACH_Q1_REV02)
	int count;
	if( module_version == 0x09 || module_version == 0x10 )
		count = sprintf(buf, "0x10\n");	
	else
	count = sprintf(buf, "0x%x\n", Q1_TK_FIRMWARE_VER);
	return count;
#else
#if defined(CONFIG_MACH_C1_NA_SPR_EPIC2_REV00)
	int count;
	count = sprintf(buf, "0x%x\n", EPIC2_TK_FIRMWARE_VER);
	return count;
#else
#if defined(CONFIG_MACH_C1_KDDI_REV00) 
	int count;
	count = sprintf(buf, "0x%x\n", K1_FIRMWARE_VERSION);
	return count;
#else //#if defined(CONFIG_MACH_C1_KDDI_REV00) 
	/*TO DO IT */
	int count;
	count = sprintf(buf, "0x%x\n", TOUCH_FIRMWARE_V04);
	return count;
#endif //#if defined(CONFIG_MACH_C1_KDDI_REV00) 
#endif
#endif
}

static ssize_t set_touchkey_update_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#if defined(CONFIG_MACH_C1_KDDI_REV00)
	if(touchkey_firmware_update_k1(0x18))
	{
		return sprintf(buf, "Fail FW Update\n");
	}
	else
	{
		return sprintf(buf, "OK FW Update\n");
	}
#else //#if defined(CONFIG_MACH_C1_KDDI_REV00)
	/* TO DO IT */
	int count = 0;
	int retry = 3;
	touchkey_update_status = 1;

#if defined(CONFIG_MACH_C1_NA_SPR_EPIC2_REV00)
	char data[3] = { 0, };

	i2c_touchkey_read(KEYCODE_REG, data, 3);

	if (data[2] != EPIC2_TK_MODULE_VER) {
		printk("[TouchKey] diferrent module.(binary_ver:%d, read_ver:%d)\n",
			EPIC2_TK_MODULE_VER, data[2]);
		printk("[TouchKey] do not excute firm update\n");
		touchkey_update_status = -1;
		return count;
	}
	if (data[1] > EPIC2_TK_FIRMWARE_VER) {
		printk("[TouchKey] read version is latest(binary_ver:%d, read_ver:%d)\n",
			EPIC2_TK_FIRMWARE_VER, data[1]);
		printk("[TouchKey] do not excute firm update\n");
		touchkey_update_status = -1;
		return count;
	}
#endif
	while (retry--) {
			if (ISSP_main() == 0) {
				printk(KERN_ERR"[TOUCHKEY]Touchkey_update succeeded\n");
				touchkey_update_status = 0;
				count = 1;
				break;
			}
			printk(KERN_ERR"touchkey_update failed... retry...\n");
	}
	if (retry <= 0) {
			/* disable ldo11 */
			touchkey_ldo_on(0);
			msleep(300);
			count = 0;
			printk(KERN_ERR"[TOUCHKEY]Touchkey_update fail\n");
			touchkey_update_status = -1;
			return count;
	}

	init_hw();	/* after update, re initalize. */
#ifdef CONFIG_MACH_C1_NA_SPR_EPIC2_REV00
	touchkey_autocalibration();
#endif

	return count;
#endif //#if defined(CONFIG_MACH_C1_KDDI_REV00)
}

static ssize_t set_touchkey_firm_version_read_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	char data[3] = { 0, };
	int count;
#if defined(CONFIG_MACH_C1_KDDI_REV00)
	data[1] = touch_version;
	data[2] = module_version;
#else
	init_hw();
	i2c_touchkey_read(KEYCODE_REG, data, 3);
#endif
	count = sprintf(buf, "0x%x\n", data[1]);

	printk(KERN_DEBUG "[TouchKey] touch_version_read 0x%x\n", data[1]);
	printk(KERN_DEBUG "[TouchKey] module_version_read 0x%x\n", data[2]);
	return count;
}

static ssize_t set_touchkey_firm_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = 0;

	printk(KERN_DEBUG "[TouchKey] touch_update_read: touchkey_update_status %d\n", touchkey_update_status);

	if (touchkey_update_status == 0) {
		count = sprintf(buf, "PASS\n");
	} else if (touchkey_update_status == 1) {
		count = sprintf(buf, "Downloading\n");
	} else if (touchkey_update_status == -1) {
		count = sprintf(buf, "Fail\n");
	}

	return count;
}

#ifdef CONFIG_GENERIC_BLN
static ssize_t touchkey_bln_control(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t size)
{
	int data, value, errnum;

	if (sscanf(buf, "%d\n", &data) == 1) {
		printk(KERN_ERR "[TouchKey] touchkey_bln_control: %d \n", data);
		value = touchkey_convert_led_value(data);
		printk("[TouchKey] led %s\n", (value == 0x1 || value == 0x10) ? "on" : "off");
		errnum = i2c_touchkey_write((u8 *)&value, 1);
		if (errnum == -ENODEV) {
			touchled_cmd_reversed = 1;
		}
		touchkey_led_status = data;
	} else {
		printk(KERN_ERR "[TouchKey] touchkey_bln_control Error\n");
	}

	return size;
}
#endif

static DEVICE_ATTR(touch_version, S_IRUGO | S_IWUSR | S_IWGRP, touch_version_read, touch_version_write);
static DEVICE_ATTR(touch_update, S_IRUGO | S_IWUSR | S_IWGRP, touch_update_read, touch_update_write);
static DEVICE_ATTR(brightness, S_IRUGO | S_IWUSR | S_IWGRP, NULL, touch_led_control);
static DEVICE_ATTR(enable_disable, S_IRUGO | S_IWUSR | S_IWGRP, NULL, touchkey_enable_disable);
static DEVICE_ATTR(touchkey_menu, S_IRUGO | S_IWUSR | S_IWGRP, touchkey_menu_show, NULL);
static DEVICE_ATTR(touchkey_back, S_IRUGO | S_IWUSR | S_IWGRP, touchkey_back_show, NULL);
#if defined(CONFIG_TARGET_LOCALE_NA) || defined(CONFIG_TARGET_LOCALE_NAATT)
static DEVICE_ATTR(touchkey_home, S_IRUGO, touchkey_home_show, NULL);
static DEVICE_ATTR(touchkey_search, S_IRUGO, touchkey_search_show, NULL);
#endif /* CONFIG_TARGET_LOCALE_NA  */
static DEVICE_ATTR(touch_sensitivity, S_IRUGO | S_IWUSR | S_IWGRP, NULL, touch_sensitivity_control);
/*20110223N1 firmware sync*/
static DEVICE_ATTR(touchkey_firm_update, S_IRUGO | S_IWUSR | S_IWGRP, set_touchkey_update_show, NULL);		/* firmware update */
static DEVICE_ATTR(touchkey_firm_update_status, S_IRUGO | S_IWUSR | S_IWGRP, set_touchkey_firm_status_show, NULL);	/* firmware update status return */
static DEVICE_ATTR(touchkey_firm_version_phone, S_IRUGO | S_IWUSR | S_IWGRP, set_touchkey_firm_version_show, NULL);/* PHONE*/	/* firmware version resturn in phone driver version */
static DEVICE_ATTR(touchkey_firm_version_panel, S_IRUGO | S_IWUSR | S_IWGRP, set_touchkey_firm_version_read_show, NULL);/*PART*/	/* firmware version resturn in touchkey panel version */
/*end N1 firmware sync*/

static DEVICE_ATTR(touchkey_brightness, S_IRUGO | S_IWUSR | S_IWGRP, NULL, brightness_control);
#ifdef CONFIG_GENERIC_BLN
static DEVICE_ATTR(touchkey_bln_control, S_IWUGO, NULL, touchkey_bln_control);
#endif

#if defined(CONFIG_TARGET_LOCALE_NAATT)
static DEVICE_ATTR(touchkey_autocal_start, S_IRUGO | S_IWUSR | S_IWGRP, NULL, set_touchkey_autocal_testmode);
#endif

#if defined(CONFIG_TARGET_LOCALE_NAATT) || defined(CONFIG_TARGET_LOCALE_NA) || defined(CONFIG_MACH_Q1_REV02)
static DEVICE_ATTR(touchkey_raw_data0, S_IRUGO, touchkey_raw_data0_show, NULL);
static DEVICE_ATTR(touchkey_raw_data1, S_IRUGO, touchkey_raw_data1_show, NULL);
static DEVICE_ATTR(touchkey_raw_data2, S_IRUGO, touchkey_raw_data2_show, NULL);
static DEVICE_ATTR(touchkey_raw_data3, S_IRUGO, touchkey_raw_data3_show, NULL);
static DEVICE_ATTR(touchkey_idac0, S_IRUGO, touchkey_idac0_show, NULL);
static DEVICE_ATTR(touchkey_idac1, S_IRUGO, touchkey_idac1_show, NULL);
static DEVICE_ATTR(touchkey_idac2, S_IRUGO, touchkey_idac2_show, NULL);
static DEVICE_ATTR(touchkey_idac3, S_IRUGO, touchkey_idac3_show, NULL);
static DEVICE_ATTR(touchkey_threshold, S_IRUGO, touchkey_threshold_show, NULL);
#endif

#if defined(CONFIG_TARGET_LOCALE_NA) || defined(CONFIG_MACH_Q1_REV02)
static DEVICE_ATTR(autocal_enable, S_IRUGO | S_IWUSR | S_IWGRP, NULL, autocalibration_enable);
static DEVICE_ATTR(autocal_stat, S_IRUGO | S_IWUSR | S_IWGRP, autocalibration_status, NULL);
#endif /* CONFIG_TARGET_LOCALE_NA */
#if defined(CONFIG_MACH_C1_KDDI_REV00)
extern int sec_isLpmMode(void);
#endif

static int __init touchkey_init(void)
{
	int ret = 0;

	#if defined(CONFIG_MACH_C1_KDDI_REV00)
	if (sec_isLpmMode()) {
		printk(KERN_ERR "LPM MODE (sec_touchkey)!\n");
		return 0;
	}
	#endif

	if (gpio_request(_3_TOUCH_SDA_28V, "_3_TOUCH_SDA_28V"))
		WARN(1, "Fail to request gpio (_3_TOUCH_SDA_28V)\n");
	if (gpio_request(_3_TOUCH_SCL_28V, "_3_TOUCH_SCL_28V"))
		WARN(1, "Fail to request gpio (_3_TOUCH_SCL_28V)\n");
	if (gpio_request(_3_GPIO_TOUCH_EN, "_3_GPIO_TOUCH_EN"))
		WARN(1, "Fail to request gpio (_3_GPIO_TOUCH_EN)\n");
	if (gpio_request(_3_GPIO_TOUCH_INT, "_3_GPIO_TOUCH_INT"))
		WARN(1, "Fail to request gpio (_3_GPIO_TOUCH_INT)\n");

	/*20110222 N1_firmware_sync*/
	sec_touchkey = device_create(sec_class, NULL, 0, NULL, "sec_touchkey");

	if (IS_ERR(sec_touchkey)) {
		printk(KERN_ERR "Failed to create device(sec_touchkey)!\n");
	}

	if (device_create_file(sec_touchkey, &dev_attr_touchkey_firm_update) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_touchkey_firm_update.attr.name);
	}
	if (device_create_file(sec_touchkey, &dev_attr_touchkey_firm_update_status) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_touchkey_firm_update_status.attr.name);
	}
	if (device_create_file(sec_touchkey, &dev_attr_touchkey_firm_version_phone) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_touchkey_firm_version_phone.attr.name);
	}
	if (device_create_file(sec_touchkey, &dev_attr_touchkey_firm_version_panel) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_touchkey_firm_version_panel.attr.name);
	}
	if (device_create_file(sec_touchkey, &dev_attr_touchkey_brightness) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_touchkey_brightness.attr.name);
	}
	#if defined(CONFIG_TARGET_LOCALE_NAATT)
	if (device_create_file(sec_touchkey, &dev_attr_touchkey_autocal_start) < 0) {
			printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_touchkey_brightness.attr.name);
	}
	#endif
	/*end N1_firmware_sync*/

	ret = misc_register(&touchkey_update_device);
	if (ret) {
		printk(KERN_ERR "[TouchKey] %s misc_register fail\n", __func__);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touch_version) < 0) {
		printk(KERN_ERR "[TouchKey] %s device_create_file fail dev_attr_touch_version\n", __func__);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touch_update) < 0) {
		printk(KERN_ERR "[TouchKey] %s device_create_file fail dev_attr_touch_update\n", __func__);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_brightness) < 0) {
		printk(KERN_ERR "[TouchKey] %s device_create_file fail dev_attr_touch_update\n", __func__);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_enable_disable) < 0) {
		printk(KERN_ERR "[TouchKey] %s device_create_file fail dev_attr_touch_update\n", __func__);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touchkey_menu) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_touchkey_menu\n", __func__);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touchkey_back) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_touchkey_back\n", __func__);
	}
#if defined(CONFIG_TARGET_LOCALE_NAATT) || defined(CONFIG_TARGET_LOCALE_NA) || defined(CONFIG_MACH_Q1_REV02)
	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touchkey_raw_data0) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_touchkey_raw_data0\n", __func__);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touchkey_raw_data1) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_touchkey_raw_data1\n", __func__);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touchkey_raw_data2) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_touchkey_raw_data2\n", __func__);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touchkey_raw_data3) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_touchkey_raw_data3\n", __func__);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touchkey_idac0) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_touchkey_idac0\n", __func__);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touchkey_idac1) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_touchkey_idac1\n", __func__);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touchkey_idac2) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_touchkey_idac2\n", __func__);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touchkey_idac3) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_touchkey_idac3\n", __func__);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touchkey_threshold) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_touchkey_threshold\n", __func__);
	}
#endif

#if defined(CONFIG_TARGET_LOCALE_NA) || defined(CONFIG_TARGET_LOCALE_NAATT)
	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touchkey_home) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_touchkey_home\n", __func__);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touchkey_search) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_touchkey_search\n", __func__);
	}
#endif /* CONFIG_TARGET_LOCALE_NA  */

#if defined(CONFIG_TARGET_LOCALE_NA) || defined(CONFIG_MACH_Q1_REV02)
	if (device_create_file(touchkey_update_device.this_device, &dev_attr_autocal_enable) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_autocal_enable\n", __func__);
	}

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_autocal_stat) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_autocal_stat\n", __func__);
	}
#endif /* CONFIG_TARGET_LOCALE_NA */

	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touch_sensitivity) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_touch_sensitivity\n", __func__);
	}

#ifdef CONFIG_GENERIC_BLN
	if (device_create_file(touchkey_update_device.this_device, &dev_attr_touchkey_bln_control) < 0) {
		printk(KERN_ERR "%s device_create_file fail dev_attr_touchkey_bln_control\n", __func__);
	}
#endif

	touchkey_wq = create_singlethread_workqueue("melfas_touchkey_wq");
	if (!touchkey_wq) {
		return -ENOMEM;
	}

	INIT_WORK(&touchkey_work, touchkey_work_func);

	init_hw();

	ret = i2c_add_driver(&touchkey_i2c_driver);

	if (ret) {
		printk(KERN_ERR "[TouchKey] melfas touch keypad registration failed, module not inserted.ret= %d\n", ret);
	}
#ifdef CONFIG_TARGET_LOCALE_NA
	store_module_version = get_touchkey_module_version();
#endif /* CONFIG_TARGET_LOCALE_NA */

#ifdef CONFIG_GENERIC_BLN
	wake_lock_init(&bln_wake_lock, WAKE_LOCK_SUSPEND, "bln_wake_lock");
	register_bln_implementation(&cypress_touchkey_bln);
#endif

	return ret;
}

static void __exit touchkey_exit(void)
{
	printk(KERN_DEBUG "[TouchKey] %s \n", __func__);
	i2c_del_driver(&touchkey_i2c_driver);
	misc_deregister(&touchkey_update_device);

#ifdef CONFIG_CM_BLN
	misc_deregister(&cm_led_device);
	wake_lock_destroy(&cm_led_wake_lock);
	del_timer(&cm_led_timer);
	del_timer(&cm_notification_timer);
#endif

#ifdef CONFIG_GENERIC_BLN
	wake_lock_destroy(&bln_wake_lock);
#endif

	if (touchkey_wq) {
		destroy_workqueue(touchkey_wq);
	}

	gpio_free(_3_TOUCH_SDA_28V);
	gpio_free(_3_TOUCH_SCL_28V);
	gpio_free(_3_GPIO_TOUCH_EN);
	gpio_free(_3_GPIO_TOUCH_INT);
}

late_initcall(touchkey_init);
module_exit(touchkey_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("@@@");
MODULE_DESCRIPTION("melfas touch keypad");
