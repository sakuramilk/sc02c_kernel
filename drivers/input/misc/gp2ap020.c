/* linux/driver/input/misc/gp2a.c
 * Copyright (C) 2010 Samsung Electronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/gp2a.h>
#include <linux/gp2ap020.h>
#include <plat/adc.h>


/* Note about power vs enable/disable:
 *  The chip has two functions, proximity and ambient light sensing.
 *  There is no separate power enablement to the two functions (unlike
 *  the Capella CM3602/3623).
 *  This module implements two drivers: /dev/proximity and /dev/light.
 *  When either driver is enabled (via sysfs attributes), we give power
 *  to the chip.  When both are disabled, we remove power from the chip.
 *  In suspend, we remove power if light is disabled but not if proximity is
 *  enabled (proximity is allowed to wakeup from suspend).
 *
 *  There are no ioctls for either driver interfaces.  Output is via
 *  input device framework and control via sysfs attributes.
 */


/*********** for debug **********************************************************/
#if 0
#define gprintk(fmt, x... ) printk( "%s(%d): " fmt, __FUNCTION__ ,__LINE__, ## x)
#else
#define gprintk(x...) do { } while (0)
#endif
/*******************************************************************************/


#define LIGHT_BUFFER_UP	5
#define LIGHT_BUFFER_DOWN	5

static const int adc_table[4] = {
	35, //15 lux match adc value
	290, //150 lux match adc value
	2900, //1500 lux match adc value
	27000, //15000 lux match adc value
};

/* global var */
static struct i2c_client *opt_i2c_client;
static state_type cur_state = LIGHT_INIT;	// LIGHT_INIT declared in gp2ap020.h
static int cur_adc_value = 0;

static int adc_value_buf[ADC_BUFFER_NUM] = {0};  // ADC_BUFFER_NUM declared in gp2ap020.h

int autobrightness_mode = OFF;
int LightSensor_Log_Cnt =0;
int proximity_enable=0;

u8 lightsensor_mode = 0; // 0 = low, 1 = high

enum {
	LIGHT_ENABLED = BIT(0),
	PROXIMITY_ENABLED = BIT(1),
};

#if defined (CONFIG_FB_S3C_MDNIE_TUNINGMODE_FOR_BACKLIGHT) || defined (CONFIG_MACH_C1_KDDI_REV00)
static int pre_val = -1;
extern int current_gamma_value;
extern u16 *pmDNIe_Gamma_set[];

typedef enum
{
	mDNIe_UI_MODE,
	mDNIe_VIDEO_MODE,
	mDNIe_VIDEO_WARM_MODE,
	mDNIe_VIDEO_COLD_MODE,
	mDNIe_CAMERA_MODE,
	mDNIe_NAVI
}Lcd_mDNIe_UI;


struct gp2a_data {
	struct input_dev *proximity_input_dev;
	struct input_dev *light_input_dev;
	struct gp2a_platform_data *pdata;
	struct i2c_client *i2c_client;
	struct class *lightsensor_class;
	struct class *proximity_class;
	struct device *switch_cmd_dev;
	struct device *proximity_dev;
	struct delayed_work work_light;  /* for lightsensor */
	struct hrtimer timer;
	ktime_t light_poll_delay;
	struct mutex data_mutex;
	struct mutex power_lock;
	struct wake_lock prx_wake_lock;
	struct workqueue_struct *wq;
	char	val_state;
	u8		power_state;
	state_type light_data;
	int light_buffer;
	int light_count;
	int light_level_state;
	bool light_first_level;
	int testmode;
	int		wakeup;		/* configure the button as a wake-up source */
	int   prox_delay;
	int   prox_data;
	int   irq;
	int   average[PROX_READ_NUM]; //for proximity adc average
  	struct kobject *uevent_kobj;
};

/* initial value for sensor register */
#define COL 12
static u8 gp2a_original_image[COL][2] =
{
  //{Regster, Value}
 	{0x01 , 0x63},   //PRST :01(4 cycle at Detection/Non-detection),ALSresolution :16bit, range *128   //0x1F -> 5F by sharp 
	{0x02 , 0x72},   //ALC : 0, INTTYPE : 1, PS mode resolution : 12bit, range*1
	{0x03 , 0x3C},   //LED drive current 110mA, Detection/Non-detection judgment output
	{0x04 , 0x00},
	{0x05 , 0x00},
	{0x06 , 0xFF},
	{0x07 , 0xFF},
	{0x08 , 0x07},  //PS mode LTH(Loff):  (??mm)
	{0x09 , 0x00},  //PS mode LTH(Loff) : 
	{0x0A , 0x08},  //PS mode HTH(Lon) : (??mm)
	{0x0B , 0x00},  // PS mode HTH(Lon) :
//  	{0x13 , 0x08}, // by sharp // for internal calculation (type:0)
	{0x00 , 0xC0}   //alternating mode (PS+ALS), TYPE=1(0:externel 1:auto calculated mode) //umfa.cal
};

int value_buf[4] = {0};

static int proximity_onoff(u8 onoff);

extern Lcd_mDNIe_UI current_mDNIe_UI;

extern void mDNIe_Mode_set_for_backlight(u16 *buf);

static struct gp2a_data *temp_gp2a;

int IsChangedADC(int val)
{
	int j = 0;
	int ret = 0;

	int adc_index = 0;
	static int adc_index_count = 0;

	adc_index = (adc_index_count)%4;
	adc_index_count++;

	if(pre_val == -1) { //ADC buffer initialize
		for(j = 0; j<4; j++)
			value_buf[j] = val;

		pre_val = 0;
	} else {
		value_buf[adc_index] = val;
	}

	ret = ((value_buf[0] == value_buf[1])&& \
		(value_buf[1] == value_buf[2])&& \
		(value_buf[2] == value_buf[3]))? 1 : 0;

	if(adc_index_count == 4)
		adc_index_count = 0;

	return ret;
}
#endif

static int StateToLux(state_type state)
{
	int lux = 0;

	if(state== LIGHT_LEVEL5) {
		lux = 15000;
	} else if(state == LIGHT_LEVEL4) {
		lux = 9000;
	} else if(state == LIGHT_LEVEL3) {
		lux = 5000;
	} else if(state == LIGHT_LEVEL2) {
		lux = 1000;
	} else if(state == LIGHT_LEVEL1) {
		lux = 6;
	} else {
		lux = 5000;
	}

	return lux;
}

int lightsensor_get_adc(void);
static int lightsensor_onoff(u8 onoff);

char proximity_sensor_detection = 0;

int opt_i2c_read(u8 reg, unsigned char *rbuf, int len )
{

	int ret=-1;
	struct i2c_msg msg;
	uint8_t start_reg;

	if(opt_i2c_client == NULL)
	{
		printk("opt_i2c_client [0x%x], adapter [0x%x]\n",
					(int)opt_i2c_client, (int)opt_i2c_client->adapter);
		printk("temp_gp2a = [0x%x]\n",(int)temp_gp2a);
	}

	msg.addr = opt_i2c_client->addr;
	msg.flags = 0;//I2C_M_WR;
	msg.len = 1;
	msg.buf = &start_reg;
	start_reg = reg;
	ret = i2c_transfer(opt_i2c_client->adapter, &msg, 1);

	if(ret>=0) {
		msg.flags = I2C_M_RD;
		msg.len = len;
		msg.buf = rbuf;
		ret = i2c_transfer(opt_i2c_client->adapter, &msg, 1 );
	}

	if( ret < 0 )
	{
    	gprintk("%s %d i2c transfer error ret=%d\n", __func__, __LINE__, ret);

	}

    return ret;
}

int opt_i2c_write( u8 reg, u8 *val )
{
    int err = 0;
    struct i2c_msg msg[1];
    unsigned char data[2];
    int retry = 10;

    if( (opt_i2c_client == NULL) || (!opt_i2c_client->adapter) ){
		printk("opt_i2c_client [0x%x], adapter [0x%x]\n",
					(int)opt_i2c_client, (int)opt_i2c_client->adapter);
		printk("temp_gp2a = [0x%x]\n",(int)temp_gp2a);
        return -ENODEV;
    }

    while(retry--)
    {
        data[0] = reg;
        data[1] = *val;

        msg->addr = opt_i2c_client->addr;
        msg->flags = I2C_M_WR;
        msg->len = 2;
        msg->buf = data;

        err = i2c_transfer(opt_i2c_client->adapter, msg, 1);

        if (err >= 0) return 0;
    }
    gprintk("%s %d i2c transfer error(%d)\n", __func__, __LINE__, err);
    return err;
}

static void gp2a_light_enable(struct gp2a_data *gp2a)
{
	gprintk("starting poll timer, delay %lldns\n",
		    ktime_to_ns(gp2a->light_poll_delay));
	gp2a->light_count = 0;
	gp2a->light_buffer = 0;
	gp2a->light_first_level =true;
	hrtimer_start(&gp2a->timer, gp2a->light_poll_delay, HRTIMER_MODE_REL);
}

static void gp2a_light_disable(struct gp2a_data *gp2a)
{
	gprintk("cancelling poll timer\n");
	hrtimer_cancel(&gp2a->timer);
	cancel_delayed_work_sync(&gp2a->work_light); //earlier it was cancel_work_sync
	/* mark the adc buff as not initialized
	 * so that it will be filled again on next light sensor start
	 */
}

static ssize_t poll_delay_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gp2a_data *gp2a = dev_get_drvdata(dev);
	return sprintf(buf, "%lld\n", ktime_to_ns(gp2a->light_poll_delay));
}


static ssize_t poll_delay_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct gp2a_data *gp2a = dev_get_drvdata(dev);
	int64_t new_delay;
	int err;

	err = strict_strtoll(buf, 10, &new_delay);
	if (err < 0)
		return err;

	gprintk("new delay = %lldns, old delay = %lldns\n",
		    new_delay, ktime_to_ns(gp2a->light_poll_delay));
	mutex_lock(&gp2a->power_lock);
	if (new_delay != ktime_to_ns(gp2a->light_poll_delay)) {
		gp2a->light_poll_delay = ns_to_ktime(new_delay);
		if (gp2a->power_state & LIGHT_ENABLED) {
			gp2a_light_disable(gp2a);
			gp2a_light_enable(gp2a);
		}
	}
	mutex_unlock(&gp2a->power_lock);

	return size;
}

static ssize_t light_enable_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct gp2a_data *data = input_get_drvdata(input_data);

	return sprintf(buf, "%d\n",
			(data->power_state & LIGHT_ENABLED) ? 1 : 0 );
}

static ssize_t light_enable_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct gp2a_data *data = input_get_drvdata(input_data);
	int value = simple_strtoul(buf, NULL, 10);

	if (value != 0 && value != 1) {
		return count;
	}

	mutex_lock(&data->power_lock);

	if ( !(data->power_state & LIGHT_ENABLED) && value) {
		if (!data->power_state)
			data->pdata->power(true);
		data->power_state |= LIGHT_ENABLED;
		lightsensor_onoff(1);
		gp2a_light_enable(data);
		gprintk("timer started.\n");
	}

	if ((data->power_state & LIGHT_ENABLED) && !value) {
		gp2a_light_disable(data);
		lightsensor_onoff(0);
		data->power_state &= ~LIGHT_ENABLED;
		if(!data->power_state)
			data->pdata->power(false);
		gprintk("timer canceled.\n");
	}

	input_report_abs(input_data, ABS_CONTROL_REPORT, (value<<16) | ktime_to_ns(data->light_poll_delay));

	mutex_unlock(&data->power_lock);

	return count;
}


static ssize_t
light_wake_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	static int cnt = 1;

	input_report_abs(input_data, ABS_WAKE, cnt++);

	return count;
}

static ssize_t
light_data_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct gp2a_data *data = input_get_drvdata(input_data);
	//unsigned long flags;
	int light_lux;

	light_lux = StateToLux(data->light_data);

	return sprintf(buf, "%d\n", light_lux);
}

static ssize_t
light_status_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
	//struct input_dev *input_data = to_input_dev(dev);
	//struct sensor_data *data = input_get_drvdata(input_data);

	printk(KERN_INFO "%s : cur_state(%d)\n", __func__, cur_state);

	return sprintf(buf, "%d\n", cur_state);
}

static ssize_t light_autobrightness_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct gp2a_data *data = input_get_drvdata(input_data);

	int adc = 0;
	int sum = 0;
	int i = 0;

	gprintk("called %s \n", __func__);

	if(data->power_state & LIGHT_ENABLED) {
#if defined(CONFIG_KOR_MODEL_SHV_E120L)
		if(get_hw_rev() >= 0x01) { // HW_REV00 does not work gp2a sensor 
			for(i = 0; i < 10; i++) {
				adc = lightsensor_get_adcvalue();
				msleep(20);
				sum += adc;
			}
			adc = sum/10;
		}
		else
			adc = 700;
#else
		for(i = 0; i < 10; i++) {
			adc = lightsensor_get_adcvalue();
			msleep(20);
			sum += adc;
		}
		adc = sum/10;
#endif

		gprintk("called %s  - subdued alarm(adc : %d)\n", __func__, adc);
		return sprintf(buf,"%d\n", adc);
	} else {
		gprintk("called %s  - *#0589#\n", __func__);
		return sprintf(buf,"%d\n", cur_adc_value);
	}
}

static ssize_t light_autobrightness_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	sscanf(buf, "%d", &value);

	gprintk("called %s \n", __func__);

	if(value == 1) {
		autobrightness_mode = ON;
		gprintk(KERN_DEBUG "[brightness_mode] BRIGHTNESS_MODE_SENSOR\n");
	} else if(value == 0) {
		autobrightness_mode = OFF;
		gprintk(KERN_DEBUG "[brightness_mode] BRIGHTNESS_MODE_USER\n");
#ifdef MDNIE_TUNINGMODE_FOR_BACKLIGHT
		if(pre_val == 1) {
			mDNIe_Mode_set_for_backlight(pmDNIe_Gamma_set[2]);
		}
		pre_val = -1;
#endif
	}

	return size;
}

static ssize_t light_testmode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct gp2a_data *data = input_get_drvdata(input_data);

	int testmode;

	testmode = data->testmode;

  	gprintk(" : %d \n", testmode);

	return sprintf(buf, "%d\n", testmode);
}

static ssize_t light_testmode_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct gp2a_data *data = input_get_drvdata(input_data);
	int value;

	sscanf(buf, "%d", &value);

	if(value == 1) {
		data->testmode = 1;
		gprintk("lightsensor testmode ON.\n");
	} else if(value == 0) {
		data->testmode  = 0;
		gprintk("lightsensor testmode OFF.\n");
	}

	return size;
}

static ssize_t lightsensor_file_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int adc = 0;

	adc = lightsensor_get_adcvalue();

	return sprintf(buf, "%d\n", adc);
}

/* This function is for light sensor.  It operates every a few seconds.
 * It asks for work to be done on a thread because i2c needs a thread
 * context (slow and blocking) and then reschedules the timer to run again.
 */
static enum hrtimer_restart gp2a_timer_func(struct hrtimer *timer)
{
	struct gp2a_data *gp2a = container_of(timer, struct gp2a_data, timer);
	queue_delayed_work(gp2a->wq, &gp2a->work_light,
			msecs_to_jiffies(ktime_to_ms(gp2a->light_poll_delay)));
	//earlier it was queue_work.
	hrtimer_forward_now(&gp2a->timer, gp2a->light_poll_delay);
	return HRTIMER_RESTART;
}


static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP, poll_delay_show, poll_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP, light_enable_show, light_enable_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP, NULL, light_wake_store);
static DEVICE_ATTR(data, S_IRUGO, light_data_show, NULL);
static DEVICE_ATTR(status, S_IRUGO, light_status_show, NULL);
static DEVICE_ATTR(autobrightness, S_IRUGO|S_IWUSR|S_IWGRP, light_autobrightness_show, light_autobrightness_store);
static DEVICE_ATTR(testmode, S_IRUGO|S_IWUSR|S_IWGRP, light_testmode_show, light_testmode_store);
static DEVICE_ATTR(lightsensor_file_state, 0644, lightsensor_file_state_show, NULL);

static struct attribute *lightsensor_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_wake.attr,
	&dev_attr_data.attr,
	&dev_attr_status.attr,
	&dev_attr_autobrightness.attr,
	&dev_attr_testmode.attr,
	&dev_attr_lightsensor_file_state.attr,
	&dev_attr_poll_delay.attr,
	NULL
};

static struct attribute_group lightsensor_attribute_group = {
	.attrs = lightsensor_attributes
};


static const struct file_operations light_fops = {
	.owner  = THIS_MODULE,
};

static struct miscdevice light_device = {
    .minor  = MISC_DYNAMIC_MINOR,
    .name   = "light",
    .fops   = &light_fops,
};


static void gp2a_work_func_light(struct work_struct *work)
{
	struct gp2a_data *gp2a = container_of((struct delayed_work *)work,
							struct gp2a_data, work_light);

	int i;
	int adc = 0;

#if defined(CONFIG_KOR_MODEL_SHV_E120L)
	if(get_hw_rev() >= 0x01) // HW_REV00 does not work gp2a sensor 
		adc = lightsensor_get_adcvalue();
	else
		adc = 700;		
#else
	temp_gp2a = gp2a;
	adc = lightsensor_get_adcvalue();
#endif

	for (i = 0; ARRAY_SIZE(adc_table); i++)
		if (adc <= adc_table[i])
			break;

	if (gp2a->light_buffer == i) {
		if(gp2a->light_level_state <= i||gp2a->light_first_level == true){
			if (gp2a->light_count++ == LIGHT_BUFFER_UP) {
				if (LightSensor_Log_Cnt == 10) {
                    printk("[LIGHT SENSOR] lux up 0x%0X (%d)\n", adc, adc);
                    LightSensor_Log_Cnt = 0; 
                }
				LightSensor_Log_Cnt++;
				input_report_abs(gp2a->light_input_dev, ABS_MISC, adc);
				input_sync(gp2a->light_input_dev);
				gp2a->light_count = 0;
				gp2a->light_first_level = false;
				gp2a->light_level_state = gp2a->light_buffer;
			}
		}
		else {
			if (gp2a->light_count++ == LIGHT_BUFFER_DOWN) {
				if (LightSensor_Log_Cnt == 10) {
					printk("[LIGHT SENSOR]lux down 0x%0X(%d)\n", adc, adc);
					LightSensor_Log_Cnt = 0;
				}

                LightSensor_Log_Cnt++;
				input_report_abs(gp2a->light_input_dev, ABS_MISC, adc);
				input_sync(gp2a->light_input_dev);
				gp2a->light_count = 0;
				gp2a->light_level_state = gp2a->light_buffer;
			}
		}
	} 
	else {
		gp2a->light_buffer = i;
		gp2a->light_count = 0;
    }

	queue_delayed_work(gp2a->wq, &gp2a->work_light,
		msecs_to_jiffies(ktime_to_ms(gp2a->light_poll_delay)));
}

int lightsensor_get_adc(void)
{
	unsigned char get_data[4]={0,};//test
    int D0_raw_data;
	int D1_raw_data;
    int D0_data;
	int D1_data;
    int lx =0;
	u8 value;
	int light_alpha;
	int light_beta;
	static int lx_prev=0;

	value = 0x8C;
	opt_i2c_write(COMMAND1,&value);

	opt_i2c_read(0x0C, get_data, sizeof(get_data));
    D0_raw_data =(get_data[1] << 8) | get_data[0]; // clear
	D1_raw_data =(get_data[3] << 8) | get_data[2]; // IR

#if defined(CONFIG_KOR_MODEL_SHV_E120K)
	if(lightsensor_mode) { // HIGH_MODE
		if(100 * D1_raw_data <= 67 * D0_raw_data) {
			light_alpha = 2015;
			light_beta = 2925;
		} else if (100 * D1_raw_data <= 90 * D0_raw_data) {
			light_alpha = 56;
			light_beta = 12;
		} else {
			light_alpha = 0;
			light_beta = 0;
		}
	} else { // LOW_MODE
		if(100 * D1_raw_data <= 63 * D0_raw_data) {
			light_alpha = 2015;
			light_beta = 2925;
		} else if (100 * D1_raw_data <= 90 * D0_raw_data) {
			light_alpha = 547;
			light_beta = 599;
		} else {
			light_alpha = 0;
			light_beta = 0;
		}
	}
#elif defined(CONFIG_KOR_MODEL_SHV_E120S)
	if(get_hw_rev() >= 0x09) {
		if(lightsensor_mode) { // HIGH_MODE
			if(100 * D1_raw_data <= 67 * D0_raw_data) {
				light_alpha = 2015;
				light_beta = 2925;
			} else if (100 * D1_raw_data <= 90 * D0_raw_data) {
				light_alpha = 56;
				light_beta = 12;
			} else {
				light_alpha = 0;
				light_beta = 0;
			}
		} else { // LOW_MODE
			if(100 * D1_raw_data <= 63 * D0_raw_data) {
				light_alpha = 2015;
				light_beta = 2925;
			} else if (100 * D1_raw_data <= 90 * D0_raw_data) {
				light_alpha = 547;
				light_beta = 599;
			} else {
				light_alpha = 0;
				light_beta = 0;
			}
		}
	} else { // (get_hw_rev() >= 0x09)
		if(lightsensor_mode) { // HIGH_MODE
			if(100 * D1_raw_data <= 67 * D0_raw_data) {
				light_alpha = 2015;
				light_beta = 2925;
			} else if (100 * D1_raw_data <= 90 * D0_raw_data) {
				light_alpha = 56;
				light_beta = 12;
			} else {
				light_alpha = 0;
				light_beta = 0;
			}
		} else { // LOW_MODE
			if(100 * D1_raw_data <= 60 * D0_raw_data) {
				light_alpha = 2015;
				light_beta = 2925;
			} else if (100 * D1_raw_data <= 90 * D0_raw_data) {
				light_alpha = 800;
				light_beta = 876;
			} else {
				light_alpha = 0;
				light_beta = 0;
			}
		}
	}// (get_hw_rev() >= 0x09)
#else // defined(CONFIG_KOR_MODEL_SHV_E120L)
	if(lightsensor_mode) { // HIGH_MODE
		if(100 * D1_raw_data <= 67 * D0_raw_data) {
			light_alpha = 2015;
			light_beta = 2925;
		} else if (100 * D1_raw_data <= 90 * D0_raw_data) {
			light_alpha = 56;
			light_beta = 12;
		} else {
			light_alpha = 0;
			light_beta = 0;
		}
	} else { // LOW_MODE
		if(100 * D1_raw_data <= 60 * D0_raw_data) {
			light_alpha = 2015;
			light_beta = 2925;
		} else if (100 * D1_raw_data <= 90 * D0_raw_data) {
			light_alpha = 800;
			light_beta = 876;
		} else {
			light_alpha = 0;
			light_beta = 0;
		}
	}
#endif

	if(lightsensor_mode) { // HIGH_MODE
		D0_data = D0_raw_data * 16;
		D1_data = D1_raw_data * 16;
	} else { // LOW_MODE
		D0_data = D0_raw_data;
		D1_data = D1_raw_data;
//		lx = (int)(D0_data - ((869*D1_data)/1000))*33/10;
	}
//	lx = (int)((((D0_data + D1_data)/2) - ((15*D1_data)/100))*495/100)*4;

	if(D0_data == 0 || D1_data == 0)
		lx = 0;
	else if((100 * D1_data > 90 * D0_data) || (100 * D1_data < 15 * D0_data))
		lx = lx_prev;
	else
		lx = (int)((light_alpha * D0_data) - (light_beta * D1_data))*33/10000;

	lx_prev = lx;

//	gprintk("%s: D0=%d, D1=%d, lx=%d mode=%d\n",__func__, D0_raw_data, D1_raw_data, lx, lightsensor_mode);

	if(lightsensor_mode) { // HIGH MODE
		if(D0_raw_data < 1000) {
			gprintk("%s: change to LOW_MODE detection=%d\n", __func__, proximity_sensor_detection);
			lightsensor_mode = 0; // change to LOW MODE
			if(proximity_enable) {
			if(proximity_sensor_detection)
				value = 0x23;
			else
				value = 0x63;
			opt_i2c_write(COMMAND2,&value);
//				value = 0x78;
//				opt_i2c_write(COMMAND3,&value);
			}
		}
	} else { // LOW MODE
		if(D0_raw_data > 16000 || D1_raw_data > 16000) {
			gprintk("%s: change to HIGH_MODE detection=%d\n", __func__, proximity_sensor_detection);
			lightsensor_mode = 1; // change to HIGH MODE
			if(proximity_enable) {
			if(proximity_sensor_detection)
				value = 0x27;
			else
				value = 0x67;
			opt_i2c_write(COMMAND2,&value);
//				value = 0x75;
//				opt_i2c_write(COMMAND3,&value);
		}
	}
	}

	if(proximity_enable)
		value = 0xCC;
	else
		value = 0xDC;
	opt_i2c_write(COMMAND1,&value);

	return lx;

	/*
      TYPE = 0
      illuminance[lx] = {(Alpha*D0 -Beta*D1)/2^resolution}*1024*range/4.9

      TYPE = 1
       illuminance[lx] = {0.5*D0/2^resolution}*1024*range/4.9
       resolution = 16 ,  range = 128 = 2^7,  1024 = 2^10
	*/
	//lx = (int)(D0_data*10/49);
//    return D0_data;
}

int lightsensor_get_adcvalue(void)
{
	int i = 0;
	int j = 0;
	unsigned int adc_total = 0;
	static int adc_avr_value = 0;
	unsigned int adc_index = 0;
	static unsigned int adc_index_count = 0;
	unsigned int adc_max = 0;
	unsigned int adc_min = 0;
	int value =0;

	value = lightsensor_get_adc();


	cur_adc_value = value;

	adc_index = (adc_index_count++)%ADC_BUFFER_NUM;		

	if(cur_state == LIGHT_INIT) { //ADC buffer initialize (light sensor off ---> light sensor on)
		for(j = 0; j < ADC_BUFFER_NUM; j++)
			adc_value_buf[j] = value;
	} else {
		adc_value_buf[adc_index] = value;
	}

	adc_max = adc_value_buf[0];
	adc_min = adc_value_buf[0];

	for(i = 0; i < ADC_BUFFER_NUM; i++) {
		adc_total += adc_value_buf[i];

		if(adc_max < adc_value_buf[i])
			adc_max = adc_value_buf[i];
					
		if(adc_min > adc_value_buf[i])
			adc_min = adc_value_buf[i];
	}
	adc_avr_value = (adc_total-(adc_max+adc_min))/(ADC_BUFFER_NUM-2);
	
	if(adc_index_count == ADC_BUFFER_NUM-1)
		adc_index_count = 0;

	return adc_avr_value;
}

static int lightsensor_onoff(u8 onoff)
{
	u8 value;

	printk("%s : light_sensor onoff = %d\n", __func__, onoff);

	if(onoff) {
		if(proximity_enable == 0){
			//in calling, must turn on proximity sensor
			value = 0x01;
			opt_i2c_write(COMMAND4,&value);

			value = 0x63;
	       	opt_i2c_write(COMMAND2,&value);

			value = 0xD0;
			//OP3 : 1(operating mode) OP2 :1(coutinuous operating mode) 
			//OP1 : 01(ALS mode) TYPE=0(auto)
	       	opt_i2c_write(COMMAND1,&value);
			// other setting have defualt value. 
    	}
	}
	else {
		if(proximity_enable == 0){
			//in calling, must turn on proximity sensor
			value = 0x00; //shutdown mode
			opt_i2c_write((u8)(COMMAND1),&value);
		}
	}
	
	return 0;
}


static int proximity_onoff(u8 onoff)
{
	u8 value;
    int i;
	unsigned char get_data[1];//test
	int err = 0;

	printk("%s : proximity turn on/off = %d\n", __func__, onoff);
       
	if(onoff) {
		// already on light sensor, so must simultaneously  
		// turn on light sensor and proximity sensor 

	    opt_i2c_read(COMMAND1, get_data, sizeof(get_data));
//	    if(get_data == 0xC1) return 0;
       	for(i=0;i<COL;i++) {
       		err = opt_i2c_write(gp2a_original_image[i][0],
							&gp2a_original_image[i][1]);
			if(err < 0)
				gprintk("%s : turnning on error i = %d, err=%d\n", 
								__func__, i, err);
			lightsensor_mode = 0;
		}
	}
	else
	{// light sensor turn on and proximity turn off
	
	    opt_i2c_read(COMMAND1, get_data, sizeof(get_data));
//	    if(get_data == 0xD1) return 0;
	
		if(lightsensor_mode)
			value = 0x67;//resolution :16bit, range: *128
		else
			value = 0x63;//resolution :16bit, range: *128
       	opt_i2c_write(COMMAND2,&value);
		value = 0xD0;//OP3 : 1(operating mode) OP2 :1(coutinuous operating mode) OP1 : 01(ALS mode)
       	opt_i2c_write(COMMAND1,&value);
	}
	
	return 0;
}


/* Proximity Sysfs interface */
static ssize_t
proximity_delay_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct gp2a_data *data = input_get_drvdata(input_data);
    int delay;

    delay = data->prox_delay;

    return sprintf(buf, "%d\n", delay);
}

static ssize_t
proximity_delay_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct gp2a_data *data = input_get_drvdata(input_data);
    int delay = simple_strtoul(buf, NULL, 10);

    if (delay < 0) {
        return count;
    }

    if (SENSOR_MAX_DELAY < delay) {
        delay = SENSOR_MAX_DELAY;
    }

	mutex_lock(&data->power_lock);

    data->prox_delay = delay;

    input_report_abs(input_data, ABS_CONTROL_REPORT,
			((data->power_state & PROXIMITY_ENABLED)<<16) | delay);

	mutex_unlock(&data->power_lock);

    return count;
}

static void proximity_power_control(struct gp2a_data *data, int value)
{ 
	gprintk("name [ %s ] onoff [%d]\n",data->proximity_input_dev->name,value);

	if (!(data->power_state & PROXIMITY_ENABLED) && value) {			
		/* proximity power on */

		if(!(data->power_state & PROXIMITY_ENABLED))
			data->pdata->power(true);
		data->power_state |= PROXIMITY_ENABLED;
		msleep(1);
		proximity_onoff(1);
		enable_irq_wake(data->irq);
		msleep(200);

		enable_irq(data->irq);
	}

	if ((data->power_state & PROXIMITY_ENABLED) && !value) {
		/* Proximity power off */
		disable_irq(data->irq);

		proximity_onoff(0);
		disable_irq_wake(data->irq);
		data->power_state &= ~PROXIMITY_ENABLED;
		msleep(50);

		if(!data->power_state)
			data->pdata->power(false);
	}

	proximity_enable = value;

	gprintk(" enable : [ %d ],power_state [ %d] \n", 
					proximity_enable, data->power_state);

	return;
}

static ssize_t proximity_enable_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct gp2a_data *pdata = input_get_drvdata(input_data);
	int onoff = simple_strtoul(buf, NULL, 10);

	gprintk("name [ %s ] onoff [%d]\n",
				pdata->proximity_input_dev->name,onoff);

	if (onoff != 0 && onoff != 1) {
		return count;
	}

	mutex_lock(&pdata->power_lock);

	proximity_power_control(pdata,onoff);

	input_report_abs(input_data, ABS_CONTROL_REPORT, 
					(onoff<<16) | pdata->prox_delay);

	mutex_unlock(&pdata->power_lock);

	return count;
}

static ssize_t proximity_enable_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct gp2a_data *data = input_get_drvdata(input_data);
	int enabled;

	enabled = data->power_state & PROXIMITY_ENABLED;

	return sprintf(buf, "%d\n", enabled);
}


static ssize_t
proximity_wake_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    static int cnt = 1;

    input_report_abs(input_data, ABS_WAKE, cnt++);

    return count;
}

static ssize_t
proximity_data_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
	struct gp2a_data *data = input_get_drvdata(input_data);
    int x;

	mutex_lock(&data->data_mutex);
	x = data->prox_data;
	mutex_unlock(&data->data_mutex);
	
    return sprintf(buf, "%d\n", x);
}

static ssize_t proximity_avg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
 struct gp2a_data *data = (struct gp2a_data*)(dev_get_drvdata(dev));

 int min = 0, max = 0, avg = 0;
 int i;
 int proximity_value = 0;

     for (i = 0; i < PROX_READ_NUM; i++) {
 	   proximity_value = data->average[i]; 
       if(proximity_value > 0){

		 avg += proximity_value;
 
		 if (!i)
			 min = proximity_value;
		 else if (proximity_value < min)
			 min = proximity_value;
 
		 if (proximity_value > max)
			 max = proximity_value;
      	}
	 }
	 avg /= i;

	return sprintf(buf, "%d, %d, %d\n",min,avg,max);
}

static ssize_t proximity_avg_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct gp2a_data *pdata = (struct gp2a_data*)(dev_get_drvdata(dev));
	int onoff = simple_strtoul(buf, NULL, 10);

	printk("name [ %s ] onoff [%d]\n",
			pdata->proximity_input_dev->name,onoff);

	if (onoff != 0 && onoff != 1) {
		goto out;
	}

	mutex_lock(&pdata->power_lock);

	proximity_power_control(pdata, onoff);

	mutex_unlock(&pdata->power_lock);

out:
	return size;
}

static int count = 0; //count for proximity average
static ssize_t proximity_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gp2a_data *data = (struct gp2a_data*)(dev_get_drvdata(dev));

	int D2_data;
    unsigned char get_D2_data[2]={0,};//test

	msleep(20);
	opt_i2c_read(0x10, get_D2_data, sizeof(get_D2_data));
    D2_data =(get_D2_data[1] << 8) | get_D2_data[0];
	
    data->average[count]=D2_data;
	count++;
	if(count == PROX_READ_NUM) count=0;

	return sprintf(buf, "%d\n", D2_data);	
}


static DEVICE_ATTR(pdelay, S_IRUGO|S_IWUSR|S_IWGRP,
        proximity_delay_show, proximity_delay_store);
static struct device_attribute dev_attr_proximity_enable = 
		__ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
				proximity_enable_show, proximity_enable_store);
static DEVICE_ATTR(pwake, S_IWUSR|S_IWGRP,
        NULL, proximity_wake_store);
static DEVICE_ATTR(pdata, S_IRUGO, proximity_data_show, NULL);
static DEVICE_ATTR(proximity_avg, 0644,
		   proximity_avg_show, proximity_avg_store);
static DEVICE_ATTR(proximity_state, 0644, proximity_state_show, NULL);

static struct attribute *proximity_attributes[] = {
	&dev_attr_pdelay.attr,
	&dev_attr_proximity_enable.attr,
	&dev_attr_pwake.attr,
	&dev_attr_pdata.attr,
	&dev_attr_proximity_state.attr,
	&dev_attr_proximity_avg.attr,
	NULL
};

static struct attribute_group proximity_attribute_group = {
    .attrs = proximity_attributes
};


/* interrupt happened due to transition/change of near/far proximity state */
irqreturn_t gp2a_irq_handler(int irq, void *data)
{
	struct gp2a_data *ip = data;
	u8 setting;
	int val = gpio_get_value(ip->pdata->p_out);
	int ret = 0;

	proximity_sensor_detection = !val;	// 0 : near , 1 : far

	/* 0 is close, 1 is far */
	input_report_abs(ip->proximity_input_dev, ABS_DISTANCE, val);
	input_sync(ip->proximity_input_dev);
	wake_lock_timeout(&ip->prx_wake_lock, 3*HZ);

	setting = 0x0C;
	ret = opt_i2c_write(COMMAND1, &setting);

	if(val == 0){
		if(lightsensor_mode == 0)	// low mode
			setting = 0x23;
		else						// high mode
			setting = 0x27;

		ret = opt_i2c_write(COMMAND2, &setting);
	}
	else {
		if(lightsensor_mode == 0)	// low mode
			setting = 0x63;
		else
			setting = 0x67;			// high mode
		ret = opt_i2c_write(COMMAND2,&setting);
	}

	setting = 0xCC;
	ret = opt_i2c_write(COMMAND1, &setting);

	ip->prox_data = val;
	printk(" proximity = %d, lightsensor_mode = %d \n", 
			proximity_sensor_detection, lightsensor_mode);

	return IRQ_HANDLED;
}

static int gp2a_setup_irq(struct gp2a_data *gp2a)
{
	int rc = -EIO;
	struct gp2a_platform_data *pdata = gp2a->pdata;
	int irq;

	gprintk("start\n");

	rc = gpio_request(pdata->p_out, "gpio_proximity_out");
	if (rc < 0) {
		pr_err("%s: gpio %d request failed (%d)\n",
			__func__, pdata->p_out, rc);
		return rc;
	}

	rc = gpio_direction_input(pdata->p_out);
	if (rc < 0) {
		pr_err("%s: failed to set gpio %d as input (%d)\n",
			__func__, pdata->p_out, rc);
		goto err_gpio_direction_input;
	}

	irq = gpio_to_irq(pdata->p_out);
	rc = request_threaded_irq(irq, NULL,
			 gp2a_irq_handler,
			 IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			 "proximity_int",
			 gp2a);
	if (rc < 0) {
		pr_err("%s: request_irq(%d) failed for gpio %d (%d)\n",
			__func__, irq,
			pdata->p_out, rc);
		goto err_request_irq;
	}

	/* start with interrupts disabled */
	disable_irq(irq);
	gp2a->irq = irq;

	gprintk("success\n");

	goto done;

err_request_irq:
err_gpio_direction_input:
	gpio_free(pdata->p_out);
done:
	return rc;
}

static int gp2a_i2c_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int ret = -ENODEV;
	struct input_dev *input_dev;
	struct gp2a_data *gp2a;
	struct gp2a_platform_data *pdata = client->dev.platform_data;

	if (!pdata) {
		pr_err("%s: missing pdata!\n", __func__);
		return ret;
	}

	if (!pdata->power/*|| !pdata->light_adc_value*/) {
		pr_err("%s: incomplete pdata!\n", __func__);
		return ret;
	}

#if defined(CONFIG_MACH_C1_NA_SPR_EPIC2_REV00)
	/* power on gp2a */
	pdata->power(true);
#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: i2c functionality check failed!\n", __func__);
		return ret;
	}

	gp2a = kzalloc(sizeof(struct gp2a_data), GFP_KERNEL);
	if (!gp2a) {
		pr_err("%s: failed to alloc memory for module data\n",
		       __func__);
		return -ENOMEM;
	}

	gp2a->pdata = pdata;
	gp2a->i2c_client = client;
	opt_i2c_client = gp2a->i2c_client;
	i2c_set_clientdata(client, gp2a);

	/* wake lock init */
	wake_lock_init(&gp2a->prx_wake_lock, WAKE_LOCK_SUSPEND,
		       "prx_wake_lock");
	mutex_init(&gp2a->power_lock);

	ret = gp2a_setup_irq(gp2a);
	if (ret) {
		pr_err("%s: could not setup irq\n", __func__);
		goto err_setup_irq;
	}

	mutex_init(&gp2a->data_mutex);
	
	/* allocate proximity input_device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("%s: could not allocate input device\n", __func__);
		goto err_input_allocate_device_proximity;
	}
	input_dev->name = "proximity_sensor";
	input_set_capability(input_dev, EV_ABS, ABS_DISTANCE);
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	input_set_capability(input_dev, EV_ABS, ABS_STATUS); /* status */
	input_set_capability(input_dev, EV_ABS, ABS_WAKE); /* wake */
	input_set_capability(input_dev, EV_ABS, ABS_CONTROL_REPORT); /* enabled/delay */

	gprintk("registering proximity input device\n");
	ret = input_register_device(input_dev);
	if (ret < 0) {
		pr_err("%s: could not register input device\n", __func__);
		input_free_device(input_dev);
		goto err_input_register_device_proximity;
	}
	
	gp2a->proximity_input_dev = input_dev;
	input_set_drvdata(input_dev, gp2a);
	ret = sysfs_create_group(&input_dev->dev.kobj,
				 &proximity_attribute_group);
	if (ret) {
		pr_err("%s: could not create sysfs group\n", __func__);
		goto err_sysfs_create_group_proximity;
	}

	/* hrtimer settings.  we poll for light values using a timer. */
	hrtimer_init(&gp2a->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	gp2a->light_poll_delay = ns_to_ktime(200 * NSEC_PER_MSEC);
	gp2a->timer.function = gp2a_timer_func;

	/* the timer just fires off a work queue request.  we need a thread
	 * to read the i2c (can be slow and blocking)
	 */
	gp2a->wq = create_singlethread_workqueue("gp2a_wq");
	if (!gp2a->wq) {
		ret = -ENOMEM;
		pr_err("%s: could not create workqueue\n", __func__);
		goto err_create_workqueue;
	}

	gp2a->power_state = 0;
	gp2a->prox_delay = SENSOR_DEFAULT_DELAY;
	gp2a->testmode = 0;
	gp2a->light_level_state =0;

	/* this is the thread function we run on the work queue */
	INIT_DELAYED_WORK(&gp2a->work_light, gp2a_work_func_light);

	/* allocate lightsensor-level input_device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("%s: could not allocate input device\n", __func__);
		ret = -ENOMEM;
		goto err_input_allocate_device_light;
	}
	
	input_dev->name = "light_sensor";
	input_set_capability(input_dev, EV_ABS, ABS_MISC);
	input_set_abs_params(input_dev, ABS_MISC, 0, 1, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_WAKE); /* wake */
	input_set_capability(input_dev, EV_ABS, ABS_CONTROL_REPORT); /* enabled/delay */

	gprintk("registering lightsensor-level input device\n");
	ret = input_register_device(input_dev);
	if (ret < 0) {
		pr_err("%s: could not register input device\n", __func__);
		input_free_device(input_dev);
		goto err_input_register_device_light;
	}
	gp2a->light_input_dev = input_dev;
	input_set_drvdata(input_dev, gp2a);
	
	ret = sysfs_create_group(&input_dev->dev.kobj,
				 &lightsensor_attribute_group);
	if (ret) {
		pr_err("%s: could not create sysfs group\n", __func__);
		goto err_sysfs_create_group_light;
	}

	/* set sysfs for light sensor */
	ret = misc_register(&light_device);
	if (ret)
		pr_err(KERN_ERR "misc_register failed - light\n");

	gp2a->lightsensor_class = class_create(THIS_MODULE, "lightsensor");
	if (IS_ERR(gp2a->lightsensor_class))
		pr_err("Failed to create class(lightsensor)!\n");

	gp2a->switch_cmd_dev = device_create(gp2a->lightsensor_class,
		NULL, 0, NULL, "switch_cmd");
	if (IS_ERR(gp2a->switch_cmd_dev))
		pr_err("Failed to create device(switch_cmd_dev)!\n");

	if (device_create_file(gp2a->switch_cmd_dev,
		&dev_attr_lightsensor_file_state) < 0)
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_lightsensor_file_state.attr.name);

	dev_set_drvdata(gp2a->switch_cmd_dev, gp2a);

	/* set initial proximity value as 1 */
	input_report_abs(gp2a->proximity_input_dev, ABS_DISTANCE, 1);
	input_sync(gp2a->proximity_input_dev);

	gp2a->proximity_class = class_create(THIS_MODULE, "proximity");
	if (IS_ERR(gp2a->proximity_class))
		pr_err("Failed to create class(proximity)!\n");

	gp2a->proximity_dev = device_create(gp2a->proximity_class,
			NULL,0,NULL, "proximity");
	if (IS_ERR(gp2a->proximity_dev))
		pr_err("Failed to create device file(proximity_dev)!\n");

	if (device_create_file(gp2a->proximity_dev,
			&dev_attr_proximity_state) < 0)
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_proximity_state.attr.name);

	if (device_create_file(gp2a->proximity_dev,
			&dev_attr_proximity_avg) < 0)
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_proximity_avg.attr.name);

	dev_set_drvdata(gp2a->proximity_dev,gp2a);

	goto done;

	/* error, unwind it all */
err_sysfs_create_group_light:
	input_unregister_device(gp2a->light_input_dev);
err_input_register_device_light:
err_input_allocate_device_light:
	destroy_workqueue(gp2a->wq);
err_create_workqueue:
	sysfs_remove_group(&gp2a->proximity_input_dev->dev.kobj,
			   &proximity_attribute_group);
err_sysfs_create_group_proximity:
	input_unregister_device(gp2a->proximity_input_dev);
err_input_register_device_proximity:
err_input_allocate_device_proximity:
	free_irq(gp2a->irq, gp2a);
	gpio_free(gp2a->pdata->p_out);
err_setup_irq:
	mutex_destroy(&gp2a->power_lock);
	wake_lock_destroy(&gp2a->prx_wake_lock);
	kfree(gp2a);
done:
	return ret;
}

static int gp2a_suspend(struct device *dev)
{
	/* We disable power only if proximity is disabled.  If proximity
	 * is enabled, we leave power on because proximity is allowed
	 * to wake up device.  We remove power without changing
	 * gp2a->power_state because we use that state in resume
	*/
	struct i2c_client *client = to_i2c_client(dev);
	struct gp2a_data *gp2a = i2c_get_clientdata(client);
	gprintk(" suspend power state %d \n", gp2a->power_state);
	mutex_lock(&gp2a->power_lock);
	if (gp2a->power_state & LIGHT_ENABLED)
		gp2a_light_disable(gp2a);
#if defined(CONFIG_MACH_Q1_REV02) || defined(CONFIG_MACH_C1_KDDI_REV00)
	if (gp2a->power_state == LIGHT_ENABLED)
		gp2a->pdata->power(false);
#endif
	mutex_unlock(&gp2a->power_lock);

	return 0;
}

static int gp2a_resume(struct device *dev)
{
	/* Turn power back on if we were before suspend. */
	struct i2c_client *client = to_i2c_client(dev);
	struct gp2a_data *gp2a = i2c_get_clientdata(client);
	gprintk(" resume power state %d \n", gp2a->power_state);

	mutex_lock(&gp2a->power_lock);
#if defined(CONFIG_MACH_Q1_REV02) || defined(CONFIG_MACH_C1_KDDI_REV00)
	if (gp2a->power_state == LIGHT_ENABLED)
		gp2a->pdata->power(true);
#endif
	if (gp2a->power_state & LIGHT_ENABLED)
		gp2a_light_enable(gp2a);
	mutex_unlock(&gp2a->power_lock);

	return 0;
}

static int gp2a_i2c_remove(struct i2c_client *client)
{
	struct gp2a_data *gp2a = i2c_get_clientdata(client);
	sysfs_remove_group(&gp2a->light_input_dev->dev.kobj,
			   &lightsensor_attribute_group);
	input_unregister_device(gp2a->light_input_dev);
	sysfs_remove_group(&gp2a->proximity_input_dev->dev.kobj,
			   &proximity_attribute_group);
	input_unregister_device(gp2a->proximity_input_dev);

	free_irq(gp2a->irq, gp2a);
	gpio_free(gp2a->pdata->p_out);
	if (gp2a->power_state) {
		if (gp2a->power_state & LIGHT_ENABLED)
			gp2a_light_disable(gp2a);
#if defined(CONFIG_MACH_Q1_REV02) || defined(CONFIG_MACH_C1_KDDI_REV00)
		gp2a->pdata->power(false);
#endif
	}
#if defined(CONFIG_MACH_C1_NA_SPR_EPIC2_REV00)
	gp2a->pdata->power(false);
#endif
	destroy_workqueue(gp2a->wq);
	mutex_destroy(&gp2a->power_lock);
	wake_lock_destroy(&gp2a->prx_wake_lock);
	kfree(gp2a);
	return 0;
}

static const struct i2c_device_id gp2a_device_id[] = {
	{"gp2a", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, gp2a_device_id);

static const struct dev_pm_ops gp2a_pm_ops = {
	.suspend = gp2a_suspend,
	.resume = gp2a_resume
};

static struct i2c_driver gp2a_i2c_driver = {
	.driver = {
		.name = "gp2a",
		.owner = THIS_MODULE,
		.pm = &gp2a_pm_ops
	},
	.probe		= gp2a_i2c_probe,
	.remove		= gp2a_i2c_remove,
	.id_table	= gp2a_device_id,
};

static int __init gp2a_init(void)
{
	#if defined(CONFIG_MACH_C1_KDDI_REV00)
	extern int sec_isLpmMode(void);

	if (sec_isLpmMode())
	{
		printk(KERN_ERR "LPM MODE (gp2a_init)!\n");
		return 0;
	}
	#endif

	return i2c_add_driver(&gp2a_i2c_driver);
}

static void __exit gp2a_exit(void)
{
	i2c_del_driver(&gp2a_i2c_driver);
}

module_init(gp2a_init);
module_exit(gp2a_exit);

MODULE_AUTHOR("SAMSUNG");
MODULE_DESCRIPTION("Optical Sensor driver for GP2AP020A00F");
MODULE_LICENSE("GPL");
