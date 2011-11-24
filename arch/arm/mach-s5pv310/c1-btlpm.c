/* linux/arch/arm/mach-s5pv210/c1-btlpm.c
 * Copyright (C) 2010 Samsung Electronics. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#include <asm/mach-types.h>

#include <mach/gpio-c1.h>

#include "c1.h"

#include <linux/wakelock.h>
static struct wake_lock bt_playback_wake_lock;

static struct c1_bt_lpm {
	struct hrtimer bt_lpm_timer;
	ktime_t bt_lpm_delay;
} bt_lpm;

static enum hrtimer_restart bt_enter_lpm(struct hrtimer *timer)
{
	wake_unlock(&bt_playback_wake_lock);
	gpio_set_value(GPIO_BT_WAKE, 0);
    pr_debug("[BT] GPIO_BT_WAKE = %d\n", gpio_get_value(GPIO_BT_WAKE) );

	return HRTIMER_NORESTART;
}

void c1_bt_uart_wake_peer(struct uart_port *port)
{
	if (!bt_lpm.bt_lpm_timer.function)
		return;

	hrtimer_try_to_cancel(&bt_lpm.bt_lpm_timer);
	if (!gpio_get_value(GPIO_BT_WAKE)) {
		gpio_set_value(GPIO_BT_WAKE, 1);
		wake_lock(&bt_playback_wake_lock);
		pr_debug("[BT] GPIO_BT_WAKE = %d\n", gpio_get_value(GPIO_BT_WAKE) );
	}
	hrtimer_start(&bt_lpm.bt_lpm_timer, bt_lpm.bt_lpm_delay, HRTIMER_MODE_REL);
}

static ssize_t bt_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int ret, value = 0;
	sscanf(buf, "%d", &value);

	if (value != 0) {
		if (!machine_is_c1())
			return 0;

		ret = gpio_request(GPIO_BT_WAKE, "gpio_bt_wake");
		if (ret) {
			pr_err("Failed to request gpio_bt_wake control\n");
			return 0;
		}

		wake_lock_init(&bt_playback_wake_lock, WAKE_LOCK_SUSPEND, "bt_playback");

		gpio_direction_output(GPIO_BT_WAKE, GPIO_LEVEL_LOW);

		hrtimer_init(&bt_lpm.bt_lpm_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		bt_lpm.bt_lpm_delay = ktime_set(1, 0);	/* 1 sec */
		bt_lpm.bt_lpm_timer.function = bt_enter_lpm;
	}
	return size;
}

static DEVICE_ATTR(bt_mode, 0664, NULL, bt_mode_store);

static struct miscdevice bt_lpm_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "bt_lpm",
};

static int __init bt_lpm_init(void)
{
	int ret;

	ret = misc_register(&bt_lpm_device);
	if (ret) {
		pr_err("%s misc_register(%s) fail\n", __func__, bt_lpm_device.name);
		return 1;
	}

	if (sysfs_create_file(&bt_lpm_device.this_device->kobj, &dev_attr_bt_mode.attr) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_bt_mode.attr.name);

	return 0;
}
device_initcall(bt_lpm_init);
