/*
 * LED Triggers Core for Asus a620
 *
 *  (c) 2014 olegvedi@gmail.com
 *
 * based on:
 *
 * For the HP Jornada 620/660/680/690 handhelds
 *
 * Copyright 2008 Kristoffer Ericson <kristoffer.ericson@gmail.com>
 *     this driver is based on leds-spitz.c by Richard Purdie.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <mach/asus620.h>
//#include <linux/platform_data/mmc-pxamci.h>
#include <linux/platform_data/video-pxafb.h>
#include <linux/platform_data/irda-pxaficp.h>
#include <mach/udc.h>

#include <linux/spi/ads7846.h>
#include <linux/spi/spi.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/i2c/pxa-i2c.h>

#include "../generic.h"
#include "../devices.h"


static void a620led_blue_set(struct led_classdev *led_cdev,
			       enum led_brightness value)
{
	if (value)
	    asus620_gpo_set(GPO_A620_BLUE_LED);
	else
	    asus620_gpo_clear(GPO_A620_BLUE_LED);
}

static void a620led_red_set(struct led_classdev *led_cdev,
			     enum led_brightness value)
{
	if (value)
	    asus620_gpo_set(GPO_A620_RED_LED);
	else
	    asus620_gpo_clear(GPO_A620_RED_LED);
}

//For test only
static void a620bt_pwr_set(struct led_classdev *led_cdev,
			     enum led_brightness value)
{
	if (value)
	    asus620_gpo_set(GPO_A620_BLUETOOTH);
	else
	    asus620_gpo_clear(GPO_A620_BLUETOOTH);
}

static struct led_classdev a620_red_led = {
	.name			= "a620:red",
	.default_trigger	= "ide-disk",
	.brightness_set		= a620led_red_set,
	.flags			= LED_CORE_SUSPENDRESUME,
};

static struct led_classdev a620_blue_led = {
	.name			= "a620:blue",
	.default_trigger	= "heartbeat",
	.brightness_set		= a620led_blue_set,
	.flags			= LED_CORE_SUSPENDRESUME,
};

static struct led_classdev a620_bt_pwr = {
	.name			= "a620:bt_pwr",
	.default_trigger	= "none",
	.brightness_set		= a620bt_pwr_set,
	.flags			= LED_CORE_SUSPENDRESUME,
};

static int a620led_probe(struct platform_device *pdev)
{
	int ret;

	ret = led_classdev_register(&pdev->dev, &a620_red_led);
	if (ret < 0)
		return ret;

	ret = led_classdev_register(&pdev->dev, &a620_blue_led);
	if (ret < 0)
		led_classdev_unregister(&a620_red_led);

	ret = led_classdev_register(&pdev->dev, &a620_bt_pwr);
	if (ret < 0)
		led_classdev_unregister(&a620_bt_pwr);

	return ret;
}

static int a620led_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&a620_red_led);
	led_classdev_unregister(&a620_blue_led);
	led_classdev_unregister(&a620_bt_pwr);

	return 0;
}

static struct platform_driver a620led_driver = {
	.probe		= a620led_probe,
	.remove		= a620led_remove,
	.driver		= {
		.name		= "a620-led",
		.owner		= THIS_MODULE,
	},
};

module_platform_driver(a620led_driver);

MODULE_DESCRIPTION("Asus MyPal a620 LED driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:a620-led");
