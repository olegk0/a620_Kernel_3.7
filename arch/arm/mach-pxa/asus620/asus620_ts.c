/*
 *  linux/arch/arm/mach-pxa/asus620_ts.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  Copyright (c) 2003 Adam Turowski
 *  Copyright (C) 2004 Nicolas Pouillon
 *  Copyright (C) 2006 Vincent Benony
 *  (c) 2014 olegvedi@gmail.com
 *
 *  2004-09-23: Nicolas Pouillon
 *      Initial code based on 2.6.0 from Adam Turowski ported to 2.6.7
 *
 *  2006-01-14: Vincent Benony
 *      Merge with A716 touchscreen code
 *      Some corrections
 *
 */

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/apm-emulation.h>
#include <linux/gpio.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <mach/pxa25x.h>
#include <mach/asus620.h>

#include <linux/input.h>
#include <linux/platform_device.h>

#include "../generic.h"

#define SSCR0_SSE_ENABLED	0x00000080

#define SAMPLE_TIMEOUT 10

#define TS_POLL_DELAY	1	/* ms delay before the first sample */
#define TS_POLL_PERIOD	5	/* ms delay between samples */

#define SSDR_REG		__REG(0x41000010)  // (Write / Read) SSP Port 1 Data Write Register/SSP
#define SSSR_REG		__REG(0x41000008)  /* SSP Port 1 Status Register */
#define SSCR0_REG	__REG(0x41000000)  /* SSP Port 1 Control Register 0 */
#define SSCR1_REG	__REG(0x41000004)  /* SSP Port 1 Control Register 1 */
#define SSCR0_National	(0x2 << 4)	/* National Microwire */
#define SSCR0_SSE	(1 << 7)	/* Synchronous Serial Port Enable */


#define GPIO23_SCLK             23      /* SSP clock */
#define GPIO24_SFRM             24      /* SSP Frame */
#define GPIO25_STXD             25      /* SSP transmit */
#define GPIO26_SRXD             26      /* SSP receive */

#define GPIO_bit(x)     (1 << ((x) & 0x1f))

struct ad7843{
    struct input_dev *idev;
//    spinlock_t ts_lock;
    struct mutex	lock;
    int touch_pressed;
    int disabled;
    int suspended;
    int irq;
    wait_queue_head_t	wait;
};

    spinlock_t ts_lock;

#define TS_IRQ_GPIO	GPIO27_A620_STYLUS_IRQ


/*
 * Send a touchscreen event to the input Linux system
 */

static void report_touchpanel (struct input_dev *idev, int x, int y, int pressure)
{
	//printk("report (%d, %d) with pressure %d\n", x, y, pressure);
	input_report_key (idev, BTN_TOUCH, pressure != 0);
	input_report_abs (idev, ABS_PRESSURE, pressure);
	input_report_abs (idev, ABS_X, x);
	input_report_abs (idev, ABS_Y, y);
	input_sync (idev);
}

#define CTRL_START  0x80
#define CTRL_YPOS   0x10
#define CTRL_Z1POS  0x30
#define CTRL_Z2POS  0x40
#define CTRL_XPOS   0x50
#define CTRL_TEMP0  0x04
#define CTRL_TEMP1  0x74
#define CTRL_VBAT   0x24
#define CTRL_AUX    0x64
#define CTRL_PD0    0x01
#define CTRL_PD1    0x02

#define SSSR_TNF_MSK    (1u << 2)
#define SSSR_RNE_MSK    (1u << 3)


#define ADSCTRL_ADR_SH    4

unsigned long spi_ctrl(ulong data)
{
	unsigned long ret, flags;

	spin_lock_irqsave(&ts_lock, flags);

	SSDR_REG = data;
	while ((SSSR_REG & SSSR_TNF_MSK) != SSSR_TNF_MSK) udelay(0);
	udelay(1);
	while ((SSSR_REG & SSSR_RNE_MSK) != SSSR_RNE_MSK) udelay(0);
	ret = (SSDR_REG);

	spin_unlock_irqrestore(&ts_lock, flags);

	return ret;
}

typedef struct ts_pos_s {
	unsigned long xd;
	unsigned long yd;
} ts_pos_t;

int get_pendown_state(void)
{
    return !gpio_get_value(TS_IRQ_GPIO);
}

void read_xydata(ts_pos_t *tp)
{
#define	abscmpmin(x,y,d) ( ((int)((x) - (y)) < (int)(d)) && ((int)((y) - (x)) < (int)(d)) )
	unsigned long cmd;
	unsigned int t, x, y, z[2];
	unsigned long pressure;
	int i, j, k;
	int d = 8, c = 10;
	int err = 0;

	for (i = j = k = 0, x = y = 0;; i = 1) {
		/* Pressure */
		cmd = CTRL_PD0 | CTRL_PD1 | CTRL_START | CTRL_Z1POS;
		t = spi_ctrl(cmd);
		z[i] = spi_ctrl(cmd);
 
		if (i)
		    break;

		/* X-axis */
		cmd = CTRL_PD0 | CTRL_PD1 | CTRL_START | CTRL_XPOS;
		x = spi_ctrl(cmd);
		for (j = 0; !err; j++) {
			t = x;
			x = spi_ctrl(cmd);
			if (abscmpmin(t, x, d))
				break;
			if (j > c) {
				err = 1;
				//printk("ts: x(%d,%d,%d)\n", t, x, t - x);
			}
		}

		/* Y-axis */
		cmd = CTRL_PD0 | CTRL_PD1 | CTRL_START | CTRL_YPOS;
		y = spi_ctrl(cmd);
		for (k = 0; !err; k++) {
			t = y;
			y = spi_ctrl(cmd);
			if (abscmpmin(t ,y , d))
				break;
			if (k > c) {
				err = 1;
				//printk("ts: y(%d,%d,%d)\n", t, y, t - y);
			}
		}
	}
	pressure = 1;
	for (i = 0; i < 2; i++) {
		if (!z[i])
			pressure = 0;
	}
	if (pressure) {
		for (i = 0; i < 2; i++){
			if (z[i] < 10)
				err = 1;
		}
		if (x >= 4095)
			err = 1;
	}

	cmd &= ~(CTRL_PD0 | CTRL_PD1);
	t = spi_ctrl(cmd);

	if (err == 0 && pressure != 0) {
		//printk("ts: pxyp=%d(%d/%d,%d/%d)%d\n", z[0], x, j, y, k, z[1]);
	} else {
		//printk("pxype=%d,%d,%d,%d\n", z[0], x, y, z[1]);
		x = 0; y = 0;
	}
	tp->xd = x;
	tp->yd = y;
}

/* Enable Disable subs */
static void ts_stop(struct ad7843 *ts)
{
    if (!ts->suspended) {

	ts->disabled = true;
	mb();
	wake_up(&ts->wait);
	disable_irq(ts->irq);
    }
}

static void ts_restart(struct ad7843 *ts)
{
    if (!ts->suspended) {
	ts->disabled = false;
	mb();
	enable_irq(ts->irq);
    }
}

static irqreturn_t asus620_pen(int irq, void* data)
{
    struct ad7843 *ts = data;
    ts_pos_t ts_pos;

    msleep(TS_POLL_DELAY);

    while (!ts->disabled && get_pendown_state()) {

	ts->touch_pressed = 1;
	read_xydata(&ts_pos);

	if(!ts->disabled)
	    report_touchpanel(ts->idev, ts_pos.xd, ts_pos.yd, 1);

	wait_event_timeout(ts->wait, ts->disabled, msecs_to_jiffies(TS_POLL_PERIOD));
//        msleep(TS_POLL_PERIOD);
    }


    if (ts->touch_pressed) {

	input_report_key(ts->idev, BTN_TOUCH, 0);
	input_report_abs(ts->idev, ABS_PRESSURE, 0);
	input_sync(ts->idev);

	ts->touch_pressed = 0;
    }

    return IRQ_HANDLED;

};


/*
 * Retrieve battery informations
 */

#define ASUS620_MAIN_BATTERY_MAX 1676 // ~ sometimes it's more or less (> 1700 - AC)
#define ASUS620_MAIN_BATTERY_MIN 1347
#define ASUS620_MAIN_BATTERY_RANGE (ASUS620_MAIN_BATTERY_MAX - ASUS620_MAIN_BATTERY_MIN)

//#define ASUS620_BAT_SAMPS	240
//int	battery_samples[ASUS620_BAT_SAMPS], battery_samples_nb;
//
//static void asus620_battery(unsigned long nr)
//{
//	int sample1, sample2, sample3, sample;
//
//	sample1 = spi_ctrl(CTRL_PD0 | CTRL_START | CTRL_VBAT); // main battery: min - 1347, max - 1676 (1700 AC)
//	mdelay(100);
//	sample2 = spi_ctrl(CTRL_PD0 | CTRL_START | CTRL_VBAT);
//	mdelay(100);
//	sample3 = spi_ctrl(CTRL_PD0 | CTRL_START | CTRL_VBAT);
//	spi_ctrl(CTRL_START | CTRL_VBAT);
//
//	sample = (sample1 + sample2 + sample3) / 3;
//	sample = ((sample - ASUS620_MAIN_BATTERY_MIN) * 100) / ASUS620_MAIN_BATTERY_RANGE;
//	if (sample > 100)
//		battery_power = 100;
//	else
//		battery_power = sample;
///*
//	if (battery_power < 10 && !(GPLR(GPIO20_A620_AC_IN) & GPIO_bit(GPIO20_A620_AC_IN)))
//		a716_gpo_set(GPO_A716_POWER_LED_RED);
//	else
//		a716_gpo_clear(GPO_A716_POWER_LED_RED);
//*/
//	//printk("battery: %d\n", battery_power);
//
//
//	if (battery_samples_nb < ASUS620_BAT_SAMPS)
//	{
//		battery_samples[battery_samples_nb++] = (sample1 + sample2 + sample3) / 3 - ASUS620_MAIN_BATTERY_MIN;
//		battery_time = 999999999;
//	}
//	else
//	{
//		int	i, div;
//		for (i=1; i<ASUS620_BAT_SAMPS; i++) battery_samples[i-1] = battery_samples[i];
//		battery_samples[ASUS620_BAT_SAMPS-1] = (sample1 + sample2 + sample3) / 3 - ASUS620_MAIN_BATTERY_MIN;
//		div = battery_samples[0] - battery_samples[ASUS620_BAT_SAMPS-1];
//		if (div) battery_time = ASUS620_BAT_SAMPS * battery_samples[0] / div;
//	}
//
//	mod_timer(&timer_bat, jiffies + (1000 * HZ) / 1000);
//}

static void asus620_apm_get_power_status(struct apm_power_info *info)
{
    int	sample = spi_ctrl(CTRL_PD0|CTRL_START|CTRL_VBAT);
    int	battery_power;
    int	battery_time = 99999999;


	battery_power = ((sample - ASUS620_MAIN_BATTERY_MIN) * 100) / ASUS620_MAIN_BATTERY_RANGE;

	info->battery_life = battery_power;
/*
printk("BAT:%d\n",sample);
printk("TEMP0:%d\n",spi_ctrl(CTRL_PD0|CTRL_START|CTRL_TEMP0));
printk("TEMP1:%d\n",spi_ctrl(CTRL_PD0|CTRL_START|CTRL_TEMP1));
*/
	if (!(GPLR(GPIO20_A620_AC_IN) & GPIO_bit(GPIO20_A620_AC_IN)))
		info->ac_line_status = APM_AC_OFFLINE;
	else
		info->ac_line_status = APM_AC_ONLINE;

	if (battery_power > 50)
		info->battery_status = APM_BATTERY_STATUS_HIGH;
	else if (battery_power < 10)
		info->battery_status = APM_BATTERY_STATUS_CRITICAL;
	else
		info->battery_status = APM_BATTERY_STATUS_LOW;

	info->time  = battery_time;
	info->units = APM_UNITS_SECS;
}

/*
 * Initialize touchscreen chip
 */

static unsigned long ts_pin_config[] = {
GPIO23_SSP1_SCLK,
GPIO24_SSP1_SFRM,
GPIO25_SSP1_TXD,
GPIO26_SSP1_RXD,
};

void asus620_ts_init_ssp(void)
{
	GPDR(GPIO23_SCLK) |=  GPIO_bit(GPIO23_SCLK);
	GPDR(GPIO24_SFRM) |=  GPIO_bit(GPIO24_SFRM);
	GPDR(GPIO25_STXD) |=  GPIO_bit(GPIO25_STXD);
	GPDR(GPIO26_SRXD) &= ~GPIO_bit(GPIO26_SRXD);

        pxa2xx_mfp_config(ARRAY_AND_SIZE(ts_pin_config));

	SSCR0_REG = 0;
	SSCR0_REG |= 0xB; /* 12 bits */
	SSCR0_REG |= SSCR0_National;
	SSCR0_REG |= 0x1100; /* 100 mhz */

	SSCR1_REG = 0;

	SSCR0_REG |= SSCR0_SSE;

	spi_ctrl(CTRL_YPOS | CTRL_START);
	mdelay(5);
	spi_ctrl(CTRL_Z1POS | CTRL_START);
	mdelay(5);
	spi_ctrl(CTRL_Z2POS | CTRL_START);
	mdelay(5);
	spi_ctrl(CTRL_XPOS | CTRL_START);
	mdelay(5);
}

void asus620_ts_disable_ssp(void)
{
    SSCR0_REG &= ~SSCR0_SSE;
}

/*
 * Driver init function
 */

static int a620_ts_probe(struct platform_device *pdev)
{
    struct ad7843 *ts;
    struct input_dev *idev;
    int err;

    printk("%s: initializing the touchscreen.\n", __FUNCTION__);

    ts = kzalloc(sizeof(struct ad7843), GFP_KERNEL);
    // Init input device
    idev = input_allocate_device();

    if (!idev || !ts) {
        err = -ENOMEM;
        goto err_mem;
    }

    ts->irq = PXA_GPIO_TO_IRQ(TS_IRQ_GPIO);
    idev->name = "Asus MyPal 620 touchscreen and battery driver";
    idev->phys = "ad7873";

    set_bit(EV_KEY,		idev->evbit);
    set_bit(EV_ABS,		idev->evbit);
    set_bit(BTN_TOUCH,	idev->keybit);
    set_bit(ABS_PRESSURE,	idev->absbit);
    set_bit(ABS_X,		idev->absbit);
    set_bit(ABS_Y,		idev->absbit);

    input_set_abs_params(idev, ABS_X, 212, 3880, 0, 0);
    input_set_abs_params(idev, ABS_Y, 180, 3940, 0, 0);
    input_set_abs_params(idev, ABS_PRESSURE, 0, 1, 0, 0);

    ts->idev = idev;

    asus620_ts_init_ssp();
    apm_get_power_status = asus620_apm_get_power_status;

    mutex_init(&ts->lock);
    init_waitqueue_head(&ts->wait);

    ts->touch_pressed = 0;
    ts->disabled = 0;
    ts->suspended = 0;

    err = gpio_request(TS_IRQ_GPIO, "TS CLICK");
    if(err)
        goto err_ssp;

    gpio_direction_input(TS_IRQ_GPIO);

    err = request_threaded_irq(ts->irq, NULL, asus620_pen,IRQF_TRIGGER_FALLING | IRQF_ONESHOT,idev->name, ts);
    if(err)
        goto err_gpio;

    err = input_register_device(idev);
    if (err)
	goto err_irq;

    device_init_wakeup(&pdev->dev, true);

    platform_set_drvdata(pdev, ts);

    spin_lock_init(&ts_lock);

    return 0;

err_irq:
    free_irq(ts->irq, ts);
err_gpio:
    gpio_free(TS_IRQ_GPIO);
err_ssp:
    asus620_ts_disable_ssp();
err_mem:
    input_free_device(idev);
    kfree(ts);
    return err;
}

static int __devexit a620_ts_remove(struct platform_device *pdev)
{
    struct ad7843 *ts = platform_get_drvdata(pdev);

    apm_get_power_status = NULL;
    free_irq(ts->irq, ts);
    input_unregister_device(ts->idev);
    input_free_device(ts->idev);

    asus620_ts_disable_ssp();

//	del_timer_sync(&timer_bat);
    gpio_free(TS_IRQ_GPIO);
    kfree(ts);

    return 0;
}


#ifdef CONFIG_PM_SLEEP
static int a620_ts_suspend(struct device *dev)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct ad7843 *ts = platform_get_drvdata(pdev);

    mutex_lock(&ts->lock);

    if (!ts->suspended) {

//	if (!ts->disabled)
	ts_stop(ts);

	if (device_may_wakeup(dev))
	    enable_irq_wake(ts->irq);

	asus620_ts_disable_ssp();

	ts->suspended = true;
    }
    mutex_unlock(&ts->lock);

    return 0;
}

static int a620_ts_resume(struct device *dev)
{
    struct platform_device *pdev = to_platform_device(dev);
    struct ad7843 *ts = platform_get_drvdata(pdev);

    mutex_lock(&ts->lock);

    if (ts->suspended) {

	asus620_ts_init_ssp();

	ts->suspended = false;

	if (device_may_wakeup(dev))
	    disable_irq_wake(ts->irq);

//	if (!ts->disabled)

	ts_restart(ts);
    }

    mutex_unlock(&ts->lock);
    return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(a620_ts_dev_pm_ops, a620_ts_suspend, a620_ts_resume);

static struct platform_driver a620_ts_driver = {
    .driver = {
	.name = "a620-ts",
	.owner  = THIS_MODULE,
	.pm = &a620_ts_dev_pm_ops,
    },
    .probe = a620_ts_probe,
    .remove = __devexit_p(a620_ts_remove),
};

module_platform_driver(a620_ts_driver);

MODULE_ALIAS("platform:a620-ts");
MODULE_AUTHOR("Adam Turowski, Nicolas Pouillon, Vincent Benony, olegvedi@gmail.com");
MODULE_DESCRIPTION("Touchscreen and battery support for the MyPal A620");
MODULE_LICENSE("GPL");
