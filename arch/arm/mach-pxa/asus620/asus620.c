/*
 *  linux/arch/arm/mach-pxa/asus620.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  (c) 2014 olegvedi@gmail.com
 *
 *  Copyright (c) 2006 Vincent Benony
 *  Copyright (c) 2004 Vitaliy Sardyko
 *  Copyright (c) 2003,2004 Adam Turowski
 *  Copyright (c) 2001 Cliff Brake, Accelent Systems Inc.
 *
 *
 *  2001-09-13: Cliff Brake <cbrake@accelent.com>
 *              Initial code for IDP
 *  2003-12-03: Adam Turowski
 *		code adaptation for Asus 620
 *  2004-07-23: Vitaliy Sardyko
 *		updated to 2.6.7 format,
 *		split functions between modules.
 *  2006-01-16: Vincent Benony
 *              gpo handling
 */

#include <linux/syscore_ops.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/irq.h>

#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/pda_power.h>
#include <linux/pwm_backlight.h>
#include <linux/gpio.h>
#include <linux/power_supply.h>
#include <linux/usb/gpio_vbus.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/pxa25x.h>
#include <mach/audio.h>
#include <sound/uda1380.h>

#include <mach/asus620.h>
//#include <linux/platform_data/mmc-pxamci.h>
#include <linux/platform_data/video-pxafb.h>
#include <linux/platform_data/irda-pxaficp.h>
#include <mach/udc.h>
#include <mach/reset.h>

#include <linux/spi/ads7846.h>
#include <linux/spi/spi.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/i2c/pxa-i2c.h>

#include <asm/setup.h>

#include "../generic.h"
#include "../devices.h"

#define DEBUG 1

#if DEBUG
#  define DPRINTK(fmt, args...)	printk("%s: " fmt, __FUNCTION__ , ## args)
#else
#  define DPRINTK(fmt, args...)
#endif


static int has_tcon = 0;
static int lcd_power = 0;

/******************************************************************************
 * Pin configuration
 ******************************************************************************/
static unsigned long asus620_pin_config[] __initdata = {
//	 MMC 

	/* PCMCIA-CF */
/*	MFP_CFG_IN(GPIO7, AF0),//GPIO7_A620_CF_IRQ
	MFP_CFG_IN(GPIO8, AF0),//GPIO8_A620_CF_BVD1
	MFP_CFG_IN(GPIO9, AF0),//GPIO9_A620_CF_READY_N
	MFP_CFG_IN(GPIO21, AF0),//GPIO21_A620_CF_VS1_N
	MFP_CFG_IN(GPIO22, AF0),//GPIO22_A620_CF_VS2_N
*/
/*
	GPIO48_nPOE,
	GPIO49_nPWE,
	GPIO50_nPIOR,
	GPIO51_nPIOW,
	GPIO52_nPCE_1,
	GPIO53_nPCE_2,
	GPIO55_nPREG,
	GPIO56_nPWAIT,
	GPIO57_nIOIS16,
*/

	/* PWM */
//	GPIO16_PWM0_OUT,

	/* USB */
//	MFP_CFG_OUT(GPIO33, AF0, DRIVE_LOW),//GPIO33_A620_nUSB_PULL_UP,
//	MFP_CFG_IN(GPIO10, AF0),//	GPIO10_A620_nUSB_DETECT,

	/* IrDA */
	GPIO46_FICP_RXD,//GPIO46_STUART_RXD
	GPIO47_FICP_TXD,//GPIO47_STUART_TXD

	/* FFUART */
//	MFP_CFG_IN(GPIO34, AF0),
//	MFP_CFG_IN(GPIO39, AF0),
	GPIO34_FFUART_RXD,
	GPIO39_FFUART_TXD,

        /* BTUART */
/*        GPIO42_BTUART_RXD,
	GPIO43_BTUART_TXD,
        GPIO44_BTUART_CTS,
	GPIO45_BTUART_RTS,
*/
	/* LCD */
//	MFP_CFG_OUT(GPIO75, AF0, DRIVE_LOW),//	pxa_gpio_mode (GPIO75_A620_TCON_EN|GPIO_OUT);
/*	GPIO58_LCD_LDD_0,
	GPIO59_LCD_LDD_1,
	GPIO60_LCD_LDD_2,
	GPIO61_LCD_LDD_3,
	GPIO62_LCD_LDD_4,
	GPIO63_LCD_LDD_5,
	GPIO64_LCD_LDD_6,
	GPIO65_LCD_LDD_7,
	GPIO66_LCD_LDD_8,
	GPIO67_LCD_LDD_9,
	GPIO68_LCD_LDD_10,
	GPIO69_LCD_LDD_11,
	GPIO70_LCD_LDD_12,
	GPIO71_LCD_LDD_13,
	GPIO72_LCD_LDD_14,
	GPIO73_LCD_LDD_15,
	MFP_CFG_OUT(GPIO75, AF0, DRIVE_LOW),//GPIO75_LCD_LCLK
	GPIO76_LCD_PCLK,
	GPIO77_LCD_BIAS,
*/
	/* GPIO KEYS */
	MFP_CFG_IN(GPIO41,AF0),//_A620_JOY_SW_N,
	MFP_CFG_IN(GPIO40,AF0),//_A620_JOY_SE_N,
	MFP_CFG_IN(GPIO37,AF0),//_A620_JOY_NE_N,
	MFP_CFG_IN(GPIO36,AF0),//_A620_JOY_NW_N,
	MFP_CFG_IN(GPIO35,AF0),//_A620_JOY_PUSH_N,
//	MFP_CFG_IN(GPIO0,AF0),//_A620_POWER_BUTTON_N,
//	GPIO1_A620_RESET_BUTTON_N,
	MFP_CFG_IN(GPIO2,AF0),//_A620_HOME_BUTTON_N,
	MFP_CFG_IN(GPIO3,AF0),//_A620_CALENDAR_BUTTON_N,
	MFP_CFG_IN(GPIO4,AF0),//_A620_CONTACTS_BUTTON_N,
	MFP_CFG_IN(GPIO5,AF0),//_A620_TASKS_BUTTON_N,
	MFP_CFG_IN(GPIO11,AF0),//_A620_RECORD_BUTTON_N,
	GPIO0_GPIO | WAKEUP_ON_EDGE_BOTH,
//	GPIO1_RST,

	/* MISC */
//	MFP_CFG_OUT(GPIO54, AF0, DRIVE_HIGH),//_A620_LED_ENABLE,??
//	GPIO12_A620_HEARPHONE_N,
/*	GPIO80_nCS_4,
	MFP_CFG_IN(GPIO20,AF0),//_A620_AC_IN
	MFP_CFG_OUT(GPIO13, AF0, DRIVE_LOW),
	MFP_CFG_OUT(GPIO14, AF0, DRIVE_LOW),
	MFP_CFG_OUT(GPIO17, AF0, DRIVE_LOW),
	MFP_CFG_OUT(GPIO19, AF0, DRIVE_LOW),
	MFP_CFG_OUT(GPIO28, AF0, DRIVE_LOW),
	MFP_CFG_OUT(GPIO30, AF0, DRIVE_LOW),
	MFP_CFG_OUT(GPIO31, AF0, DRIVE_LOW),
	MFP_CFG_OUT(GPIO32, AF0, DRIVE_LOW),
	MFP_CFG_OUT(GPIO54, AF0, DRIVE_LOW),
	MFP_CFG_OUT(GPIO78, AF0, DRIVE_LOW),
	MFP_CFG_OUT(GPIO79, AF0, DRIVE_LOW),
*/

	/* I2S */
	GPIO28_I2S_BITCLK_OUT,
//	GPIO28_I2S_BITCLK_IN,
	GPIO29_I2S_SDATA_IN,
	GPIO30_I2S_SDATA_OUT,
	GPIO31_I2S_SYNC,
	GPIO32_I2S_SYSCLK,


};

/******************************************************************************
 * SD/MMC card controller
 ******************************************************************************/
/*
static struct pxamci_platform_data asus620_mci_platform_data = {
	.ocr_mask		= MMC_VDD_32_33 | MMC_VDD_33_34,
	.gpio_card_detect	= GPIO_NR_asus620_SD_DETECT_N,
	.gpio_card_ro		= GPIO_NR_asus620_SD_READONLY,
	.gpio_power		= GPIO_NR_asus620_SD_POWER,
};
*/
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
/******************************************************************************
 * GPIO keys
 ******************************************************************************/
static struct gpio_keys_button asus620_pxa_buttons[] = {
//	{KEY_F1,	GPIO_NR_asus620_KEY_CONTACTS,	1, "Contacts" },
//	{KEY_F2,	GPIO_NR_asus620_KEY_CALENDAR,	1, "Calendar" },
//	{KEY_F3,	GPIO_NR_asus620_KEY_TASKS,	1, "Tasks" },
//	{KEY_F4,	GPIO_NR_asus620_KEY_NOTES,	1, "Notes" },
//	{KEY_POWER,	GPIO0_A620_POWER_BUTTON_N,	1, "Power" ,EV_KEY ,1 },
	{KEY_ENTER,	GPIO35_A620_JOY_PUSH_N,	1, "Center" ,EV_KEY ,0 },
	{KEY_LEFT,	GPIO36_A620_JOY_NW_N,	1, "Left" ,EV_KEY ,0 },
	{KEY_RIGHT,	GPIO37_A620_JOY_NE_N,	1, "Right" ,EV_KEY ,0 },
	{KEY_DOWN,	GPIO40_A620_JOY_SE_N,	1, "Down" ,EV_KEY ,0 },
	{KEY_UP,	GPIO41_A620_JOY_SW_N,		1, "Up" ,EV_KEY ,0 },
};

static struct gpio_keys_platform_data asus620_pxa_keys_data = {
	.buttons	= asus620_pxa_buttons,
	.nbuttons	= ARRAY_SIZE(asus620_pxa_buttons),
};

static struct platform_device asus620_pxa_keys = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data = &asus620_pxa_keys_data,
	},
};
#endif

/******************************************************************************
 * Backlight
 ******************************************************************************/
#define MIN_BRIGHT_PHYS 213
#define MAX_BRIGHT_PHYS 0x3ff
#define MAX_BRIGHTNESS (MAX_BRIGHT_PHYS - MIN_BRIGHT_PHYS)
/*static struct gpio a620_bl_gpios[] = {
	{ GPIO_NR_asus620_BL_POWER, GPIOF_INIT_LOW, "Backlight power" },
	{ GPIO_NR_asus620_LCD_POWER, GPIOF_INIT_LOW, "LCD power" },
};
*/
/*
static int asus620_backlight_init(struct device *dev)
{
//	return gpio_request_array(ARRAY_AND_SIZE(a620_bl_gpios));
}
*/
static int asus620_backlight_notify(struct device *dev, int brightness)
{
//	gpio_set_value(GPIO_NR_asus620_BL_POWER, brightness);
//	gpio_set_value(GPIO_NR_asus620_LCD_POWER, brightness);
    DPRINTK("bright: %d\n", brightness);

//    PWM_CTRL0   = 7;
//    PWM_PWDUTY0 = brightness + MAX_BRIGHTNESS;
//    PWM_PERVAL0 = 0x3ff;

    if (brightness)
	CKEN |= (1<<CKEN_PWM0);
//		pxa_set_cken(CKEN_PWM0, 1);
    else
//		pxa_set_cken(CKEN_PWM0, 0);
	CKEN &= ~(1<<CKEN_PWM0);

//TODO
//GPIO16_A620_BACKLIGHT
//GPO_A620_BACKLIGHT
	return brightness;
}
/*
static void asus620_backlight_exit(struct device *dev)
{
//	gpio_free_array(ARRAY_AND_SIZE(a620_bl_gpios));
}
*/
static struct platform_pwm_backlight_data asus620_backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= MAX_BRIGHT_PHYS,
	.dft_brightness	= MAX_BRIGHT_PHYS,
	.pwm_period_ns	= 300000,
//	.init		= asus620_backlight_init,
	.notify		= asus620_backlight_notify,
//	.exit		= asus620_backlight_exit,
};

static struct platform_device asus620_backlight = {
	.name	= "pwm-backlight",
	.dev	= {
		.parent		= &pxa25x_device_pwm0.dev,
		.platform_data	= &asus620_backlight_data,
	},
};

/******************************************************************************
 * IrDA
 ******************************************************************************/
static void asus620_irda_transceiver_mode(struct device *dev, int mode) 
{
    unsigned long flags;

    local_irq_save(flags);

    if (mode & IR_SIRMODE)
	asus620_gpo_clear(GPO_A620_IRDA_FIR_MODE);
    else if (mode & IR_FIRMODE)
	asus620_gpo_set(GPO_A620_IRDA_FIR_MODE);

    if (mode & IR_OFF)
	asus620_gpo_set(GPO_A620_IRDA_POWER_N);
    else
	asus620_gpo_clear(GPO_A620_IRDA_POWER_N);

    local_irq_restore(flags);
}

static struct pxaficp_platform_data asus620_ficp_platform_data = {
    .gpio_pwdown		= -1,
    .transceiver_cap	= IR_SIRMODE | IR_OFF | IR_FIRMODE,
    .transceiver_mode	= asus620_irda_transceiver_mode,
};

/******************************************************************************
 * UDC
 ******************************************************************************/

static struct gpio_vbus_mach_info asus620_udc_info = {
    .gpio_pullup		= GPIO33_A620_USB_PULL_UP_N,
    .gpio_pullup_inverted	= 0,
    .gpio_vbus		= GPIO10_A620_USB_DETECT_N,
    .gpio_vbus_inverted	= 0,
};

static struct platform_device asus620_gpio_vbus = {
	.name	= "gpio-vbus",
	.id	= -1,
	.dev	= {
		.platform_data	= &asus620_udc_info,
	},
};


/******************************************************************************
 * Power supply
 ******************************************************************************/
static int power_supply_init(struct device *dev)
{
	int ret;

	ret = gpio_request(GPIO20_A620_AC_IN, "CABLE_STATE_AC");
	if (ret)
		goto err1;
	ret = gpio_direction_input(GPIO20_A620_AC_IN);
	if (ret)
		goto err2;

	return 0;

err2:
	gpio_free(GPIO20_A620_AC_IN);
err1:
	return ret;
}

static int asus620_is_ac_online(void)
{
	return !!gpio_get_value(GPIO20_A620_AC_IN);
}

static void power_supply_exit(struct device *dev)
{
	gpio_free(GPIO20_A620_AC_IN);
}

static char *asus620_supplicants[] = {
	"main-battery",
};

static struct pda_power_pdata power_supply_info = {
	.init            = power_supply_init,
	.is_ac_online    = asus620_is_ac_online,
	.exit            = power_supply_exit,
	.supplied_to     = asus620_supplicants,
	.num_supplicants = ARRAY_SIZE(asus620_supplicants),
};

static struct platform_device power_supply = {
	.name = "pda-power",
	.id   = -1,
	.dev  = {
		.platform_data = &power_supply_info,
	},
};

/******************************************************************************
 * Audio
 ******************************************************************************/

static struct uda1380_platform_data uda1380_info = {
	.gpio_power = -1,
	.gpio_reset = -1,
	.dac_clk    = UDA1380_DAC_CLK_SYSCLK,//WSPLL,SYSCLK
};

static struct i2c_board_info i2c_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("uda1380", 0x1a),//0x18   0x1a
		.platform_data = &uda1380_info,
	},
};


static struct platform_device audio = {
    .name	= "a620-audio",
    .id	= -1,
};

/******************************************************************************
 * Framebuffer
 ******************************************************************************/
/*
  Adam's asus620: (probably with TCON)
 LCCR0: 0x003000f9 -- ENB | LDM | SFM | IUM | EFM | PAS | BM | OUM
 LCCR1: 0x15150cef -- PPL=0xef, HSW=0x3, ELW=0x15, BLW=0x15
 LCCR2: 0x06060d3f -- LPP=0x13f, VSW=0x3, EFW=0x6, BFW=0x6
 LCCR3: 0x04700007 -- PCD=0x7, ACB=0x0, API=0x0, VSP, HSP, PCP, BPP=0x4

  Nipo's asus620: (No TCON)
 LCCR0: 0x001000f9 -- ENB | LDM | SFM | IUM | EFM | PAS | BM
 LCCR1: 0x0a0afcef -- PPL=0xef, HSW=0xfc, ELW=0x0a, BLW=0x0a
 LCCR2: 0x0303153f -- LPP=0x13f, VSW=0x3, EFW=0x3, BFW=0x3
 LCCR3: 0x04700007 -- PCD=0x7, ACB=0x0, API=0x0, VSP, HSP, PCP, BPP=0x4 
*/

static struct pxafb_mode_info asus620_mode_tcon = {
        .pixclock =     171521,
        .bpp =          16,
        .xres =         240,
        .yres =         320,
        .hsync_len =    4,
        .vsync_len =    4,
        .left_margin =  22,
        .upper_margin = 6,
        .right_margin = 22,
        .lower_margin = 6,
        .sync =         0,
};

static struct pxafb_mach_info asus620_fb_info_tcon = {
    .modes = &asus620_mode_tcon,
    .num_modes = 1,
        .lccr0 =        ( LCCR0_LDM | LCCR0_SFM | LCCR0_IUM | LCCR0_EFM |
			  LCCR0_PAS | LCCR0_QDM | LCCR0_BM | LCCR0_OUM ),
        .lccr3 =        ( LCCR3_HorSnchL | LCCR3_VrtSnchL | LCCR3_BPP(16) |
			  LCCR3_PCP ),
	.lcd_conn	= LCD_COLOR_TFT_16BPP | LCD_PCLK_EDGE_FALL,
};

static struct pxafb_mode_info asus620_mode_notcon = {
    .pixclock =	171521,
    .bpp =		16,
    .xres =		240,
    .yres =		320,
    .hsync_len =	252,
    .vsync_len =	4,
    .left_margin =	11,
    .upper_margin =	4,
    .right_margin =	11,
    .lower_margin =	4,
    .sync =		0,
};

static struct pxafb_mach_info asus620_fb_info_notcon = {
	.modes = &asus620_mode_notcon,
	.num_modes = 1,
	.lccr0 =	( LCCR0_ENB | LCCR0_LDM | LCCR0_SFM | LCCR0_IUM | LCCR0_EFM |
		  LCCR0_PAS | LCCR0_BM ),
	.lccr3 =	( LCCR3_HorSnchL | LCCR3_VrtSnchL | LCCR3_BPP(16) | LCCR3_PCP ),
	.lcd_conn	= LCD_COLOR_TFT_16BPP | LCD_PCLK_EDGE_FALL,
};

static int asus620_lcd_set_power (int setp)
{
    DPRINTK("pwer : %d\n", setp);
    lcd_power = setp;

    switch (setp) {
	case FB_BLANK_UNBLANK:
	case FB_BLANK_NORMAL:
//			asus620_gpo_set(GPO_A620_LCD_ENABLE);
//	    asus620_gpo_set(GPIO74_A620_TCON_HERE_N);//TODO
	    asus620_gpo_set(GPO_A620_LCD_POWER3);
	    mdelay(30);
	    asus620_gpo_set(GPO_A620_LCD_POWER1);
	    mdelay(30);
	    break;
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
                        break;
	case FB_BLANK_POWERDOWN:
	    asus620_gpo_clear(GPO_A620_LCD_POWER1);
	    mdelay(65);
	    asus620_gpo_clear(GPO_A620_LCD_POWER3);
//			asus620_gpo_clear(GPO_A620_LCD_ENABLE);
//	    asus620_gpo_clear(GPIO74_A620_TCON_HERE_N);//TODO
	    break;
    }

    return 0;
}

static unsigned long tcon_pin_config[] __initdata = {
	MFP_CFG_OUT(GPIO33, AF2, DRIVE_LOW),//GPIO33_A620_nUSB_PULL_UP,
};

int __init asus620_lcd_init(void)
{
    struct pxafb_mach_info *info;
    int ret;

    printk( "Registering Asus A620 FrameBuffer device\n");

    ret = gpio_request(GPIO74_A620_TCON_HERE_N, "TCON_HERE");
    if (ret)
    	pr_err("Error req TCON_HERE");
    ret = gpio_direction_input(GPIO74_A620_TCON_HERE_N);
    if (ret)
    	pr_err("Error ch direct TCON_HERE");

    has_tcon = gpio_get_value(GPIO74_A620_TCON_HERE_N);
    gpio_free(GPIO74_A620_TCON_HERE_N);


    if (has_tcon)
    {
	info = &asus620_fb_info_tcon;
	pxa2xx_mfp_config(ARRAY_AND_SIZE(tcon_pin_config));//	pxa_gpio_mode (GPIO75_A620_TCON_EN|GPIO_ALT_FN_2_OUT);
	printk(" TCON here\n");
    }
    else
    {
	info = &asus620_fb_info_notcon;
/*    ret = gpio_request(GPIO75_A620_TCON_EN, "TCON_EN");
    if (ret)
    	pr_err("Error req TCON_EN");
    ret = gpio_direction_output(GPIO75_A620_TCON_EN,1);
    if (ret)
    	pr_err("Error ch direct TCON_EN");
*/ //	pxa_gpio_mode (GPIO75_A620_TCON_EN|GPIO_OUT);
	printk(" TCON absent\n");
    }
    
    pxa_set_fb_info(NULL, info);

    printk(" LCD and backlight devices successfully registered\n");
    return 0;
}
/******************************************************************************
 * PCMCIA - CF
******************************************************************************/
static struct platform_device asus620_pcmcia_device = {
        .name           = "a620-pcmcia",
        .id             = -1,
        .dev            = {
//                .platform_data  = &asus620_pcmcia_info
        }
};

/******************************************************************************
 * TS - ad7873 on spi
******************************************************************************/
/*
static struct ads7846_platform_data __initdata ad7873_pdata = {
	.model		= 7873,		// AD7873 
	.x_max		= 0xfff,
	.y_max		= 0xfff,
	.x_plate_ohms	= 620,
	.debounce_max	= 1,
	.debounce_rep	= 0,
	.debounce_tol	= (~0),
	.gpio_pendown     = GPIO27_A620_STYLUS_IRQ,
//	.irq_flags	= IRQF_TRIGGER_RISING,
//	.get_pendown_state = ads7873_get_pendown_state,
};
*/
/*
static struct pxa2xx_spi_chip ads7873_chip = {
    .tx_threshold = 1,
    .rx_threshold = 2,
    .timeout      = 64,
//    .gpio_cs      = GPIO88_HX4700_TSC2046_CS,
};
*/
/*
static struct spi_board_info ads7873_board_info[] __initdata = {
    {
		.modalias = "ads7846",
		.max_speed_hz = 1200000,     // max spi clock (SCK) speed in HZ 204800
		.bus_num = 1,
		.irq = PXA_GPIO_TO_IRQ(GPIO27_A620_STYLUS_IRQ),
//		.chip_select = GPIO_PF10 + MAX_CTRL_CS,	// GPIO controlled SSEL 
//		.chip_select = 0,
		.platform_data = &ad7873_pdata,
//		.mode = SPI_MODE_0,
    },
};
*/
/*
static struct pxa2xx_spi_master pxa_ssp1_master_info = {
    .num_chipselect = 1,
    .clock_enable   = CKEN_SSP1,
//    .enable_dma     = 1,
};
*/

static struct platform_device a620_ts_device = {
	.name			= "a620-ts",
	.id			= -1,
};

/******************************************************************************
 * LEDs
******************************************************************************/

static struct platform_device a620_leds_device = {
	.name			= "a620-led",
	.id			= -1,
//	.dev.platform_data	= ,
};


/******************************************************************************
 * Bluetooth
******************************************************************************/

static struct platform_device a620_bt_pwr_device = {
	.name			= "a620_bt_power",
	.id			= -1,
//	.dev.platform_data	= ,
};


/*****************************************************
 *
 * Devices
 *
 *****************************************************/

// DoC
/*
static struct resource mtd_resources[]=
{
    [0] = {
	.start = 0x00000000,
	.end   = 0x00008000,
	.flags = IORESOURCE_MEM,
    }
};

static struct platform_device mtd_device =
{
    .name          = "mtd",
    .id            = -1,
    .resource      = mtd_resources,
    .num_resources = ARRAY_SIZE(mtd_resources),
};
*/
/*****************************************************
 *
 * Additional IO mapping
 *
 *****************************************************/
/*
static struct map_desc doc_io_desc[] __initdata = {
    { 0xef800000, 0x00000000, 0x00002000, MT_DEVICE }, // DiskOnChip Millenium +
};

static void __init asus620_map_io(void)
{
//    pxa_map_io();
    pxa25x_map_io();
//    iotable_init(doc_io_desc,ARRAY_SIZE(doc_io_desc));
}
*/
/******************************************************************************
 * PM.
 ******************************************************************************/
#ifdef CONFIG_PM
static u32 *addr_a2000400;
static u32 *addr_a2000404;
static u32 *addr_a2000408;
static u32 *addr_a200040c;
static u32 save_a2000400;
static u32 save_a2000404;
static u32 save_a2000408;
static u32 save_a200040c;


static int a620_cpu_suspend(void)
{
    printk("A620 suspend prepare\n");
    asus620_lcd_set_power (FB_BLANK_POWERDOWN);

//    asus620_gpo_suspend();

    PGSR0 = 0x00020000;			// Sleep GPIO 17 : GPIO_NR_A620_CF_ENABLE
    PGSR1 = 0x00700000;			// 52, 53, 54 : CF
    PGSR2 = 0x00010000;			// 80 : nCS[4] SRAM

//	PGSR0 = 0x00080000;			// GPIO(19)    : CF related to sleep mode
//	PGSR1 = 0x03ff0000;			// GPIO(48-57) : CF by PXA memory controller
//	PGSR2 = 0x00010400;			// GPIO(74,80) : LCD enable control, nCS[4] bank 4 SRAM (GPO)

    PWER = PWER_RTC | PWER_GPIO0;		// Alarm or power button
    PFER = PWER_GPIO0;
    PRER = 0;
    PEDR = 0;

    PCFR = PCFR_OPDE;


    save_a2000400 = *addr_a2000400;
    save_a2000404 = *addr_a2000404;
    save_a2000408 = *addr_a2000408;
    save_a200040c = *addr_a200040c;

    *addr_a2000400 = 0xe3a00101; // mov r0, #0x40000000
    *addr_a2000404 = 0xe380060f; // orr r0, r0, #0x0f000000
    *addr_a2000408 = 0xe3800008; // orr r0, r0, #8
    *addr_a200040c = 0xe590f000; // ldr pc, [r0]
/*
    *addr_a2000400 = 0xe3a00201;
    *(addr_a2000400+4) = 0xe3a01000;
    *(addr_a2000400+8) = 0xe1c010b0;
    *(addr_a2000400+12) = 0xe3a02401;
    *(addr_a2000400+16) = 0xe2522001;
    *(addr_a2000400+20) = 0x1afffffd;

    *(addr_a2000400+24) = 0xe3e01000;
    *(addr_a2000400+28) = 0xe1c010b0;
    *(addr_a2000400+32) = 0xe3a02401;
    *(addr_a2000400+36) = 0xe2522001;
    *(addr_a2000400+40) = 0x1afffffd;
    *(addr_a2000400+44) = 0xeafffff3;
*/
    //*addr_a2000404 = 0xe3800008; // orr r0, r0, #8
    //*addr_a2000408 = 0xe590f000; // ldr pc, [r0]

    return 0;
}

static void a620_cpu_resume(void)
{
    *addr_a2000400 = save_a2000400;
    *addr_a2000404 = save_a2000404;
    *addr_a2000408 = save_a2000408;
    *addr_a200040c = save_a200040c;

//    asus620_lcd_init();
    asus620_gpo_resume();
asus620_gpo_set(GPO_A620_BLUE_LED);
    asus620_lcd_set_power (FB_BLANK_UNBLANK);
    printk("A620 resume\n");

}

static struct syscore_ops a620_cpu_syscore_ops = {
	.suspend	= a620_cpu_suspend,
	.resume		= a620_cpu_resume,
};
#endif
/*
static void a620_poweroff(void)
{
    pxa_restart('g', NULL);
}
*/
/*
static void a620_restart(char mode, const char *cmd)
{
	void *ptr = phys_to_virt(0xa0008000);
	asm volatile(
	"ldr pc, [%0]				\n\
	"
	: "=r" (ptr)
	:
	: );

}
*/
/******************************************************************************
 * Machine init
 ******************************************************************************/
static struct platform_device *devices[] __initdata = {
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
	&asus620_pxa_keys,
#endif
	&asus620_backlight,
	&power_supply,
	&asus620_pcmcia_device,
//	&mtd_device,
	&audio,
	&a620_leds_device,
	&a620_ts_device,
//	&a620_bt_pwr_device,
	&asus620_gpio_vbus,
};
/*
static unsigned long asus620_ffuart_conf[] __initdata = {

	// FFUART 
	GPIO34_FFUART_RXD,
	GPIO39_FFUART_TXD,

};
*/
static void __init asus620_init(void)
{
//    static char cmd_line[COMMAND_LINE_SIZE];
//    void *adr1 = (void *)0xEF800000;
    printk("A620 init\n");
//    printk("DoC addr:%X",virt_to_phys(adr1));

    GPDR0 = 0xD38B6000;
    GPDR1 = 0xFCFFA881;
    GPDR2 = 0x0001FBFF;

    GAFR0_L = 0x00000004;
    GAFR0_U = 0x001A8002;
    GAFR1_L = 0x69908010;
    GAFR1_U = 0xAAA58AAA;

    if (GPLR2 & 0x400)
	GAFR2_L = 0x0A8AAAAA;
    else
	GAFR2_L = 0x0A0AAAAA;
    GAFR2_U = 0x00000002;


    pxa_register_device(&pxa_device_gpio, NULL);
    pxa2xx_mfp_config(ARRAY_AND_SIZE(asus620_pin_config));
    asus620_gpo_init();

/*    strlcpy(cmd_line, boot_command_line, COMMAND_LINE_SIZE);
    if(strcmp(cmd_line,"console=ttyS0")){
	pxa2xx_mfp_config(ARRAY_AND_SIZE(asus620_ffuart_conf));
        pxa_set_ffuart_info(NULL);
    }
*/
    pxa_set_ffuart_info(NULL);
    pxa_set_btuart_info(NULL);
    pxa_set_stuart_info(NULL);

    asus620_lcd_init();
    asus620_lcd_set_power (FB_BLANK_UNBLANK);
//	pxa_set_mci_info(&asus620_mci_platform_data);

    pxa_set_ficp_info(&asus620_ficp_platform_data);

//asus620_gpo_set(GPO_A620_BLUE_LED);

//    pxa27x_set_i2c_power_info(NULL);
    pxa_set_i2c_info(NULL);
    i2c_register_board_info(0, ARRAY_AND_SIZE(i2c_board_info));

//    pxa2xx_set_spi_info(1, &pxa_ssp1_master_info);
//    spi_register_board_info(ARRAY_AND_SIZE(ads7873_board_info));

//    set_irq_type(PXA_GPIO_TO_IRQ(GPIO27_A620_STYLUS_IRQ), IRQT_FALLING);
//    irq_set_irq_type(gpio_to_irq(GPIO27_A620_STYLUS_IRQ), IRQ_TYPE_EDGE_FALLING);

//asus620_gpo_set(GPO_A620_RED_LED);

//	CKEN |= (1<<CKEN_I2S);
//	CKEN &= ~(1<<CKEN_AC97);

//    init_gpio_reset(GPIO1_A620_RESET_BUTTON_N, 0, 0);
//    pm_power_off = a620_poweroff;

#ifdef CONFIG_PM
    addr_a2000400 = phys_to_virt(0xa2000400);
    addr_a2000404 = phys_to_virt(0xa2000404);
    addr_a2000408 = phys_to_virt(0xa2000408);
    addr_a200040c = phys_to_virt(0xa200040c);

    register_syscore_ops(&a620_cpu_syscore_ops);
#endif
    platform_add_devices(devices, ARRAY_SIZE(devices));

}

MACHINE_START(A620, "ASUS a620")
//    .restart_mode	= 'g',
//    .restart	= a620_restart,


	.atag_offset	= 0x100,
	.map_io		= pxa25x_map_io,
//	.map_io  	= asus620_map_io,
	.nr_irqs	= PXA_NR_IRQS,
	.init_irq	= pxa25x_init_irq,
	.handle_irq	= pxa25x_handle_irq,
	.timer		= &pxa_timer,
	.init_machine	= asus620_init,
	.restart	= pxa_restart,
MACHINE_END
