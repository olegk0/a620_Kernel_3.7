/*
 * Defintions for ASUS MyPal A620 device
 *
 * Authors:
 *	Jamey Hicks, Michael Opdenacker, Koen Kooi
 *
 * Copyright (C) Lukasz Dalek <luk0104@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __ASM_ARCH_A620_H
#define __ASM_ARCH_A620_H

/*
 * arch/arm/include/asm/hardware/h2200.h
 * History:
 *
 * 2003-12-08 Jamey Hicks		Copied over h22xx gpio definitions
 *					and beginnings of h22xx (hamcop)
 *					asic declarations
 * 2004-01-26 Michael Opdenacker	Replaced "_H5200_GPIO_H_" by
 *					"_A620_GPIO_H_"
 *					Added definition for
 *					IRQ_GPIO_A620_ASIC_IN
 *					(like in h5400-gpio.h)
 * 2004-02-05 Koen Kooi			Added definition for
 *					IRQ_NR_A620_CF_DETECT_N
 * 2012-11-29 Lukasz Dalek		Renamed pins, added some
 *					IRQ and MEM defintions
 */

#define GPIO0_A620_POWER_BUTTON_N		0
#define GPIO1_A620_RESET_BUTTON_N		1
#define GPIO2_A620_HOME_BUTTON_N		2
#define GPIO3_A620_CALENDAR_BUTTON_N	3
#define GPIO4_A620_CONTACTS_BUTTON_N	4
#define GPIO5_A620_TASKS_BUTTON_N		5
/* 6 missing */

/* hand probed */
#define GPIO7_A620_CF_IRQ			(7)
/* hand probed */
#define GPIO8_A620_CF_BVD1			(8)

/* Not sure, just a guess */
#define GPIO9_A620_CF_READY_N			(9)

#define GPIO10_A620_USB_DETECT_N		(10)
#define GPIO11_A620_RECORD_BUTTON_N		(11)
#define GPIO12_A620_HEARPHONE_N			(12)
/* 13 missing (output) */
#define GPIO13_A620_CF_RESET			(13)
/* hand probed */
#define GPIO14_A620_CF_POWER			(14)
#define GPIO15_A620_COM_DETECT_N		(15)
#define GPIO16_A620_BACKLIGHT			(16)
/* Not sure, guessed */
#define GPIO17_A620_CF_ENABLE			(17)


/*
 * 18 Power detect, see below
 * 19 missing (output)
 *
 * 18,20
 * going 0/1 with Power
 * input
 * 18: Interrupt on none
 * 20: Interrupt on RE/FE
 *
 * Putting GPIO_NR_A620_AC_IN on 20 is arbitrary
 */
#define GPIO20_A620_AC_IN			(20)

/* hand probed */
#define GPIO21_A620_CF_VS1_N			(21)
//#define GPIO21_A620_CF_DETECT_N			(21)
#define GPIO22_A620_CF_VS2_N			(22)

/* 23-26 is SSP to AD7873 */
#define GPIO27_A620_STYLUS_IRQ			(27)

/*
 * 28-32 IIS to UDA1380
 *
 * 33 USB pull-up
 * Must be input when disconnected (floating)
 * Must be output set to 1 when connected
 */
#define GPIO33_A620_USB_PULL_UP_N		(33)

/*
 * Serial connector has no handshaking
 * 34,39 Serial port
 *
 * 38 missing
 *
 * Joystick directions and button
 * Center is 35
 * Directions triggers one or two of 36,67,40,41
 *
 *   36   36,37     37
 *
 *  36,41   35    37,40
 *
 *   41   40,41     40
 */

#define GPIO35_A620_JOY_PUSH_N			(35)
#define GPIO36_A620_JOY_NW_N			(36)
#define GPIO37_A620_JOY_NE_N			(37)
#define GPIO40_A620_JOY_SE_N			(40)
#define GPIO41_A620_JOY_SW_N			(41)

#define GPIO_A620_JOY_DIR	(((GPLR1&0x300)>>6)|((GPLR1&0x30)>>4))

/*
 * 42-45 Bluetooth
 * 46-47 IrDA
 *
 * 48-57 Card ?
 * 54 is led control
 */
#define GPIO54_A620_LED_ENABLE			(54)

/*
 *
 * 58 through 77 is LCD signals
 * 74,75 seems related to TCON chip
 *  -74 is TCON presence probing
 *  -75 is set to GAFR2 when TCON is here
 */

#define GPIO74_A620_TCON_HERE_N		(74)
#define GPIO75_A620_TCON_EN			(75)

/*
 * Power management (scaling to 200, 300, 400MHz)
 * Chart is:
 *     78 79
 * 200  1  0
 * 300  0  1
 * 400  1  1
 */
#define GPIO78_A620_PWRMGMT0			(78)
#define GPIO79_A620_PWRMGMT1			(79)

/*
 * 81-84 missing
 */



/*
 * Other infos
 */

#define ASUS_IDE_BASE 			0x00000000
#define ASUS_IDE_SIZE  			0x04000000
#define ASUS_IDE_VIRTUAL		0xF8000000

#define GPO_A620_USB			(1 <<  0) // 0x00000001
#define GPO_A620_LCD_POWER1		(1 <<  1) // 0x00000002
#define GPO_A620_LCD_POWER2		(1 <<  2) // 0x00000004
#define GPO_A620_LCD_POWER3		(1 <<  3) // 0x00000008
#define GPO_A620_BACKLIGHT		(1 <<  4) // 0x00000010
#define GPO_A620_BLUETOOTH		(1 <<  5) // 0x00000020
#define GPO_A620_MICROPHONE		(1 <<  6) // 0x00000040
#define GPO_A620_SOUND			(1 <<  7) // 0x00000080
#define GPO_A620_UNK_1			(1 <<  8) // 0x00000100
#define GPO_A620_BLUE_LED		(1 <<  9) // 0x00000200
#define GPO_A620_UNK_2			(1 << 10) // 0x00000400
#define GPO_A620_RED_LED		(1 << 11) // 0x00000800
#define GPO_A620_UNK_3			(1 << 12) // 0x00001000
#define GPO_A620_CF_RESET		(1 << 13) // 0x00002000
#define GPO_A620_IRDA_FIR_MODE		(1 << 14) // 0x00004000
#define GPO_A620_IRDA_POWER_N		(1 << 15) // 0x00008000

#define GPIO_I2C_EARPHONES_DETECTED       (1 << 2) // 0x00000004


void asus620_gpo_set(unsigned long bits);
void asus620_gpo_clear(unsigned long bits);
void asus620_gpo_init(void);
void asus620_gpo_suspend(void);
void asus620_gpo_resume(void);

/*
 * General Purpose I/O
 */

#define GPLR0		__REG(0x40E00000)  /* GPIO Pin-Level Register GPIO<31:0> */
#define GPLR1		__REG(0x40E00004)  /* GPIO Pin-Level Register GPIO<63:32> */
#define GPLR2		__REG(0x40E00008)  /* GPIO Pin-Level Register GPIO<80:64> */

#define GPDR0		__REG(0x40E0000C)  /* GPIO Pin Direction Register GPIO<31:0> */
#define GPDR1		__REG(0x40E00010)  /* GPIO Pin Direction Register GPIO<63:32> */
#define GPDR2		__REG(0x40E00014)  /* GPIO Pin Direction Register GPIO<80:64> */

#define GPSR0		__REG(0x40E00018)  /* GPIO Pin Output Set Register GPIO<31:0> */
#define GPSR1		__REG(0x40E0001C)  /* GPIO Pin Output Set Register GPIO<63:32> */
#define GPSR2		__REG(0x40E00020)  /* GPIO Pin Output Set Register GPIO<80:64> */

#define GPCR0		__REG(0x40E00024)  /* GPIO Pin Output Clear Register GPIO<31:0> */
#define GPCR1		__REG(0x40E00028)  /* GPIO Pin Output Clear Register GPIO <63:32> */
#define GPCR2		__REG(0x40E0002C)  /* GPIO Pin Output Clear Register GPIO <80:64> */

#define GRER0		__REG(0x40E00030)  /* GPIO Rising-Edge Detect Register GPIO<31:0> */
#define GRER1		__REG(0x40E00034)  /* GPIO Rising-Edge Detect Register GPIO<63:32> */
#define GRER2		__REG(0x40E00038)  /* GPIO Rising-Edge Detect Register GPIO<80:64> */

#define GFER0		__REG(0x40E0003C)  /* GPIO Falling-Edge Detect Register GPIO<31:0> */
#define GFER1		__REG(0x40E00040)  /* GPIO Falling-Edge Detect Register GPIO<63:32> */
#define GFER2		__REG(0x40E00044)  /* GPIO Falling-Edge Detect Register GPIO<80:64> */

#define GEDR0		__REG(0x40E00048)  /* GPIO Edge Detect Status Register GPIO<31:0> */
#define GEDR1		__REG(0x40E0004C)  /* GPIO Edge Detect Status Register GPIO<63:32> */
#define GEDR2		__REG(0x40E00050)  /* GPIO Edge Detect Status Register GPIO<80:64> */

#define GAFR0_L		__REG(0x40E00054)  /* GPIO Alternate Function Select Register GPIO<15:0> */
#define GAFR0_U		__REG(0x40E00058)  /* GPIO Alternate Function Select Register GPIO<31:16> */
#define GAFR1_L		__REG(0x40E0005C)  /* GPIO Alternate Function Select Register GPIO<47:32> */
#define GAFR1_U		__REG(0x40E00060)  /* GPIO Alternate Function Select Register GPIO<63:48> */
#define GAFR2_L		__REG(0x40E00064)  /* GPIO Alternate Function Select Register GPIO<79:64> */
#define GAFR2_U		__REG(0x40E00068)  /* GPIO Alternate Function Select Register GPIO<95-80> */
#define GAFR3_L		__REG(0x40E0006C)  /* GPIO Alternate Function Select Register GPIO<111:96> */
#define GAFR3_U		__REG(0x40E00070)  /* GPIO Alternate Function Select Register GPIO<127:112> */

#define GPLR3		__REG(0x40E00100)  /* GPIO Pin-Level Register GPIO<127:96> */
#define GPDR3		__REG(0x40E0010C)  /* GPIO Pin Direction Register GPIO<127:96> */
#define GPSR3		__REG(0x40E00118)  /* GPIO Pin Output Set Register GPIO<127:96> */
#define GPCR3		__REG(0x40E00124)  /* GPIO Pin Output Clear Register GPIO<127:96> */
#define GRER3		__REG(0x40E00130)  /* GPIO Rising-Edge Detect Register GPIO<127:96> */
#define GFER3		__REG(0x40E0013C)  /* GPIO Falling-Edge Detect Register GPIO<127:96> */
#define GEDR3		__REG(0x40E00148)  /* GPIO Edge Detect Status Register GPIO<127:96> */

#endif /* __ASM_ARCH_A620_H */
