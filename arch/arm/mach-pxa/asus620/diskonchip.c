/*
 * drivers/mtd/nand/diskonchip.c
 *
 * (C) 2003 Red Hat, Inc.
 * (C) 2004 Dan Brown <dan_brown@ieee.org>
 * (C) 2004 Kalev Lember <kalev@smartlink.ee>
 * (C) 2005 Raphael Zimmerer <killekulla@rdrz.de>
 
* 2014 olegvedi@gmail.ru some changes for ASUS a620 support in RO mode only
 *
 * Author: David Woodhouse <dwmw2@infradead.org>
 * Additional Diskonchip 2000 and Millennium support by Dan Brown <dan_brown@ieee.org>
 * Diskonchip Millennium Plus 16 MB support by Kalev Lember <kalev@smartlink.ee>
 * Diskonchip Millennium Plus 32 MB support by Raphael Zimmerer <killekulla@rdrz.de>
 *                                         and Anti Sullin <anti.sullin@artecdesign.ee>
 *
 * Error correction code lifted from the old docecc code
 * Author: Fabrice Bellard (fabrice.bellard@netgem.com)
 * Copyright (C) 2000 Netgem S.A.
 * converted to the generic Reed-Solomon library by Thomas Gleixner <tglx@linutronix.de>
 *
 * Interface to generic NAND code for M-Systems DiskOnChip devices
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/rslib.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <asm/io.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/doc2000.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/inftl.h>
#include <linux/module.h>


//#define DEBUG

/* Where to look for the devices? */
#ifndef CONFIG_MTD_NAND_DISKONCHIP_PROBE_ADDRESS
#define CONFIG_MTD_NAND_DISKONCHIP_PROBE_ADDRESS 0x800
#endif

static unsigned long doc_locations[] __initdata = {
	0x800 };

static struct mtd_info *doclist = NULL;

struct doc_priv {
	void __iomem *virtadr;
	unsigned long physadr;
	u_char ChipID;
	u_char CDSNControl;
	u_char interleave; /* Internal interleaving, for Millennium Plus chips */
	u_char interface_16; /* Interface setting for Millennium Plus chips */
	int chips_per_floor;	/* The number of chips detected on each floor */
	int curfloor;
	int curchip;
	int mh0_page;
	int mh1_page;
	struct mtd_info *nextdoc;
};

/* This is the syndrome computed by the HW ecc generator upon reading an empty
   page, one with all 0xff for data and stored ecc code. */
static u_char empty_read_syndrome[6] = { 0x26, 0xff, 0x6d, 0x47, 0x73, 0x7a };

/* This is the ecc value computed by the HW ecc generator upon writing an empty
   page, one with all 0xff for data. */
static u_char empty_write_ecc[6] = { 0x4b, 0x00, 0xe2, 0x0e, 0x93, 0xf7 };

#define INFTL_BBT_RESERVED_BLOCKS 4

static void doc200x_hwcontrol(struct mtd_info *mtd, int cmd,
			      unsigned int bitmask);

static int no_ecc_failures = 0;
module_param(no_ecc_failures, int, 0);

static int no_autopart = 1;
module_param(no_autopart, int, 0);

static int show_firmware_partition = 0;
module_param(show_firmware_partition, int, 0);

#ifdef CONFIG_MTD_NAND_DISKONCHIP_BBTWRITE
static int inftl_bbt_write = 1;
#else
static int inftl_bbt_write = 0;
#endif
module_param(inftl_bbt_write, int, 0);

static unsigned long doc_config_location = CONFIG_MTD_NAND_DISKONCHIP_PROBE_ADDRESS;
module_param(doc_config_location, ulong, 0);
MODULE_PARM_DESC(doc_config_location, "Physical memory address at which to probe for DiskOnChip");

/* Sector size for HW ECC */
#define SECTOR_SIZE 512
/* The sector bytes are packed into NB_DATA 10 bit words */
#define NB_DATA (((SECTOR_SIZE + 1) * 8 + 6) / 10)
/* Number of roots */
#define NROOTS 4
/* First consective root */
#define FCR 510
/* Number of symbols */
#define NN 1023


/**
 * Check if the opcode's address should be sent only on the lower 8 bits
 * @command: opcode to check
*/
static inline int nand_opcode_8bits(unsigned int command)
{
     switch (command) {
     case NAND_CMD_READID:
     case NAND_CMD_PARAM:
     case NAND_CMD_GET_FEATURES:
     case NAND_CMD_SET_FEATURES:
             return 1;
     default:
             break;
     }
     return 0;
}

/*
 * On 32MB DoC, the flash chips are interleaved bytewise and commands address both
 * chips at once. The resulting write block is twice the size. M-Sys is splitting
 * the block into two sub-blocks, both having separate ECCs. We are using it as one
 * block but calculate the ECCs separately and fill in the OOB data where needed.
 *       +-----------+-------+-------+-------+--------------+---------+-----------+
 * RAW   | 0 --- 511 |512-517|518-519|520-521| 522 --- 1033 |1034-1039|1040 - 1055|
 *       +-----------+-------+-------+-------+--------------+---------+-----------+
 *       | CMD_READ0 | CMD_READ1                       | CMD_READOOB              |
 *       +-----------+-------+-------+-------+--------------+---------+-----------+
 * M-Sys | Data 0    | ECC 0 |Flags0 |Flags1 | Data 1       | ECC 1   | OOB 1 + 2 |
 *       +-----------+-------+-------+-------+--------------+---------+-----------+
 * Linux | Data 0    | ECC 0 | OOB0          | Data 1       | ECC 1   | OOB1      |
 *       +-----------+-------+-------+-------+--------------+---------+-----------+
 */

/* the Reed Solomon control structure */
static struct rs_control *rs_decoder;

/*
 * The HW decoder in the DoC ASIC's provides us a error syndrome,
 * which we must convert to a standard syndrome usable by the generic
 * Reed-Solomon library code.
 *
 * Fabrice Bellard figured this out in the old docecc code. I added
 * some comments, improved a minor bit and converted it to make use
 * of the generic Reed-Solomon library. tglx
 */
static int doc_ecc_decode(struct rs_control *rs, uint8_t *data, uint8_t *ecc)
{
	int i, j, nerr, errpos[8];
	uint8_t parity;
	uint16_t ds[4], s[5], tmp, errval[8], syn[4];

#ifdef DEBUG
		printk("doc_ecc_decode\n");
#endif

	memset(syn, 0, sizeof(syn));
	/* Convert the ecc bytes into words */
	ds[0] = ((ecc[4] & 0xff) >> 0) | ((ecc[5] & 0x03) << 8);
	ds[1] = ((ecc[5] & 0xfc) >> 2) | ((ecc[2] & 0x0f) << 6);
	ds[2] = ((ecc[2] & 0xf0) >> 4) | ((ecc[3] & 0x3f) << 4);
	ds[3] = ((ecc[3] & 0xc0) >> 6) | ((ecc[0] & 0xff) << 2);
	parity = ecc[1];

	/* Initialize the syndrome buffer */
	for (i = 0; i < NROOTS; i++)
		s[i] = ds[0];
	/*
	 *  Evaluate
	 *  s[i] = ds[3]x^3 + ds[2]x^2 + ds[1]x^1 + ds[0]
	 *  where x = alpha^(FCR + i)
	 */
	for (j = 1; j < NROOTS; j++) {
		if (ds[j] == 0)
			continue;
		tmp = rs->index_of[ds[j]];
		for (i = 0; i < NROOTS; i++)
			s[i] ^= rs->alpha_to[rs_modnn(rs, tmp + (FCR + i) * j)];
	}

	/* Calc syn[i] = s[i] / alpha^(v + i) */
	for (i = 0; i < NROOTS; i++) {
		if (s[i])
			syn[i] = rs_modnn(rs, rs->index_of[s[i]] + (NN - FCR - i));
	}
	/* Call the decoder library */
	nerr = decode_rs16(rs, NULL, NULL, 1019, syn, 0, errpos, 0, errval);

	/* Incorrectable errors ? */
	if (nerr < 0)
		return nerr;

	/*
	 * Correct the errors. The bitpositions are a bit of magic,
	 * but they are given by the design of the de/encoder circuit
	 * in the DoC ASIC's.
	 */
	for (i = 0; i < nerr; i++) {
		int index, bitpos, pos = 1015 - errpos[i];
		uint8_t val;
		if (pos >= NB_DATA && pos < 1019)
			continue;
		if (pos < NB_DATA) {
			/* extract bit position (MSB first) */
			pos = 10 * (NB_DATA - 1 - pos) - 6;
			/* now correct the following 10 bits. At most two bytes
			   can be modified since pos is even */
			index = (pos >> 3) ^ 1;
			bitpos = pos & 7;
			if ((index >= 0 && index < SECTOR_SIZE) || index == (SECTOR_SIZE + 1)) {
				val = (uint8_t) (errval[i] >> (2 + bitpos));
				parity ^= val;
				if (index < SECTOR_SIZE)
					data[index] ^= val;
			}
			index = ((pos >> 3) + 1) ^ 1;
			bitpos = (bitpos + 10) & 7;
			if (bitpos == 0)
				bitpos = 8;
			if ((index >= 0 && index < SECTOR_SIZE) || index == (SECTOR_SIZE + 1)) {
				val = (uint8_t) (errval[i] << (8 - bitpos));
				parity ^= val;
				if (index < SECTOR_SIZE)
					data[index] ^= val;
			}
		}
	}
	/* If the parity is wrong, no rescue possible */
	return parity ? -EBADMSG : nerr;
}

static void DoC_Delay(struct doc_priv *doc, unsigned short cycles)
{
	volatile char dummy;
	int i;

	for (i = 0; i < cycles; i++) {
		dummy = ReadDOC(doc->virtadr, Mplus_NOP);
	}

}

#define CDSN_CTRL_FR_B_MASK	(CDSN_CTRL_FR_B0 | CDSN_CTRL_FR_B1)

/* DOC_WaitReady: Wait for RDY line to be asserted by the flash chip */
static int _DoC_WaitReady(struct doc_priv *doc)
{
	void __iomem *docptr = doc->virtadr;
	unsigned long timeo = jiffies + (HZ * 10);

#ifdef DEBUG
		printk("_DoC_WaitReady...\n");
#endif
	/* Out-of-line routine to wait for chip response */
	while ((ReadDOC(docptr, Mplus_FlashControl) & CDSN_CTRL_FR_B_MASK) != CDSN_CTRL_FR_B_MASK) {
		if (time_after(jiffies, timeo)) {
			printk("_DoC_WaitReady timed out.\n");
			return -EIO;
		}
		udelay(1);
		cond_resched();
	}

	return 0;
}

static inline int DoC_WaitReady(struct doc_priv *doc)
{
	void __iomem *docptr = doc->virtadr;
	int ret = 0;

	DoC_Delay(doc, 4);

	if ((ReadDOC(docptr, Mplus_FlashControl) & CDSN_CTRL_FR_B_MASK) != CDSN_CTRL_FR_B_MASK)
		/* Call the out-of-line routine to wait */
		ret = _DoC_WaitReady(doc);
#ifdef DEBUG
		printk("DoC_WaitReady OK\n");
#endif
	return ret;
}

static int doc200x_wait(struct mtd_info *mtd, struct nand_chip *this)
{
	struct doc_priv *doc = this->priv;

	int status;

	DoC_WaitReady(doc);
	this->cmdfunc(mtd, NAND_CMD_STATUS, -1, -1);
	DoC_WaitReady(doc);
	status = (int)this->read_byte(mtd);

	return status;
}

static void doc2001_write_byte(struct mtd_info *mtd, u_char datum)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = this->priv;
	void __iomem *docptr = doc->virtadr;

	WriteDOC(datum, docptr, CDSNSlowIO);
	WriteDOC(datum, docptr, Mil_CDSN_IO);
	WriteDOC(datum, docptr, WritePipeTerm);
}

static u_char doc2001plus_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = this->priv;
	void __iomem *docptr = doc->virtadr;
	u_char ret;

	ReadDOC(docptr, Mplus_ReadPipeInit);
	ReadDOC(docptr, Mplus_ReadPipeInit);
	ret = ReadDOC(docptr, Mplus_LastDataRead);
#ifdef DEBUG
		printk("read_byte returns %02x\n", ret);
#endif
	return ret;
}

//IAM for 16bit wide operation
static void doc2001plus_writebuf(struct mtd_info *mtd, const u_char *buf, int len)
{
/*
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = this->priv;
	void __iomem *docptr = doc->virtadr;
	int i;

#ifdef DEBUG
		printk("writebuf of %d bytes: ", len);
#endif
	len >>=1;
	for (i = 0; i < len; i++) {
		WriteDOC16_(((ushort *)buf)[i], docptr, DoC_Mil_CDSN_IO);
#ifdef DEBUG
		if (i < 16)
			printk("%02x ", buf[i]);
#endif
	}
#ifdef DEBUG
		printk("\n");
#endif
*/
}

static void doc2001plus_readbuf(struct mtd_info *mtd, u_char *buf, int len)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = this->priv;
	void __iomem *docptr = doc->virtadr;
	int i;

#ifdef DEBUG
	int k=len;
	printk("readbuf of %d bytes: ", len);
#endif
	len >>=1;
	for (i=0; i < len; i++) {
		((ushort *)buf)[i] = ReadDOC16_(docptr, DoC_Mil_CDSN_IO);
	}

#ifdef DEBUG
	if(k>16) k=16;
	for (i=0; i < k; i++) {
	    printk("%02x ", buf[i]);
	}

	if(len==5 && doc->curfloor){
	    for (i=0; i < 3; i++) {
		k = ReadDOC16_(docptr, DoC_Mil_CDSN_IO);
		printk("%02x %02x ", k&0xff,(k>>8)&0xff);
	    }
	}
	printk("\n");
#endif
}

static void doc2001plus_select_chip(struct mtd_info *mtd, int chip)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = this->priv;
	void __iomem *docptr = doc->virtadr;


#ifdef DEBUG
		printk("select chip (%d)\n", chip);
#endif
	if (chip == -1) {
		/* Disable flash internally */
		WriteDOC(0, docptr, Mplus_FlashSelect);
		return;
	}

	chip &= 0x3;

	WriteDOC(chip, docptr, Mplus_DeviceSelect);

	/* Assert ChipEnable and deassert WriteProtect */
//	WriteDOC((DOC_FLASH_CE), docptr, Mplus_FlashSelect);
	WriteDOC((DOC_FLASH_CE | DOC_FLASH_WP ), docptr, Mplus_FlashSelect);
	this->cmdfunc(mtd, NAND_CMD_RESET, -1, -1);

	doc->curchip = chip;

}

#define CDSN_CTRL_MSK (CDSN_CTRL_CE | CDSN_CTRL_CLE | CDSN_CTRL_ALE)

static void doc200x_hwcontrol(struct mtd_info *mtd, int cmd,
			      unsigned int ctrl)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = this->priv;
	void __iomem *docptr = doc->virtadr;

	if (ctrl & NAND_CTRL_CHANGE) {
		doc->CDSNControl &= ~CDSN_CTRL_MSK;
		doc->CDSNControl |= ctrl & CDSN_CTRL_MSK;
#ifdef DEBUG
		printk("hwcontrol(%d): %02x\n", cmd, doc->CDSNControl);
#endif
		WriteDOC(doc->CDSNControl, docptr, CDSNControl);
		/* 11.4.3 -- 4 NOPs after CSDNControl write */
		DoC_Delay(doc, 4);
	}
	if (cmd != NAND_CMD_NONE) {
		doc2001_write_byte(mtd, cmd);
	}
}

static void doc2001plus_command(struct mtd_info *mtd, unsigned command, int column, int page_addr)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = this->priv;
	void __iomem *docptr = doc->virtadr;
//	int readcmd;
//	int pa;

	doc->curfloor = page_addr&1;

	/*
	 * Write out the command to the device.
	 */
	switch (command) {
	case NAND_CMD_PAGEPROG:
		/*
		 * Must terminate write pipeline before sending any commands
		 * to the device.
		 */
		WriteDOC(0x00, docptr, Mplus_WritePipeTerm);
		WriteDOC(0x00, docptr, Mplus_WritePipeTerm);
		break;
/*
	case NAND_CMD_SEQIN:
		if (column >= mtd->writesize) {
			// OOB area 
			column -= mtd->writesize;
			readcmd = NAND_CMD_READOOB;
		} else if ((column >> doc->interleave) < 256) {
			// First (256 >> interleave) bytes: READ0 
			readcmd = NAND_CMD_READ0;
		} else {
			// READ1 
			column -= (256 << doc->interleave);
			readcmd = NAND_CMD_READ1;
		}
		WriteDOC(readcmd, docptr, Mplus_FlashCmd);
		break;
*/
	case NAND_CMD_READ0:

	    if(doc->curfloor){
		command = NAND_CMD_READ1;
		column = 5;
	    }
/*		if (column >= mtd->writesize) {
			// OOB area 
			column -= mtd->writesize;
			command = NAND_CMD_READOOB;
		} else if ((column >> doc->interleave) >= 256) {
			// READ1 
			column -= (256 << doc->interleave);
			command = NAND_CMD_READ1;
		}
*/
		break;
	case NAND_CMD_READOOB:
	    column = 8;

		break;

	}

#ifdef DEBUG
	    printk("CMD:0x%X column:%d, page_addr:%d(0x%X) HWpage_addr:%d\n",command,column, page_addr, page_addr*512, page_addr>>1);
#endif

	WriteDOC(command, docptr, Mplus_FlashCmd);
	WriteDOC(0, docptr, Mplus_WritePipeTerm);
	WriteDOC(0, docptr, Mplus_WritePipeTerm);

	if (column != -1) {
		/* Serially input address */
//		if (column != -1) {
			/* Adjust columns for 16 bit buswidth */
//IAM from patch			if (this->options & NAND_BUSWIDTH_16 &&
//					!nand_opcode_8bits(command))
//				column >>= 1;
	    /* Adjust columns for interleaved operation */
//			column >>= doc->interleave;

			WriteDOC(column, docptr, Mplus_FlashAddress);
//		}

		if (page_addr != -1) {

			page_addr >>= 1;

			WriteDOC((unsigned char)(page_addr & 0xff), docptr, Mplus_FlashAddress);
			WriteDOC((unsigned char)((page_addr >> 8) & 0xff), docptr, Mplus_FlashAddress);
		}
		WriteDOC(0, docptr, Mplus_WritePipeTerm);
		WriteDOC(0, docptr, Mplus_WritePipeTerm);
	}

	/*
	 * program and erase have their own busy handlers
	 * status and sequential in needs no delay
	 */
	switch (command) {

	case NAND_CMD_PAGEPROG:
	case NAND_CMD_ERASE1:
	case NAND_CMD_ERASE2:
	case NAND_CMD_SEQIN:
	case NAND_CMD_STATUS:
		return;

	case NAND_CMD_RESET:
		if (this->dev_ready)
			break;
		udelay(this->chip_delay);
		WriteDOC(NAND_CMD_STATUS, docptr, Mplus_FlashCmd);
		WriteDOC(0, docptr, Mplus_WritePipeTerm);
		WriteDOC(0, docptr, Mplus_WritePipeTerm);
		DoC_WaitReady(doc);
		return;

	case NAND_CMD_READID:
		DoC_WaitReady(doc);
		/* deassert ALE */
		WriteDOC(0, docptr, Mplus_FlashControl);
		WriteDOC(0, docptr, Mplus_NOP);
		WriteDOC(0, docptr, Mplus_NOP);
		return;

	case NAND_CMD_READ0:
	case NAND_CMD_READ1:
	case NAND_CMD_READOOB:
		DoC_WaitReady(doc);
		/* deassert ALE */
		WriteDOC(0, docptr, Mplus_FlashControl);
		WriteDOC(0, docptr, Mplus_NOP);
		WriteDOC(0, docptr, Mplus_NOP);

		/* Start read pipeline */
		ReadDOC(docptr, Mplus_ReadPipeInit);
		ReadDOC(docptr, Mplus_ReadPipeInit);
		return;

		/* This applies to read commands */
	default:
		/*
		 * If we don't have access to the busy pin, we apply the given
		 * command delay
		 */
		if (!this->dev_ready) {
			udelay(this->chip_delay);
			return;
		}
	}

	/* Apply this short delay always to ensure that we do wait tWB in
	 * any case on any machine. */
	ndelay(100);
	/* wait until command is processed */
	while (!this->dev_ready(mtd)) ;
}

static int doc200x_dev_ready(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = this->priv;
	void __iomem *docptr = doc->virtadr;

	/* 11.4.2 -- must NOP four times before checking FR/B# */
	DoC_Delay(doc, 4);
	if ((ReadDOC(docptr, Mplus_FlashControl) & CDSN_CTRL_FR_B_MASK) != CDSN_CTRL_FR_B_MASK) {
#ifdef DEBUG
	    printk("not ready\n");
#endif
		return 0;
	}
#ifdef DEBUG
	printk("was ready\n");
#endif
	return 1;
}

static int doc200x_block_bad(struct mtd_info *mtd, loff_t ofs, int getchip)
{
	/* This is our last resort if we couldn't find or create a BBT.  Just
	   pretend all blocks are good. */
	return 0;
}

static void doc2001plus_enable_hwecc(struct mtd_info *mtd, int mode)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = this->priv;
	void __iomem *docptr = doc->virtadr;

	/* Prime the ECC engine */
	switch (mode) {
	case NAND_ECC_READ:
		WriteDOC(DOC_ECC_RESET, docptr, Mplus_ECCConf);
		WriteDOC(DOC_ECC_EN, docptr, Mplus_ECCConf);
		break;
	case NAND_ECC_WRITE:
		WriteDOC(DOC_ECC_RESET, docptr, Mplus_ECCConf);
		WriteDOC(DOC_ECC_EN | DOC_ECC_RW, docptr, Mplus_ECCConf);
		break;
	}
}

/* This code is only called on write */
static int doc200x_calculate_ecc(struct mtd_info *mtd, const u_char *dat, unsigned char *ecc_code)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = this->priv;
	void __iomem *docptr = doc->virtadr;
	int i;
	int emptymatch = 1;


	/* flush the pipeline */
	WriteDOC(0, docptr, Mplus_NOP);
	WriteDOC(0, docptr, Mplus_NOP);
	WriteDOC(0, docptr, Mplus_NOP);

	for (i = 0; i < 6; i++) {
		ecc_code[i] = ReadDOC_(docptr, DoC_Mplus_ECCSyndrome0 + i);
		if (ecc_code[i] != empty_write_ecc[i])
			emptymatch = 0;
	}
	WriteDOC(DOC_ECC_DIS, docptr, Mplus_ECCConf);
#if 0
	/* If emptymatch=1, we might have an all-0xff data buffer.  Check. */
	if (emptymatch) {
		/* Note: this somewhat expensive test should not be triggered
		   often.  It could be optimized away by examining the data in
		   the writebuf routine, and remembering the result. */
		for (i = 0; i < 512; i++) {
			if (dat[i] == 0xff)
				continue;
			emptymatch = 0;
			break;
		}
	}
	/* If emptymatch still =1, we do have an all-0xff data buffer.
	   Return all-0xff ecc value instead of the computed one, so
	   it'll look just like a freshly-erased page. */
	if (emptymatch)
		memset(ecc_code, 0xff, 6);
#endif
	return 0;
}

static int doc200x_correct_data(struct mtd_info *mtd, u_char *dat,
				u_char *read_ecc, u_char *isnull)
{
	int i, ret = 0;
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = this->priv;
	void __iomem *docptr = doc->virtadr;
	uint8_t calc_ecc[6];
	volatile u_char dummy;
	int emptymatch = 1;

#ifdef DEBUG
	printk("correct_data\n");
#endif
	/* flush the pipeline */
	dummy = ReadDOC(docptr, Mplus_ECCConf);
	dummy = ReadDOC(docptr, Mplus_ECCConf);
	dummy = ReadDOC(docptr, Mplus_ECCConf);

	/* Error occurred ? */
	if (dummy & 0x80) {
		for (i = 0; i < 6; i++) {
			calc_ecc[i] = ReadDOC_(docptr, DoC_Mplus_ECCSyndrome0 + i);
			if (calc_ecc[i] != empty_read_syndrome[i])
				emptymatch = 0;
		}
		/* If emptymatch=1, the read syndrome is consistent with an
		   all-0xff data and stored ecc block.  Check the stored ecc. */
		if (emptymatch) {
			for (i = 0; i < 6; i++) {
				if (read_ecc[i] == 0xff)
					continue;
				emptymatch = 0;
				break;
			}
		}
		/* If emptymatch still =1, check the data block. */
		if (emptymatch) {
			/* Note: this somewhat expensive test should not be triggered
			   often.  It could be optimized away by examining the data in
			   the readbuf routine, and remembering the result. */
			for (i = 0; i < 512; i++) {
				if (dat[i] == 0xff)
					continue;
				emptymatch = 0;
				break;
			}
		}
		/* If emptymatch still =1, this is almost certainly a freshly-
		   erased block, in which case the ECC will not come out right.
		   We'll suppress the error and tell the caller everything's
		   OK.  Because it is. */
		if (!emptymatch)
			ret = doc_ecc_decode(rs_decoder, dat, calc_ecc);
		if (ret > 0)
			printk(KERN_ERR "doc200x_correct_data corrected %d errors\n", ret);
	}
	WriteDOC(DOC_ECC_DIS, docptr, Mplus_ECCConf);
	if (no_ecc_failures && mtd_is_eccerr(ret)) {
		printk(KERN_ERR "suppressing ECC failure\n");
		ret = 0;
	}
	return ret;
}

/* Override write_page_raw: write ECC bytes. */
static int doc2001plus_write_page_raw_syndrome(struct mtd_info *mtd, struct nand_chip *chip,
				const uint8_t *buf, int oob_required)
{
/*
	int eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	uint8_t *oob = chip->oob_poi;
	int i, size;

	for (i = 0; i < chip->ecc.steps; i++) {
		chip->write_buf(mtd, buf, eccsize);
		buf += eccsize;

		chip->write_buf(mtd, oob, eccbytes);
		oob += eccbytes;

		if (chip->ecc.postpad) {
			chip->write_buf(mtd, oob, chip->ecc.postpad);
			oob += chip->ecc.postpad;
		}
	}

	size = mtd->oobsize - (oob - chip->oob_poi);
	if (size)
		chip->write_buf(mtd, oob, size);
    return(0);
*/
    return(1);
}

/* Override read_oob function: do not use NAND_CMD_RNDOUT */
static int doc2001plus_read_oob_syndrome(struct mtd_info *mtd, struct nand_chip *chip,
				  int page)
{
	uint8_t *buf = chip->oob_poi;
	int length = mtd->oobsize;
//	int chunk = chip->ecc.bytes + chip->ecc.prepad + chip->ecc.postpad;
//	int eccsize = chip->ecc.size;
//	uint8_t *bufpoi = buf;
//	int i, toread, sndrnd = 0, pos;

#ifdef DEBUG
	printk("read_oob_syndrome, page:%d, len:%d\n",page,length);
#endif
	chip->cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
	chip->read_buf(mtd, buf, length);

/*
	chip->cmdfunc(mtd, NAND_CMD_READ0, chip->ecc.size, page);
	for (i = 0; i < chip->ecc.steps; i++) {
		if (sndrnd) {
			pos = eccsize + i * (eccsize + chunk);
			chip->cmdfunc(mtd, NAND_CMD_READ0, pos, page);
		} else
			sndrnd = 1;
		toread = min_t(int, length, chunk);
		chip->read_buf(mtd, bufpoi, toread);
		bufpoi += toread;
		length -= toread;
	}
	if (length > 0)
		chip->read_buf(mtd, bufpoi, length);
*/
	return 1;
}

/* Override write_oob function: do not use NAND_CMD_RNDIN */
static int doc2001plus_write_oob_syndrome(struct mtd_info *mtd,
				   struct nand_chip *chip, int page)
{
/*
	int chunk = chip->ecc.bytes + chip->ecc.prepad + chip->ecc.postpad;
	int eccsize = chip->ecc.size, length = mtd->oobsize;
	int i, len, pos, status = 0, sndcmd = 0, steps = chip->ecc.steps;
	const uint8_t *bufpoi = chip->oob_poi;

#ifdef DEBUG
	    printk("write_oob_syndrome\n");
#endif
	pos = eccsize;
	chip->cmdfunc(mtd, NAND_CMD_SEQIN, pos, page);
	for (i = 0; i < steps; i++) {
		if (sndcmd) {
			uint32_t fill = 0xFFFFFFFF;

			len = eccsize;
			while (len > 0) {
				int num = min_t(int, len, 4);
				chip->write_buf(mtd, (uint8_t *)&fill,
						num);
				len -= num;
			}
		} else
			sndcmd = 1;
		len = min_t(int, length, chunk);
		chip->write_buf(mtd, bufpoi, len);
		bufpoi += len;
		length -= len;
	}
	if (length > 0)
		chip->write_buf(mtd, bufpoi, length);

	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	status = chip->waitfunc(mtd, chip);

	return status & NAND_STATUS_FAIL ? -EIO : 0;
*/
    return 1;
}

//u_char mydatabuf[528];

/* The strange out-of-order .oobfree list below is a (possibly unneeded)
 * attempt to retain compatibility.  It used to read:
 * 	.oobfree = { {8, 8} }
 * Since that leaves two bytes unusable, it was changed.  But the following
 * scheme might affect existing jffs2 installs by moving the cleanmarker:
 * 	.oobfree = { {6, 10} }
 * jffs2 seems to handle the above gracefully, but the current scheme seems
 * safer.  The only problem with it is that any code that parses oobfree must
 * be able to handle out-of-order segments.
 */

static struct nand_ecclayout doc200x_oobinfo = {
	.eccbytes = 6,
	.eccpos = {0, 1, 2, 3, 4, 5},
	.oobfree = {{8, 8}, {6, 2}}
};

static struct nand_ecclayout docmilplus32_1024_oobinfo = {
	.eccbytes = 12,
	.eccpos = {0, 1, 2, 3, 4, 5, 10, 11, 12, 13, 14, 15},
	.oobfree = {{6, 4}, {16,16}}
};

/* Find the (I)NFTL Media Header, and optionally also the mirror media header.
   On successful return, buf will contain a copy of the media header for
   further processing.  id is the string to scan for, and will presumably be
   either "ANAND" or "BNAND".  If findmirror=1, also look for the mirror media
   header.  The page #s of the found media headers are placed in mh0_page and
   mh1_page in the DOC private structure. */
static int __init find_media_headers(struct mtd_info *mtd, u_char *buf, const char *id, int findmirror)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = this->priv;
	unsigned offs;
	int ret;
	size_t retlen;

	for (offs = 0; offs < mtd->size; offs += mtd->erasesize) {
		ret = mtd_read(mtd, offs, mtd->writesize, &retlen, buf);
		if (retlen != mtd->writesize)
			continue;
		if (ret) {
			printk(KERN_WARNING "ECC error scanning DOC at 0x%x\n", offs);
		}

		if (memcmp(buf, id, 6))
			continue;
		printk(KERN_INFO "Found DiskOnChip %s Media Header at 0x%x\n", id, offs);
		if (doc->mh0_page == -1) {
			doc->mh0_page = offs >> this->page_shift;
			if (!findmirror)
				return 1;
			continue;
		}
		doc->mh1_page = offs >> this->page_shift;
		return 2;
	}

	if (doc->mh0_page == -1) {
		printk(KERN_WARNING "DiskOnChip %s Media Header not found.\n", id);
		return 0;
	}
	/* Only one mediaheader was found.  We want buf to contain a
	   mediaheader on return, so we'll have to re-read the one we found. */
	offs = doc->mh0_page << this->page_shift;
	ret = mtd_read(mtd, offs, mtd->writesize, &retlen, buf);
	if (retlen != mtd->writesize) {
		/* Insanity.  Give up. */
		printk(KERN_ERR "Read DiskOnChip Media Header once, but can't reread it???\n");
		return 0;
	}
	return 1;
}

/* This is a stripped-down copy of the code in inftlmount.c */
static inline int __init inftl_partscan(struct mtd_info *mtd, struct mtd_partition *parts)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = this->priv;
	int ret = 0;
	u_char *buf;
	struct INFTLMediaHeader *mh;
	struct INFTLPartition *ip;
	int numparts = 0;
	int blocks;
	int vshift, lastvunit = 0;
	int i;
	int end = mtd->size;


	if (inftl_bbt_write)
		end -= (INFTL_BBT_RESERVED_BLOCKS << this->phys_erase_shift);

	buf = kmalloc(mtd->writesize, GFP_KERNEL);

	if (!buf) {
		return 0;
	}

	if (!find_media_headers(mtd, buf, "BNAND", 0))
		goto out;
	doc->mh1_page = doc->mh0_page + (4096 >> this->page_shift);
	mh = (struct INFTLMediaHeader *)buf;

	le32_to_cpus(&mh->NoOfBootImageBlocks);
	le32_to_cpus(&mh->NoOfBinaryPartitions);
	le32_to_cpus(&mh->NoOfBDTLPartitions);
	le32_to_cpus(&mh->BlockMultiplierBits);
	le32_to_cpus(&mh->FormatFlags);
	le32_to_cpus(&mh->PercentUsed);

	printk(KERN_INFO "    bootRecordID          = %s\n"
			 "    NoOfBootImageBlocks   = %d\n"
			 "    NoOfBinaryPartitions  = %d\n"
			 "    NoOfBDTLPartitions    = %d\n"
			 "    BlockMultiplerBits    = %d\n"
			 "    FormatFlgs            = %d\n"
			 "    OsakVersion           = %d.%d.%d.%d\n"
			 "    PercentUsed           = %d\n",
		mh->bootRecordID, mh->NoOfBootImageBlocks,
		mh->NoOfBinaryPartitions,
		mh->NoOfBDTLPartitions,
		mh->BlockMultiplierBits, mh->FormatFlags,
		((unsigned char *) &mh->OsakVersion)[0] & 0xf,
		((unsigned char *) &mh->OsakVersion)[1] & 0xf,
		((unsigned char *) &mh->OsakVersion)[2] & 0xf,
		((unsigned char *) &mh->OsakVersion)[3] & 0xf,
		mh->PercentUsed);

	vshift = this->phys_erase_shift + mh->BlockMultiplierBits;

	blocks = mtd->size >> vshift;
	if (blocks > 32768) {
		printk(KERN_ERR "BlockMultiplierBits=%d is inconsistent with device size.  Aborting.\n", mh->BlockMultiplierBits);
		goto out;
	}

	blocks = doc->chips_per_floor << (this->chip_shift - this->phys_erase_shift);
	if (inftl_bbt_write && (blocks > mtd->erasesize)) {
		printk(KERN_ERR "Writeable BBTs spanning more than one erase block are not yet supported.  FIX ME!\n");
		goto out;
	}

	/* Scan the partitions */
	for (i = 0; (i < 5); i++) {
		ip = &(mh->Partitions[i]);
		le32_to_cpus(&ip->virtualUnits);
		le32_to_cpus(&ip->firstUnit);
		le32_to_cpus(&ip->lastUnit);
		le32_to_cpus(&ip->flags);
		le32_to_cpus(&ip->spareUnits);
		le32_to_cpus(&ip->Reserved0);

		printk(KERN_INFO	"    PARTITION[%d] ->\n"
			"        virtualUnits    = %d\n"
			"        firstUnit       = %d\n"
			"        lastUnit        = %d\n"
			"        flags           = 0x%x\n"
			"        spareUnits      = %d\n",
			i, ip->virtualUnits, ip->firstUnit,
			ip->lastUnit, ip->flags,
			ip->spareUnits);

		if ((show_firmware_partition == 1) &&
		    (i == 0) && (ip->firstUnit > 0)) {
			parts[0].name = " DiskOnChip IPL / Media Header partition";
			parts[0].offset = 0;
			parts[0].size = mtd->erasesize * ip->firstUnit;
			numparts = 1;
		}

		if (ip->flags & INFTL_BINARY)
			parts[numparts].name = " DiskOnChip BDK partition";
		else
			parts[numparts].name = " DiskOnChip BDTL partition";
		parts[numparts].offset = ip->firstUnit << vshift;
		parts[numparts].size = (1 + ip->lastUnit - ip->firstUnit) << vshift;
		numparts++;
		if (ip->lastUnit > lastvunit)
			lastvunit = ip->lastUnit;
		if (ip->flags & INFTL_LAST)
			break;
	}
	lastvunit++;
	if ((lastvunit << vshift) < end) {
		parts[numparts].name = " DiskOnChip Remainder partition";
		parts[numparts].offset = (lastvunit << vshift);
		parts[numparts].size = end - parts[numparts].offset;
		numparts++;
	}
	ret = numparts;
 out:
	kfree(buf);
	return ret;
}

static int __init inftl_scan_bbt(struct mtd_info *mtd)
{
	int ret, numparts;
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = this->priv;
	struct mtd_partition parts[5];

	if (this->numchips > doc->chips_per_floor) {
		printk(KERN_ERR "Multi-floor INFTL devices not yet supported.\n");
		return -EIO;
	}

	this->bbt_td->options = NAND_BBT_2BIT | NAND_BBT_ABSPAGE;
	if (inftl_bbt_write)
	    this->bbt_td->options |= NAND_BBT_WRITE;
	this->bbt_td->pages[0] = 2;
	this->bbt_md = NULL;

	/* It's safe to set bd=NULL below because NAND_BBT_CREATE is not set.
	   At least as nand_bbt.c is currently written. */
	if ((ret = nand_scan_bbt(mtd, NULL)))
		return ret;
	memset((char *)parts, 0, sizeof(parts));
	numparts = inftl_partscan(mtd, parts);
	/* At least for now, require the INFTL Media Header.  We could probably
	   do without it for non-INFTL use, since all it gives us is
	   autopartitioning, but I want to give it more thought. */
	if (!numparts)
		return -EIO;
	mtd_device_register(mtd, NULL, 0);
	if (!no_autopart)
		mtd_device_register(mtd, parts, numparts);
	return 0;
}

/* Set proper interleaving according to DocMilPlus type */
static void __init doc2001plus_init_interface(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = this->priv;
	u_char conf;

	/* Work out the interface setting */
	doc->interface_16 = (ReadDOC(doc->virtadr, Mplus_Configuration) & 0x80) >> 7;
	printk(KERN_INFO "diskonchip: DiskOnChip Millennium Plus interface is set to %s data bits.\n",
	       doc->interface_16 ? "16":"8");

//IAM switch to 16 bit
	if(! doc->interface_16)
		printk(KERN_WARNING "diskonchip: 8 bits data interface not yet supported!!!\n");

	/* Work out the intended interleave setting */
	doc->interleave = 0;
	if (doc->ChipID == DOC_ChipID_DocMilPlus32)
		doc->interleave = 1;

	/* Check the ASIC agrees */
	conf = ReadDOC(doc->virtadr, Mplus_Configuration);
	if ( (doc->interleave << 2) != (conf & 4) ) {
		printk(KERN_NOTICE "diskonchip: Setting DiskOnChip Millennium Plus interleave to %s.\n",
		       doc->interleave ? "on":"off");
		conf ^= 4;
		WriteDOC(conf, doc->virtadr, Mplus_Configuration);
	}
}

static int __init doc2001plus_ident_chip(struct mtd_info *mtd, int nr)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = this->priv;
	void __iomem *docptr = doc->virtadr;
	u_char id;

	/* select device */
	WriteDOC(nr, docptr, Mplus_DeviceSelect);

	/* get Chip ID */
	id = ReadDOC(docptr, ChipID);

	if (id == DOC_ChipID_DocMilPlus16 || id == DOC_ChipID_DocMilPlus32)
	{
		/* Check the TOGGLE bit in the ECC register */
		u_char tmpa = ReadDOC(docptr, Mplus_Toggle) & DOC_TOGGLE_BIT;
		u_char tmpb = ReadDOC(docptr, Mplus_Toggle) & DOC_TOGGLE_BIT;
		u_char tmpc = ReadDOC(docptr, Mplus_Toggle) & DOC_TOGGLE_BIT;
		if (tmpa != tmpb && tmpa == tmpc)
			return id;
	}
	return -1;
}

static int __init doc2001plus_count_chips(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	struct doc_priv *doc = this->priv;
	uint16_t mfrid;
	int i;

	/* Get the id of the first chip */
	mfrid = doc2001plus_ident_chip(mtd, 0);

	/* Find number of chips, Max 4 chips on Millenium Plus */
	for (i = 1; i < 4; i++) {
		if (doc2001plus_ident_chip(mtd, i) != mfrid)
			break;
	}

	printk(KERN_INFO "diskonchip: Detected %d chips. Total size: %d MiB.\n",
	       i, ( (doc->ChipID == DOC_ChipID_DocMilPlus32) ? 32 : 16 ) * i);
	return i;
}

static int __init doc_probe(unsigned long physadr)
{
	unsigned char ChipID;
	struct mtd_info *mtd;
	struct nand_chip *nand;
	struct doc_priv *doc;
	void __iomem *virtadr;
	unsigned char save_control;
	unsigned char tmp, tmpb, tmpc;
	int reg, len, numchips;
	int ret = 0;

	if (!request_mem_region(physadr, DOC_IOREMAP_LEN, "DiskOnChip"))
		return -EBUSY;
	virtadr = ioremap(physadr, DOC_IOREMAP_LEN);
	if (!virtadr) {
		printk(KERN_ERR "Diskonchip ioremap failed: 0x%x bytes at 0x%lx\n", DOC_IOREMAP_LEN, physadr);
		ret = -EIO;
		goto error_ioremap;
	}

	/* It's not possible to cleanly detect the DiskOnChip - the
	 * bootup procedure will put the device into reset mode, and
	 * it's not possible to talk to it without actually writing
	 * to the DOCControl register. So we store the current contents
	 * of the DOCControl register's location, in case we later decide
	 * that it's not a DiskOnChip, and want to put it back how we
	 * found it.
	 */
	save_control = ReadDOC(virtadr, DOCControl);

	/* Reset the DiskOnChip ASIC */
	WriteDOC(DOC_MODE_CLR_ERR | DOC_MODE_MDWREN | DOC_MODE_RESET, virtadr, DOCControl);
	WriteDOC(DOC_MODE_CLR_ERR | DOC_MODE_MDWREN | DOC_MODE_RESET, virtadr, DOCControl);

	/* Enable the DiskOnChip ASIC */
	WriteDOC(DOC_MODE_CLR_ERR | DOC_MODE_MDWREN | DOC_MODE_NORMAL, virtadr, DOCControl);
	WriteDOC(DOC_MODE_CLR_ERR | DOC_MODE_MDWREN | DOC_MODE_NORMAL, virtadr, DOCControl);

	ChipID = ReadDOC(virtadr, ChipID);

	switch (ChipID) {
	case DOC_ChipID_DocMilPlus16:
	case DOC_ChipID_DocMilPlus32:
//	case 0:
		/* Possible Millennium Plus, need to do more checks */
		/* Possibly release from power down mode */
		for (tmp = 0; (tmp < 4); tmp++)
			ReadDOC(virtadr, Mplus_Power);

		/* Reset the Millennium Plus ASIC */
		tmp = DOC_MODE_RESET | DOC_MODE_MDWREN | DOC_MODE_RST_LAT | DOC_MODE_BDECT;
		WriteDOC(tmp, virtadr, Mplus_DOCControl);
		WriteDOC(~tmp, virtadr, Mplus_CtrlConfirm);

		mdelay(1);
		/* Enable the Millennium Plus ASIC */
		tmp = DOC_MODE_NORMAL | DOC_MODE_MDWREN | DOC_MODE_RST_LAT | DOC_MODE_BDECT;
		WriteDOC(tmp, virtadr, Mplus_DOCControl);
		WriteDOC(~tmp, virtadr, Mplus_CtrlConfirm);
		mdelay(1);

		ChipID = ReadDOC(virtadr, ChipID);

		switch (ChipID) {
		case DOC_ChipID_DocMilPlus16:
		case DOC_ChipID_DocMilPlus32:
			reg = DoC_Mplus_Toggle;
			break;
		default:
			ret = -ENODEV;
			goto notfound;
		}
		break;

	default:
		printk(KERN_WARNING "Only MilPlus DiskOnChip support!\n");
		ret = -ENODEV;
		goto notfound;
	}
	/* Check the TOGGLE bit in the ECC register */
	tmp = ReadDOC_(virtadr, reg) & DOC_TOGGLE_BIT;
	tmpb = ReadDOC_(virtadr, reg) & DOC_TOGGLE_BIT;
	tmpc = ReadDOC_(virtadr, reg) & DOC_TOGGLE_BIT;
	if ((tmp == tmpb) || (tmp != tmpc)) {
		printk(KERN_WARNING "Possible DiskOnChip at 0x%lx failed TOGGLE test, dropping.\n", physadr);
		ret = -ENODEV;
		goto notfound;
	}

	for (mtd = doclist; mtd; mtd = doc->nextdoc) {
		unsigned char oldval;
		unsigned char newval;
		nand = mtd->priv;
		doc = nand->priv;
		/* Use the alias resolution register to determine if this is
		   in fact the same DOC aliased to a new address.  If writes
		   to one chip's alias resolution register change the value on
		   the other chip, they're the same chip. */
		switch (ChipID) {
		case DOC_ChipID_DocMilPlus16:
		case DOC_ChipID_DocMilPlus32:
			oldval = ReadDOC(doc->virtadr, Mplus_AliasResolution);
			newval = ReadDOC(virtadr, Mplus_AliasResolution);
			break;
		default:
			oldval = ReadDOC(doc->virtadr, AliasResolution);
			newval = ReadDOC(virtadr, AliasResolution);
		}
		if (oldval != newval)
			continue;
		switch (ChipID) {
		case DOC_ChipID_DocMilPlus16:
		case DOC_ChipID_DocMilPlus32:
			WriteDOC(~newval, virtadr, Mplus_AliasResolution);
			oldval = ReadDOC(doc->virtadr, Mplus_AliasResolution);
			WriteDOC(newval, virtadr, Mplus_AliasResolution);	// restore it
			break;
		default:
			WriteDOC(~newval, virtadr, AliasResolution);
			oldval = ReadDOC(doc->virtadr, AliasResolution);
			WriteDOC(newval, virtadr, AliasResolution);	// restore it
		}
		newval = ~newval;
		if (oldval == newval) {
			printk(KERN_DEBUG "Found alias of DOC at 0x%lx to 0x%lx\n", doc->physadr, physadr);
			goto notfound;
		}
	}

	printk(KERN_NOTICE "DiskOnChip found at 0x%lx\n", physadr);

	len = sizeof(struct mtd_info) +
	    sizeof(struct nand_chip) + sizeof(struct doc_priv) + (2 * sizeof(struct nand_bbt_descr));
	mtd = kzalloc(len, GFP_KERNEL);
	if (!mtd) {
		ret = -ENOMEM;
		goto fail;
	}

	nand			= (struct nand_chip *) (mtd + 1);
	doc			= (struct doc_priv *) (nand + 1);
	nand->bbt_td		= (struct nand_bbt_descr *) (doc + 1);
	nand->bbt_md		= nand->bbt_td + 1;

	mtd->priv		= nand;
	mtd->owner		= THIS_MODULE;

	nand->priv		= doc;
	nand->cmd_ctrl		= doc200x_hwcontrol;
	nand->dev_ready		= doc200x_dev_ready;
	nand->waitfunc		= doc200x_wait;
	nand->block_bad		= doc200x_block_bad;

	nand->read_byte = doc2001plus_read_byte;
	nand->write_buf = doc2001plus_writebuf;
	nand->read_buf = doc2001plus_readbuf;
	nand->scan_bbt = inftl_scan_bbt;
//	nand->cmd_ctrl = NULL;
	nand->select_chip = doc2001plus_select_chip;
	nand->cmdfunc = doc2001plus_command;
	nand->ecc.hwctl = doc2001plus_enable_hwecc;
	nand->ecc.calculate	= doc200x_calculate_ecc;
	nand->ecc.correct	= doc200x_correct_data;

	nand->ecc.layout	= &doc200x_oobinfo;
	nand->ecc.mode		= NAND_ECC_HW_SYNDROME;
	nand->ecc.size		= 512;
	nand->ecc.bytes		= 6;
	nand->ecc.strength	= 2;
	nand->bbt_options	= NAND_BBT_USE_FLASH | NAND_NO_SUBPAGE_WRITE;;


	nand->ecc.write_page_raw = doc2001plus_write_page_raw_syndrome;
	nand->ecc.read_oob = doc2001plus_read_oob_syndrome;
	nand->ecc.write_oob = doc2001plus_write_oob_syndrome;


	doc->physadr		= physadr;
	doc->virtadr		= virtadr;
	doc->ChipID		= ChipID;
	doc->curfloor		= -1;
	doc->mh0_page		= -1;
	doc->mh1_page		= -1;
	doc->interleave		= 0;
	doc->nextdoc		= doclist;
	doc->curchip		= 0;

//	doc->cached_ind 	= 0;
//	doc->cache_adr		= -1;
//	doc->cache_chip		= doc->curchip;

//	nand->options &= NAND_BUSWIDTH_16;

	nand_scan_ident(mtd, 2, NULL);
	if (ret)
	    goto notfound1;

	doc2001plus_init_interface(mtd);

	doc->chips_per_floor = doc2001plus_count_chips(mtd);

	if (doc->ChipID == DOC_ChipID_DocMilPlus16)
		mtd->name = "DiskOnChip Millennium Plus 16MB";
	else
		mtd->name = "DiskOnChip Millennium Plus 32MB";

	/* On interleave mode, preset page-size and erase-size */
	if (doc->interleave)
	{
//		mtd->writesize = 0x400; /* 1kB */
		mtd->erasesize = 0x8000; /* 32kB */
//		mtd->erasesize = 0x4000; /* 16kB TODO*/
		nand->phys_erase_shift = ffs(mtd->erasesize) - 1;
//		nand->ecc.layout = &docmilplus32_1024_oobinfo;
//		nand->ecc.postpad = 4; /* OOB0 */

//mtd->oobsize = 6;
//printk("chip_shift:%d  page_shift:%d\n",nand->chip_shift,nand->page_shift);
//nand->page_shift = ffs(mtd->writesize) - 1;

//nand->pagemask = (nand->chipsize >> nand->page_shift) - 1;

//chip->chip_shift = ffs((unsigned)chip->chipsize) - 1;

//printk("chip_shift:%d  page_shift:%d\n",nand->chip_shift,nand->page_shift);
	}

	numchips =  doc->chips_per_floor;

	ret = nand_scan_tail(mtd);

//	if ((ret = nand_scan(mtd, numchips))) {
	if (ret)
	    goto notfound1;


	/* Success! */
	doclist = mtd;
	return 0;

notfound1:
	nand_release(mtd);
	kfree(mtd);
notfound:
	/* Put back the contents of the DOCControl register, in case it's not
	   actually a DiskOnChip.  */
	WriteDOC(save_control, virtadr, DOCControl);
 fail:
	iounmap(virtadr);

error_ioremap:
	release_mem_region(physadr, DOC_IOREMAP_LEN);

	return ret;
}

static void release_nanddoc(void)
{
	struct mtd_info *mtd, *nextmtd;
	struct nand_chip *nand;
	struct doc_priv *doc;

	for (mtd = doclist; mtd; mtd = nextmtd) {
		nand = mtd->priv;
		doc = nand->priv;

		nextmtd = doc->nextdoc;
		nand_release(mtd);
		iounmap(doc->virtadr);
		release_mem_region(doc->physadr, DOC_IOREMAP_LEN);
		kfree(mtd);
	}
}

static int __init init_nanddoc(void)
{
	int i, ret = 0;

	/* We could create the decoder on demand, if memory is a concern.
	 * This way we have it handy, if an error happens
	 *
	 * Symbolsize is 10 (bits)
	 * Primitve polynomial is x^10+x^3+1
	 * first consecutive root is 510
	 * primitve element to generate roots = 1
	 * generator polinomial degree = 4
	 */
	rs_decoder = init_rs(10, 0x409, FCR, 1, NROOTS);
	if (!rs_decoder) {
		printk(KERN_ERR "DiskOnChip: Could not create a RS decoder\n");
		return -ENOMEM;
	}

	if (doc_config_location) {
		printk(KERN_INFO "Using configured DiskOnChip probe address 0x%lx\n", doc_config_location);
		ret = doc_probe(doc_config_location);
		if (ret < 0)
			goto outerr;
	} else {
		for (i = 0; (doc_locations[i] != 0xffffffff); i++) {
			doc_probe(doc_locations[i]);
		}
	}
	/* No banner message any more. Print a message if no DiskOnChip
	   found, so the user knows we at least tried. */
	if (!doclist) {
		printk(KERN_INFO "No valid DiskOnChip devices found\n");
		ret = -ENODEV;
		goto outerr;
	}
	return 0;
 outerr:
	free_rs(rs_decoder);
	return ret;
}

static void __exit cleanup_nanddoc(void)
{
	/* Cleanup the nand/DoC resources */
	release_nanddoc();

	/* Free the reed solomon resources */
	if (rs_decoder) {
		free_rs(rs_decoder);
	}
}

module_init(init_nanddoc);
module_exit(cleanup_nanddoc);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("David Woodhouse <dwmw2@infradead.org>");
MODULE_DESCRIPTION("M-Systems DiskOnChip 2000, Millennium and Millennium Plus device driver");
