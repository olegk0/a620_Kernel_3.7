/*
 * inftlcore.c -- Linux driver for Inverse Flash Translation Layer (INFTL)
 *
 * Copyright © 2002, Greg Ungerer (gerg@snapgear.com)
 *
 * Based heavily on the nftlcore.c code which is:
 * Copyright © 1999 Machine Vision Holdings, Inc.
 * Copyright © 1999 David Woodhouse <dwmw2@infradead.org>
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/hdreg.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nftl.h>
#include <linux/mtd/inftl.h>
#include <linux/mtd/nand.h>
#include <asm/uaccess.h>
#include <asm/errno.h>
#include <asm/io.h>


//#define DEBUG_TABLES
//#define DEBUG_MAP

/*
 * Maximum number of loops while examining next block, to have a
 * chance to detect consistency problems (they should never happen
 * because of the checks done in the mounting.
 */
#define MAX_LOOPS 10000

/*
 * find_boot_record: Find the INFTL Media Header and its Spare copy which
 *	contains the various device information of the INFTL partition and
 *	Bad Unit Table. Update the PUtable[] table according to the Bad
 *	Unit Table. PUtable[] is used for management of Erase Unit in
 *	other routines in inftlcore.c and inftlmount.c.
 */
static int find_boot_record(struct INFTLrecord *inftl)
{
	struct inftl_unittail h1;
	//struct inftl_oob oob;
	unsigned int i, block;
	u8 buf[SECTORSIZE];
	struct INFTLMediaHeader *mh = &inftl->MediaHdr;
	struct mtd_info *mtd = inftl->mbd.mtd;
	struct INFTLPartition *ip;
	size_t retlen;

	pr_debug("INFTL: find_boot_record(inftl=%p)\n", inftl);

        /*
	 * Assume logical EraseSize == physical erasesize for starting the
	 * scan. We'll sort it out later if we find a MediaHeader which says
	 * otherwise.
	 */
	inftl->EraseSize = inftl->mbd.mtd->erasesize;
        inftl->nb_blocks = (u32)inftl->mbd.mtd->size / inftl->EraseSize;

	inftl->MediaUnit = BLOCK_NIL;

	/* Search for a valid boot record */
	for (block = 0; block < inftl->nb_blocks; block++) {
		int ret;

		/*
		 * Check for BNAND header first. Then whinge if it's found
		 * but later checks fail.
		 */
		ret = mtd_read(mtd, block * inftl->EraseSize, SECTORSIZE,
			       &retlen, buf);
		/* We ignore ret in case the ECC of the MediaHeader is invalid
		   (which is apparently acceptable) */
		if (retlen != SECTORSIZE) {
			static int warncount = 5;

			if (warncount) {
				printk(KERN_WARNING "INFTL: block read at 0x%x "
					"of mtd%d failed: %d\n",
					block * inftl->EraseSize,
					inftl->mbd.mtd->index, ret);
				if (!--warncount)
					printk(KERN_WARNING "INFTL: further "
						"failures for this block will "
						"not be printed\n");
			}
			continue;
		}

		if (retlen < 6 || memcmp(buf, "BNAND", 6)) {
			/* BNAND\0 not found. Continue */
			continue;
		}

		/* To be safer with BIOS, also use erase mark as discriminant */
		ret = inftl_read_oob(mtd,
				     block * inftl->EraseSize + SECTORSIZE + 8,
				     8, &retlen,(char *)&h1);
		if (ret < 0) {
			printk(KERN_WARNING "INFTL: ANAND header found at "
				"0x%x in mtd%d, but OOB data read failed "
				"(err %d)\n", block * inftl->EraseSize,
				inftl->mbd.mtd->index, ret);
			continue;
		}


		/*
		 * This is the first we've seen.
		 * Copy the media header structure into place.
		 */
		memcpy(mh, buf, sizeof(struct INFTLMediaHeader));

		/* Read the spare media header at offset 4096 */
		mtd_read(mtd, block * inftl->EraseSize + 4096, SECTORSIZE,
			 &retlen, buf);
		if (retlen != SECTORSIZE) {
			printk(KERN_WARNING "INFTL: Unable to read spare "
			       "Media Header\n");
			return -1;
		}
		/* Check if this one is the same as the first one we found. */
		if (memcmp(mh, buf, sizeof(struct INFTLMediaHeader))) {
			printk(KERN_WARNING "INFTL: Primary and spare Media "
			       "Headers disagree.\n");
			return -1;
		}

		mh->NoOfBootImageBlocks = le32_to_cpu(mh->NoOfBootImageBlocks);
		mh->NoOfBinaryPartitions = le32_to_cpu(mh->NoOfBinaryPartitions);
		mh->NoOfBDTLPartitions = le32_to_cpu(mh->NoOfBDTLPartitions);
		mh->BlockMultiplierBits = le32_to_cpu(mh->BlockMultiplierBits);
		mh->FormatFlags = le32_to_cpu(mh->FormatFlags);
		mh->PercentUsed = le32_to_cpu(mh->PercentUsed);

		pr_debug("INFTL: Media Header ->\n"
			 "    bootRecordID          = %s\n"
			 "    NoOfBootImageBlocks   = %d\n"
			 "    NoOfBinaryPartitions  = %d\n"
			 "    NoOfBDTLPartitions    = %d\n"
			 "    BlockMultiplerBits    = %d\n"
			 "    FormatFlgs            = %d\n"
			 "    OsakVersion           = 0x%x\n"
			 "    PercentUsed           = %d\n",
			 mh->bootRecordID, mh->NoOfBootImageBlocks,
			 mh->NoOfBinaryPartitions,
			 mh->NoOfBDTLPartitions,
			 mh->BlockMultiplierBits, mh->FormatFlags,
			 mh->OsakVersion, mh->PercentUsed);

		if (mh->NoOfBDTLPartitions == 0) {
			printk(KERN_WARNING "INFTL: Media Header sanity check "
				"failed: NoOfBDTLPartitions (%d) == 0, "
				"must be at least 1\n", mh->NoOfBDTLPartitions);
			return -1;
		}

		if ((mh->NoOfBDTLPartitions + mh->NoOfBinaryPartitions) > 4) {
			printk(KERN_WARNING "INFTL: Media Header sanity check "
				"failed: Total Partitions (%d) > 4, "
				"BDTL=%d Binary=%d\n", mh->NoOfBDTLPartitions +
				mh->NoOfBinaryPartitions,
				mh->NoOfBDTLPartitions,
				mh->NoOfBinaryPartitions);
			return -1;
		}

		if (mh->BlockMultiplierBits > 1) {
			printk(KERN_WARNING "INFTL: sorry, we don't support "
				"UnitSizeFactor 0x%02x\n",
				mh->BlockMultiplierBits);
			return -1;
		} else if (mh->BlockMultiplierBits == 1) {
			printk(KERN_WARNING "INFTL: support for INFTL with "
				"UnitSizeFactor 0x%02x is experimental\n",
				mh->BlockMultiplierBits);
			inftl->EraseSize = inftl->mbd.mtd->erasesize <<
				mh->BlockMultiplierBits;
			inftl->nb_blocks = (u32)inftl->mbd.mtd->size / inftl->EraseSize;
			block >>= mh->BlockMultiplierBits;
		}

		/* Scan the partitions */
		for (i = 0; (i < 4); i++) {
			ip = &mh->Partitions[i];
			ip->virtualUnits = le32_to_cpu(ip->virtualUnits);
			ip->firstUnit = le32_to_cpu(ip->firstUnit);
			ip->lastUnit = le32_to_cpu(ip->lastUnit);
			ip->flags = le32_to_cpu(ip->flags);
			ip->spareUnits = le32_to_cpu(ip->spareUnits);
			ip->Reserved0 = le32_to_cpu(ip->Reserved0);

			pr_debug("    PARTITION[%d] ->\n"
				 "        virtualUnits    = %d\n"
				 "        firstUnit       = %d\n"
				 "        lastUnit        = %d\n"
				 "        flags           = 0x%x\n"
				 "        spareUnits      = %d\n",
				 i, ip->virtualUnits, ip->firstUnit,
				 ip->lastUnit, ip->flags,
				 ip->spareUnits);

			if (ip->Reserved0 != ip->firstUnit) {
				struct erase_info *instr = &inftl->instr;

				instr->mtd = inftl->mbd.mtd;

				/*
				 * 	Most likely this is using the
				 * 	undocumented qiuck mount feature.
				 * 	We don't support that, we will need
				 * 	to erase the hidden block for full
				 * 	compatibility.
				 */
				instr->addr = ip->Reserved0 * inftl->EraseSize;
				instr->len = inftl->EraseSize;
				mtd_erase(mtd, instr);
			}
			if ((ip->lastUnit - ip->firstUnit + 1) < ip->virtualUnits) {
				printk(KERN_WARNING "INFTL: Media Header "
					"Partition %d sanity check failed\n"
					"    firstUnit %d : lastUnit %d  >  "
					"virtualUnits %d\n", i, ip->lastUnit,
					ip->firstUnit, ip->Reserved0);
				return -1;
			}
			if (ip->Reserved1 != 0) {
				printk(KERN_WARNING "INFTL: Media Header "
					"Partition %d sanity check failed: "
					"Reserved1 %d != 0\n",
					i, ip->Reserved1);
				return -1;
			}

			if (ip->flags & INFTL_BDTL)
				break;
		}

		if (i >= 4) {
			printk(KERN_WARNING "INFTL: Media Header Partition "
				"sanity check failed:\n       No partition "
				"marked as Disk Partition\n");
			return -1;
		}

		inftl->nb_boot_blocks = ip->firstUnit;
		inftl->numvunits = ip->virtualUnits;
		if (inftl->numvunits > (inftl->nb_blocks -
		    inftl->nb_boot_blocks - 2)) {
			printk(KERN_WARNING "INFTL: Media Header sanity check "
				"failed:\n        numvunits (%d) > nb_blocks "
				"(%d) - nb_boot_blocks(%d) - 2\n",
				inftl->numvunits, inftl->nb_blocks,
				inftl->nb_boot_blocks);
			return -1;
		}

		inftl->mbd.size  = inftl->numvunits *
			(inftl->EraseSize / SECTORSIZE);

		/*
		 * Block count is set to last used EUN (we won't need to keep
		 * any meta-data past that point).
		 */
		inftl->firstEUN = ip->firstUnit;
		inftl->lastEUN = ip->lastUnit;
		inftl->nb_blocks = ip->lastUnit + 1;

		/* Memory alloc */
		inftl->PUtable = kmalloc(inftl->nb_blocks * sizeof(u16), GFP_KERNEL);
		if (!inftl->PUtable) {
			printk(KERN_WARNING "INFTL: allocation of PUtable "
				"failed (%zd bytes)\n",
				inftl->nb_blocks * sizeof(u16));
			return -ENOMEM;
		}

		inftl->VUtable = kmalloc(inftl->nb_blocks * sizeof(u16), GFP_KERNEL);
		if (!inftl->VUtable) {
			kfree(inftl->PUtable);
			printk(KERN_WARNING "INFTL: allocation of VUtable "
				"failed (%zd bytes)\n",
				inftl->nb_blocks * sizeof(u16));
			return -ENOMEM;
		}

		/* Mark the blocks before INFTL MediaHeader as reserved */
		for (i = 0; i < inftl->nb_boot_blocks; i++)
			inftl->PUtable[i] = BLOCK_RESERVED;
		/* Mark all remaining blocks as potentially containing data */
		for (; i < inftl->nb_blocks; i++)
			inftl->PUtable[i] = BLOCK_NOTEXPLORED;

		/* Mark this boot record (NFTL MediaHeader) block as reserved */
		inftl->PUtable[block] = BLOCK_RESERVED;

		/* Read Bad Erase Unit Table and modify PUtable[] accordingly */
		for (i = 0; i < inftl->nb_blocks; i++) {
			int physblock;
			/* If any of the physical eraseblocks are bad, don't
			   use the unit. */
			for (physblock = 0; physblock < inftl->EraseSize; physblock += inftl->mbd.mtd->erasesize) {
				if (mtd_block_isbad(inftl->mbd.mtd,
						    i * inftl->EraseSize + physblock))
					inftl->PUtable[i] = BLOCK_RESERVED;
			}
		}

		inftl->MediaUnit = block;
		return 0;
	}

	/* Not found. */
	return -1;
}

/*
 * format_chain: Format an invalid Virtual Unit chain. It frees all the Erase
 *	Units in a Virtual Unit Chain, i.e. all the units are disconnected.
 *
 *	Since the chain is invalid then we will have to erase it from its
 *	head (normally for INFTL we go from the oldest). But if it has a
 *	loop then there is no oldest...
 */
static void format_chain(struct INFTLrecord *inftl, unsigned int first_block)
{
	unsigned int block = first_block, block1;

//	printk(KERN_WARNING "INFTL: formatting chain at block %d\n",
//		first_block);

	for (;;) {
		block1 = inftl->PUtable[block];

		printk(KERN_WARNING "INFTL: mark block %d as reserved\n", block);
//		if (INFTL_formatblock(inftl, block) < 0) {
			/*
			 * Cannot format !!!! Mark it as Bad Unit,
			 */
			inftl->PUtable[block] = BLOCK_RESERVED;
//		} else {
//			inftl->PUtable[block] = BLOCK_FREE;
//		}

		/* Goto next block on the chain */
		block = block1;

		if (block == BLOCK_NIL || block >= inftl->lastEUN)
			break;
	}
}
#ifdef DEBUG_TABLES
void INFTL_dumptables(struct INFTLrecord *s)
{

	int i;

	printk("-------------------------------------------"
		"----------------------------------\n");

	printk("VUtable[%d] ->", s->nb_blocks);
	for (i = 0; i < s->nb_blocks; i++) {
		if ((i % 8) == 0)
			printk("\n%04x: ", i);
		printk("%04x ", s->VUtable[i]);
	}

	printk("\n-------------------------------------------"
		"----------------------------------\n");

	printk("PUtable[%d-%d=%d] ->", s->firstEUN, s->lastEUN, s->nb_blocks);
	for (i = 0; i <= s->lastEUN; i++) {
		if ((i % 8) == 0)
			printk("\n%04x: ", i);
		printk("%04x ", s->PUtable[i]);
	}

	printk("\n-------------------------------------------"
		"----------------------------------\n");

	printk("INFTL ->\n"
		"  EraseSize       = %d\n"
		"  h/s/c           = %d/%d/%d\n"
		"  numvunits       = %d\n"
		"  firstEUN        = %d\n"
		"  lastEUN         = %d\n"
		"  numfreeEUNs     = %d\n"
		"  LastFreeEUN     = %d\n"
		"  nb_blocks       = %d\n"
		"  nb_boot_blocks  = %d",
		s->EraseSize, s->heads, s->sectors, s->cylinders,
		s->numvunits, s->firstEUN, s->lastEUN, s->numfreeEUNs,
		s->LastFreeEUN, s->nb_blocks, s->nb_boot_blocks);

	printk("\n-------------------------------------------"
		"----------------------------------\n");

}
#endif

#ifdef DEBUG_MAP
void INFTL_dumpVUchains(struct INFTLrecord *s)
{

	int logical, block, i;

	printk("-------------------------------------------"
		"----------------------------------\n");

	printk("INFTL Virtual Unit Chains:\n");
	for (logical = 0; logical < s->nb_blocks; logical++) {
		block = s->VUtable[logical];
		if (block > s->nb_blocks)
			continue;
		printk("  LOGICAL %d --> %d ", logical, block);
		for (i = 0; i < s->nb_blocks; i++) {
			if (s->PUtable[block] == BLOCK_NIL)
				break;
			block = s->PUtable[block];
			printk("%d ", block);
		}
		printk("\n");
	}

	printk("-------------------------------------------"
		"----------------------------------\n");
}
#endif

int INFTL_mount(struct INFTLrecord *s)
{
	struct mtd_info *mtd = s->mbd.mtd;
	unsigned int block, first_block, prev_block, last_block;
	unsigned int first_logical_block, logical_block, erase_mark;
	int chain_length, do_format_chain;
	struct inftl_unithead1 h0;
	struct inftl_unittail h1;
	size_t retlen;
	int i;
	u8 *ANACtable, ANAC;

	pr_debug("INFTL: INFTL_mount(inftl=%p)\n", s);

	/* Search for INFTL MediaHeader and Spare INFTL Media Header */
	if (find_boot_record(s) < 0) {
		printk(KERN_WARNING "INFTL: could not find valid boot record?\n");
		return -ENXIO;
	}

	/* Init the logical to physical table */
	for (i = 0; i < s->nb_blocks; i++)
		s->VUtable[i] = BLOCK_NIL;

	logical_block = block = BLOCK_NIL;

	/* Temporary buffer to store ANAC numbers. */
	ANACtable = kcalloc(s->nb_blocks, sizeof(u8), GFP_KERNEL);
	if (!ANACtable) {
		printk(KERN_WARNING "INFTL: allocation of ANACtable "
				"failed (%zd bytes)\n",
				s->nb_blocks * sizeof(u8));
		return -ENOMEM;
	}

	/*
	 * First pass is to explore each physical unit, and construct the
	 * virtual chains that exist (newest physical unit goes into VUtable).
	 * Any block that is in any way invalid will be left in the
	 * NOTEXPLORED state. Then at the end we will try to format it and
	 * mark it as free.
	 */
	pr_debug("INFTL: pass 1, explore each unit\n");
	for (first_block = s->firstEUN; first_block <= s->lastEUN; first_block++) {
		if (s->PUtable[first_block] != BLOCK_NOTEXPLORED)
			continue;

		do_format_chain = 0;
		first_logical_block = BLOCK_NIL;
		last_block = BLOCK_NIL;
		block = first_block;

		for (chain_length = 0; ; chain_length++) {

			if ((chain_length == 0) &&
			    (s->PUtable[block] != BLOCK_NOTEXPLORED)) {
				/* Nothing to do here, onto next block */
				break;
			}

			if (inftl_read_oob(mtd, block * s->EraseSize + 8,
					   8, &retlen, (char *)&h0) < 0 ||
			    inftl_read_oob(mtd, block * s->EraseSize +
					   2 * SECTORSIZE + 8, 8, &retlen,
					   (char *)&h1) < 0) {
				/* Should never happen? */
				do_format_chain++;
				break;
			}

			logical_block = le16_to_cpu(h0.virtualUnitNo);
			prev_block = le16_to_cpu(h0.prevUnitNo);
			erase_mark = le16_to_cpu((h1.EraseMark | h1.EraseMark1));
			ANACtable[block] = h0.ANAC;
// if(logical_block<10)
//    printk("bl:%d  lbl:%d em:%X\n",block,logical_block,erase_mark);

			/* Previous block is relative to start of Partition */
			if (prev_block < s->nb_blocks)
				prev_block += s->firstEUN;

			/* Already explored partial chain? */
			if (s->PUtable[block] != BLOCK_NOTEXPLORED) {
				/* Check if chain for this logical */
				if (logical_block == first_logical_block) {
					if (last_block != BLOCK_NIL)
						s->PUtable[last_block] = block;
				}
				break;
			}

			/* Check for invalid block */
			if (erase_mark != ERASE_MARK) {
				printk(KERN_WARNING "INFTL: corrupt block %d "
					"in chain %d, chain length %d, erase "
					"mark 0x%x?\n", block, first_block,
					chain_length, erase_mark);
				/*
				 * Assume end of chain, probably incomplete
				 * fold/erase...
				 */
				if (chain_length == 0)
					do_format_chain++;
				break;
			}

			/* Check for it being free already then... */
			if ((logical_block == BLOCK_FREE) ||
			    (logical_block == BLOCK_NIL)) {
				s->PUtable[block] = BLOCK_FREE;
				break;
			}

			/* Sanity checks on block numbers */
			if ((logical_block >= s->nb_blocks) ||
			    ((prev_block >= s->nb_blocks) &&
			     (prev_block != BLOCK_NIL))) {
				if (chain_length > 0) {
					printk(KERN_WARNING "INFTL: corrupt "
						"block %d in chain %d?\n",
						block, first_block);
					do_format_chain++;
				}
				break;
			}

			if (first_logical_block == BLOCK_NIL) {
				first_logical_block = logical_block;
			} else {
				if (first_logical_block != logical_block) {
					/* Normal for folded chain... */
					break;
				}
			}

			/*
			 * Current block is valid, so if we followed a virtual
			 * chain to get here then we can set the previous
			 * block pointer in our PUtable now. Then move onto
			 * the previous block in the chain.
			 */
			s->PUtable[block] = BLOCK_NIL;
			if (last_block != BLOCK_NIL)
				s->PUtable[last_block] = block;
			last_block = block;
			block = prev_block;

			/* Check for end of chain */
			if (block == BLOCK_NIL)
				break;

			/* Validate next block before following it... */
			if (block > s->lastEUN) {
				printk(KERN_WARNING "INFTL: invalid previous "
					"block %d in chain %d?\n", block,
					first_block);
				do_format_chain++;
				break;
			}
		}

		if (do_format_chain) {
//return -ENXIO;
			format_chain(s, first_block);
			continue;
		}

		/*
		 * Looks like a valid chain then. It may not really be the
		 * newest block in the chain, but it is the newest we have
		 * found so far. We might update it in later iterations of
		 * this loop if we find something newer.
		 */
		s->VUtable[first_logical_block] = first_block;
		logical_block = BLOCK_NIL;
	}
#ifdef DEBUG_TABLES
	INFTL_dumptables(s);
#endif
	/*
	 * Second pass, check for infinite loops in chains. These are
	 * possible because we don't update the previous pointers when
	 * we fold chains. No big deal, just fix them up in PUtable.
	 */
	pr_debug("INFTL: pass 2, validate virtual chains\n");
	for (logical_block = 0; logical_block < s->numvunits; logical_block++) {
		block = s->VUtable[logical_block];
		last_block = BLOCK_NIL;

		/* Check for free/reserved/nil */
		if (block >= BLOCK_RESERVED)
			continue;

		ANAC = ANACtable[block];
		for (i = 0; i < s->numvunits; i++) {
			if (s->PUtable[block] == BLOCK_NIL)
				break;
			if (s->PUtable[block] > s->lastEUN) {
				printk(KERN_WARNING "INFTL: invalid prev %d, "
					"in virtual chain %d\n",
					s->PUtable[block], logical_block);
				s->PUtable[block] = BLOCK_NIL;

			}
			if (ANACtable[block] != ANAC) {
				/*
				 * Chain must point back to itself. This is ok,
				 * but we will need adjust the tables with this
				 * newest block and oldest block.
				 */
				s->VUtable[logical_block] = block;
				s->PUtable[last_block] = BLOCK_NIL;
				break;
			}

			ANAC--;
			last_block = block;
			block = s->PUtable[block];
		}

		if (i >= s->nb_blocks) {
			/*
			 * Uhoo, infinite chain with valid ANACS!
			 * Format whole chain...
			 */
			format_chain(s, first_block);
		}
	}
#ifdef DEBUG_TABLES
	INFTL_dumptables(s);
#endif
#ifdef DEBUG_MAP
	INFTL_dumpVUchains(s);
#endif
	/*
	 * Third pass, format unreferenced blocks and init free block count.
	 */
	s->numfreeEUNs = 0;
	s->LastFreeEUN = BLOCK_NIL;
/*
	pr_debug("INFTL: pass 3, format unused blocks\n");
	for (block = s->firstEUN; block <= s->lastEUN; block++) {
		if (s->PUtable[block] == BLOCK_NOTEXPLORED) {
			printk("INFTL: unreferenced block %d, formatting it\n",
				block);
//			if (INFTL_formatblock(s, block) < 0)
				s->PUtable[block] = BLOCK_RESERVED;
//			else
//				s->PUtable[block] = BLOCK_FREE;
		}
		if (s->PUtable[block] == BLOCK_FREE) {
			s->numfreeEUNs++;
			if (s->LastFreeEUN == BLOCK_NIL)
				s->LastFreeEUN = block;
		}
	}
*/
	kfree(ANACtable);
	return 0;
}



static void inftl_add_mtd(struct mtd_blktrans_ops *tr, struct mtd_info *mtd)
{
	struct INFTLrecord *inftl;
	unsigned long temp;

	if (mtd->type != MTD_NANDFLASH || mtd->size > UINT_MAX)
		return;
	/* OK, this is moderately ugly.  But probably safe.  Alternatives? */
	if (memcmp(mtd->name, "DiskOnChip", 10))
		return;

	if (!mtd->_block_isbad) {
		printk(KERN_ERR
"INFTL no longer supports the old DiskOnChip drivers loaded via docprobe.\n"
"Please use the new diskonchip driver under the NAND subsystem.\n");
		return;
	}

	pr_debug("INFTL: add_mtd for %s\n", mtd->name);

	inftl = kzalloc(sizeof(*inftl), GFP_KERNEL);

	if (!inftl)
		return;

	inftl->mbd.mtd = mtd;
	inftl->mbd.devnum = -1;

	inftl->mbd.tr = tr;

	if (INFTL_mount(inftl) < 0) {
		printk(KERN_WARNING "INFTL: could not mount device\n");
		kfree(inftl);
		return;
	}

	/* OK, it's a new one. Set up all the data structures. */

	/* Calculate geometry */
	inftl->cylinders = 1024;
	inftl->heads = 16;

	temp = inftl->cylinders * inftl->heads;
	inftl->sectors = inftl->mbd.size / temp;
	if (inftl->mbd.size % temp) {
		inftl->sectors++;
		temp = inftl->cylinders * inftl->sectors;
		inftl->heads = inftl->mbd.size / temp;

		if (inftl->mbd.size % temp) {
			inftl->heads++;
			temp = inftl->heads * inftl->sectors;
			inftl->cylinders = inftl->mbd.size / temp;
		}
	}

	if (inftl->mbd.size != inftl->heads * inftl->cylinders * inftl->sectors) {
		/*
		  Oh no we don't have
		   mbd.size == heads * cylinders * sectors
		*/
		printk(KERN_WARNING "INFTL: cannot calculate a geometry to "
		       "match size of 0x%lx.\n", inftl->mbd.size);
		printk(KERN_WARNING "INFTL: using C:%d H:%d S:%d "
			"(== 0x%lx sects)\n",
			inftl->cylinders, inftl->heads , inftl->sectors,
			(long)inftl->cylinders * (long)inftl->heads *
			(long)inftl->sectors );
	}

	if (add_mtd_blktrans_dev(&inftl->mbd)) {
		kfree(inftl->PUtable);
		kfree(inftl->VUtable);
		kfree(inftl);
		return;
	}
kfree(inftl->PUtable);
#ifdef PSYCHO_DEBUG
	printk(KERN_INFO "INFTL: Found new inftl%c\n", inftl->mbd.devnum + 'a');
#endif
	return;
}

static void inftl_remove_dev(struct mtd_blktrans_dev *dev)
{
	struct INFTLrecord *inftl = (void *)dev;

	pr_debug("INFTL: remove_dev (i=%d)\n", dev->devnum);

	del_mtd_blktrans_dev(dev);

	kfree(inftl->PUtable);
	kfree(inftl->VUtable);
}

/*
 * Actual INFTL access routines.
 */

/*
 * Read oob data from flash
 */
int inftl_read_oob(struct mtd_info *mtd, loff_t offs, size_t len,
		   size_t *retlen, uint8_t *buf)
{
	struct mtd_oob_ops ops;
	int res;

	ops.mode = MTD_OPS_PLACE_OOB;
//	ops.ooboffs = offs & (mtd->writesize - 1);
	ops.ooboffs = 0;
	ops.ooblen = len;
	ops.oobbuf = buf;
	ops.datbuf = NULL;

	res = mtd_read_oob(mtd, offs & ~(mtd->writesize - 1), &ops);
	*retlen = ops.oobretlen;
	return res;
}

static int inftl_readblock(struct mtd_blktrans_dev *mbd, unsigned long block,
			   char *buffer)
{
	struct INFTLrecord *inftl = (void *)mbd;
	unsigned int thisEUN = inftl->VUtable[block / (inftl->EraseSize / SECTORSIZE)];
	unsigned long blockofs = (block * SECTORSIZE) & (inftl->EraseSize - 1);
	struct mtd_info *mtd = inftl->mbd.mtd;
	unsigned int status;
	int silly = MAX_LOOPS;
	struct inftl_bci bci;
	size_t retlen;

	pr_debug("INFTL: inftl_readblock(inftl=%p,block=%ld,"
		"buffer=%p)\n", inftl, block, buffer);
/*
	while (thisEUN < inftl->nb_blocks) {
		if (inftl_read_oob(mtd, (thisEUN * inftl->EraseSize) +
				  blockofs, 8, &retlen, (char *)&bci) < 0)
			status = SECTOR_IGNORE;
		else
			status = bci.Status | bci.Status1;

		switch (status) {
		case SECTOR_DELETED:
			thisEUN = BLOCK_NIL;
			goto foundit;
		case SECTOR_USED:
			goto foundit;
		case SECTOR_FREE:
		case SECTOR_IGNORE:
			break;
		default:
			printk(KERN_WARNING "INFTL4: unknown status for "
				"block %ld in EUN %d: 0x%04x  page:%d\n",
				block, thisEUN, status,(thisEUN * inftl->EraseSize + blockofs)/SECTORSIZE + blockofs);
			break;
		}

		if (!silly--) {
			printk(KERN_WARNING "INFTL: infinite loop in "
				"Virtual Unit Chain 0x%lx\n",
				block / (inftl->EraseSize / SECTORSIZE));
			return 1;
		}

		thisEUN = inftl->PUtable[thisEUN];
	}
*/
foundit:
	if (thisEUN == BLOCK_NIL) {
		/* The requested block is not on the media, return all 0x00 */
		memset(buffer, 0, SECTORSIZE);
	} else {
		size_t retlen;
		loff_t ptr = (thisEUN * inftl->EraseSize) + blockofs;
//printk("INFTL: inftl_readblock(blockin=%ld, EUN:%d, blockout=%ld\n", block,thisEUN, ((thisEUN * inftl->EraseSize) + blockofs)/32768);

		int ret = mtd_read(mtd, ptr, SECTORSIZE, &retlen, buffer);

		/* Handle corrected bit flips gracefully */
		if (ret < 0 && !mtd_is_bitflip(ret))
			return -EIO;
	}
	return 0;
}

static int inftl_getgeo(struct mtd_blktrans_dev *dev, struct hd_geometry *geo)
{
	struct INFTLrecord *inftl = (void *)dev;

	geo->heads = inftl->heads;
	geo->sectors = inftl->sectors;
	geo->cylinders = inftl->cylinders;

	return 0;
}

static struct mtd_blktrans_ops inftl_tr = {
	.name		= "inftl",
	.major		= INFTL_MAJOR,
	.part_bits	= INFTL_PARTN_BITS,
	.blksize 	= 512,
	.getgeo		= inftl_getgeo,
	.readsect	= inftl_readblock,
//	.writesect	= inftl_writeblock,
	.add_mtd	= inftl_add_mtd,
	.remove_dev	= inftl_remove_dev,
	.owner		= THIS_MODULE,
};

static int __init init_inftl(void)
{
	return register_mtd_blktrans(&inftl_tr);
}

static void __exit cleanup_inftl(void)
{
	deregister_mtd_blktrans(&inftl_tr);
}

module_init(init_inftl);
module_exit(cleanup_inftl);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Greg Ungerer <gerg@snapgear.com>, David Woodhouse <dwmw2@infradead.org>, Fabrice Bellard <fabrice.bellard@netgem.com> et al.");
MODULE_DESCRIPTION("Support code for Inverse Flash Translation Layer, used on M-Systems DiskOnChip 2000, Millennium and Millennium Plus");



