/*
 *---------------------------------------------------------------------------
 *
 * rootdisk.c
 *
 * Linux block device driver for mapping a rootdisk in memory.
 *
 * Copyright (c) 2009-2011 Motorola, Inc.
 * Copyright (c) 2012 Motorola Mobility, Inc.
 * Copyright (c) 2013-2014 ARRIS Enterprises, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * GPLv2 license:
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 *---------------------------------------------------------------------------
 */

#include <linux/bio.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/pagemap.h>
#include <linux/blkdev.h>
#include <linux/genhd.h>
#include <linux/root_dev.h>

#define ROOTDISK_MEMORY 'm'
#define ROOTDISK_FLASH 'f'
#define ROOTDISK_BLOCK_SIZE 512

unsigned long rootdisk_offset = 0;
EXPORT_SYMBOL(rootdisk_offset);
unsigned long rootdisk_size = 0;
EXPORT_SYMBOL(rootdisk_size);
char rootdisk_type = ROOTDISK_FLASH;
EXPORT_SYMBOL(rootdisk_type);

struct rootdisk_dev {
        int size;
        u8 *data;
        short users;
        spinlock_t lock;
        struct request_queue *queue;
        struct gendisk *gd;
};

struct rootdisk_dev rdev;

static int rootdisk_open(struct block_device *bdev, fmode_t mode)
{
        bdev->bd_disk->private_data = &rdev;
        spin_lock(&rdev.lock);
        rdev.users++;
        spin_unlock(&rdev.lock);
        return 0;
}

static void rootdisk_transfer(struct rootdisk_dev *dev, unsigned long sector,
			      unsigned long nbytes, char *buffer, int dir)
{
        unsigned long offset = sector * ROOTDISK_BLOCK_SIZE;
        if (dir == WRITE) {
                memcpy(dev->data + offset, buffer, nbytes);
        }
        else {
                memcpy(buffer, dev->data + offset, nbytes);
        }
}

static void rootdisk_request(struct request_queue *q)
{
        struct request *req;
	req = blk_fetch_request(q);
	while (req != NULL) {
                struct rootdisk_dev *dev = req->rq_disk->private_data;
                if (unlikely(req->cmd_type != REQ_TYPE_FS)) {
                        __blk_end_request_cur(req, -EIO);
                        continue;
                }
                rootdisk_transfer(dev, blk_rq_pos(req), blk_rq_cur_bytes(req),
                                  req->buffer, rq_data_dir(req));
                if (!__blk_end_request_cur(req, 0)) {
			req = blk_fetch_request(q);
		}
        }
}

static int __init early_setup_rootdisk(char *str)
{
        if (sscanf(str,"%c:0x%lx@0x%lx",
                   &rootdisk_type, &rootdisk_size, &rootdisk_offset) != 3) {
                printk(KERN_WARNING "Invalid format of rootdisk parameter, "
		       "should be [mf]:<size>@<offset>\n");
        }
	return 0;
}
early_param("rootdisk", early_setup_rootdisk);

static struct block_device_operations rootdisk_bd_op = {
	.owner = THIS_MODULE,
	.open =	rootdisk_open,
};

static int __init rootdisk_init(void)
{
        if (rootdisk_type == 'm') {
                memset(&rdev, 0, sizeof(struct rootdisk_dev));
                spin_lock_init(&rdev.lock);
                rdev.queue = blk_init_queue(rootdisk_request, &rdev.lock);
                rdev.data = (u8*)__phys_to_virt(rootdisk_offset);
                rdev.size = rootdisk_size;

                rdev.gd = alloc_disk(1);
                if(rdev.gd==0) {
                  printk(KERN_ERR "couldn't allocate ramdisk\n");
                }

                if (register_blkdev(RAMDISK_MAJOR, "rootdisk") < 0) {
                  printk(KERN_WARNING "ramdisk: unable to get major number\n");
                  return -EIO;
                }

                rdev.gd->major = RAMDISK_MAJOR;
                rdev.gd->first_minor = 0;
                rdev.gd->fops = &rootdisk_bd_op;
                rdev.gd->queue = rdev.queue;
                rdev.gd->private_data = &rdev;
                snprintf(rdev.gd->disk_name, 32, "rootdisk%d", 0);
                blk_queue_logical_block_size(rdev.queue, ROOTDISK_BLOCK_SIZE);
                set_capacity(rdev.gd, rootdisk_size / ROOTDISK_BLOCK_SIZE);
                add_disk(rdev.gd);
                printk(KERN_DEBUG "Added KreaTV rootdisk, %s (0x%08x@0x%p)\n",
                       rdev.gd->disk_name, rdev.size, rdev.data);
        }
        return 0;
}

static void __exit rootdisk_cleanup(void)
{
	// TODO
}

module_init(rootdisk_init);
module_exit(rootdisk_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jonas Widén <jonas.widen@arris.com>");
MODULE_DESCRIPTION("Linux block device driver for mapping a rootdisk in memory");
