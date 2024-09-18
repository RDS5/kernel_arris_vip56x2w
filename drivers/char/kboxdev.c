// -*- Mode: C -*-
//
// Copyright (c) 2003 Kreatel Communications AB
// Copyright (c) 2008-2010 Motorola, Inc.
// Copyright (c) 2012 Motorola Mobility, Inc.
// Copyright (c) 2013-2016 ARRIS Enterprises, Inc.
// Copyright (c) 2016 ARRIS Enterprises, LLC.
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. The name of the author may not be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// Alternatively, this software may be distributed under the terms of the
// GNU General Public License ("GPL") version 2 as published by the Free
// Software Foundation.
//
// THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
// OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
// IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
// NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// GPLv2 license:
// This program is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License version 2 as published
// by the Free Software Foundation.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/init.h>
#include <linux/sysfs.h>

/* Declared and allocated in arch/arm/mm/init.c */
extern void *video_memory;
extern unsigned int video_memory_size;

#include <linux/memblock.h>
#include <asm/setup.h>

static struct proc_dir_entry* kreatv_mem_entry = NULL;
static struct proc_dir_entry* kreatv_fb_entry = NULL;
#if defined(CONFIG_KBOX_RESERVE_SECURE_PCIE_MEMORY)
static struct proc_dir_entry* kreatv_pcie_entry = NULL;
#endif

#if defined(CONFIG_KBOX_SECONDARY_FB_MEMORY)
extern void *secondary_fb_memory;
extern unsigned int secondary_fb_memory_size;
#endif

#if defined(CONFIG_KBOX_RESERVE_SECURE_PCIE_MEMORY)
extern phys_addr_t secure_pcie_memory;
extern unsigned int secure_pcie_memory_size;
#endif

static int
kreatv_mem_proc_show(struct seq_file *m, void *v)
{
	unsigned int size = video_memory_size;

	if (!video_memory) {
		size = 0;
	}

	seq_printf(m, "Reserved API memory: 0x%08x@0x%08x",
		   size, (unsigned int)video_memory);
	return 0;
}

#if defined(CONFIG_KBOX_SECONDARY_FB_MEMORY)
static int kreatv_fb_proc_show(struct seq_file *m, void *v)
{
	unsigned int size = secondary_fb_memory_size;

	if (!secondary_fb_memory) {
		size = 0;
	}

	seq_printf(m, "Reserved FB memory: 0x%08x@0x%08x",
		   size, (unsigned int)secondary_fb_memory);
	return 0;
}
#endif

#if defined(CONFIG_KBOX_RESERVE_SECURE_PCIE_MEMORY)
static int kreatv_pcie_proc_show(struct seq_file *m, void *v)
{
	unsigned int size = secure_pcie_memory_size;

	if (!secure_pcie_memory) {
		size = 0;
	}

	seq_printf(m, "Reserved PCIe memory: 0x%08x@0x%08x",
		   size, secure_pcie_memory);
	return 0;
}
#endif

static int kreatv_mem_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, kreatv_mem_proc_show, NULL);
}

#if defined(CONFIG_KBOX_SECONDARY_FB_MEMORY)
static int kreatv_fb_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, kreatv_fb_proc_show, NULL);
}
#endif

#if defined(CONFIG_KBOX_RESERVE_SECURE_PCIE_MEMORY)
static int kreatv_pcie_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, kreatv_pcie_proc_show, NULL);
}
#endif

static const struct file_operations kreatv_mem_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= kreatv_mem_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#if defined(CONFIG_KBOX_SECONDARY_FB_MEMORY)
static const struct file_operations kreatv_fb_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= kreatv_fb_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif
#if defined(CONFIG_KBOX_RESERVE_SECURE_PCIE_MEMORY)
static const struct file_operations kreatv_pcie_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= kreatv_pcie_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif

int __init
kboxdev_init(void)
{
	kreatv_mem_entry =
		proc_create("kreatv_mem", 0, NULL, &kreatv_mem_proc_fops);
#if defined(CONFIG_KBOX_SECONDARY_FB_MEMORY)
	kreatv_fb_entry =
		proc_create("kreatv_fb", 0, NULL, &kreatv_fb_proc_fops);
#endif
#if defined(CONFIG_KBOX_RESERVE_SECURE_PCIE_MEMORY)
	if (secure_pcie_memory && secure_pcie_memory_size > 0) {
		kreatv_pcie_entry =
			proc_create("kreatv_pcie", 0, NULL,
				    &kreatv_pcie_proc_fops);
	}
#endif
	return 0;
}

static void __exit
kboxdev_cleanup(void)
{
	if (kreatv_mem_entry) {
		proc_remove(kreatv_mem_entry);
	}
	if (kreatv_fb_entry) {
		proc_remove(kreatv_fb_entry);
	}
#if defined(CONFIG_KBOX_RESERVE_SECURE_PCIE_MEMORY)
	if (kreatv_pcie_entry) {
		proc_remove(kreatv_pcie_entry);
	}
#endif
}

module_init(kboxdev_init);
module_exit(kboxdev_cleanup);

MODULE_AUTHOR("Fredrik Hallenberg <fredrik.hallenberg@arris.com>");
MODULE_DESCRIPTION("Handle proc entries on KreaTV VIP STBs");
MODULE_LICENSE("GPL");
