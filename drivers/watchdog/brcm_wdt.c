/*
 *	Broadcom watchdog.
 *
 *	Based on wdt.c.
 *
 *	Original copyright messages:
 *
 *      (c) Copyright 1996-1997 Alan Cox <alan@redhat.com>, All Rights Reserved.
 *                              http://www.redhat.com
 *
 *      This program is free software; you can redistribute it and/or
 *      modify it under the terms of the GNU General Public License
 *      as published by the Free Software Foundation; either version
 *      2 of the License, or (at your option) any later version.
 *
 *      Neither Alan Cox nor CymruNet Ltd. admit liability nor provide
 *      warranty for any of this software. This material is provided
 *      "AS-IS" and at no charge.
 *
 *      (c) Copyright 1995    Alan Cox <alan@lxorguk.ukuu.org.uk>
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <asm/io.h>


#define DEFAULT_TIMEOUT_SECOND 10
#define MIN_TIMEOUT_SECOND 1
#define MAX_TIMEOUT_SECOND 159

static unsigned int heartbeat = DEFAULT_TIMEOUT_SECOND;
module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat, "Watchdog heartbeat in seconds. (0<heartbeat<159, default=" __MODULE_STRING(DEFAULT_TIMEOUT_SECOND) ")");

static int nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default=CONFIG_WATCHDOG_NOWAYOUT)");

extern void show_registers(struct pt_regs *regs);
extern void show_regs(struct pt_regs *regs);

static struct timer_list warning_timer;

struct brcm_wdt_data {
	void __iomem		*wd_timeout_reg;
	void __iomem		*wd_cmd_reg;
	void __iomem		*wd_ctrl_reg;
}brcm_wdt;

static void print_warning(unsigned long ignored)
{
	struct pt_regs* u;
	struct task_struct* const p = current;

	printk(KERN_WARNING "brcmwdt: watchdog is hungry, will reboot unless fed very soon\n");
	task_lock(p);
	printk(KERN_CRIT "Do not trust kernel register info above (except pc and sp)\n");
	u = task_pt_regs(p);
	if (u) {
		printk(KERN_CRIT "Userspace registers are:\n");
		show_regs(u);
	}
	else {
		printk(KERN_CRIT "No userspace registers info found\n");
	}

	task_unlock(p);

	if (0) { /* debug only. should be disabled in production */
		p->exit_state = EXIT_ZOMBIE;
		while (1)
			schedule();
	}
}

static int brcm_wdt_start(struct watchdog_device *wdog)
{
	writel_relaxed(0, brcm_wdt.wd_ctrl_reg);
	writel_relaxed(27000000 * heartbeat, brcm_wdt.wd_timeout_reg);
	writel_relaxed(0xff00, brcm_wdt.wd_cmd_reg);
	writel_relaxed(0x00ff, brcm_wdt.wd_cmd_reg);

	// If the timeout is more than 3 seconds we add a timer to get a
	// warning 3 seconds before the watchdog reboots the box. With 3
	// seconds procman has a fair chance to send the log to any listening
	// logclient. It will also be printed on the serial console.
	if (heartbeat > 3) {
		warning_timer.expires = jiffies + HZ * (heartbeat - 3);
		add_timer(&warning_timer);
	}

	return 0;
}

static int brcm_wdt_stop(struct watchdog_device *wdog)
{
	del_timer(&warning_timer);

	writel_relaxed(0xee00, brcm_wdt.wd_cmd_reg);
	writel_relaxed(0x00ee, brcm_wdt.wd_cmd_reg);

	return 0;
}

static int brcm_wdt_keepalive(struct watchdog_device *wdog)
{
	brcm_wdt_stop(wdog);
	brcm_wdt_start(wdog);
	return 0;
}

static int brcm_wdt_set_timeout(struct watchdog_device *wdog, unsigned int t)
{
	wdog->timeout = heartbeat = t;
	return 0;
}

static struct watchdog_ops brcm_wdt_ops = {
	.owner =	THIS_MODULE,
	.start =	brcm_wdt_start,
	.stop  =	brcm_wdt_stop,
	.ping  =        brcm_wdt_keepalive,
	.set_timeout =	brcm_wdt_set_timeout,
};

static struct watchdog_info brcm_wdt_info = {
	.options =	WDIOF_SETTIMEOUT | WDIOF_MAGICCLOSE |
			WDIOF_KEEPALIVEPING,
	.identity =	"Broadcom Watchdog timer",
};

static struct watchdog_device brcm_wdt_wdd = {
	.info =		&brcm_wdt_info,
	.ops =		&brcm_wdt_ops,
	.min_timeout =	MIN_TIMEOUT_SECOND,
	.max_timeout =	MAX_TIMEOUT_SECOND,
	.timeout =	DEFAULT_TIMEOUT_SECOND,
};

static int brcm_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int err;
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "wd-timeout");
	brcm_wdt.wd_timeout_reg = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(brcm_wdt.wd_timeout_reg)) {
		pr_err("%s Ioremap resource(wd-timeout) not done\n", __func__);
		return PTR_ERR(brcm_wdt.wd_timeout_reg);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "wd-cmd");
	brcm_wdt.wd_cmd_reg = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(brcm_wdt.wd_cmd_reg)) {
		pr_err("%s Ioremap resource(wd-cmd) not done\n", __func__);
		return PTR_ERR(brcm_wdt.wd_cmd_reg);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "wd-ctrl");
	brcm_wdt.wd_ctrl_reg = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(brcm_wdt.wd_ctrl_reg)) {
		pr_err("%s Ioremap resource(wd-ctrl) not done\n", __func__);
		return PTR_ERR(brcm_wdt.wd_ctrl_reg);
	}

	watchdog_init_timeout(&brcm_wdt_wdd, heartbeat, dev);
	watchdog_set_nowayout(&brcm_wdt_wdd, nowayout);
	err = watchdog_register_device(&brcm_wdt_wdd);
	if (err) {
		dev_err(dev, "Failed to register watchdog device");
		return err;
	}

	printk(KERN_INFO "brcmwdt: Broadcom watchdog driver "
		"heartbeat=%d sec (nowayout=%d) "
		"watchdog register timer:cmd:ctrl(%p:%p:%p) \n",
		heartbeat, nowayout,
		brcm_wdt.wd_timeout_reg, brcm_wdt.wd_cmd_reg, brcm_wdt.wd_ctrl_reg);

	setup_timer(&warning_timer, print_warning, 0);

	return 0;
}

static int brcm_wdt_remove(struct platform_device *pdev)
{
	if (watchdog_active(&brcm_wdt_wdd))
		brcm_wdt_stop(&brcm_wdt_wdd);

	watchdog_unregister_device(&brcm_wdt_wdd);

	return 0;
}

static const struct of_device_id brcm_wdt_of_match[] = {
	{ .compatible = "brcm,brcmstb-watchdog", },
	{},
};
MODULE_DEVICE_TABLE(of, brcm_wdt_of_match);

#ifdef CONFIG_PM_SLEEP
static int brcm_wdt_suspend(struct device *dev)
{
	if (watchdog_active(&brcm_wdt_wdd))
		brcm_wdt_stop(&brcm_wdt_wdd);

	return 0;
}

static int brcm_wdt_resume(struct device *dev)
{
	if (watchdog_active(&brcm_wdt_wdd)) {
		brcm_wdt_start(&brcm_wdt_wdd);
	}

	return 0;
}

SIMPLE_DEV_PM_OPS(brcm_wdt_pm_ops, brcm_wdt_suspend, brcm_wdt_resume);
#define BRCM_WDT_PM (&brcm_wdt_pm_ops)
#else
#define BRCM_WDT_PM NULL
#endif

static struct platform_driver brcm_wdt_driver = {
	.probe		= brcm_wdt_probe,
	.remove		= brcm_wdt_remove,
	.driver = {
		.name   = "brcm-wdt",
		.owner  = THIS_MODULE,
		.pm     = BRCM_WDT_PM,
		.of_match_table = brcm_wdt_of_match,
	},
};

module_platform_driver(brcm_wdt_driver);

MODULE_AUTHOR("ARRIS Enterprises, LLC");
MODULE_DESCRIPTION("BRCM WDT");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_ALIAS_MISCDEV(TEMP_MINOR);
MODULE_LICENSE("Dual BSD/GPL");
