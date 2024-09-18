/*
 * Driver for Broadcom BCM7xxx PINMUX unit
 *
 * Copyright (c) 2015 ARRIS Enterprises, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#include <linux/brcmstb/brcmstb.h>

#define MODULE_NAME "pinmux-bcm7xxx"

static int pmux_setup(struct platform_device *pdev, bool standby)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct property *pp;
	struct device_node *pins;
	u32 pinmux_address;
	u32 pinmux_current_settings;
	u32 pinmux_function;
	u32 pinmux_standby_function;
	u32 pinmux_mask;
	u32 pinmux_shift;

	pins = of_get_child_by_name(np, "brcm,pmux-setup");

	if (pins) {
		for_each_property_of_node(pins, pp) {
			const __be32 *values = pp->value;
			if (!strcmp(pp->name, "name") ||
				pp->length / sizeof(u32) != 4) {
				continue;
			}
			pinmux_address = be32_to_cpu(values[0]);
			pinmux_address |= BCHP_PHYSICAL_OFFSET;
			pinmux_shift = be32_to_cpu(values[1]);
			pinmux_function = be32_to_cpu(values[2]);
			pinmux_standby_function = be32_to_cpu(values[3]);

			pinmux_current_settings = BDEV_RD(pinmux_address);
			pinmux_mask = 0xf << pinmux_shift;
			if (standby) {
				pinmux_function = (0xf & pinmux_standby_function) << pinmux_shift;
			}
			else {
				pinmux_function = (0xf & pinmux_function) << pinmux_shift;
			}

			BDEV_WR(pinmux_address,
				(pinmux_current_settings & ~pinmux_mask) | pinmux_function);
		}
	}
	return 0;
}

static int bcm7xxx_pinmux_probe(struct platform_device *pdev)
{
	return pmux_setup(pdev, false);
}

#ifdef CONFIG_PM
static int bcm7xxx_pinmux_suspend(struct platform_device *pdev, pm_message_t state)
{
	pmux_setup(pdev, true);
	return 0;
}

static int bcm7xxx_pinmux_resume(struct platform_device *pdev)
{
	pmux_setup(pdev, false);
	return 0;
}
#endif /* CONFIG_PM */

static struct of_device_id bcm7xxx_pinmux_match[] = {
	{ .compatible = "brcm,bcm7xxx-pinmux" },
	{}
};
MODULE_DEVICE_TABLE(of, bcm7xxx_pinmux_match);

static struct platform_driver bcm7xxx_pinmux_driver = {
	.probe = bcm7xxx_pinmux_probe,
#ifdef CONFIG_PM
	.suspend = bcm7xxx_pinmux_suspend,
	.resume = bcm7xxx_pinmux_resume,
#endif
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = bcm7xxx_pinmux_match,
	},
};
module_platform_driver(bcm7xxx_pinmux_driver);

MODULE_AUTHOR("ARRIS Enterprises, LLC");
MODULE_DESCRIPTION("BCM7xxx Pin mux setup");
MODULE_LICENSE("GPL");
