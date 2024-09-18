/*
 * Driver for Broadcom BCM7xxx GPIO unit (pinctrl + GPIO)
 *
 * Copyright (c) 2015-2016 ARRIS Enterprises, Inc. All rights reserved.
 *
 * This driver is inspired by:
 * pinctrl-bcm2853.c, please see original file for copyright information
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

#include <linux/bitmap.h>
#include <linux/brcmstb/brcmstb.h>
#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include "core.h"

#define MODULE_NAME "pinctrl-bcm7xxx"

/* GPIO register offsets */
#define GPDATA		0x04	/* Pin Level */
#define GPIODIR		0x08	/* IO direction */

struct bcm7xxx_pinctrl {
	struct device *dev;
	unsigned base;
	struct pinctrl_dev *pctl_dev;
	struct gpio_chip gpio_chip;
	struct pinctrl_gpio_range gpio_range;
};

enum bcm7xxx_fsel {
	BCM7xxx_FSEL_GPIO_IN = 0,
	BCM7xxx_FSEL_GPIO_OUT = 1,
	BCM7xxx_FSEL_COUNT = 2,
	BCM7xxx_FSEL_MASK = 0x1,
};

static const char * const bcm7xxx_functions[BCM7xxx_FSEL_COUNT] = {
	[BCM7xxx_FSEL_GPIO_IN] = "gpio_in",
	[BCM7xxx_FSEL_GPIO_OUT] = "gpio_out"
};

static inline int bcm7xxx_gpio_get_bit(struct bcm7xxx_pinctrl *pc, unsigned reg,
				       unsigned bit)
{
	return (BDEV_RD(pc->base + reg) >> bit) & 1;
}

static u32 bcm7xxx_gpio_get_pin(struct gpio_chip *chip, unsigned offset)
{
	struct bcm7xxx_pinctrl *pc = dev_get_drvdata(chip->dev);
	return pc->pctl_dev->desc->pins[offset].number;
}

static inline void bcm7xxx_gpio_set_bit(struct bcm7xxx_pinctrl *pc,
					unsigned reg, unsigned bit, unsigned val)
{
	unsigned v = BDEV_RD(pc->base + reg);
	val ? (v |= BIT(bit)) : (v &= ~BIT(bit));
	BDEV_WR(pc->base + reg, v);
}

static inline void bcm7xxx_pinctrl_fsel_set(
	struct bcm7xxx_pinctrl *pc,
	unsigned pin,
	enum bcm7xxx_fsel fsel)
{
	bcm7xxx_gpio_set_bit(pc, GPIODIR,
			     bcm7xxx_gpio_get_pin(&pc->gpio_chip, pin),
			     fsel == BCM7xxx_FSEL_GPIO_IN ? 1 : 0);
}

static int bcm7xxx_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	return pinctrl_request_gpio(chip->base + offset);
}

static void bcm7xxx_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	pinctrl_free_gpio(chip->base + offset);
}

static int bcm7xxx_gpio_direction(struct gpio_chip *chip, unsigned offset)
{
	struct bcm7xxx_pinctrl *pc = dev_get_drvdata(chip->dev);
	if (bcm7xxx_gpio_get_bit(pc, GPIODIR,
				 bcm7xxx_gpio_get_pin(&pc->gpio_chip, offset)))
		return GPIOF_DIR_IN;
	else
		return GPIOF_DIR_OUT;
}

static int bcm7xxx_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	return pinctrl_gpio_direction_input(chip->base + offset);
}

static int bcm7xxx_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct bcm7xxx_pinctrl *pc = dev_get_drvdata(chip->dev);
	return bcm7xxx_gpio_get_bit(pc, GPDATA, bcm7xxx_gpio_get_pin(chip, offset));
}

static int bcm7xxx_gpio_direction_output(struct gpio_chip *chip,
					 unsigned offset, int value)
{
	struct bcm7xxx_pinctrl *pc = dev_get_drvdata(chip->dev);
	bcm7xxx_gpio_set_bit(pc, GPDATA,
			     bcm7xxx_gpio_get_pin(chip, offset), value);
	return pinctrl_gpio_direction_output(chip->base + offset);
}

static void bcm7xxx_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct bcm7xxx_pinctrl *pc = dev_get_drvdata(chip->dev);
	bcm7xxx_gpio_set_bit(pc, GPDATA,
			     bcm7xxx_gpio_get_pin(chip, offset), value);
}

static int bcm7xxx_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	return -ENOTSUPP;
}

static struct gpio_chip bcm7xxx_gpio_chip = {
	.label = MODULE_NAME,
	.owner = THIS_MODULE,
	.request = bcm7xxx_gpio_request,
	.free = bcm7xxx_gpio_free,
	.get_direction = bcm7xxx_gpio_direction,
	.direction_input = bcm7xxx_gpio_direction_input,
	.direction_output = bcm7xxx_gpio_direction_output,
	.get = bcm7xxx_gpio_get,
	.set = bcm7xxx_gpio_set,
	.to_irq = bcm7xxx_gpio_to_irq,
	.base = -1,
	.ngpio = -1,
	.can_sleep = false,
};

static int bcm7xxx_pctl_get_groups_count(struct pinctrl_dev *pctldev)
{
	return 0;
}

static const char *bcm7xxx_pctl_get_group_name(struct pinctrl_dev *pctldev,
					       unsigned selector)
{
	return "";
}

static int bcm7xxx_pctl_get_group_pins(struct pinctrl_dev *pctldev,
				       unsigned selector,
				       const unsigned **pins,
				       unsigned *num_pins)
{
	return 0;
}

static void bcm7xxx_pctl_pin_dbg_show(struct pinctrl_dev *pctldev,
				      struct seq_file *s,
				      unsigned offset)
{
	seq_printf(s, "%s", dev_name(pctldev->dev));
}

static void bcm7xxx_pctl_dt_free_map(struct pinctrl_dev *pctldev,
				     struct pinctrl_map *maps, unsigned num_maps)
{
	int i;
	for (i = 0; i < num_maps; i++)
		if (maps[i].type == PIN_MAP_TYPE_CONFIGS_PIN)
			kfree(maps[i].data.configs.configs);

	kfree(maps);
}

static int bcm7xxx_pctl_dt_node_to_map(struct pinctrl_dev *pctldev,
				       struct device_node *np,
				       struct pinctrl_map **map, unsigned *num_maps)
{
	return 0;
}

static const struct pinctrl_ops bcm7xxx_pctl_ops = {
	.get_groups_count = bcm7xxx_pctl_get_groups_count,
	.get_group_name = bcm7xxx_pctl_get_group_name,
	.get_group_pins = bcm7xxx_pctl_get_group_pins,
	.pin_dbg_show = bcm7xxx_pctl_pin_dbg_show,
	.dt_node_to_map = bcm7xxx_pctl_dt_node_to_map,
	.dt_free_map = bcm7xxx_pctl_dt_free_map,
};

static int bcm7xxx_pmx_get_functions_count(struct pinctrl_dev *pctldev)
{
	return BCM7xxx_FSEL_COUNT;
}

static const char *bcm7xxx_pmx_get_function_name(struct pinctrl_dev *pctldev,
						 unsigned selector)
{
	return bcm7xxx_functions[selector];
}

static int bcm7xxx_pmx_get_function_groups(struct pinctrl_dev *pctldev,
					   unsigned selector,
					   const char * const **groups,
					   unsigned * const num_groups)
{
	return 0;
}

static int bcm7xxx_pmx_enable(struct pinctrl_dev *pctldev,
			      unsigned func_selector,
			      unsigned group_selector)
{
	struct bcm7xxx_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	bcm7xxx_pinctrl_fsel_set(pc, group_selector, func_selector);
	return 0;
}

static void bcm7xxx_pmx_disable(struct pinctrl_dev *pctldev,
				unsigned func_selector,
				unsigned group_selector)
{
	struct bcm7xxx_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);

	/* disable by setting to GPIO_IN */
	bcm7xxx_pinctrl_fsel_set(pc, group_selector, BCM7xxx_FSEL_GPIO_IN);
}

static void bcm7xxx_pmx_gpio_disable_free(struct pinctrl_dev *pctldev,
					  struct pinctrl_gpio_range *range,
					  unsigned offset)
{
	struct bcm7xxx_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	bcm7xxx_pinctrl_fsel_set(pc, offset, BCM7xxx_FSEL_GPIO_IN);
}

static int bcm7xxx_pmx_gpio_set_direction(struct pinctrl_dev *pctldev,
					  struct pinctrl_gpio_range *range,
					  unsigned offset,
					  bool input)
{
	struct bcm7xxx_pinctrl *pc = pinctrl_dev_get_drvdata(pctldev);
	enum bcm7xxx_fsel fsel = input ?
		BCM7xxx_FSEL_GPIO_IN : BCM7xxx_FSEL_GPIO_OUT;

	bcm7xxx_pinctrl_fsel_set(pc, offset, fsel);

	return 0;
}

static const struct pinmux_ops bcm7xxx_pmx_ops = {
	.get_functions_count = bcm7xxx_pmx_get_functions_count,
	.get_function_name = bcm7xxx_pmx_get_function_name,
	.get_function_groups = bcm7xxx_pmx_get_function_groups,
	.enable = bcm7xxx_pmx_enable,
	.disable = bcm7xxx_pmx_disable,
	.gpio_disable_free = bcm7xxx_pmx_gpio_disable_free,
	.gpio_set_direction = bcm7xxx_pmx_gpio_set_direction,
};

static int bcm7xxx_pinconf_get(struct pinctrl_dev *pctldev,
			       unsigned pin, unsigned long *config)
{
	return -ENOTSUPP;
}

static int bcm7xxx_pinconf_set(struct pinctrl_dev *pctldev,
			       unsigned pin, unsigned long *configs,
			       unsigned num_configs)
{
	return -ENOTSUPP;
}

static const struct pinconf_ops bcm7xxx_pinconf_ops = {
	.pin_config_get = bcm7xxx_pinconf_get,
	.pin_config_set = bcm7xxx_pinconf_set,
};

static int bcm7xxx_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct bcm7xxx_pinctrl *pc;
	struct resource iomem;
	int err, i;
	u32 offset, pin, first_pin, num_pins;
	struct pinctrl_pin_desc *bcm7xxx_gpio_pins;
	struct pinctrl_desc *bcm7xxx_pinctrl_desc;
	struct pinctrl_gpio_range bcm7xxx_pinctrl_gpio_range = {
		.name = MODULE_NAME,
	};

	pc = devm_kzalloc(dev, sizeof(*pc), GFP_KERNEL);
	if (!pc) {
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, pc);
	pc->dev = dev;

	err = of_address_to_resource(np, 0, &iomem);
	if (err) {
		dev_err(dev, "could not get IO memory\n");
		return err;
	}

	pc->base = iomem.start;

	if (of_property_read_u32(np, "brcm,gpio_offset", &offset)) {
		dev_err(pc->dev, "%s: missing brcm,gpio_offset property\n",
			of_node_full_name(np));
		return -EINVAL;
	}

	if (!of_find_property(np, "brcm,pin_group", NULL)) {
		dev_err(pc->dev, "%s: missing brcm,pin_group property\n",
			of_node_full_name(np));
		return -EINVAL;
	}

	if (of_property_read_u32_index(np, "brcm,pin_group", 0, &first_pin)) {
		dev_err(pc->dev, "%s: invalid brcm,pin_group property\n",
			of_node_full_name(np));
		return -EINVAL;
	}
	if (of_property_read_u32_index(np, "brcm,pin_group", 1, &num_pins)) {
		dev_err(pc->dev, "%s: invalid brcm,pin_group property\n",
			of_node_full_name(np));
		return -EINVAL;
	}

	bcm7xxx_gpio_pins = devm_kzalloc(dev, sizeof(struct pinctrl_pin_desc) * num_pins,
					GFP_KERNEL);
	if (bcm7xxx_gpio_pins == NULL) {
		return -ENOMEM;
	}
	for (i = 0, pin = first_pin; i < num_pins; i++, pin++) {
		bcm7xxx_gpio_pins[i].number = pin;
		bcm7xxx_gpio_pins[i].name = devm_kasprintf(dev, GFP_KERNEL, "gpio_%d",
							   offset+i);
	}

	pc->gpio_chip = bcm7xxx_gpio_chip;
	pc->gpio_chip.base = offset;
	pc->gpio_chip.ngpio = num_pins;
	pc->gpio_chip.dev = dev;
	pc->gpio_chip.of_node = np;

	err = gpiochip_add(&pc->gpio_chip);
	if (err) {
		dev_err(dev, "could not add GPIO chip\n");
		return err;
	}

	bcm7xxx_pinctrl_desc = (struct pinctrl_desc *)devm_kzalloc(dev, sizeof(struct pinctrl_desc),
								GFP_KERNEL);
	if (bcm7xxx_pinctrl_desc == NULL) {
		return -ENOMEM;
	}
	bcm7xxx_pinctrl_desc->name = MODULE_NAME;
	bcm7xxx_pinctrl_desc->pctlops = &bcm7xxx_pctl_ops;
	bcm7xxx_pinctrl_desc->pmxops = &bcm7xxx_pmx_ops;
	bcm7xxx_pinctrl_desc->confops = &bcm7xxx_pinconf_ops;
	bcm7xxx_pinctrl_desc->owner = THIS_MODULE;
	bcm7xxx_pinctrl_desc->pins = bcm7xxx_gpio_pins;
	bcm7xxx_pinctrl_desc->npins = num_pins;

	pc->pctl_dev = pinctrl_register(bcm7xxx_pinctrl_desc, dev, pc);
	if (!pc->pctl_dev) {
		gpiochip_remove(&pc->gpio_chip);
		return -EINVAL;
	}

	bcm7xxx_pinctrl_gpio_range.base = offset;
	bcm7xxx_pinctrl_gpio_range.pin_base = bcm7xxx_gpio_pins[0].number;
	bcm7xxx_pinctrl_gpio_range.npins = num_pins;
	bcm7xxx_pinctrl_gpio_range.gc = &pc->gpio_chip;
	pc->gpio_range = bcm7xxx_pinctrl_gpio_range;
	pinctrl_add_gpio_range(pc->pctl_dev, &pc->gpio_range);

	return 0;
}

static int bcm7xxx_pinctrl_remove(struct platform_device *pdev)
{
	struct bcm7xxx_pinctrl *pc = platform_get_drvdata(pdev);

	pinctrl_unregister(pc->pctl_dev);
	gpiochip_remove(&pc->gpio_chip);

	return 0;
}

static struct of_device_id bcm7xxx_pinctrl_match[] = {
	{ .compatible = "brcm,bcm7xxx-gpio" },
	{}
};
MODULE_DEVICE_TABLE(of, bcm7xxx_pinctrl_match);

static struct platform_driver bcm7xxx_pinctrl_driver = {
	.probe = bcm7xxx_pinctrl_probe,
	.remove = bcm7xxx_pinctrl_remove,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = bcm7xxx_pinctrl_match,
	},
};
module_platform_driver(bcm7xxx_pinctrl_driver);

MODULE_AUTHOR("Roger Björegren");
MODULE_DESCRIPTION("BCM7xxx Pin control driver");
MODULE_LICENSE("GPL");
