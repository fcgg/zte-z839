/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/spmi.h>
#include <linux/err.h>

#define VMBMS_BATT_PRES_STATUS     0x1208
#define BAT_PRES_BIT		       BIT(7)

struct spmi_lite {
	struct spmi_device *spmi;
	u16                 reg_bat_pres;
};

struct spmi_lite *chip = NULL;

static int qpnp_read_wrapper(struct spmi_lite *chip, u8 *val,
					u16 base, int count)
{
	int rc;
	struct spmi_device *spmi = chip->spmi;

	rc = spmi_ext_register_readl(spmi->ctrl, spmi->sid, base, val, count);
	if (rc)
		pr_err("SPMI read failed rc=%d\n", rc);

	return rc;
}
#if 0
static int qpnp_write_wrapper(struct spmi_lite *chip, u8 *val,
			u16 base, int count)
{
	int rc;
	struct spmi_device *spmi = chip->spmi;

	rc = spmi_ext_register_writel(spmi->ctrl, spmi->sid, base, val, count);
	if (rc)
		pr_err("SPMI write failed rc=%d\n", rc);

	return rc;
}
#endif

bool spmi_lite_is_battery_present(void)
{
      int rc;
      u8 batt_pres;

      rc = qpnp_read_wrapper(chip, &batt_pres,
				chip->reg_bat_pres, 1);
	  pr_debug("reg_bat_pres=%d\n", batt_pres);
      if (!rc && (batt_pres & BAT_PRES_BIT))
            return true;
      else
            return false;
}

static int spmi_lite_probe(struct spmi_device *spmi)
{
	pr_info("%s enter\n", __func__);   

	chip = devm_kzalloc(&spmi->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->spmi = spmi;
	chip->reg_bat_pres = VMBMS_BATT_PRES_STATUS;

	pr_info("batt_present=%d\n",spmi_lite_is_battery_present());
	return 0;
}

static int spmi_lite_remove(struct spmi_device *spmi)
{

	return 0;
}

static struct of_device_id spmi_match_table[] = {
	{	.compatible = "zte,spmi-lite",
	},
	{}
};

static struct spmi_driver spmi_lite_driver = {
	.driver		= {
		.name	= "zte,spmi-lite",
		.of_match_table = spmi_match_table,
	},
	.probe		= spmi_lite_probe,
	.remove		= spmi_lite_remove,
};

static int __init spmi_lite_init(void)
{
	return spmi_driver_register(&spmi_lite_driver);
}
fs_initcall(spmi_lite_init);

static void __exit spmi_lite_exit(void)
{
	return spmi_driver_unregister(&spmi_lite_driver);
}
module_exit(spmi_lite_exit);

MODULE_DESCRIPTION("zte spmi lite driver");
MODULE_LICENSE("GPL v2");

