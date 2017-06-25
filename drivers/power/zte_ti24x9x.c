/* Copyright (c) 2013-2014 The Linux Foundation. All rights reserved.
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
#define pr_fmt(fmt) "[CHG] %s(%d): " fmt, __func__,__LINE__
//#define DEBUG
#define FEATURE_ZTE_LC709203F_FG   //defined for OnSemi bms
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/wakelock.h>
#include <linux/reboot.h>	/*zte add:For kernel_power_off() */

#include <linux/qpnp/qpnp-adc.h>
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_I2C
int syna_ts_notifier_call_chain(unsigned long val);
#endif

/*zte feature definition*/
#ifndef FEATURE_SOFT_CC
#define FEATURE_SOFT_CC 0
#define SOFT_CC_DEBUG 0
#endif
#define PRINT_TI24296_REG 1
#define _TIC_MASK(BITS, POS) \
	((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define TIC_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
		_TIC_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
				(RIGHT_BIT_POS))

/* Charger Registers */
#define INPUT_SOURCE_CTRL_REG		0x00
#define IINLIMIT_MASK				TIC_MASK(2, 0)
#define IINLIMIT_SHIFT				0
#define VINLIMIT_MASK				TIC_MASK(6, 3)
#define VINLIMIT_SHIFT				3
#define EN_HIZ_MASK					TIC_MASK(7,7)
#define EN_HIZ_SHIFT				7

#define PON_CFG_REG					0x01
//bit 0:  2419x: 0-500mA, 1-1.3A; 2x29x: 0-1A, 1-1.5A
#define BOOST_LIM_MASK				TIC_MASK(0,0)
#define BOOST_LIM_SHIFT				0
#define V_SYS_MIN_MASK				TIC_MASK(3,1)
#define V_SYS_MIN_SHIFT				1
#define CHG_CFG_MASK				TIC_MASK(5,4)
#define CHG_CFG_SHIFT				4
#define I2C_WDOG_RESET_MASK			TIC_MASK(6,6)
#define I2C_WDOG_RESET_SHIFT		6
#define REGISTER_RESET_MASK			TIC_MASK(7,7)
#define REGISTER_RESET_SHIFT		7

#define CHG_I_CTRL_REG				0x02
//different between 2419x and 2429x, bit 1
#define FORCE_20PCT_MASK			TIC_MASK(0,0)
#define FORCE_20PCT_SHIFT			0
#define IFAST_CHG_MASK				TIC_MASK(7,2)
#define IFAST_CHG_SHIFT				2
//for 2429x
#define TI2429X_BOOST_COLD_MASK		TIC_MASK(1,1)
#define TI2429X_BOOST_COLD_SHIFT	1

#define IPRECHG_ITERM_CTRL_REG		0x03
#define ITERM_MASK					TIC_MASK(3,0)
#define ITERM_SHIFT					0
#define IPRECHG_MASK				TIC_MASK(7,4)
#define IPRECHG_SHIFT				4

#define VCHG_CTRL_REG				0x04
#define VRECHG_MASK					TIC_MASK(0,0)
#define VRECHG_SHIFT				0
#define V_BAT_LOW_THRE_MASK			TIC_MASK(1,1)			//Battery Precharge to Fast Charge Threshold
#define V_BAT_LOW_THRE_SHIFT		1
#define VCHG_MASK					TIC_MASK(7,2)
#define VCHG_SHIFT					2

#define CHG_TERM_TIMER_CTRL_REG		0x05
//different between 2419x and 2429x, bit 0, bit 6
#define JEITA_ISET_MASK				TIC_MASK(0,0)			//2429x not use
#define JEITA_ISET_SHIFT			0
#define FAST_CHG_TIMER_MASK			TIC_MASK(2,1)
#define FAST_CHG_TIMER_SHIFT		1
#define SAFE_TIMER_EN_MASK			TIC_MASK(3,3)
#define SAFE_TIMER_EN_SHIFT			3
#define I2C_WDOG_TIMER_MASK			TIC_MASK(5,4)
#define I2C_WDOG_TIMER_SHIFT		4
#define TERM_STAT_MASK				TIC_MASK(6,6)			//2429x not use
#define TERM_STAT_SHIFT				6
#define EN_TERM_MASK				TIC_MASK(7,7)
#define EN_TERM_SHIFT				7

#define IR_THERM_REG				0x06
//different between 2419x and 2429x, bit 2~7
#define THERM_THRE_MASK				TIC_MASK(1,0)
#define THERM_THRE_SHIFT			0
#define TI2419X_IR_COMP_V_MASK		TIC_MASK(4,2)
#define TI2419X_IR_COMP_V_SHIFT		2
#define TI2419X_IR_COMP_R_MASK		TIC_MASK(7,5)
#define TI2419X_IR_COMP_R_SHIFT		5
//for 2429x
#define TI2429X_BOOST_HOT_MASK		TIC_MASK(3,2)
#define TI2429X_BOOST_HOT_SHIFT		2
#define TI2429X_BOOST_V_MASK		TIC_MASK(7,4)
#define TI2429X_BOOST_V_SHIFT		4

#define MISC_OPERA_CTRL_REG			0x07
//different between 2419x and 2429x, bit 4
#define INT_MASK					TIC_MASK(1,0)
#define INT_SHIFT					0
#define JEITA_VSET_MASK				TIC_MASK(4,4)		//2429x not use
#define JEITA_VSET_SHIFT			4
#define BATFET_DISABLE_MASK			TIC_MASK(5,5)
#define BATFET_DISABLE_SHIFT		5
#define TMR2X_EN_MASK				TIC_MASK(6,6)
#define TMR2X_EN_SHIFT				6
#define DPDM_EN_MASK				TIC_MASK(7,7)
#define DPDM_EN_SHIFT				7

#define SYS_STAT_REG				0x08
#define VSYS_STAT_MASK				TIC_MASK(0,0)
#define VSYS_STAT_SHIFT				0
#define THERM_STAT_MASK				TIC_MASK(1,1)
#define THERM_STAT_SHIFT			1
#define PG_STAT_MASK				TIC_MASK(2,2)
#define PG_STAT_SHIFT				2
#define DPM_STAT_MASK				TIC_MASK(3,3)
#define DPM_STAT_SHIFT				3
#define CHG_STAT_MASK				TIC_MASK(5,4)
#define CHG_STAT_SHIFT				4
#define VBUS_STAT_MASK				TIC_MASK(7,6)
#define VBUS_STAT_SHIFT				6

#define BATT_NOT_CHG_VAL			0x0
#define BATT_PRE_CHG_VAL			0x1
#define BATT_FAST_CHG_VAL			0x2
#define BATT_CHG_DONE				0x3

#define FAULT_REG					0x09
//different between 2419x and 2429x, bit 0~2, bit 6
#define TI2419X_NTC_FAULT_MASK		TIC_MASK(2,0)
#define TI2419X_NTC_FAULT_SHIFT		0
#define BAT_FAULT_MASK				TIC_MASK(3,3)
#define BAT_FAULT_SHIFT				3
#define CHG_FAULT_MASK				TIC_MASK(5,4)
#define CHG_FAULT_SHIFT				4
#define TI2419X_BOOST_FAULT_MASK	TIC_MASK(6,6)
#define TI2419X_BOOST_FAULT_SHIFT	6
#define WDOG_FAULT_MASK				TIC_MASK(7,7)
#define WDOG_FAULT_SHIFT			7
//for 2429x
#define TI2429X_NTC_FAULT_MASK		TIC_MASK(1,0)
#define TI2429X_NTC_FAULT_SHIFT		0
#define TI2429X_OTG_FAULT_MASK		TIC_MASK(6,6)
#define TI2429X_OTG_FAULT_SHIFT		6

#define VENDOR_REG					0x0A
//for 2419x
#define TI2419X_DEV_REG_MASK		TIC_MASK(1,0)
#define TI2419X_TS_PROFILE_MASK		TIC_MASK(2,2)
#define TI2419X_PN_MASK				TIC_MASK(5,3)
//for 2429x
#define TI2429X_REVISION_MASK		TIC_MASK(2,0)
#define TI2429X_PN_MASK				TIC_MASK(7,5)

#define CHARGER_IC_2419X			0
#define CHARGER_IC_2429X			1

enum {
	USER	= BIT(0),
	THERMAL = BIT(1),
	CURRENT = BIT(2),
	TEMP = BIT(3),			//temperature
};

//ti chip charging status
enum {
	NOT_CHARGING,
	PRE_CHARGE,
	FAST_CHARGE,
	CHARGE_TERM_DONE,
};

//for usb-otg
struct ti2419x_otg_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct ti2419x_chip {
	struct i2c_client              *client;
	struct device                  *dev;
	unsigned short                  default_i2c_addr;

	/* configuration data - charger */
	int                             fake_battery_soc;  //soc set from user
	bool                            charging_disabled;
	bool                            hw_iterm_disabled;
	int                             iterm_ma;
	int                             vfloat_mv;
	int                             safety_time;
	int                             resume_delta_mv;
	int                             warm_resume_delta_mv;
	int                             vbatdet_max_err_mv;
	unsigned int                    thermal_levels;
	unsigned int                    therm_lvl_sel;
	unsigned int                    *thermal_mitigation;

	/* status tracking */
	bool                             usb_present;
	bool                             batt_present;    //how to check battery present
	bool                             bat_is_cool;     //zte
	bool                             bat_is_cold;     //zte
	bool                             bat_is_warm;
	bool                             bat_is_hot;      //zte
	unsigned int                     hot_batt_p;      //ZTE, for PMIC batt therm correcting
	unsigned int                     cold_batt_p;     //ZTE, for PMIC batt therm correcting

	bool                             batt_full;
	bool                             batt_warm_full;
	bool                             chg_done;
	bool                             resume_completed;
	bool                             irq_waiting;
	u8                               irq_cfg_mask[3];
	int                              usb_psy_ma;
	int                              charging_disabled_status;
	int                              max_iusb;
	int                              max_ibat;
	int                              max_input_voltage;

	int                              skip_writes;
	int                              skip_reads;
	u8                               reg_addr;
	struct dentry                   *debug_root;
#ifdef FEATURE_ZTE_LC709203F_FG
	struct qpnp_vadc_chip           *vadc_dev;
#endif
	struct power_supply             *usb_psy;
	struct power_supply              batt_psy;
	struct power_supply             *bms_psy;
	struct power_supply             *vm_bms_psy;
	struct ti2419x_otg_regulator     otg_vreg;    //for usb-otg
	struct mutex                     irq_complete;
	struct mutex                     charging_disable_lock;
	struct mutex                     current_change_lock;
	struct mutex                     read_write_lock;
	struct mutex                     jeita_configure_lock;

	struct delayed_work              update_heartbeat_work;
	struct delayed_work              charger_eoc_work;
	struct delayed_work              temp_control_work;
#if FEATURE_SOFT_CC
	struct delayed_work              soft_cc_monitor_work;  //zte add
#endif
	struct work_struct               poweroff_work;	        //ZTE
	struct wake_lock                 charger_wake_lock;     //zte add
	struct wake_lock                 charger_valid_lock;    //zte add
	int                              chargeIC_type;

	unsigned int                     warm_bat_mv;
	unsigned int                     cool_bat_mv;
	unsigned int                     warm_bat_chg_ma;
	unsigned int                     cool_bat_chg_ma;
	int                              warm_bat_decidegc;
	int                              cool_bat_decidegc;
	int                              hot_bat_decidegc;
	int                              cold_bat_decidegc;
	int                              low_temp_threshold;
	int                              high_temp_threshold;
	int                              health;
	int                              soc;
};

static int chg_time[] = {
	5,			//hours
	8,
	12,
	20,
};

static int input_current_limit[] = {
	100, 150, 500, 900, 1000, 1500, 2000, 3000,
};
static int ichg_current_limit[] = {
	512, 512, 512, 1152, 1152, 1664, 2048, 3008,
};

struct ti2419x_chip *the_ti2419x_chip =	NULL;

#if 0
static int is_between(int value, int left, int right)
{
	if (left >= right && left >= value && value >= right)
		return 1;
	if (left <= right && left <= value && value <= right)
		return 1;

	return 0;
}

static int bound(int val, int min, int max)
{
	if (val < min)
		return min;
	if (val > max)
		return max;

	return val;
}
#endif

/* ZTE-CHG-JZN add,20150907
*  when in poweroff charging, once the charger is absent, run kernel poweroff quickly
*/
extern int offcharging_flag;
static void
offcharge_poweroff_work(struct work_struct *work)
{
	printk("%s,ZTE shutdown for charger remove at offcharging mode \n",__func__); //XJB

	kernel_power_off();   //ZTE
	//kernel_restart(NULL);
}


static int __ti2419x_read(struct ti2419x_chip *chip, int reg,
				u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else {
		*val = ret;
	}

	return 0;
}

static int __ti2419x_write(struct ti2419x_chip *chip, int reg,
						u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	pr_debug("Writing 0x%02x=0x%02x\n", reg, val);
	return 0;
}

static int ti2419x_read(struct ti2419x_chip *chip, int reg,
				u8 *val)
{
	int rc;

	if (chip->skip_reads) {
		*val = 0;
		return 0;
	}
	mutex_lock(&chip->read_write_lock);
	rc = __ti2419x_read(chip, reg, val);
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

#if 0
static int ti2419x_write(struct ti2419x_chip *chip, int reg,
						u8 val)
{
	int rc;

	if (chip->skip_writes)
		return 0;

	mutex_lock(&chip->read_write_lock);
	rc = __ti2419x_write(chip, reg, val);
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

static int ti2419x_read_bytes(struct ti2419x_chip *chip, int reg,
						u8 *val, u8 bytes)
{
	s32 rc;

	if (chip->skip_reads) {
		*val = 0;
		return 0;
	}

	mutex_lock(&chip->read_write_lock);
	rc = i2c_smbus_read_i2c_block_data(chip->client, reg, bytes, val);
	if (rc < 0)
		dev_err(chip->dev,
			"i2c read fail: can't read %d bytes from %02x: %d\n",
							bytes, reg, rc);
	mutex_unlock(&chip->read_write_lock);

	return (rc < 0) ? rc : 0;
}
#endif

static int ti2419x_masked_write(struct ti2419x_chip *chip, int reg,
						u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	if (chip->skip_writes || chip->skip_reads)
		return 0;

	mutex_lock(&chip->read_write_lock);
	rc = __ti2419x_read(chip, reg, &temp);
	if (rc < 0) {
		dev_err(chip->dev, "read failed: reg=%03X, rc=%d\n", reg, rc);
		goto out;
	}
	temp &= ~mask;
	temp |= val & mask;
	rc = __ti2419x_write(chip, reg, temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"write failed: reg=%03X, rc=%d\n", reg, rc);
	}
out:
	mutex_unlock(&chip->read_write_lock);
	return rc;
}

static int
ti2419x_is_chg_plugged_in(struct ti2419x_chip *chip)
{
	int rc;
	u8 reg = 0;

	rc = ti2419x_read(chip, SYS_STAT_REG, &reg);
	if (rc) {
		pr_err("Couldn't read SYS_STAT_REG rc=%d\n", rc);
		return 0;
	}		

	pr_debug("chgr usb sts %d\n", (reg & PG_STAT_MASK) ? 1 : 0);

	return (reg & PG_STAT_MASK) ? 1 : 0;
}

#define MIN_FLOAT_MV		3504
#define MAX_FLOAT_MV		4400
#define VFLOAT_STEP_MV		16
static int ti2419x_float_voltage_set(struct ti2419x_chip *chip, int vfloat_mv)
{
	u8 temp;

	if ((vfloat_mv < MIN_FLOAT_MV) || (vfloat_mv > MAX_FLOAT_MV)) {
		dev_err(chip->dev, "bad float voltage mv =%d asked to set\n",
					vfloat_mv);
		return -EINVAL;
	}

	temp = (vfloat_mv - MIN_FLOAT_MV) / VFLOAT_STEP_MV;

	return ti2419x_masked_write(chip, VCHG_CTRL_REG,
				VCHG_MASK, temp << VCHG_SHIFT);
}

static int ti2419x_float_voltage_get(struct ti2419x_chip *chip, int *vfloat_mv)
{
	int rc;
	u8 reg = 0;
	int vol = 0;

	rc = ti2419x_read(chip, VCHG_CTRL_REG, &reg);
	if (rc) {
		pr_err("Couldn't read SYS_STAT_REG rc=%d\n", rc);
		return -1;
	}
	reg &= VCHG_MASK;
	reg >>= VCHG_SHIFT;

	vol = reg * VFLOAT_STEP_MV + MIN_FLOAT_MV;
	if ((vol < MIN_FLOAT_MV) || (vol > MAX_FLOAT_MV)) {
		dev_err(chip->dev, "bad float voltage mv =%d\n", vol);
		return -EINVAL;
	}

	*vfloat_mv = vol;

	return 0;
}

#define MIN_RECHG_MV		50
#define MAX_RECHG_MV		300
static int ti2419x_recharge_threshold_set(struct ti2419x_chip *chip,
							int resume_mv)
{
	u8 temp;

	if ((resume_mv < MIN_RECHG_MV) || (resume_mv > MAX_RECHG_MV)) {
		dev_err(chip->dev, "bad rechg_thrsh =%d asked to set\n",
							resume_mv);
		return -EINVAL;
	}

	temp = resume_mv / MAX_RECHG_MV;

	return ti2419x_masked_write(chip, VCHG_CTRL_REG,
		VRECHG_MASK, temp << VRECHG_SHIFT);
}

static int __ti2419x_charging_disable(struct ti2419x_chip *chip, bool disable)
{
	int rc;

	rc = ti2419x_masked_write(chip, PON_CFG_REG,
			CHG_CFG_MASK, disable ? 0 : 1 << CHG_CFG_SHIFT);
	if (rc < 0)
		pr_err("Couldn't set CHG_CFG disable=%d rc = %d\n",
							disable, rc);
	else
		pr_debug("CHG_CFG status=%d\n", !disable);

	return rc;
}

static int ti2419x_charging_disable(struct ti2419x_chip *chip, int reason,
								int disable)
{
	int rc = 0;
	int disabled;

	mutex_lock(&chip->charging_disable_lock);

	disabled = chip->charging_disabled_status;

	pr_info("reason=%d requested_disable=%d disabled_status=%d\n",
					reason, disable, disabled);

	if (disable == true)
		disabled |= reason;
	else
		disabled &= ~reason;

	if (disabled)
		rc = __ti2419x_charging_disable(chip, true);
	else
		rc = __ti2419x_charging_disable(chip, false);

	if (rc)
		pr_err("Couldn't disable charging for reason=%d rc=%d\n",
							rc, reason);
	else
		chip->charging_disabled_status = disabled;

	mutex_unlock(&chip->charging_disable_lock);

	return rc;
}

static void ti2419x_set_appropriate_float_voltage(struct ti2419x_chip *chip)
{
	pr_info("is_cold=%d is_cool=%d is_warm=%d cool_bat_mv=%dmv warm_bat_mv=%dmv vfloat_mv=%dmv warm_resume_delta_mv=%dmv resume_delta_mv=%dmv\n",
			chip->bat_is_cold,
			chip->bat_is_cool,
			chip->bat_is_warm,
			chip->cool_bat_mv,
			chip->warm_bat_mv,
			chip->vfloat_mv,
			chip->warm_resume_delta_mv,
			chip->resume_delta_mv
			);
	if (chip->bat_is_cool)
	{
		ti2419x_float_voltage_set(chip, chip->cool_bat_mv);
		ti2419x_recharge_threshold_set(chip,chip->warm_resume_delta_mv);
	}
	else if (chip->bat_is_warm)
	{
		ti2419x_float_voltage_set(chip, chip->warm_bat_mv);
		ti2419x_recharge_threshold_set(chip,chip->warm_resume_delta_mv);
	}
	else
	{
		ti2419x_float_voltage_set(chip, chip->vfloat_mv);
		ti2419x_recharge_threshold_set(chip,chip->resume_delta_mv);
	}
}

static enum power_supply_property ti2419x_battery_properties[] = {
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	//POWER_SUPPLY_PROP_RESISTANCE,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_TECHNOLOGY,

};

extern int hw_version;
extern bool spmi_lite_is_battery_present(void);
static int ti2419x_get_prop_batt_present(struct ti2419x_chip *chip)
{
#ifdef FEATURE_ZTE_LC709203F_FG

	#ifdef CONFIG_BOARD_ABBY
	/* Note(JZN)20151012:
	 * return TRUE for hw board error in hw version 0
	 */
	 //pr_debug("P890A57 hw_version:0x%02x\n",hw_version);
	if(hw_version == 0){
		chip->batt_present = 1;
		return chip->batt_present;
	 }
	#endif

	// check batt present by PMIC REG
	chip->batt_present = spmi_lite_is_battery_present();
	return  chip->batt_present;

#if 0
/* check batt present by BATT THERM,
     true: -28-65 degree; false: others
*/
	int rc = 0;
	struct qpnp_vadc_result results;
	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &results);
	if (rc) {
		pr_debug("%s ,Unable to read batt temperature rc=%d\n",__func__, rc);
		if(rc==-EPROBE_DEFER){
			chip->vadc_dev = qpnp_get_vadc(chip->dev, "tichg");
			if (IS_ERR(chip->vadc_dev)) {
				rc = PTR_ERR(chip->vadc_dev);
				if (rc != -EPROBE_DEFER)
					pr_err("%s ,vadc property missing\n",__func__);
				else
					pr_err("%s ,Unable to get battery therm VADC. rc=%d\n",__func__, rc);
			}
		}
		pr_err("%s ,failed to read batt temp vadc \n",__func__);
		return 0;
	}
	chip->batt_present	= ((int)results.physical>-280 && (int)results.physical <650 )?true:false;
	return chip->batt_present;
#endif
#else
// check batt present from bms_psy
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_PRESENT, &ret);

		chip->batt_present	=	ret.intval;
		return chip->batt_present;
	}

	return chip->batt_present;
#endif
}

/*
  * return -1: error
  * return 0: not charge
  * return 1: pre charge
  * return 2: fast charge
  * return 3: charge term
  */
static int ti2419x_get_charging_status(struct ti2419x_chip *chip)
{
	int rc;
	u8 reg = 0;

	rc = ti2419x_read(chip, SYS_STAT_REG, &reg);
	if (rc) {
		pr_err("Couldn't read SYS_STAT_REG rc=%d\n", rc);
		return -1;
	}

	reg &= CHG_STAT_MASK;
	reg >>= CHG_STAT_SHIFT;

	return reg;
}

static int ti2419x_get_prop_batt_capacity(struct ti2419x_chip *chip);
#define DEFAULT_STATUS 0 
static int ti2419x_get_prop_batt_bms_status(struct ti2419x_chip *chip)
{
#ifdef FEATURE_ZTE_LC709203F_FG
/*This parapeter is not supported by LC709203F_FG  */
#else
    union power_supply_propval ret = {0,};

       if (chip->bms_psy) {
               chip->bms_psy->get_property(chip->bms_psy,
                               POWER_SUPPLY_PROP_BMS_STATUS, &ret);

               return ret.intval;
       }
#endif
       return DEFAULT_STATUS;
}

static int ti2419x_get_prop_batt_bms_flags(struct ti2419x_chip *chip)
{
#ifdef FEATURE_ZTE_LC709203F_FG
/*This parapeter is not supported by LC709203F_FG  */

#else
    union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_BMS_FLAGS, &ret);

		return ret.intval;
	}
#endif
	return DEFAULT_STATUS;
}
static int ti2419x_get_prop_batt_bms_remainingcapacity(struct ti2419x_chip *chip)
{
#ifdef FEATURE_ZTE_LC709203F_FG
/*This parapeter is not supported by LC709203F_FG  */
#else
    union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_BMS_RC, &ret);

		return ret.intval;
	}
#endif
	return DEFAULT_STATUS;
}

static int ti2419x_get_prop_batt_bms_qmax(struct ti2419x_chip *chip)
{
#ifdef FEATURE_ZTE_LC709203F_FG
/*This parapeter is not supported by LC709203F_FG  */
#else
    union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_QMAX, &ret);

		return ret.intval;
	}
#endif
	return DEFAULT_STATUS;
}




#define DEFAULT_FCC 3000000 
static int ti2419x_get_prop_batt_bms_fcc(struct ti2419x_chip *chip)
{
    union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CHARGE_FULL, &ret);

		return ret.intval;
	}

	return DEFAULT_FCC;
}


static int ti2419x_get_prop_batt_status(struct ti2419x_chip *chip)
{
	int is_chg_in, stat;

	is_chg_in = ti2419x_is_chg_plugged_in(chip);
	stat      = ti2419x_get_charging_status(chip);
	if(stat < 0) {
		pr_err("Couldn't read SYS_STAT_REG.\n");
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	if(is_chg_in && (chip->soc == 100 || chip->batt_full))
	    return POWER_SUPPLY_STATUS_FULL;

	if(((stat==0x1) || (stat==0x2)) && is_chg_in)//prechg or fastchg
		return POWER_SUPPLY_STATUS_CHARGING;

	if(stat==0 || stat==0x3)//discharging or charge done
	    return POWER_SUPPLY_STATUS_DISCHARGING;

	return POWER_SUPPLY_STATUS_UNKNOWN;
}

static int ti2419x_is_charging_enabled(struct ti2419x_chip *chip)
{
	int rc;
	u8 reg = 0;

	rc = ti2419x_read(chip, PON_CFG_REG, &reg);
	if (rc) {
		pr_err("Couldn't read PON_CFG_REG rc=%d\n", rc);
		return 0;
	}

	reg &= CHG_CFG_MASK;
	reg >>= CHG_CFG_SHIFT;
	return (reg == 0x1) ? 1 : 0;
}

static int ti2419x_get_prop_charge_type(struct ti2419x_chip *chip)
{
	int rc;
	u8 reg = 0;
	u8 chg_type;

	rc = ti2419x_read(chip, SYS_STAT_REG, &reg);
	if (rc) {
		pr_err("Couldn't read SYS_STAT_REG rc=%d\n", rc);
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}

	chg_type = (reg & CHG_STAT_MASK) >> CHG_STAT_SHIFT;

	if (chg_type == BATT_NOT_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	else if ((chg_type == BATT_FAST_CHG_VAL) ||
			(chg_type == BATT_CHG_DONE))
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	else if (chg_type == BATT_PRE_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

//updated by temp control work
static int ti2419x_get_prop_batt_health(struct ti2419x_chip *chip)
{
	if (chip->bat_is_hot)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (chip->bat_is_cool)
		return POWER_SUPPLY_HEALTH_COOL;
	else if (chip->bat_is_cold)
		return POWER_SUPPLY_HEALTH_COLD;
	else if (chip->bat_is_warm)
		return POWER_SUPPLY_HEALTH_WARM;
	else
		return POWER_SUPPLY_HEALTH_GOOD;
}

//zte jiangfeng add
static int zero_reported = false;
//int enable_to_shutdown=1;    //ZTE add, export to zte_misc driver
extern int enable_to_shutdown;//defined in ****-charger.c
#define SHUTDOWN_VOLTAGE 3400000
#define LOW_TEMP_SHUTDOWN_VOLTAGE 3200000
int ti2419x_get_prop_voltage_now(struct ti2419x_chip *chip);
#if FEATURE_SOFT_CC
static int soft_cc_adjust_soc(struct ti2419x_chip *chip);
#endif
/*delay 40s before poweroff while the battery is low,it only enable the phone first power on,add by sunshaojie*/
#define ZERO_REPORT_DELAY_DELTA HZ*40*1 // 40s

static bool zero_report_check(void)
{
    static unsigned long  report_zero_jiffies = 0;

    if (report_zero_jiffies == 0) {
        report_zero_jiffies = jiffies;
        pr_info("start check at %ld\n", report_zero_jiffies);
        return false;
    } else {
        pr_info("capacity zero\n");
        if (time_after(jiffies,report_zero_jiffies+ZERO_REPORT_DELAY_DELTA ))
            return true;
        else
            return false;
    }
}
/*delay 40s before poweroff while the battery is low,it only enable the phone first power on,add by sunshaojie*/


int smooth_capacity(struct ti2419x_chip *chip,int capacity)
{
	static bool turnon_flags = true;
	int cap = capacity;
	int batt_vol,is_chg_in;
	static int count=0;

	if((turnon_flags)&&(capacity > 0))
		turnon_flags = false;

	pr_debug("CHG:before smooth capacity= %d\n",capacity);

	if (zero_reported) {
		pr_info("ZERO reported already, just return 0 directly\n");
		return 0;
	}
#if 0
	if((capacity == 100)&&(chip->batt_full	==	false)&&(ti2419x_is_chg_plugged_in(chip))){
		pr_debug("capacity %d, chip->batt_full %d,ti2419x_is_chg_plugged_in %d\n",capacity,chip->batt_full,ti2419x_is_chg_plugged_in(chip));
		return 99;
	}
#endif
	if( chip->batt_full == true ){
		pr_info("[CHG]: batt_full=1; before smooth capacity= %d\n",capacity);
		return 100;
	}

	/* Note by zte JZN 20160301:
	* 1)if battery voltage continuously lower than SHUTDOWN_VOLTAGE for 5 times,
	*    force capacity=0,to avoid over discharging of the battery
	* 2) it's especially useful in low temperature situation
	*/
	batt_vol   = ti2419x_get_prop_voltage_now(chip);
	is_chg_in = ti2419x_is_chg_plugged_in(chip);
	if( ( batt_vol <=  LOW_TEMP_SHUTDOWN_VOLTAGE) &&  (is_chg_in ==0)){
		count ++;
		pr_info("[CHG]: batt voltage=%d,count=%d\n",batt_vol,count);
		if (count ==5){
			pr_info("[CHG]: batt voltage is critically low,force soc=0\n");
			capacity = 0;
			count = 0;
		}

	} else
		count = 0;

	if(capacity == 0)
	{
		if(!enable_to_shutdown){
			pr_debug("CHG: enable_to_shutdown=0,return soc=1\n");
			return 1;
		}

		if( batt_vol > SHUTDOWN_VOLTAGE){
			pr_debug("CHG: vol is higher than shutdown vol,return soc=1\n");
			return 1;
		}

#if FEATURE_SOFT_CC
		/*
		* ZTE:when charger in and the soc=0,we need to do a 60s check for this situation.
		* If after 60s monitor, the soft_cc_toal is still positive, that means the charger
		* can't change in anymore, report zero to uplayer.
		*/
		if (ti2419x_is_chg_plugged_in(chip)) {
			//return 0 or 1
			cap = soft_cc_adjust_soc(chip);
		}
		pr_debug("CHG:after soft cc capacity= %d\n",cap);
#endif
		if (capacity == 0){
			if(!turnon_flags)
				zero_reported = true;
			else if(zero_report_check()){
				zero_reported = true;
				turnon_flags = false;
			}else
				cap = 1;
		}
	}

	pr_debug("CHG:after smooth capacity= %d\n",cap);
	return cap;
}
//zte jiangfeng add, end

//static int enable_to_shutdown=1;    //ZTE add
#define DEFAULT_CAPACITY	50
//static bool report_zero = false;   //ZTE
static int ti2419x_get_prop_batt_capacity_real(struct ti2419x_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
        return ret.intval;
	}

	return DEFAULT_CAPACITY;
}

//need get from bms
static int vmbms_get_prop_batt_capacity(struct ti2419x_chip *chip)
{
	union power_supply_propval ret = {0,};
	int VM_CAPACITY = 0;
	if (chip->vm_bms_psy) {
		chip->vm_bms_psy->get_property(chip->vm_bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
		VM_CAPACITY = ret.intval;
		return ret.intval;
	}
	return DEFAULT_CAPACITY;
}

static int ti2419x_get_prop_batt_capacity(struct ti2419x_chip *chip)
{
    chip->soc = ti2419x_get_prop_batt_capacity_real(chip);
    return smooth_capacity(chip,chip->soc);
}

//ITE(INDICATOR_TO_EMPTY):this parameter is only for ON BMS
static int ti2419x_get_prop_batt_ite(struct ti2419x_chip *chip)
{
#ifdef FEATURE_ZTE_LC709203F_FG
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY_LEVEL, &ret);
		return  ret.intval;
	}
#endif

	return 0;
}


static int ti2419x_get_prop_chg_full_design(struct ti2419x_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &ret);
		return ret.intval;
	}

	return DEFAULT_FCC;
}

#define DEFAULT_TEMP		250
static int
ti2419x_get_prop_batt_temp(struct ti2419x_chip *chip)
{
#ifdef FEATURE_ZTE_LC709203F_FG

	int rc = 0;
	struct qpnp_vadc_result results;
       //bool init_adc=false;
	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &results);
	if (rc) {
		pr_debug("Unable to read batt temperature rc=%d\n", rc);
		if(rc==-EPROBE_DEFER){
		chip->vadc_dev = qpnp_get_vadc(chip->dev, "tichg");
			if (IS_ERR(chip->vadc_dev)) {
				rc = PTR_ERR(chip->vadc_dev);
				if (rc != -EPROBE_DEFER)
					pr_err("%s ,vadc property missing\n",__func__);
				else
					pr_err("Unable to get battery therm VADC. rc=%d\n", rc);
			}
		}
		return DEFAULT_TEMP;
	}
	pr_debug("get_bat_temp 0x%x, %lld\n",
		results.adc_code, results.physical);

	#if defined (CONFIG_BOARD_ABBY) ||defined(CONFIG_BOARD_BENZ)||defined(CONFIG_BOARD_CAPTAIN)
		/*Note(JZN)20150906:
		 * return default temp for P890A57/P890A61/P890A62 hw version 0x00,
		 * due to hw board error
		 */
		//pr_info("P890A57 hw_version:0x%02x\n",hw_version);
		if(hw_version == 0)
			return DEFAULT_TEMP;
	#endif

	return (int)results.physical;

#else 
	/*get battery tempp from TI BMS*/
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_TEMP, &ret);
		return ret.intval;
	}

	return DEFAULT_TEMP;

#endif
}


#define DEFAULT_VOLTAGE		3700000
//return uV
int vmbms_get_prop_voltage_now(struct ti2419x_chip *chip)
{
    
	union power_supply_propval ret = {0,};
    if (chip->vm_bms_psy) {
		chip->vm_bms_psy->get_property(chip->vm_bms_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);
        return ret.intval;
	}
    return DEFAULT_VOLTAGE;
}

int ti2419x_get_prop_voltage_now(struct ti2419x_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);
		return ret.intval;
	}

	return DEFAULT_VOLTAGE;
}

#if 0
static int ti2419x_get_prop_batt_resistance(struct ti2419x_chip *chip)
{
	u8 reg = 0;
	int rc;

	rc = ti2419x_read(chip, SHDW_FG_ESR_ACTUAL, &reg);
	if (rc) {
		pr_err("Failed to read FG_ESR_ACTUAL rc=%d\n", rc);
		return rc;
	}

	pr_debug("reg=0x%02x resistance=%d\n", reg, reg * 2);

	return reg * 2;
}
#endif

//return uA
static int ti2419x_get_prop_current_now(struct ti2419x_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CURRENT_NOW, &ret);
		return ret.intval;
	}

	return 0;
}
int set_ibat(struct ti2419x_chip *chip, int ma);

static int ti2419x_set_appropriate_usb_current(struct ti2419x_chip *chip)
{
	int rc = 0, i, therm_ma, current_ma;
	int path_current = chip->usb_psy_ma;

	/*
	 * If battery is absent do not modify the current at all, these
	 * would be some appropriate values set by the bootloader or default
	 * configuration and since it is the only source of power we should
	 * not change it
	 */
	if (!chip->batt_present) {
		pr_debug("ignoring current request since battery is absent\n");
		return 0;
	}

	if (chip->therm_lvl_sel > 0
			&& chip->therm_lvl_sel < (chip->thermal_levels - 1))
		/*
		 * consider thermal limit only when it is active and not at
		 * the highest level
		 */
		therm_ma = chip->thermal_mitigation[chip->therm_lvl_sel];
	else
		therm_ma = path_current;

	current_ma = min(therm_ma, path_current);

	if(chip->max_iusb > 0)
		current_ma = min(current_ma, chip->max_iusb);

	if (current_ma <= 2) {
		//when usb report 2MA, must set to the smallest current
		current_ma = 100;
	}

	for (i = ARRAY_SIZE(input_current_limit) - 1; i >= 0; i--) {
		if (input_current_limit[i] <= current_ma)
			break;
	}
	if (i < 0) {
		pr_debug("Couldn't find ICL mA rc=%d\n", rc);
		i = 0;
	}

	/* set input current limit */
	rc = ti2419x_masked_write(chip, INPUT_SOURCE_CTRL_REG,
					IINLIMIT_MASK, i);
	if (rc)
		pr_err("Couldn't set ICL mA rc=%d\n", rc);

	/*ZTE NOTE 20151125:do not set ibat here*/
	//set_ibat(chip,ichg_current_limit[i]);

	pr_info("input current set to = %d,set ichg_current to %d\n", input_current_limit[i],ichg_current_limit[i]);

#if 0
	/* enable charging, as it could have been disabled earlier */
	rc = ti2419x_charging_disable(chip, CURRENT, false);
	if (rc < 0)
		pr_err("Unable to enable charging rc=%d\n", rc);
#endif

	return rc;
}

static int ti2419x_system_temp_level_set(struct ti2419x_chip *chip,
							int lvl_sel)
{
	int rc = 0;
	int prev_therm_lvl;

	if (!chip->thermal_mitigation) {
		pr_err("Thermal mitigation not supported\n");
		return -EINVAL;
	}

	if (lvl_sel < 0) {
		pr_err("Unsupported level selected %d\n", lvl_sel);
		return -EINVAL;
	}

	if (lvl_sel >= chip->thermal_levels) {
		pr_err("Unsupported level selected %d forcing %d\n", lvl_sel,
				chip->thermal_levels - 1);
		lvl_sel = chip->thermal_levels - 1;
	}

	if (lvl_sel == chip->therm_lvl_sel)
		return 0;

	mutex_lock(&chip->current_change_lock);
	prev_therm_lvl = chip->therm_lvl_sel;
	chip->therm_lvl_sel = lvl_sel;
	if (chip->therm_lvl_sel == (chip->thermal_levels - 1)) {
		/* Disable charging if highest value selected */
		rc = ti2419x_charging_disable(chip, THERMAL, true);
		if (rc < 0) {
			pr_err("Couldn't disable charging rc %d\n", rc);
			goto out;
		}
		goto out;
	}

	ti2419x_set_appropriate_usb_current(chip);

	if (prev_therm_lvl == chip->thermal_levels - 1) {
		/*
		 * If previously highest value was selected charging must have
		 * been disabed. Hence enable charging.
		 */
		rc = ti2419x_charging_disable(chip, THERMAL, false);
		if (rc < 0) {
			pr_err("Couldn't enable charging rc %d\n", rc);
			goto out;
		}
	}
out:
	mutex_unlock(&chip->current_change_lock);
	return rc;
}

static int ti2419x_battery_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	struct ti2419x_chip *chip = container_of(psy,
				struct ti2419x_chip, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		ti2419x_charging_disable(chip, USER, !val->intval);
		power_supply_changed(&chip->batt_psy);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		chip->fake_battery_soc = val->intval;
		pr_info("fake_soc set to %d\n", chip->fake_battery_soc);
		power_supply_changed(&chip->batt_psy);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		ti2419x_system_temp_level_set(chip, val->intval);
		break;
	case  POWER_SUPPLY_PROP_PRESENT:
		//need get from bms
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ti2419x_battery_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

static int ti2419x_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct ti2419x_chip *chip = container_of(psy,
				struct ti2419x_chip, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = ti2419x_get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = ti2419x_get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = ti2419x_get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = ti2419x_is_charging_enabled(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = ti2419x_get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = ti2419x_get_prop_batt_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = ti2419x_get_prop_chg_full_design(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = ti2419x_get_prop_voltage_now(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = ti2419x_get_prop_current_now(chip);
		break;
#if 0	//not support in 2419x
	case POWER_SUPPLY_PROP_RESISTANCE:
		val->intval = ti2419x_get_prop_batt_resistance(chip);
		break;
#endif
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = ti2419x_get_prop_batt_temp(chip);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		val->intval = chip->therm_lvl_sel;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->usb_present;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = ti2419x_get_prop_batt_bms_fcc(chip);
		break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
void set_charger_state(struct ti2419x_chip *chip,bool disable)
{
    pr_info("disable=%d chip->batt_full=%d\n",disable,chip->batt_full);
    if(disable)
    {
        chip->batt_full = true;
	    ti2419x_charging_disable(chip, CURRENT, true);
        power_supply_changed(&chip->batt_psy);
        pr_info("charger_disable1=%d chip->batt_full1=%d\n",disable,chip->batt_full);
    }else{
	    ti2419x_charging_disable(chip, CURRENT, false);
        power_supply_changed(&chip->batt_psy);
        pr_info("charger_disable2=%d chip->batt_full2=%d\n",disable,chip->batt_full);
    }
}

static void ti2419x_external_power_changed(struct power_supply *psy)
{
	struct ti2419x_chip *chip = container_of(psy,
				struct ti2419x_chip, batt_psy);
	union power_supply_propval prop = {0,};
	int rc, current_limit = 0;

	pr_info("external power changed\n");
	if (!chip->bms_psy)
		chip->bms_psy = power_supply_get_by_name("on_bms");

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (rc < 0)
		dev_err(chip->dev,
			"could not read USB current_max property, rc=%d\n", rc);
	else
		current_limit = prop.intval / 1000;

	pr_info("current_limit = %d chip->usb_psy_ma = %d\n", current_limit, chip->usb_psy_ma);

	if (chip->usb_psy_ma != current_limit) {
		mutex_lock(&chip->current_change_lock);
		chip->usb_psy_ma = current_limit;
		rc = ti2419x_set_appropriate_usb_current(chip);
		if (rc < 0)
			pr_err("Couldn't set usb current rc = %d\n", rc);
		mutex_unlock(&chip->current_change_lock);
		pr_info("usb_psy_ma: %d\n", chip->usb_psy_ma);
	}

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_ONLINE, &prop);
	if (rc < 0)
		pr_err("could not read USB ONLINE property, rc=%d\n", rc);

	/* update online property */
	rc = 0;
	if (chip->usb_present && chip->usb_psy_ma != 0) {
		if (prop.intval == 0)
			rc = power_supply_set_online(chip->usb_psy, true);
	} else {
		if (prop.intval == 1)
			rc = power_supply_set_online(chip->usb_psy, false);
	}
	if (rc < 0)
		pr_err("could not set usb online, rc=%d\n", rc);
}

static int sys_ov_handler(struct ti2419x_chip *chip, u8 rt_stat)
{
	pr_info("rt_stat = 0x%02x\n", rt_stat);
	return 0;
}

static int therm_handler(struct ti2419x_chip *chip, u8 rt_stat)
{
	pr_info("rt_stat = 0x%02x\n", rt_stat);
	return 0;
}

static int power_good_handler(struct ti2419x_chip *chip, u8 rt_stat)
{
	bool usb_present = rt_stat;
	pr_info("chip->usb_present = %d usb_present = %d\n",
				chip->usb_present, usb_present);

	/* in poweroff charging,once the charger is absent,run poweroff work*/
	if(offcharging_flag && !usb_present)
	{
		pr_info("usb removed!!!poweroff\n");
		schedule_work(&chip->poweroff_work);
	}

	if (chip->usb_present ^ usb_present) {
		wake_lock_timeout(&chip->charger_wake_lock, 5 * HZ);
	}

	if (chip->usb_present && !usb_present) {
		/* USB removed */
		chip->usb_present = usb_present;
		power_supply_set_present(chip->usb_psy, usb_present);
		#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_I2C
		syna_ts_notifier_call_chain(0);
		#endif
		wake_unlock(&chip->charger_valid_lock);
		chip->batt_full = false;
		chip->batt_warm_full = false;        
		ti2419x_charging_disable(chip, CURRENT, false);
	}

	if (!chip->usb_present && usb_present) {
		/* USB inserted */
		wake_lock(&chip->charger_valid_lock);

		chip->usb_present = usb_present;
		power_supply_set_present(chip->usb_psy, usb_present);
		
		#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_I2C
		syna_ts_notifier_call_chain(1);
		#endif
		cancel_delayed_work_sync(&chip->charger_eoc_work);   // stop charger_eoc_work.
		schedule_delayed_work(&chip->charger_eoc_work, round_jiffies_relative(msecs_to_jiffies(2000)));
		
	}

	return 0;
}

static int dpm_handler(struct ti2419x_chip *chip, u8 rt_stat)
{
	pr_info("%s, rt_stat = 0x%02x\n", __func__,rt_stat);
	return 0;
}

static int chg_stat_handler(struct ti2419x_chip *chip, u8 rt_stat)
{
	int rc;
	u8 reg = 0;

	rc = ti2419x_read(chip, SYS_STAT_REG, &reg);
	reg &= CHG_STAT_MASK;
	reg >>= CHG_STAT_SHIFT;

	pr_info("%s, charge status = 0x%02x\n", __func__,reg);

	return 0;
}

static int vbus_stat_handler(struct ti2419x_chip *chip, u8 rt_stat)
{
	pr_info("%s, rt_stat = 0x%02x\n", __func__,rt_stat);
	return 0;
}

static int hot_cold_handler(struct ti2419x_chip *chip, u8 rt_stat)
{
	pr_info("%s, not support stop charge automatically when hot or cold.\n", __func__);
	return 0;
}

static int bat_fault_handler(struct ti2419x_chip *chip, u8 rt_stat)
{
	//over charge
	pr_info("%s, rt_stat = 0x%02x\n", __func__,rt_stat);
	return 0;
}

static int chg_fault_handler(struct ti2419x_chip *chip, u8 rt_stat)
{
	pr_info("%s, rt_stat = 0x%02x\n", __func__,rt_stat);
	return 0;
}

static int boost_fault_handler(struct ti2419x_chip *chip, u8 rt_stat)
{
	pr_info("%s, rt_stat = 0x%02x\n", __func__,rt_stat);
	return 0;
}

static int wdog_fault_handler(struct ti2419x_chip *chip, u8 rt_stat)
{
	pr_info("%s, rt_stat = 0x%02x\n", __func__,rt_stat);
	return 0;
}


struct ti2419x_irq_info {
	const char		*name;
	int			(*ti2419x_irq)(struct ti2419x_chip *chip,
							u8 rt_stat);
	int			high;
	int			low;
};

struct irq_handler_info {
	u8			stat_reg;
	u8			val;
	u8			prev_val;
	struct ti2419x_irq_info	irq_info[8];
};

static struct irq_handler_info handlers[] = {
	{SYS_STAT_REG, 0, 0,
		{
			{
				.name		= "sys_voltage_status",
				.ti2419x_irq	= sys_ov_handler,
			},
			{
				.name		= "THERM_status",
				.ti2419x_irq	= therm_handler,
			},
			{
				.name		= "PG_status",
				.ti2419x_irq	= power_good_handler,
			},
			{
				.name		= "DPM_status",
				.ti2419x_irq	= dpm_handler,
			},
			{
				.name		= "chg_status0",
				.ti2419x_irq	= chg_stat_handler,
			},
			{
				.name		= "chg_status1",
				.ti2419x_irq	= chg_stat_handler,
			},
			{
				.name		= "VBUS_status0",
				.ti2419x_irq	= vbus_stat_handler,
			},
			{
				.name		= "VBUS_status1",
				.ti2419x_irq	= vbus_stat_handler,
			},
		},
	},
	{FAULT_REG, 0, 0,
		{
			{
				.name		= "NTC_fault0",
				.ti2419x_irq	= hot_cold_handler,
			},
			{
				.name		= "NTC_fault1",
				.ti2419x_irq	= hot_cold_handler,
			},
			{
				.name		= "NTC_fault2",
				.ti2419x_irq	= hot_cold_handler,
			},
			{
				.name		= "BAT_fault",
				.ti2419x_irq	= bat_fault_handler,
			},
			{
				.name		= "CHG_fault0",
				.ti2419x_irq	= chg_fault_handler,
			},
			{
				.name		= "CHG_fault1",
				.ti2419x_irq	= chg_fault_handler,
			},
			{
				.name		= "BOOST_fault",
				.ti2419x_irq	= boost_fault_handler,
			},
			{
				.name		= "WDOG_fault",
				.ti2419x_irq	= wdog_fault_handler,
			},
		},
	},
};

#define IRQ_STATUS_MASK		0x01
static irqreturn_t ti2419x_stat_handler(int irq, void *dev_id)
{
	struct ti2419x_chip *chip = dev_id;
	int i, j;
	u8 changed;
	u8 rt_stat, prev_rt_stat;
	int rc;
	int handler_count = 0;

	//pr_info("enter..\n");
	mutex_lock(&chip->irq_complete);
	chip->irq_waiting = true;
	if (!chip->resume_completed) {
		dev_dbg(chip->dev, "IRQ triggered before device-resume\n");
		disable_irq_nosync(irq);
		mutex_unlock(&chip->irq_complete);
		return IRQ_HANDLED;
	}
	chip->irq_waiting = false;

	for (i = 0; i < ARRAY_SIZE(handlers); i++) {
		rc = ti2419x_read(chip, handlers[i].stat_reg,
					&handlers[i].val);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't read %d rc = %d\n",
					handlers[i].stat_reg, rc);
			continue;
		}
		pr_info("[%d]reg=0x%x val=0x%x prev_val=0x%x\n",
				i, handlers[i].stat_reg, handlers[i].val, handlers[i].prev_val);

		for (j = 0; j < ARRAY_SIZE(handlers[i].irq_info); j++) {
			rt_stat = handlers[i].val
				& (IRQ_STATUS_MASK << j);
			prev_rt_stat = handlers[i].prev_val
				& (IRQ_STATUS_MASK << j);
			changed = prev_rt_stat ^ rt_stat;

			if (changed)
				rt_stat ? handlers[i].irq_info[j].high++ :
						handlers[i].irq_info[j].low++;

			if (changed	&& handlers[i].irq_info[j].ti2419x_irq != NULL) {
				handler_count++;
				pr_info("call %pf, handler_count=%d\n",handlers[i].irq_info[j].ti2419x_irq,handler_count);
				rc = handlers[i].irq_info[j].ti2419x_irq(chip,
								rt_stat);
				if (rc < 0)
					dev_err(chip->dev,
						"Couldn't handle %d irq for reg 0x%02x rc = %d\n",
						j, handlers[i].stat_reg, rc);
			}
		}
		handlers[i].prev_val = handlers[i].val;
	}

	pr_debug("handler count = %d\n", handler_count);
	if (handler_count)
	{
		cancel_delayed_work(&chip->update_heartbeat_work);
		schedule_delayed_work(&chip->update_heartbeat_work,0);

		cancel_delayed_work(&chip->charger_eoc_work);
		schedule_delayed_work(&chip->charger_eoc_work, 0);
	}

	mutex_unlock(&chip->irq_complete);

	return IRQ_HANDLED;
}

static int show_irq_count(struct seq_file *m, void *data)
{
	int i, j, total = 0;

	for (i = 0; i < ARRAY_SIZE(handlers); i++)
		for (j = 0; j < 8; j++) {
			seq_printf(m, "%s=%d\t(high=%d low=%d)\n",
						handlers[i].irq_info[j].name,
						handlers[i].irq_info[j].high
						+ handlers[i].irq_info[j].low,
						handlers[i].irq_info[j].high,
						handlers[i].irq_info[j].low);
			total += (handlers[i].irq_info[j].high
					+ handlers[i].irq_info[j].low);
		}

	seq_printf(m, "\n\tTotal = %d\n", total);

	return 0;
}

static int irq_count_debugfs_open(struct inode *inode, struct file *file)
{
	struct ti2419x_chip *chip = inode->i_private;

	return single_open(file, show_irq_count, chip);
}

static const struct file_operations irq_count_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= irq_count_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define LAST_CNFG_REG	0xA

static int show_cnfg_regs(struct seq_file *m, void *data)
{
	struct ti2419x_chip *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
		rc = ti2419x_read(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cnfg_debugfs_open(struct inode *inode, struct file *file)
{
	struct ti2419x_chip *chip = inode->i_private;

	return single_open(file, show_cnfg_regs, chip);
}

static const struct file_operations cnfg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cnfg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

//not support now, need change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//okay, it is time to have a change.
static int ti2419x_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct ti2419x_chip *chip = rdev_get_drvdata(rdev);
	pr_info("%s\n", __func__);
	rc = ti2419x_masked_write(chip, PON_CFG_REG, CHG_CFG_MASK,
			(2 << CHG_CFG_SHIFT));
	if (rc < 0 )
		pr_err("Couldn't enable  OTG boost power, rc=%d\n", rc);

	return rc;
}

static int ti2419x_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct ti2419x_chip *chip = rdev_get_drvdata(rdev);
	pr_info("%s\n", __func__);
	rc = ti2419x_masked_write(chip, PON_CFG_REG, CHG_CFG_MASK,
			(1 << CHG_CFG_SHIFT));
	if (rc < 0)
		pr_err("Couldn't disable OTG boost power, rc=%d\n", rc);

	return rc;
}

static int ti2419x_otg_regulator_is_enabled(struct regulator_dev *rdev)
{
	u8 reg = 0;
	int rc = 0;
	struct ti2419x_chip *chip = rdev_get_drvdata(rdev);
	pr_info("%s\n", __func__);
	rc = ti2419x_read(chip, PON_CFG_REG, &reg);
	if (rc) {
		pr_err("Couldn't read OTG boost power state, rc=%d\n", rc);
		return rc;
	}

	return  ((reg & CHG_CFG_MASK) & (2 << CHG_CFG_SHIFT))? 1 : 0;
}

struct regulator_ops ti2419x_otg_reg_ops = {
	.enable		= ti2419x_otg_regulator_enable,
	.disable	= ti2419x_otg_regulator_disable,
	.is_enabled	= ti2419x_otg_regulator_is_enabled,
};

static int ti2419x_otg_is_enabled(struct ti2419x_chip *chip)
{
	u8 reg = 0;
	int rc = 0;
	rc = ti2419x_read(chip, PON_CFG_REG, &reg);
	if (rc) {
		pr_err("Couldn't read OTG boost power state, rc=%d\n", rc);
		return rc;
	}

	return  ((reg & CHG_CFG_MASK) & (2 << CHG_CFG_SHIFT))? 1 : 0;
}

static int ti2419x_regulator_init(struct ti2419x_chip *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(chip->dev, chip->dev->of_node);
	if (!init_data) {
		dev_err(chip->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &ti2419x_otg_reg_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = chip->dev->of_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->otg_vreg.rdev = regulator_register(
					&chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			chip->otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
					"OTG reg failed, rc=%d\n", rc);
		}
	}

	return rc;
}
//end, not support now, change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//changed

static int determine_initial_status(struct ti2419x_chip *chip)
{
#if 0
	int rc;
	u8 reg = 0;
	union power_supply_propval ret = {0,};

	/*
	 * It is okay to read the IRQ status as the irq's are
	 * not registered yet.
	 */
	chip->batt_present = true;
	//need get from bms, battery detect
	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_PRESENT, &ret);
		chip->batt_present = ret.intval;
	}

	rc = ti2419x_read(chip, CHG_TERM_TIMER_CTRL_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read CHG_TERM_TIMER_CTRL_REG rc = %d\n", rc);
		return rc;
	}
	if (!(reg & TERM_STAT_MASK))
		chip->batt_full = true;

	//hot, cold, need change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	rc = ti2419x_read(chip, IRQ_A_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read irq A rc = %d\n", rc);
		return rc;
	}

	if (reg & IRQ_A_HOT_HARD_BIT)
		chip->batt_hot = true;
	if (reg & IRQ_A_COLD_HARD_BIT)
		chip->batt_cold = true;
	if (reg & IRQ_A_HOT_SOFT_BIT)
		chip->batt_warm = true;
	if (reg & IRQ_A_COLD_SOFT_BIT)
		chip->batt_cool = true;

	rc = ti2419x_read(chip, SYS_STAT_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read SYS_STAT_REG rc = %d\n", rc);
		return rc;
	}
#endif
	//set usb_present to false when initialize
	chip->usb_present = false;
	chip->bat_is_cold = false;
	chip->bat_is_hot = false;
	chip->bat_is_cool = false;
	chip->bat_is_warm = false;
	chip->high_temp_threshold = chip->warm_bat_decidegc;
	chip->low_temp_threshold = chip->cool_bat_decidegc;
	chip->soc = -1;

	return 0;
}


#define MIN_ITERM_MA		128
#define MAX_ITERM_MA		1024
static void set_iterm(struct ti2419x_chip *chip,int ma)
{
	u8 temp;
	int rc;

	if ((ma < MIN_ITERM_MA) || (ma > MAX_ITERM_MA)) {
		dev_err(chip->dev, "bad terminate current mv =%d asked to set\n",
					ma);
		return;
	}

	temp = (ma - MIN_ITERM_MA) / MIN_ITERM_MA;

	rc = ti2419x_masked_write(chip, IPRECHG_ITERM_CTRL_REG,
			ITERM_MASK, temp << ITERM_SHIFT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set ITERM rc=%d\n", rc);
		return;
	}

	rc = ti2419x_masked_write(chip, CHG_TERM_TIMER_CTRL_REG,
			EN_TERM_MASK | TERM_STAT_MASK, 0x2 << TERM_STAT_SHIFT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't enable EN_TERM rc=%d\n", rc);
	}
}

#define TI2419X_WDOG_DISABLE	0
#define TI2419X_WDOG_40S		1
#define TI2419X_WDOG_80S		2
#define TI2419X_WDOG_160S		3
int set_charge_wdog(struct ti2419x_chip *chip, int time)
{
	int rc;
	if(time < TI2419X_WDOG_DISABLE || time > TI2419X_WDOG_160S)
	{
		dev_err(chip->dev, "invalid charge watch dog setting\n");
		return -EINVAL;
	}

	rc = ti2419x_masked_write(chip, CHG_TERM_TIMER_CTRL_REG,
			I2C_WDOG_TIMER_MASK, time << I2C_WDOG_TIMER_SHIFT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set watch dog timer rc=%d\n", rc);
		return rc;
	}

	return 0;
}

#define MIN_IBAT_MA		512
#define MAX_IBAT_MA		3008
#define MAX_IBAT_STEP	64
int set_ibat(struct ti2419x_chip *chip, int ma)
{
	int rc;
	u8 reg;

	if(ma < MIN_IBAT_MA){
        ma = MIN_IBAT_MA;
        dev_err(chip->dev, "bad battery charge current ma =%d asked to set\n",
					ma);
	}else if(ma > MAX_IBAT_MA){
		ma = MAX_IBAT_MA;
		dev_err(chip->dev, "bad battery charge current ma =%d asked to set\n",
					ma);
	}

	reg = (ma - MIN_IBAT_MA)/MAX_IBAT_STEP;

	rc = ti2419x_masked_write(chip, CHG_I_CTRL_REG,
			IFAST_CHG_MASK, reg << IFAST_CHG_SHIFT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set battery charge current rc=%d\n", rc);
		return rc;
	}
	pr_info("ibat current set to = %d\n", (reg*MAX_IBAT_STEP+512) );

	return 0;
}

void ti2419x_set_appropriate_ibat(struct ti2419x_chip *chip)
{
	unsigned int chg_current = chip->max_ibat;

	if (chip->bat_is_cool)
		chg_current = min(chg_current, chip->cool_bat_chg_ma);
	if (chip->bat_is_warm)
		chg_current = min(chg_current, chip->warm_bat_chg_ma);
	pr_debug("setting %d mA\n", chg_current);

	set_ibat(chip, chg_current);
}

#define MIN_INPUT_VOLTAGE_MV    3880
#define MAX_INPUT_VOLTAGE_MV	5080
#define MAX_INPUT_VOLTAGE_STEP	80
int set_input_voltage(struct ti2419x_chip *chip, int mv)
{
	int rc;
	u8 reg;

	if ((mv < MIN_INPUT_VOLTAGE_MV) || (mv > MAX_INPUT_VOLTAGE_MV)) {
		dev_err(chip->dev, "bad input voltage mv =%d asked to set\n",
					mv);
		return -EINVAL;
	}

	reg = (mv - MIN_INPUT_VOLTAGE_MV)/MAX_INPUT_VOLTAGE_STEP;

	rc = ti2419x_masked_write(chip, INPUT_SOURCE_CTRL_REG,
			VINLIMIT_MASK, reg << VINLIMIT_SHIFT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set input voltage rc=%d\n", rc);
		return rc;
	}

	return 0;
}

int get_charge_IC_type(struct ti2419x_chip *chip)
{
	int rc;
	u8	reg;

	rc = ti2419x_read(chip, VENDOR_REG, &reg);
	if (rc) {
		pr_err("Couldn't read VENDOR_REG rc=%d\n", rc);
		return -1;
	}

	reg &= TI2419X_DEV_REG_MASK;
	return (reg) ? CHARGER_IC_2419X : CHARGER_IC_2429X;
}

static int ti2419x_enable_ship_mode(struct ti2419x_chip *chip)
{
	int rc;

	/* NOTE:
	*  this function is for TI BQ24298,to enable ship mode:
	*  step 1:disable watchdog timer(REG05[5:4]=00)
	*  step 2:disabling BATFET (REG07[5] bit=1)
	*  by doing this,BATFET will be off after 7.5s
	*/

	rc = ti2419x_masked_write(chip, CHG_TERM_TIMER_CTRL_REG,
			I2C_WDOG_TIMER_MASK, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't disable WDOG timer rc=%d\n", rc);
		return rc;
	}

	rc = ti2419x_masked_write(chip, MISC_OPERA_CTRL_REG,
			BATFET_DISABLE_MASK, 1<<BATFET_DISABLE_SHIFT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't turn OFF BATTFET rc=%d\n", rc);
		return rc;
	}

	return 0;
}

int ti2419x_turn_off_battery_switch(void)
{
	pr_info("ti2419x is going to turn_off battery switch\n");
	return ti2419x_enable_ship_mode(the_ti2419x_chip);
}

static int ti2419x_hw_init(struct ti2419x_chip *chip)
{
	int rc;
	int i;
	u8 reg;

	//wdog timer, 80s
	set_charge_wdog(chip, TI2419X_WDOG_160S);

	//charge IC type
	chip->chargeIC_type = get_charge_IC_type(chip);

	//EN_HIZ
	rc = ti2419x_masked_write(chip, INPUT_SOURCE_CTRL_REG,
			EN_HIZ_MASK, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set INPUT_SOURCE_CTRL_REG rc=%d\n", rc);
		return rc;
	}

	if(chip->max_ibat > 0)
		set_ibat(chip, chip->max_ibat);

	//sare time, not 2X
	rc = ti2419x_masked_write(chip, MISC_OPERA_CTRL_REG,
					TMR2X_EN_MASK, 0);
	if (rc)
		pr_err("Couldn't set TMR2X_EN rc=%d\n", rc);

	//input voltage
	set_input_voltage(chip, chip->max_input_voltage);

	//charge config
	rc = ti2419x_masked_write(chip, PON_CFG_REG,
					CHG_CFG_MASK, 0x1 << CHG_CFG_SHIFT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set PON_CFG_REG rc=%d\n",
				rc);
		return rc;
	}


	/* set the float voltage */
	if (chip->vfloat_mv != -EINVAL) {
		rc = ti2419x_float_voltage_set(chip, chip->vfloat_mv);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set float voltage rc = %d\n", rc);
			return rc;
		}
	}

	/* set iterm */
	if (chip->iterm_ma == -EINVAL) {
		pr_err("error: iterm_ma invalide, need set in dts");
	} else  if (chip->hw_iterm_disabled) {
		pr_info("using sw iterm function");
		ti2419x_masked_write(chip, CHG_TERM_TIMER_CTRL_REG,
				EN_TERM_MASK , 0);
		if (rc) {
			dev_err(chip->dev, "Couldn't disable hw iterm rc = %d\n",
								rc);
			return rc;
		}
	} else {
		pr_info("using hw iterm function,iterm_ma=%d\n",chip->iterm_ma);
		set_iterm(chip, chip->iterm_ma);
	}

	/* set the safety time voltage */
	if (chip->safety_time != -EINVAL) {
		if (chip->safety_time == 0) {
			/* safety timer disabled */
			rc = ti2419x_masked_write(chip, CHG_TERM_TIMER_CTRL_REG,
						SAFE_TIMER_EN_MASK, 0);
			if (rc < 0) {
				dev_err(chip->dev,
				"Couldn't disable safety timer rc = %d\n",
								rc);
				return rc;
			}
		} else {
			for (i = 0; i < ARRAY_SIZE(chg_time); i++) {
				if (chip->safety_time <= chg_time[i]) {
					reg = i << FAST_CHG_TIMER_SHIFT;
					break;
				}
			}
			rc = ti2419x_masked_write(chip, CHG_TERM_TIMER_CTRL_REG,
				FAST_CHG_TIMER_MASK, reg);
			if (rc < 0) {
				dev_err(chip->dev,
					"Couldn't set safety timer rc = %d\n",
									rc);
				return rc;
			}

			rc = ti2419x_masked_write(chip, CHG_TERM_TIMER_CTRL_REG,
						SAFE_TIMER_EN_MASK, 1 << SAFE_TIMER_EN_SHIFT);
			if (rc < 0) {
				dev_err(chip->dev,
					"Couldn't enable safety timer rc = %d\n",
									rc);
				return rc;
			}
		}
	}

	/* configure resume threshold, auto recharge*/
	if (chip->resume_delta_mv != -EINVAL) {
		rc = ti2419x_recharge_threshold_set(chip,
						chip->resume_delta_mv);
		if (rc) {
			dev_err(chip->dev,
				"Couldn't set rechg thresh rc = %d\n",
								rc);
			return rc;
		}
	}

//not support battery detect, need change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#if 0
	/* battery missing detection */
	rc = ti2419x_masked_write(chip, CFG_BATT_MISSING_REG,
				BATT_MISSING_SRC_THERM_BIT,
				BATT_MISSING_SRC_THERM_BIT);
	rc = ti2419x_masked_write(chip, CFG_BATT_MISSING_REG,
				BATT_MISSING_SRC_BMD_BIT,
				0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set batt_missing config = %d\n",
									rc);
		return rc;
	}
#endif

	/* interrupt enabling - active low */
	if (chip->client->irq) {
		rc = ti2419x_masked_write(chip, MISC_OPERA_CTRL_REG, INT_MASK, INT_MASK);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set irq config rc = %d\n",
					rc);
			return rc;
		}
	}

	rc = ti2419x_charging_disable(chip, USER, !!chip->charging_disabled);
	if (rc)
		dev_err(chip->dev, "Couldn't '%s' charging rc = %d\n",
			chip->charging_disabled ? "disable" : "enable", rc);

	return rc;
}

#define ZTE_HOT_TEMP_DEFAULT        500
#define ZTE_COLD_TEMP_DEFAULT       0
#define ZTE_WARM_TEMP_DEFAULT       450
#define ZTE_COOL_TEMP_DEFAULT       100
#define HYSTERISIS_DECIDEGC         20
#define MAX_TEMP                    800
#define MIN_TEMP                   -300
enum {
	TI_TM_HIGHER_STATE,
	TI_TM_LOWER_STATE,
	TI_TM_NORMAL_STATE,
};

static void ti_temp_control_func(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct ti2419x_chip *chip = container_of(dwork, struct ti2419x_chip, temp_control_work);
	bool bat_warm = false, bat_cool = false, bat_hot = false,bat_cold = false;
	int state = TI_TM_NORMAL_STATE;
	int temp=0;
	if (!chip->resume_completed) {
		pr_info("ti_temp_control_func launched before device-resume, schedule to 2s later\n");
		schedule_delayed_work(&chip->temp_control_work,round_jiffies_relative(msecs_to_jiffies(2000)));
		return;
	}
	if(ti2419x_otg_is_enabled(chip)==1){
		pr_info("ti2419x_otg_is_enabled, schedule to 1 min later\n");
		schedule_delayed_work(&chip->temp_control_work,round_jiffies_relative(msecs_to_jiffies(60000)));
		return;
	}

	temp = ti2419x_get_prop_batt_temp(chip);

	if (temp > MAX_TEMP)
		temp = MAX_TEMP;
	if (temp < MIN_TEMP)
		temp = MIN_TEMP;

	if (temp > chip->high_temp_threshold)
		state = TI_TM_HIGHER_STATE;
	else if (temp < chip->low_temp_threshold)
		state = TI_TM_LOWER_STATE;
	else
		state = TI_TM_NORMAL_STATE;

	mutex_lock(&chip->jeita_configure_lock);
	if (state == TI_TM_HIGHER_STATE) {			//warm notification
		pr_debug("TI_TM_HIGHER_STATE");
		if(chip->bat_is_hot == false && chip->bat_is_warm == false && chip->bat_is_cool == false && chip->bat_is_cold == true)	//cold to cool
		{
			bat_hot = false;
			bat_warm = false;
			bat_cool = true;
			bat_cold = false;
			chip->low_temp_threshold = chip->cold_bat_decidegc;
			chip->high_temp_threshold = chip->cool_bat_decidegc;
		}
		else if(chip->bat_is_hot == false && chip->bat_is_warm == false && chip->bat_is_cool == true && chip->bat_is_cold == false)	//cool to normal
		{
			bat_hot = false;
			bat_warm = false;
			bat_cool = false;
			bat_cold = false;
			chip->low_temp_threshold = chip->cool_bat_decidegc - HYSTERISIS_DECIDEGC;
			chip->high_temp_threshold = chip->warm_bat_decidegc;
		}
		else if(chip->bat_is_hot == false && chip->bat_is_warm == false && chip->bat_is_cool == false && chip->bat_is_cold == false) //normal to warm
		{
			bat_hot = false;
			bat_warm = true;
			bat_cool = false;
			bat_cold = false;
			chip->low_temp_threshold =chip->warm_bat_decidegc - HYSTERISIS_DECIDEGC;
			chip->high_temp_threshold = chip->hot_bat_decidegc;
		}
		else if(chip->bat_is_hot == false && chip->bat_is_warm == true && chip->bat_is_cool == false && chip->bat_is_cold == false)		//warm to hot
		{
			bat_hot = true;
			bat_warm = false;
			bat_cool = false;
			bat_cold = false;
			chip->low_temp_threshold = chip->hot_bat_decidegc - HYSTERISIS_DECIDEGC;
			chip->high_temp_threshold = MAX_TEMP;
		}
	} else if (state == TI_TM_LOWER_STATE){										//cool notification
		pr_debug("TI_TM_LOWER_STATE");
		if(chip->bat_is_hot == false && chip->bat_is_warm == false && chip->bat_is_cool == true && chip->bat_is_cold == false)	//cool to cold
		{
			bat_hot = false;
			bat_warm = false;
			bat_cool = false;
			bat_cold = true;
			chip->low_temp_threshold = MIN_TEMP;
			chip->high_temp_threshold = chip->cold_bat_decidegc + HYSTERISIS_DECIDEGC;
		}
		else if(chip->bat_is_hot == false && chip->bat_is_warm == false && chip->bat_is_cool == false && chip->bat_is_cold == false)  //normal to cool
		{
			bat_hot = false;
			bat_warm = false;
			bat_cool = true;
			bat_cold = false;
			chip->low_temp_threshold = chip->cold_bat_decidegc;
			chip->high_temp_threshold = chip->cool_bat_decidegc + HYSTERISIS_DECIDEGC;
		}
		else if(chip->bat_is_hot == false && chip->bat_is_warm == true && chip->bat_is_cool == false && chip->bat_is_cold == false)	//warm to normal
		{
			bat_hot = false;
			bat_warm = false;
			bat_cool = false;
			bat_cold = false;
			chip->low_temp_threshold =chip->cool_bat_decidegc;
			chip->high_temp_threshold = chip->warm_bat_decidegc + HYSTERISIS_DECIDEGC;
		}
		else if(chip->bat_is_hot == true && chip->bat_is_warm == false && chip->bat_is_cool == false && chip->bat_is_cold == false)	//hot to warm
		{
			bat_hot = false;
			bat_warm = true;
			bat_cool = false;
			bat_cold = false;
			chip->low_temp_threshold = chip->warm_bat_decidegc;
			chip->high_temp_threshold = chip->hot_bat_decidegc;
		}
	} else {
		pr_debug("TI_TM_NORMAL_STATE");
		goto check_again;
	}

	pr_info("temp = %d bat_cold = %d,bat_cool = %d,bat_warm = %d, bat_hot = %d\n",temp,bat_cold,bat_cool,bat_warm,bat_hot);

	if (chip->bat_is_warm ^ bat_warm || chip->bat_is_cool ^ bat_cool) {
		chip->bat_is_warm = bat_warm;
		chip->bat_is_cool = bat_cool;
		pr_info("battery warm/cool, temp = %d bat_cold = %d,bat_cool = %d,bat_warm = %d, bat_hot = %d\n",temp,bat_cold,bat_cool,bat_warm,bat_hot);
		/**
		 * set appropriate voltages and currents.
		 *
		 * (TODO)Note that when the battery is hot or cold, the charger
		 * driver will not resume with SoC. Only vbatdet is used to
		 * determine resume of charging.
		 */

		/* NOTE(by ZTE JZN):
		*  if vbatt>warm_bat_mv and charging is enabled, battery will discharging and iusb=0 ;
		*  if vbatt>warm_bat_mv and charging is disabled, iusb will be the currnt source and ibat=0 .
		*  so, there is a bug in the below codes:if vbatt>warm_bat_mv,battery will discharging and iusb=0
		*/
		ti2419x_set_appropriate_usb_current(chip);
		ti2419x_set_appropriate_ibat(chip);
		ti2419x_set_appropriate_float_voltage(chip);
        /*if battery is warm and the battery is warm_full,when the the state turn normal,it should recharge*/
		ti2419x_charging_disable(chip, CURRENT, false);
		chip->batt_warm_full = false;
		power_supply_changed(&chip->batt_psy);
	}
	/**
	  * Hot/cool Temp Procedure(ZTE)
	  *
	  * add for stop charging when hot/cool temp,
	  * only called when status changed
	  */

	if (chip->bat_is_hot ^ bat_hot  || chip->bat_is_cold ^ bat_cold) {
		chip->bat_is_hot = bat_hot;
		chip->bat_is_cold = bat_cold;
		pr_info("battery hot/cool, temp = %d bat_cold = %d,bat_cool = %d,bat_warm = %d, bat_hot = %d\n",temp,bat_cold,bat_cool,bat_warm,bat_hot);
		if (chip->bat_is_hot || chip->bat_is_cold) {
			/* disable charging */
			ti2419x_charging_disable(chip, TEMP, true);
		} else {
			/* enable charging */
			ti2419x_charging_disable(chip, TEMP, false);
		}
		power_supply_changed(&chip->batt_psy);
	}
check_again:
	mutex_unlock(&chip->jeita_configure_lock);

	pr_debug("bat_is_warm %d,bat_cold = %d, bat_is_cool %d bat_is_hot %d, low = %d, high = %d\n",
			chip->bat_is_warm,chip->bat_is_cold, chip->bat_is_cool, chip->bat_is_hot,
			chip->low_temp_threshold, chip->high_temp_threshold);

	schedule_delayed_work(&chip->temp_control_work,round_jiffies_relative(msecs_to_jiffies(2000)));
}

static int heartbeat_ms = 0;
static int set_heartbeat_ms(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	if (the_ti2419x_chip) {
		pr_info("set_heartbeat_ms to %d\n", heartbeat_ms);
		cancel_delayed_work_sync(&the_ti2419x_chip->update_heartbeat_work);
		schedule_delayed_work(&the_ti2419x_chip->update_heartbeat_work,
				      round_jiffies_relative(msecs_to_jiffies
							     (heartbeat_ms)));
		return 0;
	}
	return -EINVAL;
}
module_param_call(heartbeat_ms, set_heartbeat_ms, param_get_uint,
					&heartbeat_ms, 0644);

void check_fullcharged_state(struct ti2419x_chip *chip)
{
#ifdef FEATURE_ZTE_LC709203F_FG
	int charger_status=0;
	int is_chg_in, is_batt_present, capatity;
	static int is_first_time_check=1;

	is_chg_in          = ti2419x_is_chg_plugged_in(chip);
	is_batt_present= ti2419x_get_prop_batt_present(chip);
	capatity             = ti2419x_get_prop_batt_capacity_real(chip);
	charger_status  = ti2419x_get_charging_status(chip);

	if (charger_status == CHARGE_TERM_DONE) {
		pr_info("charger_status %d DONE,old batt_full=%d\n",charger_status,chip->batt_full);
		if( is_chg_in && is_batt_present && (capatity<=99) && is_first_time_check ){
			__ti2419x_charging_disable(chip, true);
			__ti2419x_charging_disable(chip, false);//re-charging if soc not 100
			pr_info("[CHG] start re-charging when charger reported DONE and soc =%d\n",capatity);
			return;
		}
		chip->batt_full = true;
		power_supply_changed(&chip->batt_psy);
		pr_info("[CHG]set batt_full=1 and soc =%d\n",capatity);
		//ti2419x_charging_disable(chip, CURRENT, true);
		//set_charger_state(chip,1);
	}
	else{
		pr_info("charger_status %d\n",charger_status);
		//set_charger_state(chip,0);
	}
	is_first_time_check = 0;
#else
    int bms_flags,is_chg_in;
    bool fc_bit;

    is_chg_in = ti2419x_is_chg_plugged_in(chip);
    bms_flags = ti2419x_get_prop_batt_bms_flags(chip);
    fc_bit = (bms_flags&0x0200)?1:0;
    pr_info("chip->batt_full:%d,fc_bit:%d,is_chg_in:%d\n",chip->batt_full,fc_bit,is_chg_in);
     if((chip->batt_full!=fc_bit)&&is_chg_in)
    {
        set_charger_state(chip,fc_bit);
    }
#endif
}

void print_ti2419x_regs(struct ti2419x_chip *chip)
{
	int i;
	u8  reg[10];
	for (i=0;i<=8;i++)
	{
		ti2419x_read(chip, i, &reg[i]);
	}
	pr_info("TI_24296_REG:[0]=%x,[1]=%x,[2]=%x,[3]=%x,[4]=%x,[5]=%x,[6]=%x,[7]=%x,[8]=%x\n",
		reg[0],reg[1],reg[2],reg[3],reg[4],reg[5],reg[6],reg[7],reg[8]);
}

/*print the BMS regs add by zte ssj*/
void print_bms_regs(struct ti2419x_chip *chip)
{
    int bms_capacity,bms_current,bms_voltage,bms_status,bms_fcc,bms_flags,bms_rc,bms_qmax,vmbms_vol,vmbms_cap,temp;//,bms_qmax;
    bms_voltage = ti2419x_get_prop_voltage_now(chip)/1000;
    if(!chip->vm_bms_psy){
        chip->vm_bms_psy = power_supply_get_by_name("bms");
        vmbms_vol = 0;
        vmbms_cap = 0;
        pr_info("vmbms not good");
    }else{
        vmbms_vol = vmbms_get_prop_voltage_now(chip)/1000;
        vmbms_cap= vmbms_get_prop_batt_capacity(chip);
    }
    bms_capacity = ti2419x_get_prop_batt_capacity_real(chip);
    bms_current = ti2419x_get_prop_current_now(chip)/1000;
    bms_status = ti2419x_get_prop_batt_bms_status(chip);
    bms_fcc = ti2419x_get_prop_batt_bms_fcc(chip);
    bms_flags = ti2419x_get_prop_batt_bms_flags(chip);
    bms_rc    = ti2419x_get_prop_batt_bms_remainingcapacity(chip);
    bms_qmax = ti2419x_get_prop_batt_bms_qmax(chip);
	temp = ti2419x_get_prop_batt_temp(chip);
    pr_info("bms_voltage:%d,bms_capacity:%d,bms_current:%d,bms_status:0x%x,bms_fcc:%d,bms_flags:0x%x,bms_rc:%d,bms_qmax:%d,temp:%d\n",
        bms_voltage,bms_capacity,bms_current,bms_status,bms_fcc,bms_flags,bms_rc,bms_qmax,temp);
    pr_info("ti_vol=%d,vmbms_vol=%d,ti_cap=%d,vmbms_cap=%d,",bms_voltage,vmbms_vol,bms_capacity,vmbms_cap);
}
/*print the BMS regs add by zte ssj*/


static unsigned int poweroff_enable = 1;

module_param(poweroff_enable, uint, 0644);
MODULE_PARM_DESC(poweroff_enable, "poweroff enable flag bit - " \
                "0 disables poweroff");

#define OFFCHG_FORCE_POWEROFF_DELTA HZ*60*10 // 10mins
#define NORMAL_FORCE_POWEROFF_DELTA HZ*60*1 // 1mins
static void force_power_off_check(int capacity)
{
    static unsigned long  report_zero_jiffies = 0;

    /*
      *report zero, but the uplayer is not shutdown in 60s,
      *kernel should power off directly.
      */
	if (capacity == 0) {
		if (report_zero_jiffies == 0) {
		    report_zero_jiffies = jiffies;
			pr_info("start check at %ld\n", report_zero_jiffies);
		} else {
			pr_info("offcharging_flag=%d %ld,%ld\n",
			offcharging_flag,  jiffies, report_zero_jiffies);
			if ((offcharging_flag && time_after(jiffies,report_zero_jiffies+OFFCHG_FORCE_POWEROFF_DELTA )) ||
				(!offcharging_flag && time_after(jiffies,report_zero_jiffies+NORMAL_FORCE_POWEROFF_DELTA )) )
				kernel_power_off();
		}
	}else
		report_zero_jiffies = 0;
}

#define LOW_SOC_HEARTBEAT_MS  20000
#define HEARTBEAT_MS          60000
static void update_heartbeat(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct ti2419x_chip *chip = container_of(dwork, struct ti2419x_chip, update_heartbeat_work);
	int rc;
	int period = 0;

	int temp,voltage,cap,status,charge_type,present,chg_current,usb_present,health,ite;
	static int old_temp = 0;
	static int old_cap = 0;
	static int old_status = 0;
	static int old_present = 0;
	static int old_usb_present = 0;
	static int old_health = 0;
	static int count=0;

    //zte add start by ssj
	if (!chip->resume_completed) {
		pr_info("update_heartbeat launched before device-resume, schedule to 5s later\n");
		schedule_delayed_work(&chip->update_heartbeat_work,
                          round_jiffies_relative(msecs_to_jiffies(5000)));
		return;
	}
    //zte add end by ssj
	//kick watch dog
	rc = ti2419x_masked_write(chip, PON_CFG_REG,
			I2C_WDOG_RESET_MASK, I2C_WDOG_RESET_MASK);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't reset watch dog rc=%d\n", rc);
		schedule_delayed_work(&chip->update_heartbeat_work,
                          round_jiffies_relative(msecs_to_jiffies(5000)));//add by ssj
		return;
	}

	//update state
	if(chip==NULL)
	{
		pr_err("pmic fatal error:the_chip=null\n!!");
		return;
	}

	if (!chip->bms_psy){
		chip->bms_psy = power_supply_get_by_name("on_bms");
		if(chip->bms_psy)
			pr_info("bms psy successful\n");
	}

#if PRINT_TI24296_REG
     print_ti2419x_regs(chip);
     print_bms_regs(chip);

#endif

	temp        = ti2419x_get_prop_batt_temp(chip)/10;
	voltage     = ti2419x_get_prop_voltage_now(chip)/1000;
	cap         = ti2419x_get_prop_batt_capacity(chip);
	status      = ti2419x_get_prop_batt_status(chip);
	charge_type = ti2419x_get_prop_charge_type(chip);
	present     = ti2419x_get_prop_batt_present(chip);
	chg_current = ti2419x_get_prop_current_now(chip)/1000;
	health      = ti2419x_get_prop_batt_health(chip);
	usb_present = chip->usb_present;
	ite         = ti2419x_get_prop_batt_ite(chip); //for ON BMS only

	/*if heatbeat_ms is bigger than 500ms, it means users need this information, must output the logs directly.*/
	if ((heartbeat_ms>=500) || (abs(temp-old_temp) >= 1) || (old_cap != cap) || (old_status != status)
		|| (old_present != present) || (old_usb_present != usb_present ) || (old_health != health)||(count%5==0))
	{
		pr_info("***temp=%d,vol=%d,cap=%d,ite=%d,status=%d,chg_state=%d,current=%d,batt_present=%d,usb_present=%d,chg_en=%d(%d)\n",
			temp, voltage, cap,ite,status, charge_type, chg_current, present,usb_present,
			chip->charging_disabled_status,ti2419x_is_charging_enabled(chip));
#if 0
		do {
			int usbin = get_adc_val(chip,USBIN);
			int vchg = get_adc_val(chip,VCHG_SNS);
			int vbat = get_adc_val(chip,VBAT_SNS);
			int vsys = get_adc_val(chip,VSYS);
			int iusb_max = qpnp_chg_usb_iusbmax_get(chip);
			pr_info("***usbin=%d vchg=%d vbat=%d vsys=%d iusb_max=%d\n",usbin,vchg,vbat,vsys,iusb_max);
		} while(0);
#endif

		old_temp = temp;
		old_cap = cap;
		old_status = status;
		old_present = present;
		old_usb_present = usb_present;
		old_health = health;
		count=0;
	}

	count++;

	/*
	  *report zero, but the uplayer is not shutdown in 60s,
	  *kernel should power off directly.
	  */
	  if(poweroff_enable == 0)
	  {
			force_power_off_check(0);
	  }else{
			force_power_off_check(cap);
	  }
	power_supply_changed(&chip->batt_psy);

	if (heartbeat_ms >= 500) {
		period = heartbeat_ms;
	} else {
		if (cap <= 20)
			period = LOW_SOC_HEARTBEAT_MS;
		else
			period = HEARTBEAT_MS;
	}
	schedule_delayed_work(&chip->update_heartbeat_work,
				      round_jiffies_relative(msecs_to_jiffies
							     (period)));
}

//Move from 8974-UFI
#if 1  //ZTE_XJB add EOC
#define CONSECUTIVE_COUNT	5
#define EOC_CHECK_PERIOD_MS	10000
static void charger_eoc(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct ti2419x_chip *chip = container_of(dwork, struct ti2419x_chip, charger_eoc_work);
	static int count=0;
	static int vbat_low_count=0;
	int vbat_mv,capacity_soc;
	int charger_status = 0;
	int delay_timer;
	int is_chg_in = 0;
	int rc = 0;
	int max_float_voltage = 0;
	if (!chip->resume_completed) {
		pr_info("charger_eoc launched before device-resume, schedule to 2s later\n");
		schedule_delayed_work(&chip->charger_eoc_work, round_jiffies_relative(msecs_to_jiffies(5000)));
		return;
	}

	capacity_soc = ti2419x_get_prop_batt_capacity(chip);
	is_chg_in    = ti2419x_is_chg_plugged_in(chip);

	if(is_chg_in == 0){
		pr_info("no chg connected, go through directly\n");
		count = 0;
		vbat_low_count = 0;
		goto check_again_later;
	}

	if(!chip->bat_is_warm && !chip->bat_is_cool)
	{
		pr_debug("temp not in  warm/cool mode\n");
		check_fullcharged_state(chip);
		goto check_again_later;
    }

    pr_debug("temp in warm mode\n");

	/*check for recharging*/
#if 0
	if (chip->batt__full) {
		if((capacity_soc <= 99)&&(vbat_mv <= (max_float_voltage - chip->resume_delta_mv - chip->vbatdet_max_err_mv))) {
			pr_info("need recharging now\n");
			ti2419x_charging_disable(chip, CURRENT, false);
		}else
			pr_debug("No need recharging, check it again\n");
		
	} else {
		/*check for full charging*/
		charger_status = ti2419x_get_charging_status(chip);
		if (charger_status == FAST_CHARGE) {
			if (((capacity_soc <= 99) || (vbat_mv <  (max_float_voltage - chip->resume_delta_mv  - chip->vbatdet_max_err_mv)))&&(chip->bat_is_warm||chip->bat_is_cool)) {
				pr_info("woke up too early vbat_mv = %d, max_mv = %d, resume_mv = %d tolerance_mv = %d low_count = %d soc=%d\n",
						vbat_mv, max_float_voltage,
						chip->resume_delta_mv,
						chip->vbatdet_max_err_mv,
						vbat_low_count,
						capacity_soc);

				count = 0;
				goto check_again_later;
			}
	}
#endif
	rc = ti2419x_float_voltage_get(chip, &max_float_voltage);
	if (rc < 0) {
		pr_err("error when get float voltage from register\n");
		goto check_again_later;
	}

	vbat_mv = ti2419x_get_prop_voltage_now(chip)/1000;
	if (chip->batt_warm_full) {
		if(vbat_mv <= (max_float_voltage - chip->warm_resume_delta_mv - chip->vbatdet_max_err_mv)) {
			pr_info("need recharging now\n");
			ti2419x_charging_disable(chip, CURRENT, false);
			chip->batt_warm_full = false;
		}else
			pr_debug("No need recharging, check it again\n");
		
	} else {
		/*check for full charging*/
		charger_status = ti2419x_get_charging_status(chip);
		if (charger_status == FAST_CHARGE) {
			if ((vbat_mv <  (max_float_voltage - chip->warm_resume_delta_mv  - chip->vbatdet_max_err_mv))&&(chip->bat_is_warm ||chip->bat_is_cool)) {
				pr_info("woke up too early vbat_mv = %d, max_mv = %d, resume_mv = %d tolerance_mv = %d low_count = %d soc=%d\n",
						vbat_mv, max_float_voltage,
						chip->resume_delta_mv,
						chip->vbatdet_max_err_mv,
						vbat_low_count,
						capacity_soc);

				count = 0;
				goto check_again_later;
			}
#ifdef FEATURE_ZTE_LC709203F_FG
           /*check for warm full by soc */
		   /*SOC 75% ~ 4.05v  */
			if (capacity_soc > 75) {
				if (count == CONSECUTIVE_COUNT) {
					chip->batt_warm_full = true;
					ti2419x_charging_disable(chip, CURRENT, true);

					pr_info("End of warm/cool Charging.\n");
					pr_info("chip->batt_warm_full:%d\n",chip->batt_warm_full);
					power_supply_changed(&chip->batt_psy);
				} else {
					count += 1;
					pr_info("EOC count = %d\n", count);
				}
			 }
#else
			/*check for warm full by ibat current */
			int ibat_ma;
			ibat_ma	= ti2419x_get_prop_current_now(chip)/1000;
			if ((ibat_ma * -1) > chip->iterm_ma) {
				pr_info("Not at EOC, battery current too high, ibat_ma=%d\n", ibat_ma);
				count = 0;
			} else if (ibat_ma > 0) {
				pr_info("Charging but system demand increased\n");
				count = 0;
			} else {
				if (count == CONSECUTIVE_COUNT) {
					#if 0
						chip->batt_full = true;
					#else
						chip->batt_warm_full = true;
					#endif
					ti2419x_charging_disable(chip, CURRENT, true);

					pr_info("End of Charging.\n");
                    pr_info("chip->batt_warm_full:%d\n",chip->batt_warm_full);
					power_supply_changed(&chip->batt_psy);
					//goto stop_eoc;
				} else {
					count += 1;
					pr_info("EOC count = %d\n", count);
				}
			}
#endif
		}else
			pr_debug("not in fast charging\n");
	}	

check_again_later:
	if ((capacity_soc<=60) || (is_chg_in == 0))//cap is below 60% or charger is not in(100s)
		delay_timer=EOC_CHECK_PERIOD_MS *10;
    else
		delay_timer=EOC_CHECK_PERIOD_MS;
	
	schedule_delayed_work(&chip->charger_eoc_work, round_jiffies_relative(msecs_to_jiffies(delay_timer)));
}
#endif

#if FEATURE_SOFT_CC
/*ZTE:  Soft CC*/
#define SOFT_CC_NUM 60
struct qpnp_soft_cc {
	int data[SOFT_CC_NUM];
	int pos;
	int total;
	bool monitor_launched;
	bool total_valid;
};

static struct qpnp_soft_cc soft_cc;
#if SOFT_CC_DEBUG
static char debug_buff[1024] = {0};
#endif

static void soft_cc_append_data(int data)
{
	struct qpnp_soft_cc *sc = &soft_cc;

	int pos = (sc->pos + 1) % SOFT_CC_NUM;
	int old_data = sc->data[pos];

	if (sc->pos >= SOFT_CC_NUM-1)
		sc->total_valid= true;

	sc->data[pos] = data;
	sc->pos = pos;
	sc->total = sc->total - old_data + data;
#if SOFT_CC_DEBUG
	{
	int i = 0,cnt = 0;
	memset(debug_buff,0,sizeof(debug_buff));
	for(i = 0; i < SOFT_CC_NUM; i++){
		if (i == sc->pos)
			cnt += sprintf(debug_buff + cnt,"%5d*",sc->data[i]);
		else
			cnt += sprintf(debug_buff + cnt,"%5d ",sc->data[i]);
	}
	pr_info("==soft cc dump==\n");
	pr_info("%s\n",debug_buff);
	pr_info("\ttotal=%d\n",sc->total);
	pr_info("\tpos=%d\n",sc->pos);
	pr_info("\ttotal_valid=%d\n",sc->total_valid);
	pr_info("\tmonitor_launched=%d\n",sc->monitor_launched);
	}
#endif
}

static void soft_cc_reset(void)
{
	struct qpnp_soft_cc *sc = &soft_cc;
	int i = 0;

	for ( i = 0; i < SOFT_CC_NUM; i++)
		sc->data[i] = 0;
	sc->pos  = -1;
	sc->total = 0;
	sc->total_valid= false;
	sc->monitor_launched = false;
}

// this function called when soc=0 and charger in
static int soft_cc_adjust_soc(struct ti2419x_chip *chip)
{
	struct qpnp_soft_cc *sc = &soft_cc;

#if SOFT_CC_DEBUG
	pr_info("sc->total_valid=%d sc->total=%d \n",sc->total_valid,sc->total);
#endif
	if (sc->total_valid && (sc->total > 0)) {
		pr_info("soc=0 and charger in, soft_cc_total>0, report 0\n");
		return 0;
	} else {
		if (!sc->monitor_launched) {
			soft_cc_reset();
			sc->monitor_launched = true;
			pr_info("soc=0 and charger is in, launching soft_cc_monitor_work\n");
			schedule_delayed_work(&chip->soft_cc_monitor_work,
				round_jiffies_relative(msecs_to_jiffies(1000)));
		}
		return 1;
	}
}

static void soft_cc_monitor(struct work_struct *work)
{
	int chg_current = 0;
	int usb_present = 0;
	int cap = 0;
	int vol = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct ti2419x_chip *chip = container_of(dwork,
				struct ti2419x_chip, soft_cc_monitor_work);

	if(chip == NULL)
	{
		pr_err("pmic fatal error:the_chip=null\n!!");
		return;
	}
	if (!chip->resume_completed) {
		pr_info("soft_cc_monitor launched before device-resume, schedule to 2s later\n");
		schedule_delayed_work(&chip->soft_cc_monitor_work,
				      round_jiffies_relative(msecs_to_jiffies
							     (1000)));
		return;
	}
	chg_current = ti2419x_get_prop_current_now(chip)/1000;//mA
	usb_present = ti2419x_is_chg_plugged_in(chip);
	cap         = ti2419x_get_prop_batt_capacity(chip);
	vol         = ti2419x_get_prop_voltage_now(chip);
	if ((vol <= 3600000) && (cap <= 2) &&  usb_present){
		soft_cc_append_data(chg_current);
		if (cap != 0)
			schedule_delayed_work(&chip->soft_cc_monitor_work,
				      round_jiffies_relative(msecs_to_jiffies
							     (1000)));
	} else {
		soft_cc_reset();
		pr_info("soft cc reset , cap=%d usb_present=%d\n",cap,usb_present);
	}
}
#endif//FEATURE_SOFT_CC

void bq27x00_notify(void)
{
	pr_info("\n");
	if(the_ti2419x_chip)
	{
		cancel_delayed_work(&the_ti2419x_chip->update_heartbeat_work);
		schedule_delayed_work(&the_ti2419x_chip->update_heartbeat_work,0);
	}
}


static int ti2419x_parse_dt(struct ti2419x_chip *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	rc = of_property_read_u32(node, "zte,float-voltage-mv",
						&chip->vfloat_mv);
	if (rc < 0)
		chip->vfloat_mv = -EINVAL;

	rc = of_property_read_u32(node, "zte,charging-timeout",
						&chip->safety_time);
	if (rc < 0)
		chip->safety_time = -EINVAL;

	if (!rc && (chip->safety_time > chg_time[ARRAY_SIZE(chg_time) - 1])) {
		dev_err(chip->dev, "Bad charging-timeout %d\n",
						chip->safety_time);
		return -EINVAL;
	}

	rc = of_property_read_u32(node, "zte,recharge-thresh-mv",
						&chip->resume_delta_mv);
	if (rc < 0)
		chip->resume_delta_mv = -EINVAL;
	rc = of_property_read_u32(node, "zte,warm_recharge-thresh-mv",
						&chip->warm_resume_delta_mv);
	if (rc < 0)
		chip->resume_delta_mv = -EINVAL;
    

	rc = of_property_read_u32(node, "zte,vbatdet-max-err-mv",
						&chip->vbatdet_max_err_mv);
	if (rc < 0)
		chip->vbatdet_max_err_mv = -EINVAL;

	rc = of_property_read_u32(node, "zte,iterm-ma", &chip->iterm_ma);
	if (rc < 0)
		chip->iterm_ma = -EINVAL;

	chip->hw_iterm_disabled = of_property_read_bool(node,
						"zte,hw-iterm-disabled");

	chip->charging_disabled = of_property_read_bool(node,
						"zte,charging-disabled");
	pr_info("hw-iterm-disabled: %d charging-disabled: %d\n",
		chip->hw_iterm_disabled,
		chip->charging_disabled);

	//iusb
	rc = of_property_read_u32(node,
						"zte,max_usb_current", &chip->max_iusb);
	if (rc < 0)
		chip->max_iusb = -EINVAL;
	pr_info("zte,max usb input current: %d\n", chip->max_iusb);

	//ibat
	rc = of_property_read_u32(node,
						"zte,max_battery_current", &chip->max_ibat);
	if (rc < 0)
		chip->max_ibat = -EINVAL;
	pr_info("zte,max battery charge current: %d\n", chip->max_ibat);

	//input voltage
	rc = of_property_read_u32(node,
						"zte,input_voltage_mv", &chip->max_input_voltage);
	if (rc < 0)
		chip->max_input_voltage = -EINVAL;
	pr_info("zte,input voltage: %d\n", chip->max_input_voltage);

	rc = of_property_read_u32(node,"zte,warm_bat_mv", &chip->warm_bat_mv);
	if (rc < 0)
		chip->warm_bat_mv = -EINVAL;
	pr_info("warm_bat_mv: %d\n", chip->warm_bat_mv);

	rc = of_property_read_u32(node,"zte,cool_bat_mv", &chip->cool_bat_mv);
	if (rc < 0)
		chip->cool_bat_mv = -EINVAL;
	pr_info("cool_bat_mv: %d\n", chip->cool_bat_mv);

	//move from qpnp-charger.c
	rc = of_property_read_u32(node,"zte,warm-bat-decidegc", &chip->warm_bat_decidegc);
	if (rc < 0)
		chip->warm_bat_decidegc = ZTE_WARM_TEMP_DEFAULT; //default 45 deg C
	pr_info("warm_bat_decidegc: %d\n", chip->warm_bat_decidegc);

	rc = of_property_read_u32(node,"zte,cool-bat-decidegc", &chip->cool_bat_decidegc);
	if (rc < 0)
		chip->cool_bat_decidegc = ZTE_COOL_TEMP_DEFAULT; //default 10 deg C
	pr_info("cool_bat_decidegc: %d\n", chip->cool_bat_decidegc);

	rc = of_property_read_u32(node,"zte,hot-bat-decidegc", &chip->hot_bat_decidegc);
	if (rc < 0)
		chip->hot_bat_decidegc = ZTE_HOT_TEMP_DEFAULT;  //default 50 deg C
	pr_info("hot_bat_decidegc: %d\n", chip->hot_bat_decidegc);

	rc = of_property_read_u32(node,"zte,cold-bat-decidegc", &chip->cold_bat_decidegc);
	if (rc < 0)
		chip->cold_bat_decidegc = ZTE_COLD_TEMP_DEFAULT;  //default 0 deg C
	pr_info("cold_bat_decidegc: %d\n", chip->cold_bat_decidegc);

	rc = of_property_read_u32(node,"zte,batt-hot-percentage", &chip->hot_batt_p);
	if (rc < 0)
		chip->hot_batt_p = -EINVAL;
	pr_info("hot_batt_p: %d\n", chip->hot_batt_p);

	rc = of_property_read_u32(node,"zte,batt-cold-percentage", &chip->cold_batt_p);
	if (rc < 0)
		chip->cold_batt_p = -EINVAL;
	pr_info("cold_batt_p: %d\n", chip->cold_batt_p);

	//end move from qpnp-charger.c

	rc = of_property_read_u32(node,"zte,warm_bat_chg_ma", &chip->warm_bat_chg_ma);
	if (rc < 0)
		chip->warm_bat_chg_ma = -EINVAL;
	pr_info("warm_bat_chg_ma: %d\n", chip->warm_bat_chg_ma);

	rc = of_property_read_u32(node,"zte,cool_bat_chg_ma", &chip->cool_bat_chg_ma);
	if (rc < 0)
		chip->cool_bat_chg_ma = -EINVAL;
	pr_info("cool_bat_chg_ma: %d\n", chip->cool_bat_chg_ma);

	if (of_find_property(node, "zte,thermal-mitigation",
					&chip->thermal_levels)) {
		chip->thermal_mitigation = devm_kzalloc(chip->dev,
					chip->thermal_levels,
						GFP_KERNEL);

		if (chip->thermal_mitigation == NULL) {
			pr_err("thermal mitigation kzalloc() failed.\n");
			return -ENOMEM;
		}

		chip->thermal_levels /= sizeof(int);
		rc = of_property_read_u32_array(node,
				"zte,thermal-mitigation",
				chip->thermal_mitigation, chip->thermal_levels);
		if (rc) {
			pr_err("Couldn't read threm limits rc = %d\n", rc);
			return rc;
		}
	}

	/* fg params */
	return 0;
}

static int ti2419x_debug_data_set(void *data, u64 val)
{
	struct ti2419x_chip *chip = data;

	pr_info("reg=%d val=%llu\n",chip->reg_addr,val);
	__ti2419x_write(chip, chip->reg_addr,val);
	return 0;
}

static int ti2419x_debug_data_get(void *data, u64 *val)
{
	struct ti2419x_chip *chip = data;
	u8 temp = 0;

	ti2419x_read(chip, chip->reg_addr, &temp);
	*val =(u64) temp;
	pr_info("reg=%d val=%llu\n",chip->reg_addr,*val);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(ti2419x_debug_data_fops, ti2419x_debug_data_get,
			ti2419x_debug_data_set, "%llu\n");


extern void set_batt_hot_cold_threshold(unsigned int vbatt_hot_threshold,
	unsigned int vbatt_cold_threshold);	//zte charge add for pmic therm adc


static int ti2419x_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	u8 reg;
	int rc;
	struct ti2419x_chip *chip;
	struct power_supply *usb_psy, *bms_psy ,*vm_bms_psy;

	pr_info("enter\n");

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_dbg(&client->dev, "USB supply not found; defer probe\n");
		return -EPROBE_DEFER;
	}

	bms_psy = power_supply_get_by_name("on_bms");
	if (!bms_psy) {
		dev_dbg(&client->dev, "BMS supply not found; defer probe\n");
		//return -EPROBE_DEFER;
	}

	vm_bms_psy = power_supply_get_by_name("bms");
	if (!vm_bms_psy) {
		dev_dbg(&client->dev, "BMS supply not found; defer probe\n");
		//return -EPROBE_DEFER;
	}/**/

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}
	chip->client = client;
	chip->dev = &client->dev;
	chip->usb_psy = usb_psy;
	chip->bms_psy = bms_psy;
	chip->vm_bms_psy = vm_bms_psy;
	chip->fake_battery_soc = -EINVAL;
	mutex_init(&chip->read_write_lock);

	/* probe the device to check if its actually connected */
	rc = ti2419x_read(chip, VENDOR_REG, &reg);
	if (rc) {
		pr_err("Failed to detect TI 2419x, device may be absent\n");
		return -ENODEV;
	}

	rc = ti2419x_parse_dt(chip);
	if (rc < 0) {
		dev_err(&client->dev, "Unable to parse DT nodes\n");
		return rc;
	}

	device_init_wakeup(chip->dev, 1);
	i2c_set_clientdata(client, chip);
	chip->resume_completed = true;
	mutex_init(&chip->irq_complete);
	mutex_init(&chip->charging_disable_lock);
	mutex_init(&chip->current_change_lock);
	mutex_init(&chip->jeita_configure_lock);
	chip->default_i2c_addr = client->addr;

	pr_debug("default_i2c_addr=%x\n", chip->default_i2c_addr);

	INIT_WORK(&chip->poweroff_work, offcharge_poweroff_work);
	wake_lock_init(&chip->charger_wake_lock, WAKE_LOCK_SUSPEND, "zte_chg_event");
	wake_lock_init(&chip->charger_valid_lock, WAKE_LOCK_SUSPEND, "zte_chg_valid");
	INIT_DELAYED_WORK(&chip->update_heartbeat_work, update_heartbeat);
	INIT_DELAYED_WORK(&chip->charger_eoc_work, charger_eoc);
	INIT_DELAYED_WORK(&chip->temp_control_work, ti_temp_control_func);
#if FEATURE_SOFT_CC
	INIT_DELAYED_WORK(&chip->soft_cc_monitor_work, soft_cc_monitor);//zte
	soft_cc_reset();//ZTE:initialize soft_cc parameters
#endif

	//for usb-otg
	rc = ti2419x_regulator_init(chip);
	if  (rc) {
		dev_err(&client->dev,
			"Couldn't initialize ti2419x ragulator rc=%d\n", rc);
		return rc;
	}

	rc = ti2419x_hw_init(chip);
	if (rc < 0) {
		dev_err(&client->dev,
			"Unable to intialize hardware rc = %d\n", rc);
		goto fail_hw_init;
	}

	rc = determine_initial_status(chip);
	if (rc < 0) {
		dev_err(&client->dev,
			"Unable to determine init status rc = %d\n", rc);
		goto fail_hw_init;
	}

	chip->batt_psy.name		= "battery";
	chip->batt_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.get_property	= ti2419x_battery_get_property;
	chip->batt_psy.set_property	= ti2419x_battery_set_property;
	chip->batt_psy.properties	= ti2419x_battery_properties;
	chip->batt_psy.num_properties  = ARRAY_SIZE(ti2419x_battery_properties);
	chip->batt_psy.external_power_changed = ti2419x_external_power_changed;
	chip->batt_psy.property_is_writeable = ti2419x_battery_is_writeable;

	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		dev_err(&client->dev,
			"Unable to register batt_psy rc = %d\n", rc);
		goto fail_hw_init;
	}

	/* NOTE(by zte jzn 20150714):
	*  Here set_batt_hot_cold_threshold() is used for correcting different batteries' NTC therm;
	*  it is defined in qpnp-adc-common.c
	*/
	set_batt_hot_cold_threshold(chip->hot_batt_p, chip->cold_batt_p);
	pr_info("hot threshold: %d, cold threshold: %d\n",
		chip->hot_batt_p, chip->cold_batt_p);

#ifdef FEATURE_ZTE_LC709203F_FG
	chip->vadc_dev = qpnp_get_vadc(chip->dev, "tichg");
			if (IS_ERR(chip->vadc_dev)) {
				rc = PTR_ERR(chip->vadc_dev);
				if (rc != -EPROBE_DEFER)
					pr_err("%s ,vadc property missing\n",__func__);
				else
					pr_err("%s ,vadc property fail \n",__func__);
				pr_info("ZTE !!!!error rc=%d\n",rc);
				//goto fail_chg_enable;
			}
#endif
	/* STAT irq configuration */
	if (client->irq) {
		rc = devm_request_threaded_irq(&client->dev, client->irq, NULL,
				ti2419x_stat_handler, IRQF_ONESHOT,
				"ti2419x_stat_irq", chip);
		if (rc < 0) {
			dev_err(&client->dev,
				"request_irq for irq=%d  failed rc = %d\n",
				client->irq, rc);
			goto unregister_batt_psy;
		}
		enable_irq_wake(client->irq);
	}

	chip->debug_root = debugfs_create_dir("ti2419x", NULL);
	if (!chip->debug_root)
		dev_err(chip->dev, "Couldn't create debug dir\n");

	if (chip->debug_root) {
		struct dentry *ent;

		ent = debugfs_create_file("registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &cnfg_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create cnfg debug file\n");
#if 1
		ent = debugfs_create_u8("address", S_IRUSR | S_IWUSR,
					chip->debug_root,
					&chip->reg_addr);
		if (!ent) {
			dev_err(chip->dev,
				"Couldn't create address debug file\n");
		}

		ent = debugfs_create_file("data",  S_IRUSR | S_IWUSR,
					chip->debug_root, chip,
					&ti2419x_debug_data_fops);
		if (!ent) {
			dev_err(chip->dev,
				"Couldn't create data debug file\n");
		}
#endif

		ent = debugfs_create_x32("skip_writes",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root,
					  &(chip->skip_writes));
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create data debug file\n");

		ent = debugfs_create_x32("skip_reads",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root,
					  &(chip->skip_reads));
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create data debug file\n");

		ent = debugfs_create_file("irq_count", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &irq_count_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create count debug file\n");
	}

	if (client->irq) {
		pr_info("call ti2419x_stat_handler when probe finish\n");
		ti2419x_stat_handler(client->irq, chip);
	}

	dev_info(chip->dev, "TI 2419x probe success! batt=%d usb=%d soc=%d\n",
			ti2419x_get_prop_batt_present(chip),
			chip->usb_present,
			ti2419x_get_prop_batt_capacity(chip));

	/*if in offcharging and usb is absent,run power off*/
	if(offcharging_flag && !chip->usb_present)
	{
		pr_info("in offcharging and usb removed.poweroff!!!\n");
		//kernel_power_off();
		schedule_work(&chip->poweroff_work);
	}

	schedule_delayed_work(&chip->update_heartbeat_work,0);
	schedule_delayed_work(&chip->charger_eoc_work, 0);
	schedule_delayed_work(&chip->temp_control_work, 0);
	the_ti2419x_chip	=	chip;
	return 0;

unregister_batt_psy:
	power_supply_unregister(&chip->batt_psy);
fail_hw_init:
	regulator_unregister(chip->otg_vreg.rdev); //for usb-otg
	return rc;
}

static int ti2419x_remove(struct i2c_client *client)
{
	struct ti2419x_chip *chip = i2c_get_clientdata(client);

    cancel_delayed_work_sync(&chip->charger_eoc_work);   // stop charger_eoc_work.
	cancel_delayed_work_sync(&chip->temp_control_work);   // stop temp_control_work.
	cancel_delayed_work_sync(&chip->update_heartbeat_work);
	cancel_work_sync(&chip->poweroff_work);
	regulator_unregister(chip->otg_vreg.rdev); //for usb-otg
	power_supply_unregister(&chip->batt_psy);
	mutex_destroy(&chip->charging_disable_lock);
	mutex_destroy(&chip->current_change_lock);
	mutex_destroy(&chip->read_write_lock);
	mutex_destroy(&chip->irq_complete);
	mutex_destroy(&chip->jeita_configure_lock);
	debugfs_remove_recursive(chip->debug_root);

	pr_info("ti chg driver exit...\n");
	return 0;
}

static int ti2419x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ti2419x_chip *chip = i2c_get_clientdata(client);

	pr_info("enter ti2419x_suspend\n");
	set_charge_wdog(chip, TI2419X_WDOG_DISABLE);

	mutex_lock(&chip->irq_complete);
	chip->resume_completed = false;
	mutex_unlock(&chip->irq_complete);
	cancel_delayed_work_sync(&chip->charger_eoc_work);   // stop charger_eoc_work.
	cancel_delayed_work_sync(&chip->temp_control_work);   // stop temp_control_work.
	cancel_delayed_work_sync(&chip->update_heartbeat_work);
#if FEATURE_SOFT_CC
	cancel_delayed_work_sync(&chip->soft_cc_monitor_work);
#endif
	
	return 0;
}

static int ti2419x_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ti2419x_chip *chip = i2c_get_clientdata(client);

	pr_info("enter ti2419x_suspend_noirq\n");
	if (chip->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int ti2419x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ti2419x_chip *chip = i2c_get_clientdata(client);

	pr_info("enter ti2419x_resume\n");
	chip->fake_battery_soc = -EINVAL;
	mutex_lock(&chip->irq_complete);
        chip->resume_completed = true;
        if (chip->irq_waiting) {
        	mutex_unlock(&chip->irq_complete);
        	ti2419x_stat_handler(client->irq, chip);
		enable_irq(client->irq);
        } else {
        	mutex_unlock(&chip->irq_complete);
        }
        //kick watch dog zte ssj
	if (ti2419x_masked_write(chip, PON_CFG_REG,I2C_WDOG_RESET_MASK, I2C_WDOG_RESET_MASK) < 0) 
	{
		dev_err(chip->dev, "Couldn't reset watch dog \n");
	}
	set_charge_wdog(chip, TI2419X_WDOG_160S);

	/* NOTE(by zte JZN)--20150720
	* 1)delay updateheart workqueue to 5s later.
	* 2)If the system be awake less than 5s, it's not need to run update_heartbeat_work.
	* 3)In ti2419x_suspend, the below workqueues will be canceled
	*/
	schedule_delayed_work(&chip->update_heartbeat_work,round_jiffies_relative(msecs_to_jiffies(5000)));
	schedule_delayed_work(&chip->charger_eoc_work, 0);
	schedule_delayed_work(&chip->temp_control_work, 0);

	power_supply_changed(&chip->batt_psy);
	return 0;
}

static const struct dev_pm_ops ti2419x_pm_ops = {
	.resume		= ti2419x_resume,
	.suspend_noirq	= ti2419x_suspend_noirq,
	.suspend	= ti2419x_suspend,
};

static struct of_device_id ti2419x_match_table[] = {
	{ .compatible = "zte,ti2419x-chg",},
	{ },
};

static const struct i2c_device_id ti2419x_id[] = {
	{"ti2419x-chg", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, ti2419x_id);

static struct i2c_driver ti2419x_driver = {
	.driver		= {
		.name		= "ti2419x-chg",
		.owner		= THIS_MODULE,
		.of_match_table	= ti2419x_match_table,
		.pm		= &ti2419x_pm_ops,			////irq can't be disabled individually
	},
	.probe		= ti2419x_probe,
	.remove		= ti2419x_remove,
	.id_table	= ti2419x_id,
};
/*
//zte add
static int set_enable_to_shutdown(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}
	if (the_ti2419x_chip) {
		pr_warn("set_enable_to_shutdown to %d\n", enable_to_shutdown);
		return 0;
	}
	return -EINVAL;
}
module_param_call(enable_to_shutdown, set_enable_to_shutdown, param_get_uint,
					&enable_to_shutdown, 0644);
//zte add, end
*/

/* NOTE(by JZN 20151026):
   change this driver from module_i2c_driver to late init,
   for power_off work probe sequence conflict
*/
#if 0
module_i2c_driver(ti2419x_driver);
#else  //DEBUG init sequence
static int __init ti2419x_init(void)
{
	int ret;

	printk("%s()\n", __func__);
	ret = i2c_add_driver(&ti2419x_driver);

	if (ret)
		printk(KERN_ERR "Unable to register ti2419x_init i2c driver\n");

	return ret;
}

static void __exit ti2419x_exit(void)
{
	printk("%s()\n", __func__);
	i2c_del_driver(&ti2419x_driver);
}
//module_init(ti2419x_init);
late_initcall(ti2419x_init);
//module_exit(lc709203f_exit);
#endif


MODULE_DESCRIPTION("TI 2419x Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:ti2419x-chg");
