/* Lite-On LTR-559ALS Android / Linux Driver
 *
 * Copyright (C) 2013-2014 Lite-On Technology Corp (Singapore)
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 */


#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
//#include <asm/mach-types.h>
#include <linux/types.h>
#include <asm/setup.h>
#include <linux/version.h>
#include <linux/sensors.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sysfs.h>

#include "ltr559.h"

static ssize_t als_adc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint16_t value;
	int ret;
	struct ltr559_data *ltr559 = sensor_info;

	/*ltr559->mode = ALS;*/
	/*value = read_adc_value(ltr559);*/
	value = read_als_adc_value(ltr559);
	input_report_abs(ltr559->als_input_dev, ABS_MISC, value);
	input_sync(ltr559->als_input_dev);
	//ret = snprintf(buf, sizeof(buf), "%d", value);
	ret = sprintf(buf, "%d", value);
	return ret;
}

//static DEVICE_ATTR(als_adc, 0444, als_adc_show, NULL);


static ssize_t ps_adc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint16_t value;
	int ret;
	struct ltr559_data *ltr559 = sensor_info;

	/*ltr559->mode = PS;*/
	/*value = read_adc_value(ltr559);*/
	value = read_ps_adc_value(ltr559);
	//ret = snprintf(buf, sizeof(buf), "%d", value);
	ret = sprintf(buf, "%d", value);
	return ret;
}

//static DEVICE_ATTR(ps_adc, 0444, ps_adc_show, NULL);


static ssize_t psadcsaturationBit_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint16_t value;
	uint8_t saturation_bit;
	int ret;
	uint8_t buffer[3];
	struct ltr559_data *ltr559 = sensor_info;

	buffer[0] = LTR559_PS_DATA_0;
	ret = I2C_Read(buffer, 2);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s | 0x%02X", __func__, buffer[0]);
		return ret;
	}
	value = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
	/*ltr559->mode = PS_W_SATURATION_BIT;*/
	/*value = read_adc_value(ltr559);*/
	saturation_bit = (value >> 15);
	value &= PS_VALID_MEASURE_MASK;
	//ret = snprintf(buf, sizeof(buf), "%d %d\n", value, saturation_bit);
	ret = sprintf(buf, "%d %d\n", value, saturation_bit);
	return ret;
}

//static DEVICE_ATTR(psadcsaturationBit, 0444, psadcsaturationBit_show, NULL);


static ssize_t ltr559help_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	pr_info("To show ALS value : cat als_adc\n");
	pr_info("To show PS value : cat ps_adc\n");
	pr_info("To show PS value with saturation bit : cat psadcsaturationBit\n\n");

	/* address 0x80*/
	pr_info("Address 0x80 (ALS_CONTR)\n");
	pr_info("ALS active mode : echo 1 > enable\n");
	pr_info("ALS standby mode : echo 0 > enable\n");
	pr_info("To read ALS mode : cat enable\n\n");

	pr_info("ALS SW reset : echo 1 > alsswresetsetup\n");
	pr_info("ALS SW not reset : echo 0 > alsswresetsetup\n");
	pr_info("To read ALS SW reset bit : cat alsswresetsetup\n\n");

	pr_info("ALS gain 1x : echo 1 > alsgainsetup\n");
	pr_info("ALS gain 2x : echo 2 > alsgainsetup\n");
	pr_info("ALS gain 4x : echo 4 > alsgainsetup\n");
	pr_info("ALS gain 8x : echo 8 > alsgainsetup\n");
	pr_info("ALS gain 48x : echo 48 > alsgainsetup\n");
	pr_info("ALS gain 96x : echo 96 > alsgainsetup\n");
	pr_info("To read ALS gain : cat alsgainsetup\n\n");

	pr_info("Write ALS_CONTR register: echo [hexcode value] > alscontrsetup\n");
	pr_info("Example...to write 0x13 : echo 13 > alscontrsetup\n");
	pr_info("To read register ALS_CONTR (0x80) : cat alscontrsetup\n\n");
	/* address 0x80*/

	/* address 0x81*/
	pr_info("Address 0x81 (PS_CONTR)\n");
	pr_info("PS active mode : echo 1 > enable\n");
	pr_info("PS standby mode : echo 0 > enable\n");
	pr_info("To read PS mode : cat enable\n\n");

	pr_info("PS gain x16 : echo 16 > psgainsetup\n");
	pr_info("PS gain x32 : echo 32 > psgainsetup\n");
	pr_info("PS gain x64 : echo 64 > psgainsetup\n");
	pr_info("To read PS gain : cat psgainsetup\n\n");

	pr_info("PS saturation indicator enable : echo 1 > pssatuindicasetup\n");
	pr_info("PS saturation indicator disable : echo 0 > pssatuindicasetup\n");
	pr_info("To read back PS saturation indicator : cat pssatuindicasetup\n\n");

	pr_info("Example...to write 0x13 : echo 13 > pscontrsetup\n");
	pr_info("To read register PS_CONTR (0x81) : cat pscontrsetup\n\n");
	/* address 0x81*/

	/* address 0x82*/
	pr_info("Address 0x82 (PS_LED)\n");
	pr_info("LED current 5mA : echo 5 > psledcurrsetup\n");
	pr_info("LED current 10mA : echo 10 > psledcurrsetup\n");
	pr_info("LED current 20mA : echo 20 > psledcurrsetup\n");
	pr_info("LED current 50mA : echo 50 > psledcurrsetup\n");
	pr_info("LED current 100mA : echo 100 > psledcurrsetup\n");
	pr_info("To read LED current : cat psledcurrsetup\n\n");

	pr_info("LED current duty 25%% : echo 25 > psledcurrduty\n");
	pr_info("LED current duty 50%% : echo 50 > psledcurrduty\n");
	pr_info("LED current duty 75%% : echo 75 > psledcurrduty\n");
	pr_info("LED current duty 100%% : echo 100 > psledcurrduty\n");
	pr_info("To read LED current duty : cat psledcurrduty\n\n");

	pr_info("LED pulse freq 30kHz : echo 30 > psledpulsefreqsetup\n");
	pr_info("LED pulse freq 40kHz : echo 40 > psledpulsefreqsetup\n");
	pr_info("LED pulse freq 50kHz : echo 50 > psledpulsefreqsetup\n");
	pr_info("LED pulse freq 60kHz : echo 60 > psledpulsefreqsetup\n");
	pr_info("LED pulse freq 70kHz : echo 70 > psledpulsefreqsetup\n");
	pr_info("LED pulse freq 80kHz : echo 80 > psledpulsefreqsetup\n");
	pr_info("LED pulse freq 90kHz : echo 90 > psledpulsefreqsetup\n");
	pr_info("LED pulse freq 100kHz : echo 100 > psledpulsefreqsetup\n");
	pr_info("To read LED pulse freq : cat psledpulsefreqsetup\n\n");

	pr_info("Example...to write 0x13 : echo 13 > psledsetup\n");
	pr_info("To read register PS_LED (0x82) : cat psledsetup\n\n");
	/* address 0x82*/

	/* address 0x83*/
	pr_info("Address 0x83 (PS_N_PULSES)\n");
	pr_info("[pulse count num] must be 0 to 15, inclusive\n");
	pr_info("Example...to set 0 count : echo 0 > psledpulsecountsetup\n");
	pr_info("Example...to set 13 counts : echo 13 > psledpulsecountsetup\n");
	/* address 0x83*/
	/* address 0x84*/
	pr_info("Address 0x84 (PS_MEAS_RATE)\n");
	pr_info("PS meas repeat rate 50ms : echo 50 > psmeasratesetup\n");
	pr_info("PS meas repeat rate 70ms : echo 70 > psmeasratesetup\n");
	pr_info("PS meas repeat rate 100ms : echo 100 > psmeasratesetup\n");
	pr_info("PS meas repeat rate 200ms : echo 200 > psmeasratesetup\n");
	pr_info("PS meas repeat rate 500ms : echo 500 > psmeasratesetup\n");
	pr_info("PS meas repeat rate 1000ms : echo 1000 > psmeasratesetup\n");
	pr_info("PS meas repeat rate 2000ms : echo 2000 > psmeasratesetup\n");
	pr_info("PS meas repeat rate 10ms : echo 10 > psmeasratesetup\n");
	pr_info("To read register PS_MEAS_RATE (0x84) : cat psmeasratesetup\n\n");
	/* address 0x84*/

	/* address 0x85*/
	pr_info("Address 0x85 (ALS_MEAS_RATE)\n");
	pr_info("ALS meas repeat rate 50ms : echo 50 > alsmeasratesetup\n");
	pr_info("ALS meas repeat rate 100ms : echo 100 > alsmeasratesetup\n");
	pr_info("ALS meas repeat rate 200ms : echo 200 > alsmeasratesetup\n");
	pr_info("ALS meas repeat rate 500ms : echo 500 > alsmeasratesetup\n");
	pr_info("ALS meas repeat rate 1000ms : echo 1000 > alsmeasratesetup\n");
	pr_info("ALS meas repeat rate 2000ms : echo 2000 > alsmeasratesetup\n");
	pr_info("To read ALS meas repeat rate : cat alsmeasratesetup\n\n");

	pr_info("ALS integration time 100ms : echo 100 > alsintegtimesetup\n");
	pr_info("ALS integration time 50ms : echo 50 > alsintegtimesetup\n");
	pr_info("ALS integration time 200ms : echo 200 > alsintegtimesetup\n");
	pr_info("ALS integration time 400ms : echo 400 > alsintegtimesetup\n");
	pr_info("ALS integration time 150ms : echo 150 > alsintegtimesetup\n");
	pr_info("ALS integration time 250ms : echo 250 > alsintegtimesetup\n");
	pr_info("ALS integration time 300ms : echo 300 > alsintegtimesetup\n");
	pr_info("ALS integration time 350ms : echo 350 > alsintegtimesetup\n");
	pr_info("To read ALS integration time : cat alsintegtimesetup\n\n");

	pr_info("Example...to write 0x13 : echo 13 > alsmeasrateregsetup\n");
	pr_info("To read register ALS_MEAS (0x85) : cat alsmeasrateregsetup\n\n");
	/* address 0x85*/

	/* address 0x86*/
	pr_info("To read part ID : cat partid\n");
	pr_info("To read revision ID : cat revid\n");
	pr_info("To read PART_ID register (0x86) : cat partidreg\n\n");
	/* address 0x86*/

	/* address 0x87*/
	pr_info("To read manufacturing ID : cat manuid\n\n");
	/* address 0x87*/

	/* address 0x8C*/
	pr_info("Address 0x8C (ALS_PS_STATUS)\n");
	pr_info("To read PS data status : cat psdatastatus\n");
	pr_info("To read PS interrupt status : cat psinterruptstatus\n");
	pr_info("To read ALS data status : cat alsdatastatus\n");
	pr_info("To read ALS interrupt status : cat alsinterruptstatus\n");
	pr_info("To read ALS gain status : cat alsgainstatus\n");
	pr_info("To read ALS validity status : cat alsdatavaliditystatus\n");
	pr_info("To read register ALS_PS_STATUS (0x8C) : cat alspsstatusreg\n\n");
	/* address 0x8C*/

	/* address 0x88, 0x89, 0x8A, 0x8B*/
	pr_info("ALS raw and calculated data, address 0x88, 0x89, 0x8A, 0x8B\n");
	pr_info("To read raw and calculated ALS data : cat alsch0ch1rawcalc\n\n");
	/* address 0x88, 0x89, 0x8A, 0x8B*/

	/* address 0x94, 0x95*/
	pr_info("Example...to write 55 : echo 55 > setpsoffset\n");
	pr_info("To read back the offset value : cat setpsoffset\n\n");
	/* address 0x94, 0x95*/

	/* address 0x8F*/
	pr_info("Address 0x8F (INTERRUPT)\n");
	pr_info("INT output pin inactive : echo 0 > interruptmodesetup\n");
	pr_info("Only PS triggers interrupt : echo 1 > interruptmodesetup\n");
	pr_info("Only ALS triggers interrupt : echo 2 > interruptmodesetup\n");
	pr_info("Both ALS PS trigger interrupt : echo 3 > interruptmodesetup\n");
	pr_info("To read interrupt mode : cat interruptmodesetup\n\n");

	pr_info("INT output pin active low : echo 0 > interruptpolarsetup\n");
	pr_info("INT output pin active high : echo 1 > interruptpolarsetup\n");
	pr_info("To read interrupt pin polarity : cat interruptpolarsetup\n\n");

	pr_info("Example...to write 0x13 : echo 13 > interruptsetup\n");
	pr_info("To read register INTERRUPT (0x8F) : cat interruptsetup\n\n");
	/* address 0x8F*/

	/* address 0x9E*/
	pr_info("Address 0x9E (INTERRUPT PERSIST)\n");
	pr_info("Example...to write 0x13 : echo 13 > interruptpersistsetup\n");
	/* address 0x9E*/

	/* ALS threshold setting*/
	pr_info("To read the threshold values : cat dispalsthrerange\n\n");
	/* ALS threshold setting*/

	/* PS threshold setting*/
	pr_info("PS threshold setting 0x90, 0x91, 0x92, 0x93\n");
	pr_info("To read the threshold values : cat disppsthrerange\n\n");
	/* PS threshold setting*/

	return 0;
}

//static DEVICE_ATTR(ltr559help, 0444, ltr559help_show, NULL);


static ssize_t alsmodesetup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = als_contr_readback(ALS_MODE_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: ALS_MODE_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	//ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);
	ret = sprintf(buf, "%d\n", rdback_val);
	return ret;
}

static ssize_t alsmodesetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int8_t ret;
	struct ltr559_data *ltr559 = sensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&ltr559->i2c_client->dev,
				"%s: store value = %d\n", __func__, param);

ret = als_mode_setup((uint8_t)param, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
				"%s: ALS mode setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

//static DEVICE_ATTR(enable_als_sensor, 0664,alsmodesetup_show, alsmodesetup_store);


static ssize_t alsswresetsetup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = als_contr_readback(ALS_SWRT_RDBCK, &rdback_val, ltr559);

	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: ALS_SWRT_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	//ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);
	ret = sprintf(buf, "%d\n", rdback_val);
	return ret;
}


static ssize_t alsswresetsetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int8_t ret;

	struct ltr559_data *ltr559 = sensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&ltr559->i2c_client->dev,
			"%s: store value = %d\n", __func__, param);

	ret = als_sw_reset_setup((uint8_t)param, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: ALS sw reset setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

//static DEVICE_ATTR(alsswresetsetup, 0664, alsswresetsetup_show, alsswresetsetup_store);


static ssize_t alsgainsetup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = als_contr_readback(ALS_GAIN_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: ALS_GAIN_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	//ret = snprintf(buf, sizeof(buf), "%d\n", rdback_val);
	ret = sprintf(buf, "%d\n", rdback_val);
	return ret;
}


static ssize_t alsgainsetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	/*int *param_temp = buf;*/
	int param_temp[2];

	struct ltr559_data *ltr559 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
	} else if (count == 2) {
		param_temp[0] -= 48;
		param_temp[1] = 0;

		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count >= 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
	}

	param = ((param_temp[0] * 10) + param_temp[1]);
	dev_dbg(&ltr559->i2c_client->dev,
				"%s: store value = %d\n", __func__, param);

	ret = als_gain_setup((uint8_t)param, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
				"%s: ALS gain setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

static ssize_t alscontrsetup_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = als_contr_readback(ALS_CONTR_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: ALS_CONTR_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

//static DEVICE_ATTR(alsgainsetup, 0664, alsgainsetup_show, alsgainsetup_store);

static ssize_t alscontrsetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	/*int *param_temp = buf;*/
	int param_temp[2];

	struct ltr559_data *ltr559 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 48;
		param_temp[1] = 48;
	} else if (count == 2) {
		param_temp[1] = param_temp[0];
		param_temp[0] = 48;
	}


	if (param_temp[0] >= 65 && param_temp[0] <= 70)
		param_temp[0] -= 55;
	else if (param_temp[0] >= 97 && param_temp[0] <= 102)
		param_temp[0] -= 87;
	else if (param_temp[0] >= 48 && param_temp[0] <= 57)
		param_temp[0] -= 48;
	else
		param_temp[0] = 0;

	if (param_temp[1] >= 65 && param_temp[1] <= 70)
		param_temp[1] -= 55;
	else if (param_temp[1] >= 97 && param_temp[1] <= 102)
		param_temp[1] -= 87;
	else if (param_temp[1] >= 48 && param_temp[1] <= 57)
		param_temp[1] -= 48;
	else
		param_temp[1] = 0;

	param = ((param_temp[0] << 4) + (param_temp[1]));
	dev_dbg(&ltr559->i2c_client->dev,
			"%s: store value = %d\n", __func__, param);

	ret = als_contr_setup(param, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
				"%s: ALS contr setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

//static DEVICE_ATTR(alscontrsetup, 0664, alscontrsetup_show,	alscontrsetup_store);

static ssize_t psmodesetup_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = ps_contr_readback(PS_MODE_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: PS_MODE_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t psmodesetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int8_t ret;
	struct ltr559_data *ltr559 = sensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&ltr559->i2c_client->dev,
		"%s: store value = %d\n", __func__, param);

	ret = ps_mode_setup((uint8_t)param, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
				"%s: PS mode setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;

}

//static DEVICE_ATTR(enable_ps_sensor, 0664, psmodesetup_show, psmodesetup_store);


static ssize_t psgainsetup_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = ps_contr_readback(PS_GAIN_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: PS_GAIN_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t psgainsetup_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	uint8_t param;
	int8_t ret;
	int param_temp[2];

	struct ltr559_data *ltr559 = sensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 2) {
		param_temp[0] = 0;
		param_temp[1] = 0;
	} else if (count >= 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
	}

	param = ((param_temp[0] * 10) + param_temp[1]);

	dev_dbg(&ltr559->i2c_client->dev,
	"%s: store value = %d\n", __func__, param);

	ret = ps_gain_setup(param, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: PS gain setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

//static DEVICE_ATTR(psgainsetup, 0664, psgainsetup_show, psgainsetup_store);


static ssize_t pssatuindicasetup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = ps_contr_readback(PS_SATUR_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: PS_SATUR_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t pssatuindicasetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int8_t ret;
	struct ltr559_data *ltr559 = sensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&ltr559->i2c_client->dev,
		"%s: store value = %d\n", __func__, param);

	ret = ps_satu_indica_setup((uint8_t)param, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: PS saturation indicator setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

//static DEVICE_ATTR(pssatuindicasetup, 0664, pssatuindicasetup_show, pssatuindicasetup_store);


static ssize_t pscontrsetup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
struct ltr559_data *ltr559 = sensor_info;

	ret = ps_contr_readback(PS_CONTR_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: PS_CONTR_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t pscontrsetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	/*int *param_temp = buf;*/
	int param_temp[2];

	struct ltr559_data *ltr559 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 48;
		param_temp[1] = 48;
	} else if (count == 2) {
		param_temp[1] = param_temp[0];
		param_temp[0] = 48;
	}

	if (param_temp[0] >= 65 && param_temp[0] <= 70)
		param_temp[0] -= 55;
	else if (param_temp[0] >= 97 && param_temp[0] <= 102)
		param_temp[0] -= 87;
	else if (param_temp[0] >= 48 && param_temp[0] <= 57)
		param_temp[0] -= 48;
	else
		param_temp[0] = 0;

	if (param_temp[1] >= 65 && param_temp[1] <= 70)
		param_temp[1] -= 55;
	else if (param_temp[1] >= 97 && param_temp[1] <= 102)
		param_temp[1] -= 87;
	else if (param_temp[1] >= 48 && param_temp[1] <= 57)
		param_temp[1] -= 48;
	else
		param_temp[1] = 0;

	param = ((param_temp[0] << 4) + (param_temp[1]));
	dev_dbg(&ltr559->i2c_client->dev,
		"%s: store value = %d\n", __func__, param);

	ret = ps_contr_setup(param, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: PS contr setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

//static DEVICE_ATTR(pscontrsetup, 0664, pscontrsetup_show, pscontrsetup_store);


static ssize_t psledcurrsetup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = ps_led_readback(LED_CURR_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
				"%s: LED_CURR_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;

}


static ssize_t psledcurrsetup_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	/*int *param_temp = buf;*/
	int param_temp[3];

	struct ltr559_data *ltr559 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];

	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
	} else if (count == 2) {
		param_temp[0] -= 48;
		param_temp[1] = 0;
		param_temp[2] = 0;

		param_temp[2] = param_temp[0];
		param_temp[0] = 0;
		param_temp[1] = 0;
	} else if (count == 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;

		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count > 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
	}

	param = ((param_temp[0] * 100) + (param_temp[1] * 10) + param_temp[2]);
	dev_dbg(&ltr559->i2c_client->dev,
		"%s: store value = %d\n", __func__, param);

	ret = ps_ledCurrent_setup(param, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: PS LED current setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;

}

//static DEVICE_ATTR(psledcurrsetup, 0664,psledcurrsetup_show, psledcurrsetup_store);


static ssize_t psledcurrduty_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = ps_led_readback(LED_CURR_DUTY_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: LED_CURR_DUTY_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;

}


static ssize_t psledcurrduty_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	/*int *param_temp = buf;*/
	int param_temp[3];

	struct ltr559_data *ltr559 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];

	if (count < 3) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
	} else if (count == 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;

		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count > 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
	}

	param = ((param_temp[0] * 100) + (param_temp[1] * 10) + param_temp[2]);
	dev_dbg(&ltr559->i2c_client->dev,
			"%s: store value = %d\n", __func__, param);

	ret = ps_ledCurrDuty_setup(param, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: PS LED curent duty setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

//static DEVICE_ATTR(psledcurrduty, 0664,	psledcurrduty_show, psledcurrduty_store);


static ssize_t psledpulsefreqsetup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = ps_led_readback(LED_PUL_FREQ_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: LED_PUL_FREQ_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;

}


static ssize_t psledpulsefreqsetup_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	/*int *param_temp = buf;*/
	int param_temp[3];

	struct ltr559_data *ltr559 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];

	if (count < 3) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
	} else if (count == 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;

		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count > 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
	}

	param = ((param_temp[0] * 100) + (param_temp[1] * 10) + param_temp[2]);
	dev_dbg(&ltr559->i2c_client->dev,
		"%s: store value = %d\n", __func__, param);

	ret = ps_ledPulseFreq_setup(param, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: PS LED pulse frequency setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

//static DEVICE_ATTR(psledpulsefreqsetup, 0664,	psledpulsefreqsetup_show, psledpulsefreqsetup_store);


static ssize_t psledsetup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = ps_led_readback(PS_LED_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: PS_LED_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t psledsetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	/*int *param_temp = buf;*/
	int param_temp[2];

	struct ltr559_data *ltr559 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 48;
		param_temp[1] = 48;
	} else if (count == 2) {
	param_temp[1] = param_temp[0];
		param_temp[0] = 48;
	}

	if (param_temp[0] >= 65 && param_temp[0] <= 70)
		param_temp[0] -= 55;
	else if (param_temp[0] >= 97 && param_temp[0] <= 102)
		param_temp[0] -= 87;
	else if (param_temp[0] >= 48 && param_temp[0] <= 57)
		param_temp[0] -= 48;
	else
		param_temp[0] = 0;

	if (param_temp[1] >= 65 && param_temp[1] <= 70)
		param_temp[1] -= 55;
	else if (param_temp[1] >= 97 && param_temp[1] <= 102)
		param_temp[1] -= 87;
	else if (param_temp[1] >= 48 && param_temp[1] <= 57)
		param_temp[1] -= 48;
	else
		param_temp[1] = 0;

	param = ((param_temp[0] << 4) + (param_temp[1]));
	dev_dbg(&ltr559->i2c_client->dev,
		"%s: store value = %d\n", __func__, param);

	ret = ps_led_setup(param, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: PS LED setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

//static DEVICE_ATTR(psledsetup, 0664, psledsetup_show, psledsetup_store);


static ssize_t psledpulsecountsetup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = ps_ledPulseCount_readback(&rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: PS LED pulse count readback Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t psledpulsecountsetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	/*int *param_temp = buf;*/
	int param_temp[2];

	struct ltr559_data *ltr559 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if ((count <= 1) || (count > 3)) {
		param_temp[0] = 0;
		param_temp[1] = 0;
	} else if (count == 2) {
		param_temp[0] -= 48;
		param_temp[1] = 0;

		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count == 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
	}

	param = ((param_temp[0] * 10) + param_temp[1]);
	if (param > 15)
		param = 15;

	dev_dbg(&ltr559->i2c_client->dev,
		"%s: store value = %d\n", __func__, param);

	ret = ps_ledPulseCount_setup(param, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: PS LED pulse count setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

//static DEVICE_ATTR(psledpulsecountsetup, 0664,		psledpulsecountsetup_show, psledpulsecountsetup_store);


static ssize_t psmeasratesetup_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = ps_meas_rate_readback(&rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: PS meas rate readback Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;

}


static ssize_t psmeasratesetup_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint16_t param;
	/*int *param_temp = buf;*/
	int param_temp[4];

	struct ltr559_data *ltr559 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];

	if (count <= 2) {
	param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
	} else if (count == 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;

		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count > 4) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;
	}

	param = ((param_temp[0] * 1000) + (param_temp[1] * 100) +
				(param_temp[2] * 10) + param_temp[3]);
	dev_dbg(&ltr559->i2c_client->dev,
			"%s: store value = %d\n", __func__, param);

	ret = ps_meas_rate_setup(param, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: PS measurement rate setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

//static DEVICE_ATTR(psmeasratesetup, 0664,	psmeasratesetup_show, psmeasratesetup_store);


static ssize_t alsmeasratesetup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = als_meas_rate_readback(ALS_MEAS_RPT_RATE_RDBCK,
		&rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: ALS_MEAS_RPT_RATE_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;

}


static ssize_t alsmeasratesetup_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint16_t param;
	/*int *param_temp = buf;*/
	int param_temp[4];

	struct ltr559_data *ltr559 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];

	if (count <= 2) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
	} else if (count == 3) {
	param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;

		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count > 4) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;
	}

	param = ((param_temp[0] * 1000) + (param_temp[1] * 100) +
				(param_temp[2] * 10) + param_temp[3]);
	dev_dbg(&ltr559->i2c_client->dev,
			"%s: store value = %d\n", __func__, param);
	ret = als_meas_rate_setup(param, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: ALS measurement rate setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

////static DEVICE_ATTR(alsmeasratesetup, 0664,		alsmeasratesetup_show, alsmeasratesetup_store);


static ssize_t alsintegtimesetup_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = als_meas_rate_readback(ALS_INTEG_TM_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
				"%s: ALS_INTEG_TM_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;

}


static ssize_t alsintegtimesetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint16_t param;
	/*int *param_temp = buf;*/

	int param_temp[3];

	struct ltr559_data *ltr559 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];

	if (count <= 2) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
	} else if (count == 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;

		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count > 3) {
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
	}

	param = ((param_temp[0] * 100) + (param_temp[1] * 10) + param_temp[2]);
	dev_dbg(&ltr559->i2c_client->dev,
			"%s: store value = %d\n", __func__, param);

	ret = als_integ_time_setup(param, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: ALS integration time setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

//static DEVICE_ATTR(alsintegtimesetup, 0664,		alsintegtimesetup_show, alsintegtimesetup_store);


static ssize_t alsmeasrateregsetup_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = als_meas_rate_readback(ALS_MEAS_RATE_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: ALS_MEAS_RATE_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;

}


static ssize_t alsmeasrateregsetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	/*int *param_temp = buf;*/
	int param_temp[2];

	struct ltr559_data *ltr559 = sensor_info;

/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 48;
		param_temp[1] = 48;
	} else if (count == 2) {
		param_temp[1] = param_temp[0];
		param_temp[0] = 48;
	}

	if (param_temp[0] >= 65 && param_temp[0] <= 70)
		param_temp[0] -= 55;
else if (param_temp[0] >= 97 && param_temp[0] <= 102)
		param_temp[0] -= 87;
	else if (param_temp[0] >= 48 && param_temp[0] <= 57)
		param_temp[0] -= 48;
	else
		param_temp[0] = 0;

	if (param_temp[1] >= 65 && param_temp[1] <= 70)
		param_temp[1] -= 55;
	else if (param_temp[1] >= 97 && param_temp[1] <= 102)
		param_temp[1] -= 87;
	else if (param_temp[1] >= 48 && param_temp[1] <= 57)
		param_temp[1] -= 48;
	else
		param_temp[1] = 0;

	param = ((param_temp[0] << 4) + (param_temp[1]));
	dev_dbg(&ltr559->i2c_client->dev,
		"%s: store value = %d\n", __func__, param);

	ret = als_meas_rate_reg_setup(param, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: ALS meas rate register setup Fail...\n", __func__);
		return -EPERM;
}

	return count;
}

//static DEVICE_ATTR(alsmeasrateregsetup, 0664,		alsmeasrateregsetup_show, alsmeasrateregsetup_store);


static ssize_t partid_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

ret = part_ID_reg_readback(PART_NUM_ID_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
				"%s: PART_NUM_ID_RDBCK Fail...\n", __func__);
	return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

//static DEVICE_ATTR(partid, 0444, partid_show, NULL);


static ssize_t revid_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = part_ID_reg_readback(REVISION_ID_RDBCK, &rdback_val, ltr559);
if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
				"%s: REVISION_ID_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

//static DEVICE_ATTR(revid, 0444, revid_show, NULL);


static ssize_t partidreg_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = part_ID_reg_readback(PART_ID_REG_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
				"%s: PART_ID_REG_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

//static DEVICE_ATTR(partidreg, 0444, partidreg_show, NULL);


static ssize_t manuid_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = manu_ID_reg_readback(&rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: Manufacturing ID readback Fail...\n", __func__);
		return -EPERM;
	}
	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}
//static DEVICE_ATTR(manuid, 0444, manuid_show, NULL);


static ssize_t psdatastatus_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = als_ps_status_reg(PS_DATA_STATUS_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: PS_DATA_STATUS_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

//static DEVICE_ATTR(psdatastatus, 0444, psdatastatus_show, NULL);


static ssize_t psinterruptstatus_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
struct ltr559_data *ltr559 = sensor_info;

	ret = als_ps_status_reg(PS_INTERR_STATUS_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: PS_INTERR_STATUS_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

//static DEVICE_ATTR(psinterruptstatus, 0444, psinterruptstatus_show, NULL);


static ssize_t alsdatastatus_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = als_ps_status_reg(ALS_DATA_STATUS_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: ALS_DATA_STATUS_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

//static DEVICE_ATTR(alsdatastatus, 0444, alsdatastatus_show, NULL);


static ssize_t alsinterruptstatus_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = als_ps_status_reg(ALS_INTERR_STATUS_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: ALS_INTERR_STATUS_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

//static DEVICE_ATTR(alsinterruptstatus, 0444, alsinterruptstatus_show, NULL);


static ssize_t alsgainstatus_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = als_ps_status_reg(ALS_GAIN_STATUS_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: ALS_GAIN_STATUS_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

return ret;
}

//static DEVICE_ATTR(alsgainstatus, 0444, alsgainstatus_show, NULL);


static ssize_t alsdatavaliditystatus_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = als_ps_status_reg(ALS_VALID_STATUS_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: ALS_VALID_STATUS_RDBCK Fail...\n", __func__);
		return -EPERM;
	}
	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}

//static DEVICE_ATTR(alsdatavaliditystatus, 0444,				alsdatavaliditystatus_show, NULL);


static ssize_t alspsstatusreg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = als_ps_status_reg(ALS_PS_STATUS_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: ALS_PS_STATUS_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;

}

//static DEVICE_ATTR(alspsstatusreg, 0444, alspsstatusreg_show, NULL);

static ssize_t alsch0ch1rawcalc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint16_t rdback_val1 = 0, rdback_val2 = 0, rdback_val3 = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = als_ch0ch1raw_calc_readback(&rdback_val1,
				&rdback_val2, &rdback_val3, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: ALS CH0 CH1 Calc reading readback Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d %d %d\n",
			rdback_val1, rdback_val2, rdback_val3);

	return ret;

}

//static DEVICE_ATTR(alsch0ch1rawcalc, 0444, alsch0ch1rawcalc_show, NULL);


static ssize_t setpsoffset_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint16_t rdback_val;
	struct ltr559_data *ltr559 = sensor_info;

	ret = ps_offset_readback(&rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
				"%s: PS offset readback Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t setpsoffset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint16_t ps_offset = 0;
	uint8_t param_temp[4];
	struct ltr559_data *ltr559 = sensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];

	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
	} else if (count == 2) { /* 1 digit*/
		param_temp[0] -= 48;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[0];
		param_temp[2] = 0;
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 3) { /* 2 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) { /* 3 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;

		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count >= 5) { /* 4 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;
	}

	ps_offset = ((param_temp[0] * 1000) + (param_temp[1] * 100) +
					(param_temp[2] * 10) + param_temp[3]);
	if (ps_offset > 1023)
		ps_offset = 1023;

	dev_dbg(&ltr559->i2c_client->dev,
			"%s: store value = %d\n", __func__, ps_offset);

	ret = ps_offset_setup(ps_offset, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
				"%s: set ps offset Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

//static DEVICE_ATTR(setpsoffset, 0664, setpsoffset_show, setpsoffset_store);

static ssize_t interruptmodesetup_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = interrupt_readback(INT_MODE_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
				"%s: INT_MODE_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t interruptmodesetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int8_t ret;

	struct ltr559_data *ltr559 = sensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&ltr559->i2c_client->dev,
				"%s: store value = %d\n", __func__, param);

	ret = interrupt_mode_setup((uint8_t)param, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: interrupt mode setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

//static DEVICE_ATTR(interruptmodesetup, 0664,		interruptmodesetup_show, interruptmodesetup_store);


static ssize_t interruptpolarsetup_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = interrupt_readback(INT_POLAR_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
				"%s: INT_POLAR_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t interruptpolarsetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int param;
	int8_t ret;

	struct ltr559_data *ltr559 = sensor_info;

	sscanf(buf, "%d", &param);
	dev_dbg(&ltr559->i2c_client->dev,
			"%s: store value = %d\n", __func__, param);

	ret = interrupt_polarity_setup((uint8_t)param, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: interrupt polarity setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

//static DEVICE_ATTR(interruptpolarsetup, 0664,			interruptpolarsetup_show, interruptpolarsetup_store);


static ssize_t interruptsetup_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = interrupt_readback(INT_INTERRUPT_RDBCK, &rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: INT_INTERRUPT_RDBCK Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t interruptsetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint8_t param;
	/*int *param_temp = buf;*/
	int param_temp[2];

	struct ltr559_data *ltr559 = sensor_info;

	/*sscanf(buf, "%d", param_temp);*/
	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 48;
		param_temp[1] = 48;
	} else if (count == 2) {
		param_temp[1] = param_temp[0];
		param_temp[0] = 48;
	}

	if (param_temp[0] >= 65 && param_temp[0] <= 70)
		param_temp[0] -= 55;
	else if (param_temp[0] >= 97 && param_temp[0] <= 102)
		param_temp[0] -= 87;
	else if (param_temp[0] >= 48 && param_temp[0] <= 57)
		param_temp[0] -= 48;
	else
		param_temp[0] = 0;

	if (param_temp[1] >= 65 && param_temp[1] <= 70)
		param_temp[1] -= 55;
	else if (param_temp[1] >= 97 && param_temp[1] <= 102)
		param_temp[1] -= 87;
	else if (param_temp[1] >= 48 && param_temp[1] <= 57)
		param_temp[1] -= 48;
	else
		param_temp[1] = 0;

	param = ((param_temp[0] << 4) + (param_temp[1]));
	dev_dbg(&ltr559->i2c_client->dev,
				"%s: store value = %d\n", __func__, param);

	ret = interrupt_setup(param, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
				"%s: interrupt setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

//static DEVICE_ATTR(interruptsetup, 0664,			interruptsetup_show, interruptsetup_store);


static ssize_t interruptpersistsetup_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint8_t rdback_val = 0;
	struct ltr559_data *ltr559 = sensor_info;

	ret = interrupt_prst_readback(&rdback_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: Interrupt persist readback Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d\n", rdback_val);

	return ret;
}


static ssize_t interruptpersistsetup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret = 0;
	/*uint8_t als_or_ps, prst_val;*/
	uint8_t prst_val;
	/*int *param_temp = buf;*/
	int param_temp[2];

	struct ltr559_data *ltr559 = sensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];

	if (count <= 1) {
		param_temp[0] = 48;
		param_temp[1] = 48;
} else if (count == 2) {
		param_temp[1] = param_temp[0];
		param_temp[0] = 48;
	}

	if (param_temp[0] >= 65 && param_temp[0] <= 70)
		param_temp[0] -= 55;
	else if (param_temp[0] >= 97 && param_temp[0] <= 102)
		param_temp[0] -= 87;
	else if (param_temp[0] >= 48 && param_temp[0] <= 57)
		param_temp[0] -= 48;
	else
		param_temp[0] = 0;

	if (param_temp[1] >= 65 && param_temp[1] <= 70)
		param_temp[1] -= 55;
	else if (param_temp[1] >= 97 && param_temp[1] <= 102)
	param_temp[1] -= 87;
	else if (param_temp[1] >= 48 && param_temp[1] <= 57)
		param_temp[1] -= 48;
	else
		param_temp[1] = 0;

	prst_val = ((param_temp[0] << 4) + (param_temp[1]));
	dev_dbg(&ltr559->i2c_client->dev,
			"%s: store value = %d\n", __func__, prst_val);

	/*ret = interrupt_persist_setup(als_or_ps, prst_val, ltr559);*/
	ret = interrupt_persist_setup(prst_val, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: Interrupt persist setup Fail...\n", __func__);
		return -EPERM;
	}

	return count;

}

//static DEVICE_ATTR(interruptpersistsetup, 0664,	interruptpersistsetup_show, interruptpersistsetup_store);


static ssize_t setalslothrerange_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	int lo_thr = 0;
	uint8_t param_temp[5];
	struct ltr559_data *ltr559 = sensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];
	param_temp[4] = buf[4];

	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
	param_temp[2] = 0;
		param_temp[3] = 0;
		param_temp[4] = 0;
	} else if (count == 2) { /* 1 digit*/
		param_temp[0] -= 48;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
		param_temp[4] = 0;

		param_temp[4] = param_temp[0];
		param_temp[3] = 0;
		param_temp[2] = 0;
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 3) { /*2 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;
		param_temp[4] = 0;

		param_temp[4] = param_temp[1];
		param_temp[3] = param_temp[0];
		param_temp[2] = 0;
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) { /* 3 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;
		param_temp[4] = 0;

	param_temp[4] = param_temp[2];
		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 5) { /* 4 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;
		param_temp[4] = 0;

		param_temp[4] = param_temp[3];
		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count >= 6) { /* 5 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
	param_temp[3] -= 48;
		param_temp[4] -= 48;
	}

	lo_thr = ((param_temp[0] * 10000) + (param_temp[1] * 1000) +
	(param_temp[2] * 100) + (param_temp[3] * 10) + param_temp[4]);
	if (lo_thr > 65535)
		lo_thr = 65535;
	dev_dbg(&ltr559->i2c_client->dev,
			"%s: store value = %d\n", __func__, lo_thr);

	ret = set_als_range((uint16_t)lo_thr, 0, LO_LIMIT);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: set ALS lo threshold Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

//static DEVICE_ATTR(setalslothrerange, 0220, NULL, setalslothrerange_store);


static ssize_t setalshithrerange_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	int hi_thr = 0;
	uint8_t param_temp[5];
	struct ltr559_data *ltr559 = sensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];
	param_temp[4] = buf[4];

	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
		param_temp[4] = 0;
	} else if (count == 2) { /* 1 digit*/
		param_temp[0] -= 48;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
		param_temp[4] = 0;

		param_temp[4] = param_temp[0];
		param_temp[3] = 0;
		param_temp[2] = 0;
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 3) { /* 2 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;
		param_temp[4] = 0;

		param_temp[4] = param_temp[1];
		param_temp[3] = param_temp[0];
		param_temp[2] = 0;
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) { /* 3 digits*/
		param_temp[0] -= 48; 
	  param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;
		param_temp[4] = 0;

		param_temp[4] = param_temp[2];
		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 5) { /* 4 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
	param_temp[3] -= 48;
		param_temp[4] = 0;

		param_temp[4] = param_temp[3];
		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count >= 6) { /* 5 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;
		param_temp[4] -= 48;
	}

	hi_thr = ((param_temp[0] * 10000) + (param_temp[1] * 1000) +
	(param_temp[2] * 100) + (param_temp[3] * 10) + param_temp[4]);
	if (hi_thr > 65535)
		hi_thr = 65535;
	dev_dbg(&ltr559->i2c_client->dev,
			"%s: store value = %d\n", __func__, hi_thr);

ret = set_als_range(0, (uint16_t)hi_thr, HI_LIMIT);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: set ALS hi threshold Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

//static DEVICE_ATTR(setalshithrerange, 0220, NULL, setalshithrerange_store);

static ssize_t dispalsthrerange_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint16_t rdback_lo, rdback_hi;
	struct ltr559_data *ltr559 = sensor_info;

	ret = als_range_readback(&rdback_lo, &rdback_hi, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
			"%s: ALS threshold range readback Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d %d\n", rdback_lo, rdback_hi);

	return ret;
}

//static DEVICE_ATTR(dispalsthrerange, 0444, dispalsthrerange_show, NULL);


static ssize_t setpslothrerange_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint16_t lo_thr = 0;
	uint8_t param_temp[4];
	struct ltr559_data *ltr559 = sensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];

	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
	} else if (count == 2) { /* 1 digit*/
		param_temp[0] -= 48;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[0];
		param_temp[2] = 0;
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 3) { /* 2 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) { /* 3 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;

		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count >= 5) { /* 4 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;
	}

	lo_thr = ((param_temp[0] * 1000) + (param_temp[1] * 100) +
				(param_temp[2] * 10) + param_temp[3]);
	if (lo_thr > 2047)
		lo_thr = 2047;
	dev_dbg(&ltr559->i2c_client->dev,
			"%s: store value = %d\n", __func__, lo_thr);

	ret = set_ps_range(lo_thr, 0, LO_LIMIT, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
				"%s: set PS lo threshold Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

//static DEVICE_ATTR(setpslothrerange, 0220, NULL, setpslothrerange_store);


static ssize_t setpshithrerange_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int8_t ret;
	uint16_t hi_thr = 0;
	uint8_t param_temp[4];
	struct ltr559_data *ltr559 = sensor_info;

	param_temp[0] = buf[0];
	param_temp[1] = buf[1];
	param_temp[2] = buf[2];
	param_temp[3] = buf[3];

	if (count <= 1) {
		param_temp[0] = 0;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;
	} else if (count == 2) { /* 1 digit*/
		param_temp[0] -= 48;
		param_temp[1] = 0;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[0];
	param_temp[2] = 0;
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 3) { /* 2 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] = 0;
		param_temp[3] = 0;

		param_temp[3] = param_temp[1];
		param_temp[2] = param_temp[0];
		param_temp[1] = 0;
		param_temp[0] = 0;
	} else if (count == 4) { /* 3 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] = 0;

		param_temp[3] = param_temp[2];
		param_temp[2] = param_temp[1];
		param_temp[1] = param_temp[0];
		param_temp[0] = 0;
	} else if (count >= 5) { /* 4 digits*/
		param_temp[0] -= 48;
		param_temp[1] -= 48;
		param_temp[2] -= 48;
		param_temp[3] -= 48;
	}

	hi_thr = ((param_temp[0] * 1000) + (param_temp[1] * 100) +
				(param_temp[2] * 10) + param_temp[3]);
	if (hi_thr > 2047)
		hi_thr = 2047;
	dev_dbg(&ltr559->i2c_client->dev,
			"%s: store value = %d\n", __func__, hi_thr);

	ret = set_ps_range(0, hi_thr, HI_LIMIT, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
				"%s: set PS hi threshold Fail...\n", __func__);
		return -EPERM;
	}

	return count;
}

//static DEVICE_ATTR(setpshithrerange, 0220, NULL, setpshithrerange_store);


static ssize_t disppsthrerange_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int8_t ret = 0;
	uint16_t rdback_lo, rdback_hi;
	struct ltr559_data *ltr559 = sensor_info;

	ret = ps_range_readback(&rdback_lo, &rdback_hi, ltr559);
	if (ret < 0) {
		dev_err(&ltr559->i2c_client->dev,
		"%s: PS threshold range readback Fail...\n", __func__);
		return -EPERM;
	}

	ret = sprintf(buf, "%d %d\n", rdback_lo, rdback_hi);

	return ret;
}

//static DEVICE_ATTR(disppsthrerange, 0444, disppsthrerange_show, NULL);
//yanglimin add for dump reg,start
static ssize_t ltr559_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int8_t ret = 0, i;
	ssize_t count=0;
	uint8_t buffer[26];

    LTR559_DBG("%s, in\n", __func__);
	
    buffer[0] = 0x80;
	ret = I2C_Read(buffer, 6);
    for(i = 0; i < 6; i++)
    {
    	count += snprintf(buf+count, 512-count, "R[%X]=%X\n", 0X80+i, buffer[i]);
    }

    buffer[0] = 0x8c;
	ret = I2C_Read(buffer, 4);
	for(i = 0; i < 4; i++)
    {
    	count += snprintf(buf+count, 512-count, "R[%X]=%X\n", 0X8c+i, buffer[i]);
    }
	

	if (ret < 0) {
		LTR559_DBG("%s, I2C_Read error\n", __func__);
		return -EPERM;
	}
	
	return count;
}

//xym add begin
static void check_prox_mean(int prox_mean ,int *detection_threshold, int *hsyteresis_threshold);

static ssize_t ltr559_calibrate_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ltr559_data *data = sensor_info;
	LTR559_DBG("%s: %d,%d,%d,%d\n", __FUNCTION__, 
		data->default_ps_highthresh,
		0,//data->led_pulse_count,
		0,
		data->default_ps_lowthresh);
	return sprintf(buf, "%d,%d,%d,%d\n", 
		data->default_ps_highthresh,
		0,//data->led_pulse_count,
		0,
		data->default_ps_lowthresh);
}

//+++Add offset solution ZTE_SNS_LPZ_20150907
#define LTR559_CALI_CNT 20
#define CT_LIMIT_LOW  10
#define CT_LIMIT_HIGH 1200 //change from 800 to 1200
static ssize_t ltr559_calibrate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ltr559_data *data = sensor_info;
	uint16_t cross_talk = 0;
    //uint16_t offset = 0;
    int ret=0;
	
	LTR559_DBG("%s: +++\n", __FUNCTION__);
	
	if(data->enable_ps_sensor==0)
	{
		LTR559_DBG("%s: prox now is power off\n", __FUNCTION__);
		return 3;
	}
    
	LTR559_DBG("%s: prox now is power on and begin to do calibrate\n", __FUNCTION__);	
	LTR559_DBG("%s: old ps thresh is %d %d\n", 
		__FUNCTION__, data->default_ps_highthresh, data->default_ps_lowthresh);

    //set offset to 0 for checking the real crosstalk
    //Disable offset ZTE_SNS_LPZ_20150922
	//ps_offset_setup2(0);
	//msleep(30);	

    //Get crosstalk for checking
	cross_talk = getCrosstalk(LTR559_CALI_CNT);
	LTR559_DBG("%s: cross_talk=%d for checking\n", __FUNCTION__, cross_talk);

    //Check whether it is one good prox or not
	if((cross_talk<CT_LIMIT_LOW)||(cross_talk>CT_LIMIT_HIGH)){
		ret = -110;
		LTR559_DBG("%s: cross_talk is not in the range of 10-1200\n", __FUNCTION__);
		LTR559_DBG("%s: ---return %d\n", __FUNCTION__, ret);
		return ret;
	}
	
	/*added by Wee Liat for offset cencellation*/
    //Disable offset ZTE_SNS_LPZ_20150922
	//offset=offset_cancellation(cross_talk);		

    //Get crosstalk for geting threshold
    //Disable offset ZTE_SNS_LPZ_20150922
	//cross_talk -= offset;
	//LTR559_DBG("%s: cross_talk=%d for getting threshold(offset=%d)\n", __FUNCTION__, cross_talk, offset);

    //Update prox threshold
	ret = updateThreshold(cross_talk);
    LTR559_DBG("%s: ---return %d\n", __FUNCTION__, ret);
    
	return ret;
}
//---

static ssize_t ltr559_set_calibrate_data_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
  struct ltr559_data *data = sensor_info;

  int i, j;
  u16 prox_threshold_hi = 0;
  u16 prox_threshold_lo = 0;
  u8 prox_pulse_cnt = 0;
  u8 prox_gain = 0;
  char cal_data_char[4][20] = {{0},{0},{0},{0}};
  char buf_data[100] = {0};
  char *tmp = buf_data;
  
  LTR559_DBG("%s: +++\n", __FUNCTION__); 
  LTR559_DBG("%s: user insert buf is %s\n", __FUNCTION__, buf); 
  sprintf(buf_data, "%s\n", buf);
  LTR559_DBG("%s: user insert buf_data is %s\n", __FUNCTION__, buf_data);
  
  for(i = 0, j = 0; *tmp != '\0' && i <= 3 && j < 20; ) 
  {
    if(*tmp  == ',') 
    {
      *(cal_data_char[i] +j) = '\0';
      i ++;
      j = 0;
      if (i >3 ) 
      {
        LTR559_DBG("%s: sensor_hal: array bounds!\n", __FUNCTION__);
        return 0;
      }
    }
    if((*tmp >= '0') && (*tmp <= '9')) 
    {
      *(cal_data_char[i] +j) = *tmp;
      j ++;
    }
    tmp ++;
  }
  
  prox_threshold_hi = simple_strtoul(cal_data_char[0], NULL, 10);
  prox_pulse_cnt = simple_strtoul(cal_data_char[1], NULL, 10);
  prox_gain = simple_strtoul(cal_data_char[2], NULL, 10);
  prox_threshold_lo = simple_strtoul(cal_data_char[3], NULL, 10);
  
  LTR559_DBG("%s: transform to number: high:%d, pluse:%d, gain:%d, low:%d\n", 
    __FUNCTION__, prox_threshold_hi,prox_pulse_cnt, prox_gain, prox_threshold_lo);
  
  data->default_ps_highthresh = prox_threshold_hi;
  data->led_pulse_count = prox_pulse_cnt;//not use
  data->default_ps_lowthresh = prox_threshold_lo;
  
  LTR559_DBG("%s: ---\n", __FUNCTION__); 

  return count;

}


static struct device_attribute ltr559_attributes[] = {
 	__ATTR(als_adc, 0444, als_adc_show, NULL),
 	__ATTR(ps_adc, 0444, ps_adc_show, NULL),
 	__ATTR(psadcsaturationBit, 0444, psadcsaturationBit_show, NULL),
 	__ATTR(ltr559help, 0444, ltr559help_show, NULL),
 	__ATTR(enable_als_sensor, 0664, alsmodesetup_show, alsmodesetup_store),
 	__ATTR(alsswresetsetup, 0664, alsswresetsetup_show, alsswresetsetup_store),
 	__ATTR(alsgainsetup, 0664, alsgainsetup_show, alsgainsetup_store),
 	__ATTR(alscontrsetup, 0664, alscontrsetup_show,	alscontrsetup_store),
 	__ATTR(enable_ps_sensor, 0664, psmodesetup_show, psmodesetup_store),
 	__ATTR(psgainsetup, 0664, psgainsetup_show, psgainsetup_store),
 	__ATTR(pssatuindicasetup, 0664, pssatuindicasetup_show, pssatuindicasetup_store),
 	__ATTR(pscontrsetup, 0664, pscontrsetup_show, pscontrsetup_store),
 	__ATTR(psledcurrsetup, 0664, psledcurrsetup_show, psledcurrsetup_store),
 	__ATTR(psledcurrduty, 0664, psledcurrduty_show, psledcurrduty_store),
 	__ATTR(psledpulsefreqsetup, 0664, psledpulsefreqsetup_show, psledpulsefreqsetup_store),
 	__ATTR(psledsetup, 0664, psledsetup_show, psledsetup_store),
 	__ATTR(psledpulsecountsetup, 0664, psledpulsecountsetup_show, psledpulsecountsetup_store),
 	__ATTR(psmeasratesetup, 0664,	psmeasratesetup_show, psmeasratesetup_store),
 	__ATTR(alsmeasratesetup, 0664,	alsmeasratesetup_show, alsmeasratesetup_store),
 	__ATTR(alsintegtimesetup, 0664,	alsintegtimesetup_show, alsintegtimesetup_store),
 	__ATTR(alsmeasrateregsetup, 0664, alsmeasrateregsetup_show, alsmeasrateregsetup_store),
 	__ATTR(partid, 0444, partid_show, NULL),
 	__ATTR(revid, 0444, revid_show, NULL),
 	__ATTR(partidreg, 0444, partidreg_show, NULL),
 	__ATTR(manuid, 0444, manuid_show, NULL),
 	__ATTR(psdatastatus, 0444, psdatastatus_show, NULL),
 	__ATTR(psinterruptstatus, 0444, psinterruptstatus_show, NULL),
 	__ATTR(alsdatastatus, 0444, alsdatastatus_show, NULL),
 	__ATTR(alsinterruptstatus, 0444, alsinterruptstatus_show, NULL),
 	__ATTR(alsgainstatus, 0444, alsgainstatus_show, NULL),
 	__ATTR(alsdatavaliditystatus, 0444, alsdatavaliditystatus_show, NULL),
 	__ATTR(alspsstatusreg, 0444, alspsstatusreg_show, NULL),
 	__ATTR(alsch0ch1rawcalc, 0444, alsch0ch1rawcalc_show, NULL),
 	__ATTR(setpsoffset, 0664, setpsoffset_show, setpsoffset_store),
 	__ATTR(interruptmodesetup, 0664, interruptmodesetup_show, interruptmodesetup_store),
 	__ATTR(interruptpolarsetup, 0664, interruptpolarsetup_show, interruptpolarsetup_store),
 	__ATTR(interruptsetup, 0664, interruptsetup_show, interruptsetup_store),
 	__ATTR(interruptpersistsetup, 0664, interruptpersistsetup_show, interruptpersistsetup_store),
 	__ATTR(setalslothrerange, 0220, NULL, setalslothrerange_store),
 	__ATTR(setalshithrerange, 0220, NULL, setalshithrerange_store),
 	__ATTR(dispalsthrerange, 0444, dispalsthrerange_show, NULL),
 	__ATTR(setpslothrerange, 0220, NULL, setpslothrerange_store),
 	__ATTR(setpshithrerange, 0220, NULL, setpshithrerange_store),
 	__ATTR(disppsthrerange, 0444, disppsthrerange_show, NULL),
 	__ATTR(ltr559_reg, 0444, ltr559_reg_show, NULL),
 	__ATTR(calibrate, 0664, ltr559_calibrate_show, ltr559_calibrate_store), //xym add
 	__ATTR(set_calibrate_data, 0222, NULL, ltr559_set_calibrate_data_store), //xym add
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	int err;
	for (i = 0; i < ARRAY_SIZE(ltr559_attributes); i++) {
		err = device_create_file(dev, ltr559_attributes + i);
		if (err)
			goto error;
	}
	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, ltr559_attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return err;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(ltr559_attributes); i++)
		device_remove_file(dev, ltr559_attributes + i);
	return 0;
}



