/*
 * Fuel Gauge driver for LC709203F
 *
 * Copyright (C) 2014 ON Semiconductor
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
 
//#define DEBUG
#define pr_fmt(fmt) "[CHG] %s(%d): " fmt, __func__,__LINE__

#include <linux/module.h>
#include <linux/param.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/debugfs.h>
#include "zte_lc709203f_fg_battery_data.h"


//#define DEBUG_PRINT(fmt, args...) printk(fmt, ## args);

//////////////////////////////////////////////////////////////////////////////YM150424//
#define DRIVER_VERSION				"0.2"
//////////////////////////////////////////////////////////////////////////////YM150424//

#define LC709203F_BEFORER_RSOC              0x04   //the high voltage
#define LC709203F_THERMISTOR_B              0x06
#define LC709203F_INITIAL_RSOC              0x07
#define LC709203F_CELL_TEMP                 0x08
#define LC709203F_CELL_VOLTAGE              0x09
#define LC709203F_CURRENT_DIRECTION         0x0A
#define LC709203F_ADJUSTMENT_PACK_APPLI		0x0B
#define LC709203F_ADJUSTMENT_PACK_THERM		0x0C
#define LC709203F_RSOC                      0x0D
#define LC709203F_INDICATOR_TO_EMPTY        0x0F
#define LC709203F_IC_VERSION                0x11
#define LC709203F_CHANGE_OF_THE_PARAM		0x12
#define LC709203F_ALARM_LOW_RSOC            0x13
#define LC709203F_ALARM_LOW_CELL_VOLTAGE    0x14
#define LC709203F_IC_POWER_MODE             0x15
#define LC709203F_STATUS_BIT                0x16
#define LC709203F_NUMBER_OF_THE_PARAM       0x1A

#define LC709203_REG_PW_Mode                0x15
#define LC709203_REG_READ_VENDOR            0x1A
#define LC709203_REG_ENABLE_WRITE_MODE      0x00
#define LC709203_REG_ENTER_WRITE_MODE       0x01
#define LC709203_REG_READ_RESULT            0x07
#define LC709203_REG_RESET                  0x08

#define LC709203_REG_CONFIG_DATA            0x8100
#define DEFAULT_VALUE_ADJUSTMENT_PACK_APPLI 0x0028   //16mohm,zte N9132
#define VALUE_THERMISTOR_B                  0xFD2    //0x0D33  -->4050  NTC-B


#define __USE_ITE_FOR_CAPACITY__

#define DEFAULT_ITE_THRESHOLD_HIGH        986  // by zte,for 4.35V battery
#define DEFAULT_ITE_THRESHOLD_LOW         15   // by zte,for 4.35V battery
#define DEFAULT_BATTERY_FCC               3000
#define WORK_INTERVAL                     8
#define DEFAULT_CURRENT_VALUE             150

#define __USE_ITE_FOR_ESTIMATE_CURRENT__
static unsigned int last_estimate_time = 0;
static int last_estimate_rsoc = 0;
static int last_estimate_ite  = 0;


//#define LC709203F_THERMISTOR_MODE    //get battery temp from LC709203F

struct lc709203f_chip {
	struct i2c_client	*client;
	struct device *dev;

	unsigned long update_time;

	int cell_temp;
	int cell_voltage;
	int batt_status;
	int rsoc;
	int ite;
	int cur;
	int cell_fcc;       //zte
	int current_sec;    //zte,for estimate current by ite.(FCC * 3600 / 100) ([mAsec]/percent )
	int ite_threshold_high;
	int ite_threshold_low;
	int pack_appli_value;

	struct power_supply bat;
	struct workqueue_struct *monitor_wqueue;
	struct delayed_work monitor_work;
	struct power_supply     *bms_psy;
	struct power_supply     *batt_psy;
	int is_first_time_get_batt_psy;
	struct dentry           *debug_root;
	u8                       reg_addr;

};

static unsigned int cache_time = 1000;
module_param(cache_time, uint, 0644);
MODULE_PARM_DESC(cache_time, "cache time in milliseconds");

static int rescaled_rsoc(struct lc709203f_chip *chip, int ite)
{

    return (((ite)-chip->ite_threshold_low)*100)/(chip->ite_threshold_high - chip->ite_threshold_low);

}

static int get_battery_status(struct lc709203f_chip *chip)
{
	union power_supply_propval ret = {0,};
	int rc;

	if (chip->batt_psy == NULL)
		chip->batt_psy = power_supply_get_by_name("battery");
	if (chip->batt_psy) {
		/* if battery has been registered, use the status property */
		rc = chip->batt_psy->get_property(chip->batt_psy,
					POWER_SUPPLY_PROP_STATUS, &ret);
		if (rc) {
			pr_debug("Battery does not export status: %d\n", rc);
			return POWER_SUPPLY_STATUS_UNKNOWN;
		}
		return ret.intval;
	}

	/* Default to false if the battery power supply is not registered. */
	pr_debug("battery power supply is not registered\n");
	return POWER_SUPPLY_STATUS_UNKNOWN;
}

static int lc709203f_read(struct lc709203f_chip *chip, unsigned char reg)
{
	struct i2c_client *client = to_i2c_client(chip->dev);
	struct i2c_msg msg[2];
	unsigned char data[3];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	msg[1].len = 3;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		return ret;

	ret = (data[1] << 8) | data[0];
	pr_debug("reg=0x%02x,return value=%d\n", reg,ret);

	return ret;
}

#define dPOLYNOMIAL8 (0x8380)
static unsigned char u1_CRC_8_u1u1(unsigned char u1ArgBeforeData,
				   unsigned char u1ArgAfterData)
{
	unsigned char  u1TmpLooper  = 0;
	unsigned char  u1TmpOutData = 0;
	unsigned short u2TmpValue   = 0;

	u2TmpValue = (unsigned short)(u1ArgBeforeData ^ u1ArgAfterData);
	u2TmpValue <<= 8;

	for (u1TmpLooper = 0; u1TmpLooper < 8; u1TmpLooper++) {
		if (u2TmpValue & 0x8000)
			u2TmpValue ^= dPOLYNOMIAL8;
		u2TmpValue <<= 1;
	}

	u1TmpOutData = (unsigned char)(u2TmpValue >> 8);

	return u1TmpOutData;
}

static int lc709203f_write(struct lc709203f_chip *chip,
			   unsigned char reg, unsigned short val)
{
	struct i2c_client *client = to_i2c_client(chip->dev);
	struct i2c_msg msg[1];
	unsigned char data[4], crc8 = 0;
	int ret;

	if (!client->adapter)
		return -ENODEV;

	pr_debug("addr = 0x%02x,reg=0x%02x,value=%d\n", client->addr,reg,val);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = data;
	msg[0].len = 4;

	data[0] = reg;
	data[1] = val & 0xff;
	data[2] = val >> 8;

	crc8 = u1_CRC_8_u1u1(crc8, client->addr << 1);
	crc8 = u1_CRC_8_u1u1(crc8, data[0]);
	crc8 = u1_CRC_8_u1u1(crc8, data[1]);
	crc8 = u1_CRC_8_u1u1(crc8, data[2]);

	data[3] = crc8;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		pr_err("i2c write error (%d)\n", ret);
		return ret;
	}

	return 0;
}


#define DEFAULT_TEMP		250
static int lc709203f_get_batt_temperature(struct lc709203f_chip *chip)
{

#ifdef LC709203F_THERMISTOR_MODE
    /*get battery temp from lc709203f */
	int ret;

	ret = lc709203f_read(chip,LC709203F_CELL_TEMP);
	if(ret<0){
		dev_err(chip->dev,"Failed to read cell temp:%d\n", ret);
		return DEFAULT_TEMP;
	}

	ret = ret - 2732;	//in Celsius degree 
	return ret;
	
#else
	/*get batt temp from power sypply batt_psy property*/
	union power_supply_propval ret = {0,};

	if (chip->batt_psy) {
		chip->batt_psy->get_property(chip->batt_psy,
					POWER_SUPPLY_PROP_TEMP, &ret);
		return ret.intval; //return 0.1 Deg
	}
	pr_debug("battery power supply is not registered,return DEFAULT_TEMP.\n");
	return DEFAULT_TEMP;

#endif
}


static int lc709203f_read_ite(struct lc709203f_chip *chip)
{

    int ret;
	ret = lc709203f_read(chip,LC709203F_INDICATOR_TO_EMPTY);
    if(ret < 0){
         dev_err(chip->dev,"Failed to read ITE register:%d\n", ret);
         ret = chip->ite;
    }
    pr_debug("%s, ite =%d \n", __func__,ret);
    return ret;
}


/*read battery voltage in mV. by zte 20150625*/
static int lc709203f_read_voltage(struct lc709203f_chip *chip)
{

    int ret;
	static int voltage_error_count=0;
    ret = lc709203f_read(chip,LC709203F_CELL_VOLTAGE);
    if(ret < 0){
        dev_err(chip->dev,"Failed to read Voltage register:%d\n", ret);
        ret = chip->cell_voltage;
    }else if((ret > 4600)||(ret < 2000))
	{
		voltage_error_count++;
		dev_err(chip->dev,"read voltage error register:%d,voltage_error_count:%d\n", ret,voltage_error_count);

		ret = chip->cell_voltage;
	}
    pr_debug("%s, voltage =%d \n", __func__,ret);
    return ret;
}

/*read rsoc. by zte 20150625*/
static int lc709203f_read_rsoc(struct lc709203f_chip *chip)
{
    int ret;
	int rsoc;

#ifdef __USE_ITE_FOR_CAPACITY__
    ret = lc709203f_read(chip,LC709203F_INDICATOR_TO_EMPTY);
    if(ret < 0){
        dev_err(chip->dev,"Failed to read ITE register:%d\n", ret);
        ret = chip->ite;
    }
	pr_debug("Before scaled, ite = %d\n", ret);
	if (ret >= chip->ite_threshold_high)
		rsoc = 100;
	else if (ret <= chip->ite_threshold_low)
		rsoc = 0;
	else
		rsoc = rescaled_rsoc(chip,ret);

	pr_debug("After scaled, rsoc=%d\n", rsoc);
#else
    rsoc = lc709203f_read(chip,LC709203F_RSOC);
    if(rsoc < 0){
        dev_err(chip->dev,"Failed to read RSOC register:%d\n", rsoc);
        rsoc = chip->rsoc;
    }
    pr_debug("LC709203F rsoc=%d\n", rsoc);
#endif
    pr_debug("%s, rsoc =%d \n", __func__,rsoc);
    return rsoc;
}

/*return estimate current in mA. by zte 20150625*/
static int lc709203f_estimate_current(struct lc709203f_chip *chip)
{
	static int cur;
	long ms=0;
	int val;

    /* Estimate Current */
    if (chip->update_time == 0) {
        pr_debug("first time Estimate Current,return default current.\n");
        last_estimate_time = jiffies_to_msecs(jiffies);
        last_estimate_rsoc = chip->rsoc;
        last_estimate_ite  = chip->ite;
        cur = DEFAULT_CURRENT_VALUE;
    } else {

#ifdef __USE_ITE_FOR_ESTIMATE_CURRENT__
	    val = chip->ite - last_estimate_ite;
#else
	    val = (chip->rsoc - last_estimate_rsoc) * 10;
#endif

	    /* 1% charging */
	    if (abs(val) >= 1) {

		    ms = jiffies_to_msecs(jiffies) - last_estimate_time;

		/**********************************************
		 * NOTE(by zte JZN):
		 * (1) I[mA] = A[mAsec] * 1000 / t[msec]
		 *           = CURRENT_SEC * 1000 * val/(10*ms)
		 *
		 * (2) I>0:discharging; I<0:charging
		 **********************************************/
		    if( ms > 0 ) {

				cur = -(chip->current_sec * 100 * val)/ms;
		    }
		    else{
				pr_err("%s, error:estimate current, delta_t =%ldms \n", __func__,ms);
				cur = DEFAULT_CURRENT_VALUE;
		    }
		    last_estimate_time = jiffies_to_msecs(jiffies);
		    last_estimate_rsoc = chip->rsoc;
		    last_estimate_ite  = chip->ite;
	    }
	}

	chip->update_time = jiffies;

	pr_debug("estimate current=%d,delta_t=%ldms,val=%d, last_estimate_time=%u\n",
					  cur, ms, val, last_estimate_time);

    return cur;
}

static int lc709203f_read_status(struct lc709203f_chip *chip)
{
	int batt_status;

	if (chip->update_time && time_before(jiffies, chip->update_time +
					     msecs_to_jiffies(cache_time))) {
		pr_debug("cache_time less than 1000ms\n");
		return 0;
	}

	if (chip->update_time == 0) {
		pr_info("first time\n");

		lc709203f_write(chip, LC709203F_IC_POWER_MODE, 0x0001);
		mdelay(1);
		lc709203f_write(chip, LC709203F_IC_POWER_MODE, 0x0001);
		lc709203f_write(chip, LC709203F_IC_POWER_MODE, 0x0001);

#ifdef LC709203F_THERMISTOR_MODE
		lc709203f_write(chip, LC709203F_THERMISTOR_B,VALUE_THERMISTOR_B);
		lc709203f_write(chip, LC709203F_STATUS_BIT, 0x0001);
#endif
		chip->ite          = lc709203f_read_ite(chip);
		chip->cell_voltage = lc709203f_read_voltage(chip);
		chip->rsoc         = lc709203f_read_rsoc(chip);
		chip->cur          = 0;//first current set to 0 

		pr_info("first time get cell_voltage=%d,rsoc=%d,ite=%d\n",
						chip->cell_voltage,chip->rsoc,chip->ite );

		lc709203f_write(chip, LC709203F_ADJUSTMENT_PACK_APPLI,
				chip->pack_appli_value);
	}

	chip->cell_temp = lc709203f_get_batt_temperature(chip);

#ifndef LC709203F_THERMISTOR_MODE
    /*if not in thermistor mode,need to write tempetature back to LC709203F*/
    lc709203f_write(chip, LC709203F_CELL_TEMP, chip->cell_temp+2732);
#endif

	batt_status = get_battery_status(chip);

	if(chip->batt_psy != NULL && chip->is_first_time_get_batt_psy){

		chip->is_first_time_get_batt_psy = 0;
		pr_info("first time get batt psy, cell_voltage=%d,rsoc=%d,batt_status=%s\n",
						chip->cell_voltage,chip->rsoc,batt_status==1?"charging":"not charging");

		if(batt_status != POWER_SUPPLY_STATUS_CHARGING){
			/* if rsoc vs. cell voltage is abnormal,
			*  re-measure soc use the HIGHEST VOLTAGE till this command is set
			*/
			if( (chip->rsoc < 4 && chip->cell_voltage >3700) ||
					(chip->rsoc < 75 && chip->cell_voltage >4200) ||
					(chip->rsoc < 14 && chip->cell_voltage >3800)){
				#if 0
				//case 1:use voltage to measure soc in 1.5ms later when this command is set
				lc709203f_write(chip, LC709203F_INITIAL_RSOC, 0xAA55);
				#else
				//case 2:use the HIGHEST VOLTAGE till this command is set to measure soc
				lc709203f_write(chip, LC709203F_BEFORER_RSOC, 0xAA55);
				pr_info("CHG:HIGHEST voltage as ocv\n");
				#endif
			}
		}
		if(batt_status == POWER_SUPPLY_STATUS_CHARGING){
			/* if rsoc vs. cell voltage is abnormal,
			*  when inserting charger before battery is installed
			*/
			if( (chip->rsoc < 93 &&chip->rsoc >87 && chip->cell_voltage < 4000) ){
				#if 1
				//case 1:use voltage to measure soc in 1.5ms later when this command is set
				lc709203f_write(chip, LC709203F_INITIAL_RSOC, 0xAA55);
				pr_info("CHG:reset vmbms in 1.5ms\n");
				#else
				//case 2:use the HIGHEST VOLTAGE till this command is set to measure soc
				lc709203f_write(chip, LC709203F_BEFORER_RSOC, 0xAA55);
				pr_info("CHG:HIGHEST voltage as ocv\n");
				#endif
			}
		}
	}

	if(batt_status != chip->batt_status){
		switch(batt_status){
		case POWER_SUPPLY_STATUS_CHARGING:
			lc709203f_write(chip, LC709203F_CURRENT_DIRECTION, 0x0000);   //auto mode
			break;
		case POWER_SUPPLY_STATUS_FULL:
			/*Note 20160325:
			 * for the situation:charger ic reported chg_done,but rsoc is not 100 yet
			 * so write registor RSOC=99 (registor ITE will be 994 and chip->rsoc will be 100)*/
			if( (chip->rsoc == 99) || (chip->rsoc ==98)){
				lc709203f_write(chip, LC709203F_RSOC, 99);
				pr_info("CHG:sync rsoc with charger ic.before sync ite=%d\n",chip->ite);
			}
			lc709203f_write(chip, LC709203F_CURRENT_DIRECTION, 0x0001);   //charging
			break;
		case POWER_SUPPLY_STATUS_DISCHARGING:
			lc709203f_write(chip, LC709203F_CURRENT_DIRECTION, 0xFFFF);   //discharge
			break;
		default:
			lc709203f_write(chip, LC709203F_CURRENT_DIRECTION, 0x0000);   //auto mode
			break;
		}
		chip->batt_status = batt_status;
	}

    chip->ite          = lc709203f_read_ite(chip);
	chip->cell_voltage = lc709203f_read_voltage(chip);
	chip->rsoc         = lc709203f_read_rsoc(chip);
	chip->cur          = lc709203f_estimate_current(chip);

    pr_debug("update_time = %ld,cell_voltage=%d,rsoc=%d,ite=%d,cell_temp=%d,cur=%d,batt_stat=%d\n", 
			   chip->update_time,chip->cell_voltage,chip->rsoc,chip->ite,chip->cell_temp,chip->cur,chip->batt_status );

	return 0;
}

static void lc709203f_work(struct work_struct *work)
{
	struct lc709203f_chip *chip = container_of(work,
				struct lc709203f_chip, monitor_work.work);

	const int interval = HZ * WORK_INTERVAL;

	pr_debug(" run\n");
	lc709203f_read_status(chip);
	queue_delayed_work(chip->monitor_wqueue,&chip->monitor_work,interval);
}

static int lc709203f_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct lc709203f_chip *chip = container_of(psy,
						struct lc709203f_chip, bat);

	lc709203f_read_status(chip);

	switch (psp) {

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = chip->cell_voltage * 1000;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = chip->rsoc;
		break;

	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = chip->ite;    //return ite as capacity level,by zte JZN
		break;

	case POWER_SUPPLY_PROP_TEMP:
		//val->intval = chip->cell_temp - 2732;
		val->intval = chip->cell_temp ;
		break;

	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = chip->cur * 1000;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = chip->cur * 1000;
		break;

    /*by zte JZN,20150629*/
	case POWER_SUPPLY_PROP_CHARGE_FULL:
        val->intval = chip->cell_fcc * 1000;
        break;
    case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
        val->intval = chip->cell_fcc * 1000;
        break;

	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property lc709203f_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_FULL,
};

#if defined(CONFIG_BOARD_SHEEN) ||defined(CONFIG_BOARD_XRAY45)||defined(CONFIG_BOARD_XRAY50)||defined(CONFIG_BOARD_BENZ)||defined(CONFIG_BOARD_CAPTAIN)

/***********************************************************
*	3 Byte Write Protcol
***********************************************************/
static int lc709203_data_write(u8 reg, unsigned short subcmd,u8 *buf, int len,
                   struct lc709203f_chip *chip)
{
	struct i2c_msg msg;
	unsigned char data[len+4];
	int ret;
	/*var for crc */
	static unsigned char u1Debug = 0;
	int i;

	if (!chip->client)
		return -ENODEV;

	memset(data, 0, sizeof(data));
	data[0] = reg;
	data[1] = subcmd & 0x00FF;
	data[2] = (subcmd & 0xFF00) >> 8;
	memcpy(&data[3], buf , len);

	/* crc Write Word Protocol*/
	u1Debug = u1_CRC_8_u1u1( 0x00 , 0x16 );        // Address

	for(i=0;i<len+3;i++)
	u1Debug = u1_CRC_8_u1u1( u1Debug , data[i] );  // Command

	data[len+3]=u1Debug;

	msg.addr = chip->client->addr;
	msg.flags = 0;
	msg.len = len+4;
	msg.buf = data;

	ret = i2c_transfer(chip->client->adapter, &msg, 1);
	if (ret < 0) {
		pr_err(" write transfer fail!\n");
		return -EIO;
	}
	pr_info("addr = 0x%02x,reg=0x%02x,data[0]:0x%02x,[1]:0x%02x,[2]:0x%02x,[3]:0x%02x\n",
			chip->client->addr,reg,data[0],data[1],data[2],data[3]);

    return ret;
}

static int lc709203_data_transfer(struct lc709203f_chip *chip)
{
	int ret;
	int i;

	for(i=0;i<0x0D;i++){
		ret = lc709203_data_write(0x02,LC709203_REG_CONFIG_DATA+i*0x80,&data_transfer[i*0x80],0x80,chip);
		if(ret<0){
			pr_err( "transfer error\n");
			return ret;
		}
		msleep(10);
		lc709203f_write(chip,0x03,0x55AA);
		msleep(10);
		lc709203f_write(chip,0x04,0x00A0);
		msleep(10);
		lc709203f_write(chip,0x05,0x55AA);
		msleep(100);
	}

	return ret;
}

static int lc709203_verify(struct lc709203f_chip *chip)
{
	static u8 verify[2]={0x00,0x87};
	int ret;

	ret = lc709203_data_write(0x06,LC709203_REG_CONFIG_DATA,verify,0x02,chip);
	if(ret<0){
		pr_err( "verify error\n");
		return ret;
	}

	return ret;
}
#endif

/* Note by zte JZN 20151208:
*  if use 4.4 V battery,need to update battery data.
*/
#define VENDOR_4V35    0X0301
#define VENDOR_4V4     0XF901


static int lc709203f_check_need_update_batt_data(struct lc709203f_chip *chip)
{
#if defined(CONFIG_BOARD_SHEEN ) ||defined(CONFIG_BOARD_XRAY45) ||defined(CONFIG_BOARD_XRAY50)||defined(CONFIG_BOARD_BENZ)||defined(CONFIG_BOARD_CAPTAIN)// for P890A07/N9131/N9133 (use 4.4V battery)
	int vendor;
	int result;

	vendor=lc709203f_read(chip, LC709203_REG_READ_VENDOR);
	pr_err("vendor = 0x%02x \n", vendor);

    // Battery Code Check
	if ( (vendor == VENDOR_4V35) || (vendor != VENDOR_4V4)){
		pr_err( "data transfer start\n");

		/*change power mode to Continuous mode*/
		lc709203f_write(chip,LC709203_REG_PW_Mode,0x0000);
		msleep(10);
		lc709203f_write(chip,LC709203_REG_PW_Mode,0x0000);
		msleep(10);

		/*enter write mode*/
		msleep(10);
		lc709203f_write(chip, LC709203_REG_ENABLE_WRITE_MODE,0x55AA);
		msleep(10);
		lc709203f_write(chip, LC709203_REG_ENTER_WRITE_MODE,0x55AA);
		msleep(100); //100ms is needed to change executing routine.

		/*data transfer #1: send 128 bytes data, with start address of write.*/
		lc709203_data_transfer(chip);
		msleep(100);

		/*start verify:sends notification of start of verify*/
		lc709203_verify(chip);
		msleep(100);

		/*data transfer #2: send 128 bytes data, with start address of verify.*/
		lc709203_data_transfer(chip);
		msleep(100);

		/*read result£¬return: 1- succeed; 0-failed */
		result=lc709203f_read(chip, LC709203_REG_READ_RESULT);
		pr_err("verify result =0x%02x \n", result);
		if(result != 1){
			pr_err("verify result failed, result=0x%02x \n", result);
			return -1;
		}

		msleep(10);

		/*reset the chip */
		lc709203f_write(chip,LC709203_REG_RESET,0x55AA); 
		msleep(100);

		vendor=lc709203f_read(chip,LC709203_REG_READ_VENDOR);
		pr_err("transfer end. vendor = 0x%02x \n", vendor);
		msleep(1500);  //msleep 1500 ms for reset

		/*change to Accurate ECO Mode*/
		lc709203f_write(chip,LC709203_REG_PW_Mode,0x0001); 
		msleep(10);
		lc709203f_write(chip,LC709203_REG_PW_Mode,0x0001);
		msleep(10);
	}else
		pr_info("battery data is for 4.4V already.\n");

	return 0;

#endif

    return 0;
}

#define LAST_CNFG_REG	0x1A

static int show_cnfg_regs(struct seq_file *m, void *data)
{
	struct lc709203f_chip *chip = m->private;
	int ret;
	u8  addr;

	for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
		ret = lc709203f_read(chip, addr);
		if (!ret)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, ret);
	}

	return 0;
}

static int cnfg_debugfs_open(struct inode *inode, struct file *file)
{
	struct lc709203f_chip *chip = inode->i_private;

	return single_open(file, show_cnfg_regs, chip);
}

static const struct file_operations cnfg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cnfg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int lc709203f_debug_data_set(void *data, u64 val)
{
	struct lc709203f_chip *chip = data;

	pr_info("reg=%d val=%llu\n",chip->reg_addr,val);
	lc709203f_write(chip, chip->reg_addr,val);
	return 0;
}

static int lc709203f_debug_data_get(void *data, u64 *val)
{
	struct lc709203f_chip *chip = data;
	int temp = 0;

	temp = lc709203f_read(chip, chip->reg_addr);
	*val =(u64) temp;
	pr_info("reg=%d val=%llu\n",chip->reg_addr,*val);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(lc709203f_debug_data_fops, lc709203f_debug_data_get,
			lc709203f_debug_data_set, "%llu\n");


static int lc709203f_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	int rc;
	int retval = 0;
	struct lc709203f_chip *chip;
	struct device_node *node;

	pr_info("enter\n");
	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto alloc_failed;
	}

	if (chip->batt_psy== NULL)
		chip->batt_psy = power_supply_get_by_name("battery");
	if (!chip->batt_psy) {
		dev_err(&client->dev, "battery supply not found; defer probe\n");
		//return -EPROBE_DEFER;
	}
	chip->is_first_time_get_batt_psy = 1;

	chip->client = client;
	chip->dev = &client->dev;
	//chip->bat.name = "lc709203f";
	chip->bat.name = "on_bms";
	chip->bat.type = POWER_SUPPLY_TYPE_BMS;
	chip->bat.properties = lc709203f_battery_props;
	chip->bat.num_properties = ARRAY_SIZE(lc709203f_battery_props);
	chip->bat.get_property = lc709203f_get_property;

	pr_info("lc709203f_check_need_update_batt_data\n");
	rc = lc709203f_check_need_update_batt_data(chip);
	if (rc < 0){
		pr_info("fail to update lc709203f batt_data,rc=%d\n",rc);
	}

	/*read dt parameters,begin*/
	node = chip->dev->of_node;
	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		//return -EINVAL;
	}
	rc = of_property_read_u32(node, "zte,bat_full_charge_capacity",
						&chip->cell_fcc);
	if (rc < 0){
		chip->cell_fcc = DEFAULT_BATTERY_FCC;
		pr_info("fail to read batt fcc from dts,return default fcc=%d\n",chip->cell_fcc);
	}

	rc = of_property_read_u32(node, "zte,lc709203f_ite_threshold_high",
						&chip->ite_threshold_high);
	if (rc < 0){
		chip->ite_threshold_high= DEFAULT_ITE_THRESHOLD_HIGH;
		pr_info("fail to read ite_threshold_high,use default=%d\n",chip->ite_threshold_high);
	}

	rc = of_property_read_u32(node, "zte,lc709203f_ite_threshold_low",
						&chip->ite_threshold_low);
	if (rc < 0){
		chip->ite_threshold_low= DEFAULT_ITE_THRESHOLD_LOW;
		pr_info("fail to read ite_threshold_low,use default=%d\n",chip->ite_threshold_low);
	}

	rc = of_property_read_u32(node, "zte,lc709203f_pack_appli_value",
						&chip->pack_appli_value);
	if (rc < 0){
		chip->pack_appli_value= DEFAULT_VALUE_ADJUSTMENT_PACK_APPLI;
		pr_info("fail to read batt fcc from dts,return default fcc=%d\n",chip->pack_appli_value);
	}
	/*read dt parameters,end*/

	chip->current_sec = (chip->cell_fcc * 36); // (FCC * 3600 / 100) ([mAsec]/percent )

	retval = power_supply_register(chip->dev, &chip->bat);
	if (retval) {
		dev_err(chip->dev, "failed to register battery: %d\n", retval);
		goto batt_failed;
	}

	pr_info("[CHG]batt_fcc=%d,ite_high=%d,ite_low=%d,apa=%d\n",
			chip->cell_fcc,chip->ite_threshold_high,chip->ite_threshold_low,chip->pack_appli_value);

	chip->debug_root = debugfs_create_dir("lc709203", NULL);
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

			ent = debugfs_create_u8("address", S_IRUSR | S_IWUSR,
						chip->debug_root,
						&chip->reg_addr);
			if (!ent) {
				dev_err(chip->dev,
					"Couldn't create address debug file\n");
			}

			ent = debugfs_create_file("data",  S_IRUSR | S_IWUSR,
						chip->debug_root, chip,
						&lc709203f_debug_data_fops);
			if (!ent) {
				dev_err(chip->dev,
					"Couldn't create data debug file\n");
			}

		}

	//set defaut value of battery status
	chip->batt_status = -1;

	INIT_DELAYED_WORK(&chip->monitor_work, lc709203f_work);
	chip->monitor_wqueue = create_singlethread_workqueue(chip->bat.name);
	if (!chip->monitor_wqueue) {
		retval = -ESRCH;
		goto workqueue_failed;
	}
	queue_delayed_work(chip->monitor_wqueue, &chip->monitor_work, HZ * 1);

	i2c_set_clientdata(client, chip);

	goto success;

 workqueue_failed:
	power_supply_unregister(&chip->bat);
 batt_failed:
	kfree(chip);
 alloc_failed:
 success:
	return 0;
}

static int lc709203f_remove(struct i2c_client *client)
{
	struct lc709203f_chip *chip = i2c_get_clientdata(client);

	pr_info("%s()\n", __func__);

/*	cancel_rearming_delayed_workqueue(chip->monitor_wqueue,
					  &chip->monitor_work);*/
	cancel_delayed_work_sync(&chip->monitor_work);

	destroy_workqueue(chip->monitor_wqueue);
	power_supply_unregister(&chip->bat);

	kfree(chip);

	return 0;
}

static const struct i2c_device_id lc709203f_id[] = {
	{ "lc709203f", 0 },
	{ },
};

static struct of_device_id lc709203f_match_table[] = {
	{ .compatible = "zte,lc709203f_fg",},
	{ },
};


MODULE_DEVICE_TABLE(i2c, lc709203f_id);

static struct i2c_driver lc709203f_i2c_driver = {
	.driver = {
		.name = "lc709203f",
		.owner		= THIS_MODULE,
		.of_match_table	= lc709203f_match_table,
	},
	.probe = lc709203f_probe,
	.remove = lc709203f_remove,
	.id_table = lc709203f_id,
};

#if 0

module_i2c_driver(lc709203f_i2c_driver);

#else

static int __init lc709203f_init(void)
{
	int ret;

	pr_info("%s()\n", __func__);

	ret = i2c_add_driver(&lc709203f_i2c_driver);

	if (ret)
		pr_err(KERN_ERR "Unable to register LC709203F i2c driver\n");

	return ret;
}

static void __exit lc709203f_exit(void)
{
	pr_info("lc709203f_exit\n");

	i2c_del_driver(&lc709203f_i2c_driver);
}

module_init(lc709203f_init);
//late_initcall(lc709203f_init);
module_exit(lc709203f_exit);

#endif
//MODULE_ALIAS("i2c:LC709203F-bms");
MODULE_DESCRIPTION("LC709203F Fuel Gauge driver");
MODULE_LICENSE("GPL");
