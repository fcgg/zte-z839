/* drivers/i2c/chips/rpr521_driver.c - ROHM RPR521 Linux kernel driver
 *
 * Copyright (C) 2012 
 * Written by Andy Mi <andy-mi@rohm.com.cn>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
/*
 *  This is Linux kernel modules for ambient light + proximity sensor
 *  Revision History
 *  2012-7-19:	Ver. 1.0	New release together with a porting guide.
 *  2012-8-14:	Ver. 1.1	Added calibration and set thresholds methods. Besides, the thresholds are automatically changed if a ps int is triggered to avoid constant interrupts.
 *  2014-1-09:	Ver. 1.2	Modified some functions for rpr521
 */
#include <linux/debugfs.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/sensors.h>

//#include <linux/input/rpr0521.h>
#include "rpr0521.h"

/*************** Global Data ******************/
/* parameter for als calculation */
#define COEFFICIENT               (4)
#if defined(CONFIG_BOARD_MAX)//N9521
const unsigned long data0_coefficient[COEFFICIENT] = {6960, 935, 666, 405};
const unsigned long data1_coefficient[COEFFICIENT] = {0, 520,  278,  144};
const unsigned long judge_coefficient[COEFFICIENT] = { 584,  1107,  1955, 2810}; 
#else
const unsigned long data0_coefficient[COEFFICIENT] = {6960, 935, 666, 405};
const unsigned long data1_coefficient[COEFFICIENT] = {0, 520,  278,  144};
const unsigned long judge_coefficient[COEFFICIENT] = { 584,  1107,  1955, 2810}; 
#endif
#define _AUTO_THRESHOLD_CHANGE_
//#define _INIT_CALIB_  //zero modify in 20140505
u8 init_calib_flag = 0;   //grace modify in 2014.12.18
u8 init_ps_high = 0;  //grace modify in 2014.12.18
u8 init_ps_low = 0;  //grace modify in 2014.12.18
u8 calib_status = 0; //grace modify in 2014.12.18

/* mode control table */
#define MODE_CTL_FACTOR (16)
static const struct MCTL_TABLE {
    short ALS; //als measure time
    short PS; //ps measure time
} mode_table[MODE_CTL_FACTOR] = {
    {  0,   0},   /*  0 */
    {  0,  10},   /*  1 */
    {  0,  40},   /*  2 */
    {  0, 100},   /*  3 */
    {  0, 400},   /*  4 */
    {100,  50},   /*  5 */
    {100, 100},   /*  6 */
    {100, 400},   /*  7 */
    {400,   0},   /*  8 */
    {400, 100},   /*  9 */
    {400,   0},   /* 10 */
    {400, 400},   /* 11 */
    {  50,  50},   /* 12 */
    {  0,   0},   /* 13 */
    {  0,   0},   /* 14 */
    {  0,   0}    /* 15 */
};

/* gain table */
#define GAIN_FACTOR (16)
static const struct GAIN_TABLE {
    /*char DATA0;
    char DATA1;*/
    unsigned char DATA0; //grace modify in 2014.5.7
    unsigned char DATA1; //grace modify in 2014.5.7
} gain_table[GAIN_FACTOR] = {
    {  1,   1},   /*  0 */
    {  0,   0},   /*  1 */
    {  0,   0},   /*  2 */
    {  0,   0},   /*  3 */
    {  2,   1},   /*  4 */
    {  2,   2},   /*  5 */
    {  0,   0},   /*  6 */
    {  0,   0},   /*  7 */
    {  0,   0},   /*  8 */
    {  0,   0},   /*  9 */
    { 64,  64},   /* 10 */
    {  0,   0},   /* 11 */
    {  0,   0},   /* 12 */
    {  0,   0},   /* 13 */
    {128,  64},   /* 14 */
    {128, 128}    /* 15 */
};




#ifdef ZTE_RPR521_SENSORS_CLASS_DEV //xym add 
static struct sensors_classdev sensors_light_cdev = {
	.name = "rpr0521-light",
	.vendor = "rohm",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "30000",
	.resolution = "0.0125",
	.sensor_power = "0.20",
	.min_delay = 1000, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};

static struct sensors_classdev sensors_proximity_cdev = {
	.name = "rpr0521-proximity",
	.vendor = "rohm",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5",
	.resolution = "5.0",
	.sensor_power = "3",
	.min_delay = 1000, /* in microseconds */
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 100,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
#endif

static int rpr521_is_boot=0;//xym

/*************** Functions ******************/
/******************************************************************************
 * NAME       : rpr521_set_measurement_time
 * FUNCTION   : set measurement time according to enable
 * REMARKS    : this function will overwrite the work mode. if it is called improperly, 
 *			   you may shutdown some part unexpectedly. please check als_ps->enable first.
 *			   I assume it is run in normal mode. If you want low noise mode, the code should be modified.
 *****************************************************************************/
static int rpr521_set_measurement_time(struct i2c_client *client, int enable)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;
	//printk("%s: \n", __func__);

	if(enable > 0xFb)
	{
		printk(KERN_ERR "%s: invalid measurement time setting.\n", __func__);
		return -EINVAL;
	}
	else
	{
		mutex_lock(&als_ps->update_lock);
		ret = i2c_smbus_write_byte_data(client, REG_MODECONTROL, enable);
		mutex_unlock(&als_ps->update_lock);
       printk("%s: write 0x%X to REG_MODECONTROL\n", __func__, enable);
			 
		als_ps->enable = enable;
		als_ps->als_time = mode_table[(enable & 0xF)].ALS;
		als_ps->ps_time = mode_table[(enable & 0xF)].PS;
       printk("%s: als_time is %d, ps_time is %d\n", __func__, als_ps->als_time, als_ps->ps_time);
		return ret;
	}
}

static int rpr521_set_ps_threshold_low(struct i2c_client *client, int threshold)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;
	unsigned short write_data;
	printk("%s: %d (0x%X)\n", __func__, threshold, threshold);

    /* check whether the parameter is valid */
	if(threshold > REG_PSTL_MAX)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}
	/*if(threshold > als_ps->ps_th_h)
	{
		printk(KERN_ERR "%s: higher than threshold high.\n", __func__);
		return -EINVAL;
	}*/ //grace modify in 2014.5.6
	
    /* write register to rpr521 via i2c */
	write_data = CONVERT_TO_BE(threshold);
	mutex_lock(&als_ps->update_lock);
	ret = i2c_smbus_write_i2c_block_data(client, REG_PSTL, sizeof(write_data), (unsigned char *)&write_data);
	mutex_unlock(&als_ps->update_lock);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c fail.\n", __func__);
		return ret;
	}
	als_ps->ps_th_l = threshold;	//Update the value after successful i2c write to avoid difference. 
		
	return 0;
}

static int rpr521_set_ps_threshold_high(struct i2c_client *client, int threshold)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;
	unsigned short write_data;
	printk("%s: %d (0x%X)\n", __func__, threshold, threshold);

    /* check whether the parameter is valid */
	if(threshold > REG_PSTH_MAX)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}
	/*if(threshold < als_ps->ps_th_l)
	{
		printk(KERN_ERR "%s: lower than threshold low.\n", __func__);
		return -EINVAL;
	}*///grace modify in 2014.5.6
	
    /* write register to rpr521 via i2c */
	write_data = CONVERT_TO_BE(threshold);
	mutex_lock(&als_ps->update_lock);
	ret = i2c_smbus_write_i2c_block_data(client, REG_PSTH, sizeof(write_data), (unsigned char *)&write_data);
	mutex_unlock(&als_ps->update_lock);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c fail.\n", __func__);
		return ret;
	}
	als_ps->ps_th_h = threshold;	//Update the value after successful i2c write to avoid difference. 
		
	return 0;
}

static int rpr521_calibrate(struct i2c_client *client)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int average;
 	unsigned int i, tmp, ps_th_h, ps_th_l;	
 	u8 infrared_data; //grace modify in 2014.12.18
	printk("%s: \n",__func__);

	average = 0;
	
	//grace modify in 2014.12.18 begin
	calib_status = 0;
	tmp = i2c_smbus_write_byte_data(client, REG_INTERRUPT, PS_ALS_SET_INTR&0XFE); //disable ps interrupt
	if(tmp < 0)	
	{		
		goto err_exit;	
	}	
  //grace modify in 2014.12.18 end
  
	//rpr521_set_measurement_time(client, 0x41);	//PS 10ms
	rpr521_set_measurement_time(client, PS_EN|PS_DOUBLE_PULSE|PS10MS);        //PS 10ms   0x61

  //grace modify in 2014.12.18 begin	
	 mdelay(20); 
	 tmp = i2c_smbus_read_byte_data(client, REG_PERSISTENCE);
	 if(tmp < 0)	
	 {		
		goto err_exit;	
	 }	
	 infrared_data = tmp;	

	 if(infrared_data>>6)  //说明红外数据高
	 {		
		goto err_exit;
	 }
	 //grace modify in 2014.12.18 end
	 
	for(i = 0; i < 10; i ++)
	{
		mdelay(20);
		tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
		if(tmp < 0)
		{
			printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
			//goto err_exit;
		}
		average += tmp & 0xFFF;	// 12 bit data
	}
	average /= 10;

//	ps_th_h = average + PS_ALS_SET_PS_TH;
//	ps_th_l = average + PS_ALS_SET_PS_TL;
	ps_th_h = average + THRES_TOLERANCE + THRES_DEFAULT_DIFF;
	ps_th_l = average + THRES_TOLERANCE;

	if(ps_th_h < 0)
	{
		printk(KERN_ERR "%s: high threshold is less than 0.\n", __func__);
		goto err_exit;
	}
	if(ps_th_h > REG_PSTH_MAX)
	{
		printk(KERN_ERR "%s: high threshold is greater than maximum allowed value.\n", __func__);
		goto err_exit;
	}
	if(ps_th_l < 0)
	{
		printk(KERN_ERR "%s: low threshold is less than 0.\n", __func__);
		goto err_exit;
	}
	if(ps_th_l > REG_PSTL_MAX)
	{
		printk(KERN_ERR "%s: low threshold is greater than maximum allowed value.\n", __func__);
		goto err_exit;
	}
	
	//grace modify in 2014.12.18 begin
	if (average > PS_CROSSTALK)
	{
		if(init_calib_flag)
		{
			ps_th_h = init_ps_high;
			ps_th_l = init_ps_low;
		}
		else
		{
			ps_th_h = PS_ALS_SET_PS_TH;
			ps_th_l = PS_ALS_SET_PS_TL;
		}		
	}
	else
	{
		calib_status = 1;
	}
	//grace modify in 2014.12.18 end

	if(!(rpr521_set_ps_threshold_high(client, ps_th_h)))
		als_ps->ps_th_h_back = ps_th_h;
	else 
		goto err_exit;
	if(!(rpr521_set_ps_threshold_low(client, ps_th_l)))
		als_ps->ps_th_l_back = ps_th_l;
	else
		goto err_exit;

	printk("%s: als_ps->ps_th_h=%d (0x%X) \n", __func__, als_ps->ps_th_h, als_ps->ps_th_h);
	printk("%s: als_ps->ps_th_l=%d (0x%X) \n", __func__, als_ps->ps_th_l, als_ps->ps_th_l);

	//rpr521_set_measurement_time(client, 0);	//disable ps
   rpr521_set_measurement_time(client, PS_ALS_SET_MODE_CONTROL);//grace modify in 2014.5.6 
   //grace modify in 2014.12.18 begin
   tmp = i2c_smbus_write_byte_data(client, REG_INTERRUPT, PS_ALS_SET_INTR|0x1); //enable ps interrupt
	if(tmp < 0)	
	{		
		goto err_exit;	
	}
	//grace modify in 2014.12.18 end
	return 0;		

err_exit:
	//rpr521_set_measurement_time(client, 0);	//disable ps
	rpr521_set_measurement_time(client, PS_ALS_SET_MODE_CONTROL);//grace modify in 2014.5.6 
	//grace modify in 2014.12.18 begin
  tmp = i2c_smbus_write_byte_data(client, REG_INTERRUPT, PS_ALS_SET_INTR|0x1); //enable ps interrupt
	if(tmp < 0)	
	{		
		goto err_exit;	
	}
	//grace modify in 2014.12.18 end

	return -1;
	
}

#if _FUNCTION_USED_	//masked because they are claimed but not used, which may cause error when compilling if the warning level is high enough. These functions provides some methods.
static int rpr521_set_persist(struct i2c_client *client, int persist)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;
	printk("%s: \n",__func__);

    /* check whether the parameter is valid */
	if(persist > PERSISTENCE_MAX)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}
	
    /* write register to rpr521 via i2c */
	mutex_lock(&als_ps->update_lock);
	ret = i2c_smbus_write_byte_data(client, REG_PERSISTENCE, persist);
	mutex_unlock(&als_ps->update_lock);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c fail.\n", __func__);
		return ret;
	}
	als_ps->persistence = persist;	//Update the value after successful i2c write to avoid difference. 

	return 0;
}

static int rpr521_set_control(struct i2c_client *client, int control)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int ret;
	unsigned char gain, led_current;
	printk("%s: \n",__func__);

	if(control > REG_ALSPSCTL_MAX)
	{
		printk(KERN_ERR "%s: exceed maximum possible value.\n", __func__);
		return -EINVAL;
	}
	
	gain = (control & 0x3C) >> 2;	//gain setting values
	led_current = control & 0x03;		//led current setting value

	if(!((gain == ALSGAIN_X1X1) || (gain == ALSGAIN_X1X2) || (gain == ALSGAIN_X2X2) || (gain == ALSGAIN_X64X64)
		|| (gain == ALSGAIN_X128X64) || (gain == ALSGAIN_X128X128)))
	{
		printk(KERN_ERR "%s: invalid gain setting. \n", __func__);
		return -EINVAL;
	}
	
	mutex_lock(&als_ps->update_lock);
	ret = i2c_smbus_write_byte_data(client, REG_ALSPSCONTROL, control);
	mutex_unlock(&als_ps->update_lock);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c fail.\n", __func__);
		return ret;
	}
	als_ps->control = control;
	als_ps->gain0 = gain_table[gain].DATA0;
	als_ps->gain1 = gain_table[gain].DATA1;
	als_ps->ledcurrent = led_current;

	return ret;
}
#endif

#if 0
/******************************************************************************
 * NAME       : long_long_divider
 * FUNCTION   : calc divider of unsigned long long int or unsgined long
 * REMARKS    :
 *****************************************************************************/
static void long_long_divider(unsigned long long data, unsigned long base_divier, unsigned long *answer, unsigned long long *overplus)
{
    volatile unsigned long long divier;
    volatile unsigned long      unit_sft;

    if ((long long)data < 0)	// . If data MSB is 1, it may go to endless loop. 
    	{
    	data /= 2;	//0xFFFFFFFFFFFFFFFF / 2 = 0x7FFFFFFFFFFFFFFF
	base_divier /= 2;
    	}
    divier = base_divier;
    if (data > MASK_LONG) {
        unit_sft = 0;
        while (data > divier) {
            unit_sft++;
            divier = divier << 1;
        }
        while (data > base_divier) {
            if (data > divier) {
                *answer += 1 << unit_sft;
                data    -= divier;
            }
            unit_sft--;
            divier = divier >> 1;
        }
        *overplus = data;
    } else {
        *answer = (unsigned long)(data & MASK_LONG) / base_divier;
        /* calculate over plus and shift 16bit */
        *overplus = (unsigned long long)(data - (*answer * base_divier));
    }
}

#else
/******************************************************************************
 * NAME       : long_long_divider
 * FUNCTION   : calc divider of unsigned long long int or unsgined long
 * REMARKS    :
 *****************************************************************************/
static int long_long_divider(long long data, unsigned long base_divier, unsigned long *answer, unsigned long long *overplus)
{
    volatile long long divier;
    volatile long      unit_sft;

    if ((data < 0) || (base_divier == 0)) {
        *answer   = 0;
        *overplus = 0;
        return (CALC_ERROR);
    }

    divier = base_divier;
    if (data > MASK_LONG) {
        unit_sft = 0;
        while ((data > divier) && (divier > 0)) {
            unit_sft++;
            divier = divier << 1;
        }
        //while ((data > base_divier) && (unit_sft > 0)) {
        while ((data > base_divier) && (unit_sft >= 0)) { //grace modify in 2014.8.19
            if (data > divier) {
                *answer += 1 << unit_sft;
                data    -= divier;
            }
            unit_sft--;
            divier = divier >> 1;
        }
        *overplus = data;
    } else {
        *answer = (unsigned long)(data & MASK_LONG) / base_divier;
        /* calculate over plus and shift 16bit */
        *overplus = (unsigned long long)(data - (*answer * base_divier));
    }

    return (0);
}
#endif

/******************************************************************************
 * NAME       : calc_rohm_als_data
 * FUNCTION   : calculate illuminance data for rpr521
 * REMARKS    : final_data is 1000 times, which is defined as CUT_UNIT, of the actual lux value
 *****************************************************************************/
//static int calc_rohm_als_data(unsigned short data0, unsigned short data1, unsigned char gain0, unsigned char gain1, unsigned char time)
static int calc_rohm_als_data(unsigned short data0, unsigned short data1, 
unsigned char gain0, unsigned char gain1, unsigned short time) //grace modify in 2014.5.7
{
#define DECIMAL_BIT      (15)
#define JUDGE_FIXED_COEF (1000)
#define MAX_OUTRANGE    (65535)
#define MAXRANGE_NMODE   (0xFFFF)
#define MAXSET_CASE      (4)

	int                result; //grace modified in 2014.4.2
	int                final_data;
	CALC_DATA          calc_data;
	CALC_ANS           calc_ans;
	unsigned long      calc_judge;
	unsigned char      set_case;
	unsigned long      div_answer;
	unsigned long long div_overplus;
	unsigned long long overplus;
	unsigned long      max_range;

	/* set the value of measured als data */
	calc_data.als_data0  = data0;
	calc_data.als_data1  = data1;
	calc_data.gain_data0 = gain0;

	/* set max range */
	if (calc_data.gain_data0 == 0) 
	{
		/* issue error value when gain is 0 */
		return (CALC_ERROR);
	}
	else
	{
		max_range = MAX_OUTRANGE / calc_data.gain_data0;
	}
	
	/* calculate data */
	if (calc_data.als_data0 == MAXRANGE_NMODE) 
	{
		calc_ans.positive = max_range;
		calc_ans.decimal  = 0;
	} 
	else 
	{
		/* get the value which is measured from power table */
		calc_data.als_time = time;
		if (calc_data.als_time == 0) 
		{
			/* issue error value when time is 0 */
			return (CALC_ERROR);
		}

		calc_judge = calc_data.als_data1 * JUDGE_FIXED_COEF;
		if (calc_judge < (calc_data.als_data0 * judge_coefficient[0])) 
		{
			set_case = 0;
		} 
		else if (calc_judge < (calc_data.als_data0 * judge_coefficient[1]))
		{
			set_case = 1;
		} 
		else if (calc_judge < (calc_data.als_data0 * judge_coefficient[2])) 
		{
			set_case = 2;
		}
		else if (calc_judge < (calc_data.als_data0 * judge_coefficient[3])) 
		{
			 set_case = 3;
		} 
		else
		{
			set_case = MAXSET_CASE;
		}
		calc_ans.positive = 0;
		if (set_case >= MAXSET_CASE) 
		{
			calc_ans.decimal = 0;	//which means that lux output is 0
		}
		else
		{
			calc_data.gain_data1 = gain1;
			if (calc_data.gain_data1 == 0) 
			{
				/* issue error value when gain is 0 */
				return (CALC_ERROR);
			}
			calc_data.data0      = (unsigned long long )(data0_coefficient[set_case] * calc_data.als_data0) * calc_data.gain_data1;
			calc_data.data1      = (unsigned long long )(data1_coefficient[set_case] * calc_data.als_data1) * calc_data.gain_data0;
			if(calc_data.data0 < calc_data.data1)	//In this case, data will be less than 0. As data is unsigned long long, it will become extremely big.
			{
				return (CALC_ERROR);
			}
			else
			{
				calc_data.data       = (calc_data.data0 - calc_data.data1);
			}
			calc_data.dev_unit   = calc_data.gain_data0 * calc_data.gain_data1 * calc_data.als_time * 10;	//24 bit at max (128 * 128 * 100 * 10)
			if (calc_data.dev_unit == 0) 
			{
				/* issue error value when dev_unit is 0 */
				return (CALC_ERROR);
			}

			/* calculate a positive number */
			div_answer   = 0;
			div_overplus = 0;
#if 0  //grace modified in 2014.4.2
			long_long_divider(calc_data.data, calc_data.dev_unit, &div_answer, &div_overplus); 
#else
			result = long_long_divider(calc_data.data, calc_data.dev_unit, &div_answer, &div_overplus);
      if (result == CALC_ERROR)
      {
         return (result);
      }
#endif
			calc_ans.positive = div_answer;
			/* calculate a decimal number */
			calc_ans.decimal = 0;
			overplus         = div_overplus;
			if (calc_ans.positive < max_range)
			{
				if (overplus != 0)
				{
					overplus     = overplus << DECIMAL_BIT;
					div_answer   = 0;
					div_overplus = 0;
#if 0 //grace modified in 2014.4.2
					long_long_divider(overplus, calc_data.dev_unit, &div_answer, &div_overplus);
#else
					result = long_long_divider(overplus, calc_data.dev_unit, &div_answer, &div_overplus);
					if (result == CALC_ERROR)
      		{
         		return (result);
      		}
#endif
					calc_ans.decimal = div_answer;
				}
			}

			else
			{
				calc_ans.positive = max_range;
			}
		}
	}
	
	final_data = calc_ans.positive * CUT_UNIT + ((calc_ans.decimal * CUT_UNIT) >> DECIMAL_BIT);
					
	return (final_data);

#undef DECIMAL_BIT
#undef JUDGE_FIXED_COEF
#undef MAX_OUTRANGE
#undef MAXRANGE_NMODE
#undef MAXSET_CASE
}


/******************************************************************************
 * NAME       : calculate_ps_data
 * FUNCTION   : calculate proximity data for rpr521
 * REMARKS    : 12 bit output
 *****************************************************************************/
static int calc_rohm_ps_data(unsigned short ps_reg_data)
{
    return (ps_reg_data & 0xFFF);
}

static unsigned int rpr521_als_data_to_level(unsigned int als_data)
{
#if 0
#define ALS_LEVEL_NUM 15
	int als_level[ALS_LEVEL_NUM]  = { 0, 50, 100, 150,  200,  250,  300, 350, 400,  450,  550, 650, 750, 900, 1100};
	int als_value[ALS_LEVEL_NUM]  = { 0, 50, 100, 150,  200,  250,  300, 350, 400,  450,  550, 650, 750, 900, 1100};
    	unsigned char idx;

	for(idx = 0; idx < ALS_LEVEL_NUM; idx ++)
	{
		if(als_data < als_value[idx])
		{
			break;
		}
	}
	if(idx >= ALS_LEVEL_NUM)
	{
		printk(KERN_ERR "rpr521 als data to level: exceed range.\n");
		idx = ALS_LEVEL_NUM - 1;
	}
	
	return als_level[idx];
#undef ALS_LEVEL_NUM
#else
	return als_data;
#endif
}

static void rpr521_reschedule_work(struct ALS_PS_DATA *als_ps,
					  unsigned long delay)
{
	unsigned long flags;

	spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags);

	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	cancel_delayed_work(&als_ps->dwork);
	schedule_delayed_work(&als_ps->dwork, delay);

	spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);
}
int zte_rpr521_read_all_reg(struct i2c_client *client)//xym add
{
#if 0
	int addr=0x40;
	int data=0x0;
	int i=0;
	for(i=0;i<0x15;i++)
	{
		if ((addr+i)==0x4A) 
			continue;

		data = i2c_smbus_read_byte_data(client, addr+i);
		printk("%s: REG[0x%X]=0x%2X \n", __func__, addr+i, data);

	}
#endif
	return 0;
}

/* ALS polling routine */
static void rpr521_als_polling_work_handler(struct work_struct *work)
{
	struct ALS_PS_DATA *als_ps = container_of(work, struct ALS_PS_DATA, als_dwork.work);
	struct i2c_client *client=als_ps->client;
	int tmp = 0;
	
	schedule_delayed_work(&als_ps->als_dwork, msecs_to_jiffies(als_ps->als_poll_delay));	// restart timer
	//printk("%s: restart timer als_poll_delay=%dms\n", __func__, als_ps->als_poll_delay);

	zte_rpr521_read_all_reg(client);
	
	tmp = i2c_smbus_read_word_data(client, REG_ALSDATA0);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read data0 fail. \n", __func__);
		//return tmp;
	}
	als_ps->als_data0_raw = (unsigned short)tmp;
	tmp = i2c_smbus_read_word_data(client, REG_ALSDATA1);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read data1 fail. \n", __func__);
		//return tmp;
	}
	als_ps->als_data1_raw = (unsigned short)tmp;

// Theorically it is not necesssary to do so, but I just want to avoid any potential error.  -- Andy 2012.6.6
	tmp = i2c_smbus_read_byte_data(client, REG_ALSPSCONTROL);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read gain fail. \n", __func__);
		//return tmp;
	}
	//printk("%s: read REG_ALSPSCONTROL=0X%x\n", __func__, tmp);
	tmp = (tmp & 0x3C) >> 2;
	als_ps->gain0 = gain_table[tmp].DATA0;
	als_ps->gain1 = gain_table[tmp].DATA1;	
	
	tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
	//printk("%s: REG_MODECONTROL=0x%X\n", __func__, tmp);

	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read time fail. \n", __func__);
		//return tmp;
	}
	tmp = tmp & 0xF;
	als_ps->als_time = mode_table[tmp].ALS;	
	
	als_ps->als_data = calc_rohm_als_data(als_ps->als_data0_raw, als_ps->als_data1_raw, 
		als_ps->gain0, als_ps->gain1, als_ps->als_time);	
	//if(als_ps->als_data == 0)
	//	als_ps->als_data ++;
	//printk("%s:als_ps->als_data=%d\n", __func__, als_ps->als_data);
	als_ps->als_level = rpr521_als_data_to_level(als_ps->als_data);

	/*printk("%s: als report: data0 = %d, data1 = %d, gain0 = %d, gain1 = %d, time = %d, lux = %d, level = %d.\n", 
		__func__, als_ps->als_data0_raw, als_ps->als_data1_raw, als_ps->gain0, als_ps->gain1, als_ps->als_time,
		als_ps->als_data, als_ps->als_level);*/

	tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
	//printk("%s: read REG_PSDATA=0x%X \n", __func__, tmp);
	//tmp = i2c_smbus_read_byte_data(client, REG_INTERRUPT);
	//printk("REG_INTERRUPT=%x\n", tmp);
	if(als_ps->als_data != CALC_ERROR)
	{
		if(rpr521_is_boot <= 20)//xym add
		{
			printk("%s: %dst data - report %d lux\n", __func__, (rpr521_is_boot), als_ps->als_level);
			rpr521_is_boot++;
		}
		input_report_abs(als_ps->input_dev_als, ABS_MISC, als_ps->als_level); // report als data. maybe necessary to convert to lux level
		input_sync(als_ps->input_dev_als);	
		//printk("%s: als report %d\n", __func__, als_ps->als_level);
	}	
}


/* PS interrupt routine */
static void rpr521_ps_int_work_handler(struct work_struct *work)
{
	struct ALS_PS_DATA *als_ps = container_of((struct delayed_work *)work, struct ALS_PS_DATA, dwork);
	struct i2c_client *client=als_ps->client;
	int tmp;

	printk("%s:\n", __func__);
	zte_rpr521_read_all_reg(client);

	tmp =  i2c_smbus_read_byte_data(client, REG_INTERRUPT);
	if(tmp < 0)
	{
		printk(KERN_ERR "%s: i2c read interrupt status fail. \n", __func__);
		//return;
		goto err_exit; //grace modify in 2014.9.5
	}
	if(tmp & PS_INT_MASK)	//Interrupt is caused by PS
	{
		tmp = i2c_smbus_read_byte_data(client, REG_ALSPSCONTROL);
		if(tmp < 0)
		{
			printk(KERN_ERR "%s: i2c read led current fail. \n", __func__);
			//return;
			goto err_exit; //grace modify in 2014.5.7
		}
		als_ps->ledcurrent = tmp & 0x3;
		
		tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
		if(tmp < 0)
		{
			printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
			//return;
			goto err_exit; //grace modify in 2014.5.7
		}
		als_ps->ps_data_raw = (unsigned short)tmp;

		als_ps->ps_data = calc_rohm_ps_data(als_ps->ps_data_raw);

		if(als_ps->ps_data > als_ps->ps_th_h)
		{		
			//grace modify in 2014.5.7 begin	
			tmp = i2c_smbus_read_byte_data(client, REG_PERSISTENCE);
			if(tmp < 0)
			{
				printk(KERN_ERR "%s: i2c read led current fail. \n", __func__);
				goto err_exit; 
			}
			if ((tmp>>6) == INFRARED_LOW)
			{
				als_ps->ps_direction = 0;//near
#ifdef _AUTO_THRESHOLD_CHANGE_
				rpr521_set_ps_threshold_high(client, REG_PSTH_MAX);
				rpr521_set_ps_threshold_low(client, als_ps->ps_th_l_back);
#endif
			}
			else
			{
				goto err_exit;
			}
			//grace modify in 2014.5.7 end			
		}
		else if(als_ps->ps_data < als_ps->ps_th_l)
		{
			als_ps->ps_direction = 1;//far
#ifdef _AUTO_THRESHOLD_CHANGE_
			rpr521_set_ps_threshold_high(client, als_ps->ps_th_h_back);
			rpr521_set_ps_threshold_low(client, 0);
#endif
		}

		tmp = i2c_smbus_read_byte_data(client, REG_PERSISTENCE);
		if(tmp < 0)
		{
			printk(KERN_ERR "%s: i2c read REG_PERSISTENCE fail. \n", __func__);
			//return;
			goto err_exit; //grace modify in 2014.9.5
		}

		/*printk(KERN_INFO "%s: ps report: raw_data = %d, data = %d, direction = %d, REG_PERSISTENCE = 0x%X. \n", 
				__func__, als_ps->ps_data_raw, als_ps->ps_data, als_ps->ps_direction, tmp);*/
				
		printk(KERN_INFO "%s: ps now thresh is %d %d, ps thresh back is %d %d. \n",
				__func__, i2c_smbus_read_word_data(client, REG_PSTH), 
				i2c_smbus_read_word_data(client, REG_PSTL), 
				als_ps->ps_th_h_back, als_ps->ps_th_l_back );


		input_report_abs(als_ps->input_dev_ps, ABS_DISTANCE, als_ps->ps_direction); 
		input_sync(als_ps->input_dev_ps);	
		printk("%s: ps report %d, %s \n", __func__, als_ps->ps_data, als_ps->ps_direction?"FAR":"NEAR");
	}
	else
	{
		printk(KERN_ERR "%s: unknown interrupt source.\n", __func__);
	}
	
	//enable_irq(client->irq);
	
//grace modify in 2014.5.7 begin
err_exit:
	enable_irq(client->irq);
//grace modify in 2014.5.7 end
}

/* assume this is ISR */
static irqreturn_t rpr521_interrupt(int vec, void *info)
{
	struct i2c_client *client=(struct i2c_client *)info;
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	disable_irq_nosync(client->irq);
	//printk("%s\n", __func__);
	rpr521_reschedule_work(als_ps, 0);

	return IRQ_HANDLED;
}

/*************** SysFS Support ******************/
static ssize_t rpr521_show_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", als_ps->enable_ps_sensor);
}

#ifdef ZTE_RPR521_CALIBRATION //xym add
static unsigned int rpr521_ps_th_h_factory ;
static unsigned int rpr521_ps_th_l_factory ;

static int rpr521_auto_calibrate(struct i2c_client *client);
#endif

static ssize_t rpr521_store_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
// 	unsigned long flags;
	int tmp;
//	struct input_dev *input_dev = als_ps->input_dev_ps;

	printk(KERN_INFO "%s: %ld ++++++\n", __func__, val);
		
	if ((val != 0) && (val != 1)) 
	{
		printk("%s: store unvalid value=%ld\n", __func__, val);
		return count;
	}
	
	//mutex_lock(&input_dev->mutex);
	if(val == 1) 
	{
		//turn on p sensor
		//wake_lock(&ps_lock);

		if (als_ps->enable_ps_sensor == 0) 
		{
			als_ps->enable_ps_sensor = 1;
		
			tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
           //printk("%s: read REG_MODECONTROL=0x%X\n", __func__, tmp);
			tmp = tmp | 0x40;
			rpr521_set_measurement_time(client, tmp);	//PS on
			
#ifdef ZTE_RPR521_CALIBRATION//xym add begin
			//disable_irq_nosync(client->irq);
			rpr521_set_ps_threshold_high(client, rpr521_ps_th_h_factory);
			rpr521_set_ps_threshold_low(client, rpr521_ps_th_l_factory);

			rpr521_auto_calibrate(client);//xym add
			//enable_irq(client->irq);
#endif			
		}
	} 
	else 
	{
		if(als_ps->enable_ps_sensor == 1)
		{
			als_ps->enable_ps_sensor = 0;
			
			tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
			tmp = tmp & (~0x40);
			rpr521_set_measurement_time(client, tmp);	//PS off
			
			//wake_unlock(&ps_lock);
		}
	}

	//mutex_unlock(&input_dev->mutex);	
	printk(KERN_INFO "%s: ------\n", __func__);
	return count;
}

static ssize_t rpr521_show_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", als_ps->enable_als_sensor);
}
static ssize_t rpr521_store_enable_als_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
 	unsigned long flags;
	int tmp;
	
	printk(KERN_INFO "%s: %ld ++++++\n", __func__, val);

	if ((val != 0) && (val != 1))
	{
		printk("%s: enable als sensor=%ld\n", __func__, val);
		return count;
	}
	
	if(val == 1)
	{
		//turn on light  sensor
		if (als_ps->enable_als_sensor==0)
		{
			als_ps->enable_als_sensor = 1;

			tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
           //printk("%s: read REG_MODECONTROL=0x%X\n", __func__, tmp);
			tmp = tmp | 0x80;
			rpr521_set_measurement_time(client, tmp);	//ALS on
		}
		
		spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags); 
       cancel_delayed_work(&als_ps->als_dwork);
		schedule_delayed_work(&als_ps->als_dwork, msecs_to_jiffies(als_ps->als_poll_delay));	// 125ms
		spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);	
	}
	else
	{
		if(als_ps->enable_als_sensor == 1)
		{
			als_ps->enable_als_sensor = 0;

			tmp = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
			tmp = tmp & (~0x80);
			rpr521_set_measurement_time(client, tmp);	//ALS off
		}
		
		spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags); 
		cancel_delayed_work(&als_ps->als_dwork);
		spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);	
	}
	
	return count;
}

static ssize_t rpr521_show_als_poll_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", als_ps->als_poll_delay*1000);	// return in micro-second
}

static ssize_t rpr521_store_als_poll_delay(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
//	int ret;
//	int poll_delay = 0;
 	unsigned long flags;
	
	if (val < PS_ALS_SET_MIN_DELAY_TIME * 1000)
	{
		val = PS_ALS_SET_MIN_DELAY_TIME * 1000;	
		printk("%s: %ld ms less than %dms, set to %dms\n", __func__, val/1000, PS_ALS_SET_MIN_DELAY_TIME, PS_ALS_SET_MIN_DELAY_TIME);
	}
	
	als_ps->als_poll_delay = val /1000;	// convert us => ms
	printk("%s: %d ms\n", __func__, als_ps->als_poll_delay);
	
	if (als_ps->enable_als_sensor == 1)//grace modified in 2013.10.09
	{
	
	/* we need this polling timer routine for sunlight canellation */
	spin_lock_irqsave(&als_ps->update_lock.wait_lock, flags); 
		
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	cancel_delayed_work(&als_ps->als_dwork);
	schedule_delayed_work(&als_ps->als_dwork, msecs_to_jiffies(als_ps->als_poll_delay));	// 125ms
			
	spin_unlock_irqrestore(&als_ps->update_lock.wait_lock, flags);	
	
	}
	return count;
}

#ifdef ZTE_RPR521_SENSORS_CLASS_DEV //xym add 
static int rpr521_ps_sensors_enable(struct sensors_classdev *sensors_cdev,
					unsigned int enabled)
{
	char buf[2]={0};
	struct ALS_PS_DATA *als_ps = container_of(sensors_cdev,
			struct ALS_PS_DATA, ps_cdev);

	printk("%s: %d\n", __func__, enabled);
	if (enabled == 1)
	{
		buf[0]='1';
	}
	else if (enabled == 0)
	{
		buf[0]='0';
	}

	return rpr521_store_enable_ps_sensor(&(als_ps->client->dev), NULL, buf, 2);
}

static int rpr521_als_sensors_enable(struct sensors_classdev *sensors_cdev,
					unsigned int enabled)
{
	char buf[2]={0};
	struct ALS_PS_DATA *als_ps = container_of(sensors_cdev,
			struct ALS_PS_DATA, als_cdev);
	
	printk("%s: %d\n", __func__, enabled);
	
	if (enabled == 1)
	{
		buf[0]='1';
	}
	else if (enabled == 0)
	{
		buf[0]='0';
	}

	return rpr521_store_enable_als_sensor(&(als_ps->client->dev), NULL, buf, 2);
}

static int rpr521_als_poll_delay(struct sensors_classdev *sensors_cdev,
					unsigned int delay_msec)
{
	char buf[20]={0};
	size_t count = 0;
	struct ALS_PS_DATA *als_ps = container_of(sensors_cdev,
			struct ALS_PS_DATA, als_cdev);
	printk("%s: %d ms\n", __func__, delay_msec);
	
	count = num_to_str(buf, 20, delay_msec*1000);
	buf[count]=0;

	return rpr521_store_als_poll_delay(&(als_ps->client->dev), NULL, buf, count);
}
#endif


//grace add in 2013.10.09 begin
static ssize_t rpr521_show_als_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	int tmp;
	int tmp1;
	tmp = i2c_smbus_read_word_data(client, REG_ALSDATA0);
	tmp1 = i2c_smbus_read_word_data(client, REG_ALSDATA1);
	als_ps->als_data = calc_rohm_als_data(als_ps->als_data0_raw, als_ps->als_data1_raw, 
		als_ps->gain0, als_ps->gain1, als_ps->als_time);	
	return sprintf(buf, "%d %d %d\n", tmp, tmp1, als_ps->als_data);
}

static ssize_t rpr521_show_ps_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int tmp = 0;
	tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
	return sprintf(buf, "%d\n", tmp);
}
//grace add in 2013.10.09 end

static ssize_t rpr521_show_ps_thres_high(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	//struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	//grace test in 2013.10.09 begin
	int ps_data = 0, ps_high = 0, ps_low = 0;
	
	ps_data = i2c_smbus_read_word_data(client, REG_PSDATA);
	if(ps_data < 0)
	{
			printk(KERN_ERR "%s: i2c read led current fail. \n", __func__);
			return -1;
	}
	
	ps_high = i2c_smbus_read_word_data(client, REG_PSTH);
	if(ps_high < 0)
	{
			printk(KERN_ERR "%s: i2c read led current fail. \n", __func__);
			return -1;
	}
	
	ps_low = i2c_smbus_read_word_data(client, REG_PSTL);
	if(ps_low < 0)
	{
			printk(KERN_ERR "%s: i2c read led current fail. \n", __func__);
			return -1;
	}
	
	//return sprintf(buf, "%d\n", als_ps->ps_th_h);	
	return sprintf(buf, "%d %d %d\n", ps_data, ps_high, ps_low);
	//grace test in 2013.10.09 end
}

static ssize_t rpr521_store_ps_thres_high(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	printk("%s: \n",__func__);

	if(!(rpr521_set_ps_threshold_high(client, val)))
		als_ps->ps_th_h_back = als_ps->ps_th_h;
	
	return count;
}

static ssize_t rpr521_show_ps_thres_low(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\n", als_ps->ps_th_l);	
}

static ssize_t rpr521_store_ps_thres_low(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	printk("%s: \n",__func__);
	
	if(!(rpr521_set_ps_threshold_low(client, val)))
		als_ps->ps_th_l_back = als_ps->ps_th_l;
	
	return count;
}

static ssize_t rpr521_show_ps_calib(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	return sprintf(buf, "%d\t%d\n", als_ps->ps_th_h, als_ps->ps_th_l);	
}

static ssize_t rpr521_store_ps_calib(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
#define SET_LOW_THRES	1
#define SET_HIGH_THRES	2
#define SET_BOTH_THRES	3

	struct i2c_client *client = to_i2c_client(dev);
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
 	unsigned int i, tmp, ps_th_h, ps_th_l;
	int average;	//This should be signed to avoid error.
	printk("%s: \n",__func__);

	switch(val)
	{
		case SET_LOW_THRES:		
		//Take 20 average for noise. use noise + THRES_TOLERANCE as low threshold.
		//If high threshold is lower than the new low threshold + THRES_DIFF, make the high one equal low + THRES_DIFF
		//Please make sure that there is NO object above the sensor, otherwise it may cause the high threshold too high to trigger which make the LCD NEVER shutdown.
		//If the noise is too large, larger than 4065, it will return -1. If so, the mechanical design MUST be redo. It is quite unlikely. 
			average = 0;
			ps_th_h = als_ps->ps_th_h_back;
			ps_th_l = als_ps->ps_th_l_back;
			for(i = 0; i < 20; i ++)
			{
				tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
				if(tmp < 0)
				{
					printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
					return -1;
				}
				average += tmp & 0xFFF;	// 12 bit data
			}
			average /= 20;		//This is the average noise
			ps_th_l = average + THRES_TOLERANCE;	
			if(ps_th_l > REG_PSTL_MAX)
			{
			//printk(KERN_ERR "%d in %s: low threshold is too high. \n", __line__, __func__);
				return -1;
			}
			if(ps_th_l < 0)
			{
			//printk(KERN_ERR "%d in %s: low threshold is too low. \n", __line__, __func__);
				return -1;
			}
			if(ps_th_h < ps_th_l + THRES_DIFF)
			{
				ps_th_h = ps_th_l + THRES_DIFF;	//It will not be minus or an error should have occured earlier. 
				if(ps_th_h > REG_PSTH_MAX)
				{
			//printk(KERN_ERR "%d in %s: high threshold is too high. \n", __line__, __func__);
					return -1;
				}
				if(!rpr521_set_ps_threshold_high(client, ps_th_h))
					als_ps->ps_th_h_back = ps_th_h;
				else
					return -1;
			}
			if(!rpr521_set_ps_threshold_low(client, ps_th_l))
				als_ps->ps_th_l_back = ps_th_l;
			else
				return -1;
			break;
		
		case SET_HIGH_THRES:	
		//Take 20 average for signal. use signal -THRES_TOLERANCE as high threshold. 
		//If low threshold is higher than the new high one - THRES_DIFF, make the low one equal high - THRES_DIFF
		//Please make sure that there IS AN object above the sensor, otherwise it will be a disaster. The high threshold will be too low which will cause the panel ALWAYS shutdown
		//Customer can use their own standard to set the test scenario. For example, a 18% grey card @ 2cm, or another example, a 90% white card 4cm, and etc. 
		//If the signal is too weak, less than 30, it will return -1. If so, the mechanical design MUST be redo. It shall not happen very frequently. 
			average = 0;
			ps_th_h = als_ps->ps_th_h_back;
			ps_th_l = als_ps->ps_th_l_back;
			for(i = 0; i < 20; i ++)
			{
				tmp = i2c_smbus_read_word_data(client, REG_PSDATA);
				if(tmp < 0)
				{
					printk(KERN_ERR "%s: i2c read ps data fail. \n", __func__);
					return -1;
				}
				average += tmp & 0xFFF;	// 12 bit data
			}
			average /= 20;		//This is the average signal
			ps_th_h = average - THRES_TOLERANCE;
			if(ps_th_h > REG_PSTH_MAX)
			{
			//printk(KERN_ERR "%d in %s: high threshold is too high. \n", __line__, __func__);
				return -1;
			}
			if(ps_th_h < 0)
			{
			//printk(KERN_ERR "%d in %s: high threshold is too low. \n", __line__, __func__);
				return -1;
			}
			if(ps_th_l > ps_th_h - THRES_DIFF)
			{
				ps_th_l = ps_th_h - THRES_DIFF;	//Given that REG_PSTH_MAX = REG_PSTL+MAX, it will not be greater than REG_PSTL_MAX or an error should have occured earlier.
				if(ps_th_l < 0)
				{
					//printk(KERN_ERR "%d in %s: low threshold is too low. \n", __line__, __func__);
					return -1;
				}
				if(!rpr521_set_ps_threshold_low(client, ps_th_l))
					als_ps->ps_th_l_back = ps_th_l;
				else
					return -1;
			}
			if(!rpr521_set_ps_threshold_high(client, ps_th_h))
				als_ps->ps_th_h_back = ps_th_h;
			else
				return -1;
			break;
		
		case SET_BOTH_THRES:	//Take 20 average for noise. use noise + PS_ALS_SET_PS_TL as low threshold, noise + PS_ALS_SET_PS_TH as high threshold
			rpr521_calibrate(client);
			break;

		default:
			return -EINVAL;	//NOT supported!
	}
			
	return count;

#undef SET_BOTH_THRES
#undef SET_HIGH_THRES
#undef SET_LOW_THRES
}

static ssize_t rpr521_show_type(struct device *dev,
                struct device_attribute *attr, char *buf){
    struct i2c_client *client = to_i2c_client(dev);
    struct ALS_PS_DATA *data = i2c_get_clientdata(client);

    return sprintf(buf, "%d\n", data->type);
}

#ifdef ZTE_RPR521_CALIBRATION //xym add
#define RPR521_AUTO_CALI_CNT 3
static int rpr521_auto_calibrate(struct i2c_client *client)
{
	struct ALS_PS_DATA *data = i2c_get_clientdata(client);

	int ret=0;
 	unsigned int i, ps_th_h, ps_th_l;	
	unsigned int temp_pdata[RPR521_AUTO_CALI_CNT];
	int average, max, min;
	u8 infrared_data = 0; //grace modify in 2014.12.18
	int tmp = 0;
	int reg_mode_backup = 0;

	
	printk("%s: +++\n", __FUNCTION__);
	
	if(data->enable_ps_sensor==0)
	{
		printk("%s: prox now is power off, exit\n", __FUNCTION__);
		return 3;
	}
	
	reg_mode_backup = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
	
	tmp = i2c_smbus_write_byte_data(client, REG_INTERRUPT, PS_ALS_SET_INTR&0XFE); //disable ps interrupt
	if(tmp < 0)	
	{		
		goto err_exit;	
	}	
	
	rpr521_set_measurement_time(client, PS_EN|PS_DOUBLE_PULSE|PS10MS);  //PS 10ms   0x61

	printk("%s: prox now is power on\n", __FUNCTION__);
	printk("%s: do calibrate\n", __FUNCTION__);
	
	printk("%s: old ps thresh is %d %d\n", 
		__FUNCTION__, data->ps_th_h_back, data->ps_th_l_back);


	average = 0;
	max = 0;
	min = 0XFFFF;
	for (i = 0; i < RPR521_AUTO_CALI_CNT; i++) {
		msleep(20);
		
		//检查红外数据是否过高begin
		tmp = i2c_smbus_read_byte_data(client, REG_PERSISTENCE);
		if(tmp < 0)	
		{		
			goto err_exit;	
		}	
		infrared_data = tmp;	

		if(infrared_data>>6)  //说明红外数据高
		{		
			printk("%s: rpr521 infra pers= %x\n", __FUNCTION__, tmp);
			goto err_exit;
		}
		//检查红外数据是否过高end
	
		temp_pdata[i] = i2c_smbus_read_word_data(client, REG_PSDATA);
		printk("%s: prox temp_pdata= %d\n", __FUNCTION__, temp_pdata[i]);
		if(temp_pdata[i] < 0)
		{
			printk(KERN_ERR "%s: i2c read ps data[%d] fail. \n", __func__, i);
		}
		data->ps_data_raw = (unsigned short)temp_pdata[i];
		data->ps_data = calc_rohm_ps_data(data->ps_data_raw);
		
		average += temp_pdata[i] & 0xFFF;	// 12 bit data
		//printk("%s: prox average= %d\n", __FUNCTION__, average);

		if(max < temp_pdata[i]) 
		{ 
		    max = temp_pdata[i];
		}
		if(min > temp_pdata[i]) 
		{ 
		    min = temp_pdata[i];
		}
	}

	average /= RPR521_AUTO_CALI_CNT;

	printk("%s: average=%d, max=%d, min=%d\n", __FUNCTION__, average, max, min);
	if (average > PS_CROSSTALK)
	{
		printk(KERN_ERR "%s: average(%d) > PS_CROSSTALK(%d)\n", __func__, average, PS_CROSSTALK);
		goto err_exit;
	}

	ps_th_h = average + THRES_TOLERANCE + THRES_DEFAULT_DIFF;
	ps_th_l = average + THRES_TOLERANCE;

	if(ps_th_h < 0)
	{
		printk(KERN_ERR "%s: high threshold is less than 0.\n", __func__);
		goto err_exit;
	}
	if(ps_th_h > REG_PSTH_MAX)
	{
		printk(KERN_ERR "%s: high threshold is greater than maximum allowed value.\n", __func__);
		goto err_exit;
	}
	if(ps_th_l < 0)
	{
		printk(KERN_ERR "%s: low threshold is less than 0.\n", __func__);
		goto err_exit;
	}
	if(ps_th_l > REG_PSTL_MAX)
	{
		printk(KERN_ERR "%s: low threshold is greater than maximum allowed value.\n", __func__);
		goto err_exit;
	}
	
	data->ps_th_h = ps_th_h;
	data->ps_th_l = ps_th_l;

	if(!(rpr521_set_ps_threshold_high(client, ps_th_h)))
		data->ps_th_h_back = ps_th_h;
	else 
		goto err_exit;
	
	if(!(rpr521_set_ps_threshold_low(client, ps_th_l)))
		data->ps_th_l_back = ps_th_l;
	else
		goto err_exit;

	printk("%s: new ps thresh is %d %d\n", __func__, data->ps_th_h, data->ps_th_l);
	
	rpr521_set_measurement_time(client, reg_mode_backup); 
	tmp = i2c_smbus_write_byte_data(client, REG_INTERRUPT, PS_ALS_SET_INTR|0x1); //enable ps interrupt
	if(tmp < 0)	
	{		
		goto err_exit;	
	}
	
	printk("%s: ---succ\n", __FUNCTION__);
	return ret;
	
err_exit:
	/*ps_th_h = PS_ALS_SET_PS_TH;
	ps_th_l = PS_ALS_SET_PS_TL;

	if(!(rpr521_set_ps_threshold_high(client, data->ps_th_h)))
		data->ps_th_h_back = data->ps_th_h;
	if(!(rpr521_set_ps_threshold_low(client, data->ps_th_l)))
		data->ps_th_l_back = data->ps_th_l;
		
	printk("%s: new ps thresh is (factory) %d %d\n", __func__, data->ps_th_h, data->ps_th_l);
*/
	rpr521_set_measurement_time(client, reg_mode_backup); 
	tmp = i2c_smbus_write_byte_data(client, REG_INTERRUPT, PS_ALS_SET_INTR|0x1); //enable ps interrupt

	printk("%s: ---fail\n", __FUNCTION__);
	return ret;
	
}

static ssize_t rpr521_calibrate_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct ALS_PS_DATA *data = i2c_get_clientdata(client);

	printk("%s: %d,%d,%d,%d\n", __FUNCTION__, 
		data->ps_th_h,
		0,//data->led_pulse_count,
		0,
		data->ps_th_l);
	return sprintf(buf, "%d,%d,%d,%d\n", 
		data->ps_th_h,
		0,//data->led_pulse_count,
		0,
		data->ps_th_l);
}

#define RPR521_CALI_CNT 20
static ssize_t rpr521_calibrate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct ALS_PS_DATA *data = i2c_get_clientdata(client);
	int tmp=0;

	int ret=0;
 	unsigned int i;	
	unsigned int temp_pdata[RPR521_CALI_CNT];
	int average, max, min;
	int reg_mode_backup = 0;

	printk("%s: +++\n", __FUNCTION__);
	
	if(data->enable_ps_sensor==0)
	{
		printk("%s: prox now is power off, exit\n", __FUNCTION__);
		return 3;
	}
	reg_mode_backup = i2c_smbus_read_byte_data(client, REG_MODECONTROL);

	tmp = i2c_smbus_write_byte_data(client, REG_INTERRUPT, PS_ALS_SET_INTR&0XFE); //disable ps interrupt
	if(tmp < 0)	
	{		
		goto err_exit;	
	}	

	rpr521_set_measurement_time(client, PS_EN|PS_DOUBLE_PULSE|PS10MS);  //PS 10ms   0x61

	printk("%s: prox now is power on\n", __FUNCTION__);
	printk("%s: do calibrate\n", __FUNCTION__);
	
	printk("%s: old ps thresh is %d %d\n", 
		__FUNCTION__, data->ps_th_h_back, data->ps_th_l_back);

	average = 0;
	max = 0;
	min = 0XFFFF;
	for (i = 0; i < RPR521_CALI_CNT; i++) {
		msleep(20);

		temp_pdata[i] = i2c_smbus_read_word_data(client, REG_PSDATA);
		printk("%s: prox temp_pdata= %d\n", __FUNCTION__, temp_pdata[i]);
		if(temp_pdata[i] < 0)
		{
			printk(KERN_ERR "%s: i2c read ps data[%d] fail. \n", __func__, i);
		}
		data->ps_data_raw = (unsigned short)temp_pdata[i];
		data->ps_data = calc_rohm_ps_data(data->ps_data_raw);
		
		average += temp_pdata[i] & 0xFFF;	// 12 bit data
		//printk("%s: prox average= %d\n", __FUNCTION__, average);

		if(max < temp_pdata[i]) 
		{ 
		    max = temp_pdata[i];
		}
		if(min > temp_pdata[i]) 
		{ 
		    min = temp_pdata[i];
		}
	}

	average /= RPR521_CALI_CNT;

	printk("%s: average=%d, max=%d, min=%d\n", __FUNCTION__, average, max, min);
	if (average > PS_CROSSTALK)
	{
		printk(KERN_ERR "%s: average(%d) > PS_CROSSTALK(%d)\n", __func__, average, PS_CROSSTALK);
		goto err_exit;
	}

	rpr521_ps_th_h_factory = average + THRES_TOLERANCE + THRES_DEFAULT_DIFF;
	rpr521_ps_th_l_factory = average + THRES_TOLERANCE;

	if(rpr521_ps_th_h_factory < 0)
	{
		printk(KERN_ERR "%s: high threshold is less than 0.\n", __func__);
		goto err_exit;
	}
	if(rpr521_ps_th_h_factory > REG_PSTH_MAX)
	{
		printk(KERN_ERR "%s: high threshold is greater than maximum allowed value.\n", __func__);
		goto err_exit;
	}
	if(rpr521_ps_th_l_factory < 0)
	{
		printk(KERN_ERR "%s: low threshold is less than 0.\n", __func__);
		goto err_exit;
	}
	if(rpr521_ps_th_l_factory > REG_PSTL_MAX)
	{
		printk(KERN_ERR "%s: low threshold is greater than maximum allowed value.\n", __func__);
		goto err_exit;
	}
	
	data->ps_th_h = rpr521_ps_th_h_factory;
	data->ps_th_l = rpr521_ps_th_l_factory;

	if(!(rpr521_set_ps_threshold_high(client, data->ps_th_h)))
		data->ps_th_h_back = data->ps_th_h;
	else 
		goto err_exit;
	if(!(rpr521_set_ps_threshold_low(client, data->ps_th_l)))
		data->ps_th_l_back = data->ps_th_l;
	else
		goto err_exit;

	printk("%s: new ps thresh is factory %d %d\n", __func__, data->ps_th_h, data->ps_th_l);
	
	rpr521_set_measurement_time(client, reg_mode_backup); 
	tmp = i2c_smbus_write_byte_data(client, REG_INTERRUPT, PS_ALS_SET_INTR|0x1); //enable ps interrupt
	if(tmp < 0)	
	{		
		goto err_exit;	
	}
	
	printk("%s: ---succ\n", __FUNCTION__);
	return ret;
	
err_exit:
	rpr521_ps_th_h_factory = PS_ALS_SET_PS_TH;
	rpr521_ps_th_l_factory = PS_ALS_SET_PS_TL;
	data->ps_th_h = PS_ALS_SET_PS_TH;
	data->ps_th_l = PS_ALS_SET_PS_TL;

	if(!(rpr521_set_ps_threshold_high(client, data->ps_th_h)))
		data->ps_th_h_back = data->ps_th_h;
	if(!(rpr521_set_ps_threshold_low(client, data->ps_th_l)))
		data->ps_th_l_back = data->ps_th_l;
		
	printk("%s: set ps thresh is default %d %d\n", __func__, data->ps_th_h, data->ps_th_l);
	rpr521_set_measurement_time(client, reg_mode_backup); 
	tmp = i2c_smbus_write_byte_data(client, REG_INTERRUPT, PS_ALS_SET_INTR|0x1); //enable ps interrupt

	printk("%s: ---fail\n", __FUNCTION__);
	return ret;
	
}



static ssize_t rpr521_set_calibrate_data_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct ALS_PS_DATA *data = i2c_get_clientdata(client);

  int i, j;
  u16 prox_threshold_hi = 0;
  u16 prox_threshold_lo = 0;
  u8 prox_pulse_cnt = 0;
  u8 prox_gain = 0;
  char cal_data_char[4][20] = {{0},{0},{0},{0}};
  char buf_data[100] = {0};
  char *tmp = buf_data;
  
  printk("%s: +++\n", __FUNCTION__); 
  printk("%s: user insert buf is %s\n", __FUNCTION__, buf); 
  sprintf(buf_data, "%s\n", buf);
  printk("%s: user insert buf_data is %s\n", __FUNCTION__, buf_data);
  
  for(i = 0, j = 0; *tmp != '\0' && i <= 3 && j < 20; ) 
  {
    if(*tmp  == ',') 
    {
      *(cal_data_char[i] +j) = '\0';
      i ++;
      j = 0;
      if (i >3 ) 
      {
        printk("%s: sensor_hal: array bounds!\n", __FUNCTION__);
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
  
  printk("%s: transform to number: high:%d, pluse:%d, gain:%d, low:%d\n", 
    __FUNCTION__, prox_threshold_hi,prox_pulse_cnt, prox_gain, prox_threshold_lo);

  rpr521_ps_th_h_factory = prox_threshold_hi;
  rpr521_ps_th_l_factory = prox_threshold_lo;
	
  data->ps_th_h = prox_threshold_hi;
  //data->led_pulse_count = prox_pulse_cnt;//not use
  data->ps_th_l = prox_threshold_lo;

	printk("%s: set factory ps threshold %d-%d\n", __FUNCTION__, data->ps_th_h, data->ps_th_l);  
	if(!(rpr521_set_ps_threshold_high(client, data->ps_th_h)))
		data->ps_th_h_back = data->ps_th_h;
	if(!(rpr521_set_ps_threshold_low(client, data->ps_th_l)))
		data->ps_th_l_back = data->ps_th_l;

	
  printk("%s: ---\n", __FUNCTION__); 

  return count;

}

#endif

#if 0//xym
static DEVICE_ATTR(als_poll_delay,  0660,
				    rpr521_show_als_poll_delay, rpr521_store_als_poll_delay);

static DEVICE_ATTR(enable_als_sensor,  0660 ,
				  rpr521_show_enable_als_sensor, rpr521_store_enable_als_sensor);

static DEVICE_ATTR(enable_ps_sensor,  0660 ,
				   rpr521_show_enable_ps_sensor, rpr521_store_enable_ps_sensor);

static DEVICE_ATTR(ps_thres_high,  0660 ,
				  rpr521_show_ps_thres_high, rpr521_store_ps_thres_high);

static DEVICE_ATTR(ps_thres_low,  0660 ,
				   rpr521_show_ps_thres_low, rpr521_store_ps_thres_low);

static DEVICE_ATTR(ps_calib,  0660 ,
				   rpr521_show_ps_calib, rpr521_store_ps_calib);
static DEVICE_ATTR(als_data, S_IRUGO, rpr521_show_als_data, NULL); //grace add in 2013.10.09
static DEVICE_ATTR(ps_data, S_IRUGO, rpr521_show_ps_data, NULL);//grace add in 2013.10.09
static DEVICE_ATTR(type, S_IRUGO, rpr521_show_type, NULL);//Add for EngineerMode

static struct attribute *rpr521_attributes[] = {
	&dev_attr_enable_ps_sensor.attr,
	&dev_attr_enable_als_sensor.attr,
	&dev_attr_als_poll_delay.attr,
	&dev_attr_ps_thres_high.attr,
	&dev_attr_ps_thres_low.attr,
	&dev_attr_ps_calib.attr,
	&dev_attr_als_data.attr,  //grace add in 2013.10.09
	&dev_attr_ps_data.attr,   //grace add in 2013.10.09
	&dev_attr_type.attr,
	NULL
};

static const struct attribute_group rpr521_attr_group = {
	.attrs = rpr521_attributes,
};
#else//xym add
static struct device_attribute rpr521_attributes[] = {
 	__ATTR(als_poll_delay, 0660, rpr521_show_als_poll_delay, rpr521_store_als_poll_delay),
 	__ATTR(enable_als_sensor, 0660, rpr521_show_enable_als_sensor, rpr521_store_enable_als_sensor),
 	__ATTR(enable_ps_sensor, 0660, rpr521_show_enable_ps_sensor, rpr521_store_enable_ps_sensor),
 	__ATTR(ps_thres_high, 0660, rpr521_show_ps_thres_high, rpr521_store_ps_thres_high),
 	__ATTR(ps_thres_low, 0660 , rpr521_show_ps_thres_low, rpr521_store_ps_thres_low),
 	__ATTR(ps_calib, 0660 , rpr521_show_ps_calib, rpr521_store_ps_calib),
 	__ATTR(als_data, 0444, rpr521_show_als_data, NULL),
 	__ATTR(ps_data, 0444, rpr521_show_ps_data, NULL),
 	__ATTR(type, 0444, rpr521_show_type, NULL),
#ifdef ZTE_RPR521_CALIBRATION //xym add
 	__ATTR(calibrate, 0664, rpr521_calibrate_show, rpr521_calibrate_store), //xym add
 	__ATTR(set_calibrate_data, 0664, NULL, rpr521_set_calibrate_data_store), //xym add
#endif
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	int err;
	for (i = 0; i < ARRAY_SIZE(rpr521_attributes); i++) {
		err = device_create_file(dev, rpr521_attributes + i);
		if (err)
			goto error;
	}
	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, rpr521_attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return err;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(rpr521_attributes); i++)
		device_remove_file(dev, rpr521_attributes + i);
	return 0;
}

#endif//xym add
/*************** Initialze Functions ******************/
static int rpr521_init_client(struct i2c_client *client)
{
    struct init_func_write_data {
        unsigned char mode_ctl;
        unsigned char psals_ctl;
        unsigned char persist;
        unsigned char reserved0;
        unsigned char reserved1;
        unsigned char reserved2;
        unsigned char reserved3;
        unsigned char reserved4;
        unsigned char reserved5;
        unsigned char intr;
        unsigned char psth_hl;
        unsigned char psth_hh;
        unsigned char psth_ll;
        unsigned char psth_lh;
        unsigned char alsth_hl;
        unsigned char alsth_hh;
        unsigned char alsth_ll;
        unsigned char alsth_lh;
    } write_data;
    int result;
    unsigned char gain;

    unsigned char mode_ctl    = PS_ALS_SET_MODE_CONTROL;
    unsigned char psals_ctl   = PS_ALS_SET_ALSPS_CONTROL;
    unsigned char persist     = PS_ALS_SET_INTR_PERSIST;
    unsigned char intr        = PS_ALS_SET_INTR;
    unsigned short psth_upper  = PS_ALS_SET_PS_TH;
    unsigned short psth_low    = PS_ALS_SET_PS_TL;
    unsigned short alsth_upper = PS_ALS_SET_ALS_TH;
    unsigned short alsth_low   = PS_ALS_SET_ALS_TL;

    struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	printk("%s: \n",__func__);

    /* execute software reset */
    result =  i2c_smbus_write_byte_data(client, REG_SYSTEMCONTROL, 0xC0);	//soft-reset
    if (result != 0) {
        return (result);
    }

    write_data.mode_ctl  = mode_ctl;
    write_data.psals_ctl = psals_ctl;
    write_data.persist   = persist;
    write_data.reserved0 = 0;
    write_data.reserved1 = 0;
    write_data.reserved2 = 0;
    write_data.reserved3 = 0;
    write_data.reserved4 = 0;
    write_data.reserved5 = 0;
    write_data.intr      = intr;
    write_data.psth_hl   = CONVERT_TO_BE(psth_upper) & MASK_CHAR;
    write_data.psth_hh   = CONVERT_TO_BE(psth_upper) >> 8;
    write_data.psth_ll   = CONVERT_TO_BE(psth_low) & MASK_CHAR;
    write_data.psth_lh   = CONVERT_TO_BE(psth_low) >> 8;
    write_data.alsth_hl  = CONVERT_TO_BE(alsth_upper) & MASK_CHAR;
    write_data.alsth_hh  = CONVERT_TO_BE(alsth_upper) >> 8;
    write_data.alsth_ll  = CONVERT_TO_BE(alsth_low) & MASK_CHAR;
    write_data.alsth_lh  = CONVERT_TO_BE(alsth_low) >> 8;
    result               = i2c_smbus_write_i2c_block_data(client, REG_MODECONTROL, sizeof(write_data), (unsigned char *)&write_data);

	if(result < 0)
	{
		printk(KERN_ERR "%s: i2c write fail. \n", __func__);
		return result;
	}

	gain = (psals_ctl & 0x3C) >> 2;	//gain setting values
	
	als_ps->enable = mode_ctl;
	als_ps->als_time = mode_table[(mode_ctl & 0xF)].ALS;
	als_ps->ps_time = mode_table[(mode_ctl & 0xF)].PS;
	als_ps->persistence = persist;
	als_ps->ps_th_l = psth_low;
	als_ps->ps_th_h = psth_upper;
	als_ps->als_th_l = alsth_low;
	als_ps->als_th_h = alsth_upper;
	als_ps->control = psals_ctl;
	als_ps->gain0 = gain_table[gain].DATA0;
	als_ps->gain1 = gain_table[gain].DATA1;
	als_ps->ledcurrent = psals_ctl & 0x03;
		
	als_ps->enable_als_sensor = 0; //grace add in 2013.10.09
	als_ps->enable_ps_sensor = 0;  //grace add in 2013.10.09

#ifdef _INIT_CALIB_
	//rpr521_calibrate(client);
	//grace modify in 2014.12.18 begin
	if ((!rpr521_calibrate(client))&&calib_status)
	{
		init_ps_high = als_ps->ps_th_h;
		init_ps_low = als_ps->ps_th_l;
		init_calib_flag =1;
	}
	else
	{
		als_ps->ps_th_h_back = als_ps->ps_th_h;
		als_ps->ps_th_l_back = als_ps->ps_th_l;
	}
	//grace modify in 2014.12.18 end
#else
	als_ps->ps_th_h_back = als_ps->ps_th_h;
	als_ps->ps_th_l_back = als_ps->ps_th_l;
#endif

    return (result);
}

#ifdef ZTE_RPR521_PINCTRL
static int rpr521_pinctrl_init(struct ALS_PS_DATA *data)
{
	struct i2c_client *client = data->client;

	data->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		dev_err(&client->dev, "Failed to get pinctrl\n");
		return PTR_ERR(data->pinctrl);
	}

	data->pin_default =
		pinctrl_lookup_state(data->pinctrl, "lpsensor_default");
	if (IS_ERR_OR_NULL(data->pin_default)) {
		dev_err(&client->dev, "Failed to look up default state\n");
		return PTR_ERR(data->pin_default);
	}

	data->pin_sleep =
		pinctrl_lookup_state(data->pinctrl, "lpsensor_sleep");
	if (IS_ERR_OR_NULL(data->pin_sleep)) {
		dev_err(&client->dev, "Failed to look up sleep state\n");
		return PTR_ERR(data->pin_sleep);
	}

	return 0;
}
#endif

static int rpr_power_on(struct ALS_PS_DATA *data, bool on)
{
	int rc;
	printk("%s on=%d.\n",__func__, on);

	if (!on)
		goto power_off;

	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}
	
#ifdef ZTE_RPR521_PINCTRL
	rc = pinctrl_select_state(data->pinctrl, data->pin_default);
	if (rc) {
		dev_err(&data->client->dev,
			"Can't select pinctrl default state\n");
	}
#endif

	return rc;

power_off:
	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
	}
	
#ifdef ZTE_RPR521_PINCTRL
	rc = pinctrl_select_state(data->pinctrl, data->pin_sleep);
	if (rc) {
		dev_err(&data->client->dev,
			"Can't select pinctrl sleep state\n");
	}
#endif

	return rc;
}
static int rpr_power_init(struct ALS_PS_DATA *data, bool on)
{
	int rc;
	printk("%s on=%d.\n",__func__, on);

	if (!on)
		goto pwr_deinit;

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FT_VTG_MIN_UV,
					   FT_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vio");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, FT_I2C_VTG_MIN_UV,
					   FT_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}

static int rpr521_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
#define ROHM_PSALS_ALSMAX (65535)
#define ROHM_PSALS_PSMAX  (4095)

	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct ALS_PS_DATA *als_ps;
	//unsigned long flags;
	struct device_node *np = client->dev.of_node;
	
	int err = 0;
	int dev_id;
	rpr521_is_boot=1;//xym add
	printk("%s: started.\n",__func__);

	//wake_lock_init(&ps_lock,WAKE_LOCK_SUSPEND,"ps wakelock");

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}

	als_ps = kzalloc(sizeof(struct ALS_PS_DATA), GFP_KERNEL);
	if (!als_ps) {
		err = -ENOMEM;
		goto exit;
	}
	als_ps->client = client;
	i2c_set_clientdata(client, als_ps);

#ifdef ZTE_RPR521_PINCTRL
	printk("%s: pinctrl init\n", __func__);	
	/* initialize pinctrl */
	err = rpr521_pinctrl_init(als_ps);
	if (err) {
		dev_err(&client->dev, "Can't initialize pinctrl\n");
			goto exit;
	}
	
	err = pinctrl_select_state(als_ps->pinctrl, als_ps->pin_sleep);
	if (err) {
		dev_err(&client->dev,
			"Can't select pinctrl sleep state\n");
		goto exit;
	}
#endif

	err = rpr_power_init(als_ps, true);
	if (err) {
		dev_err(&client->dev, "power init failed");
		goto exit_kfree;//xym add
	}
	
	err = rpr_power_on(als_ps, true);
	if (err) {
		dev_err(&client->dev, "power on failed");
		goto exit_power_uninit;//xym add
	}

//	printk("enable = %x\n", als_ps->enable);

	//check whether is rpr521 or apds9930
	dev_id = i2c_smbus_read_byte_data(client, REG_MANUFACT_ID);
	if(dev_id != 0xE0){
		//kfree(als_ps);
		printk("%s: read MANUFACT_ID error\n", __func__);
	    //return -1;
	    goto exit_power_off;
	}
	als_ps->type = 1;
	printk("%s: id(0x%x), this is rpr521!\n", __func__, dev_id);

	mutex_init(&als_ps->update_lock);

	INIT_DELAYED_WORK(&als_ps->dwork, rpr521_ps_int_work_handler);
	INIT_DELAYED_WORK(&als_ps->als_dwork, rpr521_als_polling_work_handler); 
	
//	printk("%s interrupt is hooked\n", __func__);

	/* Initialize the RPR400 chip */
	err = rpr521_init_client(client);
	if (err)
		goto exit_power_off;

	als_ps->als_poll_delay = PS_ALS_SET_MIN_DELAY_TIME;	

	/* Register to Input Device */
	als_ps->input_dev_als = input_allocate_device();
	if (!als_ps->input_dev_als) {
		err = -ENOMEM;
		printk("%s: Failed to allocate input device als\n", __func__);
		goto exit_power_off;
	}

	als_ps->input_dev_ps = input_allocate_device();
	if (!als_ps->input_dev_ps) {
		err = -ENOMEM;
		printk("%s: Failed to allocate input device ps\n", __func__);
		goto exit_free_dev_als;
	}
	
	input_set_drvdata(als_ps->input_dev_ps, als_ps); //grace modified in 2013.9.24
	input_set_drvdata(als_ps->input_dev_als, als_ps); //grace modified in 2013.9.24
	
	set_bit(EV_ABS, als_ps->input_dev_als->evbit);
	set_bit(EV_ABS, als_ps->input_dev_ps->evbit);

	input_set_abs_params(als_ps->input_dev_als, ABS_MISC, 0, ROHM_PSALS_ALSMAX, 0, 0);
	input_set_abs_params(als_ps->input_dev_ps, ABS_DISTANCE, 0, ROHM_PSALS_PSMAX, 0, 0);

	als_ps->input_dev_als->name = "light";
	als_ps->input_dev_ps->name = "proximity";
	//grace modified in 2013.3.22 begin
	als_ps->input_dev_als->id.bustype = BUS_I2C;
	als_ps->input_dev_als->dev.parent =&als_ps->client->dev;
	als_ps->input_dev_ps->id.bustype = BUS_I2C;
	als_ps->input_dev_ps->dev.parent =&als_ps->client->dev;
	//grace modified in 2013.3.22 begin


	err = input_register_device(als_ps->input_dev_als);
	if (err) {
		err = -ENOMEM;
		printk("%s: Unable to register input device als: %s\n", __func__, 
		       als_ps->input_dev_als->name);
		goto exit_free_dev_ps;
	}

	err = input_register_device(als_ps->input_dev_ps);
	if (err) {
		err = -ENOMEM;
		printk("%s: Unable to register input device ps: %s\n", __func__, 
		       als_ps->input_dev_ps->name);
		goto exit_unregister_dev_als;
	}

	/* Register sysfs hooks */
#if 0
	err = sysfs_create_group(&client->dev.kobj, &rpr521_attr_group);
	if (err)
	{
		printk("%s sysfs_create_groupX\n", __func__);
		goto exit_unregister_dev_ps;
	}
#else
	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev, "sysfs register failed\n");
		goto exit_unregister_dev_ps;
	}
#endif

	als_ps->irq_gpio = of_get_named_gpio_flags(np, "rpr,irq-gpio",
				0, &als_ps->irq_gpio_flags);
	if (als_ps->irq_gpio < 0)
		return als_ps->irq_gpio;
	
	if (gpio_is_valid(als_ps->irq_gpio)) {
		err = gpio_request(als_ps->irq_gpio, "rpr_irq_gpio");
		if (err) {
			dev_err(&client->dev, "irq gpio request failed");
		}

		err = gpio_direction_input(als_ps->irq_gpio);
		if (err) {
			dev_err(&client->dev,
				"set_direction for irq gpio failed\n");
		}
	    printk("%s: config irq_gpio %d \n", __func__, als_ps->irq_gpio);
	}
	
	printk("%s support ver. %s enabled\n", __func__, RPR0521_DRIVER_VERSION);
	if (request_irq(client->irq, rpr521_interrupt, IRQF_TRIGGER_FALLING/*IRQF_DISABLED|IRQ_TYPE_EDGE_FALLING*/,
		RPR521_DRV_NAME, (void *)client)) {
		printk("%s Could not allocate rpr521_INT !\n", __func__);	
#if 0
		//goto exit_kfree;
		goto exit_unregister_dev_ps;//xym add
#else
		goto exit_rpr521_sysfs_create;//xym add
#endif
	}

	irq_set_irq_wake(client->irq, 1);

	printk(KERN_INFO "%s: INT No. %d\n", __func__, client->irq);
#ifdef ZTE_RPR521_SENSORS_CLASS_DEV //xym add 
	/* Register to sensors class */
	als_ps->als_cdev = sensors_light_cdev;
	als_ps->als_cdev.sensors_enable = rpr521_als_sensors_enable;
	als_ps->als_cdev.sensors_poll_delay = rpr521_als_poll_delay;//NULL;

	als_ps->ps_cdev = sensors_proximity_cdev;
	als_ps->ps_cdev.sensors_enable = rpr521_ps_sensors_enable;
	als_ps->ps_cdev.sensors_poll_delay = NULL,

	err = sensors_classdev_register(&client->dev, &als_ps->als_cdev);
	if (err) {
		pr_err("%s: Unable to register to sensors class: %d\n",
				__func__, err);
		goto exit_free_irq;
	}

	err = sensors_classdev_register(&client->dev, &als_ps->ps_cdev);
	if (err) {
		pr_err("%s: Unable to register to sensors class: %d\n",
			       __func__, err);
		goto exit_rpr521_classdev_unregister;
	}
#endif
	zte_rpr521_read_all_reg(client);

	printk("%s: --- succ\n", __func__);
	return 0;

#ifdef ZTE_RPR521_SENSORS_CLASS_DEV //xym add 
exit_rpr521_classdev_unregister:
	sensors_classdev_unregister(&als_ps->als_cdev);
exit_free_irq:
	free_irq(client->irq, client);
#endif
#if 1//xym add
exit_rpr521_sysfs_create:
	remove_sysfs_interfaces(&client->dev);
#endif
exit_unregister_dev_ps:
	input_unregister_device(als_ps->input_dev_ps);	
exit_unregister_dev_als:
	printk("%s exit_unregister_dev_als:\n", __func__);
	input_unregister_device(als_ps->input_dev_als);
exit_free_dev_ps:
	input_free_device(als_ps->input_dev_ps);
exit_free_dev_als:
	input_free_device(als_ps->input_dev_als);
exit_power_off:
	rpr_power_on(als_ps, false);
exit_power_uninit:
	rpr_power_init(als_ps, false);
exit_kfree:
	kfree(als_ps);
exit:
	printk("%s: --- exit with error %d\n", __func__, err);
	return err;

#undef ROHM_PSALS_ALSMAX
#undef ROHM_PSALS_PSMAX
}

static int rpr521_remove(struct i2c_client *client)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	
	input_unregister_device(als_ps->input_dev_als);
	input_unregister_device(als_ps->input_dev_ps);
	
	input_free_device(als_ps->input_dev_als);
	input_free_device(als_ps->input_dev_ps);

	free_irq(client->irq, client);
#if 0 //xym
	sysfs_remove_group(&client->dev.kobj, &rpr521_attr_group);
#else //xym
	remove_sysfs_interfaces(&client->dev);
#endif
	/* Power down the device */
	rpr521_set_measurement_time(client, 0);

	kfree(als_ps);

	return 0;
}

#if 0//xym add
static int rpr521_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	printk("%s\n", __func__);
#if 1//xym add
	//接近关闭时才关LDO下电
	if(als_ps->enable_ps_sensor == 0)
	{
		rpr_power_on(als_ps, false);
		printk("%s: LDO power off\n", __func__);		
	}
	return 0;
#else
	disable_irq(client->irq);
	//wake_unlock(&ps_lock);
	//return rpr521_set_measurement_time(client, 0);
	return rpr521_set_measurement_time(client, PS_ALS_SET_MODE_CONTROL);//grace modify in 2014.5.7
#endif
}

static int rpr521_resume(struct i2c_client *client)
{
	struct ALS_PS_DATA *als_ps = i2c_get_clientdata(client);
	printk("%s \n", __func__);
#if 1//xym add
	//接近关闭时才恢复LDO上电,再次初始化register
	if(als_ps->enable_ps_sensor == 0)
	{
		printk("%s: ready to power on LDO\n", __func__);
		rpr_power_on(als_ps, true);
	}
	return 0;
#else
	int tmp = PS_ALS_SET_MODE_CONTROL;	
	
	enable_irq(client->irq);
	if(als_ps->enable_als_sensor == 1)
		tmp = tmp | ALS_EN;
	if(als_ps->enable_ps_sensor == 1)	
		tmp = tmp | PS_EN;
	return rpr521_set_measurement_time(client, tmp);

#endif
}
#endif

MODULE_DEVICE_TABLE(i2c, rpr521_id);

static const struct i2c_device_id rpr521_id[] = {
	{ "rpr521", 0 },
	{ }
};
#ifdef CONFIG_OF
static struct of_device_id rpr_match_table[] = {
                { .compatible = "rohm,rpr0521",},
                { },
        };
#else
#define rpr_match_table NULL
#endif 

static struct i2c_driver rpr521_driver = {
	.driver = {
		.name	= RPR521_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table =rpr_match_table,
	},
	//.suspend = rpr521_suspend,  //xym deleted
	//.resume	= rpr521_resume,   //xym deleted
	.probe	= rpr521_probe,
	.remove	= rpr521_remove,
	.id_table = rpr521_id,
};

static int __init rpr521_init(void)
{
	printk("%s: \n",__func__);
	return i2c_add_driver(&rpr521_driver);
}

static void __exit rpr521_exit(void)
{
	i2c_del_driver(&rpr521_driver);
}

MODULE_AUTHOR("Andy Mi @ ROHM");
MODULE_DESCRIPTION("rpr521 ambient light + proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(RPR0521_DRIVER_VERSION);

module_init(rpr521_init);
module_exit(rpr521_exit);
