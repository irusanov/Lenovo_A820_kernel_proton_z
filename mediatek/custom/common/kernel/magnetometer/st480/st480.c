/* 
 * Copyright (C) 2012 Senodia Corporation.
 *
 * Author: Tori Xu <tori.xz.xu@gmail.com,xuezhi_xu@senodia.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/************
MTK Platform
*************/

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/time.h>
#include <linux/hrtimer.h>

#ifdef MT6575
#include <mach/mt6575_devs.h>
#include <mach/mt6575_typedefs.h>
#include <mach/mt6575_gpio.h>
#include <mach/mt6575_pm_ldo.h>
#endif

#ifdef MT6577
#include <mach/mt6577_devs.h>
#include <mach/mt6577_typedefs.h>
#include <mach/mt6577_gpio.h>
#include <mach/mt6577_pm_ldo.h>
#endif

#ifdef MT6589
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>


#include <cust_mag.h>
#include "st480.h"
#include <linux/hwmsen_helper.h>
 
//#define MSENSOR_AUTO_DETE

#ifndef ST480_BURST_MODE
#ifdef SENSOR_AUTO_TEST
#include <linux/kthread.h>
#endif
#endif

//#define SUPPORT_I2C_9_BYTES

#ifdef ACCELEROMETER_CONTROLL
#define SENSOR_DATA_SIZE 9
#else
#define SENSOR_DATA_SIZE 6
#endif	

#ifdef ST480_BURST_MODE
#ifdef SUPPORT_I2C_9_BYTES
#define BURST_MODE 0x1F
#else
#define BURST_MODE 0x1E
#endif
#define BURST_RATE (0x01<<2)
#endif

#ifdef SUPPORT_I2C_9_BYTES
#define SINGLE_MEASUREMENT_MODE 0x3F
#define READ_MEASUREMENT 0x4F
#else
#define SINGLE_MEASUREMENT_MODE 0x3E
#define READ_MEASUREMENT 0x4E
#endif
#define WRITE_REGISTER 0x60
#define CALIBRATION_REG (0x02<<2)
/*****************************debug setting******************************/
#define SENODIA_DEBUG_MSG	0
#define SENODIA_DEBUG_FUNC	0
#define SENODIA_DEBUG_DATA	0
#define MAX_FAILURE_COUNT	3
#define SENODIA_RETRY_COUNT	10
#define SENODIA_DEFAULT_DELAY	50

#if SENODIA_DEBUG_MSG
#define SENODIADBG(format, ...)	printk(KERN_INFO "st480:SENODIA " format "\n", ## __VA_ARGS__)
#else
#define SENODIADBG(format, ...)
#endif

#if SENODIA_DEBUG_FUNC
#define SENODIAFUNC(func) printk(KERN_INFO "st480:SENODIA " func " is called\n")
#else
#define SENODIAFUNC(func)
#endif


int dg_cnt=0;



#ifdef MTK_AUTO_DETECT_MAGNETOMETER
/* Add for auto detect feature */
extern struct mag_hw* st480_get_cust_mag_hw(void); 
static int  st480_local_init(void);
static int senodia_remove(void);
static int st480_init_flag =0;
static struct sensor_init_info st480_init_info = {		
	.name = "st480",		
	.init = st480_local_init,		
	.uninit = senodia_remove,	
};
#endif

/*******************************i2c setting*********************************/
#define ST480_READ_DATA
#define SENODIA_SMBUS_READ_BYTE_BLOCK
#define SENODIA_SMBUS_READ_BYTE
/*----------------------------------------------------------------------------*/

static int st480_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int st480_i2c_remove(struct i2c_client *client);
static int st480_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
#ifndef MTK_AUTO_DETECT_MAGNETOMETER
static int senodia_probe(struct platform_device *pdev);
static int senodia_remove(struct platform_device *pdev);
#endif

/*----------------------------------------------------------------------------*/
struct senodia_data {
	struct i2c_client *client; 
	struct mag_hw *hw; 
	atomic_t layout;   
	atomic_t trace;
	struct hwmsen_convert   cvt;
#if defined(CONFIG_HAS_EARLYSUSPEND)  
	struct early_suspend senodia_early_suspend;
#endif
};

static struct senodia_data *senodia;
static struct i2c_client *this_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id st480_i2c_id[] = {{SENODIA_I2C_NAME,0},{}};


#ifdef MTK_AUTO_DETECT_MAGNETOMETER
#define ST480_I2C_ADDRESS 	0x18	// CAD0 = 0, CAD1 =0
/*the adapter id will be available in customization*/
//static unsigned short st480_force[] = {0x00, (ST480_I2C_ADDRESS>>1), I2C_CLIENT_END, I2C_CLIENT_END};
static const unsigned short normal_i2c[] = { 0x0c, I2C_CLIENT_END };
static unsigned short st480_force[] = {(ST480_I2C_ADDRESS>>1), I2C_CLIENT_END, I2C_CLIENT_END};
static const unsigned short *const st480_forces[] = { st480_force, NULL };
//static struct i2c_client_address_data st480_addr_data = { .forces = st480_forces,};
#endif



#if defined(ACER_C11)
static struct i2c_board_info __initdata i2c_st480={ I2C_BOARD_INFO("st480", (0X18>>1))};
#else
xx
static struct i2c_board_info __initdata i2c_st480={ I2C_BOARD_INFO("st480", (0X1e>>1))};
#endif
/*----------------------------------------------------------------------------*/
static struct i2c_driver st480_i2c_driver = {
	//.class		= I2C_CLASS_HWMON,
    .driver = {
#ifdef MTK_AUTO_DETECT_MAGNETOMETER
        .owner = THIS_MODULE, 
#endif
        .name  = SENODIA_I2C_NAME,
    },
	.probe      = st480_i2c_probe,
	.remove     = st480_i2c_remove,
	.detect     = st480_i2c_detect,
#if !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend    = st480_suspend,
	.resume     = st480_resume,
#endif 
	.id_table = st480_i2c_id,
#ifdef MTK_AUTO_DETECT_MAGNETOMETER
    //.address_list = (const unsigned short *) st480_forces,
	//.address_list	= normal_i2c,
#endif
};

/*----------------------------------------------------------------------------*/
#ifndef MTK_AUTO_DETECT_MAGNETOMETER
static struct platform_driver senodia_sensor_driver = {
	.probe      = senodia_probe,
	.remove     = senodia_remove,    
	.driver     = {
		.name  = "msensor",
	}
};
#endif

/*----------------------------------------------------------------------------*/
/* Addresses to scan -- protected by sense_data_mutex */
volatile static int sense_data[SENSOR_DATA_SIZE];
static struct mutex sense_data_mutex;
static struct mutex sensor_data_mutex;

static atomic_t m_flag;
static atomic_t o_flag ;

static atomic_t open_count;
static atomic_t open_flag;
static atomic_t dev_open_count;

static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);
static DECLARE_WAIT_QUEUE_HEAD(open_wq);

volatile static short senodiad_delay = SENODIA_DEFAULT_DELAY;

struct mag_3{
	s16  mag_x,
	mag_y,
	mag_z;
};
volatile static struct mag_3 mag;

#ifdef ST480_READ_DATA
static int st480_i2c_read_data(struct i2c_client *client, int len, char *buf, int length)
{
        struct i2c_msg msgs[] = {
                {
                        .addr  =  client->addr,
                        .flags  =  0,
                        .len  =  len,
                        .buf  =  buf,
						.timing = 100,
                },
                {
                        .addr  =  client->addr,
                        .flags  = I2C_M_RD,
                        .len  =  length,
                        .buf  =  buf,
						.timing = 100,
                },
        };

        if(i2c_transfer(client->adapter, msgs, 2) < 0){
                pr_err("megnetic_i2c_read_data: transfer error\n");
                return EIO;
        }
        else
                return 0;
}
#endif

#ifdef SENODIA_SMBUS_READ_WRITE_WORD
static int magnetic_smbus_read_word_data(struct i2c_client *client,
	unsigned char reg_addr, unsigned char *data, unsigned char length)
{
	s32 dummy;
	dummy = i2c_smbus_read_word_data(client, reg_addr);
	if (dummy < 0)
                return -EPERM;

	*data = dummy & 0x00ff;
	*(data+1) = (dummy & 0xff00) >> 8;
	
	return 0;		
}

static int magnetic_smbus_write_word_data(struct i2c_client *client,
	unsigned char reg_addr, unsigned char *data, unsigned char length)
{
	s32 dummy;
	u16 value = (*(data+1) << 8) | (*(data));
	dummy = i2c_smbus_write_word_data(client, reg_addr, value);
	if (dummy < 0)
	{
		printk("magnetic write word data error!\n");
                return -EPERM;
	}
        return 0;
}
#endif

#ifdef SENODIA_SMBUS_READ_BYTE
static int magnetic_smbus_read_byte(struct i2c_client *client,
                        unsigned char reg_addr, unsigned char *data)
{
        s32 dummy;
        dummy = i2c_smbus_read_byte_data(client, reg_addr);
        if (dummy < 0)
                return -EPERM;
        *data = dummy & 0x000000ff;

        return 0;
}
#endif

#ifdef SENODIA_SMBUS_WRITE_BYTE
static int magnetic_smbus_write_byte(struct i2c_client *client,
                        unsigned char reg_addr, unsigned char *data)
{
        s32 dummy;
        dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
        if (dummy < 0)
                return -EPERM;
        return 0;
}
#endif

#ifdef SENODIA_SMBUS_READ_BYTE_BLOCK
static int magnetic_smbus_read_byte_block(struct i2c_client *client,
                unsigned char reg_addr, unsigned char *data, unsigned char len)
{
        s32 dummy;
        dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
        if (dummy < 0)
                return -EPERM;
        return 0;
}
#endif

#ifdef SENODIA_SMBUS_WRITE_BYTE_BLOCK
static int magnetic_smbus_write_byte_block(struct i2c_client *client,
                unsigned char reg_addr, unsigned char *data, unsigned char len)
{
        s32 dummy;
        dummy = i2c_smbus_write_i2c_block_data(client, reg_addr, len, data);
        if (dummy < 0)
                return -EPERM;
        return 0;
}
#endif

#ifdef SENODIA_I2C_READ
static int magnetic_i2c_read_data(struct i2c_client *client, char *buf, int length)
{
        struct i2c_msg msgs[] = {
                {
                        .addr  =  client->addr,
                        .flags  =  0,
                        .len  =  1,
                        .buf  =  buf,
						.timing = 100,
                },
                {
                        .addr  =  client->addr,
                        .flags  = I2C_M_RD,
                        .len  =  length,
                        .buf  =  buf,
						.timing = 100,
                },
        };

        if(i2c_transfer(client->adapter, msgs, 2) < 0){
                pr_err("megnetic_i2c_read_data: transfer error\n");
                return EIO;
        }
        else
		return 0;
}
#endif

#ifdef SENODIA_I2C_WRITE
static int magnetic_i2c_write_data(struct i2c_client *client, char *buf, int length)
{
        struct i2c_msg msgs[] = {
                {
                        .addr = client->addr,
                        .flags = 0,
                        .len = length,
                        .buf = buf,
						.timing = 100,
                },
        };

        if (i2c_transfer(client->adapter, msgs, 1) < 0) {
        #ifdef SENODIA_DEBUG      
		pr_err("megnetic_i2c_write_data: transfer error\n");
	#endif
                return -EIO;
        } else
                return 0;
}
#endif


static void senodia_power(struct mag_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	if(hw->power_id != MT65XX_POWER_NONE)
	{        
		if(power_on == on)
		{
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "mmc328x")) 
			{
				printk(KERN_ERR "power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "mmc328x")) 
			{
				printk(KERN_ERR "power off fail!!\n");
			}
		}
	}
	power_on = on;
}


static int senodia_SetPowerMode(struct i2c_client *client, bool enable)
{
	return 0;
}


int st480_init(struct i2c_client *client)
{
	int i;
	char buf[5];
	char data[1];

	memset(buf, 0, 5);
	memset(data, 0, 1);

dg_cnt = 0x55aa;

				printk("st480_init !\n");

//init register step 1
	buf[0] = WRITE_REGISTER;
	buf[1] = 0x00;
        buf[2] = 0x7C;
        buf[3] = 0x00;  
        i=0;
	while(st480_i2c_read_data(client, 4, buf, 1)!=0)
        {
				printk("error!--------------------->one!\n");
dg_cnt = dg_cnt|1;
                i++;
              //punk  msleep(1);
                if(st480_i2c_read_data(client, 4, buf, 1)==0)
                {
                        break;
                }
                if(i>3)
                {
                        return -EIO;
                }
        }


		printk("success!--------------------->one!\n");
//init register step 2
	buf[0] = WRITE_REGISTER;
	buf[1] = 0x00;
        buf[2] = 0x00;
        buf[3] = 0x08;
        i=0;
	while(st480_i2c_read_data(client, 4, buf, 1)!=0)
        {
				
dg_cnt = dg_cnt|2;
				printk("error!--------------------->two!\n");
                i++;
                msleep(1);
                if(st480_i2c_read_data(client, 4, buf, 1)==0)
                {
                        break;
                }
                if(i>3)
                {
                        return -EIO;
                }
        }

				printk("success!--------------------->two!\n");
//set calibration register
	buf[0] = WRITE_REGISTER;
	buf[1] = 0x00;
        buf[2] = 0x1c;
        buf[3] = CALIBRATION_REG;
        i=0;
	while(st480_i2c_read_data(client, 4, buf, 1)!=0)
        {	
		
dg_cnt = dg_cnt|4;
                i++;
                msleep(1);
                if(st480_i2c_read_data(client, 4, buf, 1)==0)
                {
                        break;
                }
                if(i>3)
                {
                        return -EIO;
                }
        }

//set mode config	
#ifdef ST480_BURST_MODE
	buf[0] = WRITE_REGISTER;
	buf[1] = 0x00;
        buf[2] = 0x01; 
        buf[3] = BURST_RATE;
        i=0;
	while(st480_i2c_read_data(client, 4, buf, 1)!=0)
        {
dg_cnt = dg_cnt|8;
                i++;
                msleep(1);
                if(st480_i2c_read_data(client, 4, buf, 1)==0)
                {
                        break;
                }
                if(i>3)
                {
                        return -EIO;
                }
        }
#else
	i=0;
        while((magnetic_smbus_read_byte(senodia->client, SINGLE_MEASUREMENT_MODE, data)!=0))
        {
		printk("Set single measurement mode error!  -1\n");
                i++;
                if(magnetic_smbus_read_byte(senodia->client, SINGLE_MEASUREMENT_MODE, data)==0)
                        break;
                if(i>3)
                {
                        return -EIO;
                }
        }
#endif

	return 0;
}

static void senodia_work_func(void)
{
	char buffer[10];
	int ret;
        char data[1];

        memset(data, 0, 1);
	memset(buffer, 0, 10);

	ret=0;

#if 1
	buffer[0] = READ_MEASUREMENT;
#ifdef SUPPORT_I2C_9_BYTES
	while(st480_i2c_read_data(senodia->client, 1, buffer, 9)!=0)
	{
		printk("------------------>read data error!\n");
		ret++;

        if(st480_i2c_read_data(senodia->client, 1, buffer, 9)==0)
        {
            break;
        }
        if(ret>3)
        {
            break;
        }
    }
#else
	while(st480_i2c_read_data(senodia->client, 1, buffer, 7)!=0)
	{
		printk("------------------>read data error!\n");
		ret++;

        if(st480_i2c_read_data(senodia->client, 1, buffer, 7)==0)
        {
            break;
        }
        if(ret>3)
        {
            break;
        }
        }
#endif
#else
	while((magnetic_smbus_read_byte_block(senodia->client, READ_MEASUREMENT, buffer, 9)!=0))
	{
		ret++;

		if(magnetic_smbus_read_byte_block(senodia->client, READ_MEASUREMENT, buffer, 9)==0)
		{
			break;
		}
		if(ret>=3)
		{
			break;
		}
	}

#endif


	printk("st480, buf[0]=%x, buf[1]=%x,buf[2]=%x,buf[3]=%x\n",buffer[0],buffer[1],buffer[2],buffer[3]);
	printk("st480, buf[4]=%x, buf[5]=%x,buf[6]=%x,buf[7]=%x\n",buffer[4],buffer[5],buffer[6],buffer[7]);


	if(!((buffer[0]>>4) & 0X01))
	{
		if(SENSOR_SIZE_3X3_QFN)
		{
		#if defined (CONFIG_ST480_BOARD_LOCATION_FRONT)
			#if defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_0)
#ifdef SUPPORT_I2C_9_BYTES
			xxx
				mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                		mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                		mag.mag_z = (buffer[7]<<8)|buffer[8];
#else
				mag.mag_x = (-1)*((buffer[1]<<8)|buffer[2]);
                		mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
                		mag.mag_z = (buffer[5]<<8)|buffer[6];
#endif
			#elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_90)
				mag.mag_x = (-1)*((buffer[5]<<8)|buffer[6]);
               			mag.mag_y = (buffer[3]<<8)|buffer[4];
                		mag.mag_z = (buffer[7]<<8)|buffer[8];
			#elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_180)
						mag.mag_x = (buffer[3]<<8)|buffer[4];
                		mag.mag_y = (buffer[5]<<8)|buffer[6];
                		mag.mag_z = (buffer[7]<<8)|buffer[8];
			#elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_270)
				mag.mag_x = (buffer[5]<<8)|buffer[6];
                		mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
                		mag.mag_z = (buffer[7]<<8)|buffer[8];
			#endif
		#elif defined (CONFIG_ST480_BOARD_LOCATION_BACK)
			#if defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_0)
				mag.mag_x = (buffer[3]<<8)|buffer[4];
                		mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                		mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
        		#elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_90)
				mag.mag_x = (buffer[5]<<8)|buffer[6];
                		mag.mag_y = (buffer[3]<<8)|buffer[4];
                		mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
        		#elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_180)
				mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                		mag.mag_y = (buffer[5]<<8)|buffer[6];
                		mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
        		#elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_270)
				mag.mag_x = (-1)*((buffer[5]<<8)|buffer[6]);
                		mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
                		mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
        		#endif
		#endif	
		}
		else if (SENSOR_SIZE_2X2_BGA)
		{
                #if defined (CONFIG_ST480_BOARD_LOCATION_FRONT)
                        #if defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_0)
#ifdef SUPPORT_I2C_9_BYTES
                                mag.mag_x = (buffer[3]<<8)|buffer[4];
                                mag.mag_y = (buffer[5]<<8)|buffer[6];
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
#else
                                mag.mag_x = (buffer[1]<<8)|buffer[2];
                                mag.mag_y = (buffer[3]<<8)|buffer[4];
                                mag.mag_z = (buffer[5]<<8)|buffer[6];
#endif
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_90)
                                mag.mag_x = (buffer[5]<<8)|buffer[6];
                                mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_180)
                                mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_270)
                                mag.mag_x = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_y = (buffer[3]<<8)|buffer[4];
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #endif
                #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK)
                        #if defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_0)
				mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_y = (buffer[5]<<8)|buffer[6];
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_90)
				mag.mag_x = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_180)
				mag.mag_x = (buffer[3]<<8)|buffer[4];
                                mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_270)
				mag.mag_x = (buffer[5]<<8)|buffer[6];
                                mag.mag_y = (buffer[3]<<8)|buffer[4];
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #endif
                #endif
		}
		else if (SENSOR_SIZE_1_6X1_6_LGA)
                {
                #if defined (CONFIG_ST480_BOARD_LOCATION_FRONT)
                        #if defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_0)
#ifdef SUPPORT_I2C_9_BYTES
                                mag.mag_x = (buffer[5]<<8)|buffer[6];
                                mag.mag				mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                		mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                		mag.mag_z = (buffer[7]<<8)|buffer[8];
_y = (buffer[3]<<8)|buffer[4];
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
#else
		printk("------------SENSOR_SIZE_1_6X1_6_LGA, front, degree 0------>\n");
                                mag.mag_x = (buffer[3]<<8)|buffer[4];
                                mag.mag_y = (buffer[1]<<8)|buffer[2];
                                mag.mag_z = (-1)*((buffer[5]<<8)|buffer[6]);
#endif
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_90)
                                mag.mag_x = (buffer[3]<<8)|buffer[4];
                                mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_180)
#ifdef SUPPORT_I2C_9_BYTES
                                mag.mag_x = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
#else
		printk("------------SENSOR_SIZE_1_6X1_6_LGA, front, degree 180------>\n");
                                mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_y = (-1)*((buffer[1]<<8)|buffer[2]);
                                mag.mag_z = (-1)*((buffer[5]<<8)|buffer[6]);
#endif
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_270)
                                mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_y = (buffer[5]<<8)|buffer[6];
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #endif
                #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK)
                        #if defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_0)
                                mag.mag_x = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_y = (buffer[3]<<8)|buffer[4];
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_90)
                                mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_180)
                                mag.mag_x = (buffer[5]<<8)|buffer[6];
                                mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_270)
                                mag.mag_x = (buffer[3]<<8)|buffer[4];
                                mag.mag_y = (buffer[5]<<8)|buffer[6];
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #endif
                #endif
                }
		else if(SENSOR_SIZE_1_6X1_6_BGA)
		{
		#if defined (CONFIG_ST480_BOARD_LOCATION_FRONT)
                        #if defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_0)
                                mag.mag_x = (buffer[5]<<8)|buffer[6];
                                mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_90)
                                mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_180)
                                mag.mag_x = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_y = (buffer[3]<<8)|buffer[4];
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_FRONT_DEGREE_270)
                                mag.mag_x = (buffer[3]<<8)|buffer[4];
                                mag.mag_y = (buffer[5]<<8)|buffer[6];
                                mag.mag_z = (buffer[7]<<8)|buffer[8];
                        #endif
                #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK)
                        #if defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_0)
                                mag.mag_x = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_y = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_90)
                                mag.mag_x = (buffer[3]<<8)|buffer[4];
                                mag.mag_y = (-1)*((buffer[5]<<8)|buffer[6]);
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_180)
                                mag.mag_x = (buffer[5]<<8)|buffer[6];
                                mag.mag_y = (buffer[3]<<8)|buffer[4];
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #elif defined (CONFIG_ST480_BOARD_LOCATION_BACK_DEGREE_270)
                                mag.mag_x = (-1)*((buffer[3]<<8)|buffer[4]);
                                mag.mag_y = (buffer[5]<<8)|buffer[6];
                                mag.mag_z = (-1)*((buffer[7]<<8)|buffer[8]);
                        #endif
                #endif
		}	
#ifdef SUPPORT_I2C_9_BYTES
		if( ((buffer[1]<<8)|(buffer[2])) > 46244)
		{
			mag.mag_x = mag.mag_x * (1 + (70/128/4096) * (((buffer[1]<<8)|(buffer[2])) - 46244));
			mag.mag_y = mag.mag_y * (1 + (70/128/4096) * (((buffer[1]<<8)|(buffer[2])) - 46244));
			mag.mag_z = mag.mag_z * (1 + (70/128/4096) * (((buffer[1]<<8)|(buffer[2])) - 46244));
		} 
		else if( ((buffer[1]<<8)|(buffer[2])) < 46244)
		{
			mag.mag_x = mag.mag_x * (1 + (60/128/4096) * (((buffer[1]<<8)|(buffer[2])) - 46244));
			mag.mag_y = mag.mag_y * (1 + (60/128/4096) * (((buffer[1]<<8)|(buffer[2])) - 46244));
			mag.mag_z = mag.mag_z * (1 + (60/128/4096) * (((buffer[1]<<8)|(buffer[2])) - 46244));
		}
#endif

//#ifdef SENODIA_DEBUG
	printk("mag_x = %d, mag_y = %d, mag_z = %d\n",mag.mag_x,mag.mag_y,mag.mag_z);
//#endif	
	}

    ret=0;
    while((magnetic_smbus_read_byte(senodia->client, SINGLE_MEASUREMENT_MODE, data)!=0))
	{
		printk("Set single measurement mode error!\n");
		ret++;
		if(magnetic_smbus_read_byte(senodia->client, SINGLE_MEASUREMENT_MODE, data)==0)
			break;
		if(ret>3)
		{
			break;
		}
	}

#ifndef ACCELEROMETER_CONTROLL
	mag_acc.acc_x = acc[0];
	mag_acc.acc_y = acc[1];
	mag_acc.acc_z = acc[2];
	mag_acc.mag_x = mag.mag_x;
	mag_acc.mag_y = mag.mag_y;
	mag_acc.mag_z = mag.mag_z;
#endif

#ifdef ST480_BURST_MODE
	enable_irq(senodia->client->irq);
#endif
	printk("work_func!");
}

static void ecs_closedone(void)
{
	SENODIADBG("enter %s\n", __func__);
	atomic_set(&m_flag, 0);
	atomic_set(&o_flag , 0);
}

static int st480_GetOpenStatus(void)
{
	return atomic_read(&open_flag);
}

//=======================================================================
#define SENODIA_BUFSIZE		0x20
static ssize_t show_daemon_name(struct device_driver *ddri, char *buf)
{
	char strbuf[SENODIA_BUFSIZE];
	sprintf(strbuf, "st480");
	return sprintf(buf, "%s", strbuf);		
}

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	char strbuf[SENODIA_BUFSIZE];
	sprintf(strbuf, "st480");
	return sprintf(buf, "%s\n", strbuf);        
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(daemon,      S_IRUGO, show_daemon_name, NULL);
static DRIVER_ATTR(chipinfo,    S_IRUGO, show_chipinfo_value, NULL);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *st480_attr_list[] = {
	&driver_attr_daemon,
	&driver_attr_chipinfo,
};
/*----------------------------------------------------------------------------*/
static int st480_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(st480_attr_list)/sizeof(st480_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(err = driver_create_file(driver, st480_attr_list[idx]))
		{            
			printk(KERN_ERR "driver_create_file (%s) = %d\n", st480_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int st480_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(st480_attr_list)/sizeof(st480_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, st480_attr_list[idx]);
	}
	

	return err;
}

/***** senodiad functions ********************************************/
static int senodiad_open(struct inode *inode, struct file *file)
{
	SENODIAFUNC("senodiad_open");
	return nonseekable_open(inode, file);
}

static int senodiad_release(struct inode *inode, struct file *file)
{
	SENODIAFUNC("senodiad_release");
	return 0;
}

static long senodiad_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int senser_data[SENSOR_DATA_SIZE];
	int i;
	int status=0;
	int delay;

	memset(senser_data, 0, SENSOR_DATA_SIZE);
	
	switch (cmd) {
		case IOCTL_SENSOR_GET_DATA_MAG:	
			senodia_work_func();
			if(copy_to_user(argp, (void *)&mag,sizeof(mag))!=0)
                        {
                                printk("copy to user error.\n");
                                return -EPERM;
                        }
                        break;
	
		case IOCTL_SENSOR_WRITE_DATA_COMPASS:
		#ifdef SENODIA_DEBUG
			printk("[st480]IOCTL_SENSOR_WRITE_DATA_COMPASS!\n");
		#endif
			if(copy_from_user((void *)&senser_data, argp, (sizeof(int)*9))!=0)
                        {
                                printk("copy from user error.\n");
                                return -EPERM;
			}
			 
			for (i=0; i<SENSOR_DATA_SIZE; i++)
			{
				sense_data[i] = senser_data[i];
			}
                        break;

		case IOCTL_SENSOR_GET_ACC_FLAG:	
			status = atomic_read(&o_flag);
			if(copy_to_user(argp, &status, sizeof(status)))
			{
				printk("copy to user error.");
				return -EFAULT;
			}			
			break;

		case IOCTL_SENSOR_GET_COMPASS_FLAG:
			status = atomic_read(&open_flag);
			if(copy_to_user(argp, &status, sizeof(status)))
			{
				printk("copy to user error.");
				return -EFAULT;
			}
			break;
 
		case IOCTL_SENSOR_GET_COMPASS_DELAY:
			if(copy_to_user(argp, (void *)&senodiad_delay, sizeof(senodiad_delay))!=0)
                        {
                                printk("copy to user error.\n");
                                return -EPERM;
                        }
			break;
		}
	return 0; 
}

/*********************************************/
static struct file_operations senodiad_fops = {
	.open = senodiad_open,
	.release = senodiad_release,
	.unlocked_ioctl = senodiad_ioctl,
};

static struct miscdevice senodiad_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "senodia_dev",  
	.fops = &senodiad_fops,
};


/*********************************************/
/*----------------------------------------------------------------------------*/
static int st480_open(struct inode *inode, struct file *file)
{    
	struct senodia_data *obj = i2c_get_clientdata(this_client);    
	int ret = -1;	
	atomic_inc(&dev_open_count);
	if(atomic_read(&obj->trace) )
	{
		SENODIAFUNC("Open device node:st480\n");
	}
	ret = nonseekable_open(inode, file);
	
	return ret;
}
/*----------------------------------------------------------------------------*/
static int st480_release(struct inode *inode, struct file *file)
{
	struct senodia_data *obj = i2c_get_clientdata(this_client);
	atomic_dec(&dev_open_count);
	if(atomic_read(&obj->trace) )
	{
		SENODIAFUNC("Release device node:st480\n");
	}	
	return 0;
}
/*----------------------------------------------------------------------------*/
static long st480_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	int layout[3];
	void __user *argp = (void __user *)arg;
	struct i2c_client *client = this_client;
	struct senodia_data *data = i2c_get_clientdata(client);

	switch (cmd)
	{
		case MSENSOR_IOCTL_READ_CHIPINFO:
			break;

		case MSENSOR_IOCTL_READ_SENSORDATA:			
			break;
			
        	case MSENSOR_IOCTL_SENSOR_ENABLE:
			break;
			
		case MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:			
			break;

		case ECS_IOCTL_GET_LAYOUT:
			break;

		default:
			break;		
	}

	return 0; 
}
/*----------------------------------------------------------------------------*/
static struct file_operations st480_fops = {
	.open = st480_open,
	.release = st480_release,
	.unlocked_ioctl = st480_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice st480_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "msensor",
	.fops = &st480_fops,
};
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
int st480_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* msensor_data;

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR "Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value <= 50)
				{
					senodiad_delay = 50;
				}
				senodiad_delay = value;
			}	
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR "Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;

				if(value == 1)
				{
					atomic_set(&m_flag, 1);
					atomic_set(&open_flag, 1);
				}
				else
				{
					atomic_set(&m_flag, 0);
					if(atomic_read(&o_flag ) == 0)
					{
						atomic_set(&open_flag, 0);
					}
				}				
				// TODO: turn device into standby or normal mode
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				printk(KERN_ERR "get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				msensor_data = (hwm_sensor_data *)buff_out;
				
				msensor_data->values[0] = sense_data[0];
				msensor_data->values[1] = sense_data[1];
				msensor_data->values[2] = sense_data[2];
				msensor_data->status = SENSOR_STATUS_ACCURACY_HIGH;
				msensor_data->value_divide = 1000;
			}
			break;
		default:
			printk(KERN_ERR "msensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

/*----------------------------------------------------------------------------*/
int st480_orientation_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* osensor_data;

	switch (command)
	{
		case SENSOR_DELAY:

			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR "Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value <= 50)
				{
					senodiad_delay = 50;
				}
				senodiad_delay = value;
			}	
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR "Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value == 1)
				{
					atomic_set(&o_flag , 1);
					atomic_set(&open_flag, 1);
				}
				else
				{
					atomic_set(&o_flag , 0);
					if(atomic_read(&m_flag) == 0)
					{
						atomic_set(&open_flag, 0);
					}									
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				printk(KERN_ERR "get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				osensor_data = (hwm_sensor_data *)buff_out;
				
				osensor_data->values[0] = sense_data[6];
				osensor_data->values[1] = sense_data[7];
				osensor_data->values[2] = sense_data[8];
				osensor_data->status = SENSOR_STATUS_ACCURACY_HIGH;
				osensor_data->value_divide = 1000;
			}
			break;
		default:
			printk(KERN_ERR "gsensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

/*----------------------------------------------------------------------------*/
#ifndef	CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int st480_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct senodia_data *obj = i2c_get_clientdata(client)
	SENODIAFUNC("st480_suspend");

#ifdef ST480_BURST_MODE
	disable_irq(senodia->client->irq);
#endif
	if(msg.event == PM_EVENT_SUSPEND)
	{
		senodia_power(obj->hw, 0);
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int st480_resume(struct i2c_client *client)
{
	struct senodia_data *obj = i2c_get_clientdata(client)
	SENODIAFUNC("st480_resume");
	senodia_power(obj->hw, 1);

#ifdef ST480_BURST_MODE
	enable_irq(senodia->client->irq);
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void senodia_early_suspend(struct early_suspend *handler)
{
	struct senodia_data *obj = container_of(handler, struct senodia_data, senodia_early_suspend);   
	SENODIAFUNC("senodia_early_suspend");
	if(NULL == obj)
	{
		printk(KERN_ERR "null pointer!!\n");
		return;
	}
	
	if(senodia_SetPowerMode(obj->client, false))
	{
		printk("st480: write power control fail!!\n");
		return;
	}
}
/*----------------------------------------------------------------------------*/
static void senodia_early_resume(struct early_suspend *handler)
{
	struct senodia_data *obj = container_of(handler, struct senodia_data, senodia_early_suspend);         
	SENODIAFUNC("senodia_early_resume");
	if(NULL == obj)
	{
		printk(KERN_ERR "null pointer!!\n");
		return;
	}

	senodia_SetPowerMode(obj->client, true);
}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/
/*----------------------------------------------------------------------------*/
static int st480_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
{    
	strcpy(info->type, SENODIA_I2C_NAME);
	return 0;
}

/*----------------------------------------------------------------------------*/
#ifdef SENSOR_AUTO_TEST
static int sensor_test_read(void)

{
		printk("sensor_test_read -1 dg_cnt=%x \n");
        senodia_work_func();
        return 0;
}

static int auto_test_read(void *unused)
{
        while(1){
                sensor_test_read();
                msleep(200);
        }
        return 0;
}
xx
#else
#endif

/*----------------------------------------------------------------------------*/
static int st480_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct i2c_client *new_client;
	struct hwmsen_object sobj_m, sobj_o;

#ifdef SENSOR_AUTO_TEST
        struct task_struct *thread;
#else
#endif

	SENODIAFUNC("senodia_probe");

	printk("---------------------> probe 1\n");

	/* Allocate memory for driver data */
	senodia = kzalloc(sizeof(struct senodia_data), GFP_KERNEL);
	if (!senodia) {
		printk(KERN_ERR "SENODIA senodia_probe: memory allocation failed.\n");
		err = -ENOMEM;
		goto exit1;
	}	
	memset(senodia, 0, sizeof(struct senodia_data));
#ifdef MTK_AUTO_DETECT_MAGNETOMETER
	senodia->hw = st480_get_cust_mag_hw();	
#else
	senodia->hw = get_cust_mag_hw();	
#endif
	
	atomic_set(&senodia->layout, senodia->hw->direction);
	atomic_set(&senodia->trace, 0);

	mutex_init(&sense_data_mutex);
	mutex_init(&sensor_data_mutex);

	//punk
//		client->ext_flag = (client->ext_flag)|(I2C_DMA_FLAG);
 ////       client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG | I2C_ENEXT_FLAG;

	senodia->client = client;
	this_client=client;
	i2c_set_clientdata(client, senodia);

	
	printk("---------------------> probe 2\n");

	if(st480_init(senodia->client) != 0)
        {
                printk("st480 init error!\n");
		goto exit3;
        }

	
	printk("---------------------> probe 3\n");

	/* Register sysfs attribute */
#ifdef MTK_AUTO_DETECT_MAGNETOMETER
	if(err = st480_create_attr(&st480_init_info.platform_diver_addr->driver))
#else
	if(err = st480_create_attr(&senodia_sensor_driver.driver))
#endif
	{
		printk(KERN_ERR "create attribute err = %d\n", err);
		goto exit3;
	}

	err = misc_register(&senodiad_device);
	if (err) {
		printk(KERN_ERR
			   "SENODIA senodia_probe: senodiad_device register failed\n");
		goto exit7;
	}

	err = misc_register(&st480_device);
	if (err) {
		printk(KERN_ERR
		       "SENODIA senodia_probe: st480_device register failed\n");
		goto exit9;
	}
	

	printk("---------------------> probe 4\n");

	sobj_m.self = senodia;
	sobj_m.polling = 1;
	sobj_m.sensor_operate = st480_operate;
	if(err = hwmsen_attach(ID_MAGNETIC, &sobj_m))
	{
		printk(KERN_ERR "attach fail = %d\n", err);
		goto exit9;
	}

	sobj_o.self = senodia;
	sobj_o.polling = 1;
	sobj_o.sensor_operate = st480_orientation_operate;
	if(err = hwmsen_attach(ID_ORIENTATION, &sobj_o))
	{
		printk(KERN_ERR "attach fail = %d\n", err);
		goto exit9;
	}
	
	/* As default, report all information */
	atomic_set(&m_flag, 0);
	atomic_set(&o_flag , 0);
	atomic_set(&open_flag, 0);


	printk("---------------------> probe 5\n");

#ifdef CONFIG_HAS_EARLYSUSPEND
	senodia->senodia_early_suspend.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,	
	senodia->senodia_early_suspend.suspend = senodia_early_suspend;
	senodia->senodia_early_suspend.resume = senodia_early_resume;
	register_early_suspend(&senodia->senodia_early_suspend);
#endif


#ifdef SENSOR_AUTO_TEST
	thread=kthread_run(auto_test_read,NULL,"st480_read_test");
#endif

	st480_init_flag = 0;/* Add for auto detect feature */
	printk("Senodia compass successfully probed.");
	return 0;


exit9:
	misc_deregister(&st480_device);
exit8:
	misc_deregister(&senodiad_device);
exit7:
exit6:
exit5:
exit3:
	kfree(senodia);
exit1:
exit0:
	st480_init_flag = -1;/* Add for auto detect feature */
	return err;
	
}
/*----------------------------------------------------------------------------*/
static int st480_i2c_remove(struct i2c_client *client)
{
	int err;	
	SENODIAFUNC("SENODIA_remove");	

#ifdef MTK_AUTO_DETECT_MAGNETOMETER
	if(err = st480_delete_attr(&st480_init_info.platform_diver_addr->driver))
#else
	if(err = st480_delete_attr(&senodia_sensor_driver.driver))
#endif
	{
		printk(KERN_ERR "st480_delete_attr fail: %d\n", err);
	}

	
	this_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));	
	misc_deregister(&senodiad_device); 
	misc_deregister(&st480_device); 

	SENODIADBG("successfully removed."); 
	return 0;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#ifdef MTK_AUTO_DETECT_MAGNETOMETER
/*----------------------------------------------------------------------------*/
static int st480_local_init(void) 
{
	struct mag_hw *hw = st480_get_cust_mag_hw();
	SENODIAFUNC("st480_local_init");
	senodia_power(hw, 1);
	//st480_force[0] = hw->i2c_num;
#if 1
	if(i2c_add_driver(&st480_i2c_driver))
	{
		printk("add driver error\n");
		return -1;
	}
#endif
	if(-1 == st480_init_flag)	
	{	   
		return -1;	
	}
	return 0;
}

static int senodia_remove(void)
{
	struct mag_hw *hw = st480_get_cust_mag_hw();
 
	senodia_power(hw, 0);    
	atomic_set(&dev_open_count, 0);  
	i2c_del_driver(&st480_i2c_driver);
	return 0;
}
#else
static int senodia_probe(struct platform_device *pdev) 
{
	struct mag_hw *hw = get_cust_mag_hw();

	senodia_power(hw, 1);	
	atomic_set(&dev_open_count, 0);
	if(i2c_add_driver(&st480_i2c_driver))
	{
		printk(KERN_ERR "add driver error\n");
		return -1;
	} 
	return 0;
}
static int senodia_remove(struct platform_device *pdev)
{
	struct mag_hw *hw = get_cust_mag_hw();
 
	senodia_power(hw, 0);    
	atomic_set(&dev_open_count, 0);  
	i2c_del_driver(&st480_i2c_driver);
	return 0;
}
#endif
/*----------------------------------------------------------------------------*/

static int __init senodia_init(void)
{
#ifdef MTK_AUTO_DETECT_MAGNETOMETER
    	struct mag_hw *hw = st480_get_cust_mag_hw();
#else
    	struct mag_hw *hw = get_cust_mag_hw();
#endif
	SENODIAFUNC("senodia compass driver: initialize");

	i2c_register_board_info(hw->i2c_num, &i2c_st480, 1);	
#ifdef MTK_AUTO_DETECT_MAGNETOMETER
	hwmsen_msensor_add(&st480_init_info);
/*	if(i2c_add_driver(&st480_i2c_driver))
	{
		printk("senodia_init  -1.5 add i2c  driver error\n");
		return -1;
	}*/
#else
	if(platform_driver_register(&senodia_sensor_driver))
	{
		printk(KERN_ERR "failed to register driver");
		return -ENODEV;
	}
#endif
	return 0; 
} 
/*----------------------------------------------------------------------------*/
static void __exit senodia_exit(void)
{
	SENODIAFUNC("senodia compass driver: release");
#ifndef MTK_AUTO_DETECT_MAGNETOMETER
	platform_driver_unregister(&senodia_sensor_driver);
#endif
}
/*----------------------------------------------------------------------------*/
module_init(senodia_init);
module_exit(senodia_exit);

MODULE_AUTHOR("Tori Xu <xuezhi_xu@senodia.com>");
MODULE_DESCRIPTION("senodia compass driver for MTK");
MODULE_LICENSE("GPL");
MODULE_VERSION("7.0");
