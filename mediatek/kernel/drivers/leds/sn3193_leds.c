/*
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 * SN3193 driver
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/leds-mt65xx.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include<mach/mt_gpio.h>
#include<linux/timer.h>

#define INDICATION_LED       "sn3193"

#define SN3193_DEBUG_INFO

#if defined(SN3193_DEBUG_INFO)
#define SN3193_DEBUG(a,arg...)     printk(INDICATION_LED": "a,##arg)
#else
#define SN3193_DEBUG(arg...) 
#endif

#define SN3193_I2C_ADDRESS    0xD0
#define SN3193_RESET_PIN         GPIO74

#define SN3193_CURRENT_SET    2 //10 mA

enum{
    LED_BREATH_OFF = 0,
    LED_BREATH_POWER_ONOFF,
    LED_BREATH_INCOMING_CALL,
    LED_BREATH_BATTERY,
    LED_BREATH_SCREEN_ON,
    LED_BREATH_SCREEN_OFF,
    LED_BREATH_MISS_CALL,
    LED_BREATH_MISS_MESSAGE,
    LED_BREATH_FACTORY
};

kal_bool breath_factory_mode = KAL_FALSE;

typedef struct{
    u8 mode;
    u8 init;
    struct delayed_work  delayed_work;
    struct workqueue_struct * workqueue;
}SN3193_DATA_STRUCT;

struct i2c_client *sn3193_i2c_cilent = NULL;
static struct i2c_board_info __initdata i2c_sn3193={I2C_BOARD_INFO("SN3193", (SN3193_I2C_ADDRESS >> 1))};
static const struct i2c_device_id SN3193_id[] = {{"SN3193",0}, {}};

SN3193_DATA_STRUCT sn3193_data;
SN3193_DATA_STRUCT *sn3193_data_p = &sn3193_data;

static int  SN3193_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int  SN3193_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int  SN3193_remove(struct i2c_client *cilent);
    
static struct i2c_driver sn3193_i2c_driver = {
    .driver = {
        .name = "SN3193",
    },
    .probe = SN3193_probe,
    .remove = SN3193_remove,
    .detect = SN3193_detect,
    .id_table= SN3193_id,
};

static int SN3193_i2c_txdata(char *txdata, int len)
{
    int ret;
    struct i2c_msg msg[] = {
            {
                .addr = sn3193_i2c_cilent->addr,
                .flags = 0,
                .len =len,
                .buf = txdata,
            },
    };
    if(sn3193_i2c_cilent != NULL) {
        ret = i2c_transfer(sn3193_i2c_cilent->adapter, msg, 1);
        if(ret < 0)
            pr_err("%s i2c write erro: %d\n", __func__, ret);
    } else {
        SN3193_DEBUG("sn3193_i2c_cilent null\n");
    }
    return ret;
}

static int SN3193_write_reg(u8 addr, u8 para)
{
    u8 buf[3];
    int ret = -1;
        
    buf[0] = addr;
    buf[1] = para;
    ret = SN3193_i2c_txdata(buf,2);
    if(ret < 0) {
        SN3193_DEBUG("write reg failed! addr= %x ret= %d\n", buf[0], ret);
        return -1;
    }
    return 0;
}

static void breath_led_init(void)
{
    SN3193_DEBUG("entry led_init\n");
    mt_set_gpio_mode(SN3193_RESET_PIN,GPIO_MODE_GPIO);
    mt_set_gpio_dir(SN3193_RESET_PIN,GPIO_DIR_OUT);
    mt_set_gpio_out(SN3193_RESET_PIN,GPIO_OUT_ONE);

    SN3193_write_reg(0x2F, 1);
    mdelay(5);
    SN3193_write_reg(0x1D, 0x00);
    SN3193_write_reg(0x03, 0x03 << 2); // 011:30mA
	
    SN3193_write_reg(0x04, 170); //B
    SN3193_write_reg(0x05, 130); //G
    SN3193_write_reg(0x06, 170); //R

    SN3193_write_reg(0x0A, 0x00);
    SN3193_write_reg(0x0B, 0x00);
    SN3193_write_reg(0x0C, 0x00);

    SN3193_write_reg(0x10, 0x04 << 1);
    SN3193_write_reg(0x11, 0x04 << 1);
    SN3193_write_reg(0x12, 0x04 << 1);

    SN3193_write_reg(0x16, 0x04 << 1);  
    SN3193_write_reg(0x17, 0x04 << 1);  
    SN3193_write_reg(0x18, 0x04 << 1);  
	
    SN3193_write_reg(0x02, 0x01 << 5);
    SN3193_write_reg(0x1C, 1);
    SN3193_write_reg(0x07, 1);	
    SN3193_write_reg(0x00, 0x01);

    breath_factory_mode = KAL_FALSE;
	
    SN3193_DEBUG("exit led_init\n");
}

void breath_led_off(void)
{
    SN3193_write_reg(0x1D, 0x00);
    SN3193_write_reg(0x07, 1);	
    SN3193_write_reg(0x00, 0x01);
    breath_factory_mode = KAL_FALSE;
}


void breath_blue_led_on(void)  //Dout1
{
     SN3193_write_reg(0x00, 0x20);
     SN3193_write_reg(0x04, 170);
     SN3193_write_reg(0x02, 0x00 << 5);
     SN3193_write_reg(0x01, 0x00); //Dout3:10->red on, Dout2:01->green, Dout1:00->blue
     SN3193_write_reg(0x1D, 0x01); //Dout3 enable, xxx:Dout3-Dout1
     SN3193_write_reg(0x07, 1);	
}

void breath_green_led_on(void) //Dout2
{
     SN3193_write_reg(0x00, 0x20);
     SN3193_write_reg(0x05, 130);
     SN3193_write_reg(0x02, 0x00 << 5);
     SN3193_write_reg(0x01, 0x01); //Dout3:10->red on, Dout2:01->green, Dout1:00->blue
     SN3193_write_reg(0x1D, 0x02); //Dout3 enable, xxx:Dout3-Dout1
     SN3193_write_reg(0x07, 1);	
}

void breath_red_led_on(void) //Dout3
{
     SN3193_write_reg(0x00, 0x20);
     SN3193_write_reg(0x06, 170);
     SN3193_write_reg(0x02, 0x00 << 5);
     SN3193_write_reg(0x01, 0x02); //Dout3:10->red on, Dout2:01->green, Dout1:00->blue
     SN3193_write_reg(0x1D, 0x04); //Dout3 enable, xxx:Dout3-Dout1
     SN3193_write_reg(0x07, 1);	
}

void breath_red_led_slow_blink(void)
{
    SN3193_write_reg(0x00, 0x20);
    SN3193_write_reg(0x06, 170);
    SN3193_write_reg(0x0C, 0x00);	//R
    SN3193_write_reg(0x12, 0x68); //0x04<< 1);
    SN3193_write_reg(0x18, 0x6a); //0x04 << 1);  
    SN3193_write_reg(0x02, 0x01 << 5);
    SN3193_write_reg(0x1C, 1);
    SN3193_write_reg(0x01, 0x02); 
    SN3193_write_reg(0x1D, 0x04);
    SN3193_write_reg(0x07, 1);	 
}

void breath_green_led_slow_blink(void)
{
#if 0
    SN3193_write_reg(0x00, 0x20);
    SN3193_write_reg(0x11, 0x60); //0x04<< 1);
    SN3193_write_reg(0x17, 0x6a); //0x04 << 1);  
    SN3193_write_reg(0x02, 0x01 << 5);
    SN3193_write_reg(0x1C, 1);
    SN3193_write_reg(0x01, 0x01); 
    SN3193_write_reg(0x1D, 0x02);
    SN3193_write_reg(0x07, 1);	 
#else
    SN3193_write_reg(0x00, 0x20);

    SN3193_write_reg(0x02, 0x01 << 5); //RGB mode

    SN3193_write_reg(0x06, 130); //DOUT3,R
    SN3193_write_reg(0x05, 0); //DOUT2,G
    SN3193_write_reg(0x04, 170); //DOUT1,B
	
    SN3193_write_reg(0x0B, 0x00);
    SN3193_write_reg(0x11, 0x68); //0x04 << 1);
    SN3193_write_reg(0x17, 0x6a); //0x04 << 1);  

    SN3193_write_reg(0x0C, 0x00);	//R
    SN3193_write_reg(0x12, 0x68); //0x04 << 1);
    SN3193_write_reg(0x18, 0x6a); //0x04 << 1);  

    SN3193_write_reg(0x0A, 0x00);
    SN3193_write_reg(0x10, 0x68); //0x04 << 1);
    SN3193_write_reg(0x16, 0x6a); //0x04 << 1);  

    SN3193_write_reg(0x1C, 1);
    SN3193_write_reg(0x01, 0x03); 
    SN3193_write_reg(0x1D, 0x07);
    SN3193_write_reg(0x07, 1);	 
#endif
}

void breath_blue_led_slow_blink(void)
{
#if 0
    SN3193_write_reg(0x00, 0x20);
    SN3193_write_reg(0x0A, 0x00);
    SN3193_write_reg(0x10, 0x60); //0x04<< 1);
    SN3193_write_reg(0x16, 0x6a); //0x04 << 1);  
    SN3193_write_reg(0x02, 0x01 << 5);
    SN3193_write_reg(0x1C, 1);
    SN3193_write_reg(0x01, 0x00); 
    SN3193_write_reg(0x1D, 0x01);
    SN3193_write_reg(0x07, 1);	 
#else
    SN3193_write_reg(0x00, 0x20);
    SN3193_write_reg(0x02, 0x01 << 5); //RGB mode

    SN3193_write_reg(0x06, 0); //DOUT3,R
    SN3193_write_reg(0x05, 100); //DOUT2,G
    SN3193_write_reg(0x04, 170); //DOUT1,B
	
    SN3193_write_reg(0x0B, 0x00);
    SN3193_write_reg(0x11, 0x68); //0x04 << 1);
    SN3193_write_reg(0x17, 0x6a); //0x04 << 1);

    SN3193_write_reg(0x0C, 0x00);
    SN3193_write_reg(0x12, 0x68); //0x04 << 1);
    SN3193_write_reg(0x18, 0x6a); //0x04 << 1);

    SN3193_write_reg(0x0A, 0x00);
    SN3193_write_reg(0x10, 0x68); //0x04 << 1);
    SN3193_write_reg(0x16, 0x6a); //0x04 << 1);

    SN3193_write_reg(0x1C, 1);
    SN3193_write_reg(0x01, 0x03); 
    SN3193_write_reg(0x1D, 0x07);
    SN3193_write_reg(0x07, 1);	 
#endif
}

void breath_green_led_quick_blink(void)
{
#if 0
    SN3193_write_reg(0x00, 0x20);
    SN3193_write_reg(0x11, 0x40); //0x00 << 1);
    SN3193_write_reg(0x17, 0x44); //0x00 << 1);
    SN3193_write_reg(0x02, 0x01 << 5);
    SN3193_write_reg(0x1C, 1);
    SN3193_write_reg(0x01, 0x01); 
    SN3193_write_reg(0x1D, 0x02);
    SN3193_write_reg(0x07, 1);	  
#else
    SN3193_write_reg(0x00, 0x20);
    SN3193_write_reg(0x02, 0x01 << 5); //RGB mode

    SN3193_write_reg(0x06, 130); //DOUT3,R
    SN3193_write_reg(0x05, 0); //DOUT2,G
    SN3193_write_reg(0x04, 170); //DOUT1,B
	
    SN3193_write_reg(0x0B, 0x00);
    SN3193_write_reg(0x11, 0x40); //0x00 << 1);
    SN3193_write_reg(0x17, 0x44); //0x00 << 1);

    SN3193_write_reg(0x0C, 0x00);
    SN3193_write_reg(0x12, 0x40); //0x00 << 1);
    SN3193_write_reg(0x18, 0x44); //0x00 << 1);

    SN3193_write_reg(0x0A, 0x00);
    SN3193_write_reg(0x10, 0x40); //0x00 << 1);
    SN3193_write_reg(0x16, 0x44); //0x00 << 1);

    SN3193_write_reg(0x1C, 1);
    SN3193_write_reg(0x01, 0x03); 
    SN3193_write_reg(0x1D, 0x07);
    SN3193_write_reg(0x07, 1);	 
#endif
}

void breath_red_led_quick_blink(void)
{
    SN3193_write_reg(0x00, 0x20);
    SN3193_write_reg(0x0C, 0x00);
    SN3193_write_reg(0x12, 0x40); //0x00 << 1);
    SN3193_write_reg(0x18, 0x44); //0x00 << 1);
    SN3193_write_reg(0x02, 0x01 << 5);
    SN3193_write_reg(0x1C, 1);
    SN3193_write_reg(0x01, 0x02); 
    SN3193_write_reg(0x1D, 0x04);
    SN3193_write_reg(0x07, 1);	 
}

void breath_blue_led_quick_blink(void)
{
    SN3193_write_reg(0x00, 0x20);
    SN3193_write_reg(0x0A, 0x00);
    SN3193_write_reg(0x10, 0x40); //0x00 << 1);
    SN3193_write_reg(0x16, 0x44); //0x00 << 1);
    SN3193_write_reg(0x02, 0x01 << 5);
    SN3193_write_reg(0x1C, 1);
    SN3193_write_reg(0x01, 0x00); 
    SN3193_write_reg(0x1D, 0x01);
    SN3193_write_reg(0x07, 1);	 
}

void breath_rgb_factory_test(void)
{
    breath_factory_mode = KAL_TRUE;
	
    SN3193_write_reg(0x00, 0x20);
    SN3193_write_reg(0x02, 0x01 << 5); //RGB mode

    SN3193_write_reg(0x06, 170); //DOUT3,R
    SN3193_write_reg(0x05, 130); //DOUT2,G
    SN3193_write_reg(0x04, 170); //DOUT1,B

    SN3193_write_reg(0x0C, 0x00);	//R
    SN3193_write_reg(0x12, 0x04);
    SN3193_write_reg(0x18, 0x08);  

    SN3193_write_reg(0x0B, 0x30);	//G
    SN3193_write_reg(0x11, 0x04);
    SN3193_write_reg(0x17, 0x08);  

    SN3193_write_reg(0x0A, 0x40);	//B
    SN3193_write_reg(0x10, 0x04);
    SN3193_write_reg(0x16, 0x08);  
    SN3193_write_reg(0x1C, 1);
    SN3193_write_reg(0x07, 1);	 

    SN3193_write_reg(0x1C, 1);
    SN3193_write_reg(0x01, 0x03); 
    SN3193_write_reg(0x1D, 0x07);
    SN3193_write_reg(0x07, 1);	 
}

void led_breath_delayed_work(struct work_struct *work)
{
    u8 mode;

    mode = sn3193_data_p->mode;

    SN3193_DEBUG("led_breath_delayed_work mode = %d\n", mode);
    switch(mode)  {
	case 0: //off
		//breath_led_off();
		break;
	case 1: //keypad leds
		//breath_blue_led_on();
		break;
	case 2: //charging full
		breath_green_led_on();
		break;
	case 3: //charging
		breath_red_led_on();
		break;
	case 4: //miss call, unread message
		breath_green_led_slow_blink();
		break;
	case 5: //incoming call
		breath_green_led_quick_blink();
		break;
	case 6: //low battery
		breath_red_led_slow_blink();
		break;
	case 7: 
		breath_blue_led_slow_blink();
		break;
	case 8: 
		breath_blue_led_quick_blink();
		break;
	case 10:
		breath_rgb_factory_test();
		break;
	case 9:
	default:
		breath_led_off();
		break;
        }
}

void led_breath_set(u8 mode)
{
    SN3193_DEBUG("led_breath_set mode=%d\n",mode);

    if(mode > 10)
    {
       return;
    }
    else if((breath_factory_mode)&&(mode != 9))
    {
    	return;
    }

    if(sn3193_i2c_cilent == NULL) {
        SN3193_DEBUG("sn3193_i2c_cilent null\n");
        return;
    }
    cancel_delayed_work_sync(&(sn3193_data_p->delayed_work));
    
    sn3193_data_p->mode = mode;
        
    if(!sn3193_data_p->init) {
        breath_led_init();
        sn3193_data_p->init = 1;
    }
    queue_delayed_work(sn3193_data_p->workqueue, &(sn3193_data_p->delayed_work),HZ/100); 
}


static int  SN3193_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    SN3193_DEBUG("SN3193_probe\n");
    sn3193_i2c_cilent = client;
    memset(sn3193_data_p, 0, sizeof(SN3193_DATA_STRUCT));
    sn3193_data_p->workqueue = create_workqueue("sn3193");
    INIT_DELAYED_WORK(&(sn3193_data_p->delayed_work), led_breath_delayed_work);

    breath_led_init();
    SN3193_DEBUG("SN3193_probe\n");
    return 0;
 }

static int  SN3193_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
    strcpy(info->type, INDICATION_LED);
    return 0;
}

static int  SN3193_remove(struct i2c_client *client)
{
    destroy_workqueue(&(sn3193_data_p->delayed_work));
    return 0;
}
static int __init SN3193_init(void)
{
    SN3193_DEBUG("SN3193_init driver-----6\n");
    i2c_register_board_info(3, &i2c_sn3193, 1);
    msleep(500);
    if(i2c_add_driver(&sn3193_i2c_driver)!=0) {
        SN3193_DEBUG("unable to add i2c driver.\n");
        return -1;
     }
    
    return 0;
}

static void __exit SN3193_exit(void)
{
    i2c_del_driver(&sn3193_i2c_driver);
    return ;
}

module_init(SN3193_init);
module_exit(SN3193_exit);
MODULE_AUTHOR("zhouwl@lenovo.com");
MODULE_DESCRIPTION("sn3193 led driver");
MODULE_LICENSE("GPL");

