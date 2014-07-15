/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (540)
#define FRAME_HEIGHT (960)

#define LCM_ID_NT35516 (0x80)

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0XF5   // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
//#define read_reg(cmd)											lcm_util.DSI_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)     

#define   LCM_DSI_CMD_MODE							1
struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};



static struct LCM_setting_table lcm_initialization_setting[] = {
	
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/
{0xFF,5,{0xAA,0x55,0x25,0x01,0x01}},                                                         
                
{0xF2,35,{0x00,0x00,0x4A,0x0A,0xA8,0x00,0x00,0x00,
	    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	    0x00,0x0B,0x00,0x00,0x00,0x00,0x00,0x00,
	    0x00,0x00,0x00,0x00,0x40,0x01,0x51,0x00,
	    0x01,0x00,0x01}},                                                            
{0xF3,7,{0x02,0x03,0x07,0x45,0x88,0xD1,0x0D}},                                                     

//page 0                             
{0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},                                                         

{0xB1,1,{0xEC}},                                                                                               
{0xB6,1,{0x01}},                   
{0xB7,2,{0x72,0x72}},    
{0xB8,4,{0x01,0x03,0x03,0x03}},                                                                                                                                
{0xBB,1,{0x33}},                                                                                                                                
{0xBC,3,{0x03,0x00,0x00}},                                                                                                                                
{0xBD,5,{0x01,0x4E,0x10,0x20,0x01}},  
{0xC9,6,{0x61,0x06,0x0D,0x17,0x17,0x00}},                                              

// page 1
{0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},                                                          

{0xB0,3,{0x0C,0x0C,0x0C}},                                                      
{0xB1,3,{0x0C,0x0C,0x0C}},                                                      
{0xB2,3,{0x02,0x02,0x02}},                                                     
{0xB3,3,{0x10,0x10,0x10}},                                                    
{0xB4,3,{0x06,0x06,0x06}},                                                      
{0xB6,3,{0x44,0x44,0x44}},  
{0xB7,3,{0x24,0x24,0x24}},                                                      
{0xB8,3,{0x30,0x30,0x30}},                                                                                                   
{0xB9,3,{0x34,0x34,0x34}},                                                   
{0xBA,3,{0x24,0x24,0x24}},                                                     
{0xBC,3,{0x00,0x98,0x00}},                                                      
{0xBD,3,{0x00,0x98,0x00}},                                                      
{0xBE,1,{0x57}}, 

{0xC2,1,{0x00}},                                                                                            
{0xD0,4,{0x0F,0x0F,0x10,0x10}},                                                                                                   

                            
{0xD1,16,{0x00,0x23,0x00,0x24,0x00,0x31,0x00,0x52,
	       0x00,0x72,0x00,0xAE,0x00,0xDE,0x01,0x22}},                                                                                                                   

                            
{0xD2,16,{0x01,0x52,0x01,0x92,0x01,0xBE,0x01,0xFD,
	        0x02,0x2A,0x02,0x2B,0x02,0x53,0x02,0x7A}},                                                                                                                   

                           
{0xD3,16,{0x02,0x90,0x02,0xA8,0x02,0xB8,0x02,0xCB,
	        0x02,0xD7,0x02,0xE6,0x02,0xF2,0x03,0x00}},                                                                                                                   


{0xD4,4,{0x03,0x1C,0x03,0x52}},                                                                                                                                            
{0xD5,16,{0x00,0x23,0x00,0x24,0x00,0x31,0x00,0x52,
	        0x00,0x72,0x00,0xAE,0x00,0xDE,0x01,0x22}},                                                                                                                                               
{0xD6,16,{0x01,0x52,0x01,0x92,0x01,0xBE,0x01,0xFD,
	    0x02,0x2A,0x02,0x2B,0x02,0x53,0x02,0x7A}},                                                                                                                                   
{0xD7,16,{0x02,0x90,0x02,0xA8,0x02,0xB8,0x02,0xCB,
	0x02,0xD7,0x02,0xE6,0x02,0xF2,0x03,0x00}},      
	                                                                                                             
{0xD8,4,{0x03,0x1C,0x03,0x52}},                                                                                                        
{0xD9,16,{0x00,0x23,0x00,0x24,0x00,0x31,0x00,0x52,
	0x00,0x72,0x00,0xAE,0x00,0xDE,0x01,0x22}},                                                                                                                                          
{0xDD,16,{0x01,0x52,0x01,0x92,0x01,0xBE,0x01,0xFD,
	0x02,0x2A,0x02,0x2B,0x02,0x53,0x02,0x7A}},                                                                                                                                            
{0xDE,16,{0x02,0x90,0x02,0xA8,0x02,0xB8,0x02,0xCB,
	0x02,0xD7,0x02,0xE6,0x02,0xF2,0x03,0x00}},                                                                                                                   

{0xDF,4,{0x03,0x1C,0x03,0x52}},                                                                                                                          
{0xE0,16,{0x00,0x23,0x00,0x24,0x00,0x31,0x00,0x52,
	0x00,0x72,0x00,0xAE,0x00,0xDE,0x01,0x22}},                                                                                                                                            
{0xE1,16,{0x01,0x52,0x01,0x92,0x01,0xBE,0x01,0xFD,
	0x02,0x2A,0x02,0x2B,0x02,0x53,0x02,0x7A}},                                                                                                                                              
{0xE2,16,{0x02,0x90,0x02,0xA8,0x02,0xB8,0x02,0xCB,
	0x02,0xD7,0x02,0xE6,0x02,0xF2,0x03,0x00}},                                                                                                                   

{0xE3,4,{0x03,0x1C,0x03,0x52}},                    
{0xE4,16,{0x00,0x23,0x00,0x24,0x00,0x31,0x00,0x52,
	        0x00,0x72,0x00,0xAE,0x00,0xDE,0x01,0x22}},                                             
{0xE5,16,{0x01,0x52,0x01,0x92,0x01,0xBE,0x01,0xFD,
	        0x02,0x2A,0x02,0x2B,0x02,0x53,0x02,0x7A}},                         
{0xE6,16,{0x02,0x90,0x02,0xA8,0x02,0xB8,0x02,0xCB,
	        0x02,0xD7,0x02,0xE6,0x02,0xF2,0x03,0x00}},                                                                                                                   

{0xE7,4,{0x03,0x1C,0x03,0x52}},        
{0xE8,16,{0x00,0x23,0x00,0x24,0x00,0x31,0x00,0x52,
	        0x00,0x72,0x00,0xAE,0x00,0xDE,0x01,0x22}},                                    
{0xE9,16,{0x01,0x52,0x01,0x92,0x01,0xBE,0x01,0xFD,
	        0x02,0x2A,0x02,0x2B,0x02,0x53,0x02,0x7A}},                              
{0xEA,16,{0x02,0x90,0x02,0xA8,0x02,0xB8,0x02,0xCB,
	        0x02,0xD7,0x02,0xE6,0x02,0xF2,0x03,0x00}},                                                                                                                   

{0xEB,4,{0x03,0x1C,0x03,0x52}}, 

	{0x11, 0, {0x00}},
    {REGFLAG_DELAY, 200, {}},

    // Display ON
	{0x29, 0, {0x00}},
    {REGFLAG_DELAY,50, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_mode_in_setting[] = {

	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 200, {}},

    // Sleep Mode On

	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}

};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)

{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
		  dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		   MDELAY(2);
       	}
    }
	
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		// enable tearing-free
	 #if defined(LCM_DSI_CMD_MODE)
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
	 #else
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;//LCM_DBI_TE_MODE_VSYNC_ONLY;
	#endif
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if defined(LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 0;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

		
		params->dsi.vertical_sync_active				= 10;  //---3
		params->dsi.vertical_backporch					= 30; //---14
		params->dsi.vertical_frontporch					= 30;  //----8
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 10;  //----2
		params->dsi.horizontal_backporch				= 50; //----28
		params->dsi.horizontal_frontporch				= 50; //----50
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		// Bit rate calculation
		params->dsi.pll_div1=1;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4	
		params->dsi.fbk_div =30;//30;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	

}


static void lcm_init(void)
{
	SET_RESET_PIN(1);
    MDELAY(10);
	SET_RESET_PIN(0);
    MDELAY(200);
    SET_RESET_PIN(1);
    MDELAY(200);  
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);


}


static void lcm_suspend(void)
{
	unsigned int data_array[2];
	data_array[0] = 0x00280500; // Display Off
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(120); 
	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(120);

#ifdef BUILD_LK
	printf("zhibin uboot %s\n", __func__);
#else
	printk("zhibin kernel %s\n", __func__);
#endif

}


static void lcm_resume(void)
{
	//lcm_init();	
#ifdef BUILD_LK
	printf("zhibin uboot %s\n", __func__);
#else
	printk("zhibin kernel %s\n", __func__);
#endif
	//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
    unsigned int data_array[2];

	data_array[0] = 0x00110500; // Sleep In
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(120); 

    data_array[0] = 0x00290500; // Display Off
    dsi_set_cmdq(&data_array, 1, 1);

	MDELAY(120);


}


static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(&data_array, 3, 1);

	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(&data_array, 3, 1);
	
	data_array[0] = 0x00290508;
	dsi_set_cmdq(&data_array, 1, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}
static unsigned int lcm_compare_id(void)
{

		int   array[4];
		char  buffer[3];
		char  id0=0;
		char  id1=0;
		char  id2=0;


		SET_RESET_PIN(0);
		MDELAY(200);
		SET_RESET_PIN(1);
		MDELAY(200);
		
	array[0] = 0x00033700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x04,buffer, 3);
	
	id0 = buffer[0]; //should be 0x00
	id1 = buffer[1];//should be 0x80
	id2 = buffer[2];//should be 0x00
	
	#ifdef BUILD_LK
		printf("tengdeqiang111 uboot %s\n", __func__);
		printf("%s, id0 = 0x%08x\n", __func__, id0);//should be 0x00
		printf("%s, id1 = 0x%08x\n", __func__, id1);//should be 0xaa
		printf("%s, id2 = 0x%08x\n", __func__, id2);//should be 0x55
	#else
		printk("tengdeqiang111 kernel %s\n", __func__);	
		printk("%s, id0 = 0x%08x\n", __func__, id0);//should be 0x00
		printk("%s, id1 = 0x%08x\n", __func__, id1);//should be 0xaa
		printk("%s, id2 = 0x%08x\n", __func__, id2);//should be 0x55

	#endif
	return (LCM_ID_NT35516 == id1)? 1:0;

}

LCM_DRIVER nt35516_qhd_dsi_vdo_xinli_lcm_drv = 
{
	.name		= "nt35516_qhd_dsi_vdo_xinli",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
	.update         = lcm_update,
#endif
};
