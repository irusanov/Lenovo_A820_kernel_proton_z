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

#define LCM_ID_OMT9605A (0x9605)

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0XFD   // END OF REGISTERS MARKER

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

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   


//#define   LCM_DSI_CMD_MODE							0


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

    {0xFF, 3  ,{0x96,0x05,0x01}},
    {0x00, 1 ,{0x80}}, 
    {0xFF, 2  ,{0x96,0x05}},
    {0x00, 1 ,{0x92}}, 
    {0xFF,2,{0x10,0x02}},// Fix to 2lane setting
    {0x00, 1 ,{0x00}}, 
    {0x00, 1  ,{0x00}},
    {0x00, 1 ,{0x00}}, 
    {0x00, 1  ,{0x00}},
    
    {0x00, 1 ,{0x91}}, 
    {0xc5, 1 ,{0x76}}, 
    {0x00, 1 ,{0x00}}, 
    {0xd8, 2 ,{0x6f,0x6f}},
    
    {0x00, 1 ,{0x00}}, 
    {0xd9, 1  ,{0x21}},//0x1a
    
    {0x00, 1 ,{0xa6}}, 
    {0xc1, 1 ,{0x01}}, 
    {0x00, 1 ,{0xb4}}, 
    {0xc0, 1 ,{0x50}}, 
    
    {0x00, 1 ,{0x80}}, 
    {0xC1, 2 ,{0x36,0x66}},
    
    {0x00, 1 ,{0x89}}, 
    {0xC0, 1 ,{0x01}},
    
    {0x00, 1 ,{0xb1}}, 
    {0xC5, 1 ,{0x28}},
    
    {0x00, 1 ,{0xb2}}, 
    {0xf5, 4 ,{0x15,0x00,0x15,0x00}},
    
    {0x00, 1 ,{0xc0}}, 
    {0xC5, 1 ,{0x00}},
    
    {0x00, 1 ,{0x88}}, 
    {0xc4, 1 ,{0x01}},
    
    {0x00, 1 ,{0xA2}}, 
    {0xC0, 3 ,{0x20,0x18,0x09}},
    
    {0x00, 1 ,{0x80}}, 
    {0xC4, 1 ,{0x9c}},
    
    {0x00, 1 ,{0x80}}, 
    {0xCB, 10,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    
    {0x00, 1 ,{0x90}}, 
    {0xCB, 15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    
    {0x00, 1 ,{0xA0}}, 
    {0xCB, 15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    
    {0x00, 1 ,{0xB0}}, 
    {0xCB, 10,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

    {0x00, 1 ,{0xC0}}, 
    {0xCB, 15,{0x00,0x00,0x04,0x04,0x04,0x04,0x00,0x00,0x04,0x04,0x04,0x04,0x00,0x00,0x00}},

    {0x00, 1 ,{0xD0}}, 
    {0xCB, 15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x04,0x04,0x04,0x00,0x00,0x04,0x04}},

    {0x00, 1 ,{0xE0}}, 
    {0xCB, 10,{0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

    {0x00, 1 ,{0xF0}}, 
    {0xCB, 10,{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}},

    {0x00, 1 ,{0x80}}, 
    {0xCC, 10,{0x00,0x00,0x26,0x25,0x02,0x06,0x00,0x00,0x0A,0x0E}},

    {0x00, 1 ,{0x90}}, 
    {0xCC, 15,{0x0C,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x26,0x25,0x01}},

    {0x00, 1 ,{0xA0}}, 
    {0xCC, 15,{0x05,0x00,0x00,0x09,0x0D,0x0B,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

    {0x00, 1 ,{0xB0}}, 
    {0xCC, 10,{0x00,0x00,0x25,0x26,0x05,0x01,0x00,0x00,0x0F,0x0B}},

    {0x00, 1 ,{0xC0}}, 
    {0xCC, 15,{0x0D,0x09,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x25,0x26,0x06}},

    {0x00, 1 ,{0xD0}}, 
    {0xCC, 15,{0x02,0x00,0x00,0x10,0x0C,0x0E,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

    {0x00, 1 ,{0x80}}, 
    {0xCE, 12,{0x87,0x03,0x28,0x86,0x03,0x28,0x00,0x0F,0x00,0x00,0x0F,0x00}},

    {0x00, 1 ,{0x90}}, 
    {0xCE, 14,{0x33,0xBE,0x28,0x33,0xBF,0x28,0xF0,0x00,0x00,0xF0,0x00,0x00,0x00,0x00}},

    {0x00, 1 ,{0xA0}}, 
    {0xCE, 14,{0x38,0x03,0x83,0xC0,0x8A,0x18,0x00,0x38,0x02,0x83,0xC1,0x89,0x18,0x00}},

    {0x00, 1 ,{0xB0}}, 
    {0xCE, 14,{0x38,0x01,0x83,0xC2,0x88,0x18,0x00,0x38,0x00,0x83,0xC3,0x87,0x18,0x00}},

    {0x00, 1 ,{0xC0}}, 
    {0xCE, 14,{0x30,0x00,0x83,0xC4,0x86,0x18,0x00,0x30,0x01,0x83,0xC5,0x85,0x18,0x00}},

    {0x00, 1 ,{0xD0}}, 
    {0xCE, 14,{0x30,0x02,0x83,0xC6,0x84,0x18,0x00,0x30,0x03,0x83,0xC7,0x83,0x18,0x00}},

    {0x00, 1 ,{0xC0}}, 
    {0xCF, 10,{0x01,0x01,0x20,0x20,0x00,0x00,0x01,0x01,0x00,0x00}},

    {0x00, 1 ,{0x80}},
    {0xD6, 1  ,{0x88}},
    
    {0x00, 1 ,{0x00}}, 
    {0xE1, 16 ,{0x00,0x03,0x0D,0x13,0x0D,0x1F,0x0D,0x0D,0x01,0x05,0x02,0x08,0x10,0x24,0x21,0x03}},

    {0x00, 1 ,{0x00}}, 
    {0xE2, 16 ,{0x00,0x03,0x0D,0x13,0x0D,0x1F,0x0D,0x0D,0x01,0x05,0x02,0x08,0x10,0x24,0x21,0x03}},
    
    {0x00, 1 ,{0x00}},
    {0xFF, 3 ,{0xFF,0xFF,0xFF}},
        
    //{0x21, 1 ,{0x00}},
    
    //{0x35, 1 ,{0x00}}, //open te
    
  //  {0x11, 1 ,{0x00}}, 
  //  {REGFLAG_DELAY, 150, {}},
    
 //   {0x29, 1 ,{0x00}}, 
    
 //   {REGFLAG_DELAY,50, {}},
    
    //{0x2C, 1 ,{0x00}}, 
    	// Note
    	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.
    	// Setting ending by predefined flag
    
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
    {REGFLAG_DELAY, 150, {}},
    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},
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

    for(i = 0; i < count; i++){ 
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

       //1 SSD2075 has no TE Pin 
		// enable tearing-free
        #if defined(LCM_DSI_CMD_MODE)
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
	 #else
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
	#endif
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
		//params->dsi.mode   = BURST_VDO_MODE;
		//params->dsi.mode   = SYNC_EVENT_VDO_MODE; 
		
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
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
		params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		//params->dsi.word_count=720*3;	

		
		params->dsi.vertical_sync_active				= 1;  //---3
		params->dsi.vertical_backporch					= 16; //---14
		params->dsi.vertical_frontporch					= 15;  //----8
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 6;  //----2
		params->dsi.horizontal_backporch				= 40; //----28
		params->dsi.horizontal_frontporch				= 40; //----50
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;


        //	params->dsi.HS_PRPR=6;
	    //params->dsi.LPX=8; 
		//params->dsi.HS_PRPR=5;
		//params->dsi.HS_TRAIL=13;

		// Bit rate calculation
		//1 Every lane speed
		params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4	
		params->dsi.fbk_div =16;//30;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
#if 0
params->dsi.HS_TRAIL    = 10;//10;//min max(n*8*UI, 60ns+n*4UI)
params->dsi.HS_ZERO    = 10;//8;//min 105ns+6*UI
params->dsi.HS_PRPR    = 6;//min 40ns+4*UI; max 85ns+6UI
params->dsi.LPX    = 12;//min 50ns

params->dsi.TA_SACK=1;
params->dsi.TA_GET=60;
params->dsi.TA_SURE=18;  
params->dsi.TA_GO    = 12;//12;//4*LPX
//
params->dsi.CLK_TRAIL   = 5;//5;//min 60ns
params->dsi.CLK_ZERO    = 18;//18;//min 300ns-38ns
params->dsi.LPX_WAIT    = 10;
params->dsi.CONT_DET    = 0;

params->dsi.CLK_HS_PRPR = 4;//min 38ns; max 95ns 
#endif
		
}

static void lcm_init(void)
{
	SET_RESET_PIN(1);
    MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	
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
	MDELAY(30);
}


static void lcm_resume(void)
{
#if 0
    lcm_init();
#else	
	//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);

    unsigned int data_array[2];

	data_array[0] = 0x00110500; // Sleep In
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(120); 

    data_array[0] = 0x00290500; // Display Off
    dsi_set_cmdq(&data_array, 1, 1);

	MDELAY(30);
#endif

}
         
#if (LCM_DSI_CMD_MODE)
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
#endif

static unsigned int lcm_compare_id(void)
{

    int   array[4];
	int id=0;
    char  buffer[4];
    char  id0=0;
    char  id1=0;
    char  id2=0;
    char  id3=0;

    SET_RESET_PIN(0);
    MDELAY(200);
    SET_RESET_PIN(1);
    MDELAY(200);

	array[0] = 0x00043700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xA1,buffer, 4);
	
	//array[0] = 0x00030000;// read id return two byte,version and id
	//dsi_set_cmdq(array, 1, 1);
	//read_reg_v2(0xA1,buffer+1, 1);
	
	//array[0] = 0x00030000;// read id return two byte,version and id
	//dsi_set_cmdq(array, 1, 1);
	//read_reg_v2(0xA1,buffer+2, 1);
	
	id0 = buffer[0]; //should be 0x01
	id1 = buffer[1];//should be 0x8b
	id2 = buffer[2];//should be 0x96
	id3 = buffer[3];//should be 0x05
	id=id2<<8 | id3;
	#ifdef BUILD_LK
		printf("tengdeqiang222 uboot %s\n", __func__);
		printf("teng%s, id0 = 0x%08x\n", __func__, id0);//should be 0x00
		printf("%s, id1 = 0x%08x\n", __func__, id1);//should be 0xaa
		printf("%s, id2 = 0x%08x\n", __func__, id2);//should be 0x55
	#else
		printk("tengdeqiang222 kernel %s\n", __func__);
		printk("teng%s, id0 = 0x%08x\n", __func__, id0);//should be 0x00
		printk("%s, id1 = 0x%08x\n", __func__, id1);//should be 0xaa
		printk("%s, id2 = 0x%08x\n", __func__, id2);//should be 0x55
		printk("%s, id3 = 0x%08x\n", __func__, id3);//should be 0x55
	#endif
		return (id == LCM_ID_OMT9605A)? 1:0;
}

#if 0
static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
	char  buffer[3];
	int   array[4];

	/// please notice: the max return packet size is 1
	/// if you want to change it, you can refer to the following marked code
	/// but read_reg currently only support read no more than 4 bytes....
	/// if you need to read more, please let BinHan knows.
	/*
			unsigned int data_array[16];
			unsigned int max_return_size = 1;
			
			data_array[0]= 0x00003700 | (max_return_size << 16);	
			
			dsi_set_cmdq(&data_array, 1, 1);
	*/
	array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xA1, buffer, 2);
	if(buffer[0]==0x20 && buffer[1] == 0x75)
	{
		return FALSE;
	}
	else
	{			 
		return TRUE;
	}
#endif

}

static unsigned int lcm_esd_recover(void)
{
	lcm_init();
	lcm_resume();

	return TRUE;
}
#endif

LCM_DRIVER otm9605a_qhd_dsi_vdo_tcl_lcm_drv = 
{
    .name			= "otm9605a_qhd_dsi_vdo_tcl",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,
//	.esd_check = lcm_esd_check,
//	.esd_recover = lcm_esd_recover,
    #if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
    #endif
    };
