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

/*******************************************************************************
 *
 * Filename:
 * ---------
 * med_audio_default.h
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 * This file is the header of audio customization related function or definition.
 *
 * Author:
 * -------
 * Chipeng Chang
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$

 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#ifndef MED_AUDIO_CUSTOM_H
#define MED_AUDIO_CUSTOM_H
// normal mode parameters ------------------------
#define NORMAL_SPEECH_OUTPUT_FIR_COEFF \
    32767,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
                                      \
    32767,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
                                      \
    32767,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
                                      \
    32767,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
                                      \
    32767,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
                                      \
    32767,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0
// headset mode parameters ------------------------
#define HEADSET_SPEECH_OUTPUT_FIR_COEFF \
      -26,   301, -1127,   613,   808,\
     -440, -1615,  -748,  3392,   168,\
    -2246, -4607,  5049,  4276, -4275,\
    -8536,  4880, 17778, -7121,-20701,\
   -18898, 32767, 32767,-18898,-20701,\
    -7121, 17778,  4880, -8536, -4275,\
     4276,  5049, -4607, -2246,   168,\
     3392,  -748, -1615,  -440,   808,\
      613, -1127,   301,   -26,     0,\
                                      \
    32767,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
                                      \
    32767,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
                                      \
    32767,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
                                      \
    32767,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
                                      \
    32767,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0
// handfree mode parameters ------------------------
#define HANDFREE_SPEECH_OUTPUT_FIR_COEFF \
     -457,  -855,   520,  -387,  -120,\
       99,   231,   972,   745,  2929,\
      215,  -713, -2172, -2279, -2410,\
      832, -6305,  8020,  1405, 13811,\
    -8179, 32767, 32767, -8179, 13811,\
     1405,  8020, -6305,   832, -2410,\
    -2279, -2172,  -713,   215,  2929,\
      745,   972,   231,    99,  -120,\
     -387,   520,  -855,  -457,     0,\
                                      \
    32767,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
                                      \
    32767,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
                                      \
    32767,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
                                      \
    32767,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
                                      \
    32767,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0
// VoIP_BT mode parameters ------------------------
#define VOIPBT_SPEECH_OUTPUT_FIR_COEFF \
    32767,	  0,	 0, 	0,  0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0
// VoIP_NORMAL mode parameters ------------------------
#define VOIPNORMAL_SPEECH_OUTPUT_FIR_COEFF \
    32767,	  0,	 0, 	0,  0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0
// VoIP_Handfree mode parameters ------------------------
#define VOIPHANDFREE_SPEECH_OUTPUT_FIR_COEFF \
    32767,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0
// AUX1 mode parameters ------------------------
#define AUX1_SPEECH_OUTPUT_FIR_COEFF \
    32767,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0
// AUX2 mode parameters ------------------------
#define AUX2_SPEECH_OUTPUT_FIR_COEFF \
    32767,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    0,	  0,	 0, 	0,   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    \
    32767,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0, \
    0,	  0,	 0, 	0,	   0
#define SPEECH_OUTPUT_MED_FIR_COEFF \
    NORMAL_SPEECH_OUTPUT_FIR_COEFF,\
    HEADSET_SPEECH_OUTPUT_FIR_COEFF ,\
    HANDFREE_SPEECH_OUTPUT_FIR_COEFF ,\
    VOIPBT_SPEECH_OUTPUT_FIR_COEFF,\
    VOIPNORMAL_SPEECH_OUTPUT_FIR_COEFF,\
    VOIPHANDFREE_SPEECH_OUTPUT_FIR_COEFF,\
    AUX1_SPEECH_OUTPUT_FIR_COEFF,\
    AUX2_SPEECH_OUTPUT_FIR_COEFF
#define SPEECH_INPUT_MED_FIR_COEFF\
       -4,    13,    -1,    53,    18,\
     -170,    -8,  -193,   136,    41,\
      144,  -123,  -138,   379,  -225,\
     1064,  -556,   576, -1707,  1359,\
    -1756,  5827,  5827, -1756,  1359,\
    -1707,   576,  -556,  1064,  -225,\
      379,  -138,  -123,   144,    41,\
      136,  -193,    -8,  -170,    18,\
       53,    -1,    13,    -4,     0,\
                                      \
      173,  -217,  -191,  -659,  -297,\
     -644,   575, -1082,  -821, -1281,\
    -1484, -1776, -2471,  2983, -3774,\
     2463, -7156,  4822,-10208,   188,\
    -7720, 32767, 32767, -7720,   188,\
   -10208,  4822, -7156,  2463, -3774,\
     2983, -2471, -1776, -1484, -1281,\
     -821, -1082,   575,  -644,  -297,\
     -659,  -191,  -217,   173,     0,\
                                      \
      187,   -11,   152,  -226,   547,\
     -189,     7,  -214,   674,  -333,\
     -812,   -72,  -637,  4706,  -744,\
     5243, -3473,   870, -5593,  4917,\
    -2793, 20675, 20675, -2793,  4917,\
    -5593,   870, -3473,  5243,  -744,\
     4706,  -637,   -72,  -812,  -333,\
      674,  -214,     7,  -189,   547,\
     -226,   152,   -11,   187,     0,\
                                      \
    32767,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
                                      \
    32767,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
                                      \
    32767,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
                                      \
    32767,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
                                      \
    32767,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0,\
        0,     0,     0,     0,     0
#define FIR_output_index\
     3,     0,     0,     3,     0,     0,     0,     0
#define FIR_input_index\
     0,     0,     0,     0,     0,     0,     0,     0
#define MED_SPEECH_NORMAL_MODE_PARA \
    96,   253, 16388,    29, 57351,    31,   400,    48,\
   400,  4325,   611,     0, 20488,   371,   447,  8192
#define MED_SPEECH_EARPHONE_MODE_PARA \
     0,   189, 10756,    31, 57351,    31,   400,    48,\
    80,  4325,   611,     0, 20488,     0,     0,     0
#define MED_SPEECH_BT_EARPHONE_MODE_PARA \
     0,   253, 10756,    31, 53255,    31,   400,     0,\
    80,  4325,   611,     0, 53256,     0,     0,    86
#define MED_SPEECH_LOUDSPK_MODE_PARA \
    96,   224,  5256,    31, 57351, 24607,   400,    32,\
    84,  4325,   611,     0, 20488,     0,     0,     0
#define MED_SPEECH_CARKIT_MODE_PARA \
    96,   224,  5256,    31, 57351, 24607,   400,   132,\
    84,  4325,   611,     0, 20488,     0,     0,     0
#define MED_SPEECH_BT_CORDLESS_MODE_PARA \
    0,      0,      0,      0,      0,      0,      0,      0, \
    0,      0,      0,      0,      0,      0,      0,      0
#define MED_SPEECH_AUX1_MODE_PARA \
    0,      0,      0,      0,      0,      0,      0,      0, \
    0,      0,      0,      0,      0,      0,      0,      0
#define MED_SPEECH_AUX2_MODE_PARA \
    0,      0,      0,      0,      0,      0,      0,      0, \
    0,      0,      0,      0,      0,      0,      0,      0
#endif
