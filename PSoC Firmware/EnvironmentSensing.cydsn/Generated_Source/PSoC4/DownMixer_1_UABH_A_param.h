/*!*****************************************************************************
* \file DownMixer_1_UABH_A_param.h
* \version 1.0
*
* \brief 
*   Definitions from Component Parameter
*
********************************************************************************
* \copyright
* (c) 2014-2016, Cypress Semiconductor Corporation. All rights reserved.
* This software, including source code, documentation and related
* materials ("Software"), is owned by Cypress Semiconductor
* Corporation ("Cypress") and is protected by and subject to worldwide
* patent protection (United States and foreign), United States copyright
* laws and international treaty provisions. Therefore, you may use this
* Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the
* Software source code solely for use in connection with Cypress's
* integrated circuit products. Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO
* WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING,
* BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE. Cypress reserves the right to make
* changes to the Software without notice. Cypress does not assume any
* liability arising out of the application or use of the Software or any
* product or circuit described in the Software. Cypress does not
* authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/* excluding defs in this file with cond and endcond */
/*! @cond */

#if !defined(DownMixer_1_UABH_A_PARAM_H)
#define DownMixer_1_UABH_A_PARAM_H

#include "DownMixer_1_UABH_A_CyUAB_types.h"
#include "DownMixer_1_UABH_A_regs.h"
#include "cytypes.h"
#include "cyfitter.h"

#define DownMixer_1_UABH_A_Nonzero(x) ( (x) != 0 )

#define DownMixer_1_UABH_A_UnmappedIsZero(x)  ((uint32)(((x)==-1)? 0 : (x)))

/*!
* \addtogroup group_init
* @{
*/
/*! Component Parameters set in the component customizer */
#define DownMixer_1_UABH_A_PARAM_COMP_MASK        0
#define DownMixer_1_UABH_A_PARAM_VDAC_EMPTY_MASK  0

#define DownMixer_1_UABH_A_NONZERO_INTR_MASK (DownMixer_1_UABH_A_Nonzero( \
    ( 0 |0) ))
#define DownMixer_1_UABH_A_DEFAULT_INTR_MASK ((uint32)(\
    (((uint32)DownMixer_1_UABH_A_PARAM_COMP_MASK)<<DownMixer_1_UABH_A_INTR_COMP_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_VDAC_EMPTY_MASK)<<DownMixer_1_UABH_A_INTR_VDAC_EMPTY_SHIFT) \
))

#define DownMixer_1_UABH_A_PARAM_UAB_PWR        2
#define DownMixer_1_UABH_A_PARAM_AGND_PTS       0
#define DownMixer_1_UABH_A_PARAM_AGND_PWR       7
#define DownMixer_1_UABH_A_PARAM_REF_PTS        0
#define DownMixer_1_UABH_A_PARAM_REF_PWR        7
#define DownMixer_1_UABH_A_PARAM_CMP_DSI_LEVEL  1
#define DownMixer_1_UABH_A_PARAM_CMP_EDGE       0
#define DownMixer_1_UABH_A_PARAM_CMP_PWR        7
#define DownMixer_1_UABH_A_PARAM_OA_PWR         7

#define DownMixer_1_UABH_A_PARAM_AGND_TIED      0
#define DownMixer_1_UABH_A_PARAM_REF_TIED       0


/*
#define DownMixer_1_UABH_A_NONZERO_OA_CTRL (DownMixer_1_UABH_A_Nonzero( \
    (2|0|7|0|\
    7||1|0|7|7) ))
*/

/* force inclusion in _initPairs */
#define DownMixer_1_UABH_A_NONZERO_OA_CTRL (DownMixer_1_UABH_A_Nonzero(1))

/*Shared switches agnd_tied and ref_tied occupy the same bit position, so ommitting from DEFAULT_OA_CTRL -
handled in Init() */
#define DownMixer_1_UABH_A_DEFAULT_OA_CTRL ((uint32)(\
    (((uint32)DownMixer_1_UABH_A_PARAM_UAB_PWR)<<DownMixer_1_UABH_A_UAB_PWR_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_AGND_PTS)<<DownMixer_1_UABH_A_AGND_PTS_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_AGND_PWR)<<DownMixer_1_UABH_A_AGND_PWR_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_REF_PTS)<<DownMixer_1_UABH_A_REF_PTS_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_REF_PWR)<<DownMixer_1_UABH_A_REF_PWR_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_CMP_DSI_LEVEL)<<DownMixer_1_UABH_A_CMP_DSI_LEVEL_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_CMP_EDGE)<<DownMixer_1_UABH_A_CMP_EDGE_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_CMP_PWR)<<DownMixer_1_UABH_A_CMP_PWR_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_OA_PWR)<<DownMixer_1_UABH_A_OA_PWR_SHIFT) \
))

#define DownMixer_1_UABH_A_PARAM_CB_GND       0
#define DownMixer_1_UABH_A_PARAM_CC_GND       0
#define DownMixer_1_UABH_A_PARAM_FRC_SIGN_BIT 0
#define DownMixer_1_UABH_A_PARAM_DAC_MODE_EN  0
#define DownMixer_1_UABH_A_PARAM_DAC_MODE     0

#define DownMixer_1_UABH_A_NONZERO_CAP_CTRL ( DownMixer_1_UABH_A_Nonzero( \
    (0|0|0|0|0) ))
#define DownMixer_1_UABH_A_DEFAULT_CAP_CTRL ((uint32)(\
    (((uint32)DownMixer_1_UABH_A_PARAM_CB_GND)<<DownMixer_1_UABH_A_CB_GND_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_CC_GND)<<DownMixer_1_UABH_A_CC_GND_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_FRC_SIGN_BIT)<<DownMixer_1_UABH_A_FRC_SIGN_BIT_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_DAC_MODE_EN)<<DownMixer_1_UABH_A_DAC_MODE_EN_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_DAC_MODE)<<DownMixer_1_UABH_A_DAC_MODE_SHIFT) \
))

#define DownMixer_1_UABH_A_PARAM_CB_VAL    5
#define DownMixer_1_UABH_A_PARAM_CA_VAL    0
#define DownMixer_1_UABH_A_PARAM_SIGN_VAL  0
#define DownMixer_1_UABH_A_PARAM_CB_64     0
#define DownMixer_1_UABH_A_PARAM_CC_VAL    5
#define DownMixer_1_UABH_A_PARAM_CF_VAL    20

#define DownMixer_1_UABH_A_NONZERO_CAP_ABCF_VAL  (DownMixer_1_UABH_A_Nonzero( \
    (5|0|0|0|5|20) ))
#define DownMixer_1_UABH_A_DEFAULT_CAP_ABCF_VAL  ((uint32)(\
    (((uint32)DownMixer_1_UABH_A_PARAM_CB_VAL)<<DownMixer_1_UABH_A_CB_VAL_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_CA_VAL)<<DownMixer_1_UABH_A_CA_VAL_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SIGN_VAL)<<DownMixer_1_UABH_A_SIGN_VAL_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_CB_64)<<DownMixer_1_UABH_A_CB_64_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_CC_VAL)<<DownMixer_1_UABH_A_CC_VAL_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_CF_VAL)<<DownMixer_1_UABH_A_CF_VAL_SHIFT) \
))

#define DownMixer_1_UABH_A_NONZERO_CAP_AB_VAL_NXT (DownMixer_1_UABH_A_Nonzero( \
    (5|0|0) ))
#define DownMixer_1_UABH_A_DEFAULT_CAP_AB_VAL_NXT  ((uint32)(\
    (((uint32)DownMixer_1_UABH_A_PARAM_CB_VAL)<<DownMixer_1_UABH_A_CB_VAL_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_CA_VAL)<<DownMixer_1_UABH_A_CA_VAL_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SIGN_VAL)<<DownMixer_1_UABH_A_SIGN_VAL_SHIFT) \
))

#define DownMixer_1_UABH_A_PARAM_CC_VAL_UPDATE  0
#define DownMixer_1_UABH_A_PARAM_CF_VAL_UPDATE  0

#define DownMixer_1_UABH_A_NONZERO_CAP_CF_VAL_NXT (DownMixer_1_UABH_A_Nonzero( \
    (0|0|5|20) ))
#define DownMixer_1_UABH_A_DEFAULT_CAP_CF_VAL_NXT ((uint32)(\
    (((uint32)DownMixer_1_UABH_A_PARAM_CC_VAL)<<DownMixer_1_UABH_A_CC_VAL_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_CF_VAL)<<DownMixer_1_UABH_A_CF_VAL_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_CC_VAL_UPDATE)<<DownMixer_1_UABH_A_CC_VAL_UPDATE_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_CF_VAL_UPDATE)<<DownMixer_1_UABH_A_CF_VAL_UPDATE_SHIFT) \
))

#define DownMixer_1_UABH_A_PARAM_SW_GX0        CyUAB_SwIsClosed( 0 )
#define DownMixer_1_UABH_A_PARAM_SW_GX1        CyUAB_SwIsClosed( 0 )
#define DownMixer_1_UABH_A_PARAM_SW_GX2        CyUAB_SwIsClosed( 0 )
#define DownMixer_1_UABH_A_PARAM_SW_GX3        CyUAB_SwIsClosed( 0 )
#define DownMixer_1_UABH_A_PARAM_SW_RG         CyUAB_SwIsClosed( 0  )
#define DownMixer_1_UABH_A_PARAM_SW_GG         CyUAB_SwIsClosed( 1  )
#define DownMixer_1_UABH_A_PARAM_SW_RT         CyUAB_SwIsClosed( 0  )
#define DownMixer_1_UABH_A_PARAM_SW_GT         CyUAB_SwIsClosed( 1  )
#define DownMixer_1_UABH_A_PARAM_SW_QT         CyUAB_SwIsClosed( 0  )
#define DownMixer_1_UABH_A_PARAM_EARLY_PS      0
#define DownMixer_1_UABH_A_PARAM_EARLY_PO      1

/*strobe source from cyfitter.h*/
#define DownMixer_1_UABH_A_PARAM_STRB_RST_SEL  (DownMixer_1_UABH_A_UnmappedIsZero(DownMixer_1_UABH_A_halfuab__STRB_RST_SEL))

#define DownMixer_1_UABH_A_PARAM_STRB_RST_EN   0


/*if switch parameter references a bad x input, ignored in initialization*/
#define DownMixer_1_UABH_A_XIN_OK(xin) ( (0UL==((uint32)(xin))) || (1UL==((uint32)(xin))) || (2UL==((uint32)(xin))) || (3UL==((uint32)(xin))) )
#define DownMixer_1_UABH_A_IGNORE_VAL 0UL      
#define DownMixer_1_UABH_A_XField(xin,val)  ( DownMixer_1_UABH_A_XIN_OK((xin)) ? ((uint32)(val)) : DownMixer_1_UABH_A_IGNORE_VAL )

/*
#define DownMixer_1_UABH_A_NONZERO_SW_STATIC (DownMixer_1_UABH_A_Nonzero( \
    (0|0|0|0|0|1|0|1|0|0|1|0|0) ))
*/

/* force inclusion in _initPairs */
#define DownMixer_1_UABH_A_NONZERO_SW_STATIC (DownMixer_1_UABH_A_Nonzero(1))

#define DownMixer_1_UABH_A_DEFAULT_SW_STATIC ((uint32)(\
    DownMixer_1_UABH_A_XField(DownMixer_1_UABH_A_X0,DownMixer_1_UABH_A_PARAM_SW_GX0<<DownMixer_1_UABH_A_StaticVShift(CyUAB_SW_GX0)) |\
    DownMixer_1_UABH_A_XField(DownMixer_1_UABH_A_X1,DownMixer_1_UABH_A_PARAM_SW_GX1<<DownMixer_1_UABH_A_StaticVShift(CyUAB_SW_GX1)) |\
    DownMixer_1_UABH_A_XField(DownMixer_1_UABH_A_X2,DownMixer_1_UABH_A_PARAM_SW_GX2<<DownMixer_1_UABH_A_StaticVShift(CyUAB_SW_GX2)) |\
    DownMixer_1_UABH_A_XField(DownMixer_1_UABH_A_X3,DownMixer_1_UABH_A_PARAM_SW_GX3<<DownMixer_1_UABH_A_StaticVShift(CyUAB_SW_GX3)) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_RG)<<DownMixer_1_UABH_A_SW_RG_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_GG)<<DownMixer_1_UABH_A_SW_GG_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_RT)<<DownMixer_1_UABH_A_SW_RT_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_GT)<<DownMixer_1_UABH_A_SW_GT_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_QT)<<DownMixer_1_UABH_A_SW_QT_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_EARLY_PS)<<DownMixer_1_UABH_A_EARLY_PS_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_EARLY_PO)<<DownMixer_1_UABH_A_EARLY_PO_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_STRB_RST_SEL)<<DownMixer_1_UABH_A_STRB_RST_SEL_SHIFT) | \
    (((uint32)DownMixer_1_UABH_A_PARAM_STRB_RST_EN)<<DownMixer_1_UABH_A_STRB_RST_EN_SHIFT) \
))

/*modbitab source from cyfitter.h*/
#define DownMixer_1_UABH_A_PARAM_MODBIT0_SRC_SEL   (DownMixer_1_UABH_A_UnmappedIsZero(DownMixer_1_UABH_A_halfuab__MODBIT0_SEL))
/*modbitc source from cyfitter.h*/
#define DownMixer_1_UABH_A_PARAM_MODBIT1_SRC_SEL   (DownMixer_1_UABH_A_UnmappedIsZero(DownMixer_1_UABH_A_halfuab__MODBIT1_SEL))

/*
#define DownMixer_1_UABH_A_NONZERO_SW_MODBIT_SRC (DownMixer_1_UABH_A_Nonzero( \
    (0|0) ))
*/
/* force inclusion in _initPairs */
#define DownMixer_1_UABH_A_NONZERO_SW_MODBIT_SRC (DownMixer_1_UABH_A_Nonzero(1))

#define DownMixer_1_UABH_A_DEFAULT_SW_MODBIT_SRC ((uint32)(\
    (((uint32)DownMixer_1_UABH_A_PARAM_MODBIT0_SRC_SEL)<<DownMixer_1_UABH_A_MODBIT0_SRC_SEL_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_MODBIT1_SRC_SEL)<<DownMixer_1_UABH_A_MODBIT1_SRC_SEL_SHIFT) \
))


    
#define DownMixer_1_UABH_A_PARAM_SW_AX0_MODBIT  0
#define DownMixer_1_UABH_A_PARAM_SW_AX1_MODBIT  0
#define DownMixer_1_UABH_A_PARAM_SW_AX2_MODBIT  0
#define DownMixer_1_UABH_A_PARAM_SW_AX3_MODBIT  0
#define DownMixer_1_UABH_A_PARAM_SW_AP          0
#define DownMixer_1_UABH_A_PARAM_SW_AQ          0

#define DownMixer_1_UABH_A_NONZERO_SW_CA_IN0 (DownMixer_1_UABH_A_Nonzero( \
    (0|0|0|0|0|0) ))
#define DownMixer_1_UABH_A_DEFAULT_SW_CA_IN0 ((uint32)(\
    DownMixer_1_UABH_A_XField(DownMixer_1_UABH_A_X0,DownMixer_1_UABH_A_PARAM_SW_AX0_MODBIT<<DownMixer_1_UABH_A_DynamicVShift(CyUAB_SW_AX0_MODBIT)) |\
    DownMixer_1_UABH_A_XField(DownMixer_1_UABH_A_X1,DownMixer_1_UABH_A_PARAM_SW_AX1_MODBIT<<DownMixer_1_UABH_A_DynamicVShift(CyUAB_SW_AX1_MODBIT)) |\
    DownMixer_1_UABH_A_XField(DownMixer_1_UABH_A_X2,DownMixer_1_UABH_A_PARAM_SW_AX2_MODBIT<<DownMixer_1_UABH_A_DynamicVShift(CyUAB_SW_AX2_MODBIT)) |\
    DownMixer_1_UABH_A_XField(DownMixer_1_UABH_A_X3,DownMixer_1_UABH_A_PARAM_SW_AX3_MODBIT<<DownMixer_1_UABH_A_DynamicVShift(CyUAB_SW_AX3_MODBIT)) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_AP)<<DownMixer_1_UABH_A_SW_AP_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_AQ)<<DownMixer_1_UABH_A_SW_AQ_SHIFT) \
))

#define DownMixer_1_UABH_A_PARAM_SW_AA         0
#define DownMixer_1_UABH_A_PARAM_SW_AR_MODBIT  0
#define DownMixer_1_UABH_A_PARAM_SW_AG_MODBIT  0
#define DownMixer_1_UABH_A_PARAM_SW_AV_MODBIT  0

/*Shared switch AA intentionally ommitted - handled in Init()*/
#define DownMixer_1_UABH_A_NONZERO_SW_CA_IN1 (DownMixer_1_UABH_A_Nonzero( \
    (0|0|0) ))
#define DownMixer_1_UABH_A_DEFAULT_SW_CA_IN1 ((uint32)(\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_AR_MODBIT)<<DownMixer_1_UABH_A_SW_AR_MODBIT_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_AG_MODBIT)<<DownMixer_1_UABH_A_SW_AG_MODBIT_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_AV_MODBIT)<<DownMixer_1_UABH_A_SW_AV_MODBIT_SHIFT) \
))

#define DownMixer_1_UABH_A_PARAM_SW_RA_MODBIT  0
#define DownMixer_1_UABH_A_PARAM_SW_GA_MODBIT  0
#define DownMixer_1_UABH_A_PARAM_SW_VA_MODBIT  0
#define DownMixer_1_UABH_A_PARAM_SW_SA         0

#define DownMixer_1_UABH_A_NONZERO_SW_CA_TOP (DownMixer_1_UABH_A_Nonzero( \
    (0|0|0|0) ))
#define DownMixer_1_UABH_A_DEFAULT_SW_CA_TOP ((uint32)(\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_RA_MODBIT)<<DownMixer_1_UABH_A_SW_RA_MODBIT_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_GA_MODBIT)<<DownMixer_1_UABH_A_SW_GA_MODBIT_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_VA_MODBIT)<<DownMixer_1_UABH_A_SW_VA_MODBIT_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_SA)<<DownMixer_1_UABH_A_SW_SA_SHIFT) \
))

#define DownMixer_1_UABH_A_PARAM_SW_BX0_MODBIT 1
#define DownMixer_1_UABH_A_PARAM_SW_BX1_MODBIT 0
#define DownMixer_1_UABH_A_PARAM_SW_BX2_MODBIT 0
#define DownMixer_1_UABH_A_PARAM_SW_BX3_MODBIT 0
#define DownMixer_1_UABH_A_PARAM_SW_BP         2
#define DownMixer_1_UABH_A_PARAM_SW_BQ         0

#define DownMixer_1_UABH_A_NONZERO_SW_CB_IN0 (DownMixer_1_UABH_A_Nonzero( \
    (1|0|0|0|2|0) ))
#define DownMixer_1_UABH_A_DEFAULT_SW_CB_IN0 ((uint32)(\
    DownMixer_1_UABH_A_XField(DownMixer_1_UABH_A_X0,DownMixer_1_UABH_A_PARAM_SW_BX0_MODBIT<<DownMixer_1_UABH_A_DynamicVShift(CyUAB_SW_BX0_MODBIT)) |\
    DownMixer_1_UABH_A_XField(DownMixer_1_UABH_A_X1,DownMixer_1_UABH_A_PARAM_SW_BX1_MODBIT<<DownMixer_1_UABH_A_DynamicVShift(CyUAB_SW_BX1_MODBIT)) |\
    DownMixer_1_UABH_A_XField(DownMixer_1_UABH_A_X2,DownMixer_1_UABH_A_PARAM_SW_BX2_MODBIT<<DownMixer_1_UABH_A_DynamicVShift(CyUAB_SW_BX2_MODBIT)) |\
    DownMixer_1_UABH_A_XField(DownMixer_1_UABH_A_X3,DownMixer_1_UABH_A_PARAM_SW_BX3_MODBIT<<DownMixer_1_UABH_A_DynamicVShift(CyUAB_SW_BX3_MODBIT)) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_BP)<<DownMixer_1_UABH_A_SW_BP_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_BQ)<<DownMixer_1_UABH_A_SW_BQ_SHIFT) \
))

#define DownMixer_1_UABH_A_PARAM_SW_BB        0
#define DownMixer_1_UABH_A_PARAM_SW_BR_MODBIT 0
#define DownMixer_1_UABH_A_PARAM_SW_BG_MODBIT 0
#define DownMixer_1_UABH_A_PARAM_SW_BV_MODBIT 0

/*Shared switch BB intentionally ommitted - handled in Init()*/
#define DownMixer_1_UABH_A_NONZERO_SW_CB_IN1 (DownMixer_1_UABH_A_Nonzero( \
    (0|0|0) ))
#define DownMixer_1_UABH_A_DEFAULT_SW_CB_IN1 ((uint32)(\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_BR_MODBIT)<<DownMixer_1_UABH_A_SW_BR_MODBIT_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_BG_MODBIT)<<DownMixer_1_UABH_A_SW_BG_MODBIT_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_BV_MODBIT)<<DownMixer_1_UABH_A_SW_BV_MODBIT_SHIFT) \
))

#define DownMixer_1_UABH_A_PARAM_SW_RB_MODBIT 0
#define DownMixer_1_UABH_A_PARAM_SW_GB_MODBIT 1
#define DownMixer_1_UABH_A_PARAM_SW_VB_MODBIT 0
#define DownMixer_1_UABH_A_PARAM_SW_TB        15
#define DownMixer_1_UABH_A_PARAM_SW_SB        2

#define DownMixer_1_UABH_A_NONZERO_SW_CB_TOP (DownMixer_1_UABH_A_Nonzero( \
    (0|1|0|15|2) ))
#define DownMixer_1_UABH_A_DEFAULT_SW_CB_TOP ((uint32)(\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_RB_MODBIT)<<DownMixer_1_UABH_A_SW_RB_MODBIT_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_GB_MODBIT)<<DownMixer_1_UABH_A_SW_GB_MODBIT_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_VB_MODBIT)<<DownMixer_1_UABH_A_SW_VB_MODBIT_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_TB)<<DownMixer_1_UABH_A_SW_TB_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_SB)<<DownMixer_1_UABH_A_SW_SB_SHIFT) \
))

#define DownMixer_1_UABH_A_PARAM_SW_CX0_MODBIT 2
#define DownMixer_1_UABH_A_PARAM_SW_CX1_MODBIT 0
#define DownMixer_1_UABH_A_PARAM_SW_CX2_MODBIT 0
#define DownMixer_1_UABH_A_PARAM_SW_CX3_MODBIT 0
#define DownMixer_1_UABH_A_PARAM_SW_CP         1
#define DownMixer_1_UABH_A_PARAM_SW_CQ         0
    
#define DownMixer_1_UABH_A_NONZERO_SW_CC_IN0 (DownMixer_1_UABH_A_Nonzero( \
    (2|0|0|0|1|0) ))
#define DownMixer_1_UABH_A_DEFAULT_SW_CC_IN0 ((uint32)(\
    DownMixer_1_UABH_A_XField(DownMixer_1_UABH_A_X0,DownMixer_1_UABH_A_PARAM_SW_CX0_MODBIT<<DownMixer_1_UABH_A_DynamicVShift(CyUAB_SW_CX0_MODBIT)) |\
    DownMixer_1_UABH_A_XField(DownMixer_1_UABH_A_X1,DownMixer_1_UABH_A_PARAM_SW_CX1_MODBIT<<DownMixer_1_UABH_A_DynamicVShift(CyUAB_SW_CX1_MODBIT)) |\
    DownMixer_1_UABH_A_XField(DownMixer_1_UABH_A_X2,DownMixer_1_UABH_A_PARAM_SW_CX2_MODBIT<<DownMixer_1_UABH_A_DynamicVShift(CyUAB_SW_CX2_MODBIT)) |\
    DownMixer_1_UABH_A_XField(DownMixer_1_UABH_A_X3,DownMixer_1_UABH_A_PARAM_SW_CX3_MODBIT<<DownMixer_1_UABH_A_DynamicVShift(CyUAB_SW_CX3_MODBIT)) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_CP)<<DownMixer_1_UABH_A_SW_CP_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_CQ)<<DownMixer_1_UABH_A_SW_CQ_SHIFT) \
))

#define DownMixer_1_UABH_A_PARAM_SW_CC        0
#define DownMixer_1_UABH_A_PARAM_SW_CR_MODBIT 0
#define DownMixer_1_UABH_A_PARAM_SW_CG_MODBIT 0
#define DownMixer_1_UABH_A_PARAM_SW_CV_MODBIT 0

/*Shared switch CC intentionally ommitted - handled in Init()*/
#define DownMixer_1_UABH_A_NONZERO_SW_CC_IN1 (DownMixer_1_UABH_A_Nonzero( \
    (0|0|0) ))
#define DownMixer_1_UABH_A_DEFAULT_SW_CC_IN1 ((uint32)(\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_CR_MODBIT)<<DownMixer_1_UABH_A_SW_CR_MODBIT_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_CG_MODBIT)<<DownMixer_1_UABH_A_SW_CG_MODBIT_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_CV_MODBIT)<<DownMixer_1_UABH_A_SW_CV_MODBIT_SHIFT) \
))

#define DownMixer_1_UABH_A_PARAM_SW_RC_MODBIT 0
#define DownMixer_1_UABH_A_PARAM_SW_GC_MODBIT 2
#define DownMixer_1_UABH_A_PARAM_SW_VC_MODBIT 0
#define DownMixer_1_UABH_A_PARAM_SW_TC        15
#define DownMixer_1_UABH_A_PARAM_SW_SC        1
#define DownMixer_1_UABH_A_PARAM_SW_ZC        0

#define DownMixer_1_UABH_A_NONZERO_SW_CC_TOP (DownMixer_1_UABH_A_Nonzero( \
    (0|2|0|15|1|0) ))
#define DownMixer_1_UABH_A_DEFAULT_SW_CC_TOP ((uint32)(\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_RC_MODBIT)<<DownMixer_1_UABH_A_SW_RC_MODBIT_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_GC_MODBIT)<<DownMixer_1_UABH_A_SW_GC_MODBIT_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_VC_MODBIT)<<DownMixer_1_UABH_A_SW_VC_MODBIT_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_TC)<<DownMixer_1_UABH_A_SW_TC_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_SC)<<DownMixer_1_UABH_A_SW_SC_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_ZC)<<DownMixer_1_UABH_A_SW_ZC_SHIFT) \
))

#define DownMixer_1_UABH_A_PARAM_SW_GF 0
#define DownMixer_1_UABH_A_PARAM_SW_PF 15
#define DownMixer_1_UABH_A_PARAM_SW_PS 0
#define DownMixer_1_UABH_A_PARAM_SW_PO 3

#define DownMixer_1_UABH_A_NONZERO_SW_CF_BOT (DownMixer_1_UABH_A_Nonzero( \
    (0|15|0|3) ))
#define DownMixer_1_UABH_A_DEFAULT_SW_CF_BOT ((uint32)(\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_GF)<<DownMixer_1_UABH_A_SW_GF_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_PF)<<DownMixer_1_UABH_A_SW_PF_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_PS)<<DownMixer_1_UABH_A_SW_PS_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SW_PO)<<DownMixer_1_UABH_A_SW_PO_SHIFT) \
))

#define DownMixer_1_UABH_A_PARAM_CMP_FF     15
#define DownMixer_1_UABH_A_PARAM_VALID_WAVE 15
#define DownMixer_1_UABH_A_PARAM_TRIG_OUT   0
#define DownMixer_1_UABH_A_PARAM_STROBE_SW  0
#define DownMixer_1_UABH_A_PARAM_STROBE_RST 0

#define DownMixer_1_UABH_A_NONZERO_SW_OTHER (DownMixer_1_UABH_A_Nonzero( \
    (15|15|0|0|0) ))
#define DownMixer_1_UABH_A_DEFAULT_SW_OTHER ((uint32)(\
    (((uint32)DownMixer_1_UABH_A_PARAM_CMP_FF)<<DownMixer_1_UABH_A_CMP_FF_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_VALID_WAVE)<<DownMixer_1_UABH_A_VALID_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_TRIG_OUT)<<DownMixer_1_UABH_A_TRIG_OUT_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_STROBE_SW)<<DownMixer_1_UABH_A_STROBE_SW_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_STROBE_RST)<<DownMixer_1_UABH_A_STROBE_RST_SHIFT) \
))

#define DownMixer_1_UABH_A_PARAM_CA_BOOST  0
#define DownMixer_1_UABH_A_PARAM_CB_BOOST  1
#define DownMixer_1_UABH_A_PARAM_CC_BOOST  2
#define DownMixer_1_UABH_A_PARAM_CF_BOOST  2
#define DownMixer_1_UABH_A_PARAM_SUM_BOOST 2
#define DownMixer_1_UABH_A_PARAM_PUMP_WAVE 1
    
#define DownMixer_1_UABH_A_NONZERO_SW_BOOST_CTRL (DownMixer_1_UABH_A_Nonzero( \
    (0|1|2|2|2|1) ))
#define DownMixer_1_UABH_A_DEFAULT_SW_BOOST_CTRL ((uint32)(\
    (((uint32)DownMixer_1_UABH_A_PARAM_CA_BOOST)<<DownMixer_1_UABH_A_CA_BOOST_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_CB_BOOST)<<DownMixer_1_UABH_A_CB_BOOST_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_CC_BOOST)<<DownMixer_1_UABH_A_CC_BOOST_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_CF_BOOST)<<DownMixer_1_UABH_A_CF_BOOST_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SUM_BOOST)<<DownMixer_1_UABH_A_SUM_BOOST_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_PUMP_WAVE)<<DownMixer_1_UABH_A_PUMP_WAVE_SHIFT) \
))

#define DownMixer_1_UABH_A_PARAM_STARTUP_DELAY 0
#define DownMixer_1_UABH_A_PARAM_ALIGN_MODE    0

#define DownMixer_1_UABH_A_NONZERO_STARTUP_DELAY (DownMixer_1_UABH_A_Nonzero( \
	(0|0) ))
#define DownMixer_1_UABH_A_DEFAULT_STARTUP_DELAY ((uint32)(\
    (((uint32)DownMixer_1_UABH_A_PARAM_STARTUP_DELAY)<<DownMixer_1_UABH_A_STARTUP_DELAY_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_ALIGN_MODE)<<DownMixer_1_UABH_A_ALIGN_MODE_SHIFT) \
))

#define DownMixer_1_UABH_A_PARAM_SUBSAMPLE 0    
#define DownMixer_1_UABH_A_PARAM_SUBSAMPLE_INIT 0    
    
#define DownMixer_1_UABH_A_NONZERO_SUBSAMPLE_CTRL (DownMixer_1_UABH_A_Nonzero( \
    (0|0) ))
#define DownMixer_1_UABH_A_DEFAULT_SUBSAMPLE_CTRL ((uint32)(\
    (((uint32)DownMixer_1_UABH_A_PARAM_SUBSAMPLE)<<DownMixer_1_UABH_A_SUBSAMPLE_SHIFT) |\
    (((uint32)DownMixer_1_UABH_A_PARAM_SUBSAMPLE_INIT)<<DownMixer_1_UABH_A_SUBSAMPLE_INIT_SHIFT) \
))
    
#define DownMixer_1_UABH_A_PARAM_LAST_STEP  1

/*trigger_in source from cyfitter.h*/
#define DownMixer_1_UABH_A_PARAM_TRIG_SEL    (DownMixer_1_UABH_A_UnmappedIsZero(DownMixer_1_UABH_A_halfuab__TRIG_SEL))

#define DownMixer_1_UABH_A_PARAM_TRIGGER_EN 0

/*
#define DownMixer_1_UABH_A_NONZERO_SRAM_CTRL (DownMixer_1_UABH_A_Nonzero( \
    (1|0|0) ))
*/
/* force inclusion in _initPairs */
#define DownMixer_1_UABH_A_NONZERO_SRAM_CTRL (DownMixer_1_UABH_A_Nonzero(1))    
 
#define DownMixer_1_UABH_A_DEFAULT_SRAM_CTRL ((uint32)(\
    (((uint32)DownMixer_1_UABH_A_PARAM_LAST_STEP)<<DownMixer_1_UABH_A_LAST_STEP_SHIFT) | \
    (((uint32)DownMixer_1_UABH_A_PARAM_TRIG_SEL)<<DownMixer_1_UABH_A_TRIG_SEL_SHIFT)   | \
    (((uint32)DownMixer_1_UABH_A_PARAM_TRIGGER_EN)<<DownMixer_1_UABH_A_TRIGGER_EN_SHIFT) \
))
/*There is intentionally no run bit symbol parameter and the run bit is not set as part of the Init() function*/

#define DownMixer_1_UABH_A_PARAM_WAVE_STEP_0  5
#define DownMixer_1_UABH_A_PARAM_WAVE_STEP_1  6
#define DownMixer_1_UABH_A_PARAM_WAVE_STEP_2  0
#define DownMixer_1_UABH_A_PARAM_WAVE_STEP_3  0
#define DownMixer_1_UABH_A_PARAM_WAVE_STEP_4  0
#define DownMixer_1_UABH_A_PARAM_WAVE_STEP_5  0
#define DownMixer_1_UABH_A_PARAM_WAVE_STEP_6  0
#define DownMixer_1_UABH_A_PARAM_WAVE_STEP_7  0
#define DownMixer_1_UABH_A_PARAM_WAVE_STEP_8  0
#define DownMixer_1_UABH_A_PARAM_WAVE_STEP_9  0
#define DownMixer_1_UABH_A_PARAM_WAVE_STEP_10 0
#define DownMixer_1_UABH_A_PARAM_WAVE_STEP_11 0
#define DownMixer_1_UABH_A_PARAM_WAVE_STEP_12 0
#define DownMixer_1_UABH_A_PARAM_WAVE_STEP_13 0
#define DownMixer_1_UABH_A_PARAM_WAVE_STEP_14 0
#define DownMixer_1_UABH_A_PARAM_WAVE_STEP_15 0

#define DownMixer_1_UABH_A_NONZERO_WAVE_0  (DownMixer_1_UABH_A_Nonzero( DownMixer_1_UABH_A_PARAM_WAVE_STEP_0 ) )
#define DownMixer_1_UABH_A_NONZERO_WAVE_1  (DownMixer_1_UABH_A_Nonzero( DownMixer_1_UABH_A_PARAM_WAVE_STEP_1 ) )
#define DownMixer_1_UABH_A_NONZERO_WAVE_2  (DownMixer_1_UABH_A_Nonzero( DownMixer_1_UABH_A_PARAM_WAVE_STEP_2 ) )
#define DownMixer_1_UABH_A_NONZERO_WAVE_3  (DownMixer_1_UABH_A_Nonzero( DownMixer_1_UABH_A_PARAM_WAVE_STEP_3 ) )
#define DownMixer_1_UABH_A_NONZERO_WAVE_4  (DownMixer_1_UABH_A_Nonzero( DownMixer_1_UABH_A_PARAM_WAVE_STEP_4 ) )
#define DownMixer_1_UABH_A_NONZERO_WAVE_5  (DownMixer_1_UABH_A_Nonzero( DownMixer_1_UABH_A_PARAM_WAVE_STEP_5 ) )
#define DownMixer_1_UABH_A_NONZERO_WAVE_6  (DownMixer_1_UABH_A_Nonzero( DownMixer_1_UABH_A_PARAM_WAVE_STEP_6 ) )
#define DownMixer_1_UABH_A_NONZERO_WAVE_7  (DownMixer_1_UABH_A_Nonzero( DownMixer_1_UABH_A_PARAM_WAVE_STEP_7 ) )
#define DownMixer_1_UABH_A_NONZERO_WAVE_8  (DownMixer_1_UABH_A_Nonzero( DownMixer_1_UABH_A_PARAM_WAVE_STEP_8 ) )
#define DownMixer_1_UABH_A_NONZERO_WAVE_9  (DownMixer_1_UABH_A_Nonzero( DownMixer_1_UABH_A_PARAM_WAVE_STEP_9 ) )
#define DownMixer_1_UABH_A_NONZERO_WAVE_10 (DownMixer_1_UABH_A_Nonzero( DownMixer_1_UABH_A_PARAM_WAVE_STEP_10 ) )
#define DownMixer_1_UABH_A_NONZERO_WAVE_11 (DownMixer_1_UABH_A_Nonzero( DownMixer_1_UABH_A_PARAM_WAVE_STEP_11 ) )
#define DownMixer_1_UABH_A_NONZERO_WAVE_12 (DownMixer_1_UABH_A_Nonzero( DownMixer_1_UABH_A_PARAM_WAVE_STEP_12 ) )
#define DownMixer_1_UABH_A_NONZERO_WAVE_13 (DownMixer_1_UABH_A_Nonzero( DownMixer_1_UABH_A_PARAM_WAVE_STEP_13 ) )
#define DownMixer_1_UABH_A_NONZERO_WAVE_14 (DownMixer_1_UABH_A_Nonzero( DownMixer_1_UABH_A_PARAM_WAVE_STEP_14 ) )
#define DownMixer_1_UABH_A_NONZERO_WAVE_15 (DownMixer_1_UABH_A_Nonzero( DownMixer_1_UABH_A_PARAM_WAVE_STEP_15 ) )


/*all wave steps up to and including the last non-zero wave step 
    are included in the waveConfig array definition */
/*always include WAVE_STEP_0, since LAST_STEP minimum is 0*/
#define DownMixer_1_UABH_A_INC_STEP_0 (DownMixer_1_UABH_A_Nonzero(1))

#define DownMixer_1_UABH_A_INC_STEP_1  ( \
    DownMixer_1_UABH_A_NONZERO_WAVE_1  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_2  || DownMixer_1_UABH_A_NONZERO_WAVE_3  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_4  || DownMixer_1_UABH_A_NONZERO_WAVE_5  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_6  || DownMixer_1_UABH_A_NONZERO_WAVE_7  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_8  || DownMixer_1_UABH_A_NONZERO_WAVE_9  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_10 || DownMixer_1_UABH_A_NONZERO_WAVE_11 || \
    DownMixer_1_UABH_A_NONZERO_WAVE_12 || DownMixer_1_UABH_A_NONZERO_WAVE_13 || \
    DownMixer_1_UABH_A_NONZERO_WAVE_14 || DownMixer_1_UABH_A_NONZERO_WAVE_15 )
    
#define DownMixer_1_UABH_A_INC_STEP_2  ( \
    DownMixer_1_UABH_A_NONZERO_WAVE_2  || DownMixer_1_UABH_A_NONZERO_WAVE_3  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_4  || DownMixer_1_UABH_A_NONZERO_WAVE_5  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_6  || DownMixer_1_UABH_A_NONZERO_WAVE_7  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_8  || DownMixer_1_UABH_A_NONZERO_WAVE_9  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_10 || DownMixer_1_UABH_A_NONZERO_WAVE_11 || \
    DownMixer_1_UABH_A_NONZERO_WAVE_12 || DownMixer_1_UABH_A_NONZERO_WAVE_13 || \
    DownMixer_1_UABH_A_NONZERO_WAVE_14 || DownMixer_1_UABH_A_NONZERO_WAVE_15 )
    
#define DownMixer_1_UABH_A_INC_STEP_3  ( \
    DownMixer_1_UABH_A_NONZERO_WAVE_3  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_4  || DownMixer_1_UABH_A_NONZERO_WAVE_5  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_6  || DownMixer_1_UABH_A_NONZERO_WAVE_7  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_8  || DownMixer_1_UABH_A_NONZERO_WAVE_9  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_10 || DownMixer_1_UABH_A_NONZERO_WAVE_11 || \
    DownMixer_1_UABH_A_NONZERO_WAVE_12 || DownMixer_1_UABH_A_NONZERO_WAVE_13 || \
    DownMixer_1_UABH_A_NONZERO_WAVE_14 || DownMixer_1_UABH_A_NONZERO_WAVE_15 )
    
#define DownMixer_1_UABH_A_INC_STEP_4  ( \
    DownMixer_1_UABH_A_NONZERO_WAVE_4  || DownMixer_1_UABH_A_NONZERO_WAVE_5  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_6  || DownMixer_1_UABH_A_NONZERO_WAVE_7  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_8  || DownMixer_1_UABH_A_NONZERO_WAVE_9  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_10 || DownMixer_1_UABH_A_NONZERO_WAVE_11 || \
    DownMixer_1_UABH_A_NONZERO_WAVE_12 || DownMixer_1_UABH_A_NONZERO_WAVE_13 || \
    DownMixer_1_UABH_A_NONZERO_WAVE_14 || DownMixer_1_UABH_A_NONZERO_WAVE_15 )

#define DownMixer_1_UABH_A_INC_STEP_5  ( \
    DownMixer_1_UABH_A_NONZERO_WAVE_5  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_6  || DownMixer_1_UABH_A_NONZERO_WAVE_7  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_8  || DownMixer_1_UABH_A_NONZERO_WAVE_9  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_10 || DownMixer_1_UABH_A_NONZERO_WAVE_11 || \
    DownMixer_1_UABH_A_NONZERO_WAVE_12 || DownMixer_1_UABH_A_NONZERO_WAVE_13 || \
    DownMixer_1_UABH_A_NONZERO_WAVE_14 || DownMixer_1_UABH_A_NONZERO_WAVE_15 )

#define DownMixer_1_UABH_A_INC_STEP_6  ( \
    DownMixer_1_UABH_A_NONZERO_WAVE_6  || DownMixer_1_UABH_A_NONZERO_WAVE_7  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_8  || DownMixer_1_UABH_A_NONZERO_WAVE_9  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_10 || DownMixer_1_UABH_A_NONZERO_WAVE_11 || \
    DownMixer_1_UABH_A_NONZERO_WAVE_12 || DownMixer_1_UABH_A_NONZERO_WAVE_13 || \
    DownMixer_1_UABH_A_NONZERO_WAVE_14 || DownMixer_1_UABH_A_NONZERO_WAVE_15 )
    
#define DownMixer_1_UABH_A_INC_STEP_7  ( \
    DownMixer_1_UABH_A_NONZERO_WAVE_7  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_8  || DownMixer_1_UABH_A_NONZERO_WAVE_9  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_10 || DownMixer_1_UABH_A_NONZERO_WAVE_11 || \
    DownMixer_1_UABH_A_NONZERO_WAVE_12 || DownMixer_1_UABH_A_NONZERO_WAVE_13 || \
    DownMixer_1_UABH_A_NONZERO_WAVE_14 || DownMixer_1_UABH_A_NONZERO_WAVE_15 )
    
#define DownMixer_1_UABH_A_INC_STEP_8  ( \
    DownMixer_1_UABH_A_NONZERO_WAVE_8  || DownMixer_1_UABH_A_NONZERO_WAVE_9  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_10 || DownMixer_1_UABH_A_NONZERO_WAVE_11 || \
    DownMixer_1_UABH_A_NONZERO_WAVE_12 || DownMixer_1_UABH_A_NONZERO_WAVE_13 || \
    DownMixer_1_UABH_A_NONZERO_WAVE_14 || DownMixer_1_UABH_A_NONZERO_WAVE_15 )
    
#define DownMixer_1_UABH_A_INC_STEP_9  ( \
    DownMixer_1_UABH_A_NONZERO_WAVE_9  || \
    DownMixer_1_UABH_A_NONZERO_WAVE_10 || DownMixer_1_UABH_A_NONZERO_WAVE_11 || \
    DownMixer_1_UABH_A_NONZERO_WAVE_12 || DownMixer_1_UABH_A_NONZERO_WAVE_13 || \
    DownMixer_1_UABH_A_NONZERO_WAVE_14 || DownMixer_1_UABH_A_NONZERO_WAVE_15 )
    
#define DownMixer_1_UABH_A_INC_STEP_10  ( \
    DownMixer_1_UABH_A_NONZERO_WAVE_10 || DownMixer_1_UABH_A_NONZERO_WAVE_11 || \
    DownMixer_1_UABH_A_NONZERO_WAVE_12 || DownMixer_1_UABH_A_NONZERO_WAVE_13 || \
    DownMixer_1_UABH_A_NONZERO_WAVE_14 || DownMixer_1_UABH_A_NONZERO_WAVE_15 )
    
#define DownMixer_1_UABH_A_INC_STEP_11  ( \
    DownMixer_1_UABH_A_NONZERO_WAVE_11 || \
    DownMixer_1_UABH_A_NONZERO_WAVE_12 || DownMixer_1_UABH_A_NONZERO_WAVE_13 || \
    DownMixer_1_UABH_A_NONZERO_WAVE_14 || DownMixer_1_UABH_A_NONZERO_WAVE_15 )
    
#define DownMixer_1_UABH_A_INC_STEP_12  ( \
    DownMixer_1_UABH_A_NONZERO_WAVE_12 || DownMixer_1_UABH_A_NONZERO_WAVE_13 || \
    DownMixer_1_UABH_A_NONZERO_WAVE_14 || DownMixer_1_UABH_A_NONZERO_WAVE_15 )
    
#define DownMixer_1_UABH_A_INC_STEP_13  ( \
    DownMixer_1_UABH_A_NONZERO_WAVE_13 || \
    DownMixer_1_UABH_A_NONZERO_WAVE_14 || DownMixer_1_UABH_A_NONZERO_WAVE_15 )
    
#define DownMixer_1_UABH_A_INC_STEP_14  ( \
    DownMixer_1_UABH_A_NONZERO_WAVE_14 || DownMixer_1_UABH_A_NONZERO_WAVE_15 )
    
#define DownMixer_1_UABH_A_INC_STEP_15  ( DownMixer_1_UABH_A_NONZERO_WAVE_15 )
    
    
/*! @} group_init */

#endif /* #ifndef DownMixer_1_UABH_A_PARAM_H */
    
/*! @endcond */

/* [] END OF FILE */
