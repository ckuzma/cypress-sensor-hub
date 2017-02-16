/*!*****************************************************************************
* \file Rectifier_UABH_B_param.h
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

#if !defined(Rectifier_UABH_B_PARAM_H)
#define Rectifier_UABH_B_PARAM_H

#include "Rectifier_UABH_B_CyUAB_types.h"
#include "Rectifier_UABH_B_regs.h"
#include "cytypes.h"
#include "cyfitter.h"

#define Rectifier_UABH_B_Nonzero(x) ( (x) != 0 )

#define Rectifier_UABH_B_UnmappedIsZero(x)  ((uint32)(((x)==-1)? 0 : (x)))

/*!
* \addtogroup group_init
* @{
*/
/*! Component Parameters set in the component customizer */
#define Rectifier_UABH_B_PARAM_COMP_MASK        0
#define Rectifier_UABH_B_PARAM_VDAC_EMPTY_MASK  0

#define Rectifier_UABH_B_NONZERO_INTR_MASK (Rectifier_UABH_B_Nonzero( \
    ( 0 |0) ))
#define Rectifier_UABH_B_DEFAULT_INTR_MASK ((uint32)(\
    (((uint32)Rectifier_UABH_B_PARAM_COMP_MASK)<<Rectifier_UABH_B_INTR_COMP_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_VDAC_EMPTY_MASK)<<Rectifier_UABH_B_INTR_VDAC_EMPTY_SHIFT) \
))

#define Rectifier_UABH_B_PARAM_UAB_PWR        2
#define Rectifier_UABH_B_PARAM_AGND_PTS       0
#define Rectifier_UABH_B_PARAM_AGND_PWR       7
#define Rectifier_UABH_B_PARAM_REF_PTS        0
#define Rectifier_UABH_B_PARAM_REF_PWR        7
#define Rectifier_UABH_B_PARAM_CMP_DSI_LEVEL  0
#define Rectifier_UABH_B_PARAM_CMP_EDGE       0
#define Rectifier_UABH_B_PARAM_CMP_PWR        7
#define Rectifier_UABH_B_PARAM_OA_PWR         7

#define Rectifier_UABH_B_PARAM_AGND_TIED      0
#define Rectifier_UABH_B_PARAM_REF_TIED       0


/*
#define Rectifier_UABH_B_NONZERO_OA_CTRL (Rectifier_UABH_B_Nonzero( \
    (2|0|7|0|\
    7||0|0|7|7) ))
*/

/* force inclusion in _initPairs */
#define Rectifier_UABH_B_NONZERO_OA_CTRL (Rectifier_UABH_B_Nonzero(1))

/*Shared switches agnd_tied and ref_tied occupy the same bit position, so ommitting from DEFAULT_OA_CTRL -
handled in Init() */
#define Rectifier_UABH_B_DEFAULT_OA_CTRL ((uint32)(\
    (((uint32)Rectifier_UABH_B_PARAM_UAB_PWR)<<Rectifier_UABH_B_UAB_PWR_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_AGND_PTS)<<Rectifier_UABH_B_AGND_PTS_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_AGND_PWR)<<Rectifier_UABH_B_AGND_PWR_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_REF_PTS)<<Rectifier_UABH_B_REF_PTS_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_REF_PWR)<<Rectifier_UABH_B_REF_PWR_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_CMP_DSI_LEVEL)<<Rectifier_UABH_B_CMP_DSI_LEVEL_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_CMP_EDGE)<<Rectifier_UABH_B_CMP_EDGE_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_CMP_PWR)<<Rectifier_UABH_B_CMP_PWR_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_OA_PWR)<<Rectifier_UABH_B_OA_PWR_SHIFT) \
))

#define Rectifier_UABH_B_PARAM_CB_GND       0
#define Rectifier_UABH_B_PARAM_CC_GND       0
#define Rectifier_UABH_B_PARAM_FRC_SIGN_BIT 0
#define Rectifier_UABH_B_PARAM_DAC_MODE_EN  0
#define Rectifier_UABH_B_PARAM_DAC_MODE     0

#define Rectifier_UABH_B_NONZERO_CAP_CTRL ( Rectifier_UABH_B_Nonzero( \
    (0|0|0|0|0) ))
#define Rectifier_UABH_B_DEFAULT_CAP_CTRL ((uint32)(\
    (((uint32)Rectifier_UABH_B_PARAM_CB_GND)<<Rectifier_UABH_B_CB_GND_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_CC_GND)<<Rectifier_UABH_B_CC_GND_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_FRC_SIGN_BIT)<<Rectifier_UABH_B_FRC_SIGN_BIT_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_DAC_MODE_EN)<<Rectifier_UABH_B_DAC_MODE_EN_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_DAC_MODE)<<Rectifier_UABH_B_DAC_MODE_SHIFT) \
))

#define Rectifier_UABH_B_PARAM_CB_VAL    0
#define Rectifier_UABH_B_PARAM_CA_VAL    63
#define Rectifier_UABH_B_PARAM_SIGN_VAL  0
#define Rectifier_UABH_B_PARAM_CB_64     0
#define Rectifier_UABH_B_PARAM_CC_VAL    0
#define Rectifier_UABH_B_PARAM_CF_VAL    31

#define Rectifier_UABH_B_NONZERO_CAP_ABCF_VAL  (Rectifier_UABH_B_Nonzero( \
    (0|63|0|0|0|31) ))
#define Rectifier_UABH_B_DEFAULT_CAP_ABCF_VAL  ((uint32)(\
    (((uint32)Rectifier_UABH_B_PARAM_CB_VAL)<<Rectifier_UABH_B_CB_VAL_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_CA_VAL)<<Rectifier_UABH_B_CA_VAL_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SIGN_VAL)<<Rectifier_UABH_B_SIGN_VAL_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_CB_64)<<Rectifier_UABH_B_CB_64_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_CC_VAL)<<Rectifier_UABH_B_CC_VAL_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_CF_VAL)<<Rectifier_UABH_B_CF_VAL_SHIFT) \
))

#define Rectifier_UABH_B_NONZERO_CAP_AB_VAL_NXT (Rectifier_UABH_B_Nonzero( \
    (0|63|0) ))
#define Rectifier_UABH_B_DEFAULT_CAP_AB_VAL_NXT  ((uint32)(\
    (((uint32)Rectifier_UABH_B_PARAM_CB_VAL)<<Rectifier_UABH_B_CB_VAL_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_CA_VAL)<<Rectifier_UABH_B_CA_VAL_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SIGN_VAL)<<Rectifier_UABH_B_SIGN_VAL_SHIFT) \
))

#define Rectifier_UABH_B_PARAM_CC_VAL_UPDATE  0
#define Rectifier_UABH_B_PARAM_CF_VAL_UPDATE  0

#define Rectifier_UABH_B_NONZERO_CAP_CF_VAL_NXT (Rectifier_UABH_B_Nonzero( \
    (0|0|0|31) ))
#define Rectifier_UABH_B_DEFAULT_CAP_CF_VAL_NXT ((uint32)(\
    (((uint32)Rectifier_UABH_B_PARAM_CC_VAL)<<Rectifier_UABH_B_CC_VAL_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_CF_VAL)<<Rectifier_UABH_B_CF_VAL_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_CC_VAL_UPDATE)<<Rectifier_UABH_B_CC_VAL_UPDATE_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_CF_VAL_UPDATE)<<Rectifier_UABH_B_CF_VAL_UPDATE_SHIFT) \
))

#define Rectifier_UABH_B_PARAM_SW_GX0        CyUAB_SwIsClosed( 0 )
#define Rectifier_UABH_B_PARAM_SW_GX1        CyUAB_SwIsClosed( 0 )
#define Rectifier_UABH_B_PARAM_SW_GX2        CyUAB_SwIsClosed( 0 )
#define Rectifier_UABH_B_PARAM_SW_GX3        CyUAB_SwIsClosed( 0 )
#define Rectifier_UABH_B_PARAM_SW_RG         CyUAB_SwIsClosed( 0  )
#define Rectifier_UABH_B_PARAM_SW_GG         CyUAB_SwIsClosed( 1  )
#define Rectifier_UABH_B_PARAM_SW_RT         CyUAB_SwIsClosed( 0  )
#define Rectifier_UABH_B_PARAM_SW_GT         CyUAB_SwIsClosed( 0  )
#define Rectifier_UABH_B_PARAM_SW_QT         CyUAB_SwIsClosed( 0  )
#define Rectifier_UABH_B_PARAM_EARLY_PS      0
#define Rectifier_UABH_B_PARAM_EARLY_PO      0

/*strobe source from cyfitter.h*/
#define Rectifier_UABH_B_PARAM_STRB_RST_SEL  (Rectifier_UABH_B_UnmappedIsZero(Rectifier_UABH_B_halfuab__STRB_RST_SEL))

#define Rectifier_UABH_B_PARAM_STRB_RST_EN   0


/*if switch parameter references a bad x input, ignored in initialization*/
#define Rectifier_UABH_B_XIN_OK(xin) ( (0UL==((uint32)(xin))) || (1UL==((uint32)(xin))) || (2UL==((uint32)(xin))) || (3UL==((uint32)(xin))) )
#define Rectifier_UABH_B_IGNORE_VAL 0UL      
#define Rectifier_UABH_B_XField(xin,val)  ( Rectifier_UABH_B_XIN_OK((xin)) ? ((uint32)(val)) : Rectifier_UABH_B_IGNORE_VAL )

/*
#define Rectifier_UABH_B_NONZERO_SW_STATIC (Rectifier_UABH_B_Nonzero( \
    (0|0|0|0|0|1|0|0|0|0|0|0|0) ))
*/

/* force inclusion in _initPairs */
#define Rectifier_UABH_B_NONZERO_SW_STATIC (Rectifier_UABH_B_Nonzero(1))

#define Rectifier_UABH_B_DEFAULT_SW_STATIC ((uint32)(\
    Rectifier_UABH_B_XField(Rectifier_UABH_B_X0,Rectifier_UABH_B_PARAM_SW_GX0<<Rectifier_UABH_B_StaticVShift(CyUAB_SW_GX0)) |\
    Rectifier_UABH_B_XField(Rectifier_UABH_B_X1,Rectifier_UABH_B_PARAM_SW_GX1<<Rectifier_UABH_B_StaticVShift(CyUAB_SW_GX1)) |\
    Rectifier_UABH_B_XField(Rectifier_UABH_B_X2,Rectifier_UABH_B_PARAM_SW_GX2<<Rectifier_UABH_B_StaticVShift(CyUAB_SW_GX2)) |\
    Rectifier_UABH_B_XField(Rectifier_UABH_B_X3,Rectifier_UABH_B_PARAM_SW_GX3<<Rectifier_UABH_B_StaticVShift(CyUAB_SW_GX3)) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_RG)<<Rectifier_UABH_B_SW_RG_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_GG)<<Rectifier_UABH_B_SW_GG_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_RT)<<Rectifier_UABH_B_SW_RT_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_GT)<<Rectifier_UABH_B_SW_GT_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_QT)<<Rectifier_UABH_B_SW_QT_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_EARLY_PS)<<Rectifier_UABH_B_EARLY_PS_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_EARLY_PO)<<Rectifier_UABH_B_EARLY_PO_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_STRB_RST_SEL)<<Rectifier_UABH_B_STRB_RST_SEL_SHIFT) | \
    (((uint32)Rectifier_UABH_B_PARAM_STRB_RST_EN)<<Rectifier_UABH_B_STRB_RST_EN_SHIFT) \
))

/*modbitab source from cyfitter.h*/
#define Rectifier_UABH_B_PARAM_MODBIT0_SRC_SEL   (Rectifier_UABH_B_UnmappedIsZero(Rectifier_UABH_B_halfuab__MODBIT0_SEL))
/*modbitc source from cyfitter.h*/
#define Rectifier_UABH_B_PARAM_MODBIT1_SRC_SEL   (Rectifier_UABH_B_UnmappedIsZero(Rectifier_UABH_B_halfuab__MODBIT1_SEL))

/*
#define Rectifier_UABH_B_NONZERO_SW_MODBIT_SRC (Rectifier_UABH_B_Nonzero( \
    (0|0) ))
*/
/* force inclusion in _initPairs */
#define Rectifier_UABH_B_NONZERO_SW_MODBIT_SRC (Rectifier_UABH_B_Nonzero(1))

#define Rectifier_UABH_B_DEFAULT_SW_MODBIT_SRC ((uint32)(\
    (((uint32)Rectifier_UABH_B_PARAM_MODBIT0_SRC_SEL)<<Rectifier_UABH_B_MODBIT0_SRC_SEL_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_MODBIT1_SRC_SEL)<<Rectifier_UABH_B_MODBIT1_SRC_SEL_SHIFT) \
))


    
#define Rectifier_UABH_B_PARAM_SW_AX0_MODBIT  0
#define Rectifier_UABH_B_PARAM_SW_AX1_MODBIT  0
#define Rectifier_UABH_B_PARAM_SW_AX2_MODBIT  10
#define Rectifier_UABH_B_PARAM_SW_AX3_MODBIT  0
#define Rectifier_UABH_B_PARAM_SW_AP          0
#define Rectifier_UABH_B_PARAM_SW_AQ          0

#define Rectifier_UABH_B_NONZERO_SW_CA_IN0 (Rectifier_UABH_B_Nonzero( \
    (0|0|10|0|0|0) ))
#define Rectifier_UABH_B_DEFAULT_SW_CA_IN0 ((uint32)(\
    Rectifier_UABH_B_XField(Rectifier_UABH_B_X0,Rectifier_UABH_B_PARAM_SW_AX0_MODBIT<<Rectifier_UABH_B_DynamicVShift(CyUAB_SW_AX0_MODBIT)) |\
    Rectifier_UABH_B_XField(Rectifier_UABH_B_X1,Rectifier_UABH_B_PARAM_SW_AX1_MODBIT<<Rectifier_UABH_B_DynamicVShift(CyUAB_SW_AX1_MODBIT)) |\
    Rectifier_UABH_B_XField(Rectifier_UABH_B_X2,Rectifier_UABH_B_PARAM_SW_AX2_MODBIT<<Rectifier_UABH_B_DynamicVShift(CyUAB_SW_AX2_MODBIT)) |\
    Rectifier_UABH_B_XField(Rectifier_UABH_B_X3,Rectifier_UABH_B_PARAM_SW_AX3_MODBIT<<Rectifier_UABH_B_DynamicVShift(CyUAB_SW_AX3_MODBIT)) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_AP)<<Rectifier_UABH_B_SW_AP_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_AQ)<<Rectifier_UABH_B_SW_AQ_SHIFT) \
))

#define Rectifier_UABH_B_PARAM_SW_AA         0
#define Rectifier_UABH_B_PARAM_SW_AR_MODBIT  0
#define Rectifier_UABH_B_PARAM_SW_AG_MODBIT  9
#define Rectifier_UABH_B_PARAM_SW_AV_MODBIT  0

/*Shared switch AA intentionally ommitted - handled in Init()*/
#define Rectifier_UABH_B_NONZERO_SW_CA_IN1 (Rectifier_UABH_B_Nonzero( \
    (0|9|0) ))
#define Rectifier_UABH_B_DEFAULT_SW_CA_IN1 ((uint32)(\
    (((uint32)Rectifier_UABH_B_PARAM_SW_AR_MODBIT)<<Rectifier_UABH_B_SW_AR_MODBIT_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_AG_MODBIT)<<Rectifier_UABH_B_SW_AG_MODBIT_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_AV_MODBIT)<<Rectifier_UABH_B_SW_AV_MODBIT_SHIFT) \
))

#define Rectifier_UABH_B_PARAM_SW_RA_MODBIT  0
#define Rectifier_UABH_B_PARAM_SW_GA_MODBIT  1
#define Rectifier_UABH_B_PARAM_SW_VA_MODBIT  0
#define Rectifier_UABH_B_PARAM_SW_SA         10

#define Rectifier_UABH_B_NONZERO_SW_CA_TOP (Rectifier_UABH_B_Nonzero( \
    (0|1|0|10) ))
#define Rectifier_UABH_B_DEFAULT_SW_CA_TOP ((uint32)(\
    (((uint32)Rectifier_UABH_B_PARAM_SW_RA_MODBIT)<<Rectifier_UABH_B_SW_RA_MODBIT_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_GA_MODBIT)<<Rectifier_UABH_B_SW_GA_MODBIT_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_VA_MODBIT)<<Rectifier_UABH_B_SW_VA_MODBIT_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_SA)<<Rectifier_UABH_B_SW_SA_SHIFT) \
))

#define Rectifier_UABH_B_PARAM_SW_BX0_MODBIT 0
#define Rectifier_UABH_B_PARAM_SW_BX1_MODBIT 0
#define Rectifier_UABH_B_PARAM_SW_BX2_MODBIT 0
#define Rectifier_UABH_B_PARAM_SW_BX3_MODBIT 0
#define Rectifier_UABH_B_PARAM_SW_BP         0
#define Rectifier_UABH_B_PARAM_SW_BQ         0

#define Rectifier_UABH_B_NONZERO_SW_CB_IN0 (Rectifier_UABH_B_Nonzero( \
    (0|0|0|0|0|0) ))
#define Rectifier_UABH_B_DEFAULT_SW_CB_IN0 ((uint32)(\
    Rectifier_UABH_B_XField(Rectifier_UABH_B_X0,Rectifier_UABH_B_PARAM_SW_BX0_MODBIT<<Rectifier_UABH_B_DynamicVShift(CyUAB_SW_BX0_MODBIT)) |\
    Rectifier_UABH_B_XField(Rectifier_UABH_B_X1,Rectifier_UABH_B_PARAM_SW_BX1_MODBIT<<Rectifier_UABH_B_DynamicVShift(CyUAB_SW_BX1_MODBIT)) |\
    Rectifier_UABH_B_XField(Rectifier_UABH_B_X2,Rectifier_UABH_B_PARAM_SW_BX2_MODBIT<<Rectifier_UABH_B_DynamicVShift(CyUAB_SW_BX2_MODBIT)) |\
    Rectifier_UABH_B_XField(Rectifier_UABH_B_X3,Rectifier_UABH_B_PARAM_SW_BX3_MODBIT<<Rectifier_UABH_B_DynamicVShift(CyUAB_SW_BX3_MODBIT)) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_BP)<<Rectifier_UABH_B_SW_BP_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_BQ)<<Rectifier_UABH_B_SW_BQ_SHIFT) \
))

#define Rectifier_UABH_B_PARAM_SW_BB        0
#define Rectifier_UABH_B_PARAM_SW_BR_MODBIT 0
#define Rectifier_UABH_B_PARAM_SW_BG_MODBIT 0
#define Rectifier_UABH_B_PARAM_SW_BV_MODBIT 0

/*Shared switch BB intentionally ommitted - handled in Init()*/
#define Rectifier_UABH_B_NONZERO_SW_CB_IN1 (Rectifier_UABH_B_Nonzero( \
    (0|0|0) ))
#define Rectifier_UABH_B_DEFAULT_SW_CB_IN1 ((uint32)(\
    (((uint32)Rectifier_UABH_B_PARAM_SW_BR_MODBIT)<<Rectifier_UABH_B_SW_BR_MODBIT_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_BG_MODBIT)<<Rectifier_UABH_B_SW_BG_MODBIT_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_BV_MODBIT)<<Rectifier_UABH_B_SW_BV_MODBIT_SHIFT) \
))

#define Rectifier_UABH_B_PARAM_SW_RB_MODBIT 0
#define Rectifier_UABH_B_PARAM_SW_GB_MODBIT 0
#define Rectifier_UABH_B_PARAM_SW_VB_MODBIT 0
#define Rectifier_UABH_B_PARAM_SW_TB        0
#define Rectifier_UABH_B_PARAM_SW_SB        0

#define Rectifier_UABH_B_NONZERO_SW_CB_TOP (Rectifier_UABH_B_Nonzero( \
    (0|0|0|0|0) ))
#define Rectifier_UABH_B_DEFAULT_SW_CB_TOP ((uint32)(\
    (((uint32)Rectifier_UABH_B_PARAM_SW_RB_MODBIT)<<Rectifier_UABH_B_SW_RB_MODBIT_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_GB_MODBIT)<<Rectifier_UABH_B_SW_GB_MODBIT_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_VB_MODBIT)<<Rectifier_UABH_B_SW_VB_MODBIT_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_TB)<<Rectifier_UABH_B_SW_TB_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_SB)<<Rectifier_UABH_B_SW_SB_SHIFT) \
))

#define Rectifier_UABH_B_PARAM_SW_CX0_MODBIT 0
#define Rectifier_UABH_B_PARAM_SW_CX1_MODBIT 0
#define Rectifier_UABH_B_PARAM_SW_CX2_MODBIT 0
#define Rectifier_UABH_B_PARAM_SW_CX3_MODBIT 0
#define Rectifier_UABH_B_PARAM_SW_CP         0
#define Rectifier_UABH_B_PARAM_SW_CQ         0
    
#define Rectifier_UABH_B_NONZERO_SW_CC_IN0 (Rectifier_UABH_B_Nonzero( \
    (0|0|0|0|0|0) ))
#define Rectifier_UABH_B_DEFAULT_SW_CC_IN0 ((uint32)(\
    Rectifier_UABH_B_XField(Rectifier_UABH_B_X0,Rectifier_UABH_B_PARAM_SW_CX0_MODBIT<<Rectifier_UABH_B_DynamicVShift(CyUAB_SW_CX0_MODBIT)) |\
    Rectifier_UABH_B_XField(Rectifier_UABH_B_X1,Rectifier_UABH_B_PARAM_SW_CX1_MODBIT<<Rectifier_UABH_B_DynamicVShift(CyUAB_SW_CX1_MODBIT)) |\
    Rectifier_UABH_B_XField(Rectifier_UABH_B_X2,Rectifier_UABH_B_PARAM_SW_CX2_MODBIT<<Rectifier_UABH_B_DynamicVShift(CyUAB_SW_CX2_MODBIT)) |\
    Rectifier_UABH_B_XField(Rectifier_UABH_B_X3,Rectifier_UABH_B_PARAM_SW_CX3_MODBIT<<Rectifier_UABH_B_DynamicVShift(CyUAB_SW_CX3_MODBIT)) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_CP)<<Rectifier_UABH_B_SW_CP_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_CQ)<<Rectifier_UABH_B_SW_CQ_SHIFT) \
))

#define Rectifier_UABH_B_PARAM_SW_CC        0
#define Rectifier_UABH_B_PARAM_SW_CR_MODBIT 0
#define Rectifier_UABH_B_PARAM_SW_CG_MODBIT 0
#define Rectifier_UABH_B_PARAM_SW_CV_MODBIT 0

/*Shared switch CC intentionally ommitted - handled in Init()*/
#define Rectifier_UABH_B_NONZERO_SW_CC_IN1 (Rectifier_UABH_B_Nonzero( \
    (0|0|0) ))
#define Rectifier_UABH_B_DEFAULT_SW_CC_IN1 ((uint32)(\
    (((uint32)Rectifier_UABH_B_PARAM_SW_CR_MODBIT)<<Rectifier_UABH_B_SW_CR_MODBIT_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_CG_MODBIT)<<Rectifier_UABH_B_SW_CG_MODBIT_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_CV_MODBIT)<<Rectifier_UABH_B_SW_CV_MODBIT_SHIFT) \
))

#define Rectifier_UABH_B_PARAM_SW_RC_MODBIT 0
#define Rectifier_UABH_B_PARAM_SW_GC_MODBIT 0
#define Rectifier_UABH_B_PARAM_SW_VC_MODBIT 0
#define Rectifier_UABH_B_PARAM_SW_TC        0
#define Rectifier_UABH_B_PARAM_SW_SC        0
#define Rectifier_UABH_B_PARAM_SW_ZC        0

#define Rectifier_UABH_B_NONZERO_SW_CC_TOP (Rectifier_UABH_B_Nonzero( \
    (0|0|0|0|0|0) ))
#define Rectifier_UABH_B_DEFAULT_SW_CC_TOP ((uint32)(\
    (((uint32)Rectifier_UABH_B_PARAM_SW_RC_MODBIT)<<Rectifier_UABH_B_SW_RC_MODBIT_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_GC_MODBIT)<<Rectifier_UABH_B_SW_GC_MODBIT_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_VC_MODBIT)<<Rectifier_UABH_B_SW_VC_MODBIT_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_TC)<<Rectifier_UABH_B_SW_TC_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_SC)<<Rectifier_UABH_B_SW_SC_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_ZC)<<Rectifier_UABH_B_SW_ZC_SHIFT) \
))

#define Rectifier_UABH_B_PARAM_SW_GF 9
#define Rectifier_UABH_B_PARAM_SW_PF 10
#define Rectifier_UABH_B_PARAM_SW_PS 9
#define Rectifier_UABH_B_PARAM_SW_PO 3

#define Rectifier_UABH_B_NONZERO_SW_CF_BOT (Rectifier_UABH_B_Nonzero( \
    (9|10|9|3) ))
#define Rectifier_UABH_B_DEFAULT_SW_CF_BOT ((uint32)(\
    (((uint32)Rectifier_UABH_B_PARAM_SW_GF)<<Rectifier_UABH_B_SW_GF_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_PF)<<Rectifier_UABH_B_SW_PF_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_PS)<<Rectifier_UABH_B_SW_PS_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SW_PO)<<Rectifier_UABH_B_SW_PO_SHIFT) \
))

#define Rectifier_UABH_B_PARAM_CMP_FF     0
#define Rectifier_UABH_B_PARAM_VALID_WAVE 15
#define Rectifier_UABH_B_PARAM_TRIG_OUT   0
#define Rectifier_UABH_B_PARAM_STROBE_SW  15
#define Rectifier_UABH_B_PARAM_STROBE_RST 0

#define Rectifier_UABH_B_NONZERO_SW_OTHER (Rectifier_UABH_B_Nonzero( \
    (0|15|0|15|0) ))
#define Rectifier_UABH_B_DEFAULT_SW_OTHER ((uint32)(\
    (((uint32)Rectifier_UABH_B_PARAM_CMP_FF)<<Rectifier_UABH_B_CMP_FF_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_VALID_WAVE)<<Rectifier_UABH_B_VALID_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_TRIG_OUT)<<Rectifier_UABH_B_TRIG_OUT_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_STROBE_SW)<<Rectifier_UABH_B_STROBE_SW_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_STROBE_RST)<<Rectifier_UABH_B_STROBE_RST_SHIFT) \
))

#define Rectifier_UABH_B_PARAM_CA_BOOST  10
#define Rectifier_UABH_B_PARAM_CB_BOOST  0
#define Rectifier_UABH_B_PARAM_CC_BOOST  0
#define Rectifier_UABH_B_PARAM_CF_BOOST  10
#define Rectifier_UABH_B_PARAM_SUM_BOOST 10
#define Rectifier_UABH_B_PARAM_PUMP_WAVE 10
    
#define Rectifier_UABH_B_NONZERO_SW_BOOST_CTRL (Rectifier_UABH_B_Nonzero( \
    (10|0|0|10|10|10) ))
#define Rectifier_UABH_B_DEFAULT_SW_BOOST_CTRL ((uint32)(\
    (((uint32)Rectifier_UABH_B_PARAM_CA_BOOST)<<Rectifier_UABH_B_CA_BOOST_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_CB_BOOST)<<Rectifier_UABH_B_CB_BOOST_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_CC_BOOST)<<Rectifier_UABH_B_CC_BOOST_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_CF_BOOST)<<Rectifier_UABH_B_CF_BOOST_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SUM_BOOST)<<Rectifier_UABH_B_SUM_BOOST_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_PUMP_WAVE)<<Rectifier_UABH_B_PUMP_WAVE_SHIFT) \
))

#define Rectifier_UABH_B_PARAM_STARTUP_DELAY 22
#define Rectifier_UABH_B_PARAM_ALIGN_MODE    0

#define Rectifier_UABH_B_NONZERO_STARTUP_DELAY (Rectifier_UABH_B_Nonzero( \
	(22|0) ))
#define Rectifier_UABH_B_DEFAULT_STARTUP_DELAY ((uint32)(\
    (((uint32)Rectifier_UABH_B_PARAM_STARTUP_DELAY)<<Rectifier_UABH_B_STARTUP_DELAY_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_ALIGN_MODE)<<Rectifier_UABH_B_ALIGN_MODE_SHIFT) \
))

#define Rectifier_UABH_B_PARAM_SUBSAMPLE 0    
#define Rectifier_UABH_B_PARAM_SUBSAMPLE_INIT 0    
    
#define Rectifier_UABH_B_NONZERO_SUBSAMPLE_CTRL (Rectifier_UABH_B_Nonzero( \
    (0|0) ))
#define Rectifier_UABH_B_DEFAULT_SUBSAMPLE_CTRL ((uint32)(\
    (((uint32)Rectifier_UABH_B_PARAM_SUBSAMPLE)<<Rectifier_UABH_B_SUBSAMPLE_SHIFT) |\
    (((uint32)Rectifier_UABH_B_PARAM_SUBSAMPLE_INIT)<<Rectifier_UABH_B_SUBSAMPLE_INIT_SHIFT) \
))
    
#define Rectifier_UABH_B_PARAM_LAST_STEP  1

/*trigger_in source from cyfitter.h*/
#define Rectifier_UABH_B_PARAM_TRIG_SEL    (Rectifier_UABH_B_UnmappedIsZero(Rectifier_UABH_B_halfuab__TRIG_SEL))

#define Rectifier_UABH_B_PARAM_TRIGGER_EN 1

/*
#define Rectifier_UABH_B_NONZERO_SRAM_CTRL (Rectifier_UABH_B_Nonzero( \
    (1|31|1) ))
*/
/* force inclusion in _initPairs */
#define Rectifier_UABH_B_NONZERO_SRAM_CTRL (Rectifier_UABH_B_Nonzero(1))    
 
#define Rectifier_UABH_B_DEFAULT_SRAM_CTRL ((uint32)(\
    (((uint32)Rectifier_UABH_B_PARAM_LAST_STEP)<<Rectifier_UABH_B_LAST_STEP_SHIFT) | \
    (((uint32)Rectifier_UABH_B_PARAM_TRIG_SEL)<<Rectifier_UABH_B_TRIG_SEL_SHIFT)   | \
    (((uint32)Rectifier_UABH_B_PARAM_TRIGGER_EN)<<Rectifier_UABH_B_TRIGGER_EN_SHIFT) \
))
/*There is intentionally no run bit symbol parameter and the run bit is not set as part of the Init() function*/

#define Rectifier_UABH_B_PARAM_WAVE_STEP_0  2821
#define Rectifier_UABH_B_PARAM_WAVE_STEP_1  3590
#define Rectifier_UABH_B_PARAM_WAVE_STEP_2  0
#define Rectifier_UABH_B_PARAM_WAVE_STEP_3  0
#define Rectifier_UABH_B_PARAM_WAVE_STEP_4  0
#define Rectifier_UABH_B_PARAM_WAVE_STEP_5  0
#define Rectifier_UABH_B_PARAM_WAVE_STEP_6  0
#define Rectifier_UABH_B_PARAM_WAVE_STEP_7  0
#define Rectifier_UABH_B_PARAM_WAVE_STEP_8  0
#define Rectifier_UABH_B_PARAM_WAVE_STEP_9  0
#define Rectifier_UABH_B_PARAM_WAVE_STEP_10 0
#define Rectifier_UABH_B_PARAM_WAVE_STEP_11 0
#define Rectifier_UABH_B_PARAM_WAVE_STEP_12 0
#define Rectifier_UABH_B_PARAM_WAVE_STEP_13 0
#define Rectifier_UABH_B_PARAM_WAVE_STEP_14 0
#define Rectifier_UABH_B_PARAM_WAVE_STEP_15 0

#define Rectifier_UABH_B_NONZERO_WAVE_0  (Rectifier_UABH_B_Nonzero( Rectifier_UABH_B_PARAM_WAVE_STEP_0 ) )
#define Rectifier_UABH_B_NONZERO_WAVE_1  (Rectifier_UABH_B_Nonzero( Rectifier_UABH_B_PARAM_WAVE_STEP_1 ) )
#define Rectifier_UABH_B_NONZERO_WAVE_2  (Rectifier_UABH_B_Nonzero( Rectifier_UABH_B_PARAM_WAVE_STEP_2 ) )
#define Rectifier_UABH_B_NONZERO_WAVE_3  (Rectifier_UABH_B_Nonzero( Rectifier_UABH_B_PARAM_WAVE_STEP_3 ) )
#define Rectifier_UABH_B_NONZERO_WAVE_4  (Rectifier_UABH_B_Nonzero( Rectifier_UABH_B_PARAM_WAVE_STEP_4 ) )
#define Rectifier_UABH_B_NONZERO_WAVE_5  (Rectifier_UABH_B_Nonzero( Rectifier_UABH_B_PARAM_WAVE_STEP_5 ) )
#define Rectifier_UABH_B_NONZERO_WAVE_6  (Rectifier_UABH_B_Nonzero( Rectifier_UABH_B_PARAM_WAVE_STEP_6 ) )
#define Rectifier_UABH_B_NONZERO_WAVE_7  (Rectifier_UABH_B_Nonzero( Rectifier_UABH_B_PARAM_WAVE_STEP_7 ) )
#define Rectifier_UABH_B_NONZERO_WAVE_8  (Rectifier_UABH_B_Nonzero( Rectifier_UABH_B_PARAM_WAVE_STEP_8 ) )
#define Rectifier_UABH_B_NONZERO_WAVE_9  (Rectifier_UABH_B_Nonzero( Rectifier_UABH_B_PARAM_WAVE_STEP_9 ) )
#define Rectifier_UABH_B_NONZERO_WAVE_10 (Rectifier_UABH_B_Nonzero( Rectifier_UABH_B_PARAM_WAVE_STEP_10 ) )
#define Rectifier_UABH_B_NONZERO_WAVE_11 (Rectifier_UABH_B_Nonzero( Rectifier_UABH_B_PARAM_WAVE_STEP_11 ) )
#define Rectifier_UABH_B_NONZERO_WAVE_12 (Rectifier_UABH_B_Nonzero( Rectifier_UABH_B_PARAM_WAVE_STEP_12 ) )
#define Rectifier_UABH_B_NONZERO_WAVE_13 (Rectifier_UABH_B_Nonzero( Rectifier_UABH_B_PARAM_WAVE_STEP_13 ) )
#define Rectifier_UABH_B_NONZERO_WAVE_14 (Rectifier_UABH_B_Nonzero( Rectifier_UABH_B_PARAM_WAVE_STEP_14 ) )
#define Rectifier_UABH_B_NONZERO_WAVE_15 (Rectifier_UABH_B_Nonzero( Rectifier_UABH_B_PARAM_WAVE_STEP_15 ) )


/*all wave steps up to and including the last non-zero wave step 
    are included in the waveConfig array definition */
/*always include WAVE_STEP_0, since LAST_STEP minimum is 0*/
#define Rectifier_UABH_B_INC_STEP_0 (Rectifier_UABH_B_Nonzero(1))

#define Rectifier_UABH_B_INC_STEP_1  ( \
    Rectifier_UABH_B_NONZERO_WAVE_1  || \
    Rectifier_UABH_B_NONZERO_WAVE_2  || Rectifier_UABH_B_NONZERO_WAVE_3  || \
    Rectifier_UABH_B_NONZERO_WAVE_4  || Rectifier_UABH_B_NONZERO_WAVE_5  || \
    Rectifier_UABH_B_NONZERO_WAVE_6  || Rectifier_UABH_B_NONZERO_WAVE_7  || \
    Rectifier_UABH_B_NONZERO_WAVE_8  || Rectifier_UABH_B_NONZERO_WAVE_9  || \
    Rectifier_UABH_B_NONZERO_WAVE_10 || Rectifier_UABH_B_NONZERO_WAVE_11 || \
    Rectifier_UABH_B_NONZERO_WAVE_12 || Rectifier_UABH_B_NONZERO_WAVE_13 || \
    Rectifier_UABH_B_NONZERO_WAVE_14 || Rectifier_UABH_B_NONZERO_WAVE_15 )
    
#define Rectifier_UABH_B_INC_STEP_2  ( \
    Rectifier_UABH_B_NONZERO_WAVE_2  || Rectifier_UABH_B_NONZERO_WAVE_3  || \
    Rectifier_UABH_B_NONZERO_WAVE_4  || Rectifier_UABH_B_NONZERO_WAVE_5  || \
    Rectifier_UABH_B_NONZERO_WAVE_6  || Rectifier_UABH_B_NONZERO_WAVE_7  || \
    Rectifier_UABH_B_NONZERO_WAVE_8  || Rectifier_UABH_B_NONZERO_WAVE_9  || \
    Rectifier_UABH_B_NONZERO_WAVE_10 || Rectifier_UABH_B_NONZERO_WAVE_11 || \
    Rectifier_UABH_B_NONZERO_WAVE_12 || Rectifier_UABH_B_NONZERO_WAVE_13 || \
    Rectifier_UABH_B_NONZERO_WAVE_14 || Rectifier_UABH_B_NONZERO_WAVE_15 )
    
#define Rectifier_UABH_B_INC_STEP_3  ( \
    Rectifier_UABH_B_NONZERO_WAVE_3  || \
    Rectifier_UABH_B_NONZERO_WAVE_4  || Rectifier_UABH_B_NONZERO_WAVE_5  || \
    Rectifier_UABH_B_NONZERO_WAVE_6  || Rectifier_UABH_B_NONZERO_WAVE_7  || \
    Rectifier_UABH_B_NONZERO_WAVE_8  || Rectifier_UABH_B_NONZERO_WAVE_9  || \
    Rectifier_UABH_B_NONZERO_WAVE_10 || Rectifier_UABH_B_NONZERO_WAVE_11 || \
    Rectifier_UABH_B_NONZERO_WAVE_12 || Rectifier_UABH_B_NONZERO_WAVE_13 || \
    Rectifier_UABH_B_NONZERO_WAVE_14 || Rectifier_UABH_B_NONZERO_WAVE_15 )
    
#define Rectifier_UABH_B_INC_STEP_4  ( \
    Rectifier_UABH_B_NONZERO_WAVE_4  || Rectifier_UABH_B_NONZERO_WAVE_5  || \
    Rectifier_UABH_B_NONZERO_WAVE_6  || Rectifier_UABH_B_NONZERO_WAVE_7  || \
    Rectifier_UABH_B_NONZERO_WAVE_8  || Rectifier_UABH_B_NONZERO_WAVE_9  || \
    Rectifier_UABH_B_NONZERO_WAVE_10 || Rectifier_UABH_B_NONZERO_WAVE_11 || \
    Rectifier_UABH_B_NONZERO_WAVE_12 || Rectifier_UABH_B_NONZERO_WAVE_13 || \
    Rectifier_UABH_B_NONZERO_WAVE_14 || Rectifier_UABH_B_NONZERO_WAVE_15 )

#define Rectifier_UABH_B_INC_STEP_5  ( \
    Rectifier_UABH_B_NONZERO_WAVE_5  || \
    Rectifier_UABH_B_NONZERO_WAVE_6  || Rectifier_UABH_B_NONZERO_WAVE_7  || \
    Rectifier_UABH_B_NONZERO_WAVE_8  || Rectifier_UABH_B_NONZERO_WAVE_9  || \
    Rectifier_UABH_B_NONZERO_WAVE_10 || Rectifier_UABH_B_NONZERO_WAVE_11 || \
    Rectifier_UABH_B_NONZERO_WAVE_12 || Rectifier_UABH_B_NONZERO_WAVE_13 || \
    Rectifier_UABH_B_NONZERO_WAVE_14 || Rectifier_UABH_B_NONZERO_WAVE_15 )

#define Rectifier_UABH_B_INC_STEP_6  ( \
    Rectifier_UABH_B_NONZERO_WAVE_6  || Rectifier_UABH_B_NONZERO_WAVE_7  || \
    Rectifier_UABH_B_NONZERO_WAVE_8  || Rectifier_UABH_B_NONZERO_WAVE_9  || \
    Rectifier_UABH_B_NONZERO_WAVE_10 || Rectifier_UABH_B_NONZERO_WAVE_11 || \
    Rectifier_UABH_B_NONZERO_WAVE_12 || Rectifier_UABH_B_NONZERO_WAVE_13 || \
    Rectifier_UABH_B_NONZERO_WAVE_14 || Rectifier_UABH_B_NONZERO_WAVE_15 )
    
#define Rectifier_UABH_B_INC_STEP_7  ( \
    Rectifier_UABH_B_NONZERO_WAVE_7  || \
    Rectifier_UABH_B_NONZERO_WAVE_8  || Rectifier_UABH_B_NONZERO_WAVE_9  || \
    Rectifier_UABH_B_NONZERO_WAVE_10 || Rectifier_UABH_B_NONZERO_WAVE_11 || \
    Rectifier_UABH_B_NONZERO_WAVE_12 || Rectifier_UABH_B_NONZERO_WAVE_13 || \
    Rectifier_UABH_B_NONZERO_WAVE_14 || Rectifier_UABH_B_NONZERO_WAVE_15 )
    
#define Rectifier_UABH_B_INC_STEP_8  ( \
    Rectifier_UABH_B_NONZERO_WAVE_8  || Rectifier_UABH_B_NONZERO_WAVE_9  || \
    Rectifier_UABH_B_NONZERO_WAVE_10 || Rectifier_UABH_B_NONZERO_WAVE_11 || \
    Rectifier_UABH_B_NONZERO_WAVE_12 || Rectifier_UABH_B_NONZERO_WAVE_13 || \
    Rectifier_UABH_B_NONZERO_WAVE_14 || Rectifier_UABH_B_NONZERO_WAVE_15 )
    
#define Rectifier_UABH_B_INC_STEP_9  ( \
    Rectifier_UABH_B_NONZERO_WAVE_9  || \
    Rectifier_UABH_B_NONZERO_WAVE_10 || Rectifier_UABH_B_NONZERO_WAVE_11 || \
    Rectifier_UABH_B_NONZERO_WAVE_12 || Rectifier_UABH_B_NONZERO_WAVE_13 || \
    Rectifier_UABH_B_NONZERO_WAVE_14 || Rectifier_UABH_B_NONZERO_WAVE_15 )
    
#define Rectifier_UABH_B_INC_STEP_10  ( \
    Rectifier_UABH_B_NONZERO_WAVE_10 || Rectifier_UABH_B_NONZERO_WAVE_11 || \
    Rectifier_UABH_B_NONZERO_WAVE_12 || Rectifier_UABH_B_NONZERO_WAVE_13 || \
    Rectifier_UABH_B_NONZERO_WAVE_14 || Rectifier_UABH_B_NONZERO_WAVE_15 )
    
#define Rectifier_UABH_B_INC_STEP_11  ( \
    Rectifier_UABH_B_NONZERO_WAVE_11 || \
    Rectifier_UABH_B_NONZERO_WAVE_12 || Rectifier_UABH_B_NONZERO_WAVE_13 || \
    Rectifier_UABH_B_NONZERO_WAVE_14 || Rectifier_UABH_B_NONZERO_WAVE_15 )
    
#define Rectifier_UABH_B_INC_STEP_12  ( \
    Rectifier_UABH_B_NONZERO_WAVE_12 || Rectifier_UABH_B_NONZERO_WAVE_13 || \
    Rectifier_UABH_B_NONZERO_WAVE_14 || Rectifier_UABH_B_NONZERO_WAVE_15 )
    
#define Rectifier_UABH_B_INC_STEP_13  ( \
    Rectifier_UABH_B_NONZERO_WAVE_13 || \
    Rectifier_UABH_B_NONZERO_WAVE_14 || Rectifier_UABH_B_NONZERO_WAVE_15 )
    
#define Rectifier_UABH_B_INC_STEP_14  ( \
    Rectifier_UABH_B_NONZERO_WAVE_14 || Rectifier_UABH_B_NONZERO_WAVE_15 )
    
#define Rectifier_UABH_B_INC_STEP_15  ( Rectifier_UABH_B_NONZERO_WAVE_15 )
    
    
/*! @} group_init */

#endif /* #ifndef Rectifier_UABH_B_PARAM_H */
    
/*! @endcond */

/* [] END OF FILE */
