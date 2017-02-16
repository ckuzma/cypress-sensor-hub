/***************************************************************************//**
* \file     IPS_PGA_PVT.h
* \version  1.0
*
* \brief
*  This file contains the private constants and macros used in
*  the PGA_P4 Component code.
*
********************************************************************************
* \copyright
* Copyright 2016, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PGA_P4_IPS_PGA_PVT_H) 
#define CY_PGA_P4_IPS_PGA_PVT_H

#include "IPS_PGA.h"


/***************************************
*         Internal Constants
***************************************/
    
#define IPS_PGA_POWER                  ((uint32)3u)
#define IPS_PGA_GAIN                   ((uint32)4u)
    
#define IPS_PGA_POWER_MAX              (IPS_PGA_HIGH)
#define IPS_PGA_GAIN_MAX               (IPS_PGA_GAIN_32)

/* ((Gain steps / 2) + 1) */
#define IPS_PGA_COMP_TAB_HEIGHT        ((IPS_PGA_GAIN_MAX >> 1u) + 1u)
#define IPS_PGA_COMP_TAB_WIDTH         (IPS_PGA_POWER_MAX)
#define IPS_PGA_GET_COMP_TAB_GAIN_MASK (0x00000007u)

#define IPS_PGA_VREF_INTERNAL          (0u)
#define IPS_PGA_OUTPUT_MODE_10x        (1u)

#define IPS_PGA_DEFAULT_POWER          ((uint32)IPS_PGA_POWER << IPS_PGA_OA_PWR_MODE_SHIFT)
#define IPS_PGA_DEFAULT_GAIN           ((uint32)IPS_PGA_GAIN << IPS_PGA_RES_TAP_SHIFT)
#define IPS_PGA_DEFAULT_GAIN_POWER     (IPS_PGA_DEFAULT_GAIN | IPS_PGA_DEFAULT_POWER)


/***************************************
*    Variables with External Linkage
***************************************/

extern uint32 IPS_PGA_internalGainPower;
extern const uint32 IPS_PGA_compTab[IPS_PGA_COMP_TAB_HEIGHT][IPS_PGA_COMP_TAB_WIDTH];


/***************************************
*       Macro Definitions
***************************************/

/* Returns true if component available in Deep Sleep power mode */
#define IPS_PGA_CHECK_DEEPSLEEP_SUPPORT (IPS_PGA_DEEPSLEEP_SUPPORT != 0u)
/* Returns true if component uses 10x (Class AB) output buffer mode*/
#define IPS_PGA_CHECK_OUTPUT_MODE      (IPS_PGA_OUTPUT_MODE == IPS_PGA_OUTPUT_MODE_10x)
#define IPS_PGA_GET_DEEPSLEEP_ON       ((IPS_PGA_CHECK_DEEPSLEEP_SUPPORT) ? \
                                                    (IPS_PGA_DEEPSLEEP_ON) : (0u))
#define IPS_PGA_GET_OA_DRIVE_STR       ((IPS_PGA_CHECK_OUTPUT_MODE) ? \
                                                    (IPS_PGA_OA_DRIVE_STR_SEL_10X) : \
                                                        (IPS_PGA_OA_DRIVE_STR_SEL_1X))
#define IPS_PGA_GET_POWER              (IPS_PGA_internalGainPower & \
                                                    IPS_PGA_OA_PWR_MODE_MASK)
#define IPS_PGA_DEFAULT_CTB_CTRL       (IPS_PGA_GET_DEEPSLEEP_ON | IPS_PGA_ENABLED)
#define IPS_PGA_DEFAULT_OA_RES_CTRL    (IPS_PGA_GET_OA_DRIVE_STR | IPS_PGA_OA_PUMP_EN)
#define IPS_PGA_DEFAULT_OA_RES_CTRL_MASK (IPS_PGA_OA_DRIVE_STR_SEL_MASK | IPS_PGA_OA_PUMP_EN)
#define IPS_PGA_GET_COMP_TAB_GAIN      ((IPS_PGA_internalGainPower >> \
                                                    (IPS_PGA_RES_TAP_SHIFT + 1u)) & \
                                                        IPS_PGA_GET_COMP_TAB_GAIN_MASK)
#define IPS_PGA_GET_COMP_TAB           (IPS_PGA_compTab[IPS_PGA_GET_COMP_TAB_GAIN] \
                                                                         [IPS_PGA_GET_POWER - 1u])
#define IPS_PGA_GET_C_FB               (IPS_PGA_GET_COMP_TAB & IPS_PGA_C_FB_MASK)
#define IPS_PGA_GET_OA_COMP_TRIM       (IPS_PGA_GET_COMP_TAB & IPS_PGA_OA_COMP_TRIM_MASK)


#endif /* CY_PGA_P4_IPS_PGA_PVT_H */

/* [] END OF FILE */
