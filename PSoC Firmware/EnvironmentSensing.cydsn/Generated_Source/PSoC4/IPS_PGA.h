/***************************************************************************//**
* \file     IPS_PGA.h
* \version  1.0
*
* \brief
*  This file contains the public API function prototypes and constants used in
*  the PGA_P4 Component.
*
********************************************************************************
* \copyright
* Copyright 2016, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PGA_P4_IPS_PGA_H) 
#define CY_PGA_P4_IPS_PGA_H 

#include "cytypes.h"
#include "cyfitter.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

#define IPS_PGA_VREF_MODE              (1u)
#define IPS_PGA_OUTPUT_MODE            (1u)
#define IPS_PGA_DEEPSLEEP_SUPPORT      (0u)


/***************************************
*    Variables with External Linkage
***************************************/

/**
* \addtogroup group_globals
* @{
*/
extern uint8 IPS_PGA_initVar;
/** @} globals */


/***************************************
*        Function Prototypes 
***************************************/

/**
* \addtogroup group_general
* @{
*/
void IPS_PGA_Init(void);
void IPS_PGA_Enable(void);
void IPS_PGA_Start(void);
void IPS_PGA_Stop(void);
void IPS_PGA_SetPower(uint32 powerLevel);
void IPS_PGA_SetGain(uint32 gainLevel);
void IPS_PGA_PumpControl(uint32 onOff);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void IPS_PGA_Sleep(void);
void IPS_PGA_Wakeup(void);
void IPS_PGA_SaveConfig(void);
void IPS_PGA_RestoreConfig(void);
/** @} power */


/**************************************
*           API Constants
**************************************/

/* Gain setting constants */
#define IPS_PGA__GAIN_1 0
#define IPS_PGA__GAIN_1_4 1
#define IPS_PGA__GAIN_2 2
#define IPS_PGA__GAIN_2_8 3
#define IPS_PGA__GAIN_4 4
#define IPS_PGA__GAIN_5_8 5
#define IPS_PGA__GAIN_10_7 7
#define IPS_PGA__GAIN_8 6
#define IPS_PGA__GAIN_21_3 9
#define IPS_PGA__GAIN_32 10
#define IPS_PGA__GAIN_16 8


/* Power setting constants */
#define IPS_PGA__LOW 1
#define IPS_PGA__MED 2
#define IPS_PGA__HIGH 3


/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup group_powerLevel
     * \brief Definitions for IPS_PGA_SetPower() function.
     *  @{
     */
    #define IPS_PGA_LOW                ((uint32)IPS_PGA__LOW)
    #define IPS_PGA_MED                ((uint32)IPS_PGA__MED)
    #define IPS_PGA_HIGH               ((uint32)IPS_PGA__HIGH)
    /** @} powerLevel */

    /** \addtogroup group_boostPump
     * \brief Definitions for IPS_PGA_PumpControl() function.
     *  @{
     */
    #define IPS_PGA_BOOST_ON           ((uint32)1u)
    #define IPS_PGA_BOOST_OFF          ((uint32)0u)
    /** @} boostPump */

    /** \addtogroup group_gain
     * \brief Definitions for IPS_PGA_SetGain() function.
     *  @{
     */
    #define IPS_PGA_GAIN_1             ((uint32)IPS_PGA__GAIN_1)
    #define IPS_PGA_GAIN_1_4           ((uint32)IPS_PGA__GAIN_1_4)
    #define IPS_PGA_GAIN_2             ((uint32)IPS_PGA__GAIN_2)
    #define IPS_PGA_GAIN_2_8           ((uint32)IPS_PGA__GAIN_2_8)
    #define IPS_PGA_GAIN_4             ((uint32)IPS_PGA__GAIN_4)
    #define IPS_PGA_GAIN_5_8           ((uint32)IPS_PGA__GAIN_5_8)
    #define IPS_PGA_GAIN_8             ((uint32)IPS_PGA__GAIN_8)
    #define IPS_PGA_GAIN_10_7          ((uint32)IPS_PGA__GAIN_10_7)
    #define IPS_PGA_GAIN_16            ((uint32)IPS_PGA__GAIN_16)
    #define IPS_PGA_GAIN_21_3          ((uint32)IPS_PGA__GAIN_21_3)
    #define IPS_PGA_GAIN_32            ((uint32)IPS_PGA__GAIN_32)
    /** @} gain */
/** @} group_constants */


/***************************************
* Registers
***************************************/

#define IPS_PGA_CTB_CTRL_REG           (*(reg32 *) IPS_PGA_cy_psoc4_abuf__CTB_CTB_CTRL)
#define IPS_PGA_CTB_CTRL_PTR           ( (reg32 *) IPS_PGA_cy_psoc4_abuf__CTB_CTB_CTRL)
#define IPS_PGA_OA_RES_CTRL_REG        (*(reg32 *) IPS_PGA_cy_psoc4_abuf__OA_RES_CTRL)
#define IPS_PGA_OA_RES_CTRL_PTR        ( (reg32 *) IPS_PGA_cy_psoc4_abuf__OA_RES_CTRL)
#define IPS_PGA_OA_SW_REG              (*(reg32 *) IPS_PGA_cy_psoc4_abuf__OA_SW)
#define IPS_PGA_OA_SW_PTR              ( (reg32 *) IPS_PGA_cy_psoc4_abuf__OA_SW)
#define IPS_PGA_OA_COMP_TRIM_REG       (*(reg32 *) IPS_PGA_cy_psoc4_abuf__OA_COMP_TRIM)
#define IPS_PGA_OA_COMP_TRIM_PTR       ( (reg32 *) IPS_PGA_cy_psoc4_abuf__OA_COMP_TRIM)


/***************************************
* Register Constants
***************************************/

/* IPS_PGA_CTB_CTRL_REG */
#define IPS_PGA_DEEPSLEEP_ON_SHIFT     (30u)   /* [30] Selects behavior CTB IP in the DeepSleep power mode */
#define IPS_PGA_ENABLED_SHIFT          (31u)   /* [31] Enable of the CTB IP */

#define IPS_PGA_DEEPSLEEP_ON           ((uint32)0x01u << IPS_PGA_DEEPSLEEP_ON_SHIFT)
#define IPS_PGA_ENABLED                ((uint32)0x01u << IPS_PGA_ENABLED_SHIFT)

/* IPS_PGA_OA_RES_CTRL_REG */
#define IPS_PGA_OA_PWR_MODE_SHIFT      (0u)    /* [1:0]    Power level */
#define IPS_PGA_OA_DRIVE_STR_SEL_SHIFT (2u)    /* [2]      Opamp output strenght select: 0 - 1x, 1 - 10x */
#define IPS_PGA_OA_PUMP_EN_SHIFT       (11u)   /* [11]     Pump enable */
#define IPS_PGA_RES_TAP_SHIFT          (16u)   /* [19:16]  PGA gain (resistor tap point) */
#define IPS_PGA_C_FB_SHIFT             (24u)   /* [27:24]  Feedback Capacitor */

#define IPS_PGA_OA_PWR_MODE_MASK       ((uint32)0x03u << IPS_PGA_OA_PWR_MODE_SHIFT)
#define IPS_PGA_OA_DRIVE_STR_SEL_1X    ((uint32)0x00u << IPS_PGA_OA_DRIVE_STR_SEL_SHIFT)
#define IPS_PGA_OA_DRIVE_STR_SEL_10X   ((uint32)0x01u << IPS_PGA_OA_DRIVE_STR_SEL_SHIFT)
#define IPS_PGA_OA_DRIVE_STR_SEL_MASK  ((uint32)0x01u << IPS_PGA_OA_DRIVE_STR_SEL_SHIFT)
#define IPS_PGA_OA_PUMP_EN             ((uint32)0x01u << IPS_PGA_OA_PUMP_EN_SHIFT)
#define IPS_PGA_RES_TAP_MASK           ((uint32)0x0Fu << IPS_PGA_RES_TAP_SHIFT)
#define IPS_PGA_C_FB_MASK              ((uint32)0x0Fu << IPS_PGA_C_FB_SHIFT)

/** IPS_PGA_OA_COMP_TRIM_REG */
#define IPS_PGA_OA_COMP_TRIM_SHIFT     (0u)    /* [1:0]    Opamp Compensation Capacitor Trim */
#define IPS_PGA_OA_COMP_TRIM_MASK      ((uint32)0x03u << IPS_PGA_OA_COMP_TRIM_SHIFT)

/** IPS_PGA_OA_SW_REG */
#define IPS_PGA_RBOT_TO_VSSA_SHIFT     (28u)   /* Resistor bottom  to VSSA */
#define IPS_PGA_RBOT_TO_VSSA           ((uint32)0x01u << IPS_PGA_RBOT_TO_VSSA_SHIFT)


#endif /* CY_PGA_P4_IPS_PGA_H */


/* [] END OF FILE */
