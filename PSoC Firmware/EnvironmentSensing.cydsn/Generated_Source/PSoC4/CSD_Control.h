/***************************************************************************//**
* \file CSD_Control.h
* \version 3.0
*
* \brief
*   This file provides the function prototypes of the Control Block.
*
* \see CapSense P4 v3.0 Datasheet
*
*//*****************************************************************************
* Copyright (2016), Cypress Semiconductor Corporation.
********************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress Source Code and derivative works for the sole purpose of creating
* custom software in support of licensee product to be used only in conjunction
* with a Cypress integrated circuit as specified in the applicable agreement.
* Any reproduction, modification, translation, compilation, or representation of
* this software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the
* materials described herein. Cypress does not assume any liability arising out
* of the application or use of any product or circuit described herein. Cypress
* does not authorize its products for use as critical components in life-support
* systems where a malfunction or failure may reasonably be expected to result in
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies Cypress against all charges. Use may be
* limited by and subject to the applicable Cypress software license agreement.
*******************************************************************************/

#if !defined(CY_CAPSENSE_CSD_CONTROL_H)
#define CY_CAPSENSE_CSD_CONTROL_H

#include "CSD_Configuration.h"

/*******************************************************************************
* Function Prototypes 
*******************************************************************************/

/*******************************************************************************
* HIGH LEVEL API 
*******************************************************************************/
/**
* \if SECTION_CAPSENSE_HIGH_LEVEL
* \addtogroup group_capsense_high_level
* \{
*/

cystatus CSD_Start(void);
cystatus CSD_Stop(void);
cystatus CSD_Resume(void);

cystatus CSD_ProcessAllWidgets(void);
cystatus CSD_ProcessWidget(uint32 widgetId);

void CSD_Sleep(void);
void CSD_Wakeup(void);

/** \}
* \endif */

#if (0u != CSD_SELF_TEST_EN)
    uint32 CSD_RunSelfTest(uint32 testEnMask);
#endif /* #if (0u != CSD_SELF_TEST_EN) */

/**
* \if SECTION_CAPSENSE_LOW_LEVEL
* \addtogroup group_capsense_low_level
* \{
*/

cystatus CSD_ProcessWidgetExt(uint32 widgetId, uint32 mode);
cystatus CSD_ProcessSensorExt(uint32 widgetId, uint32 sensorId, uint32 mode);

/** \}
* \endif */

/*******************************************************************************
* Function Prototypes - internal functions
*******************************************************************************/
/**
* \if SECTION_CAPSENSE_INTERNAL
* \addtogroup group_capsense_internal
* \{
*/
cystatus CSD_Initialize(void);

/** \}
* \endif */

/*******************************************************************************
* API Constants
*******************************************************************************/

/* Define Self-Test Masks */
#if CSD_SELF_TEST_EN
    #define  CSD_TST_SHORTS             (0x01Lu)
    #define  CSD_TST_GLOBAL_CRC         (0x02Lu)
    #define  CSD_TST_BSL_RAW_LIMTS      (0x04Lu)
    #define  CSD_TST_WIDGET_CRC         (0x08Lu)
    #define  CSD_TST_CSD_CAPS           (0x10Lu)
    #define  CSD_TST_CSX_CAPS           (0x20Lu)
    #define  CSD_TST_BASELINES          (0x40Lu)
#endif /* #if CSD_SELF_TEST_EN */

#endif /* End CY_CAPSENSE_CSD_CONTROL_H */

/* [] END OF FILE */
