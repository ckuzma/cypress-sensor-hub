/*******************************************************************************
* File Name: EXCITATION.h  
* Version 2.20
*
* Description:
*  This file contains the Alias definitions for Per-Pin APIs in cypins.h. 
*  Information on using these APIs can be found in the System Reference Guide.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_EXCITATION_ALIASES_H) /* Pins EXCITATION_ALIASES_H */
#define CY_PINS_EXCITATION_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define EXCITATION_0			(EXCITATION__0__PC)
#define EXCITATION_0_PS		(EXCITATION__0__PS)
#define EXCITATION_0_PC		(EXCITATION__0__PC)
#define EXCITATION_0_DR		(EXCITATION__0__DR)
#define EXCITATION_0_SHIFT	(EXCITATION__0__SHIFT)
#define EXCITATION_0_INTR	((uint16)((uint16)0x0003u << (EXCITATION__0__SHIFT*2u)))

#define EXCITATION_INTR_ALL	 ((uint16)(EXCITATION_0_INTR))


#endif /* End Pins EXCITATION_ALIASES_H */


/* [] END OF FILE */
