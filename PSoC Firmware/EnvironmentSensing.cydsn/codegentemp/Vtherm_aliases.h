/*******************************************************************************
* File Name: Vtherm.h  
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

#if !defined(CY_PINS_Vtherm_ALIASES_H) /* Pins Vtherm_ALIASES_H */
#define CY_PINS_Vtherm_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define Vtherm_0			(Vtherm__0__PC)
#define Vtherm_0_PS		(Vtherm__0__PS)
#define Vtherm_0_PC		(Vtherm__0__PC)
#define Vtherm_0_DR		(Vtherm__0__DR)
#define Vtherm_0_SHIFT	(Vtherm__0__SHIFT)
#define Vtherm_0_INTR	((uint16)((uint16)0x0003u << (Vtherm__0__SHIFT*2u)))

#define Vtherm_INTR_ALL	 ((uint16)(Vtherm_0_INTR))


#endif /* End Pins Vtherm_ALIASES_H */


/* [] END OF FILE */
