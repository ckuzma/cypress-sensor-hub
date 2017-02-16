/*******************************************************************************
* File Name: Sensor_Signal.h  
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

#if !defined(CY_PINS_Sensor_Signal_ALIASES_H) /* Pins Sensor_Signal_ALIASES_H */
#define CY_PINS_Sensor_Signal_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define Sensor_Signal_0			(Sensor_Signal__0__PC)
#define Sensor_Signal_0_PS		(Sensor_Signal__0__PS)
#define Sensor_Signal_0_PC		(Sensor_Signal__0__PC)
#define Sensor_Signal_0_DR		(Sensor_Signal__0__DR)
#define Sensor_Signal_0_SHIFT	(Sensor_Signal__0__SHIFT)
#define Sensor_Signal_0_INTR	((uint16)((uint16)0x0003u << (Sensor_Signal__0__SHIFT*2u)))

#define Sensor_Signal_INTR_ALL	 ((uint16)(Sensor_Signal_0_INTR))


#endif /* End Pins Sensor_Signal_ALIASES_H */


/* [] END OF FILE */
