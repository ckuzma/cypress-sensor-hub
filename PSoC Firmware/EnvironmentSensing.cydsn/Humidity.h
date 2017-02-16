/*******************************************************************************
* File Name: Humidity.h
*
* Version: 1.0
*
* Description:
* This is the header file for Humidity measurement.
********************************************************************************
* Copyright (2016) , Cypress Semiconductor Corporation.
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
* significant injury to the user. The inclusion of Cypress' product in a life
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies Cypress against all charges. Use may be
* limited by and subject to the applicable Cypress software license agreement.
*******************************************************************************/
#ifndef __HUMIDITY_H
#define __HUMIDITY_H

#include <project.h>    
    
    
/* Constants used to calculate humidity */
#define OFFSETCOUNT                 (1536)     /* This is raw count equivalent to trace capacitance */
#define CAPACITANCE_AT_55_RH		(1800)		/* This is the capacitance of the sensor at 55% RH with 0.1pF resolution */
#define SENSITIVITY_NUMERATOR		(31)		/* Sensitivity numerator and denominator indicate sensitivity of sensor */
#define SENSITIVITY_DENOMINATOR		(100)
#define CREF                        (1930)      /* Effective capacitance seen at reference capacitor pin (value of 
                                                    reference capacitor (180pF) + trace + pin capacitance) */
#define COFFSET                     (150)       /* Offset Capacitance */

#define NOMINAL_HUMIDITY            (550)       /* Nominal humidity is 55% RH */
#define HUMIDITY_0_PERCENT          (0)         /* Humidity 0% RH */
#define HUMIDITY_50                 (500)       /* Humidity 50% RH */
#define HUMIDITY_100_PERCENT        (1000)      /* Humidity 100% RH */
    
/* Function prototypes */
void HumidityTask(void);
uint16 CalculateCapacitance(uint16 RawCounts, uint16 RefsensorCounts);
uint16 CalculateHumidity(uint16 Capacitance);
void Humidity_Start();
void Humidity_Stop();

#endif		/* __HUMIDITY_H */

/* [] END OF FILE */