/*******************************************************************************
* File Name     : main.h
*
* Description:
* This file contains the function prototypes and constants used in main.c
*
* Code Tested With:
* 1. PSoC Creator 3.3 SP2 build 9598
*
*******************************************************************************
* Copyright (2016), Cypress Semiconductor Corporation.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software") is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and 
* foreign), United States copyright laws and international treaty provisions. 
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the 
* Cypress source code and derivative works for the sole purpose of creating 
* custom software in support of licensee product, such licensee product to be
* used only in conjunction with Cypress's integrated circuit as specified in the
* applicable agreement. Any reproduction, modification, translation, compilation,
* or representation of this Software except as specified above is prohibited 
* without the express written permission of Cypress.
* 
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes to the Software without notice. 
* Cypress does not assume any liability arising out of the application or use
* of Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use as critical components in any products 
* where a malfunction or failure may reasonably be expected to result in 
* significant injury or death ("ACTIVE Risk Product"). By including Cypress's 
* product in a ACTIVE Risk Product, the manufacturer of such system or application
* assumes all risk of such use and in doing so indemnifies Cypress against all
* liability. Use of this Software may be limited by and subject to the applicable
* Cypress software license agreement.
*******************************************************************************/

#ifndef __MAIN_H
#define __MAIN_H

#include <project.h>

/***************************************
 * System Constants
 ***************************************/
//#define PIR_LED_ENABLED     /* Enable blue LED for debugging PIR output */
//#define IPS_LED_ENABLED     /* Enable red LED for debugging IPS output */
    
#define LED_ON                  (0u)
#define LED_OFF                 (1u)
    
#define FALSE                   (0u)
#define TRUE                    (1u)
    
#define TO_SIGNED_12BIT(x)  (x - 2047)
    
/***************************************
 * Macros  
 ***************************************/
#define ADC_CHANNEL_VTH_HIGH	(0x00)
#define ADC_CHANNEL_VTH			(0x01)
#define ADC_CHANNEL_VTH_LOW		(0x02)
#define ADC_CHANNEL_ALS_DIRECT	(0x03)
#define ADC_CHANNEL_ALS_REVERSE	(0x04)
#define ADC_CHANNEL_PIR 		(0x05)
#define ADC_CHANNEL_IPS 		(0x06)
    
#define TRIGGERED               (0x01)
#define NOT_TRIGGERED           (0x00)
    
/* Definition of firmware version macros */
#define FW_VER_SET_MAJOR(x)         (x << 8)         
#define FW_VER_SET_MINOR(x)         (x) 
    
/* Defines second ROW from the last ROW */
#define CY_TEST_FLASH_ROW       (CY_FLASH_NUMBER_ROWS - 2u)

/* Defines absolute address of ROW */
#define CY_TEST_FLASH_ADDR      (CY_TEST_FLASH_ROW * CY_FLASH_SIZEOF_ROW)

/***************************************
 * TypeDefs  
 ***************************************/
    
/***************************************
 * Structures  
 ***************************************/
/* Structure that holds all the sensor values */
typedef struct PROCESS_VALUES
{
	uint16 Version;				/* Firmware version */
    uint16 Vth;					/* Voltage at thermistor potential divider */
	uint16 Rth;					/* Thermistor's resistance */
	int16 Temperature;			/* Measured temperature */
	uint16 ALSCurrent;			/* Ambient light sensor current output */
	uint16 LightIntensity;		/* Light Intensity */
	int16 PIRRaw;				/* Raw PIR ADC data */	
    int16 PIRHpf;               /* High pass filtered PIR data */
    int16 PIRWindowHigh;		/* High threshold for PIR trigger */
	int16 PIRWindowLow;			/* Low threshold for PIR trigger */
	int16 PIRTrigger;			/* PIR Trigger */  
	int16 IPSRawCounts;			/* Raw IPS ADC data */
	int16 IPSDifferenceCounts;  /* IPS difference counts */
    int16 IPSBaselineCounts;    /* IPS Baseline counts */
    uint16 IPSTrigger;			/* IPS Trigger */
    uint16 RawCountsRefCap;     /* Reference capacitor rawcounts */
    uint16 RawCountsHumidity;   /* Humidity sensor rawcounts */
    uint16 SensorCapacitance;   /* Humidity sensor capacitance */
    uint16 Humidity;            /* Relative Humidity */
}PROCESS_VALUES;


/***************************************
 * Variables  
 ***************************************/
extern PROCESS_VALUES I2CRegsProcess;
extern PROCESS_VALUES ProcessValues;
extern uint16 I2CRegsCommand;

/***************************************
 * Functions  
 ***************************************/
void InitResources(void);

CY_ISR_PROTO(TIMEBASE_ISR);

#endif		/* __MAIN_H */

/* [] END OF FILE */
