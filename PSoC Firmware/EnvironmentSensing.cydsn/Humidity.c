/*******************************************************************************
* File Name: Humidity.c
*
* Version: 1.0
*
* Description:
* This is the source file for Humidity measurement.
*
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
#include <project.h>
#include "main.h"
#include "humidity.h"
#include "filters.h"


/*******************************************************************************
* Function Name: void HumidityTask(void)
********************************************************************************
*
* Summary:
*  This function performs humidity measurement.
*  The function checks if CSD scan is complete, reads the raw counts, converts the
*  raw counts to capacitance, and capacitance to humidity.
*
* Parameters:
*  None
*
* Return:
*  None.
*
* Side Effects:
*   None
*******************************************************************************/
void HumidityTask(void)
{
	
    uint32 sensorraw;
    uint32 refraw;


	/* Check if CSD scan is complete */
	if(!(CSD_IsBusy()))
	{
		/* Read reference capacitor raw counts */
        refraw = CSD_BUTTON0_SNS1_RAW0_VALUE;
        sensorraw = CSD_BUTTON0_SNS0_RAW0_VALUE;
        
//        CSD_GetParam(CSD_BUTTON0_SNS1_RAW0_PARAM_ID,&refraw);
//        CSD_GetParam(CSD_BUTTON0_SNS0_RAW0_PARAM_ID,&sensorraw);
            
        /* Pass the raw counts through low pass filter */
		ProcessValues.RawCountsRefCap = refraw; //LowPassFilter(refraw, LPF_CHANNEL_REFCAP);
        ProcessValues.RawCountsHumidity = sensorraw; //LowPassFilter(sensorraw, LPF_CHANNEL_HUMIDITY);

        /* Convert raw counts to capacitance */
		ProcessValues.SensorCapacitance = CalculateCapacitance(ProcessValues.RawCountsHumidity, ProcessValues.RawCountsRefCap);
		
		/* Calculate humidity */
		ProcessValues.Humidity = CalculateHumidity(ProcessValues.SensorCapacitance);

        CSD_ScanAllWidgets();
	}	
}

/*******************************************************************************
* Function Name: uint16 CalculateCapacitance(uint16 RawCounts, uint16 RefsensorCounts)
********************************************************************************
*
* Summary:
*  This function calculates capacitance from raw count.
*
* Parameters:
*  uint16 RawCounts - Raw count corresponding to Humidity sensor
*  uint16 RefsensorCounts - Raw count corresponding to Reference capacitor
*
* Return:
*  Capacitance of the Humidity sensor
*
* Side Effects:
*   None
*******************************************************************************/
uint16 CalculateCapacitance(uint16 rawCounts, uint16 refsensorCounts)
{
    return (uint16)((float)(rawCounts - OFFSETCOUNT) * (CREF - COFFSET) / (float)(refsensorCounts - OFFSETCOUNT));
}

/*******************************************************************************
* Function Name: uint16 CalculateHumidity(uint16 Capacitance)
********************************************************************************
*
* Summary:
*  This function calculates humidity from capacitance

* Parameters:
*  uint16 Capacitance - Capacitance of the humidity sensor
*
* Return:
*  Calculated Humidity value
*
* Side Effects:
*   None
*******************************************************************************/
uint16 CalculateHumidity(uint16 capacitance)
{
    int16 humidity;
    int16 delta;
    
    /* Find capacitance difference from nominal capacitance at 55% RH */
    delta = capacitance - CAPACITANCE_AT_55_RH;
    
    /* Calculate humidity from capacitance difference and sensor sensitivity */
    humidity = ((delta * SENSITIVITY_DENOMINATOR) / SENSITIVITY_NUMERATOR) + NOMINAL_HUMIDITY;
    
    /* If humidity is less than zero, limit it to 0; If humidity is greater than 1000 (100%), limit to 1000 */
    humidity = (humidity < HUMIDITY_0_PERCENT) ? HUMIDITY_0_PERCENT : (humidity > HUMIDITY_100_PERCENT) ? HUMIDITY_100_PERCENT : humidity;

    /* Return Humidity value */
    return humidity;
}


void Humidity_Start(void)
{
	/* Start CSD block and start scan */
	CSD_Start();
    CSD_ScanAllWidgets();

    /* Initialize humidity values to zero */
    ProcessValues.Humidity = 0;
    ProcessValues.RawCountsHumidity = 0;
    ProcessValues.RawCountsRefCap = 0;
    ProcessValues.SensorCapacitance = 0;
}

void Humidity_Stop(void)
{
	/* Stop the CSD block */
	CSD_Stop();

    /* Initialize humidity values to zero */
    ProcessValues.Humidity = 0;
    ProcessValues.RawCountsHumidity = 0;
    ProcessValues.RawCountsRefCap = 0;
    ProcessValues.SensorCapacitance = 0;
    
}
/* [] END OF FILE */
