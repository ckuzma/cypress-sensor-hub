/*******************************************************************************
* File Name: AmbientLightSensor.c
*
* Version: 1.0
*
* Description:
* This source file has functions related to Ambient Light Sensor measurement.
*
* Owner:
* GRAA
*
* Code Tested With:
* 1. PSoC Creator 3.3 SP2 build 9598
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
#include <AmbientLightSensor.h>
#include <main.h>
#include <filters.h>

int16 CalculateAlsCurrent(int16 ADCResult);
uint16 CalculateAmbientLight(uint16 ALSCurrent);
uint16 ALSOffsetCount;

/*******************************************************************************
* Function Name: uint16 CalculateAmbientLight(uint16 ALSCurrent)
********************************************************************************
*
* Summary:
*  This function takes photo diode current as parameter, calculates and returns
*  ambient light in lumens
*
* Parameters:
*  uint16 ALSCurrent 	- 	Current in nA.  This is an unsigned integer
*
* Return:
*  uint16 LightIntensity - Returns light intensity in Lumens
*
* Side Effects:
*   None
*******************************************************************************/
uint16 CalculateAmbientLight(uint16 ALSCurrent)
{
	uint32 LightIntensity;
    
	/* Calculate light intensity */
	LightIntensity = (ALSCurrent * ALS_LIGHT_SCALE_FACTOR_NUMERATOR)/ALS_LIGHT_SCALE_FACTOR_DENOMINATOR;
	
	return (int16)LightIntensity;
}

/*******************************************************************************
* Function Name: void AmbientLightSensorTask(void)
********************************************************************************
*
* Summary:
*  This function performs the ambient light measurement task
*  The function reads ADC counts from ALS channel, passes the result through
*  a low pass filter, calls respective functions to calculate photo diode
*  current and ambient light and updates the respective variables in the
*  ProcessValues structure
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*   None
*******************************************************************************/
void AmbientLightSensorTask(void)
{
	uint16 ADCResult_ALS_Direct, ADCResult_ALS_Reverse, ADCResult;
    
    ADCResult_ALS_Direct = ADC_GetResult16(ADC_CHANNEL_ALS_DIRECT);
	ADCResult_ALS_Reverse = ADC_GetResult16(ADC_CHANNEL_ALS_REVERSE);
    
	ADCResult = LowPassFilter((int16)(ADCResult_ALS_Direct - ADCResult_ALS_Reverse)/2, LPF_CHANNEL_ALS);	
	
	if(I2CRegsCommand)
	{
		/* Read the offset count */		
		ALSOffsetCount = ADCResult;
		
		/* Clear the calibration command register */
		I2CRegsCommand = 0;
		
		/* Write to flash */
		WriteALSOffset();
	}
	
  	ProcessValues.ALSCurrent = CalculateAlsCurrent(ADCResult - ALSOffsetCount);
	
	ProcessValues.LightIntensity = CalculateAmbientLight(ProcessValues.ALSCurrent);
}	

/*******************************************************************************
* Function Name: int16 CalculateAlsCurrent(int16 ADCResult)
********************************************************************************
*
* Summary:
*  This function takes ADC Counts as input and calculates and returns photo
*  diode current.  The function limits the current to a +ve value
*
* Parameters:
*  int16 ADCResult 	- 	16 bit ADC result
*
* Return:
*  int16 DiodeCurrent - Returns diode current in nA
*
* Side Effects:
*   None
*******************************************************************************/
int16 CalculateAlsCurrent(int16 ADCResult)
{
	int32 DiodeCurrent;
	
	/* Calculate photo diode current */
	DiodeCurrent = (ADCResult * ALS_CURRENT_SCALE_FACTOR_NUMERATOR)/ALS_CURRENT_SCALE_FACTOR_DENOMINATOR; 
	
	/* If the calculated current is -ve, limit this to zero */
	if(DiodeCurrent < 0)
	{
		DiodeCurrent = 0;
	}
    
	return DiodeCurrent;
}

void AmbientLightSensor_Start(void)
{
	/* Start the TIA */
	Opamp_TIA_Start();

    /* Initialize the ALS current and light intensity values to zero */
    ProcessValues.ALSCurrent = 0;
    ProcessValues.LightIntensity = 0;
	
	/* Read ALS offset from flash */
	ALSOffsetCount = (*((uint16 *) CY_TEST_FLASH_ADDR));
}

void AmbientLightSensor_Stop(void)
{
	/* Stop the TIA */
	Opamp_TIA_Stop();
    
    /* Initialize the ALS current and light intensity values to zero */
    ProcessValues.ALSCurrent = 0;
    ProcessValues.LightIntensity = 0;
}

void WriteALSOffset(void)
{
	uint16 rowPattern[CY_FLASH_SIZEOF_ROW/2];
	
	cystatus returnValue = CYRET_SUCCESS;
	
	rowPattern[0] = ALSOffsetCount;
	
	/* Erases a row of Flash and programs it with the rowPattern */
    returnValue = CySysFlashWriteRow(CY_TEST_FLASH_ROW, (uint8 *)rowPattern);

    /* Check if operration is successful */
    if (returnValue != CY_SYS_FLASH_SUCCESS)
    {
		
    }
}

/* [] END OF FILE */
