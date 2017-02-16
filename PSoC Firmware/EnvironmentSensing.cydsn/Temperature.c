/*******************************************************************************
* File Name: Temperature.c
*
* Version: 1.0
*
* Description:
* This is the source file for Temperature measurement.
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
#include <Temperature.h>
#include <Filters.h>
#include <main.h>

/*******************************************************************************
* Function Name: void TemperatureTask(void)
********************************************************************************
*
* Summary:
*  This function performs temperature measurement.
*  The function reads ADC result form thermistor channel, passes the result through
*  a low pass filter, calculates resistance and temperature and updates the
*  corresponding variables in the ProcessValues structure
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
void TemperatureTask(void)
{
	uint16 ADCResult_VTH_High, ADCResult_VTH, ADCResult_VTH_Low;
	uint16 VRefMeasured;
    int16 Temperature;
	
    
	/* Read the ADC result for reference and thermistor voltages */
	ADCResult_VTH_High = ADC_GetResult16(ADC_CHANNEL_VTH_HIGH);
	ADCResult_VTH = ADC_GetResult16(ADC_CHANNEL_VTH);
	ADCResult_VTH_Low = ADC_GetResult16(ADC_CHANNEL_VTH_LOW);
	
	/* Filter the measured ADC counts of VREF */
	VRefMeasured = LowPassFilter((ADCResult_VTH_High - ADCResult_VTH),LPF_CHANNEL_TEMP_VREF);
	
	/* Store the filtered ADC counts of VTH in register */
	ProcessValues.Vth = LowPassFilter((ADCResult_VTH - ADCResult_VTH_Low),LPF_CHANNEL_TEMP_VTH);
	
	/* Calculate thermistor resistance. Apply low pass filter for Vref and Vth values. */
	ProcessValues.Rth = Thermistor_GetResistance(VRefMeasured, ProcessValues.Vth);	
	
	/* Calculate temperature in Celsius degree using the component API */
	//Temperature = LowPassFilter(Thermistor_GetTemperature(ProcessValues.Rth), LPF_CHANNEL_TEMP);
	Temperature = Thermistor_GetTemperature(ProcessValues.Rth);
    
    ProcessValues.Temperature = Temperature/10;
    
}

void Temperature_Start(void)
{
    /* There isnt any individual resource for Temperature sensing.
    Temperature uses VREF and ADC which are global resources.    
    So no resources to start */
    
    /* Initialize temperature related process values to zero */
    ProcessValues.Temperature = 0;
    ProcessValues.Vth = 0;
    ProcessValues.Rth = 0;
}

void Temperature_Stop(void)
{
    /* Initialize temperature related process values to zero */
    ProcessValues.Temperature = 0;
    ProcessValues.Vth = 0;
    ProcessValues.Rth = 0;
}

/* [] END OF FILE */