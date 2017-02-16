/*******************************************************************************
* File Name: PIR.c
*
* Description:
* This is the source code file for PIR detection
*
* Code Tested With:
* 1. PSoC Creator 3.3 SP2 build 9598
*
********************************************************************************
* Copyright (2015) , Cypress Semiconductor Corporation.
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
#include <pir.h>
#include <main.h>
#include <filters.h>

uint16 PIRTriggerTimer = 0;

void PIRTask(void)
{
    /* Read the ADC */
	ProcessValues.PIRRaw = TO_SIGNED_12BIT(ADC_GetResult16(ADC_CHANNEL_PIR));
    
    if (filterInit == FALSE)
    {
        InitHighPassFilter(ProcessValues.PIRRaw);
        filterInit = TRUE;
        return;
    }
    
	ProcessValues.PIRHpf = HighPassFilter(ProcessValues.PIRRaw);
	
	/* Check the ADC result against the thresholds */
	if((ProcessValues.PIRHpf > PIR_WINDOW_HIGH) || (ProcessValues.PIRHpf < PIR_WINDOW_LOW))
	{
        /* Once the motion is detected, the RGB LED is driven with RED color for 5s
           and the motion detected variable in I2C buffer is latched to '1' for 5s.
           If another motion is detected before 5s elapsed the timer is restarted 
           to maintain 5s time window */
       	
		/* Stop the timer */
        Timebase5s_Stop();
		
		/* Reload the counter */
		Timebase5s_WriteCounter(Timebase5s_TC_PERIOD_VALUE);
		
		/* Start the 5s timer */
        Timebase5s_Start();
		
		/* Update the status of motion detection */
		ProcessValues.PIRTrigger = TRIGGERED;
                      
		#ifdef PIR_LED_ENABLED 
            LED_Blue_Write(LED_ON); 
        #endif
	}
}

void PIR_Start(void)
{
	/* Start the first stage PIR amplifier */
	PIRAmplifierStage1_Start();
	
	/* Update PIR window high and low thresholds */
	ProcessValues.PIRWindowHigh = PIR_WINDOW_HIGH;
	ProcessValues.PIRWindowLow = PIR_WINDOW_LOW;
    
    /* Initialize PIR related process values to zero */
    ProcessValues.PIRRaw = 0;
    ProcessValues.PIRHpf = 0;
    ProcessValues.PIRTrigger = NOT_TRIGGERED;
    
    /* Enable timebase ISR */
    isr_Timebase5s_Start();
    isr_Timebase5s_StartEx(TIMEBASE_ISR); 
    
    /* Initialize high pass filter */
    filterInit = FALSE;
    
    #ifdef PIR_LED_ENABLED 
        LED_Blue_Write(LED_OFF); 
    #endif
}

void PIR_Stop(void)
{
	/* Stop the first stage PIR amplifier */
	PIRAmplifierStage1_Start();
    
    /* Initialize PIR related process values to zero */
    ProcessValues.PIRRaw = 0;
    ProcessValues.PIRHpf = 0;
    ProcessValues.PIRTrigger = NOT_TRIGGERED;
    
    /* Reset high pass filter */
    filterInit = FALSE;
    
    #ifdef PIR_LED_ENABLED 
        LED_Blue_Write(LED_OFF); 
    #endif
}

/* [] END OF FILE */
