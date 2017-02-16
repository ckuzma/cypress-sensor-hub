/*******************************************************************************
* File Name: InductiveProx.c
*
* Description:
* This is the source file for Inductive Proximity Sensing.
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
#include <InductiveProx.h>
#include <main.h>
#include <filters.h>

uint16 IPSBaselineSamples = 0;      /* Number of baseline samples acquired */

/*******************************************************************************
* Function Name: void IPSTask(void)
********************************************************************************
*
* Summary:
* This function reads the inductive proximity sensor data, decides the presence of metal and 
* updates the I2C variables.  It may also indicate the presence of metal using the red LED.
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
void IPSTask(void)
{
    /* Get the ADC Count */
	ProcessValues.IPSRawCounts = LowPassFilter(ADC_GetResult16(ADC_CHANNEL_IPS), LPF_CHANNEL_IPS);
	
    /* Update baseline */
    IPS_UpdateBaseline();
    
    /* Check if sensor data is below threshold */
    if(ProcessValues.IPSDifferenceCounts > IPS_THRESHOLD)
    {
        /* Enable Trigger variable */
        ProcessValues.IPSTrigger = TRIGGERED;
        
        #ifdef IPS_LED_ENABLED 
            LED_Red_Write(LED_ON); 
        #endif
    }    
    else
    {
        ProcessValues.IPSTrigger = NOT_TRIGGERED; 
        
        #ifdef IPS_LED_ENABLED 
            LED_Red_Write(LED_OFF); 
        #endif
    }
}

/*******************************************************************************
* Function Name: void IPS_UpdateBaseline(void)
********************************************************************************
*
* Summary:
* This function initializes a 'baseline' value of IPS signal during program startup
* which is used during normal operation as a reference for comparison of the signal.
* The difference count is also calculated only when signal from metal is detected.
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
void IPS_UpdateBaseline(void)
{   
    /* Initialize baseline with the first sample of IPSrawcount */
    if (IPSBaselineSamples == 0)
    {
        ProcessValues.IPSBaselineCounts = ProcessValues.IPSRawCounts;
        IPSBaselineSamples++;
    }
    /* Update baseline only during startup/ initial # of samples */
    else if (IPSBaselineSamples < IPS_BASELINE_SAMPLES)
    {
        ProcessValues.IPSBaselineCounts += ProcessValues.IPSRawCounts;
        ProcessValues.IPSBaselineCounts = ProcessValues.IPSBaselineCounts >> 1;
        IPSBaselineSamples++;
    }
    /* Calculate difference counts during normal sensing */
    else
    {
        /* For metal detection, rawcounts should drop below baseline, i.e. diffCounts must be positive */
        int16 diffCounts = ProcessValues.IPSBaselineCounts - ProcessValues.IPSRawCounts;
        
        /* Update difference count variable with signal from metal only */
        ProcessValues.IPSDifferenceCounts = diffCounts > 0 ? diffCounts : 0;
    }
    
}

void IPS_Start(void)
{
	 /* Start Excitation PWM */
    ExcitationPWM_Start();	

    /* Start Down Mixer Component */
    DownMixer_Start();
    
    /* Start Rectifier Component */
    Rectifier_Start();

    /* Initialize IPS related process values to zero */
    ProcessValues.IPSRawCounts = 0;
    ProcessValues.IPSBaselineCounts = 0;
    ProcessValues.IPSDifferenceCounts = 0;
    ProcessValues.IPSTrigger = 0;
    
    #ifdef IPS_LED_ENABLED 
        LED_Red_Write(LED_ON); 
    #endif
    
}

void IPS_Stop(void)
{
 	/* Stop Excitation PWM */
    ExcitationPWM_Stop();
   
    /* Initialize IPS related process values to zero */
    ProcessValues.IPSRawCounts = 0;
    ProcessValues.IPSBaselineCounts = 0;
    ProcessValues.IPSDifferenceCounts = 0;
    ProcessValues.IPSTrigger = 0;
    
    #ifdef IPS_LED_ENABLED 
        LED_Red_Write(LED_OFF); 
    #endif
}

/* [] END OF FILE */
