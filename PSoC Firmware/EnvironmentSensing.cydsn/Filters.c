/*******************************************************************************
* File Name: Filters.c
*
* Version: 1.0
*
* Description:
* This is source code for IIR low pass and high pass filters.
*
* Code Tested With:
* PSoC Creator 3.3 SP2 build 9598
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
#include <filters.h>

int32 Lpf[FILTER_CHANNELS] = {0};
int16 IIRFilterCoefficient[FILTER_CHANNELS] = {0};

int32 vlp = 0;

/****************************************************************************** 
* Function Name: InitFilters
*******************************************************************************
*
* Summary:
*  This function initializes the filters
*  
*
* Parameters:  
*  None
*
* Return: 
*  None
*
*******************************************************************************/
void InitFilters(void)
{
	uint8 Channel;
	for (Channel=0; Channel<FILTER_CHANNELS; Channel++)
	{
		Lpf[Channel]=(1 << (ADC_SAR_MAX_RESOLUTION - 1) << 8);
	}
}

/*******************************************************************************
* Function Name: int32 LowPassFilter(int32 input, uint8 Channel)
********************************************************************************
*
* Summary:
*  This function implements an IIR low pass filter
*
* Parameters:
*  int32 input - Input value to be filtered
*  uint8 Channel - IIR filter channel number
*
* Return:
*  int32 - Filtered output
*
* Side Effects:
*   None
*******************************************************************************/
int32 LowPassFilter(int32 input, uint8 Channel)
{
    input <<= 8;
    Lpf[Channel] = (input + (IIRFilterCoefficient[Channel] - 1)*Lpf[Channel])/IIRFilterCoefficient[Channel];
    return(Lpf[Channel] >> 8);
}

/*******************************************************************************
* Function Name: void SetFilterCoefficient(uint16 FilterCoefficient, uint8 Channel)
********************************************************************************
*
* Summary:
*  This function sets the filter coefficient for a filter channel
*
* Parameters:
*  uint16 FilterCoefficient - Filter coefficient for the particular channel
*  uint8 Channel - IIR filter channel number
*
* Return:
*  None
*
* Side Effects:
*   None
*******************************************************************************/
void SetFilterCoefficient(uint16 FilterCoefficient, uint8 Channel)
{
	IIRFilterCoefficient[Channel] = FilterCoefficient;
}

/*******************************************************************************
* Function Name: void InitLowPassFilter(int32 InitValue, uint8 Channel)
********************************************************************************
*
* Summary:
*  This function initializes a particular low pass filter channel
*
* Parameters:
*  int32 InitValue - Value to be initialized
*  uint8 Channel - IIR filter channel number
*
* Return:
*  None
*
* Side Effects:
*   None
*******************************************************************************/
void InitLowPassFilter(int32 InitValue, uint8 Channel)
{
	Lpf[Channel] = InitValue << 8;
}

/*******************************************************************************
* Function Name: void InitHighPassFilter(int16 input)
********************************************************************************
*
* Summary:
*  This function initializes the high pass filter with a particular value
*
* Parameters:
*  int16 input - initialization value
*
* Return:
*  None
*
* Side Effects:
*   None
*******************************************************************************/
void InitHighPassFilter(int16 input)
{
	vlp = (int32)input;	
}


/*******************************************************************************
* Function Name: int16 HighPassFilter(int16 input)
********************************************************************************
*
* Summary:
*  This function implements a high pass filter
*
* Parameters:
*  int16 input - 16 bit signed input
*
* Return:
*  int16 - High pass filtered output
*
* Side Effects:
*   None
*******************************************************************************/
int16 HighPassFilter(int16 input)
{
	int16 vhp;
    uint8 scale = 6;
	
	vhp = input-vlp;
	vlp += vhp >> scale;
	return vhp - (1 << (scale - 1));
}


/* [] END OF FILE */
