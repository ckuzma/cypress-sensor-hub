/*******************************************************************************
* File Name: Filters.h
*
* Version: 1.0
*
* Description:
* This is the header file for filters.
*
* Owner:
* ARVI
*
* Code Tested With:
* 1. PSoC Creator 3.2 SP2
*
********************************************************************************
* Copyright (2011) , Cypress Semiconductor Corporation.
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
#ifndef __FILTERS_H
#define __FILTERS_H

#include <project.h>
	


/* IIR FILTER CONSTANT decides cut off frequency */
/* Cut off frequency = fs/(2 * pi * iir_filter_constant) */
#define IIR_FILTER_CONSTANT     (128) 
#define ADC_SAR_MAX_RESOLUTION 	(12)

/* IIR Filter coefficients for each signal */
/* Cut off frequency = fs/(2 * pi * iir_filter_constant) */
#define FILTER_COEFFICIENT_TEMPERATURE	(64)
#define FILTER_COEFFICIENT_ALS			(16)
#define FILTER_COEFFICIENT_IPS			(16)
	
/* Function prototypes */
int32 LowPassFilter(int32 input, uint8 Channel);
void SetFilterCoefficient(uint16 FilterCoefficient, uint8 Channel);
int16 HighPassFilter(int16 input);
void InitHighPassFilter(int16 input);

/* Number of filter channels */
#define FILTER_CHANNELS			    (4)

/* Filter Channel defines */
#define LPF_CHANNEL_TEMP_VREF	    (0x00)
#define LPF_CHANNEL_TEMP_VTH	    (0x01)
#define LPF_CHANNEL_ALS			    (0x02)
#define LPF_CHANNEL_IPS 		    (0x03)

#endif		/* __FILTERS_H */

/* [] END OF FILE */
