/******************************************************************************
* Project Name		: PSoC Analog Coprocessor Environment Sensing Solution demo
* Version			: 1.0
* Device Used		: CY8C4A45LQI-L483
* Software Used		: PSoC Creator 3.3 Service Pack 2 build 9598
* Compiler Used		: ARM GCC 4.9.3 
* Related Hardware	: CY8CKIT-048 PSoC Analog Coprocessor Pioneer Kit rev06
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

/*******************************************************************************
* Theory of Operation: This solution demonstrates measurement of environmental
* parameters using sensors such as thermistor (temperature), ambient light sensor, 
* humidity sensor, PIR sensor (IR based motion detection) and 
* Inductive sensor (proximity of metal) using the PSoC Analog Coprocessor device. 
*
*******************************************************************************/

/* Header File Includes */
#include <project.h>
#include <main.h>
#include <filters.h>
#include <temperature.h>
#include "AmbientLightSensor.h"
#include "InductiveProx.h"
#include "PIR.h"
#include "math.h"
#include <stdio.h>
#include <humidity.h>
#include <CyFlash.h>

/* Global Variables */
PROCESS_VALUES I2CRegsProcess = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
PROCESS_VALUES ProcessValues = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

uint16 I2CRegsCommand = 0;

/* Functions */

/*******************************************************************************
* Function Name: void UpdateI2CRegisters(void)
********************************************************************************
*
* Summary:
*  This function updates the I2C register map.  The function first checks if
*  there is an I2C transaction in progress, and updates the I2C registers only if
*  there is no I2C transaction in progress.
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
void I2CTask(void)
{
    if(!(EzI2C_EzI2CGetActivity() & EzI2C_EZI2C_STATUS_BUSY))
	{
    	I2CRegsProcess = ProcessValues;
	}
}

/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  This is the main function for the project.  The function initializes all the
*  resources, and in an infinite loop, performs tasks to measure all the process
*  parameters from sensors and updates I2C register map
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
int main()
{
	/* Enable global interrupts */
	CyGlobalIntEnable;
	
	/* Initialize hardware resources */
	InitResources();

	/* Infinite Loop */
	for(;;)
    {
		/* Check if ADC data is ready */
		if(ADC_IsEndConversion(ADC_RETURN_STATUS))
		{
			/* Call the temperature task to measure temperature */
			TemperatureTask();
			
			/* Call the Ambient Light Sensor task to measure ambient light */
			AmbientLightSensorTask();
			
			/* Call the PIR task to process PIR sensor signal */
			PIRTask();	
			
			 /* Call the IPS task to process IPS sensor signal */
			IPSTask();
		}
        
        /* Do humidity task */
        HumidityTask();
        
        /* Do I2C task */
        I2CTask();
    }
}

/*******************************************************************************
* Function Name: void InitResources(void)
********************************************************************************
*
* Summary:
*  This function initializes all the hardware resources
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
void InitResources(void)
{
    /* Version Major#.Minor# stored as MSB.LSB */
    ProcessValues.Version = FW_VER_SET_MAJOR(1) + FW_VER_SET_MINOR(0);
    
	/* Set IIR Low Pass filter coefficients */
	SetFilterCoefficient(FILTER_COEFFICIENT_TEMPERATURE, LPF_CHANNEL_TEMP_VREF);
	SetFilterCoefficient(FILTER_COEFFICIENT_TEMPERATURE, LPF_CHANNEL_TEMP_VTH);
	SetFilterCoefficient(FILTER_COEFFICIENT_ALS, LPF_CHANNEL_ALS);
	SetFilterCoefficient(FILTER_COEFFICIENT_IPS, LPF_CHANNEL_IPS);
    
    /* Start EzI2C and initialize buffer */
	EzI2C_Start();
	EzI2C_EzI2CSetBuffer1(sizeof(I2CRegsProcess), 0, (uint8*)&I2CRegsProcess);
	EzI2C_EzI2CSetBuffer2(sizeof(I2CRegsCommand), 2, (uint8*)&I2CRegsCommand);
	
	/* Start the VREF buffer */
	VrefBuffer_Start();
    
    /* Start the programmable voltage reference source */
    PVref_Start();
    PVref_Enable();
    
    /* Initialize sensor interfaces */
    IPS_Start();
    Temperature_Start();
    PIR_Start();
    AmbientLightSensor_Start();
    Humidity_Start();
    
    /* Start ADC and start conversion */
	ADC_Start();	    
    ADC_StartConvert();
    
    /* Set start up delay */
    CY_SET_REG32(Rectifier_UABH_B_halfuab__STARTUP_DELAY,22);
    
    /* Configure UAB to use SAR output trigger */
    CY_SET_REG32(Rectifier_UABH_B_halfuab__SRAM_CTRL, (CY_GET_REG32(Rectifier_UABH_B_halfuab__SRAM_CTRL) & 0x00ffffff) | 0xbf000000);
    
    /* Configure SAR to generate trigger for the UAB */
    CY_SET_REG32(ADC_cy_psoc4_sar_1__SAR_SAMPLE_CTRL, (CY_GET_REG32(ADC_cy_psoc4_sar_1__SAR_SAMPLE_CTRL) & 0xbfbfffff) | 0x40400000);
        
}


/*******************************************************************************
* Function Name: CY_ISR(TIMEBASE_ISR)
********************************************************************************
*
* Summary:
*  This function implements the ISR for 5s timebase
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
CY_ISR(TIMEBASE_ISR)
{
   	/* Reset the motion detection flag */
    ProcessValues.PIRTrigger = NOT_TRIGGERED;
    
    /* Turn the LED off */
    #ifdef PIR_LED_ENABLED 
        LED_Blue_Write(LED_OFF); 
    #endif
    
    /* Stop the 5s timer */
    Timebase5s_Stop();
}

/* [] END OF FILE */