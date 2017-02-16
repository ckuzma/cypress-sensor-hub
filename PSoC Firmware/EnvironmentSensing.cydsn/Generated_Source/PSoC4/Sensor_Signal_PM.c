/*******************************************************************************
* File Name: Sensor_Signal.c  
* Version 2.20
*
* Description:
*  This file contains APIs to set up the Pins component for low power modes.
*
* Note:
*
********************************************************************************
* Copyright 2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "Sensor_Signal.h"

static Sensor_Signal_BACKUP_STRUCT  Sensor_Signal_backup = {0u, 0u, 0u};


/*******************************************************************************
* Function Name: Sensor_Signal_Sleep
****************************************************************************//**
*
* \brief Stores the pin configuration and prepares the pin for entering chip 
*  deep-sleep/hibernate modes. This function must be called for SIO and USBIO
*  pins. It is not essential if using GPIO or GPIO_OVT pins.
*
* <b>Note</b> This function is available in PSoC 4 only.
*
* \return 
*  None 
*  
* \sideeffect
*  For SIO pins, this function configures the pin input threshold to CMOS and
*  drive level to Vddio. This is needed for SIO pins when in device 
*  deep-sleep/hibernate modes.
*
* \funcusage
*  \snippet Sensor_Signal_SUT.c usage_Sensor_Signal_Sleep_Wakeup
*******************************************************************************/
void Sensor_Signal_Sleep(void)
{
    #if defined(Sensor_Signal__PC)
        Sensor_Signal_backup.pcState = Sensor_Signal_PC;
    #else
        #if (CY_PSOC4_4200L)
            /* Save the regulator state and put the PHY into suspend mode */
            Sensor_Signal_backup.usbState = Sensor_Signal_CR1_REG;
            Sensor_Signal_USB_POWER_REG |= Sensor_Signal_USBIO_ENTER_SLEEP;
            Sensor_Signal_CR1_REG &= Sensor_Signal_USBIO_CR1_OFF;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(Sensor_Signal__SIO)
        Sensor_Signal_backup.sioState = Sensor_Signal_SIO_REG;
        /* SIO requires unregulated output buffer and single ended input buffer */
        Sensor_Signal_SIO_REG &= (uint32)(~Sensor_Signal_SIO_LPM_MASK);
    #endif  
}


/*******************************************************************************
* Function Name: Sensor_Signal_Wakeup
****************************************************************************//**
*
* \brief Restores the pin configuration that was saved during Pin_Sleep().
*
* For USBIO pins, the wakeup is only triggered for falling edge interrupts.
*
* <b>Note</b> This function is available in PSoC 4 only.
*
* \return 
*  None
*  
* \funcusage
*  Refer to Sensor_Signal_Sleep() for an example usage.
*******************************************************************************/
void Sensor_Signal_Wakeup(void)
{
    #if defined(Sensor_Signal__PC)
        Sensor_Signal_PC = Sensor_Signal_backup.pcState;
    #else
        #if (CY_PSOC4_4200L)
            /* Restore the regulator state and come out of suspend mode */
            Sensor_Signal_USB_POWER_REG &= Sensor_Signal_USBIO_EXIT_SLEEP_PH1;
            Sensor_Signal_CR1_REG = Sensor_Signal_backup.usbState;
            Sensor_Signal_USB_POWER_REG &= Sensor_Signal_USBIO_EXIT_SLEEP_PH2;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(Sensor_Signal__SIO)
        Sensor_Signal_SIO_REG = Sensor_Signal_backup.sioState;
    #endif
}


/* [] END OF FILE */
