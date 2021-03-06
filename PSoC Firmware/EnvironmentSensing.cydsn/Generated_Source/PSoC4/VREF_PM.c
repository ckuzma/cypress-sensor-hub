/*******************************************************************************
* File Name: VREF.c  
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
#include "VREF.h"

static VREF_BACKUP_STRUCT  VREF_backup = {0u, 0u, 0u};


/*******************************************************************************
* Function Name: VREF_Sleep
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
*  \snippet VREF_SUT.c usage_VREF_Sleep_Wakeup
*******************************************************************************/
void VREF_Sleep(void)
{
    #if defined(VREF__PC)
        VREF_backup.pcState = VREF_PC;
    #else
        #if (CY_PSOC4_4200L)
            /* Save the regulator state and put the PHY into suspend mode */
            VREF_backup.usbState = VREF_CR1_REG;
            VREF_USB_POWER_REG |= VREF_USBIO_ENTER_SLEEP;
            VREF_CR1_REG &= VREF_USBIO_CR1_OFF;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(VREF__SIO)
        VREF_backup.sioState = VREF_SIO_REG;
        /* SIO requires unregulated output buffer and single ended input buffer */
        VREF_SIO_REG &= (uint32)(~VREF_SIO_LPM_MASK);
    #endif  
}


/*******************************************************************************
* Function Name: VREF_Wakeup
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
*  Refer to VREF_Sleep() for an example usage.
*******************************************************************************/
void VREF_Wakeup(void)
{
    #if defined(VREF__PC)
        VREF_PC = VREF_backup.pcState;
    #else
        #if (CY_PSOC4_4200L)
            /* Restore the regulator state and come out of suspend mode */
            VREF_USB_POWER_REG &= VREF_USBIO_EXIT_SLEEP_PH1;
            VREF_CR1_REG = VREF_backup.usbState;
            VREF_USB_POWER_REG &= VREF_USBIO_EXIT_SLEEP_PH2;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(VREF__SIO)
        VREF_SIO_REG = VREF_backup.sioState;
    #endif
}


/* [] END OF FILE */
