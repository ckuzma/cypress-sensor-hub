/*******************************************************************************
* File Name: EXCITATION.c  
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
#include "EXCITATION.h"

static EXCITATION_BACKUP_STRUCT  EXCITATION_backup = {0u, 0u, 0u};


/*******************************************************************************
* Function Name: EXCITATION_Sleep
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
*  \snippet EXCITATION_SUT.c usage_EXCITATION_Sleep_Wakeup
*******************************************************************************/
void EXCITATION_Sleep(void)
{
    #if defined(EXCITATION__PC)
        EXCITATION_backup.pcState = EXCITATION_PC;
    #else
        #if (CY_PSOC4_4200L)
            /* Save the regulator state and put the PHY into suspend mode */
            EXCITATION_backup.usbState = EXCITATION_CR1_REG;
            EXCITATION_USB_POWER_REG |= EXCITATION_USBIO_ENTER_SLEEP;
            EXCITATION_CR1_REG &= EXCITATION_USBIO_CR1_OFF;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(EXCITATION__SIO)
        EXCITATION_backup.sioState = EXCITATION_SIO_REG;
        /* SIO requires unregulated output buffer and single ended input buffer */
        EXCITATION_SIO_REG &= (uint32)(~EXCITATION_SIO_LPM_MASK);
    #endif  
}


/*******************************************************************************
* Function Name: EXCITATION_Wakeup
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
*  Refer to EXCITATION_Sleep() for an example usage.
*******************************************************************************/
void EXCITATION_Wakeup(void)
{
    #if defined(EXCITATION__PC)
        EXCITATION_PC = EXCITATION_backup.pcState;
    #else
        #if (CY_PSOC4_4200L)
            /* Restore the regulator state and come out of suspend mode */
            EXCITATION_USB_POWER_REG &= EXCITATION_USBIO_EXIT_SLEEP_PH1;
            EXCITATION_CR1_REG = EXCITATION_backup.usbState;
            EXCITATION_USB_POWER_REG &= EXCITATION_USBIO_EXIT_SLEEP_PH2;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(EXCITATION__SIO)
        EXCITATION_SIO_REG = EXCITATION_backup.sioState;
    #endif
}


/* [] END OF FILE */
