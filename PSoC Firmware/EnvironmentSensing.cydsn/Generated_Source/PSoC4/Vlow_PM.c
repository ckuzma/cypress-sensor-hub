/*******************************************************************************
* File Name: Vlow.c  
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
#include "Vlow.h"

static Vlow_BACKUP_STRUCT  Vlow_backup = {0u, 0u, 0u};


/*******************************************************************************
* Function Name: Vlow_Sleep
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
*  \snippet Vlow_SUT.c usage_Vlow_Sleep_Wakeup
*******************************************************************************/
void Vlow_Sleep(void)
{
    #if defined(Vlow__PC)
        Vlow_backup.pcState = Vlow_PC;
    #else
        #if (CY_PSOC4_4200L)
            /* Save the regulator state and put the PHY into suspend mode */
            Vlow_backup.usbState = Vlow_CR1_REG;
            Vlow_USB_POWER_REG |= Vlow_USBIO_ENTER_SLEEP;
            Vlow_CR1_REG &= Vlow_USBIO_CR1_OFF;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(Vlow__SIO)
        Vlow_backup.sioState = Vlow_SIO_REG;
        /* SIO requires unregulated output buffer and single ended input buffer */
        Vlow_SIO_REG &= (uint32)(~Vlow_SIO_LPM_MASK);
    #endif  
}


/*******************************************************************************
* Function Name: Vlow_Wakeup
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
*  Refer to Vlow_Sleep() for an example usage.
*******************************************************************************/
void Vlow_Wakeup(void)
{
    #if defined(Vlow__PC)
        Vlow_PC = Vlow_backup.pcState;
    #else
        #if (CY_PSOC4_4200L)
            /* Restore the regulator state and come out of suspend mode */
            Vlow_USB_POWER_REG &= Vlow_USBIO_EXIT_SLEEP_PH1;
            Vlow_CR1_REG = Vlow_backup.usbState;
            Vlow_USB_POWER_REG &= Vlow_USBIO_EXIT_SLEEP_PH2;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(Vlow__SIO)
        Vlow_SIO_REG = Vlow_backup.sioState;
    #endif
}


/* [] END OF FILE */
