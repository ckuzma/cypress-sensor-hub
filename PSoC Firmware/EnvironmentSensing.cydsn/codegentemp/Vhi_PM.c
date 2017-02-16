/*******************************************************************************
* File Name: Vhi.c  
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
#include "Vhi.h"

static Vhi_BACKUP_STRUCT  Vhi_backup = {0u, 0u, 0u};


/*******************************************************************************
* Function Name: Vhi_Sleep
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
*  \snippet Vhi_SUT.c usage_Vhi_Sleep_Wakeup
*******************************************************************************/
void Vhi_Sleep(void)
{
    #if defined(Vhi__PC)
        Vhi_backup.pcState = Vhi_PC;
    #else
        #if (CY_PSOC4_4200L)
            /* Save the regulator state and put the PHY into suspend mode */
            Vhi_backup.usbState = Vhi_CR1_REG;
            Vhi_USB_POWER_REG |= Vhi_USBIO_ENTER_SLEEP;
            Vhi_CR1_REG &= Vhi_USBIO_CR1_OFF;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(Vhi__SIO)
        Vhi_backup.sioState = Vhi_SIO_REG;
        /* SIO requires unregulated output buffer and single ended input buffer */
        Vhi_SIO_REG &= (uint32)(~Vhi_SIO_LPM_MASK);
    #endif  
}


/*******************************************************************************
* Function Name: Vhi_Wakeup
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
*  Refer to Vhi_Sleep() for an example usage.
*******************************************************************************/
void Vhi_Wakeup(void)
{
    #if defined(Vhi__PC)
        Vhi_PC = Vhi_backup.pcState;
    #else
        #if (CY_PSOC4_4200L)
            /* Restore the regulator state and come out of suspend mode */
            Vhi_USB_POWER_REG &= Vhi_USBIO_EXIT_SLEEP_PH1;
            Vhi_CR1_REG = Vhi_backup.usbState;
            Vhi_USB_POWER_REG &= Vhi_USBIO_EXIT_SLEEP_PH2;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(Vhi__SIO)
        Vhi_SIO_REG = Vhi_backup.sioState;
    #endif
}


/* [] END OF FILE */
