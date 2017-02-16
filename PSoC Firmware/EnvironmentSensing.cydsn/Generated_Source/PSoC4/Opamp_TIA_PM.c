/*******************************************************************************
* File Name: Opamp_TIA_PM.c
* Version 1.20
*
* Description:
*  This file provides the power management source code to the API for the
*  Opamp (Analog Buffer) component.
*
*
********************************************************************************
* Copyright 2013-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "Opamp_TIA.h"

#if(!Opamp_TIA_CHECK_DEEPSLEEP_SUPPORT)
    static Opamp_TIA_BACKUP_STRUCT Opamp_TIA_backup =
    {
        0u, /* enableState */
    };
#endif /* (Opamp_TIA_CHECK_DEEPSLEEP_SUPPORT) */


/*******************************************************************************
* Function Name: Opamp_TIA_SaveConfig
********************************************************************************
*
* Summary:
*  Empty function. Included for consistency with other components.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Opamp_TIA_SaveConfig(void)
{

}


/*******************************************************************************
* Function Name: Opamp_TIA_RestoreConfig
********************************************************************************
*
* Summary:
*  Empty function. Included for consistency with other components.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Opamp_TIA_RestoreConfig(void)
{
    
}


/*******************************************************************************
* Function Name: Opamp_TIA_Sleep
********************************************************************************
*
* Summary:
*  This is the preferred API to prepare the component for sleep. The Sleep() API 
*  saves the current component state. Call the Sleep() function before calling the 
*  CySysPmDeepSleep() or the CySysPmHibernate() functions. The "Deep sleep operation" 
*  option has an influence on this function implementation.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  Opamp_TIA_backup: The structure field 'enableState' is modified
*  depending on the enable state of the block before entering the sleep mode.
*
*******************************************************************************/
void Opamp_TIA_Sleep(void)
{
#if(!Opamp_TIA_CHECK_DEEPSLEEP_SUPPORT)
    if(Opamp_TIA_CHECK_PWR_MODE_OFF)
    {
        Opamp_TIA_backup.enableState = 1u;
        Opamp_TIA_Stop();
    }
    else /* The component is disabled */
    {
        Opamp_TIA_backup.enableState = 0u;
    }
#endif /* (Opamp_TIA_CHECK_DEEPSLEEP_SUPPORT) */
}


/*******************************************************************************
* Function Name: Opamp_TIA_Wakeup
********************************************************************************
*
* Summary:
*  This is the preferred API to restore the component to the state when Sleep() 
*  is called. If the component has been enabled before the Sleep() function is 
*  called, the Wakeup() function will also re-enable the component.
*  The "Deep sleep operation" option has an influence on this function
*  implementation.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  Opamp_TIA_backup: The structure field 'enableState' is used to
*  restore the enable state of block after wakeup from sleep mode.
*
*******************************************************************************/
void Opamp_TIA_Wakeup(void)
{
#if(!Opamp_TIA_CHECK_DEEPSLEEP_SUPPORT)
    if(0u != Opamp_TIA_backup.enableState)
    {
        /* Enable Opamp's operation */
        Opamp_TIA_Enable();
    } /* Do nothing if Opamp was disabled before */
#endif /* (Opamp_TIA_CHECK_DEEPSLEEP_SUPPORT) */
}


/* [] END OF FILE */
