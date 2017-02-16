/*******************************************************************************
* File Name: ExcitationPWM_PM.c
* Version 2.10
*
* Description:
*  This file contains the setup, control, and status commands to support
*  the component operations in the low power mode.
*
* Note:
*  None
*
********************************************************************************
* Copyright 2013-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "ExcitationPWM.h"

static ExcitationPWM_BACKUP_STRUCT ExcitationPWM_backup;


/*******************************************************************************
* Function Name: ExcitationPWM_SaveConfig
********************************************************************************
*
* Summary:
*  All configuration registers are retention. Nothing to save here.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void ExcitationPWM_SaveConfig(void)
{

}


/*******************************************************************************
* Function Name: ExcitationPWM_Sleep
********************************************************************************
*
* Summary:
*  Stops the component operation and saves the user configuration.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void ExcitationPWM_Sleep(void)
{
    if(0u != (ExcitationPWM_BLOCK_CONTROL_REG & ExcitationPWM_MASK))
    {
        ExcitationPWM_backup.enableState = 1u;
    }
    else
    {
        ExcitationPWM_backup.enableState = 0u;
    }

    ExcitationPWM_Stop();
    ExcitationPWM_SaveConfig();
}


/*******************************************************************************
* Function Name: ExcitationPWM_RestoreConfig
********************************************************************************
*
* Summary:
*  All configuration registers are retention. Nothing to restore here.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void ExcitationPWM_RestoreConfig(void)
{

}


/*******************************************************************************
* Function Name: ExcitationPWM_Wakeup
********************************************************************************
*
* Summary:
*  Restores the user configuration and restores the enable state.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void ExcitationPWM_Wakeup(void)
{
    ExcitationPWM_RestoreConfig();

    if(0u != ExcitationPWM_backup.enableState)
    {
        ExcitationPWM_Enable();
    }
}


/* [] END OF FILE */
