/***************************************************************************//**
* \file     IPS_PGA_PM.c  
* \version  1.0
*
* \brief
*  This file provides the power management source code to the API for PGA_P4 
*  Component.
*
********************************************************************************
* \copyright
* Copyright 2016, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "IPS_PGA_PVT.h"


/*******************************************************************************
* Function Name: IPS_PGA_SaveConfig
****************************************************************************//**
*
*  Empty function. Included for consistency with other components.
*
*******************************************************************************/
void IPS_PGA_SaveConfig(void)
{
    /* Nothing to save as registers are System reset on retention flops */
}


/*******************************************************************************  
* Function Name: IPS_PGA_RestoreConfig
****************************************************************************//**
*
*  Empty function. Included for consistency with other components.
*
*******************************************************************************/
void IPS_PGA_RestoreConfig(void)
{
    /* Nothing to restore */
}


/*******************************************************************************   
* Function Name: IPS_PGA_Sleep
****************************************************************************//**
*
*  When the "Deep sleep operation" is disabled then the function disables 
*  component's operation and saves its configuration. Should be called 
*  just prior to entering sleep.
*  When the "Deep sleep operation" is enabled then the function does nothing
*  and the component continues to operate during Deep Sleep state.
*
*  \internal
*   The IPS_PGA_backup.enableState is modified 
*   depending on the enable state of the block before entering to sleep mode.
*
*******************************************************************************/
void IPS_PGA_Sleep(void)
{
    #if(!IPS_PGA_CHECK_DEEPSLEEP_SUPPORT)
        if((IPS_PGA_OA_RES_CTRL_REG & IPS_PGA_OA_PWR_MODE_MASK) != 0u)
        {
            IPS_PGA_internalGainPower |= IPS_PGA_ENABLED;
            IPS_PGA_Stop();
        }
        else /* The component is disabled */
        {
            IPS_PGA_internalGainPower &= (uint32) ~IPS_PGA_ENABLED;
        }
        /* Save the configuration */
        IPS_PGA_SaveConfig();
    #endif /* (IPS_PGA_CHECK_DEEPSLEEP_SUPPORT) */
}


/*******************************************************************************
* Function Name: IPS_PGA_Wakeup
****************************************************************************//**
*
*  When the "Deep sleep operation" is disabled then the function enables
*  block's operation and restores its configuration. Should be called
*  just after awaking from sleep.
*  When the "Deep sleep operation" is enabled then the function does nothing
*  and the component continues to operate during Active state.
*
*  \internal
*   The IPS_PGA_backup.enableState is used to 
*   restore the enable state of block after wakeup from sleep mode.
* 
*******************************************************************************/
void IPS_PGA_Wakeup(void)
{
    #if(!IPS_PGA_CHECK_DEEPSLEEP_SUPPORT)
        /* Restore the configurations */
        IPS_PGA_RestoreConfig();
        /* Enables the component operation */
        if((IPS_PGA_internalGainPower & IPS_PGA_ENABLED) != 0u)
        {
            IPS_PGA_Enable();
        } /* Do nothing if component was disabled before */
    #endif /* (IPS_PGA_CHECK_DEEPSLEEP_SUPPORT) */
}


/* [] END OF FILE */
