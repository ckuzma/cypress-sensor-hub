/***************************************************************************//**
* \file     IPS_PGA.c
* \version  1.0
*
* \brief
*  This file provides the source code to the API for the PGA_P4 Component
*
********************************************************************************
* \copyright
* Copyright 2016, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "IPS_PGA_PVT.h"


uint8 IPS_PGA_initVar = 0u; /**< Describes the init state of the component */

uint32 IPS_PGA_internalGainPower = 0u; /**< Stores component Gain, Power and Enable values */

/***************************************************************************//**
* Compensation table 
*
* There is a feedback compensation capacitor recomended setting (for certain 
* gain and power setting) under the mask 0x0F000000u
* or (IPS_PGA_C_FB_MASK)
*
* Also there is an OpAmp compensation recomended setting (for certain 
* gain and power setting) under the mask 0x00000003u
* or (IPS_PGA_GET_OA_COMP_TRIM)
*
* There is only 6 values for whole gain range because the recommended 
* values are for each pair of adjacent gain values, e.g.:
* the first (index = 0) value is for gain = 1 and 1.4,
* the second (index = 1) value is for gain = 2 and 2.8,and so on,
* and the sixth (index = 5) value is for gain = 32 only.
*******************************************************************************/
const uint32 IPS_PGA_compTab[IPS_PGA_COMP_TAB_HEIGHT][IPS_PGA_COMP_TAB_WIDTH] =
{
#if (IPS_PGA_CHECK_OUTPUT_MODE) /* Class AB */
    {0x0F000002u, 0x0F000002u, 0x0F000002u},
    {0x07000002u, 0x09000002u, 0x09000002u},
    {0x04000002u, 0x04000002u, 0x04000002u},
    {0x03000001u, 0x02000002u, 0x03000002u},
    {0x01000001u, 0x00000002u, 0x00000002u},
    {0x01000001u, 0x01000001u, 0x02000001u}
#else /* Class A */
    {0x0F000002u, 0x0F000002u, 0x0F000002u},
    {0x08000002u, 0x09000002u, 0x09000002u},
    {0x03000001u, 0x05000002u, 0x04000002u},
    {0x00000001u, 0x02000002u, 0x03000002u},
    {0x00000001u, 0x00000002u, 0x01000002u},
    {0x00000001u, 0x00000001u, 0x00000001u}
#endif /* IPS_PGA_OUTPUT_MODE */
};


/*******************************************************************************   
* Function Name: IPS_PGA_Init
****************************************************************************//**
*
*  Initializes component's parameters to the values set by user in the 
*  customizer of the component placed onto schematic. Usually it is called in 
*  IPS_PGA_Start().
*
*******************************************************************************/
void IPS_PGA_Init(void)
{
    /* Set default internal variable value */
    IPS_PGA_internalGainPower = IPS_PGA_DEFAULT_GAIN_POWER;
    
    /* Set default register values */
    IPS_PGA_CTB_CTRL_REG |= IPS_PGA_DEFAULT_CTB_CTRL;
    IPS_PGA_OA_RES_CTRL_REG &= (uint32) ~IPS_PGA_DEFAULT_OA_RES_CTRL_MASK;
    IPS_PGA_OA_RES_CTRL_REG |= IPS_PGA_DEFAULT_OA_RES_CTRL;
    
    #if (IPS_PGA_VREF_MODE == IPS_PGA_VREF_INTERNAL)
        /* Configure resistor matrix bottom to the internal Vss */
        IPS_PGA_OA_SW_REG |= IPS_PGA_RBOT_TO_VSSA;
    #endif /* Vref Internal */
    
    /* Check if component has been enabled */
    if((IPS_PGA_OA_RES_CTRL_REG & IPS_PGA_OA_PWR_MODE_MASK) != 0u)
    {
        /* Set the default power level */
        IPS_PGA_Enable();
    }
    
    /* Set default gain and correspondent Cfb and OA_trim values */
    IPS_PGA_SetGain(IPS_PGA_GAIN);
}


/*******************************************************************************   
* Function Name: IPS_PGA_Enable
****************************************************************************//**
*
*  Powers up amplifier (to default power level or restored after 
*  previous IPS_PGA_Stop() call).
*  Usually it is called in IPS_PGA_Start().
*  
*******************************************************************************/
void IPS_PGA_Enable(void)
{
    IPS_PGA_OA_RES_CTRL_REG |= IPS_PGA_GET_POWER;
}


/*******************************************************************************
* Function Name: IPS_PGA_Start
****************************************************************************//**
*
*  Enables the amplifier operation (calls IPS_PGA_Enable).
*  Also on the first call (after the system reset) initializes all the component
*  related registers with default values (calls IPS_PGA_Init).
*
*  \globalvars
*   The IPS_PGA_initVar variable is used to indicate initial
*   configuration of this component. The variable is initialized to zero (0u)
*   and set to one (1u) at the first time IPS_PGA_Start() is called.
*   This allows to enable the component without re-initialization in all
*   subsequent calls of the IPS_PGA_Start() routine.
*
*******************************************************************************/
void IPS_PGA_Start(void)
{
    if(IPS_PGA_initVar == 0u)
    {
        IPS_PGA_Init();
        IPS_PGA_initVar = 1u;
    }

    IPS_PGA_Enable();
}


/*******************************************************************************
* Function Name: IPS_PGA_Stop
****************************************************************************//**
*
*  Powers down the amplifier.
*
*******************************************************************************/
void IPS_PGA_Stop(void)
{ 
    IPS_PGA_OA_RES_CTRL_REG &= (uint32)~IPS_PGA_OA_PWR_MODE_MASK;
}


/*******************************************************************************
* Function Name: IPS_PGA_SetPower
****************************************************************************//**
*
*  Sets the power level of amplifier.
*
* \param powerLevel
*  The power level setting of amplifier. Possible values:
*   * IPS_PGA_LOW - The lowest power consumption.
*   * IPS_PGA_MED - The middle setting.
*   * IPS_PGA_HIGH - The highest amplifier bandwidth.
*
*  \internal
*   The IPS_PGA_internalGainPower variable is used to store the
*   current gain and power level to set appropriate compensation settings.
*
*******************************************************************************/
void IPS_PGA_SetPower(uint32 powerLevel)
{
    uint32 locTmp;
    
    /* Save the powerLevel */
    IPS_PGA_internalGainPower &= (uint32) ~IPS_PGA_OA_PWR_MODE_MASK;
    IPS_PGA_internalGainPower |= powerLevel & IPS_PGA_OA_PWR_MODE_MASK;
    
    /* Save the rest of register bitfields */
    locTmp = IPS_PGA_OA_RES_CTRL_REG &
        (uint32)~(IPS_PGA_OA_PWR_MODE_MASK | IPS_PGA_C_FB_MASK);
           
    /* Set the power and the feedback capacitor values into the control register */
    IPS_PGA_OA_RES_CTRL_REG  = locTmp | IPS_PGA_GET_POWER | IPS_PGA_GET_C_FB;
    
    /* Set the OA compensation capacitor value */
    IPS_PGA_OA_COMP_TRIM_REG = IPS_PGA_GET_OA_COMP_TRIM;
}


/*******************************************************************************
* Function Name: IPS_PGA_SetGain
****************************************************************************//**
*
*  Sets values of the input and feedback resistors to set a 
*  specific gain of the amplifier.
*
* \param gainLevel
*  The gain setting of amplifier. Possible values:
*   * IPS_PGA_GAIN_1    - gain 1.0.
*   * IPS_PGA_GAIN_1_4  - gain 1.4.
*   * IPS_PGA_GAIN_2    - gain 2.0.
*   * IPS_PGA_GAIN_2_8  - gain 2.8.
*   * IPS_PGA_GAIN_4    - gain 4.0.
*   * IPS_PGA_GAIN_5_8  - gain 5.8.
*   * IPS_PGA_GAIN_8    - gain 8.0.
*   * IPS_PGA_GAIN_10_7 - gain 10.7.
*   * IPS_PGA_GAIN_16   - gain 16.0
*   * IPS_PGA_GAIN_21_3 - gain 21.3.
*   * IPS_PGA_GAIN_32   - gain 32.0.
*
*  \internal
*   The IPS_PGA_internalGainPower variable is used to store the
*   current gain and power level to set appropriate compensation settings.
*
*******************************************************************************/
void IPS_PGA_SetGain(uint32 gainLevel)
{
    uint32 locTmp;
    
    /* Save the gainLevel */
    IPS_PGA_internalGainPower &= (uint32) ~IPS_PGA_RES_TAP_MASK;
    IPS_PGA_internalGainPower |= ((uint32)(gainLevel << IPS_PGA_RES_TAP_SHIFT)) &
                                                                 IPS_PGA_RES_TAP_MASK;
    /* Save the rest of register bitfields */
    locTmp = IPS_PGA_OA_RES_CTRL_REG &
        (uint32)~(IPS_PGA_RES_TAP_MASK | IPS_PGA_C_FB_MASK);
    
    /* Set the gain and the feedback capacitor values into the control register */
    IPS_PGA_OA_RES_CTRL_REG = locTmp | (IPS_PGA_internalGainPower &
        IPS_PGA_RES_TAP_MASK) | IPS_PGA_GET_C_FB;
    
    /* Set the OA compensation capacitor value */
    IPS_PGA_OA_COMP_TRIM_REG = IPS_PGA_GET_OA_COMP_TRIM;
}


/*******************************************************************************
* Function Name: IPS_PGA_PumpControl
****************************************************************************//**
*
*  Allows the user to turn the amplifier's boost pump on or off.
*  By Default the IPS_PGA_Init() function turns the pump on.
*  
* \param onOff
*  The boost pump setting. Possible values:
*   * IPS_PGA_BOOST_OFF - Turn off the pump.
*   * IPS_PGA_BOOST_ON  - Turn on the pump.
*
**********************************************************************************/
void IPS_PGA_PumpControl(uint32 onOff)
{
    if(onOff == IPS_PGA_BOOST_ON)
    {
        IPS_PGA_OA_RES_CTRL_REG |= IPS_PGA_OA_PUMP_EN;
    }
    else
    {
        IPS_PGA_OA_RES_CTRL_REG &= (uint32)~IPS_PGA_OA_PUMP_EN;
    }
}


/* [] END OF FILE */
