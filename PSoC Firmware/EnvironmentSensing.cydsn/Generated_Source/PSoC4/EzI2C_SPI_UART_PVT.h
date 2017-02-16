/***************************************************************************//**
* \file EzI2C_SPI_UART_PVT.h
* \version 3.20
*
* \brief
*  This private file provides constants and parameter values for the
*  SCB Component in SPI and UART modes.
*  Please do not use this file or its content in your project.
*
* Note:
*
********************************************************************************
* \copyright
* Copyright 2013-2016, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SCB_SPI_UART_PVT_EzI2C_H)
#define CY_SCB_SPI_UART_PVT_EzI2C_H

#include "EzI2C_SPI_UART.h"


/***************************************
*     Internal Global Vars
***************************************/

#if (EzI2C_INTERNAL_RX_SW_BUFFER_CONST)
    extern volatile uint32  EzI2C_rxBufferHead;
    extern volatile uint32  EzI2C_rxBufferTail;
    
    /**
    * \addtogroup group_globals
    * @{
    */
    
    /** Sets when internal software receive buffer overflow
     *  was occurred.
    */  
    extern volatile uint8   EzI2C_rxBufferOverflow;
    /** @} globals */
#endif /* (EzI2C_INTERNAL_RX_SW_BUFFER_CONST) */

#if (EzI2C_INTERNAL_TX_SW_BUFFER_CONST)
    extern volatile uint32  EzI2C_txBufferHead;
    extern volatile uint32  EzI2C_txBufferTail;
#endif /* (EzI2C_INTERNAL_TX_SW_BUFFER_CONST) */

#if (EzI2C_INTERNAL_RX_SW_BUFFER)
    extern volatile uint8 EzI2C_rxBufferInternal[EzI2C_INTERNAL_RX_BUFFER_SIZE];
#endif /* (EzI2C_INTERNAL_RX_SW_BUFFER) */

#if (EzI2C_INTERNAL_TX_SW_BUFFER)
    extern volatile uint8 EzI2C_txBufferInternal[EzI2C_TX_BUFFER_SIZE];
#endif /* (EzI2C_INTERNAL_TX_SW_BUFFER) */


/***************************************
*     Private Function Prototypes
***************************************/

void EzI2C_SpiPostEnable(void);
void EzI2C_SpiStop(void);

#if (EzI2C_SCB_MODE_SPI_CONST_CFG)
    void EzI2C_SpiInit(void);
#endif /* (EzI2C_SCB_MODE_SPI_CONST_CFG) */

#if (EzI2C_SPI_WAKE_ENABLE_CONST)
    void EzI2C_SpiSaveConfig(void);
    void EzI2C_SpiRestoreConfig(void);
#endif /* (EzI2C_SPI_WAKE_ENABLE_CONST) */

void EzI2C_UartPostEnable(void);
void EzI2C_UartStop(void);

#if (EzI2C_SCB_MODE_UART_CONST_CFG)
    void EzI2C_UartInit(void);
#endif /* (EzI2C_SCB_MODE_UART_CONST_CFG) */

#if (EzI2C_UART_WAKE_ENABLE_CONST)
    void EzI2C_UartSaveConfig(void);
    void EzI2C_UartRestoreConfig(void);
#endif /* (EzI2C_UART_WAKE_ENABLE_CONST) */


/***************************************
*         UART API Constants
***************************************/

/* UART RX and TX position to be used in EzI2C_SetPins() */
#define EzI2C_UART_RX_PIN_ENABLE    (EzI2C_UART_RX)
#define EzI2C_UART_TX_PIN_ENABLE    (EzI2C_UART_TX)

/* UART RTS and CTS position to be used in  EzI2C_SetPins() */
#define EzI2C_UART_RTS_PIN_ENABLE    (0x10u)
#define EzI2C_UART_CTS_PIN_ENABLE    (0x20u)


/***************************************
* The following code is DEPRECATED and
* must not be used.
***************************************/

/* Interrupt processing */
#define EzI2C_SpiUartEnableIntRx(intSourceMask)  EzI2C_SetRxInterruptMode(intSourceMask)
#define EzI2C_SpiUartEnableIntTx(intSourceMask)  EzI2C_SetTxInterruptMode(intSourceMask)
uint32  EzI2C_SpiUartDisableIntRx(void);
uint32  EzI2C_SpiUartDisableIntTx(void);


#endif /* (CY_SCB_SPI_UART_PVT_EzI2C_H) */


/* [] END OF FILE */
