/***************************************************************************//**
* \file EzI2C_SPI_UART_INT.c
* \version 3.20
*
* \brief
*  This file provides the source code to the Interrupt Service Routine for
*  the SCB Component in SPI and UART modes.
*
* Note:
*
********************************************************************************
* \copyright
* Copyright 2013-2016, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "EzI2C_PVT.h"
#include "EzI2C_SPI_UART_PVT.h"


#if (EzI2C_SCB_IRQ_INTERNAL)
/*******************************************************************************
* Function Name: EzI2C_SPI_UART_ISR
****************************************************************************//**
*
*  Handles the Interrupt Service Routine for the SCB SPI or UART modes.
*
*******************************************************************************/
CY_ISR(EzI2C_SPI_UART_ISR)
{
#if (EzI2C_INTERNAL_RX_SW_BUFFER_CONST)
    uint32 locHead;
#endif /* (EzI2C_INTERNAL_RX_SW_BUFFER_CONST) */

#if (EzI2C_INTERNAL_TX_SW_BUFFER_CONST)
    uint32 locTail;
#endif /* (EzI2C_INTERNAL_TX_SW_BUFFER_CONST) */

#ifdef EzI2C_SPI_UART_ISR_ENTRY_CALLBACK
    EzI2C_SPI_UART_ISR_EntryCallback();
#endif /* EzI2C_SPI_UART_ISR_ENTRY_CALLBACK */

    if (NULL != EzI2C_customIntrHandler)
    {
        EzI2C_customIntrHandler();
    }

    #if(EzI2C_CHECK_SPI_WAKE_ENABLE)
    {
        /* Clear SPI wakeup source */
        EzI2C_ClearSpiExtClkInterruptSource(EzI2C_INTR_SPI_EC_WAKE_UP);
    }
    #endif

    #if (EzI2C_CHECK_RX_SW_BUFFER)
    {
        if (EzI2C_CHECK_INTR_RX_MASKED(EzI2C_INTR_RX_NOT_EMPTY))
        {
            do
            {
                /* Move local head index */
                locHead = (EzI2C_rxBufferHead + 1u);

                /* Adjust local head index */
                if (EzI2C_INTERNAL_RX_BUFFER_SIZE == locHead)
                {
                    locHead = 0u;
                }

                if (locHead == EzI2C_rxBufferTail)
                {
                    #if (EzI2C_CHECK_UART_RTS_CONTROL_FLOW)
                    {
                        /* There is no space in the software buffer - disable the
                        * RX Not Empty interrupt source. The data elements are
                        * still being received into the RX FIFO until the RTS signal
                        * stops the transmitter. After the data element is read from the
                        * buffer, the RX Not Empty interrupt source is enabled to
                        * move the next data element in the software buffer.
                        */
                        EzI2C_INTR_RX_MASK_REG &= ~EzI2C_INTR_RX_NOT_EMPTY;
                        break;
                    }
                    #else
                    {
                        /* Overflow: through away received data element */
                        (void) EzI2C_RX_FIFO_RD_REG;
                        EzI2C_rxBufferOverflow = (uint8) EzI2C_INTR_RX_OVERFLOW;
                    }
                    #endif
                }
                else
                {
                    /* Store received data */
                    EzI2C_PutWordInRxBuffer(locHead, EzI2C_RX_FIFO_RD_REG);

                    /* Move head index */
                    EzI2C_rxBufferHead = locHead;
                }
            }
            while(0u != EzI2C_GET_RX_FIFO_ENTRIES);

            EzI2C_ClearRxInterruptSource(EzI2C_INTR_RX_NOT_EMPTY);
        }
    }
    #endif


    #if (EzI2C_CHECK_TX_SW_BUFFER)
    {
        if (EzI2C_CHECK_INTR_TX_MASKED(EzI2C_INTR_TX_NOT_FULL))
        {
            do
            {
                /* Check for room in TX software buffer */
                if (EzI2C_txBufferHead != EzI2C_txBufferTail)
                {
                    /* Move local tail index */
                    locTail = (EzI2C_txBufferTail + 1u);

                    /* Adjust local tail index */
                    if (EzI2C_TX_BUFFER_SIZE == locTail)
                    {
                        locTail = 0u;
                    }

                    /* Put data into TX FIFO */
                    EzI2C_TX_FIFO_WR_REG = EzI2C_GetWordFromTxBuffer(locTail);

                    /* Move tail index */
                    EzI2C_txBufferTail = locTail;
                }
                else
                {
                    /* TX software buffer is empty: complete transfer */
                    EzI2C_DISABLE_INTR_TX(EzI2C_INTR_TX_NOT_FULL);
                    break;
                }
            }
            while (EzI2C_SPI_UART_FIFO_SIZE != EzI2C_GET_TX_FIFO_ENTRIES);

            EzI2C_ClearTxInterruptSource(EzI2C_INTR_TX_NOT_FULL);
        }
    }
    #endif

#ifdef EzI2C_SPI_UART_ISR_EXIT_CALLBACK
    EzI2C_SPI_UART_ISR_ExitCallback();
#endif /* EzI2C_SPI_UART_ISR_EXIT_CALLBACK */

}

#endif /* (EzI2C_SCB_IRQ_INTERNAL) */


/* [] END OF FILE */
