/***************************************************************************//**
* \file EzI2C_SPI_UART.c
* \version 3.20
*
* \brief
*  This file provides the source code to the API for the SCB Component in
*  SPI and UART modes.
*
* Note:
*
*******************************************************************************
* \copyright
* Copyright 2013-2016, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "EzI2C_PVT.h"
#include "EzI2C_SPI_UART_PVT.h"

/***************************************
*        SPI/UART Private Vars
***************************************/

#if(EzI2C_INTERNAL_RX_SW_BUFFER_CONST)
    /* Start index to put data into the software receive buffer.*/
    volatile uint32 EzI2C_rxBufferHead;
    /* Start index to get data from the software receive buffer.*/
    volatile uint32 EzI2C_rxBufferTail;
    /**
    * \addtogroup group_globals
    * \{
    */
    /** Sets when internal software receive buffer overflow
    *  was occurred.
    */
    volatile uint8  EzI2C_rxBufferOverflow;
    /** \} globals */
#endif /* (EzI2C_INTERNAL_RX_SW_BUFFER_CONST) */

#if(EzI2C_INTERNAL_TX_SW_BUFFER_CONST)
    /* Start index to put data into the software transmit buffer.*/
    volatile uint32 EzI2C_txBufferHead;
    /* Start index to get data from the software transmit buffer.*/
    volatile uint32 EzI2C_txBufferTail;
#endif /* (EzI2C_INTERNAL_TX_SW_BUFFER_CONST) */

#if(EzI2C_INTERNAL_RX_SW_BUFFER)
    /* Add one element to the buffer to receive full packet. One byte in receive buffer is always empty */
    volatile uint8 EzI2C_rxBufferInternal[EzI2C_INTERNAL_RX_BUFFER_SIZE];
#endif /* (EzI2C_INTERNAL_RX_SW_BUFFER) */

#if(EzI2C_INTERNAL_TX_SW_BUFFER)
    volatile uint8 EzI2C_txBufferInternal[EzI2C_TX_BUFFER_SIZE];
#endif /* (EzI2C_INTERNAL_TX_SW_BUFFER) */


#if(EzI2C_RX_DIRECTION)
    /*******************************************************************************
    * Function Name: EzI2C_SpiUartReadRxData
    ****************************************************************************//**
    *
    *  Retrieves the next data element from the receive buffer.
    *   - RX software buffer is disabled: Returns data element retrieved from
    *     RX FIFO. Undefined data will be returned if the RX FIFO is empty.
    *   - RX software buffer is enabled: Returns data element from the software
    *     receive buffer. Zero value is returned if the software receive buffer
    *     is empty.
    *
    * \return
    *  Next data element from the receive buffer. 
    *  The amount of data bits to be received depends on RX data bits selection 
    *  (the data bit counting starts from LSB of return value).
    *
    * \globalvars
    *  EzI2C_rxBufferHead - the start index to put data into the 
    *  software receive buffer.
    *  EzI2C_rxBufferTail - the start index to get data from the 
    *  software receive buffer.
    *
    *******************************************************************************/
    uint32 EzI2C_SpiUartReadRxData(void)
    {
        uint32 rxData = 0u;

    #if (EzI2C_INTERNAL_RX_SW_BUFFER_CONST)
        uint32 locTail;
    #endif /* (EzI2C_INTERNAL_RX_SW_BUFFER_CONST) */

        #if (EzI2C_CHECK_RX_SW_BUFFER)
        {
            if (EzI2C_rxBufferHead != EzI2C_rxBufferTail)
            {
                /* There is data in RX software buffer */

                /* Calculate index to read from */
                locTail = (EzI2C_rxBufferTail + 1u);

                if (EzI2C_INTERNAL_RX_BUFFER_SIZE == locTail)
                {
                    locTail = 0u;
                }

                /* Get data from RX software buffer */
                rxData = EzI2C_GetWordFromRxBuffer(locTail);

                /* Change index in the buffer */
                EzI2C_rxBufferTail = locTail;

                #if (EzI2C_CHECK_UART_RTS_CONTROL_FLOW)
                {
                    /* Check if RX Not Empty is disabled in the interrupt */
                    if (0u == (EzI2C_INTR_RX_MASK_REG & EzI2C_INTR_RX_NOT_EMPTY))
                    {
                        /* Enable RX Not Empty interrupt source to continue
                        * receiving data into software buffer.
                        */
                        EzI2C_INTR_RX_MASK_REG |= EzI2C_INTR_RX_NOT_EMPTY;
                    }
                }
                #endif

            }
        }
        #else
        {
            /* Read data from RX FIFO */
            rxData = EzI2C_RX_FIFO_RD_REG;
        }
        #endif

        return (rxData);
    }


    /*******************************************************************************
    * Function Name: EzI2C_SpiUartGetRxBufferSize
    ****************************************************************************//**
    *
    *  Returns the number of received data elements in the receive buffer.
    *   - RX software buffer disabled: returns the number of used entries in
    *     RX FIFO.
    *   - RX software buffer enabled: returns the number of elements which were
    *     placed in the receive buffer. This does not include the hardware RX FIFO.
    *
    * \return
    *  Number of received data elements.
    *
    * \globalvars
    *  EzI2C_rxBufferHead - the start index to put data into the 
    *  software receive buffer.
    *  EzI2C_rxBufferTail - the start index to get data from the 
    *  software receive buffer.
    *
    *******************************************************************************/
    uint32 EzI2C_SpiUartGetRxBufferSize(void)
    {
        uint32 size;
    #if (EzI2C_INTERNAL_RX_SW_BUFFER_CONST)
        uint32 locHead;
    #endif /* (EzI2C_INTERNAL_RX_SW_BUFFER_CONST) */

        #if (EzI2C_CHECK_RX_SW_BUFFER)
        {
            locHead = EzI2C_rxBufferHead;

            if(locHead >= EzI2C_rxBufferTail)
            {
                size = (locHead - EzI2C_rxBufferTail);
            }
            else
            {
                size = (locHead + (EzI2C_INTERNAL_RX_BUFFER_SIZE - EzI2C_rxBufferTail));
            }
        }
        #else
        {
            size = EzI2C_GET_RX_FIFO_ENTRIES;
        }
        #endif

        return (size);
    }


    /*******************************************************************************
    * Function Name: EzI2C_SpiUartClearRxBuffer
    ****************************************************************************//**
    *
    *  Clears the receive buffer and RX FIFO.
    *
    * \globalvars
    *  EzI2C_rxBufferHead - the start index to put data into the 
    *  software receive buffer.
    *  EzI2C_rxBufferTail - the start index to get data from the 
    *  software receive buffer.
    *
    *******************************************************************************/
    void EzI2C_SpiUartClearRxBuffer(void)
    {
        #if (EzI2C_CHECK_RX_SW_BUFFER)
        {
            /* Lock from component interruption */
            EzI2C_DisableInt();

            /* Flush RX software buffer */
            EzI2C_rxBufferHead = EzI2C_rxBufferTail;
            EzI2C_rxBufferOverflow = 0u;

            EzI2C_CLEAR_RX_FIFO;
            EzI2C_ClearRxInterruptSource(EzI2C_INTR_RX_ALL);

            #if (EzI2C_CHECK_UART_RTS_CONTROL_FLOW)
            {
                /* Enable RX Not Empty interrupt source to continue receiving
                * data into software buffer.
                */
                EzI2C_INTR_RX_MASK_REG |= EzI2C_INTR_RX_NOT_EMPTY;
            }
            #endif
            
            /* Release lock */
            EzI2C_EnableInt();
        }
        #else
        {
            EzI2C_CLEAR_RX_FIFO;
        }
        #endif
    }

#endif /* (EzI2C_RX_DIRECTION) */


#if(EzI2C_TX_DIRECTION)
    /*******************************************************************************
    * Function Name: EzI2C_SpiUartWriteTxData
    ****************************************************************************//**
    *
    *  Places a data entry into the transmit buffer to be sent at the next available
    *  bus time.
    *  This function is blocking and waits until there is space available to put the
    *  requested data in the transmit buffer.
    *
    *  \param txDataByte: the data to be transmitted.
    *   The amount of data bits to be transmitted depends on TX data bits selection 
    *   (the data bit counting starts from LSB of txDataByte).
    *
    * \globalvars
    *  EzI2C_txBufferHead - the start index to put data into the 
    *  software transmit buffer.
    *  EzI2C_txBufferTail - start index to get data from the software
    *  transmit buffer.
    *
    *******************************************************************************/
    void EzI2C_SpiUartWriteTxData(uint32 txData)
    {
    #if (EzI2C_INTERNAL_TX_SW_BUFFER_CONST)
        uint32 locHead;
    #endif /* (EzI2C_INTERNAL_TX_SW_BUFFER_CONST) */

        #if (EzI2C_CHECK_TX_SW_BUFFER)
        {
            /* Put data directly into the TX FIFO */
            if ((EzI2C_txBufferHead == EzI2C_txBufferTail) &&
                (EzI2C_SPI_UART_FIFO_SIZE != EzI2C_GET_TX_FIFO_ENTRIES))
            {
                /* TX software buffer is empty: put data directly in TX FIFO */
                EzI2C_TX_FIFO_WR_REG = txData;
            }
            /* Put data into TX software buffer */
            else
            {
                /* Head index to put data */
                locHead = (EzI2C_txBufferHead + 1u);

                /* Adjust TX software buffer index */
                if (EzI2C_TX_BUFFER_SIZE == locHead)
                {
                    locHead = 0u;
                }

                /* Wait for space in TX software buffer */
                while (locHead == EzI2C_txBufferTail)
                {
                }

                /* TX software buffer has at least one room */

                /* Clear old status of INTR_TX_NOT_FULL. It sets at the end of transfer when TX FIFO is empty. */
                EzI2C_ClearTxInterruptSource(EzI2C_INTR_TX_NOT_FULL);

                EzI2C_PutWordInTxBuffer(locHead, txData);

                EzI2C_txBufferHead = locHead;

                /* Check if TX Not Full is disabled in interrupt */
                if (0u == (EzI2C_INTR_TX_MASK_REG & EzI2C_INTR_TX_NOT_FULL))
                {
                    /* Enable TX Not Full interrupt source to transmit from software buffer */
                    EzI2C_INTR_TX_MASK_REG |= (uint32) EzI2C_INTR_TX_NOT_FULL;
                }
            }
        }
        #else
        {
            /* Wait until TX FIFO has space to put data element */
            while (EzI2C_SPI_UART_FIFO_SIZE == EzI2C_GET_TX_FIFO_ENTRIES)
            {
            }

            EzI2C_TX_FIFO_WR_REG = txData;
        }
        #endif
    }


    /*******************************************************************************
    * Function Name: EzI2C_SpiUartPutArray
    ****************************************************************************//**
    *
    *  Places an array of data into the transmit buffer to be sent.
    *  This function is blocking and waits until there is a space available to put
    *  all the requested data in the transmit buffer. The array size can be greater
    *  than transmit buffer size.
    *
    * \param wrBuf: pointer to an array of data to be placed in transmit buffer. 
    *  The width of the data to be transmitted depends on TX data width selection 
    *  (the data bit counting starts from LSB for each array element).
    * \param count: number of data elements to be placed in the transmit buffer.
    *
    * \globalvars
    *  EzI2C_txBufferHead - the start index to put data into the 
    *  software transmit buffer.
    *  EzI2C_txBufferTail - start index to get data from the software
    *  transmit buffer.
    *
    *******************************************************************************/
    void EzI2C_SpiUartPutArray(const uint8 wrBuf[], uint32 count)
    {
        uint32 i;

        for (i=0u; i < count; i++)
        {
            EzI2C_SpiUartWriteTxData((uint32) wrBuf[i]);
        }
    }


    /*******************************************************************************
    * Function Name: EzI2C_SpiUartGetTxBufferSize
    ****************************************************************************//**
    *
    *  Returns the number of elements currently in the transmit buffer.
    *   - TX software buffer is disabled: returns the number of used entries in
    *     TX FIFO.
    *   - TX software buffer is enabled: returns the number of elements currently
    *     used in the transmit buffer. This number does not include used entries in
    *     the TX FIFO. The transmit buffer size is zero until the TX FIFO is
    *     not full.
    *
    * \return
    *  Number of data elements ready to transmit.
    *
    * \globalvars
    *  EzI2C_txBufferHead - the start index to put data into the 
    *  software transmit buffer.
    *  EzI2C_txBufferTail - start index to get data from the software
    *  transmit buffer.
    *
    *******************************************************************************/
    uint32 EzI2C_SpiUartGetTxBufferSize(void)
    {
        uint32 size;
    #if (EzI2C_INTERNAL_TX_SW_BUFFER_CONST)
        uint32 locTail;
    #endif /* (EzI2C_INTERNAL_TX_SW_BUFFER_CONST) */

        #if (EzI2C_CHECK_TX_SW_BUFFER)
        {
            /* Get current Tail index */
            locTail = EzI2C_txBufferTail;

            if (EzI2C_txBufferHead >= locTail)
            {
                size = (EzI2C_txBufferHead - locTail);
            }
            else
            {
                size = (EzI2C_txBufferHead + (EzI2C_TX_BUFFER_SIZE - locTail));
            }
        }
        #else
        {
            size = EzI2C_GET_TX_FIFO_ENTRIES;
        }
        #endif

        return (size);
    }


    /*******************************************************************************
    * Function Name: EzI2C_SpiUartClearTxBuffer
    ****************************************************************************//**
    *
    *  Clears the transmit buffer and TX FIFO.
    *
    * \globalvars
    *  EzI2C_txBufferHead - the start index to put data into the 
    *  software transmit buffer.
    *  EzI2C_txBufferTail - start index to get data from the software
    *  transmit buffer.
    *
    *******************************************************************************/
    void EzI2C_SpiUartClearTxBuffer(void)
    {
        #if (EzI2C_CHECK_TX_SW_BUFFER)
        {
            /* Lock from component interruption */
            EzI2C_DisableInt();

            /* Flush TX software buffer */
            EzI2C_txBufferHead = EzI2C_txBufferTail;

            EzI2C_INTR_TX_MASK_REG &= (uint32) ~EzI2C_INTR_TX_NOT_FULL;
            EzI2C_CLEAR_TX_FIFO;
            EzI2C_ClearTxInterruptSource(EzI2C_INTR_TX_ALL);

            /* Release lock */
            EzI2C_EnableInt();
        }
        #else
        {
            EzI2C_CLEAR_TX_FIFO;
        }
        #endif
    }

#endif /* (EzI2C_TX_DIRECTION) */


/*******************************************************************************
* Function Name: EzI2C_SpiUartDisableIntRx
****************************************************************************//**
*
*  Disables the RX interrupt sources.
*
*  \return
*   Returns the RX interrupt sources enabled before the function call.
*
*******************************************************************************/
uint32 EzI2C_SpiUartDisableIntRx(void)
{
    uint32 intSource;

    intSource = EzI2C_GetRxInterruptMode();

    EzI2C_SetRxInterruptMode(EzI2C_NO_INTR_SOURCES);

    return (intSource);
}


/*******************************************************************************
* Function Name: EzI2C_SpiUartDisableIntTx
****************************************************************************//**
*
*  Disables TX interrupt sources.
*
*  \return
*   Returns TX interrupt sources enabled before function call.
*
*******************************************************************************/
uint32 EzI2C_SpiUartDisableIntTx(void)
{
    uint32 intSourceMask;

    intSourceMask = EzI2C_GetTxInterruptMode();

    EzI2C_SetTxInterruptMode(EzI2C_NO_INTR_SOURCES);

    return (intSourceMask);
}


#if(EzI2C_SCB_MODE_UNCONFIG_CONST_CFG)
    /*******************************************************************************
    * Function Name: EzI2C_PutWordInRxBuffer
    ****************************************************************************//**
    *
    *  Stores a byte/word into the RX buffer.
    *  Only available in the Unconfigured operation mode.
    *
    *  \param index:      index to store data byte/word in the RX buffer.
    *  \param rxDataByte: byte/word to store.
    *
    *******************************************************************************/
    void EzI2C_PutWordInRxBuffer(uint32 idx, uint32 rxDataByte)
    {
        /* Put data in buffer */
        if (EzI2C_ONE_BYTE_WIDTH == EzI2C_rxDataBits)
        {
            EzI2C_rxBuffer[idx] = ((uint8) rxDataByte);
        }
        else
        {
            EzI2C_rxBuffer[(uint32)(idx << 1u)]      = LO8(LO16(rxDataByte));
            EzI2C_rxBuffer[(uint32)(idx << 1u) + 1u] = HI8(LO16(rxDataByte));
        }
    }


    /*******************************************************************************
    * Function Name: EzI2C_GetWordFromRxBuffer
    ****************************************************************************//**
    *
    *  Reads byte/word from RX buffer.
    *  Only available in the Unconfigured operation mode.
    *
    *  \return
    *   Returns byte/word read from RX buffer.
    *
    *******************************************************************************/
    uint32 EzI2C_GetWordFromRxBuffer(uint32 idx)
    {
        uint32 value;

        if (EzI2C_ONE_BYTE_WIDTH == EzI2C_rxDataBits)
        {
            value = EzI2C_rxBuffer[idx];
        }
        else
        {
            value  = (uint32) EzI2C_rxBuffer[(uint32)(idx << 1u)];
            value |= (uint32) ((uint32)EzI2C_rxBuffer[(uint32)(idx << 1u) + 1u] << 8u);
        }

        return (value);
    }


    /*******************************************************************************
    * Function Name: EzI2C_PutWordInTxBuffer
    ****************************************************************************//**
    *
    *  Stores byte/word into the TX buffer.
    *  Only available in the Unconfigured operation mode.
    *
    *  \param idx:        index to store data byte/word in the TX buffer.
    *  \param txDataByte: byte/word to store.
    *
    *******************************************************************************/
    void EzI2C_PutWordInTxBuffer(uint32 idx, uint32 txDataByte)
    {
        /* Put data in buffer */
        if (EzI2C_ONE_BYTE_WIDTH == EzI2C_txDataBits)
        {
            EzI2C_txBuffer[idx] = ((uint8) txDataByte);
        }
        else
        {
            EzI2C_txBuffer[(uint32)(idx << 1u)]      = LO8(LO16(txDataByte));
            EzI2C_txBuffer[(uint32)(idx << 1u) + 1u] = HI8(LO16(txDataByte));
        }
    }


    /*******************************************************************************
    * Function Name: EzI2C_GetWordFromTxBuffer
    ****************************************************************************//**
    *
    *  Reads byte/word from the TX buffer.
    *  Only available in the Unconfigured operation mode.
    *
    *  \param idx: index to get data byte/word from the TX buffer.
    *
    *  \return
    *   Returns byte/word read from the TX buffer.
    *
    *******************************************************************************/
    uint32 EzI2C_GetWordFromTxBuffer(uint32 idx)
    {
        uint32 value;

        if (EzI2C_ONE_BYTE_WIDTH == EzI2C_txDataBits)
        {
            value = (uint32) EzI2C_txBuffer[idx];
        }
        else
        {
            value  = (uint32) EzI2C_txBuffer[(uint32)(idx << 1u)];
            value |= (uint32) ((uint32) EzI2C_txBuffer[(uint32)(idx << 1u) + 1u] << 8u);
        }

        return (value);
    }

#endif /* (EzI2C_SCB_MODE_UNCONFIG_CONST_CFG) */


/* [] END OF FILE */
