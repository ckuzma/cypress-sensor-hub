/***************************************************************************//**
* \file EzI2C_UART.c
* \version 3.20
*
* \brief
*  This file provides the source code to the API for the SCB Component in
*  UART mode.
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


#if (EzI2C_UART_WAKE_ENABLE_CONST && EzI2C_UART_RX_WAKEUP_IRQ)
    /**
    * \addtogroup group_globals
    * \{
    */
    /** This global variable determines whether to enable Skip Start
    * functionality when EzI2C_Sleep() function is called:
    * 0 – disable, other values – enable. Default value is 1.
    * It is only available when Enable wakeup from Deep Sleep Mode is enabled.
    */
    uint8 EzI2C_skipStart = 1u;
    /** \} globals */
#endif /* (EzI2C_UART_WAKE_ENABLE_CONST && EzI2C_UART_RX_WAKEUP_IRQ) */

#if(EzI2C_SCB_MODE_UNCONFIG_CONST_CFG)

    /***************************************
    *  Configuration Structure Initialization
    ***************************************/

    const EzI2C_UART_INIT_STRUCT EzI2C_configUart =
    {
        EzI2C_UART_SUB_MODE,
        EzI2C_UART_DIRECTION,
        EzI2C_UART_DATA_BITS_NUM,
        EzI2C_UART_PARITY_TYPE,
        EzI2C_UART_STOP_BITS_NUM,
        EzI2C_UART_OVS_FACTOR,
        EzI2C_UART_IRDA_LOW_POWER,
        EzI2C_UART_MEDIAN_FILTER_ENABLE,
        EzI2C_UART_RETRY_ON_NACK,
        EzI2C_UART_IRDA_POLARITY,
        EzI2C_UART_DROP_ON_PARITY_ERR,
        EzI2C_UART_DROP_ON_FRAME_ERR,
        EzI2C_UART_WAKE_ENABLE,
        0u,
        NULL,
        0u,
        NULL,
        EzI2C_UART_MP_MODE_ENABLE,
        EzI2C_UART_MP_ACCEPT_ADDRESS,
        EzI2C_UART_MP_RX_ADDRESS,
        EzI2C_UART_MP_RX_ADDRESS_MASK,
        (uint32) EzI2C_SCB_IRQ_INTERNAL,
        EzI2C_UART_INTR_RX_MASK,
        EzI2C_UART_RX_TRIGGER_LEVEL,
        EzI2C_UART_INTR_TX_MASK,
        EzI2C_UART_TX_TRIGGER_LEVEL,
        (uint8) EzI2C_UART_BYTE_MODE_ENABLE,
        (uint8) EzI2C_UART_CTS_ENABLE,
        (uint8) EzI2C_UART_CTS_POLARITY,
        (uint8) EzI2C_UART_RTS_POLARITY,
        (uint8) EzI2C_UART_RTS_FIFO_LEVEL
    };


    /*******************************************************************************
    * Function Name: EzI2C_UartInit
    ****************************************************************************//**
    *
    *  Configures the EzI2C for UART operation.
    *
    *  This function is intended specifically to be used when the EzI2C
    *  configuration is set to “Unconfigured EzI2C” in the customizer.
    *  After initializing the EzI2C in UART mode using this function,
    *  the component can be enabled using the EzI2C_Start() or
    * EzI2C_Enable() function.
    *  This function uses a pointer to a structure that provides the configuration
    *  settings. This structure contains the same information that would otherwise
    *  be provided by the customizer settings.
    *
    *  \param config: pointer to a structure that contains the following list of
    *   fields. These fields match the selections available in the customizer.
    *   Refer to the customizer for further description of the settings.
    *
    *******************************************************************************/
    void EzI2C_UartInit(const EzI2C_UART_INIT_STRUCT *config)
    {
        uint32 pinsConfig;

        if (NULL == config)
        {
            CYASSERT(0u != 0u); /* Halt execution due to bad function parameter */
        }
        else
        {
            /* Get direction to configure UART pins: TX, RX or TX+RX */
            pinsConfig  = config->direction;

        #if !(EzI2C_CY_SCBIP_V0 || EzI2C_CY_SCBIP_V1)
            /* Add RTS and CTS pins to configure */
            pinsConfig |= (0u != config->rtsRxFifoLevel) ? (EzI2C_UART_RTS_PIN_ENABLE) : (0u);
            pinsConfig |= (0u != config->enableCts)      ? (EzI2C_UART_CTS_PIN_ENABLE) : (0u);
        #endif /* !(EzI2C_CY_SCBIP_V0 || EzI2C_CY_SCBIP_V1) */

            /* Configure pins */
            EzI2C_SetPins(EzI2C_SCB_MODE_UART, config->mode, pinsConfig);

            /* Store internal configuration */
            EzI2C_scbMode       = (uint8) EzI2C_SCB_MODE_UART;
            EzI2C_scbEnableWake = (uint8) config->enableWake;
            EzI2C_scbEnableIntr = (uint8) config->enableInterrupt;

            /* Set RX direction internal variables */
            EzI2C_rxBuffer      =         config->rxBuffer;
            EzI2C_rxDataBits    = (uint8) config->dataBits;
            EzI2C_rxBufferSize  = (uint8) config->rxBufferSize;

            /* Set TX direction internal variables */
            EzI2C_txBuffer      =         config->txBuffer;
            EzI2C_txDataBits    = (uint8) config->dataBits;
            EzI2C_txBufferSize  = (uint8) config->txBufferSize;

            /* Configure UART interface */
            if(EzI2C_UART_MODE_IRDA == config->mode)
            {
                /* OVS settings: IrDA */
                EzI2C_CTRL_REG  = ((0u != config->enableIrdaLowPower) ?
                                                (EzI2C_UART_GET_CTRL_OVS_IRDA_LP(config->oversample)) :
                                                (EzI2C_CTRL_OVS_IRDA_OVS16));
            }
            else
            {
                /* OVS settings: UART and SmartCard */
                EzI2C_CTRL_REG  = EzI2C_GET_CTRL_OVS(config->oversample);
            }

            EzI2C_CTRL_REG     |= EzI2C_GET_CTRL_BYTE_MODE  (config->enableByteMode)      |
                                             EzI2C_GET_CTRL_ADDR_ACCEPT(config->multiprocAcceptAddr) |
                                             EzI2C_CTRL_UART;

            /* Configure sub-mode: UART, SmartCard or IrDA */
            EzI2C_UART_CTRL_REG = EzI2C_GET_UART_CTRL_MODE(config->mode);

            /* Configure RX direction */
            EzI2C_UART_RX_CTRL_REG = EzI2C_GET_UART_RX_CTRL_MODE(config->stopBits)              |
                                        EzI2C_GET_UART_RX_CTRL_POLARITY(config->enableInvertedRx)          |
                                        EzI2C_GET_UART_RX_CTRL_MP_MODE(config->enableMultiproc)            |
                                        EzI2C_GET_UART_RX_CTRL_DROP_ON_PARITY_ERR(config->dropOnParityErr) |
                                        EzI2C_GET_UART_RX_CTRL_DROP_ON_FRAME_ERR(config->dropOnFrameErr);

            if(EzI2C_UART_PARITY_NONE != config->parity)
            {
               EzI2C_UART_RX_CTRL_REG |= EzI2C_GET_UART_RX_CTRL_PARITY(config->parity) |
                                                    EzI2C_UART_RX_CTRL_PARITY_ENABLED;
            }

            EzI2C_RX_CTRL_REG      = EzI2C_GET_RX_CTRL_DATA_WIDTH(config->dataBits)       |
                                                EzI2C_GET_RX_CTRL_MEDIAN(config->enableMedianFilter) |
                                                EzI2C_GET_UART_RX_CTRL_ENABLED(config->direction);

            EzI2C_RX_FIFO_CTRL_REG = EzI2C_GET_RX_FIFO_CTRL_TRIGGER_LEVEL(config->rxTriggerLevel);

            /* Configure MP address */
            EzI2C_RX_MATCH_REG     = EzI2C_GET_RX_MATCH_ADDR(config->multiprocAddr) |
                                                EzI2C_GET_RX_MATCH_MASK(config->multiprocAddrMask);

            /* Configure RX direction */
            EzI2C_UART_TX_CTRL_REG = EzI2C_GET_UART_TX_CTRL_MODE(config->stopBits) |
                                                EzI2C_GET_UART_TX_CTRL_RETRY_NACK(config->enableRetryNack);

            if(EzI2C_UART_PARITY_NONE != config->parity)
            {
               EzI2C_UART_TX_CTRL_REG |= EzI2C_GET_UART_TX_CTRL_PARITY(config->parity) |
                                                    EzI2C_UART_TX_CTRL_PARITY_ENABLED;
            }

            EzI2C_TX_CTRL_REG      = EzI2C_GET_TX_CTRL_DATA_WIDTH(config->dataBits)    |
                                                EzI2C_GET_UART_TX_CTRL_ENABLED(config->direction);

            EzI2C_TX_FIFO_CTRL_REG = EzI2C_GET_TX_FIFO_CTRL_TRIGGER_LEVEL(config->txTriggerLevel);

        #if !(EzI2C_CY_SCBIP_V0 || EzI2C_CY_SCBIP_V1)
            EzI2C_UART_FLOW_CTRL_REG = EzI2C_GET_UART_FLOW_CTRL_CTS_ENABLE(config->enableCts) | \
                                            EzI2C_GET_UART_FLOW_CTRL_CTS_POLARITY (config->ctsPolarity)  | \
                                            EzI2C_GET_UART_FLOW_CTRL_RTS_POLARITY (config->rtsPolarity)  | \
                                            EzI2C_GET_UART_FLOW_CTRL_TRIGGER_LEVEL(config->rtsRxFifoLevel);
        #endif /* !(EzI2C_CY_SCBIP_V0 || EzI2C_CY_SCBIP_V1) */

            /* Configure interrupt with UART handler but do not enable it */
            CyIntDisable    (EzI2C_ISR_NUMBER);
            CyIntSetPriority(EzI2C_ISR_NUMBER, EzI2C_ISR_PRIORITY);
            (void) CyIntSetVector(EzI2C_ISR_NUMBER, &EzI2C_SPI_UART_ISR);

            /* Configure WAKE interrupt */
        #if(EzI2C_UART_RX_WAKEUP_IRQ)
            CyIntDisable    (EzI2C_RX_WAKE_ISR_NUMBER);
            CyIntSetPriority(EzI2C_RX_WAKE_ISR_NUMBER, EzI2C_RX_WAKE_ISR_PRIORITY);
            (void) CyIntSetVector(EzI2C_RX_WAKE_ISR_NUMBER, &EzI2C_UART_WAKEUP_ISR);
        #endif /* (EzI2C_UART_RX_WAKEUP_IRQ) */

            /* Configure interrupt sources */
            EzI2C_INTR_I2C_EC_MASK_REG = EzI2C_NO_INTR_SOURCES;
            EzI2C_INTR_SPI_EC_MASK_REG = EzI2C_NO_INTR_SOURCES;
            EzI2C_INTR_SLAVE_MASK_REG  = EzI2C_NO_INTR_SOURCES;
            EzI2C_INTR_MASTER_MASK_REG = EzI2C_NO_INTR_SOURCES;
            EzI2C_INTR_RX_MASK_REG     = config->rxInterruptMask;
            EzI2C_INTR_TX_MASK_REG     = config->txInterruptMask;
        
            /* Configure TX interrupt sources to restore. */
            EzI2C_IntrTxMask = LO16(EzI2C_INTR_TX_MASK_REG);

            /* Clear RX buffer indexes */
            EzI2C_rxBufferHead     = 0u;
            EzI2C_rxBufferTail     = 0u;
            EzI2C_rxBufferOverflow = 0u;

            /* Clear TX buffer indexes */
            EzI2C_txBufferHead = 0u;
            EzI2C_txBufferTail = 0u;
        }
    }

#else

    /*******************************************************************************
    * Function Name: EzI2C_UartInit
    ****************************************************************************//**
    *
    *  Configures the SCB for the UART operation.
    *
    *******************************************************************************/
    void EzI2C_UartInit(void)
    {
        /* Configure UART interface */
        EzI2C_CTRL_REG = EzI2C_UART_DEFAULT_CTRL;

        /* Configure sub-mode: UART, SmartCard or IrDA */
        EzI2C_UART_CTRL_REG = EzI2C_UART_DEFAULT_UART_CTRL;

        /* Configure RX direction */
        EzI2C_UART_RX_CTRL_REG = EzI2C_UART_DEFAULT_UART_RX_CTRL;
        EzI2C_RX_CTRL_REG      = EzI2C_UART_DEFAULT_RX_CTRL;
        EzI2C_RX_FIFO_CTRL_REG = EzI2C_UART_DEFAULT_RX_FIFO_CTRL;
        EzI2C_RX_MATCH_REG     = EzI2C_UART_DEFAULT_RX_MATCH_REG;

        /* Configure TX direction */
        EzI2C_UART_TX_CTRL_REG = EzI2C_UART_DEFAULT_UART_TX_CTRL;
        EzI2C_TX_CTRL_REG      = EzI2C_UART_DEFAULT_TX_CTRL;
        EzI2C_TX_FIFO_CTRL_REG = EzI2C_UART_DEFAULT_TX_FIFO_CTRL;

    #if !(EzI2C_CY_SCBIP_V0 || EzI2C_CY_SCBIP_V1)
        EzI2C_UART_FLOW_CTRL_REG = EzI2C_UART_DEFAULT_FLOW_CTRL;
    #endif /* !(EzI2C_CY_SCBIP_V0 || EzI2C_CY_SCBIP_V1) */

        /* Configure interrupt with UART handler but do not enable it */
    #if(EzI2C_SCB_IRQ_INTERNAL)
        CyIntDisable    (EzI2C_ISR_NUMBER);
        CyIntSetPriority(EzI2C_ISR_NUMBER, EzI2C_ISR_PRIORITY);
        (void) CyIntSetVector(EzI2C_ISR_NUMBER, &EzI2C_SPI_UART_ISR);
    #endif /* (EzI2C_SCB_IRQ_INTERNAL) */

        /* Configure WAKE interrupt */
    #if(EzI2C_UART_RX_WAKEUP_IRQ)
        CyIntDisable    (EzI2C_RX_WAKE_ISR_NUMBER);
        CyIntSetPriority(EzI2C_RX_WAKE_ISR_NUMBER, EzI2C_RX_WAKE_ISR_PRIORITY);
        (void) CyIntSetVector(EzI2C_RX_WAKE_ISR_NUMBER, &EzI2C_UART_WAKEUP_ISR);
    #endif /* (EzI2C_UART_RX_WAKEUP_IRQ) */

        /* Configure interrupt sources */
        EzI2C_INTR_I2C_EC_MASK_REG = EzI2C_UART_DEFAULT_INTR_I2C_EC_MASK;
        EzI2C_INTR_SPI_EC_MASK_REG = EzI2C_UART_DEFAULT_INTR_SPI_EC_MASK;
        EzI2C_INTR_SLAVE_MASK_REG  = EzI2C_UART_DEFAULT_INTR_SLAVE_MASK;
        EzI2C_INTR_MASTER_MASK_REG = EzI2C_UART_DEFAULT_INTR_MASTER_MASK;
        EzI2C_INTR_RX_MASK_REG     = EzI2C_UART_DEFAULT_INTR_RX_MASK;
        EzI2C_INTR_TX_MASK_REG     = EzI2C_UART_DEFAULT_INTR_TX_MASK;
    
        /* Configure TX interrupt sources to restore. */
        EzI2C_IntrTxMask = LO16(EzI2C_INTR_TX_MASK_REG);

    #if(EzI2C_INTERNAL_RX_SW_BUFFER_CONST)
        EzI2C_rxBufferHead     = 0u;
        EzI2C_rxBufferTail     = 0u;
        EzI2C_rxBufferOverflow = 0u;
    #endif /* (EzI2C_INTERNAL_RX_SW_BUFFER_CONST) */

    #if(EzI2C_INTERNAL_TX_SW_BUFFER_CONST)
        EzI2C_txBufferHead = 0u;
        EzI2C_txBufferTail = 0u;
    #endif /* (EzI2C_INTERNAL_TX_SW_BUFFER_CONST) */
    }
#endif /* (EzI2C_SCB_MODE_UNCONFIG_CONST_CFG) */


/*******************************************************************************
* Function Name: EzI2C_UartPostEnable
****************************************************************************//**
*
*  Restores HSIOM settings for the UART output pins (TX and/or RTS) to be
*  controlled by the SCB UART.
*
*******************************************************************************/
void EzI2C_UartPostEnable(void)
{
#if (EzI2C_SCB_MODE_UNCONFIG_CONST_CFG)
    #if (EzI2C_TX_SDA_MISO_PIN)
        if (EzI2C_CHECK_TX_SDA_MISO_PIN_USED)
        {
            /* Set SCB UART to drive the output pin */
            EzI2C_SET_HSIOM_SEL(EzI2C_TX_SDA_MISO_HSIOM_REG, EzI2C_TX_SDA_MISO_HSIOM_MASK,
                                           EzI2C_TX_SDA_MISO_HSIOM_POS, EzI2C_TX_SDA_MISO_HSIOM_SEL_UART);
        }
    #endif /* (EzI2C_TX_SDA_MISO_PIN_PIN) */

    #if !(EzI2C_CY_SCBIP_V0 || EzI2C_CY_SCBIP_V1)
        #if (EzI2C_RTS_SS0_PIN)
            if (EzI2C_CHECK_RTS_SS0_PIN_USED)
            {
                /* Set SCB UART to drive the output pin */
                EzI2C_SET_HSIOM_SEL(EzI2C_RTS_SS0_HSIOM_REG, EzI2C_RTS_SS0_HSIOM_MASK,
                                               EzI2C_RTS_SS0_HSIOM_POS, EzI2C_RTS_SS0_HSIOM_SEL_UART);
            }
        #endif /* (EzI2C_RTS_SS0_PIN) */
    #endif /* !(EzI2C_CY_SCBIP_V0 || EzI2C_CY_SCBIP_V1) */

#else
    #if (EzI2C_UART_TX_PIN)
         /* Set SCB UART to drive the output pin */
        EzI2C_SET_HSIOM_SEL(EzI2C_TX_HSIOM_REG, EzI2C_TX_HSIOM_MASK,
                                       EzI2C_TX_HSIOM_POS, EzI2C_TX_HSIOM_SEL_UART);
    #endif /* (EzI2C_UART_TX_PIN) */

    #if (EzI2C_UART_RTS_PIN)
        /* Set SCB UART to drive the output pin */
        EzI2C_SET_HSIOM_SEL(EzI2C_RTS_HSIOM_REG, EzI2C_RTS_HSIOM_MASK,
                                       EzI2C_RTS_HSIOM_POS, EzI2C_RTS_HSIOM_SEL_UART);
    #endif /* (EzI2C_UART_RTS_PIN) */
#endif /* (EzI2C_SCB_MODE_UNCONFIG_CONST_CFG) */

    /* Restore TX interrupt sources. */
    EzI2C_SetTxInterruptMode(EzI2C_IntrTxMask);
}


/*******************************************************************************
* Function Name: EzI2C_UartStop
****************************************************************************//**
*
*  Changes the HSIOM settings for the UART output pins (TX and/or RTS) to keep
*  them inactive after the block is disabled. The output pins are controlled by
*  the GPIO data register. Also, the function disables the skip start feature
*  to not cause it to trigger after the component is enabled.
*
*******************************************************************************/
void EzI2C_UartStop(void)
{
#if(EzI2C_SCB_MODE_UNCONFIG_CONST_CFG)
    #if (EzI2C_TX_SDA_MISO_PIN)
        if (EzI2C_CHECK_TX_SDA_MISO_PIN_USED)
        {
            /* Set GPIO to drive output pin */
            EzI2C_SET_HSIOM_SEL(EzI2C_TX_SDA_MISO_HSIOM_REG, EzI2C_TX_SDA_MISO_HSIOM_MASK,
                                           EzI2C_TX_SDA_MISO_HSIOM_POS, EzI2C_TX_SDA_MISO_HSIOM_SEL_GPIO);
        }
    #endif /* (EzI2C_TX_SDA_MISO_PIN_PIN) */

    #if !(EzI2C_CY_SCBIP_V0 || EzI2C_CY_SCBIP_V1)
        #if (EzI2C_RTS_SS0_PIN)
            if (EzI2C_CHECK_RTS_SS0_PIN_USED)
            {
                /* Set output pin state after block is disabled */
                EzI2C_uart_rts_spi_ss0_Write(EzI2C_GET_UART_RTS_INACTIVE);

                /* Set GPIO to drive output pin */
                EzI2C_SET_HSIOM_SEL(EzI2C_RTS_SS0_HSIOM_REG, EzI2C_RTS_SS0_HSIOM_MASK,
                                               EzI2C_RTS_SS0_HSIOM_POS, EzI2C_RTS_SS0_HSIOM_SEL_GPIO);
            }
        #endif /* (EzI2C_RTS_SS0_PIN) */
    #endif /* !(EzI2C_CY_SCBIP_V0 || EzI2C_CY_SCBIP_V1) */

#else
    #if (EzI2C_UART_TX_PIN)
        /* Set GPIO to drive output pin */
        EzI2C_SET_HSIOM_SEL(EzI2C_TX_HSIOM_REG, EzI2C_TX_HSIOM_MASK,
                                       EzI2C_TX_HSIOM_POS, EzI2C_TX_HSIOM_SEL_GPIO);
    #endif /* (EzI2C_UART_TX_PIN) */

    #if (EzI2C_UART_RTS_PIN)
        /* Set output pin state after block is disabled */
        EzI2C_rts_Write(EzI2C_GET_UART_RTS_INACTIVE);

        /* Set GPIO to drive output pin */
        EzI2C_SET_HSIOM_SEL(EzI2C_RTS_HSIOM_REG, EzI2C_RTS_HSIOM_MASK,
                                       EzI2C_RTS_HSIOM_POS, EzI2C_RTS_HSIOM_SEL_GPIO);
    #endif /* (EzI2C_UART_RTS_PIN) */

#endif /* (EzI2C_SCB_MODE_UNCONFIG_CONST_CFG) */

#if (EzI2C_UART_WAKE_ENABLE_CONST)
    /* Disable skip start feature used for wakeup */
    EzI2C_UART_RX_CTRL_REG &= (uint32) ~EzI2C_UART_RX_CTRL_SKIP_START;
#endif /* (EzI2C_UART_WAKE_ENABLE_CONST) */

    /* Store TX interrupt sources (exclude level triggered). */
    EzI2C_IntrTxMask = LO16(EzI2C_GetTxInterruptMode() & EzI2C_INTR_UART_TX_RESTORE);
}


/*******************************************************************************
* Function Name: EzI2C_UartSetRxAddress
****************************************************************************//**
*
*  Sets the hardware detectable receiver address for the UART in the
*  Multiprocessor mode.
*
*  \param address: Address for hardware address detection.
*
*******************************************************************************/
void EzI2C_UartSetRxAddress(uint32 address)
{
     uint32 matchReg;

    matchReg = EzI2C_RX_MATCH_REG;

    matchReg &= ((uint32) ~EzI2C_RX_MATCH_ADDR_MASK); /* Clear address bits */
    matchReg |= ((uint32)  (address & EzI2C_RX_MATCH_ADDR_MASK)); /* Set address  */

    EzI2C_RX_MATCH_REG = matchReg;
}


/*******************************************************************************
* Function Name: EzI2C_UartSetRxAddressMask
****************************************************************************//**
*
*  Sets the hardware address mask for the UART in the Multiprocessor mode.
*
*  \param addressMask: Address mask.
*   - Bit value 0 – excludes bit from address comparison.
*   - Bit value 1 – the bit needs to match with the corresponding bit
*     of the address.
*
*******************************************************************************/
void EzI2C_UartSetRxAddressMask(uint32 addressMask)
{
    uint32 matchReg;

    matchReg = EzI2C_RX_MATCH_REG;

    matchReg &= ((uint32) ~EzI2C_RX_MATCH_MASK_MASK); /* Clear address mask bits */
    matchReg |= ((uint32) (addressMask << EzI2C_RX_MATCH_MASK_POS));

    EzI2C_RX_MATCH_REG = matchReg;
}


#if(EzI2C_UART_RX_DIRECTION)
    /*******************************************************************************
    * Function Name: EzI2C_UartGetChar
    ****************************************************************************//**
    *
    *  Retrieves next data element from receive buffer.
    *  This function is designed for ASCII characters and returns a char where
    *  1 to 255 are valid characters and 0 indicates an error occurred or no data
    *  is present.
    *  - RX software buffer is disabled: Returns data element retrieved from RX
    *    FIFO.
    *  - RX software buffer is enabled: Returns data element from the software
    *    receive buffer.
    *
    *  \return
    *   Next data element from the receive buffer. ASCII character values from
    *   1 to 255 are valid. A returned zero signifies an error condition or no
    *   data available.
    *
    *  \sideeffect
    *   The errors bits may not correspond with reading characters due to
    *   RX FIFO and software buffer usage.
    *   RX software buffer is enabled: The internal software buffer overflow
    *   is not treated as an error condition.
    *   Check EzI2C_rxBufferOverflow to capture that error condition.
    *
    *******************************************************************************/
    uint32 EzI2C_UartGetChar(void)
    {
        uint32 rxData = 0u;

        /* Reads data only if there is data to read */
        if (0u != EzI2C_SpiUartGetRxBufferSize())
        {
            rxData = EzI2C_SpiUartReadRxData();
        }

        if (EzI2C_CHECK_INTR_RX(EzI2C_INTR_RX_ERR))
        {
            rxData = 0u; /* Error occurred: returns zero */
            EzI2C_ClearRxInterruptSource(EzI2C_INTR_RX_ERR);
        }

        return (rxData);
    }


    /*******************************************************************************
    * Function Name: EzI2C_UartGetByte
    ****************************************************************************//**
    *
    *  Retrieves the next data element from the receive buffer, returns the
    *  received byte and error condition.
    *   - The RX software buffer is disabled: returns the data element retrieved
    *     from the RX FIFO. Undefined data will be returned if the RX FIFO is
    *     empty.
    *   - The RX software buffer is enabled: returns data element from the
    *     software receive buffer.
    *
    *  \return
    *   Bits 7-0 contain the next data element from the receive buffer and
    *   other bits contain the error condition.
    *   - EzI2C_UART_RX_OVERFLOW - Attempt to write to a full
    *     receiver FIFO.
    *   - EzI2C_UART_RX_UNDERFLOW	Attempt to read from an empty
    *     receiver FIFO.
    *   - EzI2C_UART_RX_FRAME_ERROR - UART framing error detected.
    *   - EzI2C_UART_RX_PARITY_ERROR - UART parity error detected.
    *
    *  \sideeffect
    *   The errors bits may not correspond with reading characters due to
    *   RX FIFO and software buffer usage.
    *   RX software buffer is enabled: The internal software buffer overflow
    *   is not treated as an error condition.
    *   Check EzI2C_rxBufferOverflow to capture that error condition.
    *
    *******************************************************************************/
    uint32 EzI2C_UartGetByte(void)
    {
        uint32 rxData;
        uint32 tmpStatus;

        #if (EzI2C_CHECK_RX_SW_BUFFER)
        {
            EzI2C_DisableInt();
        }
        #endif

        if (0u != EzI2C_SpiUartGetRxBufferSize())
        {
            /* Enables interrupt to receive more bytes: at least one byte is in
            * buffer.
            */
            #if (EzI2C_CHECK_RX_SW_BUFFER)
            {
                EzI2C_EnableInt();
            }
            #endif

            /* Get received byte */
            rxData = EzI2C_SpiUartReadRxData();
        }
        else
        {
            /* Reads a byte directly from RX FIFO: underflow is raised in the
            * case of empty. Otherwise the first received byte will be read.
            */
            rxData = EzI2C_RX_FIFO_RD_REG;


            /* Enables interrupt to receive more bytes. */
            #if (EzI2C_CHECK_RX_SW_BUFFER)
            {

                /* The byte has been read from RX FIFO. Clear RX interrupt to
                * not involve interrupt handler when RX FIFO is empty.
                */
                EzI2C_ClearRxInterruptSource(EzI2C_INTR_RX_NOT_EMPTY);

                EzI2C_EnableInt();
            }
            #endif
        }

        /* Get and clear RX error mask */
        tmpStatus = (EzI2C_GetRxInterruptSource() & EzI2C_INTR_RX_ERR);
        EzI2C_ClearRxInterruptSource(EzI2C_INTR_RX_ERR);

        /* Puts together data and error status:
        * MP mode and accept address: 9th bit is set to notify mark.
        */
        rxData |= ((uint32) (tmpStatus << 8u));

        return (rxData);
    }


    #if !(EzI2C_CY_SCBIP_V0 || EzI2C_CY_SCBIP_V1)
        /*******************************************************************************
        * Function Name: EzI2C_UartSetRtsPolarity
        ****************************************************************************//**
        *
        *  Sets active polarity of RTS output signal.
        *  Only available for PSoC 4100 BLE / PSoC 4200 BLE / PSoC 4100M / PSoC 4200M /
        *  PSoC 4200L / PSoC 4000S / PSoC 4100S / PSoC Analog Coprocessor devices.
        *
        *  \param polarity: Active polarity of RTS output signal.
        *   - EzI2C_UART_RTS_ACTIVE_LOW  - RTS signal is active low.
        *   - EzI2C_UART_RTS_ACTIVE_HIGH - RTS signal is active high.
        *
        *******************************************************************************/
        void EzI2C_UartSetRtsPolarity(uint32 polarity)
        {
            if(0u != polarity)
            {
                EzI2C_UART_FLOW_CTRL_REG |= (uint32)  EzI2C_UART_FLOW_CTRL_RTS_POLARITY;
            }
            else
            {
                EzI2C_UART_FLOW_CTRL_REG &= (uint32) ~EzI2C_UART_FLOW_CTRL_RTS_POLARITY;
            }
        }


        /*******************************************************************************
        * Function Name: EzI2C_UartSetRtsFifoLevel
        ****************************************************************************//**
        *
        *  Sets level in the RX FIFO for RTS signal activation.
        *  While the RX FIFO has fewer entries than the RX FIFO level the RTS signal
        *  remains active, otherwise the RTS signal becomes inactive.
        *  Only available for PSoC 4100 BLE / PSoC 4200 BLE / PSoC 4100M / PSoC 4200M /
        *  PSoC 4200L / PSoC 4000S / PSoC 4100S / PSoC Analog Coprocessor devices.
        *
        *  \param level: Level in the RX FIFO for RTS signal activation.
        *   The range of valid level values is between 0 and RX FIFO depth - 1.
        *   Setting level value to 0 disables RTS signal activation.
        *
        *******************************************************************************/
        void EzI2C_UartSetRtsFifoLevel(uint32 level)
        {
            uint32 uartFlowCtrl;

            uartFlowCtrl = EzI2C_UART_FLOW_CTRL_REG;

            uartFlowCtrl &= ((uint32) ~EzI2C_UART_FLOW_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
            uartFlowCtrl |= ((uint32) (EzI2C_UART_FLOW_CTRL_TRIGGER_LEVEL_MASK & level));

            EzI2C_UART_FLOW_CTRL_REG = uartFlowCtrl;
        }
    #endif /* !(EzI2C_CY_SCBIP_V0 || EzI2C_CY_SCBIP_V1) */

#endif /* (EzI2C_UART_RX_DIRECTION) */


#if(EzI2C_UART_TX_DIRECTION)
    /*******************************************************************************
    * Function Name: EzI2C_UartPutString
    ****************************************************************************//**
    *
    *  Places a NULL terminated string in the transmit buffer to be sent at the
    *  next available bus time.
    *  This function is blocking and waits until there is a space available to put
    *  requested data in transmit buffer.
    *
    *  \param string: pointer to the null terminated string array to be placed in the
    *   transmit buffer.
    *
    *******************************************************************************/
    void EzI2C_UartPutString(const char8 string[])
    {
        uint32 bufIndex;

        bufIndex = 0u;

        /* Blocks the control flow until all data has been sent */
        while(string[bufIndex] != ((char8) 0))
        {
            EzI2C_UartPutChar((uint32) string[bufIndex]);
            bufIndex++;
        }
    }


    /*******************************************************************************
    * Function Name: EzI2C_UartPutCRLF
    ****************************************************************************//**
    *
    *  Places byte of data followed by a carriage return (0x0D) and line feed
    *  (0x0A) in the transmit buffer.
    *  This function is blocking and waits until there is a space available to put
    *  all requested data in transmit buffer.
    *
    *  \param txDataByte: the data to be transmitted.
    *
    *******************************************************************************/
    void EzI2C_UartPutCRLF(uint32 txDataByte)
    {
        EzI2C_UartPutChar(txDataByte);  /* Blocks control flow until all data has been sent */
        EzI2C_UartPutChar(0x0Du);       /* Blocks control flow until all data has been sent */
        EzI2C_UartPutChar(0x0Au);       /* Blocks control flow until all data has been sent */
    }


    #if !(EzI2C_CY_SCBIP_V0 || EzI2C_CY_SCBIP_V1)
        /*******************************************************************************
        * Function Name: EzI2CSCB_UartEnableCts
        ****************************************************************************//**
        *
        *  Enables usage of CTS input signal by the UART transmitter.
        *  Only available for PSoC 4100 BLE / PSoC 4200 BLE / PSoC 4100M / PSoC 4200M /
        *  PSoC 4200L / PSoC 4000S / PSoC 4100S / PSoC Analog Coprocessor devices.
        *
        *******************************************************************************/
        void EzI2C_UartEnableCts(void)
        {
            EzI2C_UART_FLOW_CTRL_REG |= (uint32)  EzI2C_UART_FLOW_CTRL_CTS_ENABLE;
        }


        /*******************************************************************************
        * Function Name: EzI2C_UartDisableCts
        ****************************************************************************//**
        *
        *  Disables usage of CTS input signal by the UART transmitter.
        *  Only available for PSoC 4100 BLE / PSoC 4200 BLE / PSoC 4100M / PSoC 4200M /
        *  PSoC 4200L / PSoC 4000S / PSoC 4100S / PSoC Analog Coprocessor devices.
        *
        *******************************************************************************/
        void EzI2C_UartDisableCts(void)
        {
            EzI2C_UART_FLOW_CTRL_REG &= (uint32) ~EzI2C_UART_FLOW_CTRL_CTS_ENABLE;
        }


        /*******************************************************************************
        * Function Name: EzI2C_UartSetCtsPolarity
        ****************************************************************************//**
        *
        *  Sets active polarity of CTS input signal.
        *  Only available for PSoC 4100 BLE / PSoC 4200 BLE / PSoC 4100M / PSoC 4200M /
        *  PSoC 4200L / PSoC 4000S / PSoC 4100S / PSoC Analog Coprocessor devices.
        *
        *  \param polarity: Active polarity of CTS output signal.
        *   - EzI2C_UART_CTS_ACTIVE_LOW  - CTS signal is active low.
        *   - EzI2C_UART_CTS_ACTIVE_HIGH - CTS signal is active high.
        *
        *******************************************************************************/
        void EzI2C_UartSetCtsPolarity(uint32 polarity)
        {
            if (0u != polarity)
            {
                EzI2C_UART_FLOW_CTRL_REG |= (uint32)  EzI2C_UART_FLOW_CTRL_CTS_POLARITY;
            }
            else
            {
                EzI2C_UART_FLOW_CTRL_REG &= (uint32) ~EzI2C_UART_FLOW_CTRL_CTS_POLARITY;
            }
        }
    #endif /* !(EzI2C_CY_SCBIP_V0 || EzI2C_CY_SCBIP_V1) */

#endif /* (EzI2C_UART_TX_DIRECTION) */


#if (EzI2C_UART_WAKE_ENABLE_CONST)
    /*******************************************************************************
    * Function Name: EzI2C_UartSaveConfig
    ****************************************************************************//**
    *
    *  Clears and enables an interrupt on a falling edge of the Rx input. The GPIO
    *  interrupt does not track in the active mode, therefore requires to be 
    *  cleared by this API.
    *
    *******************************************************************************/
    void EzI2C_UartSaveConfig(void)
    {
    #if (EzI2C_UART_RX_WAKEUP_IRQ)
        /* Set SKIP_START if requested (set by default). */
        if (0u != EzI2C_skipStart)
        {
            EzI2C_UART_RX_CTRL_REG |= (uint32)  EzI2C_UART_RX_CTRL_SKIP_START;
        }
        else
        {
            EzI2C_UART_RX_CTRL_REG &= (uint32) ~EzI2C_UART_RX_CTRL_SKIP_START;
        }
        
        /* Clear RX GPIO interrupt status and pending interrupt in NVIC because
        * falling edge on RX line occurs while UART communication in active mode.
        * Enable interrupt: next interrupt trigger should wakeup device.
        */
        EzI2C_CLEAR_UART_RX_WAKE_INTR;
        EzI2C_RxWakeClearPendingInt();
        EzI2C_RxWakeEnableInt();
    #endif /* (EzI2C_UART_RX_WAKEUP_IRQ) */
    }


    /*******************************************************************************
    * Function Name: EzI2C_UartRestoreConfig
    ****************************************************************************//**
    *
    *  Disables the RX GPIO interrupt. Until this function is called the interrupt
    *  remains active and triggers on every falling edge of the UART RX line.
    *
    *******************************************************************************/
    void EzI2C_UartRestoreConfig(void)
    {
    #if (EzI2C_UART_RX_WAKEUP_IRQ)
        /* Disable interrupt: no more triggers in active mode */
        EzI2C_RxWakeDisableInt();
    #endif /* (EzI2C_UART_RX_WAKEUP_IRQ) */
    }


    #if (EzI2C_UART_RX_WAKEUP_IRQ)
        /*******************************************************************************
        * Function Name: EzI2C_UART_WAKEUP_ISR
        ****************************************************************************//**
        *
        *  Handles the Interrupt Service Routine for the SCB UART mode GPIO wakeup
        *  event. This event is configured to trigger on a falling edge of the RX line.
        *
        *******************************************************************************/
        CY_ISR(EzI2C_UART_WAKEUP_ISR)
        {
        #ifdef EzI2C_UART_WAKEUP_ISR_ENTRY_CALLBACK
            EzI2C_UART_WAKEUP_ISR_EntryCallback();
        #endif /* EzI2C_UART_WAKEUP_ISR_ENTRY_CALLBACK */

            EzI2C_CLEAR_UART_RX_WAKE_INTR;

        #ifdef EzI2C_UART_WAKEUP_ISR_EXIT_CALLBACK
            EzI2C_UART_WAKEUP_ISR_ExitCallback();
        #endif /* EzI2C_UART_WAKEUP_ISR_EXIT_CALLBACK */
        }
    #endif /* (EzI2C_UART_RX_WAKEUP_IRQ) */
#endif /* (EzI2C_UART_RX_WAKEUP_IRQ) */


/* [] END OF FILE */
