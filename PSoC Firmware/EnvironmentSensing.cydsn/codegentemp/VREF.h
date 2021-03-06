/*******************************************************************************
* File Name: VREF.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_VREF_H) /* Pins VREF_H */
#define CY_PINS_VREF_H

#include "cytypes.h"
#include "cyfitter.h"
#include "VREF_aliases.h"


/***************************************
*     Data Struct Definitions
***************************************/

/**
* \addtogroup group_structures
* @{
*/
    
/* Structure for sleep mode support */
typedef struct
{
    uint32 pcState; /**< State of the port control register */
    uint32 sioState; /**< State of the SIO configuration */
    uint32 usbState; /**< State of the USBIO regulator */
} VREF_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   VREF_Read(void);
void    VREF_Write(uint8 value);
uint8   VREF_ReadDataReg(void);
#if defined(VREF__PC) || (CY_PSOC4_4200L) 
    void    VREF_SetDriveMode(uint8 mode);
#endif
void    VREF_SetInterruptMode(uint16 position, uint16 mode);
uint8   VREF_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void VREF_Sleep(void); 
void VREF_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(VREF__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define VREF_DRIVE_MODE_BITS        (3)
    #define VREF_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - VREF_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the VREF_SetDriveMode() function.
         *  @{
         */
        #define VREF_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define VREF_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define VREF_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define VREF_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define VREF_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define VREF_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define VREF_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define VREF_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define VREF_MASK               VREF__MASK
#define VREF_SHIFT              VREF__SHIFT
#define VREF_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in VREF_SetInterruptMode() function.
     *  @{
     */
        #define VREF_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define VREF_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define VREF_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define VREF_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(VREF__SIO)
    #define VREF_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(VREF__PC) && (CY_PSOC4_4200L)
    #define VREF_USBIO_ENABLE               ((uint32)0x80000000u)
    #define VREF_USBIO_DISABLE              ((uint32)(~VREF_USBIO_ENABLE))
    #define VREF_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define VREF_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define VREF_USBIO_ENTER_SLEEP          ((uint32)((1u << VREF_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << VREF_USBIO_SUSPEND_DEL_SHIFT)))
    #define VREF_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << VREF_USBIO_SUSPEND_SHIFT)))
    #define VREF_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << VREF_USBIO_SUSPEND_DEL_SHIFT)))
    #define VREF_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(VREF__PC)
    /* Port Configuration */
    #define VREF_PC                 (* (reg32 *) VREF__PC)
#endif
/* Pin State */
#define VREF_PS                     (* (reg32 *) VREF__PS)
/* Data Register */
#define VREF_DR                     (* (reg32 *) VREF__DR)
/* Input Buffer Disable Override */
#define VREF_INP_DIS                (* (reg32 *) VREF__PC2)

/* Interrupt configuration Registers */
#define VREF_INTCFG                 (* (reg32 *) VREF__INTCFG)
#define VREF_INTSTAT                (* (reg32 *) VREF__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define VREF_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(VREF__SIO)
    #define VREF_SIO_REG            (* (reg32 *) VREF__SIO)
#endif /* (VREF__SIO_CFG) */

/* USBIO registers */
#if !defined(VREF__PC) && (CY_PSOC4_4200L)
    #define VREF_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define VREF_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define VREF_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define VREF_DRIVE_MODE_SHIFT       (0x00u)
#define VREF_DRIVE_MODE_MASK        (0x07u << VREF_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins VREF_H */


/* [] END OF FILE */
