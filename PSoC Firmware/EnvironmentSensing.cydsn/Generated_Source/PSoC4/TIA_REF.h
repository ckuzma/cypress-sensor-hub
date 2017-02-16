/*******************************************************************************
* File Name: TIA_REF.h  
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

#if !defined(CY_PINS_TIA_REF_H) /* Pins TIA_REF_H */
#define CY_PINS_TIA_REF_H

#include "cytypes.h"
#include "cyfitter.h"
#include "TIA_REF_aliases.h"


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
} TIA_REF_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   TIA_REF_Read(void);
void    TIA_REF_Write(uint8 value);
uint8   TIA_REF_ReadDataReg(void);
#if defined(TIA_REF__PC) || (CY_PSOC4_4200L) 
    void    TIA_REF_SetDriveMode(uint8 mode);
#endif
void    TIA_REF_SetInterruptMode(uint16 position, uint16 mode);
uint8   TIA_REF_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void TIA_REF_Sleep(void); 
void TIA_REF_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(TIA_REF__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define TIA_REF_DRIVE_MODE_BITS        (3)
    #define TIA_REF_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - TIA_REF_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the TIA_REF_SetDriveMode() function.
         *  @{
         */
        #define TIA_REF_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define TIA_REF_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define TIA_REF_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define TIA_REF_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define TIA_REF_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define TIA_REF_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define TIA_REF_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define TIA_REF_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define TIA_REF_MASK               TIA_REF__MASK
#define TIA_REF_SHIFT              TIA_REF__SHIFT
#define TIA_REF_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in TIA_REF_SetInterruptMode() function.
     *  @{
     */
        #define TIA_REF_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define TIA_REF_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define TIA_REF_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define TIA_REF_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(TIA_REF__SIO)
    #define TIA_REF_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(TIA_REF__PC) && (CY_PSOC4_4200L)
    #define TIA_REF_USBIO_ENABLE               ((uint32)0x80000000u)
    #define TIA_REF_USBIO_DISABLE              ((uint32)(~TIA_REF_USBIO_ENABLE))
    #define TIA_REF_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define TIA_REF_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define TIA_REF_USBIO_ENTER_SLEEP          ((uint32)((1u << TIA_REF_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << TIA_REF_USBIO_SUSPEND_DEL_SHIFT)))
    #define TIA_REF_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << TIA_REF_USBIO_SUSPEND_SHIFT)))
    #define TIA_REF_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << TIA_REF_USBIO_SUSPEND_DEL_SHIFT)))
    #define TIA_REF_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(TIA_REF__PC)
    /* Port Configuration */
    #define TIA_REF_PC                 (* (reg32 *) TIA_REF__PC)
#endif
/* Pin State */
#define TIA_REF_PS                     (* (reg32 *) TIA_REF__PS)
/* Data Register */
#define TIA_REF_DR                     (* (reg32 *) TIA_REF__DR)
/* Input Buffer Disable Override */
#define TIA_REF_INP_DIS                (* (reg32 *) TIA_REF__PC2)

/* Interrupt configuration Registers */
#define TIA_REF_INTCFG                 (* (reg32 *) TIA_REF__INTCFG)
#define TIA_REF_INTSTAT                (* (reg32 *) TIA_REF__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define TIA_REF_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(TIA_REF__SIO)
    #define TIA_REF_SIO_REG            (* (reg32 *) TIA_REF__SIO)
#endif /* (TIA_REF__SIO_CFG) */

/* USBIO registers */
#if !defined(TIA_REF__PC) && (CY_PSOC4_4200L)
    #define TIA_REF_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define TIA_REF_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define TIA_REF_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define TIA_REF_DRIVE_MODE_SHIFT       (0x00u)
#define TIA_REF_DRIVE_MODE_MASK        (0x07u << TIA_REF_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins TIA_REF_H */


/* [] END OF FILE */
