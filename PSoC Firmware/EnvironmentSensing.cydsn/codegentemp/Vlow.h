/*******************************************************************************
* File Name: Vlow.h  
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

#if !defined(CY_PINS_Vlow_H) /* Pins Vlow_H */
#define CY_PINS_Vlow_H

#include "cytypes.h"
#include "cyfitter.h"
#include "Vlow_aliases.h"


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
} Vlow_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   Vlow_Read(void);
void    Vlow_Write(uint8 value);
uint8   Vlow_ReadDataReg(void);
#if defined(Vlow__PC) || (CY_PSOC4_4200L) 
    void    Vlow_SetDriveMode(uint8 mode);
#endif
void    Vlow_SetInterruptMode(uint16 position, uint16 mode);
uint8   Vlow_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void Vlow_Sleep(void); 
void Vlow_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(Vlow__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define Vlow_DRIVE_MODE_BITS        (3)
    #define Vlow_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - Vlow_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the Vlow_SetDriveMode() function.
         *  @{
         */
        #define Vlow_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define Vlow_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define Vlow_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define Vlow_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define Vlow_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define Vlow_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define Vlow_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define Vlow_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define Vlow_MASK               Vlow__MASK
#define Vlow_SHIFT              Vlow__SHIFT
#define Vlow_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in Vlow_SetInterruptMode() function.
     *  @{
     */
        #define Vlow_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define Vlow_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define Vlow_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define Vlow_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(Vlow__SIO)
    #define Vlow_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(Vlow__PC) && (CY_PSOC4_4200L)
    #define Vlow_USBIO_ENABLE               ((uint32)0x80000000u)
    #define Vlow_USBIO_DISABLE              ((uint32)(~Vlow_USBIO_ENABLE))
    #define Vlow_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define Vlow_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define Vlow_USBIO_ENTER_SLEEP          ((uint32)((1u << Vlow_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << Vlow_USBIO_SUSPEND_DEL_SHIFT)))
    #define Vlow_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << Vlow_USBIO_SUSPEND_SHIFT)))
    #define Vlow_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << Vlow_USBIO_SUSPEND_DEL_SHIFT)))
    #define Vlow_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(Vlow__PC)
    /* Port Configuration */
    #define Vlow_PC                 (* (reg32 *) Vlow__PC)
#endif
/* Pin State */
#define Vlow_PS                     (* (reg32 *) Vlow__PS)
/* Data Register */
#define Vlow_DR                     (* (reg32 *) Vlow__DR)
/* Input Buffer Disable Override */
#define Vlow_INP_DIS                (* (reg32 *) Vlow__PC2)

/* Interrupt configuration Registers */
#define Vlow_INTCFG                 (* (reg32 *) Vlow__INTCFG)
#define Vlow_INTSTAT                (* (reg32 *) Vlow__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define Vlow_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(Vlow__SIO)
    #define Vlow_SIO_REG            (* (reg32 *) Vlow__SIO)
#endif /* (Vlow__SIO_CFG) */

/* USBIO registers */
#if !defined(Vlow__PC) && (CY_PSOC4_4200L)
    #define Vlow_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define Vlow_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define Vlow_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define Vlow_DRIVE_MODE_SHIFT       (0x00u)
#define Vlow_DRIVE_MODE_MASK        (0x07u << Vlow_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins Vlow_H */


/* [] END OF FILE */
