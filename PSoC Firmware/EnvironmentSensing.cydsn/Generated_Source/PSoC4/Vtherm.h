/*******************************************************************************
* File Name: Vtherm.h  
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

#if !defined(CY_PINS_Vtherm_H) /* Pins Vtherm_H */
#define CY_PINS_Vtherm_H

#include "cytypes.h"
#include "cyfitter.h"
#include "Vtherm_aliases.h"


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
} Vtherm_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   Vtherm_Read(void);
void    Vtherm_Write(uint8 value);
uint8   Vtherm_ReadDataReg(void);
#if defined(Vtherm__PC) || (CY_PSOC4_4200L) 
    void    Vtherm_SetDriveMode(uint8 mode);
#endif
void    Vtherm_SetInterruptMode(uint16 position, uint16 mode);
uint8   Vtherm_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void Vtherm_Sleep(void); 
void Vtherm_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(Vtherm__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define Vtherm_DRIVE_MODE_BITS        (3)
    #define Vtherm_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - Vtherm_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the Vtherm_SetDriveMode() function.
         *  @{
         */
        #define Vtherm_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define Vtherm_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define Vtherm_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define Vtherm_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define Vtherm_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define Vtherm_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define Vtherm_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define Vtherm_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define Vtherm_MASK               Vtherm__MASK
#define Vtherm_SHIFT              Vtherm__SHIFT
#define Vtherm_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in Vtherm_SetInterruptMode() function.
     *  @{
     */
        #define Vtherm_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define Vtherm_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define Vtherm_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define Vtherm_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(Vtherm__SIO)
    #define Vtherm_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(Vtherm__PC) && (CY_PSOC4_4200L)
    #define Vtherm_USBIO_ENABLE               ((uint32)0x80000000u)
    #define Vtherm_USBIO_DISABLE              ((uint32)(~Vtherm_USBIO_ENABLE))
    #define Vtherm_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define Vtherm_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define Vtherm_USBIO_ENTER_SLEEP          ((uint32)((1u << Vtherm_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << Vtherm_USBIO_SUSPEND_DEL_SHIFT)))
    #define Vtherm_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << Vtherm_USBIO_SUSPEND_SHIFT)))
    #define Vtherm_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << Vtherm_USBIO_SUSPEND_DEL_SHIFT)))
    #define Vtherm_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(Vtherm__PC)
    /* Port Configuration */
    #define Vtherm_PC                 (* (reg32 *) Vtherm__PC)
#endif
/* Pin State */
#define Vtherm_PS                     (* (reg32 *) Vtherm__PS)
/* Data Register */
#define Vtherm_DR                     (* (reg32 *) Vtherm__DR)
/* Input Buffer Disable Override */
#define Vtherm_INP_DIS                (* (reg32 *) Vtherm__PC2)

/* Interrupt configuration Registers */
#define Vtherm_INTCFG                 (* (reg32 *) Vtherm__INTCFG)
#define Vtherm_INTSTAT                (* (reg32 *) Vtherm__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define Vtherm_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(Vtherm__SIO)
    #define Vtherm_SIO_REG            (* (reg32 *) Vtherm__SIO)
#endif /* (Vtherm__SIO_CFG) */

/* USBIO registers */
#if !defined(Vtherm__PC) && (CY_PSOC4_4200L)
    #define Vtherm_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define Vtherm_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define Vtherm_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define Vtherm_DRIVE_MODE_SHIFT       (0x00u)
#define Vtherm_DRIVE_MODE_MASK        (0x07u << Vtherm_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins Vtherm_H */


/* [] END OF FILE */
