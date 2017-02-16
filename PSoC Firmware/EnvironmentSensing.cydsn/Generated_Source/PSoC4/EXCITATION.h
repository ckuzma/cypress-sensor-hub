/*******************************************************************************
* File Name: EXCITATION.h  
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

#if !defined(CY_PINS_EXCITATION_H) /* Pins EXCITATION_H */
#define CY_PINS_EXCITATION_H

#include "cytypes.h"
#include "cyfitter.h"
#include "EXCITATION_aliases.h"


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
} EXCITATION_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   EXCITATION_Read(void);
void    EXCITATION_Write(uint8 value);
uint8   EXCITATION_ReadDataReg(void);
#if defined(EXCITATION__PC) || (CY_PSOC4_4200L) 
    void    EXCITATION_SetDriveMode(uint8 mode);
#endif
void    EXCITATION_SetInterruptMode(uint16 position, uint16 mode);
uint8   EXCITATION_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void EXCITATION_Sleep(void); 
void EXCITATION_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(EXCITATION__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define EXCITATION_DRIVE_MODE_BITS        (3)
    #define EXCITATION_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - EXCITATION_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the EXCITATION_SetDriveMode() function.
         *  @{
         */
        #define EXCITATION_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define EXCITATION_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define EXCITATION_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define EXCITATION_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define EXCITATION_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define EXCITATION_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define EXCITATION_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define EXCITATION_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define EXCITATION_MASK               EXCITATION__MASK
#define EXCITATION_SHIFT              EXCITATION__SHIFT
#define EXCITATION_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in EXCITATION_SetInterruptMode() function.
     *  @{
     */
        #define EXCITATION_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define EXCITATION_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define EXCITATION_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define EXCITATION_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(EXCITATION__SIO)
    #define EXCITATION_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(EXCITATION__PC) && (CY_PSOC4_4200L)
    #define EXCITATION_USBIO_ENABLE               ((uint32)0x80000000u)
    #define EXCITATION_USBIO_DISABLE              ((uint32)(~EXCITATION_USBIO_ENABLE))
    #define EXCITATION_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define EXCITATION_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define EXCITATION_USBIO_ENTER_SLEEP          ((uint32)((1u << EXCITATION_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << EXCITATION_USBIO_SUSPEND_DEL_SHIFT)))
    #define EXCITATION_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << EXCITATION_USBIO_SUSPEND_SHIFT)))
    #define EXCITATION_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << EXCITATION_USBIO_SUSPEND_DEL_SHIFT)))
    #define EXCITATION_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(EXCITATION__PC)
    /* Port Configuration */
    #define EXCITATION_PC                 (* (reg32 *) EXCITATION__PC)
#endif
/* Pin State */
#define EXCITATION_PS                     (* (reg32 *) EXCITATION__PS)
/* Data Register */
#define EXCITATION_DR                     (* (reg32 *) EXCITATION__DR)
/* Input Buffer Disable Override */
#define EXCITATION_INP_DIS                (* (reg32 *) EXCITATION__PC2)

/* Interrupt configuration Registers */
#define EXCITATION_INTCFG                 (* (reg32 *) EXCITATION__INTCFG)
#define EXCITATION_INTSTAT                (* (reg32 *) EXCITATION__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define EXCITATION_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(EXCITATION__SIO)
    #define EXCITATION_SIO_REG            (* (reg32 *) EXCITATION__SIO)
#endif /* (EXCITATION__SIO_CFG) */

/* USBIO registers */
#if !defined(EXCITATION__PC) && (CY_PSOC4_4200L)
    #define EXCITATION_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define EXCITATION_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define EXCITATION_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define EXCITATION_DRIVE_MODE_SHIFT       (0x00u)
#define EXCITATION_DRIVE_MODE_MASK        (0x07u << EXCITATION_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins EXCITATION_H */


/* [] END OF FILE */
