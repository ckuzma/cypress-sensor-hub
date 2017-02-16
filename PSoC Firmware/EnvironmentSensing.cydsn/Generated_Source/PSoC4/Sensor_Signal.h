/*******************************************************************************
* File Name: Sensor_Signal.h  
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

#if !defined(CY_PINS_Sensor_Signal_H) /* Pins Sensor_Signal_H */
#define CY_PINS_Sensor_Signal_H

#include "cytypes.h"
#include "cyfitter.h"
#include "Sensor_Signal_aliases.h"


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
} Sensor_Signal_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   Sensor_Signal_Read(void);
void    Sensor_Signal_Write(uint8 value);
uint8   Sensor_Signal_ReadDataReg(void);
#if defined(Sensor_Signal__PC) || (CY_PSOC4_4200L) 
    void    Sensor_Signal_SetDriveMode(uint8 mode);
#endif
void    Sensor_Signal_SetInterruptMode(uint16 position, uint16 mode);
uint8   Sensor_Signal_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void Sensor_Signal_Sleep(void); 
void Sensor_Signal_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(Sensor_Signal__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define Sensor_Signal_DRIVE_MODE_BITS        (3)
    #define Sensor_Signal_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - Sensor_Signal_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the Sensor_Signal_SetDriveMode() function.
         *  @{
         */
        #define Sensor_Signal_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define Sensor_Signal_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define Sensor_Signal_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define Sensor_Signal_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define Sensor_Signal_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define Sensor_Signal_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define Sensor_Signal_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define Sensor_Signal_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define Sensor_Signal_MASK               Sensor_Signal__MASK
#define Sensor_Signal_SHIFT              Sensor_Signal__SHIFT
#define Sensor_Signal_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in Sensor_Signal_SetInterruptMode() function.
     *  @{
     */
        #define Sensor_Signal_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define Sensor_Signal_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define Sensor_Signal_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define Sensor_Signal_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(Sensor_Signal__SIO)
    #define Sensor_Signal_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(Sensor_Signal__PC) && (CY_PSOC4_4200L)
    #define Sensor_Signal_USBIO_ENABLE               ((uint32)0x80000000u)
    #define Sensor_Signal_USBIO_DISABLE              ((uint32)(~Sensor_Signal_USBIO_ENABLE))
    #define Sensor_Signal_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define Sensor_Signal_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define Sensor_Signal_USBIO_ENTER_SLEEP          ((uint32)((1u << Sensor_Signal_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << Sensor_Signal_USBIO_SUSPEND_DEL_SHIFT)))
    #define Sensor_Signal_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << Sensor_Signal_USBIO_SUSPEND_SHIFT)))
    #define Sensor_Signal_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << Sensor_Signal_USBIO_SUSPEND_DEL_SHIFT)))
    #define Sensor_Signal_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(Sensor_Signal__PC)
    /* Port Configuration */
    #define Sensor_Signal_PC                 (* (reg32 *) Sensor_Signal__PC)
#endif
/* Pin State */
#define Sensor_Signal_PS                     (* (reg32 *) Sensor_Signal__PS)
/* Data Register */
#define Sensor_Signal_DR                     (* (reg32 *) Sensor_Signal__DR)
/* Input Buffer Disable Override */
#define Sensor_Signal_INP_DIS                (* (reg32 *) Sensor_Signal__PC2)

/* Interrupt configuration Registers */
#define Sensor_Signal_INTCFG                 (* (reg32 *) Sensor_Signal__INTCFG)
#define Sensor_Signal_INTSTAT                (* (reg32 *) Sensor_Signal__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define Sensor_Signal_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(Sensor_Signal__SIO)
    #define Sensor_Signal_SIO_REG            (* (reg32 *) Sensor_Signal__SIO)
#endif /* (Sensor_Signal__SIO_CFG) */

/* USBIO registers */
#if !defined(Sensor_Signal__PC) && (CY_PSOC4_4200L)
    #define Sensor_Signal_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define Sensor_Signal_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define Sensor_Signal_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define Sensor_Signal_DRIVE_MODE_SHIFT       (0x00u)
#define Sensor_Signal_DRIVE_MODE_MASK        (0x07u << Sensor_Signal_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins Sensor_Signal_H */


/* [] END OF FILE */
