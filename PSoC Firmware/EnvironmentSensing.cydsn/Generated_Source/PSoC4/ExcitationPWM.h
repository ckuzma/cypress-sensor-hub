/*******************************************************************************
* File Name: ExcitationPWM.h
* Version 2.10
*
* Description:
*  This file provides constants and parameter values for the ExcitationPWM
*  component.
*
* Note:
*  None
*
********************************************************************************
* Copyright 2013-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_TCPWM_ExcitationPWM_H)
#define CY_TCPWM_ExcitationPWM_H


#include "CyLib.h"
#include "cytypes.h"
#include "cyfitter.h"


/*******************************************************************************
* Internal Type defines
*******************************************************************************/

/* Structure to save state before go to sleep */
typedef struct
{
    uint8  enableState;
} ExcitationPWM_BACKUP_STRUCT;


/*******************************************************************************
* Variables
*******************************************************************************/
extern uint8  ExcitationPWM_initVar;


/***************************************
*   Conditional Compilation Parameters
****************************************/

#define ExcitationPWM_CY_TCPWM_V2                    (CYIPBLOCK_m0s8tcpwm_VERSION == 2u)
#define ExcitationPWM_CY_TCPWM_4000                  (CY_PSOC4_4000)

/* TCPWM Configuration */
#define ExcitationPWM_CONFIG                         (7lu)

/* Quad Mode */
/* Parameters */
#define ExcitationPWM_QUAD_ENCODING_MODES            (0lu)
#define ExcitationPWM_QUAD_AUTO_START                (1lu)

/* Signal modes */
#define ExcitationPWM_QUAD_INDEX_SIGNAL_MODE         (0lu)
#define ExcitationPWM_QUAD_PHIA_SIGNAL_MODE          (3lu)
#define ExcitationPWM_QUAD_PHIB_SIGNAL_MODE          (3lu)
#define ExcitationPWM_QUAD_STOP_SIGNAL_MODE          (0lu)

/* Signal present */
#define ExcitationPWM_QUAD_INDEX_SIGNAL_PRESENT      (0lu)
#define ExcitationPWM_QUAD_STOP_SIGNAL_PRESENT       (0lu)

/* Interrupt Mask */
#define ExcitationPWM_QUAD_INTERRUPT_MASK            (1lu)

/* Timer/Counter Mode */
/* Parameters */
#define ExcitationPWM_TC_RUN_MODE                    (0lu)
#define ExcitationPWM_TC_COUNTER_MODE                (0lu)
#define ExcitationPWM_TC_COMP_CAP_MODE               (2lu)
#define ExcitationPWM_TC_PRESCALER                   (0lu)

/* Signal modes */
#define ExcitationPWM_TC_RELOAD_SIGNAL_MODE          (0lu)
#define ExcitationPWM_TC_COUNT_SIGNAL_MODE           (3lu)
#define ExcitationPWM_TC_START_SIGNAL_MODE           (0lu)
#define ExcitationPWM_TC_STOP_SIGNAL_MODE            (0lu)
#define ExcitationPWM_TC_CAPTURE_SIGNAL_MODE         (0lu)

/* Signal present */
#define ExcitationPWM_TC_RELOAD_SIGNAL_PRESENT       (0lu)
#define ExcitationPWM_TC_COUNT_SIGNAL_PRESENT        (0lu)
#define ExcitationPWM_TC_START_SIGNAL_PRESENT        (0lu)
#define ExcitationPWM_TC_STOP_SIGNAL_PRESENT         (0lu)
#define ExcitationPWM_TC_CAPTURE_SIGNAL_PRESENT      (0lu)

/* Interrupt Mask */
#define ExcitationPWM_TC_INTERRUPT_MASK              (1lu)

/* PWM Mode */
/* Parameters */
#define ExcitationPWM_PWM_KILL_EVENT                 (0lu)
#define ExcitationPWM_PWM_STOP_EVENT                 (0lu)
#define ExcitationPWM_PWM_MODE                       (4lu)
#define ExcitationPWM_PWM_OUT_N_INVERT               (0lu)
#define ExcitationPWM_PWM_OUT_INVERT                 (0lu)
#define ExcitationPWM_PWM_ALIGN                      (0lu)
#define ExcitationPWM_PWM_RUN_MODE                   (0lu)
#define ExcitationPWM_PWM_DEAD_TIME_CYCLE            (0lu)
#define ExcitationPWM_PWM_PRESCALER                  (0lu)

/* Signal modes */
#define ExcitationPWM_PWM_RELOAD_SIGNAL_MODE         (0lu)
#define ExcitationPWM_PWM_COUNT_SIGNAL_MODE          (3lu)
#define ExcitationPWM_PWM_START_SIGNAL_MODE          (0lu)
#define ExcitationPWM_PWM_STOP_SIGNAL_MODE           (0lu)
#define ExcitationPWM_PWM_SWITCH_SIGNAL_MODE         (0lu)

/* Signal present */
#define ExcitationPWM_PWM_RELOAD_SIGNAL_PRESENT      (0lu)
#define ExcitationPWM_PWM_COUNT_SIGNAL_PRESENT       (0lu)
#define ExcitationPWM_PWM_START_SIGNAL_PRESENT       (0lu)
#define ExcitationPWM_PWM_STOP_SIGNAL_PRESENT        (0lu)
#define ExcitationPWM_PWM_SWITCH_SIGNAL_PRESENT      (0lu)

/* Interrupt Mask */
#define ExcitationPWM_PWM_INTERRUPT_MASK             (1lu)


/***************************************
*    Initial Parameter Constants
***************************************/

/* Timer/Counter Mode */
#define ExcitationPWM_TC_PERIOD_VALUE                (65535lu)
#define ExcitationPWM_TC_COMPARE_VALUE               (65535lu)
#define ExcitationPWM_TC_COMPARE_BUF_VALUE           (65535lu)
#define ExcitationPWM_TC_COMPARE_SWAP                (0lu)

/* PWM Mode */
#define ExcitationPWM_PWM_PERIOD_VALUE               (1lu)
#define ExcitationPWM_PWM_PERIOD_BUF_VALUE           (65535lu)
#define ExcitationPWM_PWM_PERIOD_SWAP                (0lu)
#define ExcitationPWM_PWM_COMPARE_VALUE              (1lu)
#define ExcitationPWM_PWM_COMPARE_BUF_VALUE          (65535lu)
#define ExcitationPWM_PWM_COMPARE_SWAP               (0lu)


/***************************************
*    Enumerated Types and Parameters
***************************************/

#define ExcitationPWM__LEFT 0
#define ExcitationPWM__RIGHT 1
#define ExcitationPWM__CENTER 2
#define ExcitationPWM__ASYMMETRIC 3

#define ExcitationPWM__X1 0
#define ExcitationPWM__X2 1
#define ExcitationPWM__X4 2

#define ExcitationPWM__PWM 4
#define ExcitationPWM__PWM_DT 5
#define ExcitationPWM__PWM_PR 6

#define ExcitationPWM__INVERSE 1
#define ExcitationPWM__DIRECT 0

#define ExcitationPWM__CAPTURE 2
#define ExcitationPWM__COMPARE 0

#define ExcitationPWM__TRIG_LEVEL 3
#define ExcitationPWM__TRIG_RISING 0
#define ExcitationPWM__TRIG_FALLING 1
#define ExcitationPWM__TRIG_BOTH 2

#define ExcitationPWM__INTR_MASK_TC 1
#define ExcitationPWM__INTR_MASK_CC_MATCH 2
#define ExcitationPWM__INTR_MASK_NONE 0
#define ExcitationPWM__INTR_MASK_TC_CC 3

#define ExcitationPWM__UNCONFIG 8
#define ExcitationPWM__TIMER 1
#define ExcitationPWM__QUAD 3
#define ExcitationPWM__PWM_SEL 7

#define ExcitationPWM__COUNT_UP 0
#define ExcitationPWM__COUNT_DOWN 1
#define ExcitationPWM__COUNT_UPDOWN0 2
#define ExcitationPWM__COUNT_UPDOWN1 3


/* Prescaler */
#define ExcitationPWM_PRESCALE_DIVBY1                ((uint32)(0u << ExcitationPWM_PRESCALER_SHIFT))
#define ExcitationPWM_PRESCALE_DIVBY2                ((uint32)(1u << ExcitationPWM_PRESCALER_SHIFT))
#define ExcitationPWM_PRESCALE_DIVBY4                ((uint32)(2u << ExcitationPWM_PRESCALER_SHIFT))
#define ExcitationPWM_PRESCALE_DIVBY8                ((uint32)(3u << ExcitationPWM_PRESCALER_SHIFT))
#define ExcitationPWM_PRESCALE_DIVBY16               ((uint32)(4u << ExcitationPWM_PRESCALER_SHIFT))
#define ExcitationPWM_PRESCALE_DIVBY32               ((uint32)(5u << ExcitationPWM_PRESCALER_SHIFT))
#define ExcitationPWM_PRESCALE_DIVBY64               ((uint32)(6u << ExcitationPWM_PRESCALER_SHIFT))
#define ExcitationPWM_PRESCALE_DIVBY128              ((uint32)(7u << ExcitationPWM_PRESCALER_SHIFT))

/* TCPWM set modes */
#define ExcitationPWM_MODE_TIMER_COMPARE             ((uint32)(ExcitationPWM__COMPARE         <<  \
                                                                  ExcitationPWM_MODE_SHIFT))
#define ExcitationPWM_MODE_TIMER_CAPTURE             ((uint32)(ExcitationPWM__CAPTURE         <<  \
                                                                  ExcitationPWM_MODE_SHIFT))
#define ExcitationPWM_MODE_QUAD                      ((uint32)(ExcitationPWM__QUAD            <<  \
                                                                  ExcitationPWM_MODE_SHIFT))
#define ExcitationPWM_MODE_PWM                       ((uint32)(ExcitationPWM__PWM             <<  \
                                                                  ExcitationPWM_MODE_SHIFT))
#define ExcitationPWM_MODE_PWM_DT                    ((uint32)(ExcitationPWM__PWM_DT          <<  \
                                                                  ExcitationPWM_MODE_SHIFT))
#define ExcitationPWM_MODE_PWM_PR                    ((uint32)(ExcitationPWM__PWM_PR          <<  \
                                                                  ExcitationPWM_MODE_SHIFT))

/* Quad Modes */
#define ExcitationPWM_MODE_X1                        ((uint32)(ExcitationPWM__X1              <<  \
                                                                  ExcitationPWM_QUAD_MODE_SHIFT))
#define ExcitationPWM_MODE_X2                        ((uint32)(ExcitationPWM__X2              <<  \
                                                                  ExcitationPWM_QUAD_MODE_SHIFT))
#define ExcitationPWM_MODE_X4                        ((uint32)(ExcitationPWM__X4              <<  \
                                                                  ExcitationPWM_QUAD_MODE_SHIFT))

/* Counter modes */
#define ExcitationPWM_COUNT_UP                       ((uint32)(ExcitationPWM__COUNT_UP        <<  \
                                                                  ExcitationPWM_UPDOWN_SHIFT))
#define ExcitationPWM_COUNT_DOWN                     ((uint32)(ExcitationPWM__COUNT_DOWN      <<  \
                                                                  ExcitationPWM_UPDOWN_SHIFT))
#define ExcitationPWM_COUNT_UPDOWN0                  ((uint32)(ExcitationPWM__COUNT_UPDOWN0   <<  \
                                                                  ExcitationPWM_UPDOWN_SHIFT))
#define ExcitationPWM_COUNT_UPDOWN1                  ((uint32)(ExcitationPWM__COUNT_UPDOWN1   <<  \
                                                                  ExcitationPWM_UPDOWN_SHIFT))

/* PWM output invert */
#define ExcitationPWM_INVERT_LINE                    ((uint32)(ExcitationPWM__INVERSE         <<  \
                                                                  ExcitationPWM_INV_OUT_SHIFT))
#define ExcitationPWM_INVERT_LINE_N                  ((uint32)(ExcitationPWM__INVERSE         <<  \
                                                                  ExcitationPWM_INV_COMPL_OUT_SHIFT))

/* Trigger modes */
#define ExcitationPWM_TRIG_RISING                    ((uint32)ExcitationPWM__TRIG_RISING)
#define ExcitationPWM_TRIG_FALLING                   ((uint32)ExcitationPWM__TRIG_FALLING)
#define ExcitationPWM_TRIG_BOTH                      ((uint32)ExcitationPWM__TRIG_BOTH)
#define ExcitationPWM_TRIG_LEVEL                     ((uint32)ExcitationPWM__TRIG_LEVEL)

/* Interrupt mask */
#define ExcitationPWM_INTR_MASK_TC                   ((uint32)ExcitationPWM__INTR_MASK_TC)
#define ExcitationPWM_INTR_MASK_CC_MATCH             ((uint32)ExcitationPWM__INTR_MASK_CC_MATCH)

/* PWM Output Controls */
#define ExcitationPWM_CC_MATCH_SET                   (0x00u)
#define ExcitationPWM_CC_MATCH_CLEAR                 (0x01u)
#define ExcitationPWM_CC_MATCH_INVERT                (0x02u)
#define ExcitationPWM_CC_MATCH_NO_CHANGE             (0x03u)
#define ExcitationPWM_OVERLOW_SET                    (0x00u)
#define ExcitationPWM_OVERLOW_CLEAR                  (0x04u)
#define ExcitationPWM_OVERLOW_INVERT                 (0x08u)
#define ExcitationPWM_OVERLOW_NO_CHANGE              (0x0Cu)
#define ExcitationPWM_UNDERFLOW_SET                  (0x00u)
#define ExcitationPWM_UNDERFLOW_CLEAR                (0x10u)
#define ExcitationPWM_UNDERFLOW_INVERT               (0x20u)
#define ExcitationPWM_UNDERFLOW_NO_CHANGE            (0x30u)

/* PWM Align */
#define ExcitationPWM_PWM_MODE_LEFT                  (ExcitationPWM_CC_MATCH_CLEAR        |   \
                                                         ExcitationPWM_OVERLOW_SET           |   \
                                                         ExcitationPWM_UNDERFLOW_NO_CHANGE)
#define ExcitationPWM_PWM_MODE_RIGHT                 (ExcitationPWM_CC_MATCH_SET          |   \
                                                         ExcitationPWM_OVERLOW_NO_CHANGE     |   \
                                                         ExcitationPWM_UNDERFLOW_CLEAR)
#define ExcitationPWM_PWM_MODE_ASYM                  (ExcitationPWM_CC_MATCH_INVERT       |   \
                                                         ExcitationPWM_OVERLOW_SET           |   \
                                                         ExcitationPWM_UNDERFLOW_CLEAR)

#if (ExcitationPWM_CY_TCPWM_V2)
    #if(ExcitationPWM_CY_TCPWM_4000)
        #define ExcitationPWM_PWM_MODE_CENTER                (ExcitationPWM_CC_MATCH_INVERT       |   \
                                                                 ExcitationPWM_OVERLOW_NO_CHANGE     |   \
                                                                 ExcitationPWM_UNDERFLOW_CLEAR)
    #else
        #define ExcitationPWM_PWM_MODE_CENTER                (ExcitationPWM_CC_MATCH_INVERT       |   \
                                                                 ExcitationPWM_OVERLOW_SET           |   \
                                                                 ExcitationPWM_UNDERFLOW_CLEAR)
    #endif /* (ExcitationPWM_CY_TCPWM_4000) */
#else
    #define ExcitationPWM_PWM_MODE_CENTER                (ExcitationPWM_CC_MATCH_INVERT       |   \
                                                             ExcitationPWM_OVERLOW_NO_CHANGE     |   \
                                                             ExcitationPWM_UNDERFLOW_CLEAR)
#endif /* (ExcitationPWM_CY_TCPWM_NEW) */

/* Command operations without condition */
#define ExcitationPWM_CMD_CAPTURE                    (0u)
#define ExcitationPWM_CMD_RELOAD                     (8u)
#define ExcitationPWM_CMD_STOP                       (16u)
#define ExcitationPWM_CMD_START                      (24u)

/* Status */
#define ExcitationPWM_STATUS_DOWN                    (1u)
#define ExcitationPWM_STATUS_RUNNING                 (2u)


/***************************************
*        Function Prototypes
****************************************/

void   ExcitationPWM_Init(void);
void   ExcitationPWM_Enable(void);
void   ExcitationPWM_Start(void);
void   ExcitationPWM_Stop(void);

void   ExcitationPWM_SetMode(uint32 mode);
void   ExcitationPWM_SetCounterMode(uint32 counterMode);
void   ExcitationPWM_SetPWMMode(uint32 modeMask);
void   ExcitationPWM_SetQDMode(uint32 qdMode);

void   ExcitationPWM_SetPrescaler(uint32 prescaler);
void   ExcitationPWM_TriggerCommand(uint32 mask, uint32 command);
void   ExcitationPWM_SetOneShot(uint32 oneShotEnable);
uint32 ExcitationPWM_ReadStatus(void);

void   ExcitationPWM_SetPWMSyncKill(uint32 syncKillEnable);
void   ExcitationPWM_SetPWMStopOnKill(uint32 stopOnKillEnable);
void   ExcitationPWM_SetPWMDeadTime(uint32 deadTime);
void   ExcitationPWM_SetPWMInvert(uint32 mask);

void   ExcitationPWM_SetInterruptMode(uint32 interruptMask);
uint32 ExcitationPWM_GetInterruptSourceMasked(void);
uint32 ExcitationPWM_GetInterruptSource(void);
void   ExcitationPWM_ClearInterrupt(uint32 interruptMask);
void   ExcitationPWM_SetInterrupt(uint32 interruptMask);

void   ExcitationPWM_WriteCounter(uint32 count);
uint32 ExcitationPWM_ReadCounter(void);

uint32 ExcitationPWM_ReadCapture(void);
uint32 ExcitationPWM_ReadCaptureBuf(void);

void   ExcitationPWM_WritePeriod(uint32 period);
uint32 ExcitationPWM_ReadPeriod(void);
void   ExcitationPWM_WritePeriodBuf(uint32 periodBuf);
uint32 ExcitationPWM_ReadPeriodBuf(void);

void   ExcitationPWM_WriteCompare(uint32 compare);
uint32 ExcitationPWM_ReadCompare(void);
void   ExcitationPWM_WriteCompareBuf(uint32 compareBuf);
uint32 ExcitationPWM_ReadCompareBuf(void);

void   ExcitationPWM_SetPeriodSwap(uint32 swapEnable);
void   ExcitationPWM_SetCompareSwap(uint32 swapEnable);

void   ExcitationPWM_SetCaptureMode(uint32 triggerMode);
void   ExcitationPWM_SetReloadMode(uint32 triggerMode);
void   ExcitationPWM_SetStartMode(uint32 triggerMode);
void   ExcitationPWM_SetStopMode(uint32 triggerMode);
void   ExcitationPWM_SetCountMode(uint32 triggerMode);

void   ExcitationPWM_SaveConfig(void);
void   ExcitationPWM_RestoreConfig(void);
void   ExcitationPWM_Sleep(void);
void   ExcitationPWM_Wakeup(void);


/***************************************
*             Registers
***************************************/

#define ExcitationPWM_BLOCK_CONTROL_REG              (*(reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__TCPWM_CTRL )
#define ExcitationPWM_BLOCK_CONTROL_PTR              ( (reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__TCPWM_CTRL )
#define ExcitationPWM_COMMAND_REG                    (*(reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__TCPWM_CMD )
#define ExcitationPWM_COMMAND_PTR                    ( (reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__TCPWM_CMD )
#define ExcitationPWM_INTRRUPT_CAUSE_REG             (*(reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE )
#define ExcitationPWM_INTRRUPT_CAUSE_PTR             ( (reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE )
#define ExcitationPWM_CONTROL_REG                    (*(reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__CTRL )
#define ExcitationPWM_CONTROL_PTR                    ( (reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__CTRL )
#define ExcitationPWM_STATUS_REG                     (*(reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__STATUS )
#define ExcitationPWM_STATUS_PTR                     ( (reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__STATUS )
#define ExcitationPWM_COUNTER_REG                    (*(reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__COUNTER )
#define ExcitationPWM_COUNTER_PTR                    ( (reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__COUNTER )
#define ExcitationPWM_COMP_CAP_REG                   (*(reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__CC )
#define ExcitationPWM_COMP_CAP_PTR                   ( (reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__CC )
#define ExcitationPWM_COMP_CAP_BUF_REG               (*(reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__CC_BUFF )
#define ExcitationPWM_COMP_CAP_BUF_PTR               ( (reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__CC_BUFF )
#define ExcitationPWM_PERIOD_REG                     (*(reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__PERIOD )
#define ExcitationPWM_PERIOD_PTR                     ( (reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__PERIOD )
#define ExcitationPWM_PERIOD_BUF_REG                 (*(reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__PERIOD_BUFF )
#define ExcitationPWM_PERIOD_BUF_PTR                 ( (reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__PERIOD_BUFF )
#define ExcitationPWM_TRIG_CONTROL0_REG              (*(reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__TR_CTRL0 )
#define ExcitationPWM_TRIG_CONTROL0_PTR              ( (reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__TR_CTRL0 )
#define ExcitationPWM_TRIG_CONTROL1_REG              (*(reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__TR_CTRL1 )
#define ExcitationPWM_TRIG_CONTROL1_PTR              ( (reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__TR_CTRL1 )
#define ExcitationPWM_TRIG_CONTROL2_REG              (*(reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__TR_CTRL2 )
#define ExcitationPWM_TRIG_CONTROL2_PTR              ( (reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__TR_CTRL2 )
#define ExcitationPWM_INTERRUPT_REQ_REG              (*(reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__INTR )
#define ExcitationPWM_INTERRUPT_REQ_PTR              ( (reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__INTR )
#define ExcitationPWM_INTERRUPT_SET_REG              (*(reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__INTR_SET )
#define ExcitationPWM_INTERRUPT_SET_PTR              ( (reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__INTR_SET )
#define ExcitationPWM_INTERRUPT_MASK_REG             (*(reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__INTR_MASK )
#define ExcitationPWM_INTERRUPT_MASK_PTR             ( (reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__INTR_MASK )
#define ExcitationPWM_INTERRUPT_MASKED_REG           (*(reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__INTR_MASKED )
#define ExcitationPWM_INTERRUPT_MASKED_PTR           ( (reg32 *) ExcitationPWM_cy_m0s8_tcpwm_1__INTR_MASKED )


/***************************************
*       Registers Constants
***************************************/

/* Mask */
#define ExcitationPWM_MASK                           ((uint32)ExcitationPWM_cy_m0s8_tcpwm_1__TCPWM_CTRL_MASK)

/* Shift constants for control register */
#define ExcitationPWM_RELOAD_CC_SHIFT                (0u)
#define ExcitationPWM_RELOAD_PERIOD_SHIFT            (1u)
#define ExcitationPWM_PWM_SYNC_KILL_SHIFT            (2u)
#define ExcitationPWM_PWM_STOP_KILL_SHIFT            (3u)
#define ExcitationPWM_PRESCALER_SHIFT                (8u)
#define ExcitationPWM_UPDOWN_SHIFT                   (16u)
#define ExcitationPWM_ONESHOT_SHIFT                  (18u)
#define ExcitationPWM_QUAD_MODE_SHIFT                (20u)
#define ExcitationPWM_INV_OUT_SHIFT                  (20u)
#define ExcitationPWM_INV_COMPL_OUT_SHIFT            (21u)
#define ExcitationPWM_MODE_SHIFT                     (24u)

/* Mask constants for control register */
#define ExcitationPWM_RELOAD_CC_MASK                 ((uint32)(ExcitationPWM_1BIT_MASK        <<  \
                                                                            ExcitationPWM_RELOAD_CC_SHIFT))
#define ExcitationPWM_RELOAD_PERIOD_MASK             ((uint32)(ExcitationPWM_1BIT_MASK        <<  \
                                                                            ExcitationPWM_RELOAD_PERIOD_SHIFT))
#define ExcitationPWM_PWM_SYNC_KILL_MASK             ((uint32)(ExcitationPWM_1BIT_MASK        <<  \
                                                                            ExcitationPWM_PWM_SYNC_KILL_SHIFT))
#define ExcitationPWM_PWM_STOP_KILL_MASK             ((uint32)(ExcitationPWM_1BIT_MASK        <<  \
                                                                            ExcitationPWM_PWM_STOP_KILL_SHIFT))
#define ExcitationPWM_PRESCALER_MASK                 ((uint32)(ExcitationPWM_8BIT_MASK        <<  \
                                                                            ExcitationPWM_PRESCALER_SHIFT))
#define ExcitationPWM_UPDOWN_MASK                    ((uint32)(ExcitationPWM_2BIT_MASK        <<  \
                                                                            ExcitationPWM_UPDOWN_SHIFT))
#define ExcitationPWM_ONESHOT_MASK                   ((uint32)(ExcitationPWM_1BIT_MASK        <<  \
                                                                            ExcitationPWM_ONESHOT_SHIFT))
#define ExcitationPWM_QUAD_MODE_MASK                 ((uint32)(ExcitationPWM_3BIT_MASK        <<  \
                                                                            ExcitationPWM_QUAD_MODE_SHIFT))
#define ExcitationPWM_INV_OUT_MASK                   ((uint32)(ExcitationPWM_2BIT_MASK        <<  \
                                                                            ExcitationPWM_INV_OUT_SHIFT))
#define ExcitationPWM_MODE_MASK                      ((uint32)(ExcitationPWM_3BIT_MASK        <<  \
                                                                            ExcitationPWM_MODE_SHIFT))

/* Shift constants for trigger control register 1 */
#define ExcitationPWM_CAPTURE_SHIFT                  (0u)
#define ExcitationPWM_COUNT_SHIFT                    (2u)
#define ExcitationPWM_RELOAD_SHIFT                   (4u)
#define ExcitationPWM_STOP_SHIFT                     (6u)
#define ExcitationPWM_START_SHIFT                    (8u)

/* Mask constants for trigger control register 1 */
#define ExcitationPWM_CAPTURE_MASK                   ((uint32)(ExcitationPWM_2BIT_MASK        <<  \
                                                                  ExcitationPWM_CAPTURE_SHIFT))
#define ExcitationPWM_COUNT_MASK                     ((uint32)(ExcitationPWM_2BIT_MASK        <<  \
                                                                  ExcitationPWM_COUNT_SHIFT))
#define ExcitationPWM_RELOAD_MASK                    ((uint32)(ExcitationPWM_2BIT_MASK        <<  \
                                                                  ExcitationPWM_RELOAD_SHIFT))
#define ExcitationPWM_STOP_MASK                      ((uint32)(ExcitationPWM_2BIT_MASK        <<  \
                                                                  ExcitationPWM_STOP_SHIFT))
#define ExcitationPWM_START_MASK                     ((uint32)(ExcitationPWM_2BIT_MASK        <<  \
                                                                  ExcitationPWM_START_SHIFT))

/* MASK */
#define ExcitationPWM_1BIT_MASK                      ((uint32)0x01u)
#define ExcitationPWM_2BIT_MASK                      ((uint32)0x03u)
#define ExcitationPWM_3BIT_MASK                      ((uint32)0x07u)
#define ExcitationPWM_6BIT_MASK                      ((uint32)0x3Fu)
#define ExcitationPWM_8BIT_MASK                      ((uint32)0xFFu)
#define ExcitationPWM_16BIT_MASK                     ((uint32)0xFFFFu)

/* Shift constant for status register */
#define ExcitationPWM_RUNNING_STATUS_SHIFT           (30u)


/***************************************
*    Initial Constants
***************************************/

#define ExcitationPWM_CTRL_QUAD_BASE_CONFIG                                                          \
        (((uint32)(ExcitationPWM_QUAD_ENCODING_MODES     << ExcitationPWM_QUAD_MODE_SHIFT))       |\
         ((uint32)(ExcitationPWM_CONFIG                  << ExcitationPWM_MODE_SHIFT)))

#define ExcitationPWM_CTRL_PWM_BASE_CONFIG                                                           \
        (((uint32)(ExcitationPWM_PWM_STOP_EVENT          << ExcitationPWM_PWM_STOP_KILL_SHIFT))   |\
         ((uint32)(ExcitationPWM_PWM_OUT_INVERT          << ExcitationPWM_INV_OUT_SHIFT))         |\
         ((uint32)(ExcitationPWM_PWM_OUT_N_INVERT        << ExcitationPWM_INV_COMPL_OUT_SHIFT))   |\
         ((uint32)(ExcitationPWM_PWM_MODE                << ExcitationPWM_MODE_SHIFT)))

#define ExcitationPWM_CTRL_PWM_RUN_MODE                                                              \
            ((uint32)(ExcitationPWM_PWM_RUN_MODE         << ExcitationPWM_ONESHOT_SHIFT))
            
#define ExcitationPWM_CTRL_PWM_ALIGN                                                                 \
            ((uint32)(ExcitationPWM_PWM_ALIGN            << ExcitationPWM_UPDOWN_SHIFT))

#define ExcitationPWM_CTRL_PWM_KILL_EVENT                                                            \
             ((uint32)(ExcitationPWM_PWM_KILL_EVENT      << ExcitationPWM_PWM_SYNC_KILL_SHIFT))

#define ExcitationPWM_CTRL_PWM_DEAD_TIME_CYCLE                                                       \
            ((uint32)(ExcitationPWM_PWM_DEAD_TIME_CYCLE  << ExcitationPWM_PRESCALER_SHIFT))

#define ExcitationPWM_CTRL_PWM_PRESCALER                                                             \
            ((uint32)(ExcitationPWM_PWM_PRESCALER        << ExcitationPWM_PRESCALER_SHIFT))

#define ExcitationPWM_CTRL_TIMER_BASE_CONFIG                                                         \
        (((uint32)(ExcitationPWM_TC_PRESCALER            << ExcitationPWM_PRESCALER_SHIFT))       |\
         ((uint32)(ExcitationPWM_TC_COUNTER_MODE         << ExcitationPWM_UPDOWN_SHIFT))          |\
         ((uint32)(ExcitationPWM_TC_RUN_MODE             << ExcitationPWM_ONESHOT_SHIFT))         |\
         ((uint32)(ExcitationPWM_TC_COMP_CAP_MODE        << ExcitationPWM_MODE_SHIFT)))
        
#define ExcitationPWM_QUAD_SIGNALS_MODES                                                             \
        (((uint32)(ExcitationPWM_QUAD_PHIA_SIGNAL_MODE   << ExcitationPWM_COUNT_SHIFT))           |\
         ((uint32)(ExcitationPWM_QUAD_INDEX_SIGNAL_MODE  << ExcitationPWM_RELOAD_SHIFT))          |\
         ((uint32)(ExcitationPWM_QUAD_STOP_SIGNAL_MODE   << ExcitationPWM_STOP_SHIFT))            |\
         ((uint32)(ExcitationPWM_QUAD_PHIB_SIGNAL_MODE   << ExcitationPWM_START_SHIFT)))

#define ExcitationPWM_PWM_SIGNALS_MODES                                                              \
        (((uint32)(ExcitationPWM_PWM_SWITCH_SIGNAL_MODE  << ExcitationPWM_CAPTURE_SHIFT))         |\
         ((uint32)(ExcitationPWM_PWM_COUNT_SIGNAL_MODE   << ExcitationPWM_COUNT_SHIFT))           |\
         ((uint32)(ExcitationPWM_PWM_RELOAD_SIGNAL_MODE  << ExcitationPWM_RELOAD_SHIFT))          |\
         ((uint32)(ExcitationPWM_PWM_STOP_SIGNAL_MODE    << ExcitationPWM_STOP_SHIFT))            |\
         ((uint32)(ExcitationPWM_PWM_START_SIGNAL_MODE   << ExcitationPWM_START_SHIFT)))

#define ExcitationPWM_TIMER_SIGNALS_MODES                                                            \
        (((uint32)(ExcitationPWM_TC_CAPTURE_SIGNAL_MODE  << ExcitationPWM_CAPTURE_SHIFT))         |\
         ((uint32)(ExcitationPWM_TC_COUNT_SIGNAL_MODE    << ExcitationPWM_COUNT_SHIFT))           |\
         ((uint32)(ExcitationPWM_TC_RELOAD_SIGNAL_MODE   << ExcitationPWM_RELOAD_SHIFT))          |\
         ((uint32)(ExcitationPWM_TC_STOP_SIGNAL_MODE     << ExcitationPWM_STOP_SHIFT))            |\
         ((uint32)(ExcitationPWM_TC_START_SIGNAL_MODE    << ExcitationPWM_START_SHIFT)))
        
#define ExcitationPWM_TIMER_UPDOWN_CNT_USED                                                          \
                ((ExcitationPWM__COUNT_UPDOWN0 == ExcitationPWM_TC_COUNTER_MODE)                  ||\
                 (ExcitationPWM__COUNT_UPDOWN1 == ExcitationPWM_TC_COUNTER_MODE))

#define ExcitationPWM_PWM_UPDOWN_CNT_USED                                                            \
                ((ExcitationPWM__CENTER == ExcitationPWM_PWM_ALIGN)                               ||\
                 (ExcitationPWM__ASYMMETRIC == ExcitationPWM_PWM_ALIGN))               
        
#define ExcitationPWM_PWM_PR_INIT_VALUE              (1u)
#define ExcitationPWM_QUAD_PERIOD_INIT_VALUE         (0x8000u)



#endif /* End CY_TCPWM_ExcitationPWM_H */

/* [] END OF FILE */
