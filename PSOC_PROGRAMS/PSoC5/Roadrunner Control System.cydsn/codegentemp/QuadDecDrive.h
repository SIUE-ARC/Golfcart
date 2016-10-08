/*******************************************************************************
* File Name: QuadDecDrive.h  
* Version 3.0
*
* Description:
*  This file provides constants and parameter values for the Quadrature
*  Decoder component.
*
* Note:
*  None.
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_QUADRATURE_DECODER_QuadDecDrive_H)
#define CY_QUADRATURE_DECODER_QuadDecDrive_H

#include "cyfitter.h"
#include "CyLib.h"
#include "cytypes.h"

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component QuadDec_v3_0 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#define QuadDecDrive_COUNTER_SIZE               (16u)
#define QuadDecDrive_COUNTER_SIZE_8_BIT         (8u)
#define QuadDecDrive_COUNTER_SIZE_16_BIT        (16u)
#define QuadDecDrive_COUNTER_SIZE_32_BIT        (32u)

#if (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_8_BIT)
    #include "QuadDecDrive_Cnt8.h"
#else 
    /* (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_16_BIT) || 
    *  (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT) 
    */
    #include "QuadDecDrive_Cnt16.h"
#endif /* QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_8_BIT */

extern uint8 QuadDecDrive_initVar;


/***************************************
*   Conditional Compilation Parameters
***************************************/

#define QuadDecDrive_COUNTER_RESOLUTION         (4u)


/***************************************
*       Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct
{
    uint8 enableState;
} QuadDecDrive_BACKUP_STRUCT;


/***************************************
*        Function Prototypes
***************************************/

void  QuadDecDrive_Init(void) ;
void  QuadDecDrive_Start(void) ;
void  QuadDecDrive_Stop(void) ;
void  QuadDecDrive_Enable(void) ;
uint8 QuadDecDrive_GetEvents(void) ;
void  QuadDecDrive_SetInterruptMask(uint8 mask) ;
uint8 QuadDecDrive_GetInterruptMask(void) ;
int16 QuadDecDrive_GetCounter(void) ;
void  QuadDecDrive_SetCounter(int16 value)
;
void  QuadDecDrive_Sleep(void) ;
void  QuadDecDrive_Wakeup(void) ;
void  QuadDecDrive_SaveConfig(void) ;
void  QuadDecDrive_RestoreConfig(void) ;

#if (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT)
    CY_ISR_PROTO(QuadDecDrive_ISR);
#endif /* QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT */


/***************************************
*           API Constants
***************************************/

#if (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT)
    #define QuadDecDrive_ISR_NUMBER             ((uint8) QuadDecDrive_isr__INTC_NUMBER)
    #define QuadDecDrive_ISR_PRIORITY           ((uint8) QuadDecDrive_isr__INTC_PRIOR_NUM)
#endif /* QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT */


/***************************************
*    Enumerated Types and Parameters
***************************************/

#define QuadDecDrive_GLITCH_FILTERING           (1u)
#define QuadDecDrive_INDEX_INPUT                (0u)


/***************************************
*    Initial Parameter Constants
***************************************/

#if (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_8_BIT)
    #define QuadDecDrive_COUNTER_INIT_VALUE    (0x80u)
#else 
    /* (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_16_BIT) ||
    *  (QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_32_BIT)
    */
    #define QuadDecDrive_COUNTER_INIT_VALUE    (0x8000u)
    #define QuadDecDrive_COUNTER_MAX_VALUE     (0x7FFFu)
#endif /* QuadDecDrive_COUNTER_SIZE == QuadDecDrive_COUNTER_SIZE_8_BIT */


/***************************************
*             Registers
***************************************/

#define QuadDecDrive_STATUS_REG                 (* (reg8 *) QuadDecDrive_bQuadDec_Stsreg__STATUS_REG)
#define QuadDecDrive_STATUS_PTR                 (  (reg8 *) QuadDecDrive_bQuadDec_Stsreg__STATUS_REG)
#define QuadDecDrive_STATUS_MASK                (* (reg8 *) QuadDecDrive_bQuadDec_Stsreg__MASK_REG)
#define QuadDecDrive_STATUS_MASK_PTR            (  (reg8 *) QuadDecDrive_bQuadDec_Stsreg__MASK_REG)
#define QuadDecDrive_SR_AUX_CONTROL             (* (reg8 *) QuadDecDrive_bQuadDec_Stsreg__STATUS_AUX_CTL_REG)
#define QuadDecDrive_SR_AUX_CONTROL_PTR         (  (reg8 *) QuadDecDrive_bQuadDec_Stsreg__STATUS_AUX_CTL_REG)


/***************************************
*        Register Constants
***************************************/

#define QuadDecDrive_COUNTER_OVERFLOW_SHIFT     (0x00u)
#define QuadDecDrive_COUNTER_UNDERFLOW_SHIFT    (0x01u)
#define QuadDecDrive_COUNTER_RESET_SHIFT        (0x02u)
#define QuadDecDrive_INVALID_IN_SHIFT           (0x03u)
#define QuadDecDrive_COUNTER_OVERFLOW           ((uint8) (0x01u << QuadDecDrive_COUNTER_OVERFLOW_SHIFT))
#define QuadDecDrive_COUNTER_UNDERFLOW          ((uint8) (0x01u << QuadDecDrive_COUNTER_UNDERFLOW_SHIFT))
#define QuadDecDrive_COUNTER_RESET              ((uint8) (0x01u << QuadDecDrive_COUNTER_RESET_SHIFT))
#define QuadDecDrive_INVALID_IN                 ((uint8) (0x01u << QuadDecDrive_INVALID_IN_SHIFT))

#define QuadDecDrive_INTERRUPTS_ENABLE_SHIFT    (0x04u)
#define QuadDecDrive_INTERRUPTS_ENABLE          ((uint8)(0x01u << QuadDecDrive_INTERRUPTS_ENABLE_SHIFT))
#define QuadDecDrive_INIT_INT_MASK              (0x0Fu)


/******************************************************************************************
* Following code are OBSOLETE and must not be used starting from Quadrature Decoder 2.20
******************************************************************************************/
#define QuadDecDrive_DISABLE                    (0x00u)


#endif /* CY_QUADRATURE_DECODER_QuadDecDrive_H */


/* [] END OF FILE */
