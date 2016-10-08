/*******************************************************************************
* File Name: QuadDecSteer.h  
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

#if !defined(CY_QUADRATURE_DECODER_QuadDecSteer_H)
#define CY_QUADRATURE_DECODER_QuadDecSteer_H

#include "cyfitter.h"
#include "CyLib.h"
#include "cytypes.h"

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component QuadDec_v3_0 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#define QuadDecSteer_COUNTER_SIZE               (16u)
#define QuadDecSteer_COUNTER_SIZE_8_BIT         (8u)
#define QuadDecSteer_COUNTER_SIZE_16_BIT        (16u)
#define QuadDecSteer_COUNTER_SIZE_32_BIT        (32u)

#if (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_8_BIT)
    #include "QuadDecSteer_Cnt8.h"
#else 
    /* (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_16_BIT) || 
    *  (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT) 
    */
    #include "QuadDecSteer_Cnt16.h"
#endif /* QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_8_BIT */

extern uint8 QuadDecSteer_initVar;


/***************************************
*   Conditional Compilation Parameters
***************************************/

#define QuadDecSteer_COUNTER_RESOLUTION         (4u)


/***************************************
*       Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct
{
    uint8 enableState;
} QuadDecSteer_BACKUP_STRUCT;


/***************************************
*        Function Prototypes
***************************************/

void  QuadDecSteer_Init(void) ;
void  QuadDecSteer_Start(void) ;
void  QuadDecSteer_Stop(void) ;
void  QuadDecSteer_Enable(void) ;
uint8 QuadDecSteer_GetEvents(void) ;
void  QuadDecSteer_SetInterruptMask(uint8 mask) ;
uint8 QuadDecSteer_GetInterruptMask(void) ;
int16 QuadDecSteer_GetCounter(void) ;
void  QuadDecSteer_SetCounter(int16 value)
;
void  QuadDecSteer_Sleep(void) ;
void  QuadDecSteer_Wakeup(void) ;
void  QuadDecSteer_SaveConfig(void) ;
void  QuadDecSteer_RestoreConfig(void) ;

#if (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT)
    CY_ISR_PROTO(QuadDecSteer_ISR);
#endif /* QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT */


/***************************************
*           API Constants
***************************************/

#if (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT)
    #define QuadDecSteer_ISR_NUMBER             ((uint8) QuadDecSteer_isr__INTC_NUMBER)
    #define QuadDecSteer_ISR_PRIORITY           ((uint8) QuadDecSteer_isr__INTC_PRIOR_NUM)
#endif /* QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT */


/***************************************
*    Enumerated Types and Parameters
***************************************/

#define QuadDecSteer_GLITCH_FILTERING           (1u)
#define QuadDecSteer_INDEX_INPUT                (0u)


/***************************************
*    Initial Parameter Constants
***************************************/

#if (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_8_BIT)
    #define QuadDecSteer_COUNTER_INIT_VALUE    (0x80u)
#else 
    /* (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_16_BIT) ||
    *  (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT)
    */
    #define QuadDecSteer_COUNTER_INIT_VALUE    (0x8000u)
    #define QuadDecSteer_COUNTER_MAX_VALUE     (0x7FFFu)
#endif /* QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_8_BIT */


/***************************************
*             Registers
***************************************/

#define QuadDecSteer_STATUS_REG                 (* (reg8 *) QuadDecSteer_bQuadDec_Stsreg__STATUS_REG)
#define QuadDecSteer_STATUS_PTR                 (  (reg8 *) QuadDecSteer_bQuadDec_Stsreg__STATUS_REG)
#define QuadDecSteer_STATUS_MASK                (* (reg8 *) QuadDecSteer_bQuadDec_Stsreg__MASK_REG)
#define QuadDecSteer_STATUS_MASK_PTR            (  (reg8 *) QuadDecSteer_bQuadDec_Stsreg__MASK_REG)
#define QuadDecSteer_SR_AUX_CONTROL             (* (reg8 *) QuadDecSteer_bQuadDec_Stsreg__STATUS_AUX_CTL_REG)
#define QuadDecSteer_SR_AUX_CONTROL_PTR         (  (reg8 *) QuadDecSteer_bQuadDec_Stsreg__STATUS_AUX_CTL_REG)


/***************************************
*        Register Constants
***************************************/

#define QuadDecSteer_COUNTER_OVERFLOW_SHIFT     (0x00u)
#define QuadDecSteer_COUNTER_UNDERFLOW_SHIFT    (0x01u)
#define QuadDecSteer_COUNTER_RESET_SHIFT        (0x02u)
#define QuadDecSteer_INVALID_IN_SHIFT           (0x03u)
#define QuadDecSteer_COUNTER_OVERFLOW           ((uint8) (0x01u << QuadDecSteer_COUNTER_OVERFLOW_SHIFT))
#define QuadDecSteer_COUNTER_UNDERFLOW          ((uint8) (0x01u << QuadDecSteer_COUNTER_UNDERFLOW_SHIFT))
#define QuadDecSteer_COUNTER_RESET              ((uint8) (0x01u << QuadDecSteer_COUNTER_RESET_SHIFT))
#define QuadDecSteer_INVALID_IN                 ((uint8) (0x01u << QuadDecSteer_INVALID_IN_SHIFT))

#define QuadDecSteer_INTERRUPTS_ENABLE_SHIFT    (0x04u)
#define QuadDecSteer_INTERRUPTS_ENABLE          ((uint8)(0x01u << QuadDecSteer_INTERRUPTS_ENABLE_SHIFT))
#define QuadDecSteer_INIT_INT_MASK              (0x0Fu)


/******************************************************************************************
* Following code are OBSOLETE and must not be used starting from Quadrature Decoder 2.20
******************************************************************************************/
#define QuadDecSteer_DISABLE                    (0x00u)


#endif /* CY_QUADRATURE_DECODER_QuadDecSteer_H */


/* [] END OF FILE */
