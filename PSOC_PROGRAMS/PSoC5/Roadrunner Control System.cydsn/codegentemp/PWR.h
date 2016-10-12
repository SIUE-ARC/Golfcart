/*******************************************************************************
* File Name: PWR.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_PWR_H) /* Pins PWR_H */
#define CY_PINS_PWR_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "PWR_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 PWR__PORT == 15 && ((PWR__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    PWR_Write(uint8 value);
void    PWR_SetDriveMode(uint8 mode);
uint8   PWR_ReadDataReg(void);
uint8   PWR_Read(void);
void    PWR_SetInterruptMode(uint16 position, uint16 mode);
uint8   PWR_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the PWR_SetDriveMode() function.
     *  @{
     */
        #define PWR_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define PWR_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define PWR_DM_RES_UP          PIN_DM_RES_UP
        #define PWR_DM_RES_DWN         PIN_DM_RES_DWN
        #define PWR_DM_OD_LO           PIN_DM_OD_LO
        #define PWR_DM_OD_HI           PIN_DM_OD_HI
        #define PWR_DM_STRONG          PIN_DM_STRONG
        #define PWR_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define PWR_MASK               PWR__MASK
#define PWR_SHIFT              PWR__SHIFT
#define PWR_WIDTH              1u

/* Interrupt constants */
#if defined(PWR__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in PWR_SetInterruptMode() function.
     *  @{
     */
        #define PWR_INTR_NONE      (uint16)(0x0000u)
        #define PWR_INTR_RISING    (uint16)(0x0001u)
        #define PWR_INTR_FALLING   (uint16)(0x0002u)
        #define PWR_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define PWR_INTR_MASK      (0x01u) 
#endif /* (PWR__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define PWR_PS                     (* (reg8 *) PWR__PS)
/* Data Register */
#define PWR_DR                     (* (reg8 *) PWR__DR)
/* Port Number */
#define PWR_PRT_NUM                (* (reg8 *) PWR__PRT) 
/* Connect to Analog Globals */                                                  
#define PWR_AG                     (* (reg8 *) PWR__AG)                       
/* Analog MUX bux enable */
#define PWR_AMUX                   (* (reg8 *) PWR__AMUX) 
/* Bidirectional Enable */                                                        
#define PWR_BIE                    (* (reg8 *) PWR__BIE)
/* Bit-mask for Aliased Register Access */
#define PWR_BIT_MASK               (* (reg8 *) PWR__BIT_MASK)
/* Bypass Enable */
#define PWR_BYP                    (* (reg8 *) PWR__BYP)
/* Port wide control signals */                                                   
#define PWR_CTL                    (* (reg8 *) PWR__CTL)
/* Drive Modes */
#define PWR_DM0                    (* (reg8 *) PWR__DM0) 
#define PWR_DM1                    (* (reg8 *) PWR__DM1)
#define PWR_DM2                    (* (reg8 *) PWR__DM2) 
/* Input Buffer Disable Override */
#define PWR_INP_DIS                (* (reg8 *) PWR__INP_DIS)
/* LCD Common or Segment Drive */
#define PWR_LCD_COM_SEG            (* (reg8 *) PWR__LCD_COM_SEG)
/* Enable Segment LCD */
#define PWR_LCD_EN                 (* (reg8 *) PWR__LCD_EN)
/* Slew Rate Control */
#define PWR_SLW                    (* (reg8 *) PWR__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define PWR_PRTDSI__CAPS_SEL       (* (reg8 *) PWR__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define PWR_PRTDSI__DBL_SYNC_IN    (* (reg8 *) PWR__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define PWR_PRTDSI__OE_SEL0        (* (reg8 *) PWR__PRTDSI__OE_SEL0) 
#define PWR_PRTDSI__OE_SEL1        (* (reg8 *) PWR__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define PWR_PRTDSI__OUT_SEL0       (* (reg8 *) PWR__PRTDSI__OUT_SEL0) 
#define PWR_PRTDSI__OUT_SEL1       (* (reg8 *) PWR__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define PWR_PRTDSI__SYNC_OUT       (* (reg8 *) PWR__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(PWR__SIO_CFG)
    #define PWR_SIO_HYST_EN        (* (reg8 *) PWR__SIO_HYST_EN)
    #define PWR_SIO_REG_HIFREQ     (* (reg8 *) PWR__SIO_REG_HIFREQ)
    #define PWR_SIO_CFG            (* (reg8 *) PWR__SIO_CFG)
    #define PWR_SIO_DIFF           (* (reg8 *) PWR__SIO_DIFF)
#endif /* (PWR__SIO_CFG) */

/* Interrupt Registers */
#if defined(PWR__INTSTAT)
    #define PWR_INTSTAT            (* (reg8 *) PWR__INTSTAT)
    #define PWR_SNAP               (* (reg8 *) PWR__SNAP)
    
	#define PWR_0_INTTYPE_REG 		(* (reg8 *) PWR__0__INTTYPE)
#endif /* (PWR__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_PWR_H */


/* [] END OF FILE */
