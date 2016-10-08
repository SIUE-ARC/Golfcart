/*******************************************************************************
* File Name: BRAKEPOT.h  
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

#if !defined(CY_PINS_BRAKEPOT_H) /* Pins BRAKEPOT_H */
#define CY_PINS_BRAKEPOT_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "BRAKEPOT_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 BRAKEPOT__PORT == 15 && ((BRAKEPOT__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    BRAKEPOT_Write(uint8 value);
void    BRAKEPOT_SetDriveMode(uint8 mode);
uint8   BRAKEPOT_ReadDataReg(void);
uint8   BRAKEPOT_Read(void);
void    BRAKEPOT_SetInterruptMode(uint16 position, uint16 mode);
uint8   BRAKEPOT_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the BRAKEPOT_SetDriveMode() function.
     *  @{
     */
        #define BRAKEPOT_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define BRAKEPOT_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define BRAKEPOT_DM_RES_UP          PIN_DM_RES_UP
        #define BRAKEPOT_DM_RES_DWN         PIN_DM_RES_DWN
        #define BRAKEPOT_DM_OD_LO           PIN_DM_OD_LO
        #define BRAKEPOT_DM_OD_HI           PIN_DM_OD_HI
        #define BRAKEPOT_DM_STRONG          PIN_DM_STRONG
        #define BRAKEPOT_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define BRAKEPOT_MASK               BRAKEPOT__MASK
#define BRAKEPOT_SHIFT              BRAKEPOT__SHIFT
#define BRAKEPOT_WIDTH              1u

/* Interrupt constants */
#if defined(BRAKEPOT__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in BRAKEPOT_SetInterruptMode() function.
     *  @{
     */
        #define BRAKEPOT_INTR_NONE      (uint16)(0x0000u)
        #define BRAKEPOT_INTR_RISING    (uint16)(0x0001u)
        #define BRAKEPOT_INTR_FALLING   (uint16)(0x0002u)
        #define BRAKEPOT_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define BRAKEPOT_INTR_MASK      (0x01u) 
#endif /* (BRAKEPOT__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define BRAKEPOT_PS                     (* (reg8 *) BRAKEPOT__PS)
/* Data Register */
#define BRAKEPOT_DR                     (* (reg8 *) BRAKEPOT__DR)
/* Port Number */
#define BRAKEPOT_PRT_NUM                (* (reg8 *) BRAKEPOT__PRT) 
/* Connect to Analog Globals */                                                  
#define BRAKEPOT_AG                     (* (reg8 *) BRAKEPOT__AG)                       
/* Analog MUX bux enable */
#define BRAKEPOT_AMUX                   (* (reg8 *) BRAKEPOT__AMUX) 
/* Bidirectional Enable */                                                        
#define BRAKEPOT_BIE                    (* (reg8 *) BRAKEPOT__BIE)
/* Bit-mask for Aliased Register Access */
#define BRAKEPOT_BIT_MASK               (* (reg8 *) BRAKEPOT__BIT_MASK)
/* Bypass Enable */
#define BRAKEPOT_BYP                    (* (reg8 *) BRAKEPOT__BYP)
/* Port wide control signals */                                                   
#define BRAKEPOT_CTL                    (* (reg8 *) BRAKEPOT__CTL)
/* Drive Modes */
#define BRAKEPOT_DM0                    (* (reg8 *) BRAKEPOT__DM0) 
#define BRAKEPOT_DM1                    (* (reg8 *) BRAKEPOT__DM1)
#define BRAKEPOT_DM2                    (* (reg8 *) BRAKEPOT__DM2) 
/* Input Buffer Disable Override */
#define BRAKEPOT_INP_DIS                (* (reg8 *) BRAKEPOT__INP_DIS)
/* LCD Common or Segment Drive */
#define BRAKEPOT_LCD_COM_SEG            (* (reg8 *) BRAKEPOT__LCD_COM_SEG)
/* Enable Segment LCD */
#define BRAKEPOT_LCD_EN                 (* (reg8 *) BRAKEPOT__LCD_EN)
/* Slew Rate Control */
#define BRAKEPOT_SLW                    (* (reg8 *) BRAKEPOT__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define BRAKEPOT_PRTDSI__CAPS_SEL       (* (reg8 *) BRAKEPOT__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define BRAKEPOT_PRTDSI__DBL_SYNC_IN    (* (reg8 *) BRAKEPOT__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define BRAKEPOT_PRTDSI__OE_SEL0        (* (reg8 *) BRAKEPOT__PRTDSI__OE_SEL0) 
#define BRAKEPOT_PRTDSI__OE_SEL1        (* (reg8 *) BRAKEPOT__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define BRAKEPOT_PRTDSI__OUT_SEL0       (* (reg8 *) BRAKEPOT__PRTDSI__OUT_SEL0) 
#define BRAKEPOT_PRTDSI__OUT_SEL1       (* (reg8 *) BRAKEPOT__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define BRAKEPOT_PRTDSI__SYNC_OUT       (* (reg8 *) BRAKEPOT__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(BRAKEPOT__SIO_CFG)
    #define BRAKEPOT_SIO_HYST_EN        (* (reg8 *) BRAKEPOT__SIO_HYST_EN)
    #define BRAKEPOT_SIO_REG_HIFREQ     (* (reg8 *) BRAKEPOT__SIO_REG_HIFREQ)
    #define BRAKEPOT_SIO_CFG            (* (reg8 *) BRAKEPOT__SIO_CFG)
    #define BRAKEPOT_SIO_DIFF           (* (reg8 *) BRAKEPOT__SIO_DIFF)
#endif /* (BRAKEPOT__SIO_CFG) */

/* Interrupt Registers */
#if defined(BRAKEPOT__INTSTAT)
    #define BRAKEPOT_INTSTAT            (* (reg8 *) BRAKEPOT__INTSTAT)
    #define BRAKEPOT_SNAP               (* (reg8 *) BRAKEPOT__SNAP)
    
	#define BRAKEPOT_0_INTTYPE_REG 		(* (reg8 *) BRAKEPOT__0__INTTYPE)
#endif /* (BRAKEPOT__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_BRAKEPOT_H */


/* [] END OF FILE */
