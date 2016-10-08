/*******************************************************************************
* File Name: STEERPOT.h  
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

#if !defined(CY_PINS_STEERPOT_H) /* Pins STEERPOT_H */
#define CY_PINS_STEERPOT_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "STEERPOT_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 STEERPOT__PORT == 15 && ((STEERPOT__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    STEERPOT_Write(uint8 value);
void    STEERPOT_SetDriveMode(uint8 mode);
uint8   STEERPOT_ReadDataReg(void);
uint8   STEERPOT_Read(void);
void    STEERPOT_SetInterruptMode(uint16 position, uint16 mode);
uint8   STEERPOT_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the STEERPOT_SetDriveMode() function.
     *  @{
     */
        #define STEERPOT_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define STEERPOT_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define STEERPOT_DM_RES_UP          PIN_DM_RES_UP
        #define STEERPOT_DM_RES_DWN         PIN_DM_RES_DWN
        #define STEERPOT_DM_OD_LO           PIN_DM_OD_LO
        #define STEERPOT_DM_OD_HI           PIN_DM_OD_HI
        #define STEERPOT_DM_STRONG          PIN_DM_STRONG
        #define STEERPOT_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define STEERPOT_MASK               STEERPOT__MASK
#define STEERPOT_SHIFT              STEERPOT__SHIFT
#define STEERPOT_WIDTH              1u

/* Interrupt constants */
#if defined(STEERPOT__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in STEERPOT_SetInterruptMode() function.
     *  @{
     */
        #define STEERPOT_INTR_NONE      (uint16)(0x0000u)
        #define STEERPOT_INTR_RISING    (uint16)(0x0001u)
        #define STEERPOT_INTR_FALLING   (uint16)(0x0002u)
        #define STEERPOT_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define STEERPOT_INTR_MASK      (0x01u) 
#endif /* (STEERPOT__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define STEERPOT_PS                     (* (reg8 *) STEERPOT__PS)
/* Data Register */
#define STEERPOT_DR                     (* (reg8 *) STEERPOT__DR)
/* Port Number */
#define STEERPOT_PRT_NUM                (* (reg8 *) STEERPOT__PRT) 
/* Connect to Analog Globals */                                                  
#define STEERPOT_AG                     (* (reg8 *) STEERPOT__AG)                       
/* Analog MUX bux enable */
#define STEERPOT_AMUX                   (* (reg8 *) STEERPOT__AMUX) 
/* Bidirectional Enable */                                                        
#define STEERPOT_BIE                    (* (reg8 *) STEERPOT__BIE)
/* Bit-mask for Aliased Register Access */
#define STEERPOT_BIT_MASK               (* (reg8 *) STEERPOT__BIT_MASK)
/* Bypass Enable */
#define STEERPOT_BYP                    (* (reg8 *) STEERPOT__BYP)
/* Port wide control signals */                                                   
#define STEERPOT_CTL                    (* (reg8 *) STEERPOT__CTL)
/* Drive Modes */
#define STEERPOT_DM0                    (* (reg8 *) STEERPOT__DM0) 
#define STEERPOT_DM1                    (* (reg8 *) STEERPOT__DM1)
#define STEERPOT_DM2                    (* (reg8 *) STEERPOT__DM2) 
/* Input Buffer Disable Override */
#define STEERPOT_INP_DIS                (* (reg8 *) STEERPOT__INP_DIS)
/* LCD Common or Segment Drive */
#define STEERPOT_LCD_COM_SEG            (* (reg8 *) STEERPOT__LCD_COM_SEG)
/* Enable Segment LCD */
#define STEERPOT_LCD_EN                 (* (reg8 *) STEERPOT__LCD_EN)
/* Slew Rate Control */
#define STEERPOT_SLW                    (* (reg8 *) STEERPOT__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define STEERPOT_PRTDSI__CAPS_SEL       (* (reg8 *) STEERPOT__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define STEERPOT_PRTDSI__DBL_SYNC_IN    (* (reg8 *) STEERPOT__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define STEERPOT_PRTDSI__OE_SEL0        (* (reg8 *) STEERPOT__PRTDSI__OE_SEL0) 
#define STEERPOT_PRTDSI__OE_SEL1        (* (reg8 *) STEERPOT__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define STEERPOT_PRTDSI__OUT_SEL0       (* (reg8 *) STEERPOT__PRTDSI__OUT_SEL0) 
#define STEERPOT_PRTDSI__OUT_SEL1       (* (reg8 *) STEERPOT__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define STEERPOT_PRTDSI__SYNC_OUT       (* (reg8 *) STEERPOT__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(STEERPOT__SIO_CFG)
    #define STEERPOT_SIO_HYST_EN        (* (reg8 *) STEERPOT__SIO_HYST_EN)
    #define STEERPOT_SIO_REG_HIFREQ     (* (reg8 *) STEERPOT__SIO_REG_HIFREQ)
    #define STEERPOT_SIO_CFG            (* (reg8 *) STEERPOT__SIO_CFG)
    #define STEERPOT_SIO_DIFF           (* (reg8 *) STEERPOT__SIO_DIFF)
#endif /* (STEERPOT__SIO_CFG) */

/* Interrupt Registers */
#if defined(STEERPOT__INTSTAT)
    #define STEERPOT_INTSTAT            (* (reg8 *) STEERPOT__INTSTAT)
    #define STEERPOT_SNAP               (* (reg8 *) STEERPOT__SNAP)
    
	#define STEERPOT_0_INTTYPE_REG 		(* (reg8 *) STEERPOT__0__INTTYPE)
#endif /* (STEERPOT__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_STEERPOT_H */


/* [] END OF FILE */
