/*******************************************************************************
* File Name: SENCB.h  
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

#if !defined(CY_PINS_SENCB_H) /* Pins SENCB_H */
#define CY_PINS_SENCB_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "SENCB_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 SENCB__PORT == 15 && ((SENCB__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    SENCB_Write(uint8 value);
void    SENCB_SetDriveMode(uint8 mode);
uint8   SENCB_ReadDataReg(void);
uint8   SENCB_Read(void);
void    SENCB_SetInterruptMode(uint16 position, uint16 mode);
uint8   SENCB_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the SENCB_SetDriveMode() function.
     *  @{
     */
        #define SENCB_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define SENCB_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define SENCB_DM_RES_UP          PIN_DM_RES_UP
        #define SENCB_DM_RES_DWN         PIN_DM_RES_DWN
        #define SENCB_DM_OD_LO           PIN_DM_OD_LO
        #define SENCB_DM_OD_HI           PIN_DM_OD_HI
        #define SENCB_DM_STRONG          PIN_DM_STRONG
        #define SENCB_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define SENCB_MASK               SENCB__MASK
#define SENCB_SHIFT              SENCB__SHIFT
#define SENCB_WIDTH              1u

/* Interrupt constants */
#if defined(SENCB__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in SENCB_SetInterruptMode() function.
     *  @{
     */
        #define SENCB_INTR_NONE      (uint16)(0x0000u)
        #define SENCB_INTR_RISING    (uint16)(0x0001u)
        #define SENCB_INTR_FALLING   (uint16)(0x0002u)
        #define SENCB_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define SENCB_INTR_MASK      (0x01u) 
#endif /* (SENCB__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define SENCB_PS                     (* (reg8 *) SENCB__PS)
/* Data Register */
#define SENCB_DR                     (* (reg8 *) SENCB__DR)
/* Port Number */
#define SENCB_PRT_NUM                (* (reg8 *) SENCB__PRT) 
/* Connect to Analog Globals */                                                  
#define SENCB_AG                     (* (reg8 *) SENCB__AG)                       
/* Analog MUX bux enable */
#define SENCB_AMUX                   (* (reg8 *) SENCB__AMUX) 
/* Bidirectional Enable */                                                        
#define SENCB_BIE                    (* (reg8 *) SENCB__BIE)
/* Bit-mask for Aliased Register Access */
#define SENCB_BIT_MASK               (* (reg8 *) SENCB__BIT_MASK)
/* Bypass Enable */
#define SENCB_BYP                    (* (reg8 *) SENCB__BYP)
/* Port wide control signals */                                                   
#define SENCB_CTL                    (* (reg8 *) SENCB__CTL)
/* Drive Modes */
#define SENCB_DM0                    (* (reg8 *) SENCB__DM0) 
#define SENCB_DM1                    (* (reg8 *) SENCB__DM1)
#define SENCB_DM2                    (* (reg8 *) SENCB__DM2) 
/* Input Buffer Disable Override */
#define SENCB_INP_DIS                (* (reg8 *) SENCB__INP_DIS)
/* LCD Common or Segment Drive */
#define SENCB_LCD_COM_SEG            (* (reg8 *) SENCB__LCD_COM_SEG)
/* Enable Segment LCD */
#define SENCB_LCD_EN                 (* (reg8 *) SENCB__LCD_EN)
/* Slew Rate Control */
#define SENCB_SLW                    (* (reg8 *) SENCB__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define SENCB_PRTDSI__CAPS_SEL       (* (reg8 *) SENCB__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define SENCB_PRTDSI__DBL_SYNC_IN    (* (reg8 *) SENCB__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define SENCB_PRTDSI__OE_SEL0        (* (reg8 *) SENCB__PRTDSI__OE_SEL0) 
#define SENCB_PRTDSI__OE_SEL1        (* (reg8 *) SENCB__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define SENCB_PRTDSI__OUT_SEL0       (* (reg8 *) SENCB__PRTDSI__OUT_SEL0) 
#define SENCB_PRTDSI__OUT_SEL1       (* (reg8 *) SENCB__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define SENCB_PRTDSI__SYNC_OUT       (* (reg8 *) SENCB__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(SENCB__SIO_CFG)
    #define SENCB_SIO_HYST_EN        (* (reg8 *) SENCB__SIO_HYST_EN)
    #define SENCB_SIO_REG_HIFREQ     (* (reg8 *) SENCB__SIO_REG_HIFREQ)
    #define SENCB_SIO_CFG            (* (reg8 *) SENCB__SIO_CFG)
    #define SENCB_SIO_DIFF           (* (reg8 *) SENCB__SIO_DIFF)
#endif /* (SENCB__SIO_CFG) */

/* Interrupt Registers */
#if defined(SENCB__INTSTAT)
    #define SENCB_INTSTAT            (* (reg8 *) SENCB__INTSTAT)
    #define SENCB_SNAP               (* (reg8 *) SENCB__SNAP)
    
	#define SENCB_0_INTTYPE_REG 		(* (reg8 *) SENCB__0__INTTYPE)
#endif /* (SENCB__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_SENCB_H */


/* [] END OF FILE */
