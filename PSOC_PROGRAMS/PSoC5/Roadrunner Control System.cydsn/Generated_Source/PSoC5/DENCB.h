/*******************************************************************************
* File Name: DENCB.h  
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

#if !defined(CY_PINS_DENCB_H) /* Pins DENCB_H */
#define CY_PINS_DENCB_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "DENCB_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 DENCB__PORT == 15 && ((DENCB__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    DENCB_Write(uint8 value);
void    DENCB_SetDriveMode(uint8 mode);
uint8   DENCB_ReadDataReg(void);
uint8   DENCB_Read(void);
void    DENCB_SetInterruptMode(uint16 position, uint16 mode);
uint8   DENCB_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the DENCB_SetDriveMode() function.
     *  @{
     */
        #define DENCB_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define DENCB_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define DENCB_DM_RES_UP          PIN_DM_RES_UP
        #define DENCB_DM_RES_DWN         PIN_DM_RES_DWN
        #define DENCB_DM_OD_LO           PIN_DM_OD_LO
        #define DENCB_DM_OD_HI           PIN_DM_OD_HI
        #define DENCB_DM_STRONG          PIN_DM_STRONG
        #define DENCB_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define DENCB_MASK               DENCB__MASK
#define DENCB_SHIFT              DENCB__SHIFT
#define DENCB_WIDTH              1u

/* Interrupt constants */
#if defined(DENCB__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in DENCB_SetInterruptMode() function.
     *  @{
     */
        #define DENCB_INTR_NONE      (uint16)(0x0000u)
        #define DENCB_INTR_RISING    (uint16)(0x0001u)
        #define DENCB_INTR_FALLING   (uint16)(0x0002u)
        #define DENCB_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define DENCB_INTR_MASK      (0x01u) 
#endif /* (DENCB__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define DENCB_PS                     (* (reg8 *) DENCB__PS)
/* Data Register */
#define DENCB_DR                     (* (reg8 *) DENCB__DR)
/* Port Number */
#define DENCB_PRT_NUM                (* (reg8 *) DENCB__PRT) 
/* Connect to Analog Globals */                                                  
#define DENCB_AG                     (* (reg8 *) DENCB__AG)                       
/* Analog MUX bux enable */
#define DENCB_AMUX                   (* (reg8 *) DENCB__AMUX) 
/* Bidirectional Enable */                                                        
#define DENCB_BIE                    (* (reg8 *) DENCB__BIE)
/* Bit-mask for Aliased Register Access */
#define DENCB_BIT_MASK               (* (reg8 *) DENCB__BIT_MASK)
/* Bypass Enable */
#define DENCB_BYP                    (* (reg8 *) DENCB__BYP)
/* Port wide control signals */                                                   
#define DENCB_CTL                    (* (reg8 *) DENCB__CTL)
/* Drive Modes */
#define DENCB_DM0                    (* (reg8 *) DENCB__DM0) 
#define DENCB_DM1                    (* (reg8 *) DENCB__DM1)
#define DENCB_DM2                    (* (reg8 *) DENCB__DM2) 
/* Input Buffer Disable Override */
#define DENCB_INP_DIS                (* (reg8 *) DENCB__INP_DIS)
/* LCD Common or Segment Drive */
#define DENCB_LCD_COM_SEG            (* (reg8 *) DENCB__LCD_COM_SEG)
/* Enable Segment LCD */
#define DENCB_LCD_EN                 (* (reg8 *) DENCB__LCD_EN)
/* Slew Rate Control */
#define DENCB_SLW                    (* (reg8 *) DENCB__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define DENCB_PRTDSI__CAPS_SEL       (* (reg8 *) DENCB__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define DENCB_PRTDSI__DBL_SYNC_IN    (* (reg8 *) DENCB__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define DENCB_PRTDSI__OE_SEL0        (* (reg8 *) DENCB__PRTDSI__OE_SEL0) 
#define DENCB_PRTDSI__OE_SEL1        (* (reg8 *) DENCB__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define DENCB_PRTDSI__OUT_SEL0       (* (reg8 *) DENCB__PRTDSI__OUT_SEL0) 
#define DENCB_PRTDSI__OUT_SEL1       (* (reg8 *) DENCB__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define DENCB_PRTDSI__SYNC_OUT       (* (reg8 *) DENCB__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(DENCB__SIO_CFG)
    #define DENCB_SIO_HYST_EN        (* (reg8 *) DENCB__SIO_HYST_EN)
    #define DENCB_SIO_REG_HIFREQ     (* (reg8 *) DENCB__SIO_REG_HIFREQ)
    #define DENCB_SIO_CFG            (* (reg8 *) DENCB__SIO_CFG)
    #define DENCB_SIO_DIFF           (* (reg8 *) DENCB__SIO_DIFF)
#endif /* (DENCB__SIO_CFG) */

/* Interrupt Registers */
#if defined(DENCB__INTSTAT)
    #define DENCB_INTSTAT            (* (reg8 *) DENCB__INTSTAT)
    #define DENCB_SNAP               (* (reg8 *) DENCB__SNAP)
    
	#define DENCB_0_INTTYPE_REG 		(* (reg8 *) DENCB__0__INTTYPE)
#endif /* (DENCB__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_DENCB_H */


/* [] END OF FILE */
