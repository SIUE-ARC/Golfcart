/*******************************************************************************
* File Name: SENCA.h  
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

#if !defined(CY_PINS_SENCA_H) /* Pins SENCA_H */
#define CY_PINS_SENCA_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "SENCA_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 SENCA__PORT == 15 && ((SENCA__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    SENCA_Write(uint8 value);
void    SENCA_SetDriveMode(uint8 mode);
uint8   SENCA_ReadDataReg(void);
uint8   SENCA_Read(void);
void    SENCA_SetInterruptMode(uint16 position, uint16 mode);
uint8   SENCA_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the SENCA_SetDriveMode() function.
     *  @{
     */
        #define SENCA_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define SENCA_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define SENCA_DM_RES_UP          PIN_DM_RES_UP
        #define SENCA_DM_RES_DWN         PIN_DM_RES_DWN
        #define SENCA_DM_OD_LO           PIN_DM_OD_LO
        #define SENCA_DM_OD_HI           PIN_DM_OD_HI
        #define SENCA_DM_STRONG          PIN_DM_STRONG
        #define SENCA_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define SENCA_MASK               SENCA__MASK
#define SENCA_SHIFT              SENCA__SHIFT
#define SENCA_WIDTH              1u

/* Interrupt constants */
#if defined(SENCA__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in SENCA_SetInterruptMode() function.
     *  @{
     */
        #define SENCA_INTR_NONE      (uint16)(0x0000u)
        #define SENCA_INTR_RISING    (uint16)(0x0001u)
        #define SENCA_INTR_FALLING   (uint16)(0x0002u)
        #define SENCA_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define SENCA_INTR_MASK      (0x01u) 
#endif /* (SENCA__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define SENCA_PS                     (* (reg8 *) SENCA__PS)
/* Data Register */
#define SENCA_DR                     (* (reg8 *) SENCA__DR)
/* Port Number */
#define SENCA_PRT_NUM                (* (reg8 *) SENCA__PRT) 
/* Connect to Analog Globals */                                                  
#define SENCA_AG                     (* (reg8 *) SENCA__AG)                       
/* Analog MUX bux enable */
#define SENCA_AMUX                   (* (reg8 *) SENCA__AMUX) 
/* Bidirectional Enable */                                                        
#define SENCA_BIE                    (* (reg8 *) SENCA__BIE)
/* Bit-mask for Aliased Register Access */
#define SENCA_BIT_MASK               (* (reg8 *) SENCA__BIT_MASK)
/* Bypass Enable */
#define SENCA_BYP                    (* (reg8 *) SENCA__BYP)
/* Port wide control signals */                                                   
#define SENCA_CTL                    (* (reg8 *) SENCA__CTL)
/* Drive Modes */
#define SENCA_DM0                    (* (reg8 *) SENCA__DM0) 
#define SENCA_DM1                    (* (reg8 *) SENCA__DM1)
#define SENCA_DM2                    (* (reg8 *) SENCA__DM2) 
/* Input Buffer Disable Override */
#define SENCA_INP_DIS                (* (reg8 *) SENCA__INP_DIS)
/* LCD Common or Segment Drive */
#define SENCA_LCD_COM_SEG            (* (reg8 *) SENCA__LCD_COM_SEG)
/* Enable Segment LCD */
#define SENCA_LCD_EN                 (* (reg8 *) SENCA__LCD_EN)
/* Slew Rate Control */
#define SENCA_SLW                    (* (reg8 *) SENCA__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define SENCA_PRTDSI__CAPS_SEL       (* (reg8 *) SENCA__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define SENCA_PRTDSI__DBL_SYNC_IN    (* (reg8 *) SENCA__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define SENCA_PRTDSI__OE_SEL0        (* (reg8 *) SENCA__PRTDSI__OE_SEL0) 
#define SENCA_PRTDSI__OE_SEL1        (* (reg8 *) SENCA__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define SENCA_PRTDSI__OUT_SEL0       (* (reg8 *) SENCA__PRTDSI__OUT_SEL0) 
#define SENCA_PRTDSI__OUT_SEL1       (* (reg8 *) SENCA__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define SENCA_PRTDSI__SYNC_OUT       (* (reg8 *) SENCA__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(SENCA__SIO_CFG)
    #define SENCA_SIO_HYST_EN        (* (reg8 *) SENCA__SIO_HYST_EN)
    #define SENCA_SIO_REG_HIFREQ     (* (reg8 *) SENCA__SIO_REG_HIFREQ)
    #define SENCA_SIO_CFG            (* (reg8 *) SENCA__SIO_CFG)
    #define SENCA_SIO_DIFF           (* (reg8 *) SENCA__SIO_DIFF)
#endif /* (SENCA__SIO_CFG) */

/* Interrupt Registers */
#if defined(SENCA__INTSTAT)
    #define SENCA_INTSTAT            (* (reg8 *) SENCA__INTSTAT)
    #define SENCA_SNAP               (* (reg8 *) SENCA__SNAP)
    
	#define SENCA_0_INTTYPE_REG 		(* (reg8 *) SENCA__0__INTTYPE)
#endif /* (SENCA__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_SENCA_H */


/* [] END OF FILE */
