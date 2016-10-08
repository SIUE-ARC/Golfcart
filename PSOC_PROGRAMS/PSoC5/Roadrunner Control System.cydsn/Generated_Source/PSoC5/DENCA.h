/*******************************************************************************
* File Name: DENCA.h  
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

#if !defined(CY_PINS_DENCA_H) /* Pins DENCA_H */
#define CY_PINS_DENCA_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "DENCA_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 DENCA__PORT == 15 && ((DENCA__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    DENCA_Write(uint8 value);
void    DENCA_SetDriveMode(uint8 mode);
uint8   DENCA_ReadDataReg(void);
uint8   DENCA_Read(void);
void    DENCA_SetInterruptMode(uint16 position, uint16 mode);
uint8   DENCA_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the DENCA_SetDriveMode() function.
     *  @{
     */
        #define DENCA_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define DENCA_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define DENCA_DM_RES_UP          PIN_DM_RES_UP
        #define DENCA_DM_RES_DWN         PIN_DM_RES_DWN
        #define DENCA_DM_OD_LO           PIN_DM_OD_LO
        #define DENCA_DM_OD_HI           PIN_DM_OD_HI
        #define DENCA_DM_STRONG          PIN_DM_STRONG
        #define DENCA_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define DENCA_MASK               DENCA__MASK
#define DENCA_SHIFT              DENCA__SHIFT
#define DENCA_WIDTH              1u

/* Interrupt constants */
#if defined(DENCA__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in DENCA_SetInterruptMode() function.
     *  @{
     */
        #define DENCA_INTR_NONE      (uint16)(0x0000u)
        #define DENCA_INTR_RISING    (uint16)(0x0001u)
        #define DENCA_INTR_FALLING   (uint16)(0x0002u)
        #define DENCA_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define DENCA_INTR_MASK      (0x01u) 
#endif /* (DENCA__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define DENCA_PS                     (* (reg8 *) DENCA__PS)
/* Data Register */
#define DENCA_DR                     (* (reg8 *) DENCA__DR)
/* Port Number */
#define DENCA_PRT_NUM                (* (reg8 *) DENCA__PRT) 
/* Connect to Analog Globals */                                                  
#define DENCA_AG                     (* (reg8 *) DENCA__AG)                       
/* Analog MUX bux enable */
#define DENCA_AMUX                   (* (reg8 *) DENCA__AMUX) 
/* Bidirectional Enable */                                                        
#define DENCA_BIE                    (* (reg8 *) DENCA__BIE)
/* Bit-mask for Aliased Register Access */
#define DENCA_BIT_MASK               (* (reg8 *) DENCA__BIT_MASK)
/* Bypass Enable */
#define DENCA_BYP                    (* (reg8 *) DENCA__BYP)
/* Port wide control signals */                                                   
#define DENCA_CTL                    (* (reg8 *) DENCA__CTL)
/* Drive Modes */
#define DENCA_DM0                    (* (reg8 *) DENCA__DM0) 
#define DENCA_DM1                    (* (reg8 *) DENCA__DM1)
#define DENCA_DM2                    (* (reg8 *) DENCA__DM2) 
/* Input Buffer Disable Override */
#define DENCA_INP_DIS                (* (reg8 *) DENCA__INP_DIS)
/* LCD Common or Segment Drive */
#define DENCA_LCD_COM_SEG            (* (reg8 *) DENCA__LCD_COM_SEG)
/* Enable Segment LCD */
#define DENCA_LCD_EN                 (* (reg8 *) DENCA__LCD_EN)
/* Slew Rate Control */
#define DENCA_SLW                    (* (reg8 *) DENCA__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define DENCA_PRTDSI__CAPS_SEL       (* (reg8 *) DENCA__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define DENCA_PRTDSI__DBL_SYNC_IN    (* (reg8 *) DENCA__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define DENCA_PRTDSI__OE_SEL0        (* (reg8 *) DENCA__PRTDSI__OE_SEL0) 
#define DENCA_PRTDSI__OE_SEL1        (* (reg8 *) DENCA__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define DENCA_PRTDSI__OUT_SEL0       (* (reg8 *) DENCA__PRTDSI__OUT_SEL0) 
#define DENCA_PRTDSI__OUT_SEL1       (* (reg8 *) DENCA__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define DENCA_PRTDSI__SYNC_OUT       (* (reg8 *) DENCA__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(DENCA__SIO_CFG)
    #define DENCA_SIO_HYST_EN        (* (reg8 *) DENCA__SIO_HYST_EN)
    #define DENCA_SIO_REG_HIFREQ     (* (reg8 *) DENCA__SIO_REG_HIFREQ)
    #define DENCA_SIO_CFG            (* (reg8 *) DENCA__SIO_CFG)
    #define DENCA_SIO_DIFF           (* (reg8 *) DENCA__SIO_DIFF)
#endif /* (DENCA__SIO_CFG) */

/* Interrupt Registers */
#if defined(DENCA__INTSTAT)
    #define DENCA_INTSTAT            (* (reg8 *) DENCA__INTSTAT)
    #define DENCA_SNAP               (* (reg8 *) DENCA__SNAP)
    
	#define DENCA_0_INTTYPE_REG 		(* (reg8 *) DENCA__0__INTTYPE)
#endif /* (DENCA__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_DENCA_H */


/* [] END OF FILE */
