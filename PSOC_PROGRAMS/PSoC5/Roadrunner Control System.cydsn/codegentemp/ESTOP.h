/*******************************************************************************
* File Name: ESTOP.h  
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

#if !defined(CY_PINS_ESTOP_H) /* Pins ESTOP_H */
#define CY_PINS_ESTOP_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "ESTOP_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 ESTOP__PORT == 15 && ((ESTOP__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    ESTOP_Write(uint8 value);
void    ESTOP_SetDriveMode(uint8 mode);
uint8   ESTOP_ReadDataReg(void);
uint8   ESTOP_Read(void);
void    ESTOP_SetInterruptMode(uint16 position, uint16 mode);
uint8   ESTOP_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the ESTOP_SetDriveMode() function.
     *  @{
     */
        #define ESTOP_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define ESTOP_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define ESTOP_DM_RES_UP          PIN_DM_RES_UP
        #define ESTOP_DM_RES_DWN         PIN_DM_RES_DWN
        #define ESTOP_DM_OD_LO           PIN_DM_OD_LO
        #define ESTOP_DM_OD_HI           PIN_DM_OD_HI
        #define ESTOP_DM_STRONG          PIN_DM_STRONG
        #define ESTOP_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define ESTOP_MASK               ESTOP__MASK
#define ESTOP_SHIFT              ESTOP__SHIFT
#define ESTOP_WIDTH              1u

/* Interrupt constants */
#if defined(ESTOP__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in ESTOP_SetInterruptMode() function.
     *  @{
     */
        #define ESTOP_INTR_NONE      (uint16)(0x0000u)
        #define ESTOP_INTR_RISING    (uint16)(0x0001u)
        #define ESTOP_INTR_FALLING   (uint16)(0x0002u)
        #define ESTOP_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define ESTOP_INTR_MASK      (0x01u) 
#endif /* (ESTOP__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define ESTOP_PS                     (* (reg8 *) ESTOP__PS)
/* Data Register */
#define ESTOP_DR                     (* (reg8 *) ESTOP__DR)
/* Port Number */
#define ESTOP_PRT_NUM                (* (reg8 *) ESTOP__PRT) 
/* Connect to Analog Globals */                                                  
#define ESTOP_AG                     (* (reg8 *) ESTOP__AG)                       
/* Analog MUX bux enable */
#define ESTOP_AMUX                   (* (reg8 *) ESTOP__AMUX) 
/* Bidirectional Enable */                                                        
#define ESTOP_BIE                    (* (reg8 *) ESTOP__BIE)
/* Bit-mask for Aliased Register Access */
#define ESTOP_BIT_MASK               (* (reg8 *) ESTOP__BIT_MASK)
/* Bypass Enable */
#define ESTOP_BYP                    (* (reg8 *) ESTOP__BYP)
/* Port wide control signals */                                                   
#define ESTOP_CTL                    (* (reg8 *) ESTOP__CTL)
/* Drive Modes */
#define ESTOP_DM0                    (* (reg8 *) ESTOP__DM0) 
#define ESTOP_DM1                    (* (reg8 *) ESTOP__DM1)
#define ESTOP_DM2                    (* (reg8 *) ESTOP__DM2) 
/* Input Buffer Disable Override */
#define ESTOP_INP_DIS                (* (reg8 *) ESTOP__INP_DIS)
/* LCD Common or Segment Drive */
#define ESTOP_LCD_COM_SEG            (* (reg8 *) ESTOP__LCD_COM_SEG)
/* Enable Segment LCD */
#define ESTOP_LCD_EN                 (* (reg8 *) ESTOP__LCD_EN)
/* Slew Rate Control */
#define ESTOP_SLW                    (* (reg8 *) ESTOP__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define ESTOP_PRTDSI__CAPS_SEL       (* (reg8 *) ESTOP__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define ESTOP_PRTDSI__DBL_SYNC_IN    (* (reg8 *) ESTOP__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define ESTOP_PRTDSI__OE_SEL0        (* (reg8 *) ESTOP__PRTDSI__OE_SEL0) 
#define ESTOP_PRTDSI__OE_SEL1        (* (reg8 *) ESTOP__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define ESTOP_PRTDSI__OUT_SEL0       (* (reg8 *) ESTOP__PRTDSI__OUT_SEL0) 
#define ESTOP_PRTDSI__OUT_SEL1       (* (reg8 *) ESTOP__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define ESTOP_PRTDSI__SYNC_OUT       (* (reg8 *) ESTOP__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(ESTOP__SIO_CFG)
    #define ESTOP_SIO_HYST_EN        (* (reg8 *) ESTOP__SIO_HYST_EN)
    #define ESTOP_SIO_REG_HIFREQ     (* (reg8 *) ESTOP__SIO_REG_HIFREQ)
    #define ESTOP_SIO_CFG            (* (reg8 *) ESTOP__SIO_CFG)
    #define ESTOP_SIO_DIFF           (* (reg8 *) ESTOP__SIO_DIFF)
#endif /* (ESTOP__SIO_CFG) */

/* Interrupt Registers */
#if defined(ESTOP__INTSTAT)
    #define ESTOP_INTSTAT            (* (reg8 *) ESTOP__INTSTAT)
    #define ESTOP_SNAP               (* (reg8 *) ESTOP__SNAP)
    
	#define ESTOP_0_INTTYPE_REG 		(* (reg8 *) ESTOP__0__INTTYPE)
#endif /* (ESTOP__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_ESTOP_H */


/* [] END OF FILE */
