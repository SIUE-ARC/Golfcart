/*******************************************************************************
* File Name: RLY_COAST.h  
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

#if !defined(CY_PINS_RLY_COAST_H) /* Pins RLY_COAST_H */
#define CY_PINS_RLY_COAST_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "RLY_COAST_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 RLY_COAST__PORT == 15 && ((RLY_COAST__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    RLY_COAST_Write(uint8 value);
void    RLY_COAST_SetDriveMode(uint8 mode);
uint8   RLY_COAST_ReadDataReg(void);
uint8   RLY_COAST_Read(void);
void    RLY_COAST_SetInterruptMode(uint16 position, uint16 mode);
uint8   RLY_COAST_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the RLY_COAST_SetDriveMode() function.
     *  @{
     */
        #define RLY_COAST_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define RLY_COAST_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define RLY_COAST_DM_RES_UP          PIN_DM_RES_UP
        #define RLY_COAST_DM_RES_DWN         PIN_DM_RES_DWN
        #define RLY_COAST_DM_OD_LO           PIN_DM_OD_LO
        #define RLY_COAST_DM_OD_HI           PIN_DM_OD_HI
        #define RLY_COAST_DM_STRONG          PIN_DM_STRONG
        #define RLY_COAST_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define RLY_COAST_MASK               RLY_COAST__MASK
#define RLY_COAST_SHIFT              RLY_COAST__SHIFT
#define RLY_COAST_WIDTH              1u

/* Interrupt constants */
#if defined(RLY_COAST__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in RLY_COAST_SetInterruptMode() function.
     *  @{
     */
        #define RLY_COAST_INTR_NONE      (uint16)(0x0000u)
        #define RLY_COAST_INTR_RISING    (uint16)(0x0001u)
        #define RLY_COAST_INTR_FALLING   (uint16)(0x0002u)
        #define RLY_COAST_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define RLY_COAST_INTR_MASK      (0x01u) 
#endif /* (RLY_COAST__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define RLY_COAST_PS                     (* (reg8 *) RLY_COAST__PS)
/* Data Register */
#define RLY_COAST_DR                     (* (reg8 *) RLY_COAST__DR)
/* Port Number */
#define RLY_COAST_PRT_NUM                (* (reg8 *) RLY_COAST__PRT) 
/* Connect to Analog Globals */                                                  
#define RLY_COAST_AG                     (* (reg8 *) RLY_COAST__AG)                       
/* Analog MUX bux enable */
#define RLY_COAST_AMUX                   (* (reg8 *) RLY_COAST__AMUX) 
/* Bidirectional Enable */                                                        
#define RLY_COAST_BIE                    (* (reg8 *) RLY_COAST__BIE)
/* Bit-mask for Aliased Register Access */
#define RLY_COAST_BIT_MASK               (* (reg8 *) RLY_COAST__BIT_MASK)
/* Bypass Enable */
#define RLY_COAST_BYP                    (* (reg8 *) RLY_COAST__BYP)
/* Port wide control signals */                                                   
#define RLY_COAST_CTL                    (* (reg8 *) RLY_COAST__CTL)
/* Drive Modes */
#define RLY_COAST_DM0                    (* (reg8 *) RLY_COAST__DM0) 
#define RLY_COAST_DM1                    (* (reg8 *) RLY_COAST__DM1)
#define RLY_COAST_DM2                    (* (reg8 *) RLY_COAST__DM2) 
/* Input Buffer Disable Override */
#define RLY_COAST_INP_DIS                (* (reg8 *) RLY_COAST__INP_DIS)
/* LCD Common or Segment Drive */
#define RLY_COAST_LCD_COM_SEG            (* (reg8 *) RLY_COAST__LCD_COM_SEG)
/* Enable Segment LCD */
#define RLY_COAST_LCD_EN                 (* (reg8 *) RLY_COAST__LCD_EN)
/* Slew Rate Control */
#define RLY_COAST_SLW                    (* (reg8 *) RLY_COAST__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define RLY_COAST_PRTDSI__CAPS_SEL       (* (reg8 *) RLY_COAST__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define RLY_COAST_PRTDSI__DBL_SYNC_IN    (* (reg8 *) RLY_COAST__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define RLY_COAST_PRTDSI__OE_SEL0        (* (reg8 *) RLY_COAST__PRTDSI__OE_SEL0) 
#define RLY_COAST_PRTDSI__OE_SEL1        (* (reg8 *) RLY_COAST__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define RLY_COAST_PRTDSI__OUT_SEL0       (* (reg8 *) RLY_COAST__PRTDSI__OUT_SEL0) 
#define RLY_COAST_PRTDSI__OUT_SEL1       (* (reg8 *) RLY_COAST__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define RLY_COAST_PRTDSI__SYNC_OUT       (* (reg8 *) RLY_COAST__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(RLY_COAST__SIO_CFG)
    #define RLY_COAST_SIO_HYST_EN        (* (reg8 *) RLY_COAST__SIO_HYST_EN)
    #define RLY_COAST_SIO_REG_HIFREQ     (* (reg8 *) RLY_COAST__SIO_REG_HIFREQ)
    #define RLY_COAST_SIO_CFG            (* (reg8 *) RLY_COAST__SIO_CFG)
    #define RLY_COAST_SIO_DIFF           (* (reg8 *) RLY_COAST__SIO_DIFF)
#endif /* (RLY_COAST__SIO_CFG) */

/* Interrupt Registers */
#if defined(RLY_COAST__INTSTAT)
    #define RLY_COAST_INTSTAT            (* (reg8 *) RLY_COAST__INTSTAT)
    #define RLY_COAST_SNAP               (* (reg8 *) RLY_COAST__SNAP)
    
	#define RLY_COAST_0_INTTYPE_REG 		(* (reg8 *) RLY_COAST__0__INTTYPE)
#endif /* (RLY_COAST__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_RLY_COAST_H */


/* [] END OF FILE */
