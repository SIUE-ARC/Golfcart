/*******************************************************************************
* File Name: RLY_REVERSE.h  
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

#if !defined(CY_PINS_RLY_REVERSE_H) /* Pins RLY_REVERSE_H */
#define CY_PINS_RLY_REVERSE_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "RLY_REVERSE_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 RLY_REVERSE__PORT == 15 && ((RLY_REVERSE__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    RLY_REVERSE_Write(uint8 value);
void    RLY_REVERSE_SetDriveMode(uint8 mode);
uint8   RLY_REVERSE_ReadDataReg(void);
uint8   RLY_REVERSE_Read(void);
void    RLY_REVERSE_SetInterruptMode(uint16 position, uint16 mode);
uint8   RLY_REVERSE_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the RLY_REVERSE_SetDriveMode() function.
     *  @{
     */
        #define RLY_REVERSE_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define RLY_REVERSE_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define RLY_REVERSE_DM_RES_UP          PIN_DM_RES_UP
        #define RLY_REVERSE_DM_RES_DWN         PIN_DM_RES_DWN
        #define RLY_REVERSE_DM_OD_LO           PIN_DM_OD_LO
        #define RLY_REVERSE_DM_OD_HI           PIN_DM_OD_HI
        #define RLY_REVERSE_DM_STRONG          PIN_DM_STRONG
        #define RLY_REVERSE_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define RLY_REVERSE_MASK               RLY_REVERSE__MASK
#define RLY_REVERSE_SHIFT              RLY_REVERSE__SHIFT
#define RLY_REVERSE_WIDTH              1u

/* Interrupt constants */
#if defined(RLY_REVERSE__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in RLY_REVERSE_SetInterruptMode() function.
     *  @{
     */
        #define RLY_REVERSE_INTR_NONE      (uint16)(0x0000u)
        #define RLY_REVERSE_INTR_RISING    (uint16)(0x0001u)
        #define RLY_REVERSE_INTR_FALLING   (uint16)(0x0002u)
        #define RLY_REVERSE_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define RLY_REVERSE_INTR_MASK      (0x01u) 
#endif /* (RLY_REVERSE__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define RLY_REVERSE_PS                     (* (reg8 *) RLY_REVERSE__PS)
/* Data Register */
#define RLY_REVERSE_DR                     (* (reg8 *) RLY_REVERSE__DR)
/* Port Number */
#define RLY_REVERSE_PRT_NUM                (* (reg8 *) RLY_REVERSE__PRT) 
/* Connect to Analog Globals */                                                  
#define RLY_REVERSE_AG                     (* (reg8 *) RLY_REVERSE__AG)                       
/* Analog MUX bux enable */
#define RLY_REVERSE_AMUX                   (* (reg8 *) RLY_REVERSE__AMUX) 
/* Bidirectional Enable */                                                        
#define RLY_REVERSE_BIE                    (* (reg8 *) RLY_REVERSE__BIE)
/* Bit-mask for Aliased Register Access */
#define RLY_REVERSE_BIT_MASK               (* (reg8 *) RLY_REVERSE__BIT_MASK)
/* Bypass Enable */
#define RLY_REVERSE_BYP                    (* (reg8 *) RLY_REVERSE__BYP)
/* Port wide control signals */                                                   
#define RLY_REVERSE_CTL                    (* (reg8 *) RLY_REVERSE__CTL)
/* Drive Modes */
#define RLY_REVERSE_DM0                    (* (reg8 *) RLY_REVERSE__DM0) 
#define RLY_REVERSE_DM1                    (* (reg8 *) RLY_REVERSE__DM1)
#define RLY_REVERSE_DM2                    (* (reg8 *) RLY_REVERSE__DM2) 
/* Input Buffer Disable Override */
#define RLY_REVERSE_INP_DIS                (* (reg8 *) RLY_REVERSE__INP_DIS)
/* LCD Common or Segment Drive */
#define RLY_REVERSE_LCD_COM_SEG            (* (reg8 *) RLY_REVERSE__LCD_COM_SEG)
/* Enable Segment LCD */
#define RLY_REVERSE_LCD_EN                 (* (reg8 *) RLY_REVERSE__LCD_EN)
/* Slew Rate Control */
#define RLY_REVERSE_SLW                    (* (reg8 *) RLY_REVERSE__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define RLY_REVERSE_PRTDSI__CAPS_SEL       (* (reg8 *) RLY_REVERSE__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define RLY_REVERSE_PRTDSI__DBL_SYNC_IN    (* (reg8 *) RLY_REVERSE__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define RLY_REVERSE_PRTDSI__OE_SEL0        (* (reg8 *) RLY_REVERSE__PRTDSI__OE_SEL0) 
#define RLY_REVERSE_PRTDSI__OE_SEL1        (* (reg8 *) RLY_REVERSE__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define RLY_REVERSE_PRTDSI__OUT_SEL0       (* (reg8 *) RLY_REVERSE__PRTDSI__OUT_SEL0) 
#define RLY_REVERSE_PRTDSI__OUT_SEL1       (* (reg8 *) RLY_REVERSE__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define RLY_REVERSE_PRTDSI__SYNC_OUT       (* (reg8 *) RLY_REVERSE__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(RLY_REVERSE__SIO_CFG)
    #define RLY_REVERSE_SIO_HYST_EN        (* (reg8 *) RLY_REVERSE__SIO_HYST_EN)
    #define RLY_REVERSE_SIO_REG_HIFREQ     (* (reg8 *) RLY_REVERSE__SIO_REG_HIFREQ)
    #define RLY_REVERSE_SIO_CFG            (* (reg8 *) RLY_REVERSE__SIO_CFG)
    #define RLY_REVERSE_SIO_DIFF           (* (reg8 *) RLY_REVERSE__SIO_DIFF)
#endif /* (RLY_REVERSE__SIO_CFG) */

/* Interrupt Registers */
#if defined(RLY_REVERSE__INTSTAT)
    #define RLY_REVERSE_INTSTAT            (* (reg8 *) RLY_REVERSE__INTSTAT)
    #define RLY_REVERSE_SNAP               (* (reg8 *) RLY_REVERSE__SNAP)
    
	#define RLY_REVERSE_0_INTTYPE_REG 		(* (reg8 *) RLY_REVERSE__0__INTTYPE)
#endif /* (RLY_REVERSE__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_RLY_REVERSE_H */


/* [] END OF FILE */
