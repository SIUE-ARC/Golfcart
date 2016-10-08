/*******************************************************************************
* File Name: DVDACOUT.h  
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

#if !defined(CY_PINS_DVDACOUT_H) /* Pins DVDACOUT_H */
#define CY_PINS_DVDACOUT_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "DVDACOUT_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 DVDACOUT__PORT == 15 && ((DVDACOUT__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    DVDACOUT_Write(uint8 value);
void    DVDACOUT_SetDriveMode(uint8 mode);
uint8   DVDACOUT_ReadDataReg(void);
uint8   DVDACOUT_Read(void);
void    DVDACOUT_SetInterruptMode(uint16 position, uint16 mode);
uint8   DVDACOUT_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the DVDACOUT_SetDriveMode() function.
     *  @{
     */
        #define DVDACOUT_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define DVDACOUT_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define DVDACOUT_DM_RES_UP          PIN_DM_RES_UP
        #define DVDACOUT_DM_RES_DWN         PIN_DM_RES_DWN
        #define DVDACOUT_DM_OD_LO           PIN_DM_OD_LO
        #define DVDACOUT_DM_OD_HI           PIN_DM_OD_HI
        #define DVDACOUT_DM_STRONG          PIN_DM_STRONG
        #define DVDACOUT_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define DVDACOUT_MASK               DVDACOUT__MASK
#define DVDACOUT_SHIFT              DVDACOUT__SHIFT
#define DVDACOUT_WIDTH              1u

/* Interrupt constants */
#if defined(DVDACOUT__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in DVDACOUT_SetInterruptMode() function.
     *  @{
     */
        #define DVDACOUT_INTR_NONE      (uint16)(0x0000u)
        #define DVDACOUT_INTR_RISING    (uint16)(0x0001u)
        #define DVDACOUT_INTR_FALLING   (uint16)(0x0002u)
        #define DVDACOUT_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define DVDACOUT_INTR_MASK      (0x01u) 
#endif /* (DVDACOUT__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define DVDACOUT_PS                     (* (reg8 *) DVDACOUT__PS)
/* Data Register */
#define DVDACOUT_DR                     (* (reg8 *) DVDACOUT__DR)
/* Port Number */
#define DVDACOUT_PRT_NUM                (* (reg8 *) DVDACOUT__PRT) 
/* Connect to Analog Globals */                                                  
#define DVDACOUT_AG                     (* (reg8 *) DVDACOUT__AG)                       
/* Analog MUX bux enable */
#define DVDACOUT_AMUX                   (* (reg8 *) DVDACOUT__AMUX) 
/* Bidirectional Enable */                                                        
#define DVDACOUT_BIE                    (* (reg8 *) DVDACOUT__BIE)
/* Bit-mask for Aliased Register Access */
#define DVDACOUT_BIT_MASK               (* (reg8 *) DVDACOUT__BIT_MASK)
/* Bypass Enable */
#define DVDACOUT_BYP                    (* (reg8 *) DVDACOUT__BYP)
/* Port wide control signals */                                                   
#define DVDACOUT_CTL                    (* (reg8 *) DVDACOUT__CTL)
/* Drive Modes */
#define DVDACOUT_DM0                    (* (reg8 *) DVDACOUT__DM0) 
#define DVDACOUT_DM1                    (* (reg8 *) DVDACOUT__DM1)
#define DVDACOUT_DM2                    (* (reg8 *) DVDACOUT__DM2) 
/* Input Buffer Disable Override */
#define DVDACOUT_INP_DIS                (* (reg8 *) DVDACOUT__INP_DIS)
/* LCD Common or Segment Drive */
#define DVDACOUT_LCD_COM_SEG            (* (reg8 *) DVDACOUT__LCD_COM_SEG)
/* Enable Segment LCD */
#define DVDACOUT_LCD_EN                 (* (reg8 *) DVDACOUT__LCD_EN)
/* Slew Rate Control */
#define DVDACOUT_SLW                    (* (reg8 *) DVDACOUT__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define DVDACOUT_PRTDSI__CAPS_SEL       (* (reg8 *) DVDACOUT__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define DVDACOUT_PRTDSI__DBL_SYNC_IN    (* (reg8 *) DVDACOUT__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define DVDACOUT_PRTDSI__OE_SEL0        (* (reg8 *) DVDACOUT__PRTDSI__OE_SEL0) 
#define DVDACOUT_PRTDSI__OE_SEL1        (* (reg8 *) DVDACOUT__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define DVDACOUT_PRTDSI__OUT_SEL0       (* (reg8 *) DVDACOUT__PRTDSI__OUT_SEL0) 
#define DVDACOUT_PRTDSI__OUT_SEL1       (* (reg8 *) DVDACOUT__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define DVDACOUT_PRTDSI__SYNC_OUT       (* (reg8 *) DVDACOUT__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(DVDACOUT__SIO_CFG)
    #define DVDACOUT_SIO_HYST_EN        (* (reg8 *) DVDACOUT__SIO_HYST_EN)
    #define DVDACOUT_SIO_REG_HIFREQ     (* (reg8 *) DVDACOUT__SIO_REG_HIFREQ)
    #define DVDACOUT_SIO_CFG            (* (reg8 *) DVDACOUT__SIO_CFG)
    #define DVDACOUT_SIO_DIFF           (* (reg8 *) DVDACOUT__SIO_DIFF)
#endif /* (DVDACOUT__SIO_CFG) */

/* Interrupt Registers */
#if defined(DVDACOUT__INTSTAT)
    #define DVDACOUT_INTSTAT            (* (reg8 *) DVDACOUT__INTSTAT)
    #define DVDACOUT_SNAP               (* (reg8 *) DVDACOUT__SNAP)
    
	#define DVDACOUT_0_INTTYPE_REG 		(* (reg8 *) DVDACOUT__0__INTTYPE)
#endif /* (DVDACOUT__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_DVDACOUT_H */


/* [] END OF FILE */
