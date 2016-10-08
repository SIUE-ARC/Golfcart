/*******************************************************************************
* File Name: SENCA.h  
* Version 2.20
*
* Description:
*  This file contains the Alias definitions for Per-Pin APIs in cypins.h. 
*  Information on using these APIs can be found in the System Reference Guide.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_SENCA_ALIASES_H) /* Pins SENCA_ALIASES_H */
#define CY_PINS_SENCA_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"


/***************************************
*              Constants        
***************************************/
#define SENCA_0			(SENCA__0__PC)
#define SENCA_0_INTR	((uint16)((uint16)0x0001u << SENCA__0__SHIFT))

#define SENCA_INTR_ALL	 ((uint16)(SENCA_0_INTR))

#endif /* End Pins SENCA_ALIASES_H */


/* [] END OF FILE */
