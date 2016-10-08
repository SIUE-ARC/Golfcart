/*******************************************************************************
* File Name: QuadDecSteer.c  
* Version 3.0
*
* Description:
*  This file provides the source code to the API for the Quadrature Decoder
*  component.
*
* Note:
*  None.
*   
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "QuadDecSteer.h"

#if (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT)
    #include "QuadDecSteer_PVT.h"
#endif /* QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT */

uint8 QuadDecSteer_initVar = 0u;


/*******************************************************************************
* Function Name: QuadDecSteer_Init
********************************************************************************
*
* Summary:
*  Inits/Restores default QuadDec configuration provided with customizer.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void QuadDecSteer_Init(void) 
{
    #if (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT)
        /* Disable Interrupt. */
        CyIntDisable(QuadDecSteer_ISR_NUMBER);
        /* Set the ISR to point to the QuadDecSteer_isr Interrupt. */
        (void) CyIntSetVector(QuadDecSteer_ISR_NUMBER, & QuadDecSteer_ISR);
        /* Set the priority. */
        CyIntSetPriority(QuadDecSteer_ISR_NUMBER, QuadDecSteer_ISR_PRIORITY);
    #endif /* QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT */
}


/*******************************************************************************
* Function Name: QuadDecSteer_Enable
********************************************************************************
*
* Summary:
*  This function enable interrupts from Component and also enable Component's
*  ISR in case of 32-bit counter.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void QuadDecSteer_Enable(void) 
{
    uint8 enableInterrupts;

    QuadDecSteer_SetInterruptMask(QuadDecSteer_INIT_INT_MASK);

    /* Clear pending interrupts. */
    (void) QuadDecSteer_GetEvents();
    
    enableInterrupts = CyEnterCriticalSection();

    /* Enable interrupts from Statusi register */
    QuadDecSteer_SR_AUX_CONTROL |= QuadDecSteer_INTERRUPTS_ENABLE;

    CyExitCriticalSection(enableInterrupts);        

    #if (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT)
        /* Enable Component interrupts */
        CyIntEnable(QuadDecSteer_ISR_NUMBER);
    #endif /* QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT */
}


/*******************************************************************************
* Function Name: QuadDecSteer_Start
********************************************************************************
*
* Summary:
*  Initializes UDBs and other relevant hardware.
*  Resets counter, enables or disables all relevant interrupts.
*  Starts monitoring the inputs and counting.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  QuadDecSteer_initVar - used to check initial configuration, modified on
*  first function call.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void QuadDecSteer_Start(void) 
{
    #if (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_8_BIT)
        QuadDecSteer_Cnt8_Start();
        QuadDecSteer_Cnt8_WriteCounter(QuadDecSteer_COUNTER_INIT_VALUE);
    #else
        /* (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_16_BIT) || 
        *  (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT) 
        */
        QuadDecSteer_Cnt16_Start();
        QuadDecSteer_Cnt16_WriteCounter(QuadDecSteer_COUNTER_INIT_VALUE);
    #endif /* QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_8_BIT */
    
    #if (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT)        
       QuadDecSteer_count32SoftPart = 0;
    #endif /* QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT */

    if (QuadDecSteer_initVar == 0u)
    {
        QuadDecSteer_Init();
        QuadDecSteer_initVar = 1u;
    }

    QuadDecSteer_Enable();
}


/*******************************************************************************
* Function Name: QuadDecSteer_Stop
********************************************************************************
*
* Summary:
*  Turns off UDBs and other relevant hardware.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void QuadDecSteer_Stop(void) 
{
    uint8 enableInterrupts;

    #if (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_8_BIT)
        QuadDecSteer_Cnt8_Stop();
    #else 
        /* (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_16_BIT) ||
        *  (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT)
        */
        QuadDecSteer_Cnt16_Stop();    /* counter disable */
    #endif /* (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_8_BIT) */
 
    enableInterrupts = CyEnterCriticalSection();

    /* Disable interrupts interrupts from Statusi register */
    QuadDecSteer_SR_AUX_CONTROL &= (uint8) (~QuadDecSteer_INTERRUPTS_ENABLE);

    CyExitCriticalSection(enableInterrupts);

    #if (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT)
        CyIntDisable(QuadDecSteer_ISR_NUMBER);    /* interrupt disable */
    #endif /* QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT */
}


/*******************************************************************************
* Function Name: QuadDecSteer_GetCounter
********************************************************************************
*
* Summary:
*  Reports the current value of the counter.
*
* Parameters:
*  None.
*
* Return:
*  The counter value. Return type is signed and per the counter size setting.
*  A positive value indicates clockwise movement (B before A).
*
* Global variables:
*  QuadDecSteer_count32SoftPart - used to get hi 16 bit for current value
*  of the 32-bit counter, when Counter size equal 32-bit.
*
*******************************************************************************/
int16 QuadDecSteer_GetCounter(void) 
{
    int16 count;
    uint16 tmpCnt;

    #if (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT)
        int16 hwCount;

        CyIntDisable(QuadDecSteer_ISR_NUMBER);

        tmpCnt = QuadDecSteer_Cnt16_ReadCounter();
        hwCount = (int16) ((int32) tmpCnt - (int32) QuadDecSteer_COUNTER_INIT_VALUE);
        count = QuadDecSteer_count32SoftPart + hwCount;

        CyIntEnable(QuadDecSteer_ISR_NUMBER);
    #else 
        /* (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_8_BIT) || 
        *  (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_16_BIT)
        */
        #if (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_8_BIT)
            tmpCnt = QuadDecSteer_Cnt8_ReadCounter();
        #else /* (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_16_BIT) */
            tmpCnt = QuadDecSteer_Cnt16_ReadCounter();
        #endif  /* QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_8_BIT */

        count = (int16) ((int32) tmpCnt -
                (int32) QuadDecSteer_COUNTER_INIT_VALUE);

    #endif /* QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT */ 

    return (count);
}


/*******************************************************************************
* Function Name: QuadDecSteer_SetCounter
********************************************************************************
*
* Summary:
*  Sets the current value of the counter.
*
* Parameters:
*  value:  The new value. Parameter type is signed and per the counter size
*  setting.
*
* Return:
*  None.
*
* Global variables:
*  QuadDecSteer_count32SoftPart - modified to set hi 16 bit for current
*  value of the 32-bit counter, when Counter size equal 32-bit.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void QuadDecSteer_SetCounter(int16 value) 
{
    #if ((QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_8_BIT) || \
         (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_16_BIT))
        uint16 count;
        
        if (value >= 0)
        {
            count = (uint16) value + QuadDecSteer_COUNTER_INIT_VALUE;
        }
        else
        {
            count = QuadDecSteer_COUNTER_INIT_VALUE - (uint16)(-value);
        }
        #if (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_8_BIT)
            QuadDecSteer_Cnt8_WriteCounter(count);
        #else /* (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_16_BIT) */
            QuadDecSteer_Cnt16_WriteCounter(count);
        #endif  /* QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_8_BIT */
    #else /* (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT) */
        CyIntDisable(QuadDecSteer_ISR_NUMBER);

        QuadDecSteer_Cnt16_WriteCounter(QuadDecSteer_COUNTER_INIT_VALUE);
        QuadDecSteer_count32SoftPart = value;

        CyIntEnable(QuadDecSteer_ISR_NUMBER);
    #endif  /* (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_8_BIT) ||
             * (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_16_BIT)
             */
}


/*******************************************************************************
* Function Name: QuadDecSteer_GetEvents
********************************************************************************
* 
* Summary:
*   Reports the current status of events. This function clears the bits of the 
*   status register.
*
* Parameters:
*  None.
*
* Return:
*  The events, as bits in an unsigned 8-bit value:
*    Bit      Description
*     0        Counter overflow.
*     1        Counter underflow.
*     2        Counter reset due to index, if index input is used.
*     3        Invalid A, B inputs state transition.
*
*******************************************************************************/
uint8 QuadDecSteer_GetEvents(void) 
{
    return (QuadDecSteer_STATUS_REG & QuadDecSteer_INIT_INT_MASK);
}


/*******************************************************************************
* Function Name: QuadDecSteer_SetInterruptMask
********************************************************************************
*
* Summary:
*  Enables / disables interrupts due to the events.
*  For the 32-bit counter, the overflow, underflow and reset interrupts cannot
*  be disabled, these bits are ignored.
*
* Parameters:
*  mask: Enable / disable bits in an 8-bit value, where 1 enables the interrupt.
*
* Return:
*  None.
*
*******************************************************************************/
void QuadDecSteer_SetInterruptMask(uint8 mask) 
{
    #if (QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT)
        /* Underflow, Overflow and Reset interrupts for 32-bit Counter are always enable */
        mask |= (QuadDecSteer_COUNTER_OVERFLOW | QuadDecSteer_COUNTER_UNDERFLOW |
                 QuadDecSteer_COUNTER_RESET);
    #endif /* QuadDecSteer_COUNTER_SIZE == QuadDecSteer_COUNTER_SIZE_32_BIT */

    QuadDecSteer_STATUS_MASK = mask;
}


/*******************************************************************************
* Function Name: QuadDecSteer_GetInterruptMask
********************************************************************************
*
* Summary:
*  Reports the current interrupt mask settings.
*
* Parameters:
*  None.
*
* Return:
*  Enable / disable bits in an 8-bit value, where 1 enables the interrupt.
*  For the 32-bit counter, the overflow, underflow and reset enable bits are
*  always set.
*
*******************************************************************************/
uint8 QuadDecSteer_GetInterruptMask(void) 
{
    return (QuadDecSteer_STATUS_MASK & QuadDecSteer_INIT_INT_MASK);
}


/* [] END OF FILE */
