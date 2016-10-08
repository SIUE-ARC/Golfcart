/***************************************************************************
* File Name: TMPBUFB_dma.c  
* Version 1.70
*
*  Description:
*   Provides an API for the DMAC component. The API includes functions
*   for the DMA controller, DMA channels and Transfer Descriptors.
*
*
*   Note:
*     This module requires the developer to finish or fill in the auto
*     generated funcions and setup the dma channel and TD's.
*
********************************************************************************
* Copyright 2008-2010, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/
#include <CYLIB.H>
#include <CYDMAC.H>
#include <TMPBUFB_dma.H>



/****************************************************************************
* 
* The following defines are available in Cyfitter.h
* 
* 
* 
* TMPBUFB__DRQ_CTL_REG
* 
* 
* TMPBUFB__DRQ_NUMBER
* 
* Number of TD's used by this channel.
* TMPBUFB__NUMBEROF_TDS
* 
* Priority of this channel.
* TMPBUFB__PRIORITY
* 
* True if TMPBUFB_TERMIN_SEL is used.
* TMPBUFB__TERMIN_EN
* 
* TERMIN interrupt line to signal terminate.
* TMPBUFB__TERMIN_SEL
* 
* 
* True if TMPBUFB_TERMOUT0_SEL is used.
* TMPBUFB__TERMOUT0_EN
* 
* 
* TERMOUT0 interrupt line to signal completion.
* TMPBUFB__TERMOUT0_SEL
* 
* 
* True if TMPBUFB_TERMOUT1_SEL is used.
* TMPBUFB__TERMOUT1_EN
* 
* 
* TERMOUT1 interrupt line to signal completion.
* TMPBUFB__TERMOUT1_SEL
* 
****************************************************************************/


/* Zero based index of TMPBUFB dma channel */
uint8 TMPBUFB_DmaHandle = DMA_INVALID_CHANNEL;

/*********************************************************************
* Function Name: uint8 TMPBUFB_DmaInitalize
**********************************************************************
* Summary:
*   Allocates and initialises a channel of the DMAC to be used by the
*   caller.
*
* Parameters:
*   BurstCount.
*       
*       
*   ReqestPerBurst.
*       
*       
*   UpperSrcAddress.
*       
*       
*   UpperDestAddress.
*       
*
* Return:
*   The channel that can be used by the caller for DMA activity.
*   DMA_INVALID_CHANNEL (0xFF) if there are no channels left. 
*
*
*******************************************************************/
uint8 TMPBUFB_DmaInitialize(uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) 
{

    /* Allocate a DMA channel. */
    TMPBUFB_DmaHandle = (uint8)TMPBUFB__DRQ_NUMBER;

    /* Configure the channel. */
    (void)CyDmaChSetConfiguration(TMPBUFB_DmaHandle,
                                  BurstCount,
                                  ReqestPerBurst,
                                  (uint8)TMPBUFB__TERMOUT0_SEL,
                                  (uint8)TMPBUFB__TERMOUT1_SEL,
                                  (uint8)TMPBUFB__TERMIN_SEL);

    /* Set the extended address for the transfers */
    (void)CyDmaChSetExtendedAddress(TMPBUFB_DmaHandle, UpperSrcAddress, UpperDestAddress);

    /* Set the priority for this channel */
    (void)CyDmaChPriority(TMPBUFB_DmaHandle, (uint8)TMPBUFB__PRIORITY);
    
    return TMPBUFB_DmaHandle;
}

/*********************************************************************
* Function Name: void TMPBUFB_DmaRelease
**********************************************************************
* Summary:
*   Frees the channel associated with TMPBUFB.
*
*
* Parameters:
*   void.
*
*
*
* Return:
*   void.
*
*******************************************************************/
void TMPBUFB_DmaRelease(void) 
{
    /* Disable the channel */
    (void)CyDmaChDisable(TMPBUFB_DmaHandle);
}

