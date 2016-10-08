/***************************************************************************
* File Name: TMPBUFS_dma.c  
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
#include <TMPBUFS_dma.H>



/****************************************************************************
* 
* The following defines are available in Cyfitter.h
* 
* 
* 
* TMPBUFS__DRQ_CTL_REG
* 
* 
* TMPBUFS__DRQ_NUMBER
* 
* Number of TD's used by this channel.
* TMPBUFS__NUMBEROF_TDS
* 
* Priority of this channel.
* TMPBUFS__PRIORITY
* 
* True if TMPBUFS_TERMIN_SEL is used.
* TMPBUFS__TERMIN_EN
* 
* TERMIN interrupt line to signal terminate.
* TMPBUFS__TERMIN_SEL
* 
* 
* True if TMPBUFS_TERMOUT0_SEL is used.
* TMPBUFS__TERMOUT0_EN
* 
* 
* TERMOUT0 interrupt line to signal completion.
* TMPBUFS__TERMOUT0_SEL
* 
* 
* True if TMPBUFS_TERMOUT1_SEL is used.
* TMPBUFS__TERMOUT1_EN
* 
* 
* TERMOUT1 interrupt line to signal completion.
* TMPBUFS__TERMOUT1_SEL
* 
****************************************************************************/


/* Zero based index of TMPBUFS dma channel */
uint8 TMPBUFS_DmaHandle = DMA_INVALID_CHANNEL;

/*********************************************************************
* Function Name: uint8 TMPBUFS_DmaInitalize
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
uint8 TMPBUFS_DmaInitialize(uint8 BurstCount, uint8 ReqestPerBurst, uint16 UpperSrcAddress, uint16 UpperDestAddress) 
{

    /* Allocate a DMA channel. */
    TMPBUFS_DmaHandle = (uint8)TMPBUFS__DRQ_NUMBER;

    /* Configure the channel. */
    (void)CyDmaChSetConfiguration(TMPBUFS_DmaHandle,
                                  BurstCount,
                                  ReqestPerBurst,
                                  (uint8)TMPBUFS__TERMOUT0_SEL,
                                  (uint8)TMPBUFS__TERMOUT1_SEL,
                                  (uint8)TMPBUFS__TERMIN_SEL);

    /* Set the extended address for the transfers */
    (void)CyDmaChSetExtendedAddress(TMPBUFS_DmaHandle, UpperSrcAddress, UpperDestAddress);

    /* Set the priority for this channel */
    (void)CyDmaChPriority(TMPBUFS_DmaHandle, (uint8)TMPBUFS__PRIORITY);
    
    return TMPBUFS_DmaHandle;
}

/*********************************************************************
* Function Name: void TMPBUFS_DmaRelease
**********************************************************************
* Summary:
*   Frees the channel associated with TMPBUFS.
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
void TMPBUFS_DmaRelease(void) 
{
    /* Disable the channel */
    (void)CyDmaChDisable(TMPBUFS_DmaHandle);
}

