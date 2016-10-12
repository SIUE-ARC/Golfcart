/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include <project.h>

/* Defines for RAMBUF */
#define RAMBUF_BYTES_PER_BURST      2
#define RAMBUF_REQUEST_PER_BURST    1
#define RAMBUF_SRC_BASE             (CYDEV_PERIPH_BASE)
#define RAMBUF_DST_BASE             (CYDEV_SRAM_BASE)
#define USBFS_DEVICE                (0u)
#define IN_EP_NUM                   (1u)
#define OUT_EP_NUM                  (2u)
#define BUFFER_SIZE                 (64u)

typedef uint8   byte;
typedef uint16  hword;
typedef uint32  word;

byte buffer[BUFFER_SIZE];

unsigned int adc_samp[2]   =    {0, 0};
void init();

int main()
{
    /* Variable declarations for USBFS */
    hword length = 4;
    
    CyGlobalIntEnable; /* Enable global interrupts. */
    init();
    /* Place your initialization/startup code here (e.g. MyInst_Start()) */

    USBFS_EnableOutEP(OUT_EP_NUM);
    for(;;)
    {
        buffer[0] = LO8(adc_samp[0]);
        buffer[1] = HI8(adc_samp[0]);
        buffer[2] = LO8(adc_samp[1]);
        buffer[3] = HI8(adc_samp[1]);
        
        //Wait for IN buffer to become empty (host has read data)
        while(USBFS_IN_BUFFER_EMPTY != USBFS_GetEPAckState(IN_EP_NUM));
        
        
        /* Trigger DMA to copy data into IN endpoint buffer.
        * After data has been copied, IN endpoint is ready to be read by the
        * host.
        */
        #if (USBFS_16BITS_EP_ACCESS_ENABLE)
            USBFS_LoadInEP16(IN_EP_NUM, buffer, length);
        #else
            USBFS_LoadInEP(IN_EP_NUM, buffer, length);
        #endif /* (USBFS_GEN_16BITS_EP_ACCESS) */
    }
}

void init()
{
    
    /* Variable declarations for RAMBUF */
    /* Move these variable declarations to the top of the function */
    byte RAMBUF_Chan;
    byte RAMBUF_TD[1];

    /* DMA Configuration for RAMBUF */
    RAMBUF_Chan = RAMBUF_DmaInitialize(RAMBUF_BYTES_PER_BURST, RAMBUF_REQUEST_PER_BURST, 
        HI16(RAMBUF_SRC_BASE), HI16(RAMBUF_DST_BASE));
    RAMBUF_TD[0] = CyDmaTdAllocate();
    CyDmaTdSetConfiguration(RAMBUF_TD[0], 2, RAMBUF_TD[0], TD_INC_DST_ADR);
    CyDmaTdSetAddress(RAMBUF_TD[0], LO16((uint32)POTADC_SAR_WRK0_PTR), LO16((uint32)adc_samp[0]));
    CyDmaChSetInitialTd(RAMBUF_Chan, RAMBUF_TD[0]);
    CyDmaChEnable(RAMBUF_Chan, 1);
    
    USBFS_Start(USBFS_DEVICE, USBFS_5V_OPERATION);
    while(0u == USBFS_GetConfiguration()); //wait until we are enumerated by host
}
/* [] END OF FILE */
