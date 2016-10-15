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

/* Defines for RAMBUF1 */
#define RAMBUF_BYTES_PER_BURST 2
#define RAMBUF_REQUEST_PER_BURST 1
#define RAMBUF_SRC_BASE (CYDEV_PERIPH_BASE)
#define RAMBUF_DST_BASE (CYDEV_SRAM_BASE)
/* Defines for USBFS */
#define USBFS_DEVICE                (0u)
#define IN_EP_NUM                   (1u)
#define OUT_EP_NUM                  (2u)
#define BUFFER_SIZE                (64u)

typedef uint8   byte;
typedef uint16  hword;
typedef uint32  word;

byte obuffer[BUFFER_SIZE];
byte ibuffer[BUFFER_SIZE];

hword ssample[100];
hword bsample[100];

void init();

int main()
{
    /* Variable declarations for USBFS */
    hword length;
    byte i = 0;
    byte j = 0;
    
    CyGlobalIntEnable; /* Enable global interrupts. */
    init();
    
    USBFS_EnableOutEP(OUT_EP_NUM);
    for(;;)
    {
        
        //hword sample = POTADC_GetResult16();
        j = (j == 99) ?  0:j;
        
        /* Check if configuration is changed. */
        if (0u != USBFS_IsConfigurationChanged())
        {
            /* Re-enable endpoint when device is configured. */
            if (0u != USBFS_GetConfiguration())
            {
                /* Enable OUT endpoint to receive data from host. */
                USBFS_EnableOutEP(OUT_EP_NUM);
            }
        }

        /* Check if data was received. */
        if (USBFS_OUT_BUFFER_FULL == USBFS_GetEPState(OUT_EP_NUM))
        {
            /* Read number of received data bytes. */
            length = USBFS_GetEPCount(OUT_EP_NUM);

            /* Trigger DMA to copy data from OUT endpoint buffer. */
            USBFS_ReadOutEP(OUT_EP_NUM, obuffer, length);

            /* Wait until DMA completes copying data from OUT endpoint buffer. */
            while (USBFS_OUT_BUFFER_FULL == USBFS_GetEPState(OUT_EP_NUM))
            {
            }
            
            /* Enable OUT endpoint to receive data from host. */
            USBFS_EnableOutEP(OUT_EP_NUM);

            /* Wait until IN buffer becomes empty (host has read data). */
            while (USBFS_IN_BUFFER_EMPTY != USBFS_GetEPState(IN_EP_NUM))
            {
            }

            BRAKEADC_StopConvert();
            STEERADC_StopConvert();
            i = (i >= BUFFER_SIZE) ? 0:i;
            
            ibuffer[i] = LO8(bsample[j]);
            ibuffer[i+1] = HI8(bsample[j]);
            //ibuffer[i+2] = LO8(bsample[j]);
            //ibuffer[i+3] = HI8(bsample[j]);
            
            i += 2;
            length = i;
            
        /* Trigger DMA to copy data into IN endpoint buffer.
        * After data has been copied, IN endpoint is ready to be read by the
        * host.
        */
            USBFS_LoadInEP(IN_EP_NUM, ibuffer, length);
        
            BRAKEADC_StartConvert();
            STEERADC_StartConvert();
        }
        j++;
    }
}

void init()
{
    /* Variable declarations for RAMBUF1 */
    /* Move these variable declarations to the top of the function */
    uint8 RAMBUF1_Chan;
    uint8 RAMBUF1_TD[1];
    /* Variable declarations for RAMBUF2 */
    /* Move these variable declarations to the top of the function */
    uint8 RAMBUF2_Chan;
    uint8 RAMBUF2_TD[1];

    /* DMA Configuration for RAMBUF1 */
    RAMBUF1_Chan = RAMBUF1_DmaInitialize(RAMBUF_BYTES_PER_BURST, RAMBUF_REQUEST_PER_BURST, 
        HI16(RAMBUF_SRC_BASE), HI16(RAMBUF_DST_BASE));
    RAMBUF1_TD[0] = CyDmaTdAllocate();
    CyDmaTdSetConfiguration(RAMBUF1_TD[0], 200, RAMBUF1_TD[0], TD_INC_DST_ADR | TD_AUTO_EXEC_NEXT | RAMBUF1__TD_TERMOUT_EN);
    CyDmaTdSetAddress(RAMBUF1_TD[0], LO16((uint32)STEERADC_SAR_WRK0_PTR), LO16((uint32)ssample));
    CyDmaChSetInitialTd(RAMBUF1_Chan, RAMBUF1_TD[0]);
    CyDmaChEnable(RAMBUF1_Chan, 1);
    
    /* DMA Configuration for RAMBUF2 */
    RAMBUF2_Chan = RAMBUF2_DmaInitialize(RAMBUF_BYTES_PER_BURST, RAMBUF_REQUEST_PER_BURST, 
        HI16(RAMBUF_SRC_BASE), HI16(RAMBUF_DST_BASE));
    RAMBUF2_TD[0] = CyDmaTdAllocate();
    CyDmaTdSetConfiguration(RAMBUF2_TD[0], 200, RAMBUF2_TD[0], TD_INC_DST_ADR | TD_AUTO_EXEC_NEXT | RAMBUF2__TD_TERMOUT_EN);
    CyDmaTdSetAddress(RAMBUF2_TD[0], LO16((uint32)BRAKEADC_SAR_WRK0_PTR), LO16((uint32)bsample));
    CyDmaChSetInitialTd(RAMBUF2_Chan, RAMBUF2_TD[0]);
    CyDmaChEnable(RAMBUF2_Chan, 1);
    
    USBFS_Start(USBFS_DEVICE, USBFS_5V_OPERATION);
    while(0u == USBFS_GetConfiguration()){} //wait until we are enumerated by host
    
    STEERADC_Start();
    STEERADC_StartConvert();
    STEERADC_IRQ_Disable();
    BRAKEADC_Start();
    BRAKEADC_StartConvert();
    BRAKEADC_IRQ_Disable();
    
    RAMBUF1_DONE_Start();
    RAMBUF2_DONE_Start();
}
/* [] END OF FILE */
