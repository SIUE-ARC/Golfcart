/* ========================================
 * This file contains all neccessary definitions
 * for the functions described in roadrunner.h
 * ========================================
*/
#include <roadrunner.h>

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
