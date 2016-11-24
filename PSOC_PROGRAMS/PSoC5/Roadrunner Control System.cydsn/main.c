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
#include "roadrunner.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

int main()
{
    init();
    
    for(;;)
    {
        cacheValid = FALSE;
        
        if(!(ESTOP_DR & ESTOP_MASK))
        {
            stop();
            #ifdef VERBOSE
                strcpy(ibuffer, "ESTOP engaged!\0");
                handleUSB(TRUE);
            #endif
            
            while (!(ESTOP_DR & ESTOP_MASK));
            
            #ifdef VERBOSE
                strcpy(ibuffer, "Resume\0");
                handleUSB(TRUE);
            #endif
        }
        
        // Run the PID calculations for both motors
        updateBrakeCtl();
        updateTurnCtl();
        
        handleUSB(FALSE);
    }
}
/* [] END OF FILE */
