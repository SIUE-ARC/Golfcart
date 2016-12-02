/* ========================================
 * This file contains all neccessary definitions
 * for the functions described in roadrunner.h
 * ========================================
*/
#include "roadrunner.h"
#include <stdlib.h>

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
    
    argv = (byte**)malloc(sizeof(byte*)*BUFFER_SIZE);

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
    
    DVDAC_Start();
    DVDAC_SetValue(0);
    UART_Start();
    
    STEERADC_Start();
    STEERADC_StartConvert();
    STEERADC_IRQ_Disable();
    BRAKEADC_Start();
    BRAKEADC_StartConvert();
    BRAKEADC_IRQ_Disable();
    
    RAMBUF1_DONE_Start();
    RAMBUF2_DONE_Start();
    
    CyGlobalIntEnable; /* Enable global interrupts. */
}

void command_lookup(byte argc, byte** argv)
{
    byte command = obuffer[0];
    
    switch(command)
    {
        case 'F':
        case 'f':
            if(argc > 0)
            {
                hword dval = atoi(argv[1]);
                RLY_REVERSE_DR &= ~RLY_REVERSE_MASK;
                DVDAC_SetValue(dval);
            }
            else
            {
                stpcpy(ibuffer, "No DAC val given!\r\n\0");
            }
            break;
        case 'B':
        case 'b':
            if(argc > 0)
            {
                hword dval = atoi(argv[1]);
                RLY_REVERSE_DR |= RLY_REVERSE_MASK;
                DVDAC_SetValue(dval);
            }
            else
            {
                stpcpy(ibuffer, "No DAC val given!\r\n\0");
            }
            break;
        case 'D':
        case 'd':
            {
                byte* buf0 = "Drive encoder: ";
                byte* buf1;
                hword count = QuadDecDrive_GetCounter();
                itoa(count, buf1, 10); 
                #ifdef VERBOSE 
                    strcpy(buf0, buf1);
                    strcpy(buf0, "\r\n\0");
                    strcpy(ibuffer, buf0);
                #else
                    strcpy(ibuffer, buf1);
                #endif
            }
            break;
        case 'I':
        case 'i':
            UART_WriteTxData((char)BAUD_BYTE);
            //baudSent = TRUE;
            break;
        case 'T':
        case 't':
            if(argc > 0)
                turn(atoi(argv[1]));
                
            #ifdef VERBOSE
                else strcpy(ibuffer, "No value given!\r\n");
            #endif
            break;
        case 'S':
        case 's':
            {
                byte* buf0 = "Steer encoder: ";
                byte* buf1;
                hword count = QuadDecSteer_GetCounter();
                itoa(count, buf1, 10); 
                #ifdef VERBOSE 
                    strncpy(buf0, buf1, strlen(buf1));
                    strcpy(buf0, "\r\n");
                    strncpy(ibuffer, buf0, strlen(buf0));
                #else
                    strncpy(ibuffer, buf1, strlen(buf1));
                #endif
            }
            break;
        case 'R':
        case 'r':
            calibrateSteering();
            #ifdef VERBOSE
                strcpy(ibuffer, "Shaft reset\r\n");
            #endif
            break;
        case 'H':
        case 'h':
            if(argc > 0)
                brake(atoi(argv[1]));
            #ifdef VERBOSE
                strcpy(ibuffer, "No brake value given!\r\n");
            #endif
            break;
    #ifdef EXTENDED_COMMANDS
        case 'E':
        case 'e':
            #ifdef VERBOSE
                strcpy(ibuffer, "Turning right\r\n");
            #endif
            setControllerSpeed(STEER_CTL, STEER_SPEED, RIGHT);
            break;
        case 'Q':
        case 'q':
            #ifdef VERBOSE
                strcpy(ibuffer, "Turning right\r\n");
            #endif
            setControllerSpeed(STEER_CTL, STEER_SPEED, LEFT);
            break;
        case 'L':
        case 'l':
            #ifdef VERBOSE
                strcpy(ibuffer, "Stopping\r\n");
            #endif
            stop();
            break;
        case 'J':
        case 'j':
            #ifdef VERBOSE
                strcpy(ibuffer, "Releasing brake\r\n");
            #endif
            brake(100);
            break;
        case 'P':
        case 'p':
            #ifdef VERBOSE
                byte* buf0 = "Steer pot: ";
                byte* buf1;
                strcpy(buf1, getSteerPosition());
                strcpy(buf1, "\r\n");
                strcpy(buf0, buf1);
                strncpy(ibuffer, buf0, strlen(buf0));
            #endif
            break;
    #endif
    }
}

// Tells the specified motor controller to run at the specfied speed. The
// controller will continue to run until it is explicity stopped. setControllerSpeed
// will cache the last parameters sent to each motor and only update the motor
// controller if the values have changed.
// Speed must be in the range [0, 127]
// dir is either a 0 or 1
void setControllerSpeed(byte addr, byte speed, byte dir)
{
    static byte lastValue[2];
    
    // Only send the serial packet if it changes the state of the motor controller of interest
    // speed is only a 7 bit value
    if (baudSent && (lastValue[addr == BRAKE_CTL ? 1 : 0] != (dir << 7 | speed))) 
    {
        // Packet format, 4 bytes
        // Address [128, 255], direction [0, 1], speed [0, 127], checksum
		byte TX[4];

		// This line has to be after line 320
		if((lastValue[addr == BRAKE_CTL ? 1 : 0] >> 7) != dir)
			speed = STOP;
		
        TX[0] = addr;
        TX[1] = dir;
        TX[2] = speed;
        TX[3] = (addr + dir + speed) & 0x7F;
        UART_PutArray(TX, 4);
        
        lastValue[addr == BRAKE_CTL ? 1 : 0] = (dir << 7 | speed);

        #ifdef VERBOSE
        #endif
    }
}

void turn(word count)
{
    count = max(ENCODER_LEFT_BOUND, count);
    count = min(ENCODER_RIGHT_BOUND, count);
    
    steerSetpoint = count;
    
    #ifdef VERBOSE
    #endif
}

void brake(int pVal)
{
    pVal = max(BRAKE_MIN_POS, pVal);
    pVal = min(BRAKE_MAX_POS, pVal);
    
    brakeSetpoint = pVal;
    
    #ifdef VERBOSE
    #endif 
}

void stop(void) 
{
    setControllerSpeed(STEER_CTL, STOP, STOP);
    setControllerSpeed(BRAKE_CTL, STOP, STOP);
    
    #ifdef VERBOSE
    #endif
}

hword getBrakePosition()
{
    int i;
    hword sum = 0;
    for(i = 0; i < 100; i++)
    {
        sum += bsample[i];
    }
    
    return sum/100;   
}

hword getSteerPosition()
{
    int i;
    hword sum = 0;
    for(i = 0; i < 100; i++)
    {
        sum += ssample[i];
    }
    
    return sum/100;   
}

void calibrateSteering()
{
    // Send the control off in the right direction
    byte dir = getSteerPosition() < STEER_POT_CENTER ? RIGHT : LEFT;
    setControllerSpeed(STEER_CTL, STEER_SPEED, dir);
    
    // Spin until the pot lines up
    while (abs(getSteerPosition() - STEER_POT_CENTER) > 30)
    {
        #ifdef VERBOSE
		    //UART_PutSHexInt(getSteerPotPosition() - STEER_POT_CENTER);
        #endif
    }
    // We are spinning here, so we need to invalidate the cache ourself
    cacheValid = FALSE;
    
    // Reset everything at center
    setControllerSpeed(STEER_CTL, STOP, STOP);
    steerCount = 0;
    steerSetpoint = 0;
}

void updateBrakeCtl()
{
    int error = brakeSetpoint - getBrakePosition();
    
    if (error < -20)
        setControllerSpeed(BRAKE_CTL, BRAKE_SPEED, APPLY);
    else if (error > 20)
        setControllerSpeed(BRAKE_CTL, BRAKE_SPEED, RELEASE);
    else
        setControllerSpeed(BRAKE_CTL, STOP, STOP);
}

void updateTurnCtl()
{
    int error = steerSetpoint - steerCount;
    
    if (error < -50)
        setControllerSpeed(STEER_CTL, STEER_SPEED, RIGHT);
    else if (error > 50)
        setControllerSpeed(STEER_CTL, STEER_SPEED, LEFT);
    else
        setControllerSpeed(STEER_CTL, STOP, STOP);
}

int min(int a, int b) { return a < b ? a : b; }
int max(int a, int b) { return a > b ? a : b; }
/* [] END OF FILE */
