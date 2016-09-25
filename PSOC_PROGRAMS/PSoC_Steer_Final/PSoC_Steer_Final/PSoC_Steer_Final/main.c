//----------------------------------------------------------------------------
// C main line
//----------------------------------------------------------------------------

#include <m8c.h>        // part specific constants and macros
#include "PSoCAPI.h"    // PSoC API definitions for all User Modules
#include "PSoCGPIOINT.h"
#include <string.h>
#include <stdlib.h> 

#pragma interrupt_handler PSoC_GPIO_ISR_C

/******** CONSTANTS ********/

#define POT_LEFT_BOUND 0x00FF
#define STEER_POT_CENTER 0x204
#define POT_RIGHT_BOUND 0x02EF

#define ENCODER_LEFT_BOUND -2300
#define ENCODER_RIGHT_BOUND 2300

#define BRAKE_MAX_POS 900
#define BRAKE_MIN_POS 100

#define STOP 0

#define BAUD_BYTE 0xAA

#define STEER_CTL 128
#define STEER_SPEED 80
#define LEFT 0
#define RIGHT 1

#define BRAKE_CTL 130
#define BRAKE_SPEED 120
#define RELEASE 0
#define APPLY 1


/******** CODE SECTIONS ********/

#define EXTENDED_COMMANDS
#define VERBOSE
#define LCD


/******** PROTOTYPES ********/

// Get the ADC values of the appropriate sensor. Values will be cached until cacheValid is reset
unsigned int getBrakePosition(void);
unsigned int getSteerPotPosition(void);

// Tells the specified motor controller to run at the specfied speed. The
// controller will continue to run until it is explicity stopped. setControllerSpeed
// will cache the last parameters sent to each motor and only update the motor
// controller if the values have changed.
// Speed must be in the range [0, 127]
// dir is either a 0 or 1
void setControllerSpeed(BYTE addr, BYTE speed, BYTE dir);

// Set the current turn and brake setpoints. Both are clamped between the appropriate defines
void turn(int count);
void brake(int pVal);

// Stops all motors. This does not mean the brake will be applied.
void stop(void);

// Update the motor controller speeds based on the current setpoints
void updateBrakeCtl(void);
void updateTurnCtl(void);

// Realign the steer encoder so that 0 counts occurs when the pot is centered
void calibrateSteering(void);

// Handle the supplied command. Reads from the serial buffer to get the command parameters
void command_lookup(BYTE argc, char **argv);

// Return the larger, or smaller, of two integers
int min(int, int);
int max(int, int);
int min(int a, int b) { return a < b ? a : b; }
int max(int a, int b) { return a > b ? a : b; }


/******** GLOBALS ********/

// Are currently cached analog reads valid, or should they be reaquired
BYTE cacheValid;

// Have the motor controllers been initilized
BYTE baudSent;

// The current count of the quadrature encoder connected to the steering column
int steerCount;

// The current target positions for both motor controllers
int brakeSetpoint;
int steerSetpoint;


/******** MAIN ********/

void main(void)
{
    // Enable GPIO Interrupts (see m8c.h)
    M8C_EnableGInt ; 
    M8C_EnableIntMask(INT_MSK0,INT_MSK0_GPIO);  

    // Initilize computer-psoc serial command stuff
    UART_CmdReset();
    UART_IntCntl(UART_ENABLE_RX_INT);
    UART_Start(UART_PARITY_NONE);
    
    // Initilize psoc-sabertooth serial command stuff
    TX8_Start(TX8_PARITY_NONE);
    TX8_EnableInt();

    // Initilize analog input for brake and steer pots
    Actuator_Pot_Start(Actuator_Pot_HIGHPOWER);
    Steer_Pot_Start(Steer_Pot_HIGHPOWER);
    DUALADC_Start(DUALADC_HIGHPOWER);
    
    #ifdef LCD
        // Initilize the debug LCD screen
        LCD_Start();
        LCD_Position(0,0);
        LCD_PrCString("Steering PSoC");
    #endif
    
    UART_CPutString("Steer Program Start\r\n");
        
    #ifdef HEARTBEAT
        unsigned int heartbeat = 0;
    #endif
    
    while (TRUE)
    {
        // Invalidate the cached analog values
        cacheValid = FALSE;
        
        #ifdef HEARTBEAT
            // Send a heartbeat to let computer know we aren't dead
            if(heartbeat % 500 == 0)
            {
                UART_CPutString("ping");
                UART_PutCRLF();
            }
            heartbeat++;
        #endif
        
        // If the e-stop line is low, kill everything
        if (!(PRT1DR & ESTOP_MASK)){
            stop();

            UART_CPutString("ESTOP");
            UART_PutCRLF();
            
            #ifdef LCD
                LCD_Position(0,0);
                LCD_PrCString("E-STOP");
            #endif
            
            // Spin until the e-stop is disengaged
            while (!(PRT1DR & ESTOP_MASK));

            #ifdef LCD
                LCD_Position(0,0);
                LCD_PrCString("Steering PSoC");
            #endif

            UART_CPutString("RESUME");
            UART_PutCRLF();
        }
        
        // Run the PID calculations for both motors
        updateBrakeCtl();
        updateTurnCtl();

        // Handle the first serial command in the buffer
        if(UART_bCmdCheck()) 
        {
            // Read all parameters
            BYTE argc = 0;
            char *argv[2];

            while(argv[argc] = UART_szGetParam())
                argc++;

            // Execute the command
            command_lookup(argc, argv);

            // Reset the serial buffer
            UART_CmdReset();
        }
    }
}


/******** FUNCTIONS ********/

void command_lookup(BYTE argc, char** argv)
{
    switch (*argv[0])
    {
        // Send the baud character to the motor controllers and the
        // psoc identifer to the computer
        case 'I':
        case 'i':
            UART_PutChar('S');
            TX8_PutChar((CHAR)BAUD_BYTE);
			baudSent = 1;
            break;
            
        // Reset the position of the wheels to '0'
        case 'R':
        case 'r':
            calibrateSteering();
            UART_CPutString("Shaft Reset\r\n");
            break;
            
        // Request the steer encoder count
        case 'S':
        case 's':
            #ifdef VERBOSE
                UART_CPutString("Steer encoder: ");
            #endif
            UART_PutSHexInt(steerCount);
            UART_PutCRLF();
            break;

        // Turn to a specific count
        case 'T':
        case 't':
            if (argc == 2)
                turn(atoi(argv[1]));
            
            #ifdef VERBOSE
                else UART_CPutString("No value given!\r\n");
            #endif
            break;
                        
        // Apply the brake to the specified value [100, 900]
        case 'h':
        case 'H':
            if(argc == 2)
                brake(atoi(argv[1]));
            
            #ifdef VERBOSE
                else UART_CPutString("No brake value given!!!\r\n");
            #endif
            break;
            
        #ifdef EXTENDED_COMMANDS
            // Manual turning right
            case 'E':
            case 'e':
                #ifdef VERBOSE
                    UART_CPutString("Turning Right\r\n");
                #endif
                
                setControllerSpeed(STEER_CTL, STEER_SPEED, RIGHT);
                break;
                
            // Manual turning left
            case 'Q':
            case 'q':
                #ifdef VERBOSE
                    UART_CPutString("Turning Left\r\n");
                #endif
                
                setControllerSpeed(STEER_CTL, STEER_SPEED, LEFT);
                break;
                
            //manual stopping
            case 'L':
            case 'l':
                #ifdef VERBOSE
                    UART_CPutString("Stopping\r\n");
                #endif

                stop();
                break;
                
            // Fully release the brake
            case 'J':
            case 'j':
                brake(100);
                break;
                
            // Request the value of the steer pot
            case 'P':
            case 'p':
                #ifdef VERBOSE
                    UART_CPutString("Steer pot: ");
                #endif
                UART_PutSHexInt(getSteerPotPosition());
                UART_PutCRLF();
                break;
        #endif
            
        // Invalid command
        default:
            #ifdef VERBOSE
                UART_CPutString("Invalid Command: >");
                UART_PutChar(*argv[0]);
                UART_CPutString("<\n\r");
            #endif
            break;
    }
}

void setControllerSpeed(BYTE addr, BYTE speed, BYTE dir) {
    static BYTE lastValue[2];
    
    // Only send the serial packet if it changes the state of the motor controller of interest
    // speed is only a 7 bit value
    if (baudSent && (lastValue[addr == BRAKE_CTL ? 1 : 0] != (dir << 7 | speed))) {
        
        // Packet format, 4 bytes
        // Address [128, 255], direction [0, 1], speed [0, 127], checksum
        BYTE TX[4];
        TX[0] = addr;
        TX[1] = dir;
        TX[2] = speed;
        TX[3] = (addr + dir + speed) & 0x7F;
        TX8_Write(TX, 4);
        
        lastValue[addr == BRAKE_CTL ? 1 : 0] = (dir << 7 | speed);

        #ifdef VERBOSE
            UART_CPutString(addr == BRAKE_CTL ? "Brake" : "Turn");
            if(speed != 0) {
                UART_CPutString(" controller turning ");
                UART_CPutString(dir == LEFT ? "left" : "right");
            } else {
                UART_CPutString(" controller stopping");
            }
            UART_PutCRLF();
        #endif
    }
}

void stop(void) {
    setControllerSpeed(STEER_CTL, STOP, STOP);
    setControllerSpeed(BRAKE_CTL, STOP, STOP);
    
    #ifdef VERBOSE
        UART_CPutString("Stopping all motors!!");
        UART_PutCRLF();
    #endif
}

void updateBrakeCtl(void) {
    int error = brakeSetpoint - getBrakePosition();
    
    if (error < -20)
        setControllerSpeed(BRAKE_CTL, BRAKE_SPEED, APPLY);
    else if (error > 20)
        setControllerSpeed(BRAKE_CTL, BRAKE_SPEED, RELEASE);
    else
        setControllerSpeed(BRAKE_CTL, STOP, STOP);
}

void updateTurnCtl(void) {
    int error = steerSetpoint - steerCount;
    
    if (error < -20)
        setControllerSpeed(STEER_CTL, STEER_SPEED, RIGHT);
    else if (error > 20)
        setControllerSpeed(STEER_CTL, STEER_SPEED, LEFT);
    else
        setControllerSpeed(STEER_CTL, STOP, STOP);
}

unsigned int getBrakePosition(void)
{
    static unsigned int brakePot;
    
    if (!cacheValid) {
        DUALADC_GetSamples(2);
        
        // Wait for data to be ready
        while(DUALADC_fIsDataAvailable() == 0);
            
        // Get Data and clear flag
        brakePot = DUALADC_iGetData2ClearFlag();
		
		cacheValid = TRUE;
    }
    
    return brakePot;
}

unsigned int getSteerPotPosition(void)
{
    static unsigned int steerPot;
    
    if (!cacheValid) {
        DUALADC_GetSamples(2);
        
        // Wait for data to be ready
        while(DUALADC_fIsDataAvailable() == 0);
            
        // Get Data and clear flag
        steerPot = DUALADC_iGetData1ClearFlag();
		
		cacheValid = TRUE;
    }
    
    return steerPot;
}

void turn(int count)
{
    count = max(ENCODER_LEFT_BOUND, count);
    count = min(ENCODER_RIGHT_BOUND, count);
    
    steerSetpoint = count;
    
    #ifdef VERBOSE
        UART_CPutString("Steer setpoint: ");
        UART_PutSHexInt(count);
        UART_PutCRLF();
    #endif
}

void brake(int pVal)
{
    pVal = max(BRAKE_MIN_POS, pVal);
    pVal = min(BRAKE_MAX_POS, pVal);
    
    brakeSetpoint = pVal;
    
    #ifdef VERBOSE
        UART_CPutString("Brake setpoint: ");
        UART_PutSHexInt(pVal);
        UART_PutCRLF();
    #endif 
}

void calibrateSteering(void)
{
    // Send the control off in the right direction
    BYTE dir = getSteerPotPosition() < STEER_POT_CENTER ? RIGHT : LEFT;
    setControllerSpeed(STEER_CTL, STEER_SPEED, dir);
    
    // Spin until the pot lines up
    while (abs(getSteerPotPosition() - STEER_POT_CENTER) > 30)
		UART_PutSHexInt(getSteerPotPosition() - STEER_POT_CENTER);
        // We are spinning here, so we need to invalidate the cache ourself
        cacheValid = FALSE;
    
    // Reset everything at center
    setControllerSpeed(STEER_CTL, STOP, STOP);
    steerCount = 0;
    steerSetpoint = 0;
}


/******** INTERRUPTS ********/

//  <--- Ticks++     Ticks-- --->
//I    |  |  |  |  |  |  |  |
//A ___|-----|_____|-----|____
//B   ____|-----|_____|-----|____
void PSoC_GPIO_ISR_C(void)
{
    static BYTE then;

    // Get the state of both encoder pins
    BYTE now = (PRT1DR & (OpEncA_MASK | OpEncB_MASK)) >> 4;

    // If we are turning left
    if (       then == 0 && now == 1    // Rising edge on A while B is low
            || then == 1 && now == 3    // Rising edge on B while A is high
            || then == 3 && now == 2    // Falling edge on A while B is high
            || then == 2 && now == 0)   // Falling edge on B while A is low
        steerCount--;


    // If we are turning right
    else if (  then == 0 && now == 2    // Rising edge on B while A is low
            || then == 2 && now == 3    // Rising edge on A while B is high
            || then == 3 && now == 1    // Falling edge on B while A is high
            || then == 1 && now == 0)   // Falling edge on A while B is low
        steerCount++;

    #ifdef VERBOSE
        else
            UART_CPutString("Unhandled encoder interrupt\r\n");
    #endif 

    // Update the saved encoder state
    then = now;
}

