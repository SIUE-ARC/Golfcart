//----------------------------------------------------------------------------
// C main line
//----------------------------------------------------------------------------

#include <m8c.h>        // part specific constants and macros
#include "PSoCAPI.h"    // PSoC API definitions for all User Modules
#include "PSoCGPIOINT.h"
#include <string.h>
#include <stdlib.h>	

#pragma interrupt_handler PSoC_GPIO_ISR_C

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

int command_lookup(BYTE cmd);

int min(int, int);
int max(int, int);
int min(int a, int b) { return a < b ? a : b; }
int max(int a, int b) { return a > b ? a : b; }


/******** GLOBALS ********/

// Encoder state variables
BYTE prevPrt;
BYTE curPrt;

// Are currently cached analog reads valid, or should they be reaquired
BYTE cacheValid;

// The current count of the quadrature encoder connected to the steering column
int steerCount;

// The current target positions for both motor controllers
int brakeSetpoint;
int steerSetpoint;


/******** MAIN ********/

void main(void)
{
	char* data;
	
	unsigned int heartbeat = 0;
	
	M8C_EnableGInt ; 
	M8C_EnableIntMask(INT_MSK0,INT_MSK0_GPIO); 	// Enable GPIO Interrupts (see m8c.h)
	
	UART_CmdReset();
	UART_IntCntl(UART_ENABLE_RX_INT);
	UART_Start(UART_PARITY_NONE);
	
	TX8_Start(TX8_PARITY_NONE);
	TX8_EnableInt();
	Actuator_Pot_Start(Actuator_Pot_HIGHPOWER);
	Steer_Pot_Start(Steer_Pot_HIGHPOWER);
	DUALADC_Start(DUALADC_HIGHPOWER);
	
	LCD_Start();
	LCD_Position(0,0);
	LCD_PrCString("Steering PSoC");
	
	UART_CPutString("Steer Program Start\r\n");
	//TX8_PutChar(baudChar);
	//DAC8_1_Start(DAC8_1_HIGHPOWER);
	//DAC8_1_WriteBlind(val);
		
	while (TRUE)
	{
		cacheValid = FALSE;
		
		prevPrt = (PRT1DR & (OpEncA_MASK | OpEncB_MASK)); 
		
		if(heartbeat % 500 == 0)
		{
			UART_PutCRLF();
			UART_CPutString("ping");
			UART_PutCRLF();
		}
		heartbeat++;
		
		// If the e-stop line is low
		if (!(PRT1DR & ESTOP_MASK)){
			UART_CPutString("ESTOP");
			UART_PutCRLF();
			
			stop();
			LCD_Position(0,0);
			LCD_PrCString("E-STOP");
			
			while (!(PRT1DR & ESTOP_MASK));
			UART_CPutString("RESUME");
			UART_PutCRLF();
		}
		
		updateBrakeCtl();
		updateTurnCtl();

		if(UART_bCmdCheck()) 
		{
			// Wait for command    
			if(data = UART_szGetParam()) 
			{
				//UART_PutString(data);
				command_lookup(*data);
			}   
			UART_CmdReset();  // Reset command buffer     
		}
	}
}

//Parses the command buffer when new command received
int command_lookup(BYTE cmd)
{
	BYTE* data;
	BYTE* TX;
	BYTE baud = 0xAA;
	int count = 0;
	switch (cmd)
	{
		// Turn to a specific count
		case 'T':
		case 't':
			if (data = UART_szGetParam())
				turn(atoi(data));
			
			#ifdef VERBOSE
				else UART_CPutString("No value given!\r\n");
			#endif
			break;
						
		// Apply the brake to the specified value [100, 900]
		case 'h':
		case 'H':
			if (data = UART_szGetParam())
				brake(atoi(data));
			
			#ifdef VERBOSE
				else UART_CPutString("No brake value given!!!\r\n");
			#endif
			break;
			
		// Request the steer encoder count
		case 'S':
		case 's':
			UART_PutSHexInt(steerCount);
			UART_PutCRLF();
			break;
			
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
			stop();
			break;
			
		//sending the baud character
		case 'I':
		case 'i':
			UART_PutChar('S');
			TX8_PutChar((CHAR)BAUD_BYTE);		
			break;
			
		//reset the position of the wheels to '0'
		case 'R':
		case 'r':
			calibrateSteering();
			UART_CPutString("Shaft Reset\r\n");
			break;

			
		// Fully release the brake
		case 'J':
		case 'j':
			brake(100);
			break;
			
		// Request the value of the steer pot
		case 'P':
		case 'p':
			UART_CPutString("Steer pot is at >");
			UART_PutSHexInt(getSteerPotPosition());
			UART_CPutString(" counts<\r\n");
			break;
			
		//Invalid command
		default:
			UART_CPutString("Invalid Command: >");
			UART_PutChar(cmd);
			UART_CPutString("<\n\r");
			break;
	}
	UART_CmdReset();
	return 0;
}

void setControllerSpeed(BYTE addr, BYTE speed, BYTE dir) {
	static BYTE lastValue[2];
	
	if (lastValue[addr == BRAKE_CTL ? 1 : 0] != dir << 7 | speed) {
		BYTE TX[4];
		TX[0] = addr;
		TX[1] = dir;
		TX[2] = speed;
		TX[3] = (addr + dir + speed) & 0x7F;
		TX8_Write(TX,4);
		
		lastValue[addr == BRAKE_CTL ? 1 : 0] = dir << 7 | speed;
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
	unsigned int brakePosition = getBrakePosition();
	static int error;
	
	error = brakeSetpoint - brakePosition;
	
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
	BYTE dir = 0;
	
	unsigned int steerPotvalue = getSteerPotPosition();
	
	if (steerPotvalue < STEER_POT_CENTER) dir = RIGHT;
	else if (steerPotvalue > STEER_POT_CENTER) dir = LEFT;
	
	setControllerSpeed(STEER_CTL, STEER_SPEED, dir);
	
	while (abs(steerPotvalue - STEER_POT_CENTER) > 30)
	{
		steerPotvalue = getSteerPotPosition();
	}
	
	setControllerSpeed(STEER_CTL, STOP, STOP);
	steerCount = 0;
}

/******** INTERRUPTS ********/

//A ___|-----|_____|-----|____
//B   ____|-----|_____|-----|____
void PSoC_GPIO_ISR_C(void)
{
	curPrt = (PRT1DR & (OpEncA_MASK | OpEncB_MASK));		// Setting prevPort to only bits 1[4] and 1[5]
																// of PRT1DR
		
	if ((prevPrt == 0x00) && (curPrt == 0x10))	// If prevPort is 0x00 and then after the interrupt curPrt is
												// 0x10 then A is high and B is low which means you wanted to
												// increment by turning clockwise and hitting a rising edge on A
	{
		// Increasing the count when clockwise turn interrupt occurred
		steerCount++;
	}
	else if ((prevPrt == 0x00) && (curPrt == 0x20))	// If prevPort is 0x00 and then after the interrupt curPrt is
													// 0x20 then B is high and A is low which means you wanted to
													// decrement by turning counterclockwise and hitting 
													// a rising edge on B
	{
		// Decreasing the count when the counterclockwise interrupt occurred
		steerCount--;
	}
}