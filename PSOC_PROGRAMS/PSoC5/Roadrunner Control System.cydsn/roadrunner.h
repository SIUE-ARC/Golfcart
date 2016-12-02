/* ========================================
 * This file contains defines, constants,
 * and neccessary function declarations
 * for the roadrunner control systems.
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

/******** CONSTANTS ********/
#define MAX_PARAMS                  5
#define POT_LEFT_BOUND 0x00FF
#define STEER_POT_CENTER 0x204
#define POT_RIGHT_BOUND 0x02EF

#define ENCODER_LEFT_BOUND -8000
#define ENCODER_RIGHT_BOUND 8000

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
#define RELEASE 1
#define APPLY 0

#define TRUE 1
#define FALSE 0

/******** CODE SECTIONS ********/

//#define EXTENDED_COMMANDS
#define VERBOSE
#define LCD
typedef uint8   byte;
typedef uint16  hword;
typedef uint32  word;

byte obuffer[BUFFER_SIZE];
byte ibuffer[BUFFER_SIZE];

hword ssample[100];
hword bsample[100];


// Are currently cached analog reads valid, or should they be reaquired
byte cacheValid;

// Have the motor controllers been initilized
byte baudSent;

// The current count of the quadrature encoder connected to the steering column
int steerCount;

// The current target positions for both motor controllers
int brakeSetpoint;
int steerSetpoint;

byte argc;
byte** argv;

void init();
void command_lookup(byte argc, byte** argv);
void setControllerSpeed(byte addr, byte speed, byte dir);
void turn(word count);
void brake(int pVal);
void stop();
hword getSteerPosition();
hword getBrakePotPosition();
void calibrateSteering();
void updateBrakeCtl();
void updateTurnCtl();
int min(int a, int b);
int max(int, int);
/* [] END OF FILE */
