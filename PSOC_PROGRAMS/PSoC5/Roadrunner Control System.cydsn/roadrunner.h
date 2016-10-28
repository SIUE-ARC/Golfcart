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

/******** CODE SECTIONS ********/

#define EXTENDED_COMMANDS
#define VERBOSE
#define LCD
typedef uint8   byte;
typedef uint16  hword;
typedef uint32  word;

extern byte obuffer[BUFFER_SIZE];
extern byte ibuffer[BUFFER_SIZE];

extern hword ssample[100];
extern hword bsample[100];

void init();
void command_lookup(byte argc, byte** argv);
void turnToCount(word count);
void turn(int count);
void brake(int pVal);
hword getActuatorPosition();
hword getSteerPotPosition();
void calibrateSteering();
void updateBrakeCtl(void);
void updateTurnCtl(void);
int min(int, int);
int max(int, int);
int min(int a, int b) { return a < b ? a : b; }
int max(int a, int b) { return a > b ? a : b; }
/* [] END OF FILE */
