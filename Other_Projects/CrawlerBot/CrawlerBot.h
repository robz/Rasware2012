#include "inc/hw_types.h"		// tBoolean
#include "RASLib/servo.h"

struct LegCommand {
	unsigned char hipPosition, kneePosition;
} typedef LegCommand;

struct LegServoInfo {
	servo_t hipServo, kneeServo;
} typedef LegServoInfo;

typedef enum LegID { 
	LeftLeg = 0, 
	RightLeg = 1
} LegID;

// Functions in UARTDemo
void initUART(void);
void uartDemo(void);
char getc(void);
void putc(char ch);
int keyWasPressed(void);

// Functions in ServoDemo
void initServo(void);
void servoDemo(void);

// Functions in EncoderDemo
void initEncoders(void);
void encoderDemo(void);

// Functions in Movement
int getSmoothServoValue(int x0, int x1, int dt, int t, int b0, int b1);
