#include "inc/hw_types.h"		// tBoolean
#include "CrawlerBot.h"
#include "utils/uartstdio.h"	// input/output over UART
#include "driverlib/uart.h"		// input/output over UART
#include "RASLib/init.h"
#include "RASLib/servo.h"

#define TIME_BETWEEN_COMMANDS 1000
#define COMMAND_TIME 1000
#define TIME_INC 10
#define NUM_COMMANDS 3

LegCommand initPos = {0, 0};

LegCommand COMMANDS[2][NUM_COMMANDS] = { 
	{ // left leg commands as {hip, knee} pairs
		{0,0},{0,255},{0,0}
	}, 
	{ // right leg commands as {hip, knee} pairs
		{0,0},{0,0},{0,0}
	} 
};

LegCommand* LEFT_LEG_COMMANDS = COMMANDS[LeftLeg];
LegCommand* RIGHT_LEG_COMMANDS = COMMANDS[RightLeg];

LegServoInfo LEG_SERVOS[2] = {
	{SERVO_3, SERVO_2}, // left
	{SERVO_1, SERVO_0}  // right
};

void executeCommand(LegID legId, LegCommand cur, LegCommand old);
void pause(unsigned int ms);

int main(void)
{	  	 
	LockoutProtection();
	InitializeMCU();
	
	initUART();			
	initServo();
	
  SetServoPosition(SERVO_3, 255);
	SetServoPosition(SERVO_2, 255);
	SetServoPosition(SERVO_1, 0);
	SetServoPosition(SERVO_0, 0);
	
	UARTprintf("\n\nCrawler's gotta crawl...\n");																	    
	
	while(1) {	
		char ch;
		int i;
		
		UARTprintf("press any key to continue...");
		ch = getc();
		putc(ch);
		
		UARTprintf("\n executing commands!\n");
		
		for (i = 0; i < NUM_COMMANDS; i++) {	
			UARTprintf("   command #%d...\n", i);
			if (i == 0) {
				executeCommand(RightLeg, initPos, RIGHT_LEG_COMMANDS[i]);
				executeCommand(LeftLeg, initPos, RIGHT_LEG_COMMANDS[i]);
			} else {
				executeCommand(RightLeg, RIGHT_LEG_COMMANDS[i-1], RIGHT_LEG_COMMANDS[i]);
				executeCommand(LeftLeg, LEFT_LEG_COMMANDS[i-1], RIGHT_LEG_COMMANDS[i]);
			}
			pause(TIME_BETWEEN_COMMANDS);
		}
		
		UARTprintf(" commands executed!\n");
	}
}

unsigned char pos_funct(unsigned char p0, unsigned char p1, unsigned int t) {
  return 2*(p0 - p1)*t*t/1000000*t/1000 + 3*(p1 - p0)*t*t/1000000 + p0;
}

void stepThruCommand(servo_t servo_a, unsigned char p0_a, unsigned char p1_a, 
										 servo_t servo_b, unsigned char p0_b, unsigned char p1_b) {
  unsigned int time_ms = 0;
  
  for (time_ms = 0; time_ms < COMMAND_TIME; time_ms += TIME_INC) {
		unsigned char pos_a = pos_funct(p0_a, p1_a, time_ms),
									pos_b = pos_funct(p0_b, p1_b, time_ms);
		SetServoPosition(servo_a, pos_a);
		SetServoPosition(servo_b, pos_b);
		pause(TIME_INC);
  }
}

void executeCommand(LegID legId, LegCommand cur, LegCommand old) {
	LegServoInfo legServos = LEG_SERVOS[legId];
	
	if (legId == LeftLeg) {
		stepThruCommand(legServos.hipServo, 255 - old.hipPosition, 255 - cur.hipPosition, 
										legServos.kneeServo, 255 - old.kneePosition, 255 - cur.kneePosition);
	} else {
		stepThruCommand(legServos.hipServo, old.hipPosition, cur.hipPosition,
										legServos.kneeServo, old.kneePosition, cur.kneePosition);
	}
}

void pause(unsigned int ms) {
	unsigned int i, j;
	
	for (j = 0; j < ms; j++) {
		for (i = 0; i < 10000; i++);
	}
}
