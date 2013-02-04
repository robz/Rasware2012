#include "inc/hw_types.h"		// tBoolean
#include "CrawlerBot.h"
#include "utils/uartstdio.h"	// input/output over UART
#include "driverlib/uart.h"		// input/output over UART
#include "RASLib/init.h"
#include "RASLib/servo.h"

#define COMMAND_EXECUTION_PERIOD_MS 1000
#define NUM_COMMANDS 1

LegCommand COMMANDS[2][NUM_COMMANDS] = { 
	{ // left leg commands as {hip, knee} pairs
		{0,0},
	}, 
	{ // right leg commands as {hip, knee} pairs
		{0,0},
	} 
};

LegCommand* LEFT_LEG_COMMANDS = COMMANDS[LeftLeg];
LegCommand* RIGHT_LEG_COMMANDS = COMMANDS[RightLeg];

LegServoInfo LEG_SERVOS[2] = {
	{SERVO_3, SERVO_2}, // left
	{SERVO_1, SERVO_0}  // right
};

void executeCommand(LegID leg, LegCommand c);
void pauseExecution(unsigned int ms);

int main(void)
{	  	 
	LockoutProtection();
	InitializeMCU();
	
	initUART();			
	initServo();
	
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
			
			executeCommand(RightLeg, RIGHT_LEG_COMMANDS[i]);
			executeCommand(LeftLeg, LEFT_LEG_COMMANDS[i]);
			pauseExecution(COMMAND_EXECUTION_PERIOD_MS);
		}
		
		UARTprintf(" commands executed!\n");
	}
}

void pauseExecution(unsigned int ms) {
	unsigned int i, j;
	for (j = 0; j < ms; j++) {
		for (i = 0; i < 10000; i++);
	}
}

void executeCommand(LegID legId, LegCommand c) {
	LegServoInfo legServos = LEG_SERVOS[legId];
	if (legId == LeftLeg) {
		SetServoPosition(legServos.hipServo, 255 - c.hipPosition); 
		SetServoPosition(legServos.kneeServo, 255 - c.kneePosition);
	} else {
		SetServoPosition(legServos.hipServo, c.hipPosition); 
		SetServoPosition(legServos.kneeServo, c.kneePosition);
	}
}
