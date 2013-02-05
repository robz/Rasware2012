#include "inc/hw_types.h"		// tBoolean
#include "CrawlerBot.h"
#include "utils/uartstdio.h"	// input/output over UART
#include "driverlib/uart.h"		// input/output over UART
#include "RASLib/init.h"
#include "RASLib/servo.h"

#define COMMAND_TIME 1000
#define TIME_INC 10
#define NUM_COMMANDS 2

LegCommand COMMANDS[2][NUM_COMMANDS] = { 
	{ // left leg commands as {hip, knee} pairs
		{0,0},{0,0}
	}, 
	{ // right leg commands as {hip, knee} pairs
		{0,0},{0,0}
	} 
};

LegCommand initPos = {0, 0};

LegCommand* LEFT_LEG_COMMANDS = COMMANDS[LeftLeg];
LegCommand* RIGHT_LEG_COMMANDS = COMMANDS[RightLeg];

LegServoInfo LEG_SERVOS[2] = {
	{SERVO_3, SERVO_2}, // left
	{SERVO_1, SERVO_0}  // right
};

void executeCommand(LegCommand curRight, LegCommand oldRight, LegCommand curLeft, LegCommand oldLeft);
void pause(unsigned short ms);

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
  UARTprintf("sizeof unsigned short: %d\n", sizeof(unsigned short));															    
	
	while(1) {	
		char ch;
		int i;
		
		UARTprintf("press any key to continue...");
		ch = getc();
		putc(ch);
		
		UARTprintf("\n executing commands!\n");
		
		//stepThruCommand(0, 255);
		for (i = 0; i < NUM_COMMANDS; i++) {	
			UARTprintf("   command #%d...\n", i);
			if (i == 0) {
				executeCommand(RIGHT_LEG_COMMANDS[i], initPos,
											 LEFT_LEG_COMMANDS[i], initPos);
			} else {
				executeCommand(RIGHT_LEG_COMMANDS[i], RIGHT_LEG_COMMANDS[i-1],
											 LEFT_LEG_COMMANDS[i], LEFT_LEG_COMMANDS[i-1]);
			}
			// pause(TIME_BETWEEN_COMMANDS);
		}
		
		UARTprintf(" commands executed!\n");
	}
}

unsigned char pos_funct(unsigned char p0, unsigned char p1, unsigned short t) {
  int res = 2*(p0 - p1)*t*t/(COMMAND_TIME*COMMAND_TIME)*t/COMMAND_TIME 
										+ 3*(p1 - p0)*t*t/(COMMAND_TIME*COMMAND_TIME) 
										+ p0;
	
	if (p1 > p0 && res > p1) {
		res = p1;
	} else if (p1 < p0 && res < p1) {
		res = p1;
	}
	
	return (unsigned char) res;
}

void stepCommandOnce(servo_t servo, unsigned char p0, unsigned char p1, unsigned short time_ms) {
	unsigned char pos = pos_funct(p0, p1, time_ms);
	if (servo == SERVO_2) {
		UARTprintf("%d %d %d %d\n", time_ms, p0, p1, pos);
	}
	SetServoPosition(servo, pos);
}

void stepThruCommand(servo_t servo_a, unsigned char p0_a, unsigned char p1_a, 
										 servo_t servo_b, unsigned char p0_b, unsigned char p1_b) {
  unsigned short time_ms = 0;
  
  for (time_ms = 0; time_ms <= COMMAND_TIME; time_ms += TIME_INC) {
		stepCommandOnce(servo_a, p0_a, p1_a, time_ms);
		stepCommandOnce(servo_b, p0_b, p1_b, time_ms);
		pause(TIME_INC);
  }
}

void executeCommand(LegCommand curRight, LegCommand oldRight, LegCommand curLeft, LegCommand oldLeft) {
	unsigned short time_ms = 0;
	unsigned char old_right_hip = oldRight.hipPosition,
								old_right_knee = oldRight.kneePosition,
								old_left_hip = 255 - oldLeft.hipPosition,
								old_left_knee = 255 - oldLeft.kneePosition,
								cur_right_hip = curRight.hipPosition,
								cur_right_knee = curRight.kneePosition,
								cur_left_hip = 255 - curLeft.hipPosition,
								cur_left_knee = 255 - curLeft.kneePosition;
  
  for (time_ms = 0; time_ms <= COMMAND_TIME; time_ms += TIME_INC) {
		stepCommandOnce(LEG_SERVOS[RightLeg].hipServo, old_right_hip, cur_right_hip, time_ms);
		stepCommandOnce(LEG_SERVOS[RightLeg].kneeServo, old_right_knee, cur_right_knee, time_ms);
		stepCommandOnce(LEG_SERVOS[LeftLeg].hipServo, old_left_hip, cur_left_hip, time_ms);
		stepCommandOnce(LEG_SERVOS[LeftLeg].kneeServo, old_left_knee, cur_left_knee, time_ms);
		pause(TIME_INC);
  }
}

void pause(unsigned short ms) {
	unsigned int i, j;
	
	for (j = 0; j < ms; j++) {
		for (i = 0; i < 10000; i++);
	}
}
