#include "CrawlerBot.h"
#include "utils/uartstdio.h"	// input/output over UART
#include "driverlib/uart.h"		// input/output over UART
#include "RASLib/init.h"
#include "RASLib/servo.h"

#define COMMAND_TIME 500
#define TIME_INC 1
#define NUM_COMMANDS 5

LegCommand COMMANDS[2][NUM_COMMANDS] = { 
	{ // left leg commands as {hip, knee} pairs
		//{0,0},{0,0},{0,0},{0,0},{0,0},{0,0}
		//{255,0},{255,0},{255,0},
		{120,0},{50,0},{120,140},{160,100},{120,0},
	}, 
	{ // right leg commands as {hip, knee} pairs
		//{200,100},{200,0},{100,0},
		//{0,0},{0,0},{0,0},{0,0},{0,0},
		{160,100},{120,0},{120,0},{50,0},{120,150},
	} 
}; 

LegCommand initPos = {0, 0};

LegCommand* LEFT_LEG_COMMANDS = COMMANDS[LeftLeg];
LegCommand* RIGHT_LEG_COMMANDS = COMMANDS[RightLeg];

LegServoInfo LEG_SERVOS[2] = {
	{SERVO_1, SERVO_3}, // left {hip, knee}
	{SERVO_2, SERVO_0}  // right {hip, knee}
};

void executeCommand(LegCommand curRight, LegCommand oldRight, LegCommand curLeft, LegCommand oldLeft);
void pause(unsigned short ms);

int main(void)
{	  	 
	int firstRun;
	
	LockoutProtection();
	InitializeMCU();
	
	initUART();			
	initServo();
	
  SetServoPosition(LEG_SERVOS[0].kneeServo, 255);
	SetServoPosition(LEG_SERVOS[0].hipServo, 255);
	SetServoPosition(LEG_SERVOS[1].hipServo, 0);
	SetServoPosition(LEG_SERVOS[1].hipServo, 0);
	
	UARTprintf("\n\nCrawler's gotta crawl!\npress any key to continue...");
	//getc();
	
	firstRun = true;
	
	while(1) {	
		int i;
		
		UARTprintf("\n executing commands!\n");
		
		for (i = 0; i < NUM_COMMANDS; i++) {	
			//getc();
			UARTprintf("   command #%d...\n", i);
			
			if (firstRun) {
				executeCommand(RIGHT_LEG_COMMANDS[i], initPos,
											 LEFT_LEG_COMMANDS[i], initPos);
				firstRun = false;
			} else if (i == 0) {
				executeCommand(RIGHT_LEG_COMMANDS[i], RIGHT_LEG_COMMANDS[NUM_COMMANDS-1],
											 LEFT_LEG_COMMANDS[i], LEFT_LEG_COMMANDS[NUM_COMMANDS-1]);
			} else {
				executeCommand(RIGHT_LEG_COMMANDS[i], RIGHT_LEG_COMMANDS[i-1],
											 LEFT_LEG_COMMANDS[i], LEFT_LEG_COMMANDS[i-1]);
			}
		}
		
		UARTprintf(" commands executed!\n");
	}
}

/*
// accurate to 1 tick for 0 < b < 1676, 0 <= t <= b, abs(p0 - p1) < 255
unsigned char pos_funct(int p0, int p1, int t, int b) {
  int res = 2*(p0 - p1)*t*t/b*t/b/b + 3*(p1 - p0)*t*t/b/b + p0;
	
	if (p1 > p0 && res > p1) {
		res = p1;
	} else if (p1 < p0 && res < p1) {
		res = p1;
	}
	
	return (unsigned char) res;
}
*/

int prevVal = -1;

unsigned char pos_funct(int p0, int p1, int t, int b) {
  int res = getSmoothServoValue(p0, p1, b, t, 0, 0);
	
	if (p1 > p0 && res > p1) {
		res = p1;
	} else if (p1 < p0 && res < p1) {
		res = p1;
	}
	/*
	if (-1 != prevVal) {
		if (p1 > p0 && res < prevVal) {
			res = prevVal;
		} else if (p1 < p0 && res > prevVal) {
			res = prevVal;
		}
	}
	
	prevVal = res;
	*/
	return (unsigned char) res;
}

void stepCommandOnce(servo_t servo, unsigned char p0, unsigned char p1, unsigned short time_ms, unsigned short time_interval) {
	SetServoPosition(servo, pos_funct(p0, p1, time_ms, time_interval));
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
		stepCommandOnce(LEG_SERVOS[RightLeg].hipServo,  old_right_hip,  cur_right_hip,  time_ms, COMMAND_TIME);
		stepCommandOnce(LEG_SERVOS[RightLeg].kneeServo, old_right_knee, cur_right_knee, time_ms, COMMAND_TIME);
		stepCommandOnce(LEG_SERVOS[LeftLeg].hipServo,   old_left_hip,   cur_left_hip,   time_ms, COMMAND_TIME);
		stepCommandOnce(LEG_SERVOS[LeftLeg].kneeServo,  old_left_knee,  cur_left_knee,  time_ms, COMMAND_TIME);
		pause(TIME_INC);
  }
}

void pause(unsigned short ms) {
	unsigned int i, j;
	
	for (j = 0; j < ms; j++) {
		for (i = 0; i < 10000; i++);
	}
}
