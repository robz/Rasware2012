#include "CrawlerBot.h"

/*

Movement.c
Author: Robby Nevels
Date: 3/3/2013

These functions implement smooth motion for servo actions. I have defined an 
'action' to be something that takes the servo from position x0 to position x1.
xi is of the range [0, 255]. 

I define servo movement with a cubic function, where v is the position of the 
servo, and t is time:

v(t) = c3*t^3 + c2*t^2 + c1*t + c0
     = a3 + a2 + a1 + a0

Solve for the coefficients ci by putting constraints on the endpoints of the 
function:

v(0) = x0
v(dt) = x1
v'(0) = s0
v'(dt) = s1

Where v' is the derivative of the function, dt is the range of  the time 
interval that the function is defined on, and s0 and s1 are the slopes of the 
function at the endpoints.

Plug & chug, and we get this:

a0 = x0
a1 = s0*t
a2 = -((2*s0 + s1)*dt + 3*(x0 - x1))*t*t/dt/dt
a3 = (2*(x0 - x1) + dt*(s0 + s1))*t*t/dt*t/dt/dt

From observation I found that the maximum rate that the servo can move is 
about 364 ticks/sec:

*/

#define MAX_SPEED 364

/*

Now I will define four functions for all servo actions using the equations 
above. Here is the definition of a pointer to one of these functions:

*/

typedef void (*FPtr)(
    int x0, int x1, int t, int dt, 
    int* a0, int* a1, int* a2, int* a3);

/*

This first function is for actions that move the servo in an opposite 
direction than the previous action did, and also opposite of the direction of
the action that comes after it. Since the servo is switching direction at 
both of the endpoints, and because we want the thing to move smoothly, we'll 
define the slope at each endpoint to be zero. That way, it will slow down 
for the change of directions.

TLDR: set s1 = s0 = 0

*/

void f00(int x0, int x1, int dt, int t,  
         int* a0, int* a1, int* a2, int* a3) 
{
    *a0 = x0;
    *a1 = 0;
    *a2 = ((-3*(x0 - x1)*t*t)/dt)/dt;
    *a3 = (((2*(x0 - x1)*t*t)/dt)*t)/dt/dt;
}

/*

This next function is for actions that move the servo in the same direction as 
the previous action did, but still opposite of the direction of the action that 
comes after it . So, rather than slowing down at the beginning of the interval, 
we can be going fast since the servo isn't changing direction. 

But how fast? This is a tough question. To perform optimally, the function 
would need to know a lot more about the tick intervals of the previous and 
future actions. To simply things, I just picked a sufficiently large speed. 
I will use this speed from now as for points between actions that take the 
servo in the same direction.

TLDR: set s1 = 0 and s0 = MAX_SPEED 

*/

void f01(int x0, int x1, int dt, int t, 
         int* a0, int* a1, int* a2, int* a3) 
{
    *a0 = x0;
    *a1 = MAX_SPEED*t/1000;
    *a2 = ((-(2*MAX_SPEED*dt/1000 + 3*(x0 - x1))*t*t)/dt)/dt;
    *a3 = ((((MAX_SPEED*dt/1000 + 2*(x0 - x1))*t*t)/dt)*t)/dt/dt;
}

/*

The third function here is exactly like the last one, except with the 
endpoints having opposite behavior.

So, set s1 = MAX_SPEED and s0 = 0 

*/

void f10(int x0, int x1, int dt, int t, 
         int* a0, int* a1, int* a2, int* a3) 
{
    *a0 = x0;
    *a1 = 0;
    *a2 = ((-(MAX_SPEED*dt/1000 + 3*(x0 - x1))*t*t)/dt)/dt;
    *a3 = ((((MAX_SPEED*dt/1000 + 2*(x0 - x1))*t*t)/dt)*t)/dt/dt;
}

/*

This last function is for actions that move the servo in the same direction
as the previous action, as well as the same direction as the next action.

So, set s1 = s0 = MAX_SPEED

*/

void f11(int x0, int x1, int dt, int t, 
         int* a0, int* a1, int* a2, int* a3) 
{
    *a0 = x0;
    *a1 = MAX_SPEED*t/1000;
    *a2 = ((-(3*MAX_SPEED*dt/1000 + 3*(x0 - x1))*t*t)/dt)/dt;
    *a3 = ((((2*MAX_SPEED*dt/1000 + 2*(x0 - x1))*t*t)/dt)*t)/dt/dt;
}

/* 

getSmoothServoValue is our higher-level abstraction. The b0 parameter should be 1 if
the current action is in the same direction as the one before it, or 0 if it 
isn't. The b1 parameter should be 1 if the current action is in the same 
direction as the one after it, or 0 if it isn't.

*/

FPtr functs[4] = {f00, f01, f10, f11};

int getSmoothServoValue(int x0, int x1, int dt, int t, int b0, int b1) {
    int a0, a1, a2, a3;
           
    functs[b0 + (b1 << 1)](x0, x1, dt, t, &a0, &a1, &a2, &a3);

    return (int)(a3 + a2 + a1 + a0);
}
