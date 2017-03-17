#define _POSIX_C_SOURCE 200809L
#define ULTRASONIC_TIMEOUT_US 100000
#define SPEED_OF_SOUND 0.3435 // millimeter per microsecond

#include <mraa.hpp>
#include <time.h>
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <inttypes.h>

using namespace std;
using namespace mraa;

static volatile int edge = 0; // Start with echo = LOW
static volatile int counter = 0;

unsigned long long t1;
unsigned long long t2;


void interrupt(void *args){
	struct timespec tspec;
	unsigned long long us;
	unsigned long long diff;
	float dist;

	clock_gettime(CLOCK_REALTIME, &tspec);
	us = round(tspec.tv_nsec / 1.0e3) + (tspec.tv_sec * 1000000);
	++counter;

	if (edge == 0){ // Falling edge
		t2 = us;
		diff = t2 - t1;
		dist = (float)(diff/2 * SPEED_OF_SOUND);
		fprintf(stdout, "Took: %llu us Dist: %.2f mm\n", diff, dist); // ALWAYS USE \n to flush buffer
		edge = 1;
	}
	else if (edge == 1){ // Rising edge
		t1 = us;
		edge = 0;
	}
}

int main()
{
	mraa::Gpio* led = new mraa::Gpio(13);
	mraa::Gpio* echo = new mraa::Gpio(8);
	mraa::Gpio* trig = new mraa::Gpio(9);
	led -> dir(mraa::DIR_OUT);
	echo -> dir(mraa::DIR_IN);
	trig -> dir(mraa::DIR_OUT);
	echo -> isr(mraa::EDGE_BOTH, &interrupt, NULL);

	for (;;){
		trig ->write(1); // Trigger pulse of 15us
		usleep(15);
		trig ->write(0);

		usleep(500000);
	}

	return mraa::SUCCESS;
}

