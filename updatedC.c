#define _POSIX_C_SOURCE 200809L
#define ULTRASONIC_TIMEOUT_US 100000
#define SPEED_OF_SOUND 0.35 // millimeter per microsecond (default 0.3435)
#define DIST_LIMIT 150

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

#define leftB 11
#define leftA 10

#include <mraa.h>
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>

static volatile int edge = 0; // Start with echo = LOW
static volatile int counter = 0;
unsigned long long t1;
unsigned long long t2;
unsigned int currentPosition = 0;




void encoder(void *pin){
	/* Motor: 30E-20K, gear ratio 1:20
	 * Encoder resolution: 60 ppr
	 * Encoding method: 4X
	 * Final resolution: 240 ppr*/
	static unsigned char valueB;
	static unsigned char valueA;
	static unsigned char pstate;
	int name = mraa_gpio_get_pin((mraa_gpio_context) pin);
	if (name == leftB){
		valueB = mraa_gpio_read((mraa_gpio_context) pin);
	}
	if (name == leftA){
		valueA = mraa_gpio_read((mraa_gpio_context) pin);
	}

	unsigned char state = (valueB << 1)| valueA;
//	fprintf(stdout, "B: ");
//	fprintf(stdout, BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(valueB));
//	fprintf(stdout, "\n");
//	fprintf(stdout, "A: ");
//	fprintf(stdout, BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(valueA));
//	fprintf(stdout, "\n");
//	fprintf(stdout, "STATE: ");
//	fprintf(stdout, BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(state));
//	fprintf(stdout, "\n");
//	fprintf(stdout, "PSTATE: ");
//	fprintf(stdout, BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(pstate));
//	fprintf(stdout, "\n");

	switch(state){
		case 0:
			if (pstate == 1)currentPosition++;
			else currentPosition--;
			break;
		case 1:
			if (pstate == 3)currentPosition++;
			else currentPosition--;
			break;
		case 2:
			if (pstate == 0)currentPosition++;
			else currentPosition--;
			break;
		case 3:
			if (pstate == 2)currentPosition++;
			else currentPosition--;
			break;
	}

	fprintf(stdout, "Current Position: %d\n", currentPosition);

	pstate = state;
}

void listenEcho(void* args){
	struct timespec tspec;
	unsigned long long us;
	unsigned long long diff;
	float dist;

	clock_gettime(CLOCK_REALTIME, &tspec);
	us = (unsigned long long)(tspec.tv_nsec / 1.0e3) + (tspec.tv_sec * 1000000);

	if (edge == 0){ // Falling edge
		t2 = us;
		diff = t2 - t1;
		dist = (float)(diff/2 * SPEED_OF_SOUND);
		counter = dist;
		if (dist > DIST_LIMIT){
			fprintf(stdout, "OUT OF RANGE\n"); // ALWAYS USE \n to flush buffer
		}
		else {
			fprintf(stdout, "Took: %llu us Dist: %.2f mm\n", diff, dist); // ALWAYS USE \n to flush buffer
		}
		edge = 1;
	}
	else if (edge == 1){ // Rising edge
		t1 = us;
		edge = 0;
	}
}

void pulse(*trigger){
	mraa_gpio_write((mraa_gpio_context) trigger, 0);
	usleep(1);
	mraa_gpio_write((mraa_gpio_context) trigger, 1);
	usleep(15);
	mraa_gpio_write((mraa_gpio_context) trigger, 0);
}	

int main()
{
	mraa_gpio_context led = mraa_gpio_init(13);
	mraa_gpio_context channelB = mraa_gpio_init(leftB);
	mraa_gpio_context channelA = mraa_gpio_init(leftA);
	mraa_gpio_context trig = mraa_gpio_init(9);
	mraa_gpio_context echo = mraa_gpio_init(8);
	mraa_gpio_context x = mraa_gpio_init(7);
	mraa_gpio_context y = mraa_gpio_init(6);
	
	mraa_gpio_dir(led, MRAA_GPIO_OUT);
	mraa_gpio_dir(channelB, MRAA_GPIO_IN);
	mraa_gpio_dir(channelA, MRAA_GPIO_IN);
	mraa_gpio_dir(trig, MRAA_GPIO_OUT);
	mraa_gpio_dir(echo, MRAA_GPIO_IN);
	mraa_gpio_dir(x, MRAA_GPIO_IN);
	mraa_gpio_dir(y, MRAA_GPIO_IN);

	mraa_gpio_edge_t edgeISR = MRAA_GPIO_EDGE_BOTH;
	
	mraa_gpio_isr(echo, edgeISR, &listenEcho, NULL);
	mraa_gpio_isr(channelA, edgeISR, &encoder, channelA);
	mraa_gpio_isr(channelB, edgeISR, &encoder, channelB);


	for (;;) {
//		pulse(trig);

	}

	return MRAA_SUCCESS;
}



//void encoder(void *pin){
//	unsigned char saa;
//	pin->read();
//
//	printf("%d", saa);
//
//
//}
//
//void interrupt(void *args){
//	struct timespec tspec;
//	unsigned long long us;
//	unsigned long long diff;
//	float dist;
//
//	clock_gettime(CLOCK_REALTIME, &tspec);
//	us = round(tspec.tv_nsec / 1.0e3) + (tspec.tv_sec * 1000000);
//
//	if (edge == 0){ // Falling edge
//		t2 = us;
//		diff = t2 - t1;
//		dist = (float)(diff/2 * SPEED_OF_SOUND);
//		counter = dist;
//		if (dist > DIST_LIMIT){
//			fprintf(stdout, "OUT OF RANGE\n"); // ALWAYS USE \n to flush buffer
//		}
//		else {
//			fprintf(stdout, "Took: %llu us Dist: %.2f mm\n", diff, dist); // ALWAYS USE \n to flush buffer
//		}
//		edge = 1;
//	}
//	else if (edge == 1){ // Rising edge
//		t1 = us;
//		edge = 0;
//	}
//}
//
//int main()
//{
//	mraa::Gpio* led = new mraa::Gpio(13);
//	mraa::Gpio* channelB = new mraa::Gpio(11);
//	mraa::Gpio* channelA = new mraa::Gpio(10);
//	mraa::Gpio* echo = new mraa::Gpio(8);
//	mraa::Gpio* trig = new mraa::Gpio(9);
//	channelB -> dir(mraa::DIR_IN);
//	channelA -> dir(mraa::DIR_IN);
//	led -> dir(mraa::DIR_OUT);
//	echo -> dir(mraa::DIR_IN);
//	trig -> dir(mraa::DIR_OUT);

//	echo -> isr(mraa::EDGE_BOTH, &interrupt, NULL);
//	channelB -> isr(mraa::EDGE_BOTH, &encoder, &channelB);
//	channelA -> isr(mraa::EDGE_BOTH, &encoder, &channelA);
//
//	for (;;){
////		trig ->write(1); // Trigger pulse of 15us
////		usleep(15);
////		trig ->write(0);
////
////		usleep(500000);
////		if (counter <DIST_LIMIT){
////			led->write(1);
////		}
////		else{
////			led->write(0);
////		}
//		led -> write(1);
//		sleep(1);
//		led -> write(0);
//		sleep(1);
//
//	}
//
//	return mraa::SUCCESS;
//}
//
//
////	struct timespec tspec;
////	unsigned long long us;
////	clock_gettime(CLOCK_REALTIME, &tspec);
////	us = round(tspec.tv_nsec / 1.0e3) + (tspec.tv_sec * 1000000);
////	fprintf(stdout, "D: %llu \n", us);
////	counter = 1;
//
////	unsigned char state;
////	static unsigned char pstate;
