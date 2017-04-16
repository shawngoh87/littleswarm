#define _POSIX_C_SOURCE 200809L
#define SPEED_OF_SOUND 0.35 // millimeter per microsecond (default 0.3435)
#define DIST_TIMEOUT_MM 1000 // Cutoff at 1 meter
#define ULTRASONIC_TIMEOUT_US 6000 // 2857.14 us x 2
#define WHEEL_DIAMETER_MM 81
#define DEGREE_PER_PULSE 1.5
#define PI 3.14159
#define RESOLUTION ((WHEEL_DIAMETER_MM*PI)/240)
#define ROBOT_DIAMETER_MM 193
#define SENSOR_OFFSET 120
#define DETECT_DISTANCE 300
#define AVOID_ANGLE PI/4

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

#include <mraa.h>
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>
#include "scheduler.h"

static volatile int readCount = 0, odoCount = 0;
unsigned long long t1, t2;
long int currentPositionRight = 0, currentPositionLeft = 0, desiredPositionRight = 0, desiredPositionLeft = 0;
long int errorRight, errorLeft;
float distance = 0;
float distanceArray[20];
unsigned char waitingForEcho = 0, readFlag = 0, robotState = 0;
unsigned char leftComplete=1, rightComplete=1, taskComplete, taskPtr;
unsigned char movementPutPtr=0, movementGetPtr=0, taskPutPtr = 0, taskGetPtr = 0;
unsigned long long timerCount = 0;
float taskCurrent[TASK_LENGTH];
float currentCoordinate[2] = {500,500};
float theta=PI/2;


void moveForward(float distance){
	unsigned int pulse = (distance/RESOLUTION);
	desiredPositionRight = currentPositionRight + pulse;
	desiredPositionLeft = currentPositionLeft + pulse;
}

void pointTurn(float angle, unsigned char direction){
	// Direction: 0: CW, 1: CCW
//	printf("angle: %.2f dir: %d\n", angle, direction);
	float distance = (fabs(angle)/(2*PI))*PI*ROBOT_DIAMETER_MM;
	unsigned int pulse = (distance/RESOLUTION);
	if(direction){//CCW
		desiredPositionRight = currentPositionRight + pulse;
		desiredPositionLeft = currentPositionLeft - pulse;
	}
	else{//CW
		desiredPositionRight = currentPositionRight - pulse;
		desiredPositionLeft = currentPositionLeft + pulse;
	}

}

void updateOdometry(void){
	static float prevR = 0;
	static float prevL = 0;

	float SR = RESOLUTION * (currentPositionRight - prevR);
	float SL = RESOLUTION * (currentPositionLeft - prevL);
	float S = (SR+SL)/2;

	prevR = currentPositionRight;
	prevL = currentPositionLeft;

	currentCoordinate[0] += (S) * cos(theta+S/ROBOT_DIAMETER_MM);
	currentCoordinate[1] += (S) * sin(theta+S/ROBOT_DIAMETER_MM);
	theta += (SR-SL)/(ROBOT_DIAMETER_MM);

	if (theta>6.2832)
		{printf("MORE THAN\n");
		theta -= 6.2832;}
	else if (theta<-6.2832)
		{printf("LESS THAN\n");
		theta+=6.2832;}

//	fprintf(stdout, "XYT: [%.2f,%.2f,%.2f] CR: %d DR: %d CL: %d DL: %d\n", currentCoordinate[0],currentCoordinate[1],theta,currentPositionRight, desiredPositionRight, currentPositionLeft, desiredPositionLeft);
}

void encoder(void *pin){
	/* Motor: 30E-20K, gear ratio 1:20
	 * Encoder resolution: 60 ppr
	 * Encoding method: 4X
	 * Final resolution: 240 ppr*/
	static unsigned char valueRightB,valueRightA,valueLeftB,valueLeftA;
	static unsigned char pstateRight, pstateLeft;
	unsigned char state, pstate;
	unsigned char currentMotor; // 1 = right, 0 = left
	int name = mraa_gpio_get_pin((mraa_gpio_context) pin);
	switch(name){
		case 12:
			valueRightB = mraa_gpio_read((mraa_gpio_context) pin);
			currentMotor = 1;
			break;
		case 11:
			valueRightA = mraa_gpio_read((mraa_gpio_context) pin);
			currentMotor = 1;
			break;
		case 10:
			valueLeftB = mraa_gpio_read((mraa_gpio_context) pin);
			currentMotor = 0;
			break;
		case 9:
			valueLeftA = mraa_gpio_read((mraa_gpio_context) pin);
			currentMotor = 0;
			break;
	}
//	fprintf(stdout, "RightB: ");
//	fprintf(stdout, BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(valueRightB));
//	fprintf(stdout, "\n");
//	fprintf(stdout, "RightA: ");
//	fprintf(stdout, BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(valueRightA));
//	fprintf(stdout, "\n");
//	fprintf(stdout, "RightSTATE: ");
//	fprintf(stdout, BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(stateRight));
//	fprintf(stdout, "\n");
//	fprintf(stdout, "RightPSTATE: ");
//	fprintf(stdout, BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(pstateRight));
//	fprintf(stdout, "\n");
	switch(currentMotor){
		case 1:
			state = (valueRightB << 1)| valueRightA;
			pstate = pstateRight;
			switch(state){
					case 0:
						if (pstate == 1)currentPositionRight++;
						else currentPositionRight--;
						break;
					case 1:
						if (pstate == 3)currentPositionRight++;
						else currentPositionRight--;
						break;
					case 2:
						if (pstate == 0)currentPositionRight++;
						else currentPositionRight--;
						break;
					case 3:
						if (pstate == 2)currentPositionRight++;
						else currentPositionRight--;
						break;
				}
			pstateRight = state;
			break;
		case 0:
			state = (valueLeftB << 1)| valueLeftA;
			pstate = pstateLeft;
			switch(state){
				case 0:
					if (pstate == 1)currentPositionLeft--;
					else currentPositionLeft++;
					break;
				case 1:
					if (pstate == 3)currentPositionLeft--;
					else currentPositionLeft++;
					break;
				case 2:
					if (pstate == 0)currentPositionLeft--;
					else currentPositionLeft++;
					break;
				case 3:
					if (pstate == 2)currentPositionLeft--;
					else currentPositionLeft++;
					break;
			}
			pstateLeft = state;
			break;
	}


//	fprintf(stdout, "CR: %d CL: %d DR: %d DL: %d\n", currentPositionRight, currentPositionLeft, desiredPositionRight, desiredPositionLeft);
}

void listenEcho(void* pin){
	struct timespec tspec;
	unsigned long long us;
	unsigned long long diff;
	float dist;
	int value = mraa_gpio_read((mraa_gpio_context) pin);

	clock_gettime(CLOCK_REALTIME, &tspec);
	us = (unsigned long long)(tspec.tv_nsec / 1.0e3) + (tspec.tv_sec * 1000000);

	if(!readFlag){
		if (value == 1){
			// RISING EDGE
	//		fprintf(stdout, "RISING\n");
			t1 = us;
			timerCount = 1;
			setTimer(ULTRASONIC_TIMEOUT_US);
		}
		else if (value == 0){
			// FALLING EDGE
	//		fprintf(stdout, "FALLING\n");
			t2 = us;
			diff = t2 - t1;
			dist = (float)(diff/2 * SPEED_OF_SOUND);
			distance = dist;
			distanceArray[readCount] = dist;
			readCount++;
			if (readCount >= 20) {
				readFlag = 1;
				readCount = 0;
			}
//			fprintf(stdout, "Took: %llu us Dist: %.2f mm\n", diff, dist); // ALWAYS USE \n to flush buffer
			waitingForEcho = 0;
		}
	}
}

void pulse(void *trigger){
//	fprintf(stdout, "TRIGGERED\n");
	mraa_gpio_write((mraa_gpio_context) trigger, 0);
	usleep(5);
	mraa_gpio_write((mraa_gpio_context) trigger, 1);
	usleep(15);
	mraa_gpio_write((mraa_gpio_context) trigger, 0);
	waitingForEcho = 1;
}	

int compare(const void *a, const void *b)
{
    double aa = *(double*)a, bb = *(double*)b;

    if (aa > bb) return -1;
    if (aa < bb) return 1;

    return 0;
}

void timer_handler (int signum)
{
	static int count = 0;
//	fprintf (stdout, "timer expired %d times\n", ++count);
	if (count >= timerCount){ // timerCount sometimes doesn't reset due to ISR, ALWAYS use >=
		setTimer(0);
		count = 0;
	}
	switch(signum){
		case 26:
			waitingForEcho = 0;
			break;
	}
}

void setTimer(long us){
	struct sigaction sa;
	struct itimerval timer;

	/* Install timer_handler as the signal handler for SIGVTALRM. */
	memset(&sa, 0, sizeof (sa));
	sa.sa_handler = &timer_handler;
	sigaction (SIGVTALRM, &sa, NULL);
		 /* Configure the timer to expire after 250 msec... */
	timer.it_value.tv_sec = 0;
	timer.it_value.tv_usec = us;
	/* ... and every 250 msec after that. */
	timer.it_interval.tv_sec = 0;
	timer.it_interval.tv_usec = us;
	/* Start a virtual timer. It counts down whenever this process is
	  executing. */
	setitimer (ITIMER_VIRTUAL, &timer, NULL);
}

int main()
{
	// Set pin
	mraa_gpio_context led = mraa_gpio_init(13);
	mraa_gpio_context rightB = mraa_gpio_init(12);
	mraa_gpio_context rightA = mraa_gpio_init(11);
	mraa_gpio_context leftB = mraa_gpio_init(10);
	mraa_gpio_context leftA = mraa_gpio_init(9);
	mraa_gpio_context trig = mraa_gpio_init(7);
	mraa_gpio_context echo = mraa_gpio_init(6);
	mraa_pwm_context rightPWM = mraa_pwm_init(5);
	mraa_gpio_context rightDir = mraa_gpio_init(4);
	mraa_pwm_context leftPWM = mraa_pwm_init(3);
	mraa_gpio_context leftDir = mraa_gpio_init(2);

	// Set direction
	mraa_gpio_dir(led, MRAA_GPIO_OUT);
	mraa_gpio_dir(rightB, MRAA_GPIO_IN);
	mraa_gpio_dir(rightA, MRAA_GPIO_IN);
	mraa_gpio_dir(trig, MRAA_GPIO_OUT);
	mraa_gpio_dir(echo, MRAA_GPIO_IN);
	mraa_gpio_dir(leftB, MRAA_GPIO_IN);
	mraa_gpio_dir(leftA, MRAA_GPIO_IN);
	mraa_gpio_dir(rightDir, MRAA_GPIO_OUT);
	mraa_gpio_dir(leftDir, MRAA_GPIO_OUT);

	// ISR mapping
	mraa_gpio_edge_t edgeISR = MRAA_GPIO_EDGE_BOTH;
	mraa_gpio_isr(echo, edgeISR, &listenEcho, echo);
	mraa_gpio_isr(rightA, edgeISR, &encoder, rightA);
	mraa_gpio_isr(rightB, edgeISR, &encoder, rightB);
	mraa_gpio_isr(leftA, edgeISR, &encoder, leftA);
	mraa_gpio_isr(leftB, edgeISR, &encoder, leftB);

	// PWM config
	mraa_pwm_period_us(leftPWM, 200);
	mraa_pwm_period_us(rightPWM, 200);
	mraa_pwm_enable(leftPWM, 0);
	mraa_pwm_enable(rightPWM, 0);

//	PWM Guide:
//	Left Motor:
//	CCW: dir = 0, CW: dir = 1
//	Right Motor:
//	CCW: dir = 1, CW: dir = 0
//	Forward: leftDir = 0, rightDir = 0;
//	Backward: leftDir = 1, rightDir = 1;

//	float arr[3] = {0,1,2};
//	float arr2[3];
//	if (taskPut(arr,3) == 0){
//		fprintf(stdout,"SUCCESS\n");
//		fprintf(stdout, "INSERTED: %.2f %.2f %.2f\n", task[0][0], task[0][1], task[0][2]);
//		fprintf(stdout, "PUTPTR: %d\n", taskPutPtr);
//		fprintf(stdout, "GETPTR: %d\n", taskGetPtr);
//	}
//	if (taskGet(arr2,3) == 0){
//		fprintf(stdout,"SUCCESS\n");
//		fprintf(stdout, "INSERTED: %.2f %.2f %.2f\n", arr2[0], arr2[1], arr2[2]);
//		fprintf(stdout, "PUTPTR: %d\n", taskPutPtr);
//		fprintf(stdout, "GETPTR: %d\n", taskGetPtr);
//	}
//	if (taskPut(arr,3) == 0){
//		fprintf(stdout,"SUCCESS\n");
//		fprintf(stdout, "INSERTED: %.2f %.2f %.2f\n", task[1][0], task[1][1], task[1][2]);
//		fprintf(stdout, "PUTPTR: %d\n", taskPutPtr);
//		fprintf(stdout, "GETPTR: %d\n", taskGetPtr);
//	}
//	if (taskPut(arr,3) == 0){
//		fprintf(stdout,"SUCCESS\n");
//		fprintf(stdout, "INSERTED: %.2f %.2f %.2f\n", task[2][0], task[2][1], task[2][2]);
//		fprintf(stdout, "PUTPTR: %d\n", taskPutPtr);
//		fprintf(stdout, "GETPTR: %d\n", taskGetPtr);
//	}

//	float arr1[3] = {0,100,0};
//	taskPut(arr1, 3);
//	float arr2[3] = {0,100,0};
//	taskPut(arr2, 3);
//	float arr3[3] = {0,100,0};
//	taskPut(arr3, 3);
//	float arr4[3] = {0,100,0};
//	taskPut(arr4, 3);
	float arr1[2] = {500,1500};
	movementPut(arr1, 2);
//	float arr2[2] = {660,330};
//	movementPut(arr2, 2);
//	float arr3[2] = {330,0};
//	movementPut(arr3, 2);
//	float arr4[2] = {0,0};
//	movementPut(arr4, 2);

	// DEBUG ZONE

	// DEBUG ZONE END

	// Loop
	while (1){
		float nextMove[2];
		for (j=GRID_SIZE-1;j>=0;j--){
			for (i=0;i<GRID_SIZE;i++){
				fprintf(stdout,"%d ", cellState[i][j]);
			}
			fprintf(stdout, "\n");
		}
		if (movementGet(nextMove, 2) == 0){
			float dx = nextMove[0] - currentCoordinate[0];
			float dy = nextMove[1] - currentCoordinate[1];
			float length = sqrt(pow(fabs(dx),2) + pow(fabs(dy),2));
			float angle = atan(fabs(dy)/fabs(dx));
			float temp, taskArray[3];

			if (dx != 0 ){
				if (dx > 0){
					if (dy < 0){
						angle = 2*PI - angle;
					}
				}
				else{
					if (dy > 0){
						angle = PI - angle;
					}
					else{
						angle = PI + angle;
					}
				}
			}
			else {
				if (dy > 0){
					angle = PI/2;
				}
				else {
					angle = 3*PI/2;
				}
			}
			temp = angle-theta;
			if (temp){
				if(temp>0){
					if(temp<PI){
						taskArray[2] = 1; //CCW
						taskArray[1] = temp;
					}
					else if (temp>PI){
						taskArray[2] = 0; //CW
						taskArray[1] = 2*PI-temp;
					}
				}
				if(temp<0){
					temp=fabs(temp);
					if(temp<PI){
						taskArray[2] = 0; //CW
						taskArray[1] = temp;
					}
					else if (temp>PI){
						taskArray[2] = 1; //CCW
						taskArray[1] = 2*PI-temp;
					}
				}
				taskArray[0] = 1; //Point turn task
//				printf("ANGLE TO BE TURNED: %.2f\n", temp);
				taskPut(taskArray, TASK_LENGTH);
			}

			if(length){
				float taskArray[3] = {0,length,0};
				taskPut(taskArray, TASK_LENGTH);
			}
		}
		else{
			fprintf(stdout, "EMPTY MOVEMENT LIST\n");
			mraa_pwm_enable(leftPWM, 0);
			mraa_pwm_enable(rightPWM, 0);
			return MRAA_SUCCESS;
		}

		while (taskGet(taskCurrent,TASK_LENGTH) == 0){ // Has task

			switch((int)taskCurrent[0]){
				case 0: // Format: [0,distance,_]
					moveForward(taskCurrent[1]);
					leftComplete = rightComplete = taskComplete = 0;
					mraa_pwm_write(leftPWM, 0.4);
					mraa_pwm_write(rightPWM, 0.4);
					fprintf(stdout, "Moving forward\n");
					break;
				case 1: // Format: [1,angle,directionCCW]
					pointTurn(taskCurrent[1], taskCurrent[2]);
					leftComplete = rightComplete = taskComplete = 0;
					mraa_pwm_write(leftPWM, 0.4);
					mraa_pwm_write(rightPWM, 0.4);
					fprintf(stdout, "Turning pivot\n");
					break;
			}

			usleep(500000);
			while(taskComplete == 0){
				errorLeft = desiredPositionLeft - currentPositionLeft;
				errorRight = desiredPositionRight - currentPositionRight;
				if (leftComplete == 0){
					if (errorLeft > 0){
						mraa_pwm_enable(leftPWM, 1);
						mraa_gpio_write(leftDir, 0);
					}
					if (errorLeft < 0){
						mraa_pwm_enable(leftPWM, 1);
						mraa_gpio_write(leftDir, 1);
					}
					if (errorLeft == 0){
						mraa_pwm_enable(leftPWM, 0);
						leftComplete = 1;
//						fprintf(stdout, "LEFT COMPLETE\n");
					}
				}
				if (rightComplete == 0){
					if (errorRight > 0){
						mraa_pwm_enable(rightPWM, 1);
						mraa_gpio_write(rightDir, 0);
					}
					if (errorRight < 0){
						mraa_pwm_enable(rightPWM, 1);
						mraa_gpio_write(rightDir, 1);
					}
					if (errorRight == 0){
						mraa_pwm_enable(rightPWM, 0);
						rightComplete = 1;
//						fprintf(stdout, "RIGHT COMPLETE\n");
					}
				}
				pulse(trig);
				if((leftComplete==1) && (rightComplete==1)){
					taskComplete = 1;
					updateOdometry();
					fprintf(stdout, "TASK COMPLETE, XYT:[%.2f,%.2f,%.2f]\n", currentCoordinate[0], currentCoordinate[1], theta);
				}
				odoCount++;
				if (odoCount % 15 == 0)
					updateOdometry();
				if (readFlag){
					switch(robotState){
						case 0: // normal movement
							qsort(distanceArray, 20, sizeof(distanceArray[0]), compare);
							float avg = 0;
							int i;
							for (i = 5; i < 15; i++){
								avg += distanceArray[i];
							}
							avg = avg/10;
//							printf("AVERAGE: %.2f\n", avg);
							if (avg<(DETECT_DISTANCE-SENSOR_OFFSET)){
		//						if (avg < (SENSOR_OFFSET + 50)){
								mraa_pwm_enable(leftPWM, 0);
								mraa_pwm_enable(rightPWM, 0);
								fprintf(stdout, "DETECTED, XYT:[%.2f,%.2f,%.2f]\n", currentCoordinate[0], currentCoordinate[1], theta);
								float objX, objY;
								objX = currentCoordinate[0] + (avg+SENSOR_OFFSET)*cos(theta);
								objY = currentCoordinate[1] + (avg+SENSOR_OFFSET)*sin(theta);
								fprintf(stdout, "OBJECT, XY:[%.2f,%.2f]\n", objX, objY);

								robotState = 1;
							}
							else{robotState = 0;}
							break;
						case 1: // go around the object;
							taskComplete = 1;
							taskGetPtr = taskPutPtr;
							float avoidX, avoidY, recoverX, recoverY, L;
							L = avg/(cos(AVOID_ANGLE));
							avoidX = currentCoordinate[0]+L*cos(theta+AVOID_ANGLE);
							avoidY = currentCoordinate[1]+L*sin(theta+AVOID_ANGLE);
							recoverX = nextMove[0];
							recoverY = nextMove[1];
							float arr5[2] = {avoidX, avoidY};
							movementPut(arr5, 2);
							float arr6[2] = {recoverX, recoverY};
							movementPut(arr6, 2);
//							robotState = 0;
							break;
					}
					readFlag = 0;
				}

			}
		}
	}
	return MRAA_SUCCESS;
}

// LEGACY CODE: Multiple POSIX timer
//
//
//timer_t firstTimerID;
//timer_t secondTimerID;
//timer_t thirdTimerID;
//makeTimer(&firstTimerID, 2, 2); //2ms
//makeTimer(&secondTimerID, 10, 10); //10ms
//makeTimer(&thirdTimerID, 100, 100); //100ms
//
//static void timerHandler( int sig, siginfo_t *si, void *uc )
//{
//    timer_t *tidp;
//
//    tidp = si->si_value.sival_ptr;
//
//    if ( *tidp == firstTimerID )
//        fprintf(stdout, "2ms\n");
//    else if ( *tidp == secondTimerID )
//        fprintf(stdout, "10ms\n");
//    else if ( *tidp == thirdTimerID )
//        fprintf(stdout, "100ms\n");
//}
//
//static int makeTimer( timer_t *timerID, int expireMS, int intervalMS )
//{
//    struct sigevent te;
//    struct itimerspec its;
//    struct sigaction sa;
//    int sigNo = SIGRTMIN;
//
//    /* Set up signal handler. */
//    sa.sa_flags = SA_SIGINFO;
//    sa.sa_sigaction = timerHandler;
//    sigemptyset(&sa.sa_mask);
//    if (sigaction(sigNo, &sa, NULL) == -1) {
//        perror("sigaction");
//    }
//
//    /* Set and enable alarm */
//    te.sigev_notify = SIGEV_SIGNAL;
//    te.sigev_signo = sigNo;
//    te.sigev_value.sival_ptr = timerID;
//    timer_create(CLOCK_REALTIME, &te, timerID);
//
//    its.it_interval.tv_sec = 0;
//    its.it_interval.tv_nsec = intervalMS * 1000000;
//    its.it_value.tv_sec = 0;
//    its.it_value.tv_nsec = expireMS * 1000000;
//    timer_settime(*timerID, 0, &its, NULL);
//
//    return 1;
//}

