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
#include "planner.h"

#define ROBOT_DIAMETER_MM 158.5 // BOT 1: 178-180 (Actual -> 193)    BOT 2: 158 (Actual -> 158) Tuned for point turning
#define WHEEL_DIAMETER_MM 85.5 // BOT 1: 81     BOT 2: 85.5
#define SCAN_COUNT 20 // Number of discrete stationary scans in a circle
#define _POSIX_C_SOURCE 200809L
#define SPEED_OF_SOUND 0.35 // millimeter per microsecond (default 0.3435)
#define DIST_TIMEOUT_MM 1000 // Cutoff at 1 meter
#define ULTRASONIC_TIMEOUT_US 3000 // 2857.14 us x 2
#define DEGREE_PER_PULSE 1.5
#define PI 3.1415927
#define RESOLUTION ((WHEEL_DIAMETER_MM*PI)/240)
#define L_PWM 0.5 // BOT 1: 0.4   BOT 2: 0.415 (0.39-0.45)
#define R_PWM 0.5 // BOT 1: 0.4   BOT 2: 0.5
#define SENSOR_OFFSET 160
#define DETECT_DISTANCE sqrt(2*(GRID_LENGTH/2)*(GRID_LENGTH/2))+30
#define DETECT_FILTER_ARRAY_SIZE 8
#define DETECT_FILTER_CUTOFF_SIZE 2

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


static volatile int readCount = 0, odoCount = 0;
unsigned long long t1, t2;
long int currentPositionRight = 0, currentPositionLeft = 0, desiredPositionRight = 0, desiredPositionLeft = 0;
long int errorRight, errorLeft;
float distance = 0, distanceArray[DETECT_FILTER_ARRAY_SIZE], distAVG;
float obstacleArray[10][2], obstacleArrayPtr = 0;
unsigned char waitingForEcho = 0, readFlag = 0, robotState, scanOnCompletion, robotRole=0;
unsigned char leftComplete, rightComplete, scanComplete, taskComplete, taskPtr;
unsigned char movementPutPtr=0, movementGetPtr=0, taskPutPtr = 0, taskGetPtr = 0;
unsigned long long timerCount = 0;
float targetTheta, currentPWML = L_PWM, currentPWMR = R_PWM;
float taskCurrent[TASK_LENGTH];
float currentCoordinate[2] = {GRID_LENGTH*START_X,GRID_LENGTH*START_Y};
float theta=PI/2;
/* PID test variables */
float err0, err1=0, err2=0, err3=0, errDiff=0, errSum=0, output;

void roleSwap(){
	if(!robotRole){ // If robot is explorer

	}
}

void moveForward(float distance){
	unsigned int pulse = (distance/RESOLUTION);
//	printf("DISTANCE: %.2f\n", distance);
	desiredPositionRight = currentPositionRight + pulse;
	desiredPositionLeft = currentPositionLeft + pulse;
}

void pointTurn(float angle, unsigned char direction){
	// Direction: 0: CW, 1: CCW
//	printf("ANGLE: %.2f DIR: %d\n", angle, direction);
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
		theta -= 6.2832;
	else if (theta<-6.2832)
		theta+=6.2832;

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
			if (readCount >= DETECT_FILTER_ARRAY_SIZE) {
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
			printf("TIMER UP\n");
			distanceArray[readCount] = 1100; // Limited distance
			readCount++;
			if (readCount >= DETECT_FILTER_ARRAY_SIZE) {
				readFlag = 1;
				readCount = 0;
			}
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

void setDirection(float dx, float dy){
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
//	printf("DISTANCE: %.2f\n", length);
//	printf("ANGLE: %.2f\n", temp);
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

//	float arr1[3] = {0,100,0};
//	taskPut(arr1, 3);
//	float arr2[3] = {0,100,0};
//	taskPut(arr2, 3);
//	float arr3[3] = {0,100,0};
//	taskPut(arr3, 3);
//	float arr4[3] = {0,100,0};
//	taskPut(arr4, 3);
//	float arr1[2] = {0,990};
//	movementPut(arr1, 2);
//	float arr2[2] = {1490,1490};
//	movementPut(arr2, 2);
//	float arr3[2] = {330,0};
//	movementPut(arr3, 2);
//	float arr4[2] = {0,0};
//	movementPut(arr4, 2);

	/* DEBUG ZONE */
//	updateObstacle(440.5536, 350.3241, 0, 1);
//	while(1);
	/* DEBUG ZONE END */

	// Loop
	initGrid();
	srand(time(NULL));
	while (1){
		float nextMove[2];
		int *gridGet;
		if (movementGet(nextMove,2) == -1){
			printf("A\n");
			if (!scanOnCompletion){ // If not avoiding obstacles
//				printf("CURRENT: X: %.2f Y: %.2f\n", currentCoordinate[0], currentCoordinate[1]);
				updateMap((int)round(currentCoordinate[0]/GRID_LENGTH),(int)round(currentCoordinate[1]/GRID_LENGTH), 1);
				gridGet = planGrid((int)round(currentCoordinate[0]/GRID_LENGTH),(int)round(currentCoordinate[1]/GRID_LENGTH));
				updateMap((int)round(gridGet[0]/GRID_LENGTH),(int)round(gridGet[1]/GRID_LENGTH), 0);
				viewGrid(0);
				if (gridGet[0] != -1){ // Not flocking
					scanOnCompletion = SCAN_COUNT;
					nextMove[0] = (float)gridGet[0];
					nextMove[1] = (float)gridGet[1];
					movementPut(nextMove, 2);
					printf("NEXT COORDINATE: %.2f, %.2f\n", nextMove[0], nextMove[1]);
				}
				else{
					// Flocking code
					scanOnCompletion = 0;
				}

				if (movementGet(nextMove, 2) == 0){
					float dx = nextMove[0] - currentCoordinate[0];
					float dy = nextMove[1] - currentCoordinate[1];
					setDirection(dx, dy);
				}
				else{
					fprintf(stdout, "EMPTY MOVEMENT LIST\n");
					mraa_pwm_enable(leftPWM, 0);
					mraa_pwm_enable(rightPWM, 0);
					return MRAA_SUCCESS;
				}
			}
			else { // Scanning
				int k;
				for (k = 0;k < SCAN_COUNT; k++){
					float arr[3] = {2,PI/(SCAN_COUNT/2),0};
					taskPut(arr, 3);
				}
			}
		}
		else{
			float dx = nextMove[0] - currentCoordinate[0];
			float dy = nextMove[1] - currentCoordinate[1];
			setDirection(dx, dy);
		}



		while (taskGet(taskCurrent,TASK_LENGTH) == 0){ // Has task
			printf("B\n");
			switch((int)taskCurrent[0]){
				case 0: // Format: [0,distance,_]
					moveForward(taskCurrent[1]);
					robotState = 0;
					targetTheta = theta; // latch current theta for control reference
					leftComplete = rightComplete = taskComplete = 0;
					mraa_pwm_write(leftPWM, L_PWM);
					mraa_pwm_write(rightPWM, R_PWM);
//					fprintf(stdout, "Moving forward\n");
					break;
				case 1: // Format: [1,angle,directionCCW] Point turning
					pointTurn(taskCurrent[1], taskCurrent[2]);
					robotState = 1;
					leftComplete = rightComplete = taskComplete = 0;
					mraa_pwm_write(leftPWM, L_PWM-0.1);
					mraa_pwm_write(rightPWM, R_PWM-0.1);
//					fprintf(stdout, "Turning pivot\n");
					break;
				case 2: // Format: [2,angle,directionCCW] Point turning + Scanning
					pointTurn(taskCurrent[1], taskCurrent[2]);
					robotState = 2;
					leftComplete = rightComplete = scanComplete = taskComplete = 0;
					mraa_pwm_write(leftPWM, L_PWM-0.1);
					mraa_pwm_write(rightPWM, R_PWM-0.1);
//					fprintf(stdout, "Turning pivot and scan\n");
					break;
			}
			err3 = err2 = err1 = err0 = 0;
			currentPWML = L_PWM;
			currentPWMR = R_PWM;

			usleep(500000);

			mraa_pwm_enable(rightPWM, 0);
			mraa_pwm_enable(leftPWM, 0);
			printf("C\n");
			switch(robotState){
			case 0:
				while(!taskComplete){
					// Motion loop
					errorLeft = desiredPositionLeft - currentPositionLeft;
					errorRight = desiredPositionRight - currentPositionRight;

					//DEBUG
					if(!leftComplete) mraa_pwm_enable(leftPWM, 1);
					else {
						mraa_pwm_enable(leftPWM, 0);
						leftComplete = 1;
					}
					if(!rightComplete) mraa_pwm_enable(rightPWM, 1);
					else {
						mraa_pwm_enable(rightPWM, 0);
						rightComplete = 1;
					}
					//LEFT
					if (errorLeft > 1) mraa_gpio_write(leftDir, 0);
					else if (errorLeft < -1) mraa_gpio_write(leftDir, 1);
					else {
						mraa_pwm_enable(leftPWM, 0);
						leftComplete = 1;
	//						fprintf(stdout, "LEFT COMPLETE\n");
					}
					//RIGHT
					if (errorRight > 1) mraa_gpio_write(rightDir, 0);
					else if (errorRight < -1) mraa_gpio_write(rightDir, 1);
					else {
						mraa_pwm_enable(rightPWM, 0);
						rightComplete = 1;
	//						fprintf(stdout, "RIGHT COMPLETE\n");
					}
					//DEBUG END

//					if (!leftComplete){
//						if (errorLeft > 1){
//							mraa_pwm_enable(leftPWM, 1);
//							mraa_gpio_write(leftDir, 0);
//						}
//						else if (errorLeft < -1){
//							mraa_pwm_enable(leftPWM, 1);
//							mraa_gpio_write(leftDir, 1);
//						}
//						else {
//							mraa_pwm_enable(leftPWM, 0);
//							leftComplete = 1;
//	//						fprintf(stdout, "LEFT COMPLETE\n");
//						}
//					}
//					if (!rightComplete){
//						if (errorRight > 1){
//							mraa_pwm_enable(rightPWM, 1);
//							mraa_gpio_write(rightDir, 0);
//						}
//						else if (errorRight < -1){
//							mraa_pwm_enable(rightPWM, 1);
//							mraa_gpio_write(rightDir, 1);
//						}
//						else {
//							mraa_pwm_enable(rightPWM, 0);
//							rightComplete = 1;
//	//						fprintf(stdout, "RIGHT COMPLETE\n");
//						}
//					}

					if((leftComplete) && (rightComplete)){
						taskComplete = 1;
						updateOdometry();
						fprintf(stdout, "TASK COMPLETE, XYT:[%.2f,%.2f,%.4f], L: %d R: %d\n", currentCoordinate[0], currentCoordinate[1], theta, currentPositionLeft, currentPositionRight);
					}

					// Odometry update loop
					odoCount++;
					if (odoCount % 7 == 0)
						updateOdometry();

					// Proportional-Derivative control
//					if (odoCount % 10 == 0){
//						float scale = 0.01, P = 3, D = 2; // Proportional and derivative constants
//						err0 = fabs(targetTheta) - fabs(theta); // Proportional error
//						errDiff = err3 - err0; // Derivative error
//						output = P*err0 + D*errDiff; // Total error
//						if (output<0){
//							output = (-output);
//							currentPWML += scale*output;
//							currentPWMR -= scale*output;
//						}
//						else if (output>0){
//							currentPWML -= scale*output;
//							currentPWMR += scale*output;
//						}
//						if (currentPWML > 0.65) currentPWML = 0.65;
//						else if (currentPWML < 0.35) currentPWML = 0.35;
//						if (currentPWMR > 0.65) currentPWMR = 0.65;
//						else if (currentPWMR < 0.35) currentPWMR = 0.35;
//						mraa_pwm_write(leftPWM, currentPWML);
//						mraa_pwm_write(rightPWM, currentPWMR);
//
//						err3 = err2; // shift states
//						err2 = err1;
//						err1 = err0;
//						printf("P: %.2f D: %.2f OUTPUT: %.3f pwmL: %.2f pwmR: %.2f Theta: %.3f\n", err0, errDiff, output, currentPWML, currentPWMR, theta);
//					}

					pulse(trig);
					if (readFlag){
						qsort(distanceArray, DETECT_FILTER_ARRAY_SIZE, sizeof(distanceArray[0]), compare);
						float avg = 0;
						int i;
						for (i = DETECT_FILTER_CUTOFF_SIZE; i < DETECT_FILTER_ARRAY_SIZE-DETECT_FILTER_CUTOFF_SIZE; i++){
							avg += distanceArray[i];
						}
						distAVG = avg/(DETECT_FILTER_ARRAY_SIZE-2*DETECT_FILTER_CUTOFF_SIZE);
	//					printf("AVERAGE: %.2f\n", distAVG);
//						printf("%.2f\n", distAVG);
						if (distAVG < 100){
							mraa_pwm_enable(rightPWM, 0);
							mraa_pwm_enable(leftPWM, 0);
							printf("Ciao\n");
							taskGetPtr = taskPutPtr;
							float D = sqrt(2*(distAVG+SENSOR_OFFSET)*(distAVG+SENSOR_OFFSET));
							float ccw45[3] = {1,PI/4,1};
							float frontD[3] = {0, D, 0};
							float cw90[3] = {1, PI/2, 0};
							taskPut(ccw45, 3);
							taskPut(frontD, 3);
							movementPut(nextMove, 2);
//							taskPut(cw90, 3);
//							taskPut(frontD, 3);
//							taskPut(ccw45, 3);
							taskComplete = 1;
						}
	//					printf("%.2f\n", distance);
						readFlag=0;
					}
				}
				break;
			case 1:
				while(!taskComplete){
					// Motion loop
					errorLeft = desiredPositionLeft - currentPositionLeft;
					errorRight = desiredPositionRight - currentPositionRight;
					if (!leftComplete){
						if (errorLeft > 1){
							if (errorLeft < 60) mraa_pwm_write(leftPWM, L_PWM-0.2);
							mraa_pwm_enable(leftPWM, 1);
							mraa_gpio_write(leftDir, 0);
						}
						else if (errorLeft < -1){
							if (errorLeft > -60) mraa_pwm_write(leftPWM, L_PWM-0.2);
							mraa_pwm_enable(leftPWM, 1);
							mraa_gpio_write(leftDir, 1);
						}
						else {
							mraa_pwm_enable(leftPWM, 0);
							leftComplete = 1;
		//						fprintf(stdout, "LEFT COMPLETE\n");
						}
					}
					if (!rightComplete){
						if (errorRight > 1){
							if (errorRight < 60) mraa_pwm_write(rightPWM, R_PWM-0.2);
							mraa_pwm_enable(rightPWM, 1);
							mraa_gpio_write(rightDir, 0);
						}
						else if (errorRight < -1){
							if (errorRight > -60) mraa_pwm_write(rightPWM, R_PWM-0.2);
							mraa_pwm_enable(rightPWM, 1);
							mraa_gpio_write(rightDir, 1);
						}
						else {
							mraa_pwm_enable(rightPWM, 0);
							rightComplete = 1;
		//						fprintf(stdout, "RIGHT COMPLETE\n");
						}
					}
					if((leftComplete) && (rightComplete)){
						taskComplete = 1;
						updateOdometry();
						fprintf(stdout, "TASK COMPLETE, XYT:[%.2f,%.2f,%.4f], L: %d R: %d\n", currentCoordinate[0], currentCoordinate[1], theta, currentPositionLeft, currentPositionRight);
					}

					// Odometry update loop
					odoCount++;
					if (odoCount % 7 == 0)
						updateOdometry();
				}
				break;
			case 2:
				while(!taskComplete){
					// Motion loop
					errorLeft = desiredPositionLeft - currentPositionLeft;
					errorRight = desiredPositionRight - currentPositionRight;

					//DEBUG
					if(!leftComplete) mraa_pwm_enable(leftPWM, 1);
					else {
						mraa_pwm_enable(leftPWM, 0);
						leftComplete = 1;
					}
					if(!rightComplete) mraa_pwm_enable(rightPWM, 1);
					else {
						mraa_pwm_enable(rightPWM, 0);
						rightComplete = 1;
					}
					//LEFT
					if (errorLeft > 1){
						if (errorLeft < 60) mraa_pwm_write(leftPWM, L_PWM-0.2);
						mraa_gpio_write(leftDir, 0);
					}
					else if (errorLeft < -1){
						if (errorLeft > -60) mraa_pwm_write(leftPWM, L_PWM-0.2);
						mraa_gpio_write(leftDir, 1);
					}
					else {
						mraa_pwm_enable(leftPWM, 0);
						leftComplete = 1;
	//						fprintf(stdout, "LEFT COMPLETE\n");
					}
					//RIGHT
					if (errorRight > 1){
						if (errorRight < 60) mraa_pwm_write(rightPWM, R_PWM-0.2);
						mraa_gpio_write(rightDir, 0);
					}
					else if (errorRight < -1){
						if (errorRight > -60) mraa_pwm_write(rightPWM, R_PWM-0.2);
						mraa_gpio_write(rightDir, 1);
					}
					else {
						mraa_pwm_enable(rightPWM, 0);
						rightComplete = 1;
	//						fprintf(stdout, "RIGHT COMPLETE\n");
					}
					//DEBUG END

//					if (!leftComplete){
//						if (errorLeft > 1){
//							if (errorLeft < 60) mraa_pwm_write(leftPWM, L_PWM-0.2);
//							mraa_pwm_enable(leftPWM, 1);
//							mraa_gpio_write(leftDir, 0);
//						}
//						else if (errorLeft < -1){
//							if (errorLeft > -60) mraa_pwm_write(leftPWM, L_PWM-0.2);
//							mraa_pwm_enable(leftPWM, 1);
//							mraa_gpio_write(leftDir, 1);
//						}
//						else {
//							mraa_pwm_enable(leftPWM, 0);
//							leftComplete = 1;
//		//						fprintf(stdout, "LEFT COMPLETE\n");
//						}
//					}
//					if (!rightComplete){
//						if (errorRight > 1){
//							if (errorRight < 60) mraa_pwm_write(rightPWM, R_PWM-0.2);
//							mraa_pwm_enable(rightPWM, 1);
//							mraa_gpio_write(rightDir, 0);
//						}
//						else if (errorRight < -1){
//							if (errorRight > -60) mraa_pwm_write(rightPWM, R_PWM-0.2);
//							mraa_pwm_enable(rightPWM, 1);
//							mraa_gpio_write(rightDir, 1);
//						}
//						else {
//							mraa_pwm_enable(rightPWM, 0);
//							rightComplete = 1;
//		//						fprintf(stdout, "RIGHT COMPLETE\n");
//						}
//					}
					if((leftComplete) && (rightComplete)){
						updateOdometry();
						usleep(1000); // 1ms sleep to stabilize
						pulse(trig);
						printf("D %d %d\n", readFlag, readCount);
						if (readFlag){
							qsort(distanceArray, DETECT_FILTER_ARRAY_SIZE, sizeof(distanceArray[0]), compare);
							float avg = 0;
							int i;

							for (i = DETECT_FILTER_CUTOFF_SIZE; i < DETECT_FILTER_ARRAY_SIZE-DETECT_FILTER_CUTOFF_SIZE; i++){
								avg += distanceArray[i];
							}
							distAVG = avg/(DETECT_FILTER_ARRAY_SIZE-2*DETECT_FILTER_CUTOFF_SIZE);
							printf("%.2f\n", distAVG);

							if (distAVG < DETECT_DISTANCE-130){
								float objX, objY;
								objX = currentCoordinate[0] + (distAVG+SENSOR_OFFSET-30)*cos(theta);
								objY = currentCoordinate[1] + (distAVG+SENSOR_OFFSET-30)*sin(theta);
								updateObstacle(objX, objY, 0, 0);
							}
							readFlag = 0;
							taskComplete = 1;
							scanOnCompletion--;
							if (!scanOnCompletion) {
								theta+=0.18;
							}
							fprintf(stdout, "TASK COMPLETE, XYT:[%.2f,%.2f,%.4f], L: %d R: %d\n", currentCoordinate[0], currentCoordinate[1], theta, currentPositionLeft, currentPositionRight);
						}
					}

					// Odometry update loop
					odoCount++;
					if (odoCount % 7 == 0)
						updateOdometry();
				}
				break;
			}

				// Obstacle detection loop
//				switch(robotState){
//					case 0: // /going forward
//						pulse(trig);
//						if (readFlag){
//							qsort(distanceArray, DETECT_FILTER_ARRAY_SIZE, sizeof(distanceArray[0]), compare);
//							float avg = 0;
//							int i;
//							for (i = DETECT_FILTER_CUTOFF_SIZE; i < DETECT_FILTER_ARRAY_SIZE-DETECT_FILTER_CUTOFF_SIZE; i++){
//								avg += distanceArray[i];
//							}
//							distAVG = avg/(DETECT_FILTER_ARRAY_SIZE-2*DETECT_FILTER_CUTOFF_SIZE);
//		//					printf("AVERAGE: %.2f\n", distAVG);
//							printf("%.2f\n", distAVG);
//							if (distAVG < 100){
//								mraa_pwm_enable(rightPWM, 0);
//								mraa_pwm_enable(leftPWM, 0);
//								printf("Ciao\n");
//								taskGetPtr = taskPutPtr;
//								float D = sqrt(2*(distAVG+SENSOR_OFFSET)*(distAVG+SENSOR_OFFSET));
//								float ccw45[3] = {1,PI/4,1};
//								float frontD[3] = {0, D, 0};
//								float cw90[3] = {1, PI/2, 0};
//								taskPut(ccw45, 3);
//								taskPut(frontD, 3);
//								taskPut(cw90, 3);
//								taskPut(frontD, 3);
//								taskPut(ccw45, 3);
//								taskComplete = 1;
//							}
//
//		//					printf("%.2f\n", distance);
//							readFlag=0;
//						}
//						break;
//					case 2:
//						if (leftComplete && rightComplete){
//							usleep(1000); // 1ms sleep to stabilize
//							pulse(trig);
//							if (readFlag){
//								qsort(distanceArray, DETECT_FILTER_ARRAY_SIZE, sizeof(distanceArray[0]), compare);
//								float avg = 0;
//								int i;
//								for (i = DETECT_FILTER_CUTOFF_SIZE; i < DETECT_FILTER_ARRAY_SIZE-DETECT_FILTER_CUTOFF_SIZE; i++){
//									avg += distanceArray[i];
//								}
//								distAVG = avg/(DETECT_FILTER_ARRAY_SIZE-2*DETECT_FILTER_CUTOFF_SIZE);
//								if (distAVG < DETECT_DISTANCE){
//									float objX, objY;
//									objX = currentCoordinate[0] + (distAVG+SENSOR_OFFSET)*cos(theta);
//									objY = currentCoordinate[1] + (distAVG+SENSOR_OFFSET)*sin(theta);
//									updateObstacle(objX, objY, 0, 0);
//								}
//								readFlag = 0;
//								scanComplete = 1;
//								scanOnCompletion--;
//							}
//						}
//						break;
//				}
		}
		mraa_pwm_enable(rightPWM, 0);
		mraa_pwm_enable(leftPWM, 0);
	}
	return MRAA_SUCCESS;
}

// LEGACY
//				errorLeft = desiredPositionLeft - currentPositionLeft;
//				errorRight = desiredPositionRight - currentPositionRight;
//				if (leftComplete == 0){
//					if (errorLeft > 0){
//						mraa_pwm_enable(leftPWM, 1);
//						mraa_gpio_write(leftDir, 0);
//					}
//					if (errorLeft < 0){
//						mraa_pwm_enable(leftPWM, 1);
//						mraa_gpio_write(leftDir, 1);
//					}
//					if (errorLeft == 0){
//						mraa_pwm_enable(leftPWM, 0);
//						leftComplete = 1;
////						fprintf(stdout, "LEFT COMPLETE\n");
//					}
//				}
//				if (rightComplete == 0){
//					if (errorRight > 0){
//						mraa_pwm_enable(rightPWM, 1);
//						mraa_gpio_write(rightDir, 0);
//					}
//					if (errorRight < 0){
//						mraa_pwm_enable(rightPWM, 1);
//						mraa_gpio_write(rightDir, 1);
//					}
//					if (errorRight == 0){
//						mraa_pwm_enable(rightPWM, 0);
//						rightComplete = 1;
////						fprintf(stdout, "RIGHT COMPLETE\n");
//					}
//				}
//				if((leftComplete==1) && (rightComplete==1)){
//					taskComplete = 1;
//					updateOdometry();
//					if(scanOnCompletion) scanOnCompletion--;
////					fprintf(stdout, "TASK COMPLETE, XYT:[%.2f,%.2f,%.2f]\n", currentCoordinate[0], currentCoordinate[1], theta);
//				}

//				// Obstacle detection loop
//				pulse(trig);
//				if (readFlag){
//					qsort(distanceArray, DETECT_FILTER_ARRAY_SIZE, sizeof(distanceArray[0]), compare);
//					float avg = 0;
//					int i;
//					for (i = DETECT_FILTER_CUTOFF_SIZE; i < DETECT_FILTER_ARRAY_SIZE-DETECT_FILTER_CUTOFF_SIZE; i++){
//						avg += distanceArray[i];
//					}
//					distAVG = avg/(DETECT_FILTER_ARRAY_SIZE-2*DETECT_FILTER_CUTOFF_SIZE);
////					printf("%.2f      ", distAVG);
////					if (distAVG < 300){
////						printf(" <-------------- \n");
////					}else printf("\n");
////					dummyCounter++;
//					readFlag=0;
//				}
//
//				// Decision loop
//				if (distAVG < DETECT_DISTANCE - SENSOR_OFFSET){
//					printf("\nSomething X: %.2f Y: %.2f \n", \
//							currentCoordinate[0] + (distAVG+SENSOR_OFFSET)*cos(theta), \
//							currentCoordinate[1] + (distAVG+SENSOR_OFFSET)*sin(theta));
//					if (scanOnCompletion){
//						printf("Scanning Angle: %d\n", scanOnCompletion);
//					}
//				}


// Proportional-Derivative control
//				if (odoCount % 10 == 0){
//					if (robotState == 0){ // It is moving forward
//						float scale = 0.01, P = 1, D = 1; // Proportional and derivative constants
//						err0 = fabs(targetTheta) - fabs(theta); // Proportional error
//						errDiff = err3 - err0; // Derivative error
//						output = P*err0 + D*errDiff; // Total error
//						if (output<0){
//							output = (-output);
//							currentPWML += scale*output;
//							currentPWMR -= scale*output;
//						}
//						else if (output>0){
//							currentPWML -= scale*output;
//							currentPWMR += scale*output;
//						}
//						if (currentPWML > 0.65) currentPWML = 0.65;
//						else if (currentPWML < 0.35) currentPWML = 0.35;
//						if (currentPWMR > 0.65) currentPWMR = 0.65;
//						else if (currentPWMR < 0.35) currentPWMR = 0.35;
//						mraa_pwm_write(leftPWM, currentPWML);
//						mraa_pwm_write(rightPWM, currentPWMR);
//
//						err3 = err2; // shift states
//						err2 = err1;
//						err1 = err0;
//
////						printf("P: %.2f D: %.2f OUTPUT: %.3f pwmL: %.2f pwmR: %.2f Theta: %.3f\n", err0, errDiff, output, currentPWML, currentPWMR, theta);
//					}
//				}
