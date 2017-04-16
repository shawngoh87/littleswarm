/*
 * scheduler.h
 *
 *  Created on: Apr 15, 2017
 *      Author: user
 */

#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#define TASK_QUEUE_SIZE 15
#define MOVEMENT_QUEUE_SIZE 15
#define TASK_LENGTH 3

extern unsigned char movementPutPtr, movementGetPtr, taskPutPtr, taskGetPtr;
extern float task[TASK_QUEUE_SIZE + 1][TASK_LENGTH];
extern float movement[MOVEMENT_QUEUE_SIZE][2];

int movementPut(float item[2], int size);
int movementGet(float item[2], int size);
int taskPut(float item[3], int size);
int taskGet(float item[3], int size);

#endif /* SCHEDULER_H_ */
