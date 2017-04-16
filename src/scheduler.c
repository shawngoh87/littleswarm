
#include "scheduler.h"

unsigned char movementPutPtr, movementGetPtr, taskPutPtr, taskGetPtr;
float task[TASK_QUEUE_SIZE + 1][TASK_LENGTH];
float movement[MOVEMENT_QUEUE_SIZE][2];


int movementPut(float item[2], int size){
	// May have problem if array reach max, where getPtr == putPtr again
	if(movementPutPtr == (movementGetPtr-1+MOVEMENT_QUEUE_SIZE)%MOVEMENT_QUEUE_SIZE){
		return -1;
	}
	int i = 0;
	for (i = 0; i < size; i++){
		movement[movementPutPtr][i] = item[i];
	}

	movementPutPtr = (movementPutPtr + 1) % MOVEMENT_QUEUE_SIZE;
	return 0;
}

int movementGet(float item[2], int size){
	if(movementPutPtr == movementGetPtr){
		return -1;
	}

	int i = 0;
	for (i = 0; i < size; i++){
		item[i] = movement[movementGetPtr][i];
	}
	movementGetPtr = (movementGetPtr + 1) % MOVEMENT_QUEUE_SIZE;
	return 0;
}

int taskPut(float item[3], int size){
	// May have problem if task array reach max, where getPtr == putPtr again
	if(taskPutPtr == (taskGetPtr-1+TASK_QUEUE_SIZE)%TASK_QUEUE_SIZE){
		return -1;
	}
	int i = 0;
	for (i = 0; i < size; i++){
		task[taskPutPtr][i] = item[i];
	}

	taskPutPtr = (taskPutPtr + 1) % TASK_QUEUE_SIZE;
	return 0;
}

int taskGet(float item[3], int size){
	if(taskPutPtr == taskGetPtr){
		return -1;
	}

	int i = 0;
	for (i = 0; i < size; i++){
		item[i] = task[taskGetPtr][i];
	}
	taskGetPtr = (taskGetPtr + 1) % TASK_QUEUE_SIZE;
	return 0;
}
