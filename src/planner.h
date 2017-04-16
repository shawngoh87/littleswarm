/*
 * planner.h
 *
 *  Created on: Apr 15, 2017
 *      Author: user
 */

#ifndef PLANNER_H_
#define PLANNER_H_

#define GRID_LENGTH 330
#define GRID_SIZE 4
#define CHECK_SIZE 3

#include <stdio.h>

extern int cell[GRID_SIZE][GRID_SIZE][2]; // [x,y]
extern int cellState[GRID_SIZE][GRID_SIZE]; // 0: unexplored, 1: explored
extern int cellWeight[GRID_SIZE][GRID_SIZE]; // Weightage

void initGrid(void);
int* planGrid(int x, int y);
void viewGrid(int mode);
void setGrid(int x, int y);

#endif /* PLANNER_H_ */
