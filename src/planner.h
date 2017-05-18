/*
 * planner.h
 *
 *  Created on: Apr 15, 2017
 *      Author: user
 */

#ifndef PLANNER_H_
#define PLANNER_H_

#define GRID_LENGTH 488
#define GRID_SIZE 3
#define CHECK_SIZE 3
#define START_X 1
#define START_Y 0
#define URL_MAP_UPDATE "https://hxzhpp8ds6.execute-api.us-west-2.amazonaws.com/deployed/map-state"
#define URL_BOT_UPDATE "https://hxzhpp8ds6.execute-api.us-west-2.amazonaws.com/deployed/bot-state"
#define URL_OBSTACLE_UPDATE "https://hxzhpp8ds6.execute-api.us-west-2.amazonaws.com/deployed/obstacle-state"

#include <stdio.h>
#include <time.h>
#include <stdlib.h>

extern int cell[GRID_SIZE][GRID_SIZE][2]; // [x,y]
extern int cellState[GRID_SIZE][GRID_SIZE]; // 0: unexplored, 1: explored
extern int cellWeight[GRID_SIZE][GRID_SIZE]; // Weightage

void initGrid(void);
int* planGrid(int x, int y);
void viewGrid(int mode);
void setGrid(int x, int y);
void clearGrid(int x, int y);
void updateMap(int xx, int yy, int parseFlag);
void updateObstacle(float xx, float yy, int cleared, int parseFlag);

#endif /* PLANNER_H_ */
