/*
 * planner.c
 *
 *  Created on: Apr 15, 2017
 *      Author: user
 */
#define GRID_LENGTH 330
#define GRID_SIZE 4
#define CHECK_SIZE 3

#include "planner.h"

int cell[GRID_SIZE][GRID_SIZE][2]; // [x,y]
int cellState[GRID_SIZE][GRID_SIZE]; // 0: unexplored, 1: explored
int cellWeight[GRID_SIZE][GRID_SIZE]; // Weightage

void initGrid(void){
	int i,j;
	for (j=0;j<GRID_SIZE;j++){
		for (i=0;i<GRID_SIZE;i++){
			cell[i][j][0] = i*GRID_LENGTH;
			cell[i][j][1] = j*GRID_LENGTH;
			cellState[i][j] = 0;
			cellWeight[i][j] = 0;
//			printf("%d %d\n", cell[i][j][0],cell[i][j][1]);
		}
	}
	cellState[0][0]=1; // Start from cell 0,0
}

void resetGridWeight(){
	int i,j;
	for (j=0;j<GRID_SIZE;j++){
		for (i=0;i<GRID_SIZE;i++){
			cellWeight[i][j] = 0;
		}
	}
}

int* planGrid(int x, int y){ // Optimize with recursion
	printf("INPUT X: %d Y: %d\n", x, y);
	int i,j,m,n;
	resetGridWeight();
	for (j=y-1;j<y+2;j++){
		for (i=x-1;i<x+2;i++){
//			printf("CURRENT: %d %d \n", i, j);
			if ((i == x && j == y) || (i < 0) || (i > GRID_SIZE - 1) || (j < 0) || (j > GRID_SIZE - 1) ){ // If origin or out of bounds
//				printf("CONTINUE %d %d \n", i, j);
				continue;
			}
			else{
				if (cellState[i][j] == 0){
					int count=0;
					for (n=j-1;n<j+2;n++){
						for (m=i-1;m<i+2;m++){
//							printf("CURRENTMN: %d %d \n", m, n);
							if ((m == i && n == j) || (m < 0) || (m > GRID_SIZE - 1) || (n < 0) || (n > GRID_SIZE - 1) ){ // If origin or out of bounds
//								printf("CONTINUE %d %d \n", i, j);
								continue;
							}
							else{
								if (cellState[m][n] == 1){
									count++;
//									printf("THIS IS %d %d CELL\n", m, n);
								}
							}
						}
					}
					cellWeight[i][j] = count;
//					printf("CELL %d %d HAS %d\n", i, j, count);
				}
//				else printf("CELL %d %d EXPLORED\n", i, j);
			}
		}
	}
	static int coord[2], var[2] = {-1,-1};
	int max=0;
	for (j=y-1;j<y+2;j++){
		for (i=x-1;i<x+2;i++){
			if ((i == x && j == y) || (i < 0) || (i > GRID_SIZE - 1) || (j < 0) || (j > GRID_SIZE - 1) ) // If origin or out of bounds
				continue;
			if (cellWeight[i][j] >= max){
				coord[0]=cell[i][j][0];
				coord[1]=cell[i][j][1];
				max = cellWeight[i][j];
			}
		}
	}
	if (max==0){ // All around is explored
//		printf("NOTHERE\n");
//		printf("VAR IN %d %d\n", var[0], var[1]);
		return var;
	}
//	printf("HERE\n");
//	printf("COORD IN %d %d\n", coord[0], coord[1]);
	return coord;
}

void setGrid(int x, int y){ // Use this to set the exploration state
	cellState[x][y] = 1;
}

void viewGrid(int mode){
	int i,j;
	switch(mode){
		case 0: // View cellState
			for (j=GRID_SIZE-1;j>=0;j--){
				for (i=0;i<GRID_SIZE;i++){
					fprintf(stdout,"%d ", cellState[i][j]);
				}
				fprintf(stdout, "\n");
			}
			break;
		case 1: // View cellWeight
			printf("CELLWEIGHT:\n");
			for (j=GRID_SIZE-1;j>=0;j--){
				for (i=0;i<GRID_SIZE;i++){
					printf("%d ", cellWeight[i][j]);
				}
				printf("\n");
			}
			break;
	}
}

