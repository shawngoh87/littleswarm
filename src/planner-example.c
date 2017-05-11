/*
 * planner.c
 *
 *  Created on: Apr 15, 2017
 *      Author: user
 */
#define GRID_LENGTH 330
#define GRID_SIZE 4
#define CHECK_SIZE 3

//#include "planner.h"

int cell[GRID_SIZE][GRID_SIZE][2]; // [x,y]
int cellState[GRID_SIZE][GRID_SIZE]; // 0: unexplored, 1: explored
int cellWeight[GRID_SIZE][GRID_SIZE]; // Weightage

void initGrid(void){
	int i,j;
	for (j=0;j<GRID_SIZE-1;j++){
		for (i=0;i<GRID_SIZE-1;i++){
			cell[i][j][0] = i*GRID_LENGTH;
			cell[i][j][1] = j*GRID_LENGTH;
			cellState[i][j] = 0;
			cellWeight[i][j] = 0;
//			printf("%d %d\n", cell[i][j][0],cell[i][j][1]);
		}
	}
	cellState[0][0]=1; // Start from cell 0,0
}

int* planGrid(int x, int y){ // Optimize with recursion
	int i,j,m,n;
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
							if ((m == i && n == j) || (m < 0) || (m > GRID_SIZE - 1) || (n < 0) || (n > GRID_SIZE - 1) ){ // If origin or out of bounds
				//				printf("CONTINUE %d %d \n", i, j);
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
				coord[0]=i;
				coord[1]=j;
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

int main(){
	int *coord;
	initGrid();
	cellState[0][1] = cellState[1][0] = cellState[0][2] = 1;
	coord = planGrid(0,0);
	int i,j;
	for (j=GRID_SIZE-1;j>=0;j--){
		for (i=0;i<GRID_SIZE;i++){
			printf("%d ", cellState[i][j]);
		}
		printf("\n");
	}
	printf("%d %d\n", coord[0], coord[1]);
	printf("%d %d\n", cell[coord[0]][coord[1]][0], cell[coord[0]][coord[1]][1]);
	return 0;
}
