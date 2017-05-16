/*
 * planner.c
 *
 *  Created on: Apr 15, 2017
 *      Author: user
 */

#include "planner.h"
#include <string.h>
#include "iotkit-comm/cJSON.h"
#include "curl/curl.h"

int cell[GRID_SIZE][GRID_SIZE][2]; // [x,y]
int cellState[GRID_SIZE][GRID_SIZE]; // 0: unexplored, 1: explored
int cellWeight[GRID_SIZE][GRID_SIZE]; // Weightage


//JZ:structure to store response message
struct MemoryStruct {
  char *memory;
  size_t size;
};

//JZ:function to get response message
static size_t
WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
  size_t realsize = size * nmemb;
  struct MemoryStruct *mem = (struct MemoryStruct *)userp;

  mem->memory = realloc(mem->memory, mem->size + realsize + 1);
  if(mem->memory == NULL) {
    /* out of memory! */
    printf("not enough memory (realloc returned NULL)\n");
    return 0;
  }

  memcpy(&(mem->memory[mem->size]), contents, realsize);
  mem->size += realsize;
  mem->memory[mem->size] = 0;

  return realsize;
}

//JZ:remove char c in string str
void remove_all_chars(char* str, char c) {
    char *pr = str, *pw = str;
    while (*pr) {
        *pw = *pr++;
        pw += (*pw != c);
    }
    *pw = '\0';
}

//JZ:setup and operation for posting JSON and returning response message
char* curl_operation(cJSON* post_json, char* post_url){
	//JZ:initialization for getting response
	struct MemoryStruct chunk;
	chunk.memory = malloc(1);  /* will be grown as needed by the realloc above */
	chunk.size = 0;    /* no data at this point */


	//JZ:starting of initialization, setting up and operation of CURL
	CURL *curl;
	CURLcode res;

	/* In windows, this will init the winsock stuff */
	curl_global_init(CURL_GLOBAL_ALL);

	/* get a curl handle */
	curl = curl_easy_init();
	if (curl) {
		/* First set the URL that is about to receive our POST. This URL can
		just as well be a https:// URL if that is what should receive the
		data. */
		curl_easy_setopt(curl, CURLOPT_URL, post_url);
		/* Now specify the POST data */
		curl_easy_setopt(curl, CURLOPT_POSTFIELDS, cJSON_Print(post_json, 0));

		/* send all data to this function  */
		curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);

		/* we pass our 'chunk' struct to the callback function */
		curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&chunk);

		/* Perform the request, res will get the return code */
		res = curl_easy_perform(curl);
		/* Check for errors */
		if (res != CURLE_OK)
			fprintf(stderr, "curl_easy_perform() failed: %s\n",
				curl_easy_strerror(res));

		/* always cleanup */
		curl_easy_cleanup(curl);
	}
	curl_global_cleanup();

	return chunk.memory;
}

void updateMap(int xx, int yy, int parseFlag){
	/* Curl&cJSON function by JZ
	 * param:
	 * -- URL_UPDATE (global var)	: url for database API's HTTPS endpoint.
	 * -- coord[] 					: next step's coordinate
	 * */
	cJSON *payload = NULL;
	payload = cJSON_CreateObject();
	char* strX[5], strY[5];
	sprintf(strX, "%d", xx);
	sprintf(strY, "%d", yy);
	cJSON_AddStringToObject(payload, "x", strX);
	cJSON_AddStringToObject(payload, "y", strY);
	cJSON_AddStringToObject(payload, "explored", "1");
	char* response = curl_operation(payload, URL_MAP_UPDATE);
//	printf("\nResponse from AWS:\n%s",response);

	if (parseFlag){
		cJSON *body = cJSON_GetObjectItem(cJSON_Parse(response), "Items");
		int cellCount = cJSON_GetObjectItem(cJSON_Parse(response), "Count")->valueint;
		int i;
		for (i = 0; i < cellCount; i++){
			cJSON *cell = cJSON_GetArrayItem(body,i);
			char* ptr;
			int x = strtol(cJSON_GetObjectItem(cJSON_GetObjectItem(cell, "x"),"N")->valuestring, &ptr, 10);
			int y = strtol(cJSON_GetObjectItem(cJSON_GetObjectItem(cell, "y"),"N")->valuestring, &ptr, 10);
			int exp = strtol(cJSON_GetObjectItem(cJSON_GetObjectItem(cell, "explored"),"N")->valuestring, &ptr, 10);
			if (exp){
				setGrid(x, y);
			}
		}
	}
}

void updateObstacle(float xx, float yy, int cleared, int parseFlag){
	cJSON *payload = NULL;
	payload = cJSON_CreateObject();
	char* strX[10], strY[10];
	sprintf(strX, "%.3f", xx);
	sprintf(strY, "%.3f", yy);
	cJSON_AddStringToObject(payload, "x", strX);
	cJSON_AddStringToObject(payload, "y", strY);
	if(cleared) cJSON_AddStringToObject(payload, "cleared", "1");
	else cJSON_AddStringToObject(payload, "cleared", "0");
	char* response = curl_operation(payload, URL_OBSTACLE_UPDATE);
//	printf("\nResponse from AWS:\n%s",response);
	if (parseFlag){
		cJSON *body = cJSON_GetObjectItem(cJSON_Parse(response), "Items");
		int cellCount = cJSON_GetObjectItem(cJSON_Parse(response), "Count")->valueint;
		int i;
		for (i = 0; i < cellCount; i++){
			cJSON *cell = cJSON_GetArrayItem(body,i);
			char* ptr;
			float x = strtof(cJSON_GetObjectItem(cJSON_GetObjectItem(cell, "x"),"N")->valuestring, &ptr);
			float y = strtof(cJSON_GetObjectItem(cJSON_GetObjectItem(cell, "y"),"N")->valuestring, &ptr);
			int clr = strtol(cJSON_GetObjectItem(cJSON_GetObjectItem(cell, "cleared"),"N")->valuestring, &ptr, 10);
//			printf("%.3f, %.3f, %d\n", x, y, clr);
		}
//		printf("%s\n", cJSON_Print(body, 4));
	}
}


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
//	printf("INPUT X: %d Y: %d\n", x, y);
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
//				coord[0]=cell[i][j][0];
//				coord[1]=cell[i][j][1];
				max = cellWeight[i][j];
			}
		}
	}
	// Lottery: reservoir sampling
	float r = (float)rand()/RAND_MAX;
	int sampleVolume = 0;
	for (j=0;j<GRID_SIZE;j++){
		for (i=0;i<GRID_SIZE;i++){
			if (cellWeight[i][j] == max){
				sampleVolume++;
				if (r < 1/sampleVolume){ // Select with 1/n probs
					coord[0]=cell[i][j][0];
					coord[1]=cell[i][j][1];
				}
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

void clearGrid(int x, int y){ // Use this to reset the exploration state
	cellState[x][y] = 0;
}

void viewGrid(int mode){
	int i,j;
	switch(mode){
		case 0: // View cellState
			fprintf(stdout, "\n");
			for (j=GRID_SIZE-1;j>=0;j--){
				for (i=0;i<GRID_SIZE;i++){
					fprintf(stdout,"%d ", cellState[i][j]);
				}
				fprintf(stdout, "\n");
			}
			break;
		case 1: // View cellWeight
			fprintf(stdout, "\n");
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

