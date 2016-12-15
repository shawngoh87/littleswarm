// PSO.cpp: Swarm Exploration
// Shawn Goh <shawngoh87@hotmail.com>

#include "stdafx.h"
#include <cmath>


float* add2DVectors(float* vec1, float* vec2){
	/*
	Return: address of summed array.
	Argument:
	address of vec1 array
	address of vec2 array
	*/
	static float coordinate[2];
	coordinate[0] = *vec1 + *vec2;
	coordinate[1] = *(++vec1) + *(++vec2);
	return coordinate;
}

float* rotate(float* vec, float angleDegree){
	/*
	Return: address of rotated array.
	Argument:
	address of vec array
	rotation angle in degrees
	*/
	static float vecRotate[2];
	float angle;
	angle = (angleDegree / 180) * 3.1415; // Convert to radians.
	vecRotate[0] = (*vec) * cos(angle) - (*(vec + 1)) * sin(angle);
	vecRotate[1] = (*vec) * sin(angle) + (*(vec + 1)) * cos(angle);
	return vecRotate;
}

int sumAscend(int length, int degree){
	/*
	Recursively summing from (1 to length)
	*/
	if (length > 1){
		return pow(length, degree) + sumAscend(length - 1, degree);
	}
	else return length;
}
//def roulette(array, degree)


int _tmain(int argc, _TCHAR* argv[])
{
	// Real codez

	float* newCoord;
	float vec1[] = { 1, 2 };
	/*float vec2[] = { 3, 4 };
	newCoord = add2DVectors(vec1, vec2);
	newCoord = rotate(vec1, 360);
	printf("%.2f %.2f", newCoord[0], newCoord[1]);*/
	printf("%d", sumAscend(3, 2));
	getchar();

	// Courtesy
	/*printf("Hello World!");
	getchar();
	return 0;*/
}



