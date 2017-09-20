#include "Transformation.h"

double getRotatedPoint_X(double px, double py, double pivotX, double pivotY, double theta){
	double rotatedX;
	rotatedX = px*cos(theta)-pivotX*cos(theta)-py*sin(theta)+pivotY*sin(theta)+pivotX;
	return rotatedX;
}

double getRotatedPoint_Y(double px, double py, double pivotX, double pivotY, double theta){
	double rotatedY;
	rotatedY = px*sin(theta)-pivotX*sin(theta)+py*cos(theta)-pivotY*cos(theta)+pivotY;
	return rotatedY;
}