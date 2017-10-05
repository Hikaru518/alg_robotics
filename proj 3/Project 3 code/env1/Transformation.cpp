#include "Transformation.h"

// This Transformation function is used to calculate the coordinates of the point
// after rotating around the pivot point in counterclockwise by theta.
// The Transfromation of the point includes 
// translation(-pivotX,-pivotY)-rotation(theta)-translation(pivotX,pivotY).
double getRotatedPoint_X(double px, double py, double pivotX, double pivotY, double theta){
	double rotatedX;
    // rotatedX is the x-axis coordinate of the point after the roatation
	rotatedX = px*cos(theta)-pivotX*cos(theta)-py*sin(theta)+pivotY*sin(theta)+pivotX;
	return rotatedX;
}

double getRotatedPoint_Y(double px, double py, double pivotX, double pivotY, double theta){
	double rotatedY;
	// rotatedY is the y-axis coordinate of the point after the roatation
	rotatedY = px*sin(theta)-pivotX*sin(theta)+py*cos(theta)-pivotY*cos(theta)+pivotY;
	return rotatedY;
}