///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Authors: Peiguang Wang, Sichao Zhang
// Date: Sept. 18, 2017
//////////////////////////////////////

#include "CollisionChecking.h"

// Intersect the point (x,y) with the set of rectangles. If the point lies
// outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle>& obstacles)
{
    // TODO: IMPLEMENT ME!!
    //int i = 0;
    //int N = obstacles.size();
    //int a,b,width,height;
    for (int i = 0; i<obstacles.size(); i++)
    {
    	double a = obstacles[i].x;
    	double b = obstacles[i].y;
    	double width = obstacles[i].width;
    	double height = obstacles[i].height;
    	if( (x>=a) && (x<= (a+width)) && (y>=b) && (y<=(b+height))) 
    	{
    	    return false;
    	}
    }
    return true;
}

// Intersect a circle with center (x,y) and given radius with the set of
// rectangles. If the circle lies outside of all obstacles, return true.
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle>& obstacles)
{double i=0;
 double a,b,w,h;
 double N=obstacles.size();
 for(i=0;i<N;i++)
 {
   a=obstacles[i].x;
   b=obstacles[i].y;
   w=obstacles[i].width;
   h=obstacles[i].height;   
   if ((x>=a-radius && x<=a+w+radius && y>=b && y<=b+h) || (x>=a && x<=a+w && y>=b-radius && y<=b+h+radius) || (x-a)^2+(y-b)^2<=radius^2 || (x-a)^2+(y-b-h)^2<=radius^2 || (x-a-w)^2+(y-b)^2<=radius^2 || (x-a-w)^2+(y-b-h)^2<=radius^2)
   {
    return false;
   }
 }
    // TODO: IMPLEMENT ME!!
    return true;
}

// Intersect a square with center at (x,y), orientation theta, and the given
// side length with the set of rectangles. If the square lies outside of all
// obstacles, return true.
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle>& obstacles)
{
    // TODO: IMPLEMENT ME!!
    return false;
}

// Add any custom debug / development code here. This code will be executed
// instead of the statistics checker (Project2.cpp). Any code submitted here
// MUST compile, but will not be graded.
void debugMode(const std::vector<Robot>& /*robots*/, const std::vector<Rectangle>& /*obstacles*/, const std::vector<bool>& /*valid*/)
{
}
