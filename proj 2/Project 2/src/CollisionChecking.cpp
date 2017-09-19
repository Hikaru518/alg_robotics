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
    // 	if( x>=a )
    // 	{
    // 		//std::cout << "x>=a"<<std::endl;
    // 		if (x <= (a+width)) 
    // 		{
    // // 			std::cout << "x<=a++++++"<<std::endl;
    // // 			std::cout << "Obstacles #: " << i <<std::endl;
    // // 			std::cout << "b = " << b <<std::endl;
    // //             std::cout << "x = " << x <<std::endl;
				// // std::cout << "y = " << y <<std::endl;
    // 			if (y >=b)
    // 			{
    // 				//std::cout << "x>=b"<<std::endl;
    // 				if (y<=(b+height))
    // 				{
    // 					//std::cout << "x<=b++++++"<<std::endl;
    // 				}
    // 			}

    // 		}
    // 	}
    }
    return true;
}

// Intersect a circle with center (x,y) and given radius with the set of
// rectangles. If the circle lies outside of all obstacles, return true.
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle>& obstacles)
{
    // TODO: IMPLEMENT ME!!
    return false;
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
