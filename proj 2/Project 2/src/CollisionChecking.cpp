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
{
  double i=0;
  double a,b,width,height;
  double N=obstacles.size();
  for(i=0;i<N;i++)
  {
    a=obstacles[i].x;
    b=obstacles[i].y;
    width=obstacles[i].width;
    height=obstacles[i].height;   
    if ((x>=(a-radius) && x<=(a+width+radius) && y>=b && y<=(b+height)) ||   // left and right bound
        (x>=a && x<=(a+width) && y>=(b-radius) && y<=(b+height+radius)) ||   // upper and lower bound
        (x-a)*(x-a) + (y-b)*(y-b) <= radius*radius || // lower, left corner
        (x-a)*(x-a) + (y-b-height)*(y-b-height) <= radius*radius || // upper, left corner
        (x-a-width)*(x-a-width) + (y-b)*(y-b) <= radius*radius || // lower, right corner
        (x-a-width)*(x-a-width) + (y-b-height)*(y-b-height) <= radius*radius) { // upper, right corner
      return false;
    }
  }
  return true;
}

// Intersect a square with center at (x,y), orientation theta, and the given
// side length with the set of rectangles. If the square lies outside of all
// obstacles, return true.
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle>& obstacles)
{
  int N = obstacles.size();
  // 4 corners of obstacles
  double obstacleP1_x,obstacleP1_y,obstacleP2_x,obstacleP2_y,obstacleP3_x,obstacleP3_y,obstacleP4_x,obstacleP4_y; 
  // 4 corner points of the square
  double squareP1_x,squareP1_y,squareP2_x,squareP2_y,squareP3_x,squareP3_y,squareP4_x,squareP4_y;
  // transformed corner points of the obstacles 
  double tObstacleP1_x,tObstacleP1_y,tObstacleP2_x,tObstacleP2_y,tObstacleP3_x,tObstacleP3_y,tObstacleP4_x,tObstacleP4_y; 
  // transformed corner points of square
  double tSquareP1_x,tSquareP1_y,tSquareP2_x,tSquareP2_y,tSquareP3_x,tSquareP3_y,tSquareP4_x,tSquareP4_y;

  squareP1_x = 0.5*sideLength*(sin(theta)+cos(theta)) + x;
  squareP1_y = 0.5*sideLength*(sin(theta)-cos(theta)) + y;

  squareP2_x = 0.5*sideLength*(-sin(theta)+cos(theta)) + x;
  squareP2_y = 0.5*sideLength*(sin(theta)+cos(theta)) + y;

  squareP3_x = 0.5*sideLength*(-sin(theta)-cos(theta)) + x;
  squareP3_y = 0.5*sideLength*(-sin(theta)+cos(theta)) + y;

  squareP4_x = 0.5*sideLength*(sin(theta)-cos(theta)) + x;
  squareP4_y = 0.5*sideLength*(-sin(theta)-cos(theta)) + y;

  // case 1: object's corners in obstacles
  if(!(isValidPoint(squareP1_x,squareP1_y,obstacles) &&
        isValidPoint(squareP2_x,squareP2_y,obstacles) &&
        isValidPoint(squareP3_x,squareP3_y,obstacles) &&
        isValidPoint(squareP4_x,squareP4_y,obstacles)))
  {
    std::cout<< "stops at case 1" << std::endl;
    return false;
  }

  // case 2: obstacles's corners in object
  // rotate square in clockwise direction by theta
  tSquareP1_x = x + 0.5*sideLength;
  tSquareP1_y = y - 0.5*sideLength;

  tSquareP2_x = x + 0.5*sideLength;
  tSquareP2_y = y + 0.5*sideLength;

  tSquareP3_x = x - 0.5*sideLength;
  tSquareP3_y = y + 0.5*sideLength;

  tSquareP4_x = x - 0.5*sideLength;
  tSquareP4_y = y - 0.5*sideLength;

  // definition of tSquareVector
  Rectangle tSquare;
  tSquare.x = tSquareP4_x; tSquare.y = tSquareP4_y; 
  tSquare.width = sideLength; tSquare.height = sideLength;
  std::vector<Rectangle> tSquareVector;
  tSquareVector.push_back(tSquare);

  for (int i=0; i<N; i++){    
    // rotate obstacles in clockwise direction by theta
    obstacleP1_x = obstacles[i].x; obstacleP1_y = obstacles[i].y;
    obstacleP2_x = obstacles[i].x + obstacles[i].width; obstacleP2_y = obstacles[i].y;
    obstacleP3_x = obstacles[i].x + obstacles[i].width; obstacleP3_y = obstacles[i].y + obstacles[i].height;
    obstacleP4_x = obstacles[i].x; obstacleP4_y = obstacles[i].y + obstacles[i].height;

    tObstacleP1_x = obstacleP1_x*cos(theta) + obstacleP1_y*sin(theta);
    tObstacleP1_y = -obstacleP1_x*sin(theta) + obstacleP1_y*cos(theta);

    tObstacleP2_x = obstacleP2_x*cos(theta) + obstacleP2_y*sin(theta);
    tObstacleP2_y = -obstacleP2_x*sin(theta) + obstacleP2_y*cos(theta);

    tObstacleP3_x = obstacleP3_x*cos(theta) + obstacleP3_y*sin(theta);
    tObstacleP3_y = -obstacleP3_x*sin(theta) + obstacleP3_y*cos(theta);

    tObstacleP4_x = obstacleP4_x*cos(theta) + obstacleP4_y*sin(theta);
    tObstacleP4_y = -obstacleP4_x*sin(theta) + obstacleP4_y*cos(theta);
    // judge whether transformed corner points of obstacles are in transformed square

    if(isValidPoint(tObstacleP1_x,tObstacleP1_y,tSquareVector)){
        std::cout << "x1,y1" <<std::endl;
    }

    if(!(isValidPoint(tObstacleP1_x,tObstacleP1_y,tSquareVector) &&
        isValidPoint(tObstacleP2_x, tObstacleP2_y,tSquareVector) &&
        isValidPoint(tObstacleP3_x, tObstacleP3_y,tSquareVector) &&
        isValidPoint(tObstacleP4_y, tObstacleP4_y,tSquareVector))
      )
    {
      return false;
    }
  }
  return true;
}

// Add any custom debug / development code here. This code will be executed
// instead of the statistics checker (Project2.cpp). Any code submitted here
// MUST compile, but will not be graded.
void debugMode(const std::vector<Robot>& /*robots*/, const std::vector<Rectangle>& /*obstacles*/, const std::vector<bool>& /*valid*/)
{
}
