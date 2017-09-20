///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Authors: Peiguang Wang, Sichao Zhang
// Date: Sept. 18, 2017
//////////////////////////////////////

#include "CollisionChecking.h"
#include "Transformation.h"

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

  squareP1_x = getRotatedPoint_X(x+0.5*sideLength,y-0.5*sideLength,x,y,theta);
  squareP1_y = getRotatedPoint_Y(x+0.5*sideLength,y-0.5*sideLength,x,y,theta);

  squareP2_x = getRotatedPoint_X(x+0.5*sideLength,y+0.5*sideLength,x,y,theta);
  squareP2_y = getRotatedPoint_Y(x+0.5*sideLength,y+0.5*sideLength,x,y,theta);

  squareP3_x = getRotatedPoint_X(x-0.5*sideLength,y+0.5*sideLength,x,y,theta);
  squareP3_y = getRotatedPoint_Y(x-0.5*sideLength,y+0.5*sideLength,x,y,theta);

  squareP4_x = getRotatedPoint_X(x-0.5*sideLength,y-0.5*sideLength,x,y,theta);
  squareP4_y = getRotatedPoint_Y(x-0.5*sideLength,y-0.5*sideLength,x,y,theta);

  /* DEBUG print points of square */
  //std::cout << "x = " << squareP1_x << " "<< squareP2_x<< " " << squareP3_x<< " " << squareP4_x << std::endl;
  //std::cout << "y = " << squareP1_y << " "<< squareP2_y<< " " << squareP3_y<< " " << squareP4_y << std::endl;
  // case 1: object's corners in obstacles
  if(!(isValidPoint(squareP1_x,squareP1_y,obstacles) &&
        isValidPoint(squareP2_x,squareP2_y,obstacles) &&
        isValidPoint(squareP3_x,squareP3_y,obstacles) &&
        isValidPoint(squareP4_x,squareP4_y,obstacles)))
  {
    // DEBUG
    // std::cout<< "stops at case 1" << std::endl;
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

    tObstacleP1_x = getRotatedPoint_X(obstacleP1_x,obstacleP1_y,x,y,-theta);
    tObstacleP1_y = getRotatedPoint_Y(obstacleP1_x,obstacleP1_y,x,y,-theta);

    tObstacleP2_x = getRotatedPoint_X(obstacleP2_x,obstacleP2_y,x,y,-theta);
    tObstacleP2_y = getRotatedPoint_Y(obstacleP2_x,obstacleP2_y,x,y,-theta);

    tObstacleP3_x = getRotatedPoint_X(obstacleP3_x,obstacleP3_y,x,y,-theta);
    tObstacleP3_y = getRotatedPoint_Y(obstacleP3_x,obstacleP3_y,x,y,-theta);

    tObstacleP4_x = getRotatedPoint_X(obstacleP4_x,obstacleP4_y,x,y,-theta);
    tObstacleP4_y = getRotatedPoint_Y(obstacleP4_x,obstacleP4_y,x,y,-theta);

    // judge whether transformed corner points of obstacles are in transformed square
    if(!(isValidPoint(tObstacleP1_x,tObstacleP1_y,tSquareVector) &&
        isValidPoint(tObstacleP2_x, tObstacleP2_y,tSquareVector) &&
        isValidPoint(tObstacleP3_x, tObstacleP3_y,tSquareVector) &&
        isValidPoint(tObstacleP4_x, tObstacleP4_y,tSquareVector))
      )
    {
      std::cout << "theta = "<< theta<<std::endl; 
      std::cout << "x = " << tObstacleP1_x << ' '<< tObstacleP2_x << ' ' << tObstacleP3_x << ' ' << tObstacleP4_x << " "<< tObstacleP1_x << std::endl;
      std::cout << "y = " << tObstacleP1_y << ' '<< tObstacleP2_y << ' ' << tObstacleP3_y << ' ' << tObstacleP4_y << " "<< tObstacleP1_y << std::endl;

      std::cout << "tSquare info:" << std::endl;
      std::cout << "x = " << tSquareP1_x << ' '<< tSquareP2_x << ' ' << tSquareP3_x << ' ' << tSquareP4_x << " "<< tSquareP1_x << std::endl;
      std::cout << "y = " << tSquareP1_y << ' '<< tSquareP2_y << ' ' << tSquareP3_y << ' ' << tSquareP4_y << " "<< tSquareP1_y << std::endl;

      std::cout << "i = " << i << std::endl;
      return false;
    }
  }
  tSquareVector.pop_back();
  return true;
}

// Add any custom debug / development code here. This code will be executed
// instead of the statistics checker (Project2.cpp). Any code submitted here
// MUST compile, but will not be graded.
void debugMode(const std::vector<Robot>& /*robots*/, const std::vector<Rectangle>& /*obstacles*/, const std::vector<bool>& /*valid*/)
{
}
