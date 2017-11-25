#include "ManipulatorCollisionChecking.h"
#include "isLineIntersect.h"

#include <vector>
#include <cmath>
#include <iostream>


void getObstacles(std::vector<Rectangle>& obstacles)
{
    Rectangle rect;
    rect.x = -1.0;
    rect.y = 3.0;
    rect.height = 1.0;
    rect.width = 2.0;
    obstacles.push_back(rect);
}

bool isValidLine(const std::pair<double, double>& line_start, 
                 const std::pair<double, double>& line_end,
                 const std::vector<Rectangle>& obstacles)
{
  for (int i = 0; i<obstacles.size(); i++)
  {
    double line_x1 = line_start.first;
    double line_y1 = line_start.second;
    double line_x2 = line_end.first;
    double line_y2 = line_end.second;
    double ob_x = obstacles[i].x;
    double ob_y = obstacles[i].y;
    double ob_wid = obstacles[i].width;
    double ob_hei = obstacles[i].height;
    pts[0] = std:pair<line_x1,line_y1>;
    pts[1] = std:pair<line_x2,line_y2>;
    pts[2] = std:pair<ob_x,ob_y>;
    pts[3] = std:pair<(ob_x + ob_wid),ob_y>;
    pts[4] = std:pair<ob_x,(ob_y + ob_hei)>;
    pts[5] = std:pair<(ob_x + ob_wid),(ob_y + ob_hei)>;
    
    if( bool isLineIntersect(pts[0],pts[1],pts[2],pts[3])
     || bool isLineIntersect(pts[0],pts[1],pts[2],pts[4])
     || bool isLineIntersect(pts[0],pts[1],pts[3],pts[5])
     || bool isLineIntersect(pts[0],pts[1],pts[4],pts[5]))
    {
        return false;
    }
  }
  return true;
}



