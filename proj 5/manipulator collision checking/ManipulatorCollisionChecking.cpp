//#include "ManipulatorCollisionChecking.h"
#include "isLineIntersect.h"

#include <vector>
#include <cmath>
#include <iostream>

typedef std::pair<double,double> Point2D;
typedef std::vector<Point2D> Rectangle;

bool isLinePoint(
    const std::pair<double, double>& line_start, 
    const std::pair<double, double>& line_end,
    double ob_x, double ob_y, double ob_wid, double ob_hei)
{
    double line_x1 = line_start.first;
    double line_y1 = line_start.second;
    double line_x2 = line_end.first;
    double line_y2 = line_end.second;
    double ob_x = -1;
    double ob_y = 3;
    double ob_wid = 2;
    double ob_hei = 1;
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
  return true;
}



