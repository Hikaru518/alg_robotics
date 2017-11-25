#include "isLineIntersect.h"

#include <vector>
#include <cmath>
#include <iostream>

// A node's position related to a line
int NodeRelateLine(double line_x, double line_y, double node_x, double node_y)
{
    double rel = node_x * line_y - node_x * line_y;
    if (rel > 0)
    {
     return 1;
    } 
     else if (rel < 0)
     {
      return -1;
     } 
      else
      {
       return 0;
      }
}

// If a node is in the x domain of a line
bool isPointOnLine(double line_x1, double line_x2, double node_x)
{
    if ( (std::min(line_x1, line_x2) < node_x) && (node_x < std::max(line_x1, line_x2)) ) 
    {
      return true;
    } 
    else 
    {
       return false;
    }
}

// If two lines intersect
bool isLineIntersect(
    const std::pair<double, double>& line1_start, 
    const std::pair<double, double>& line1_end,
    const std::pair<double, double>& line2_start,
    const std::pair<double, double>& line2_end)
{
    double x1 = line1_start.first, x2 = line1_end.first, x3 = line2_start.first, x4 = line2_end.first;
    double y1 = line1_start.second, y2 = line1_end.second, y3 = line2_start.second, y4 = line2_end.second;
    int rel1 = NodeRelateLine(x2 - x1, y2 - y1, x3 - x1, y3 - y1);
    int rel2 = NodeRelateLine(x2 - x1, y2 - y1, x4 - x1, y4 - y1);
    int rel3 = NodeRelateLine(x4 - x3, y4 - y3, x1 - x3, y1 - y3);
    int rel4 = NodeRelateLine(x4 - x3, y4 - y3, x2 - x3, y2 - y3);
    
    if (rel1 == 0 && isPointOnLine(x1, x2, x3)) return true;

    if (rel2 == 0 && isPointOnLine(x1, x2, x4)) return true;

    if (rel3 == 0 && isPointOnLine(x3, x4, x1)) return true;

    if (rel4 == 0 && isPointOnLine(x3, x4, x2)) return true;
    
    if (rel1 == rel2 || rel3 == rel4) 
    {
        return false;
    } 
    else 
     {
        if (rel1 != 0 && rel2 != 0 && rel3 != 0 && rel4 != 0) 
         {
            return true;
         } 
          else 
           {
            return false;
           }
     }
}

