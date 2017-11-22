#ifndef COLLISION_CHECKING_H_
#define COLLISION_CHECKING_H_

#include <vector>
#include <cmath>
#include <iostream>

struct Rectangle
{
    double x, y;
    double width;
    double height;
};

void getObstacles(std::vector<Rectangle>& obstacles);

bool isValidLine(const std::pair<double, double>& line_start, 
                 const std::pair<double, double>& line_end,
                 const std::vector<Rectangle>& obstacles);

#endif
