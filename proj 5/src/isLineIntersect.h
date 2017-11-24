#ifndef COLLISION_CHECKING_H_
#define COLLISION_CHECKING_H_

#include <vector>
#include <cmath>
#include <iostream>

int NodeRelateLine(double line_x, double line_y, double node_x, double node_y);

bool isPointOnLine(double line_x1, double line_x2, double node_x);

bool isLineIntersect(
    const std::pair<double, double>& line1_start, 
    const std::pair<double, double>& line1_end,
    const std::pair<double, double>& line2_start,
    const std::pair<double, double>& line2_end);

#endif
