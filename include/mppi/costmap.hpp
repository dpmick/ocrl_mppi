#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <iostream>
#include "mppi/path.hpp"
#include "mppi/mppi.hpp"
#include <deque>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>


namespace mppi{
{
    
} // namespace mppi{


class Costmap
{
public:
    /// @brief Empty Constructor
    Costmap() {}

    /// @brief Default Constructor
    Costmap(double minXArg, double minYArg, double resolutionArg, int widthArg, int heightArg);

    std::pair<int, int> indexToRC(int index) const;
    bool inMap(double x, double y) const;
    int xToCol(double x) const;
    int xToCol(double x) const;
    double colToX(int col) const;
    int yToRow(double y) const;
    double rowToY(int row) const;
    uint8_t get(int r, int c) const;
    uint8_t vget(double x, double y) const;

    std::vector<uint8_t> data;
    double resolution;
    double min_x;
    double min_y;
    double max_x;
    double max_y;
    int width;
    int height;

}
}