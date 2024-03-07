#pragma once

#include "mppi/path.hpp"
#include "mppi/mppi.hpp"

#include <ros/ros.h>


namespace mppi::ros1 {

void getParam(ros::NodeHandle &node, const std::string &paramName,
     double &param);

void getParam(ros::NodeHandle &node, const std::string &paramName,
     int &param);

void getParam(ros::NodeHandle &node, const std::string &paramName,
    Eigen::Matrix4d &param);

void getParam(ros::NodeHandle &node, const std::string &paramName,
    Eigen::Matrix2d &param);

struct ROSParams{
    pathParams path_params;
    mppiParams mppi_params;

};

ROSParams getParams(ros::NodeHandle &node);

}  