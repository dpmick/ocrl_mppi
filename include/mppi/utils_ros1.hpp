#pragma once

#include "mppi/path.hpp"
#include "mppi/mppi.hpp"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>

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

void odomMsgToState(const nav_msgs::Odometry::ConstPtr &odometry, Eigen::Vector4d &state);

void goalMsgToState(const geometry_msgs::PoseArray::ConstPtr &goal, std::deque<Eigen::Vector4d> &goal_array);

void controlToMsg(const Eigen::Vector2d &control, geometry_msgs::TwistStamped &cmdMsg);   

void occMsgtoMap(const nav_msgs::OccupancyGrid::ConstPtr &occMsg, mppi::Costmap &m_costmap);

}  