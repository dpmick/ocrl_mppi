#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <math.h>
#include "mppi/costmap.hpp"

namespace mppi{

struct pathParams{
  int steps;
  int dt;
  double bike_length;
  Eigen::Matrix4d Q;
  Eigen::Matrix2d R;

  double vel_standard_deviation;
  double ang_standard_deviation;
};

class Path{
public:

  Path(const pathParams params, const Eigen::Vector4d goal_state, const Eigen::Vector4d init_state, const double accel);

  Eigen::Matrix2Xd m_control_sequence;
  Eigen::Vector4d m_state;
  Eigen::Vector4d m_goal_state;
  double m_cost;
  double m_accel;
  pathParams m_params;

  void state_update(Eigen::Vector4d &state, const double input_vel, const double input_ang);
  void forward_rollout();
  double calculate_cost(const Eigen::Vector4d state, const double input_vel, const double input_ang);

  // Costmap stuff
  mppi::Costmap m_costmap; // Costmap class object

  // Occupancy grid
  nav_msgs::OccupancyGrid m_occupancyGrid;
  bool m_hascostmap = false;

  // Costmap subscriber
  ros::Subscriber m_costmap_sub; 

  void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void updatemap();

};
} //namespace mppi