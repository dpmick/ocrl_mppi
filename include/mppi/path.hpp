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

  Eigen::Matrix2Xd m_control_sequence;
  Eigen::Vector4d m_state;
  Eigen::Vector4d m_goal_state;
  double m_cost;
  double m_accel;
  double m_target_speed;
  pathParams m_params;

  Path(const pathParams params, const Eigen::Vector4d goal_state, const Eigen::Vector4d init_state, const double accel, const double m_target_speed);

  void state_update(Eigen::Vector4d &state, const double input_vel, const double input_ang);
  void forward_rollout();
  double calculate_cost(const Eigen::Vector4d state, const double input_vel, const double input_ang);

};
} //namespace mppi