#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <random>

namespace mppi{

struct pathParams{
  int steps = 300;
  int dt = .01;
  double bike_length = .48;

  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
  Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
  
  double vel_standard_deviation = 0.0; 
  double ang_standard_deviation = 0.0; 
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
};
} //namespace mppi