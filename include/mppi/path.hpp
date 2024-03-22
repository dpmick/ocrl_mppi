#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <random>

namespace mppi{

struct pathParams{
  int steps = 300;
  int dt = 0.1;
  double bike_length = .48;
  Eigen::Matrix4d Q(100, 150, 100, 30);
  Eigen::Matrix2d R(50, 10);

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
};
} //namespace mppi