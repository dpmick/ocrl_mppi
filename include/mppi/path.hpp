#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <math.h>  

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

  Eigen::VectorXd m_controls_vel;
  Eigen::VectorXd m_controls_steer;
  
  Eigen::Vector4d m_state;
  Eigen::Vector4d m_goal_state;
  Eigen::VectorXd m_cost;
  double m_accel;
  pathParams m_params;

  void forward_rollout();
  double calculate_cost(const Eigen::Vector4d state, const double input_vel, const double input_ang);
};
} //namespace mppi