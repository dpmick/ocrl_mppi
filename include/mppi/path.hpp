#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <iostream>
#include <random>
<<<<<<< HEAD
#include <math.h>  
=======
#include <math.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>

#include "mppi/costmap.hpp"
>>>>>>> darwin_saving

namespace mppi{

struct pathParams{
  int steps;
  double dt;
  double bike_length;
  Eigen::Matrix4d Q;
  Eigen::Matrix2d R;

  double throttle_standard_deviation;
  double ang_standard_deviation;
};

class Path{
public:

  Path(const pathParams params, const Eigen::Vector4d goal_state, const Eigen::Vector4d init_state, const double accel, double m_target_speed, const Eigen::Vector2d u);

  // std::default_random_engine gen; // RNG for the sampling. Might wanna place this in the header file to keep it out of even the outer loop (number_rollouts)?

  Eigen::VectorXd m_controls_vel;
  Eigen::VectorXd m_controls_ang;

  Eigen::Vector4d m_state;
  Eigen::Vector4d m_goal_state;
  Eigen::VectorXd m_cost;
  double m_target_speed;

  Eigen::Vector2d latest_u;

  double m_accel;
  pathParams m_params;

  double x;
  double y;

  double prior_vel;
  double prior_ang; 

  void state_update(Eigen::Vector4d &state, const double input_vel, const double input_ang);
  void forward_rollout(mppi::Costmap m_costmap, pcl::PointCloud<pcl::PointXYZI>::Ptr trajs, Eigen::Vector2d u);
  double calculate_cost(const Eigen::Vector4d state, const double input_vel, const double input_ang, mppi::Costmap m_costmap);
  void apply_constraints(double &input_vel, double &input_ang);

  pcl::PointXYZI point;

  double wrap2Pi(double reltheta);
  double wp_angle; // Angle of line segment between car and goal waypoint

};
} //namespace mppi