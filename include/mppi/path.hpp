#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <math.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>

#include "mppi/costmap.hpp"

namespace mppi{

struct pathParams{
  int steps;
  double dt;
  double bike_length;
  Eigen::Matrix4d Q;
  Eigen::Matrix2d R;

  double vel_standard_deviation;
  double ang_standard_deviation;
};

class Path{
public:

  Path(const pathParams params, const Eigen::Vector4d goal_state, const Eigen::Vector4d init_state, const double accel, const Eigen::Vector2d u);

  Eigen::VectorXd m_controls_vel;
  Eigen::VectorXd m_controls_ang;

  Eigen::Vector4d m_state;
  Eigen::Vector4d m_goal_state;
  Eigen::VectorXd m_cost;

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
  void apply_constraints(double &input_vel, double &input_ang, const double prior_vel, const double prior_ang);

  // // Member ROS Handle
  // ros::NodeHandle m_nh;

  // // To visualize paths
  // pcl::PointCloud<pcl::PointXYZI>::Ptr traj;
  // std::string m_vehicleframe;
  // // std::string m_mapframe;
  // ros::Publisher rvizpub;
  pcl::PointXYZI point;
  // // nav_msgs::Path pathviz;
  // // geometry_msgs::PoseStamped pose;

};
} //namespace mppi