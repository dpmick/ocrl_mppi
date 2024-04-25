#pragma once

#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <iostream>
#include "mppi/path.hpp"
#include <deque>
#include <math.h>    
#include "mppi/costmap.hpp"

namespace mppi {

struct mppiParams{
    int number_rollouts;
    double lambda;
};

class MPPI{
public:
    MPPI(const pathParams pathParams, const mppiParams mppiParams);

    pathParams m_pathParams;
    mppiParams m_mppiParams;

    Eigen::Vector2d latest_u;

    mppi::Costmap m_costmap;

    std::deque<Eigen::Vector4d> m_goal_state_buf {Eigen::Vector4d(0.0,0.0,0.0,0.0)};

    Eigen::Vector2d control(Eigen::Vector4d state, const double acceleration);

    void registerGoalState(Eigen::Vector4d goal_state);

    // Member ROS Handle
    ros::NodeHandle m_nh;

    // To visualize paths
    pcl::PointCloud<pcl::PointXYZI>::Ptr trajs;
    pcl::PointCloud<pcl::PointXYZI>::Ptr selectedTraj;
    pcl::PointXYZI selectedPoint;
    std::string m_vehicleframe;
    // std::string m_mapframe;
    ros::Publisher rvizpub2;
    ros::Publisher rvizpathpub;
    // pcl::PointXYZI point;
    // nav_msgs::Path pathviz;
    // geometry_msgs::PoseStamped pose;
};

}