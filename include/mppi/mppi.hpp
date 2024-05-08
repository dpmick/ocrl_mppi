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

    Eigen::Vector2d m_latest_u;
    
    mppi::Costmap m_costmap;

    Eigen::Vector4d goal_statedef;

    double dist;
    double m_target_speed;

    std::deque<Eigen::Vector4d> m_goal_state_buf {Eigen::Vector4d(0.0,0.0,0.0,0.0)};

    Eigen::Vector2d control(Eigen::Vector4d state, double m_target_speed, const double acceleration);

    void registerGoalState(std::deque<Eigen::Vector4d> goal_array);
    
    // To visualize paths
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_trajs;
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_selectedTraj;
    pcl::PointXYZI selectedPoint;
};

}