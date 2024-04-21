#include "mppi/mppi.hpp"
#include <math.h>
namespace mppi{

MPPI::MPPI(const pathParams pathParams, const mppiParams mppiParams):
    m_pathParams(pathParams), m_mppiParams(mppiParams), trajs(new pcl::PointCloud<pcl::PointXYZI>()){
        
        m_nh.param<std::string>("vehicle_frame", m_vehicleframe, "cmu_rc1_base_link");
        // m_nh.param<std::string>("map_frame", m_mapframe, "cmu_rc1_odom");
        // rvizpub = m_nh.advertise<nav_msgs::Path>("/cmu_rc1/mppi/paths", 10);
        rvizpub2 = m_nh.advertise<sensor_msgs::PointCloud2>("/cmu_rc1/mppi/rollouts", 10000);
    }   
    
    
Eigen::Vector2d MPPI::control(Eigen::Vector4d curr_state, const double acceleration = 0.0){

    Eigen::Matrix2Xd control_sequence(2, m_pathParams.steps); // Ig we need to get the current control_sequence from main while calling this function, 
                                                              // and set the first controls here after popping the first controls. This will now act as
                                                              // the nominal control. Initially it will be zero or sampled separately (outside the loop)?

    control_sequence.setZero();         // Do we need to set this to zero at each time step?

    // std::cout << "Entered mppi.cpp::control" << std::endl;

    Eigen::Vector2d u;
    // Eigen::Matrix2d du;

    Eigen::Vector4d goal_statedef;
    
    if (m_goal_state_buf.size() < 2){
        u = Eigen::Vector2d(0.0,0.0);
        return u;
    } else{
        if (m_goal_state_buf.size() == 2){
            goal_statedef = m_goal_state_buf.back();
        } else{
            goal_statedef = m_goal_state_buf.back();
            m_goal_state_buf.clear();
        }

        Eigen::MatrixXd du(2, m_pathParams.steps);
        Eigen::MatrixXd traj_weighted_combo(2, m_mppiParams.number_rollouts);
        Eigen::MatrixXd d_vel(m_mppiParams.number_rollouts, m_pathParams.steps);
        Eigen::MatrixXd d_steer(m_mppiParams.number_rollouts, m_pathParams.steps);

        Eigen::MatrixXd all_costs(m_mppiParams.number_rollouts, m_pathParams.steps);

        // eq. 34 on mppi paper
        trajs->points.clear();

        // storing relevant values from every rollout

        for(int k = 0; k < m_mppiParams.number_rollouts; k++){


            mppi::Path newPath(m_pathParams, goal_statedef, curr_state, acceleration);
            newPath.forward_rollout(m_costmap, trajs);

            d_vel.row(k) = newPath.m_controls_vel;
            d_steer.row(k) = newPath.m_controls_steer;
            all_costs.row(k) = newPath.m_cost;
        }

        double min_cost = 0;
        Eigen::VectorXd weighted_cost(m_mppiParams.number_rollouts);

        // weighted update rule (eq 34)

        for (int i = 0; i < m_pathParams.steps; i++){

            min_cost = all_costs.col(i).minCoeff();
            all_costs.array().col(i) -= min_cost;

            weighted_cost = exp((-1.0/m_mppiParams.lambda) * all_costs.col(i).array()) + 1e-6; // exploration/exploitation of every step in traj

            weighted_cost = weighted_cost.array() / weighted_cost.sum();  // normalization

            du(0, i) += weighted_cost.dot(d_vel.col(i));
            du(1, i) += weighted_cost.dot(d_steer.col(i));
        }
        // To visualize the trajectories
        std::cout << "TRAJ SIZE: " << trajs->points.size() << std::endl;
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*trajs, output);

        output.header.frame_id = m_vehicleframe;
        output.header.stamp = ros::Time::now();

        // std::cout << "OUTPUT SIZE: " << output.data.size() << std::endl;

        rvizpub2.publish(output);
        
        u = du.col(0);

        u(0) = std::clamp(u(0), -3., 5.);
        u(1) = std::clamp(u(1), -1 * M_PI/6, M_PI/6);

        // std::cout << "u (Inside mppi::control): " << u << std::endl;

        return u;         
    }
}

void MPPI::registerGoalState(Eigen::Vector4d goal_state){
    m_goal_state_buf.push_back(goal_state);
}

}