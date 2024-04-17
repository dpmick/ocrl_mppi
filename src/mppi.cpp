#include "mppi/mppi.hpp"
#include <math.h>
namespace mppi{

MPPI::MPPI(const pathParams pathParams, const mppiParams mppiParams):
    m_pathParams(pathParams), m_mppiParams(mppiParams){}   
    
    
Eigen::Vector2d MPPI::control(Eigen::Vector4d curr_state, const double acceleration = 0.0){

    Eigen::Matrix2Xd control_sequence(2, m_pathParams.steps); // Ig we need to get the current control_sequence from main while calling this function, 
                                                              // and set the first controls here after popping the first controls. This will now act as
                                                              // the nominal control. Initially it will be zero or sampled separately (outside the loop)?

    control_sequence.setZero();         // Do we need to set this to zero at each time step?

    std::cout << "Entered mppi.cpp::control" << std::endl;

    double weight = 0.0;
    double traj_temp_weight = 0.0;
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

        Eigen::MatrixXd du(2, m_mppiParams.number_rollouts);
        Eigen::MatrixXd traj_weighted_combo(2, m_mppiParams.number_rollouts);
        Eigen::VectorXd all_costs(m_mppiParams.number_rollouts);
        du.setZero();
        double min_cost = 0;

        for(int i = 0; i < m_mppiParams.number_rollouts; i++){

            mppi::Path newPath(m_pathParams, goal_statedef, curr_state, acceleration);
            newPath.forward_rollout(m_costmap);

            all_costs(i) = newPath.m_cost;
            min_cost = all_costs.minCoeff();

            traj_temp_weight = exp((-1.0/m_mppiParams.lambda)*(newPath.m_cost - min_cost));

            traj_weighted_combo = traj_temp_weight*newPath.m_control_sequence; // 2 x steps

            du += traj_weighted_combo / traj_weighted_combo.sum();  
        }

        u = du.col(0);

        u(0) = std::clamp(u(0), -3., 5.);
        u(1) = std::clamp(u(1), -1 * M_PI/6, M_PI/6);

        std::cout << "u (Inside mppi::control): " << u << std::endl;

        return u;         
    }
}

void MPPI::registerGoalState(Eigen::Vector4d goal_state){
    m_goal_state_buf.push_back(goal_state);
}

}