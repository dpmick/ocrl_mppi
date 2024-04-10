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
    double weight = 0.0;
    double temp_weight = 0.0;
    Eigen::Vector2d u;
    Eigen::Matrix2d du;

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
        for(int i = 0; i < m_mppiParams.number_rollouts; i++){
            mppi::Path newPath(m_pathParams, goal_statedef, curr_state, acceleration);
            newPath.forward_rollout();
            temp_weight = exp((-1.0/m_mppiParams.lambda)*newPath.m_cost);
            weight += temp_weight;          // Denominator for calculating u
            
            du += temp_weight*newPath.m_control_sequence;       // Numerator for calculating u
        }

        u = control_sequence.col(0) + du.col(0)/weight; // executing the first sequence of controls
                        // Nominal control + weighted sum of sampled trajectories
        return u;         // Ig we need to apply the 0th control and the 1st one will be the nominal control for the next time step
    }
}

void MPPI::registerGoalState(Eigen::Vector4d goal_state){
    m_goal_state_buf.push_back(goal_state);
}

}