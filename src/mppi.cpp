#include "mppi/mppi.hpp"

namespace mppi{

MPPI::MPPI(const pathParams pathParams, const mppiParams mppiParams):
    m_pathParams(pathParams), m_mppiParams(mppiParams) {}   
    
    
Eigen::Vector2d MPPI::control(Eigen::Vector4d curr_state, Eigen::Vector4d goal_state, const double acceleration = 0.0){

    Eigen::Matrix2Xd control_sequence(2, m_pathParams.steps); // Ig we need to get the current control_sequence from main while calling this function, 
                                                              // and set the first controls here after popping the first controls. This will now act as
                                                              // the nominal control. Initially it will be zero or sampled separately (outside the loop)?

    control_sequence.setZero();         // Do we need to set this to zero at each time step?

    double weight = 0.0;
    double temp_weight = 0.0;
    Eigen::Vector2d u;
    Eigen::Vector2d du;

    for(int i = 0; i < m_mppiParams.number_rollouts; i++){

        mppi::Path newPath(m_pathParams, goal_state, curr_state, acceleration);
        newPath.forward_rollout();
        
        temp_weight = exp((-1/m_mppiParams.lambda)*newPath.m_cost);
        weight += temp_weight;          // Denominator for calculating u
        du += temp_weight*newPath.m_control_sequence;       // Numerator for calculating u
    }

    u = control_sequence + du/weight;          // Nominal control + weighted sum of sampled trajectories
    return u;         // Ig we need to apply the 0th control and the 1st one will be the nominal control for the next time step
}
}