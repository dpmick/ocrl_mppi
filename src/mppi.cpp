#include "mppi/mppi.hpp"

namespace mppi{

MPPI::MPPI(const pathParams pathParams, const mppiParams mppiParams):
    m_pathParams(pathParams), m_mppiParams(mppiParams) {}   
    
    
Eigen::Vector2d MPPI::control(Eigen::Vector4d curr_state, Eigen::Vector4d goal_state, const double acceleration = 0.0){
    Eigen::Matrix2Xd control_sequence(2, m_pathParams.steps);
    control_sequence.setZero();

    double weight = 0.0;
    double temp_weight = 0.0;

    for(int i = 0; i < m_mppiParams.number_rollouts; i++){

        mppi::Path newPath(m_pathParams, goal_state, curr_state, acceleration);
        newPath.forward_rollout();
        
        temp_weight = exp((-1/m_mppiParams.lambda)*newPath.m_cost);
        weight += temp_weight;
        control_sequence += exp(newPath.m_cost)*newPath.m_control_sequence;
    }
    
    return control_sequence/weight;
}
}