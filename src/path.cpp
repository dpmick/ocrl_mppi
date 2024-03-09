#include "mppi/path.hpp"

namespace mppi {

Path::Path(const pathParams pathParams, const Eigen::Vector4d goal_state, const Eigen::Vector4d init_state, const double m_accel):
    m_params(pathParams), m_control_sequence(2, pathParams.steps), m_cost(0.0), m_goal_state(goal_state), m_state(init_state) {}

// state: x, y, theta, v
// control: v, steering angle 
void Path::state_update(Eigen::Vector4d &state, const double input_vel, const double input_ang)
{
    state(0) = input_vel*cos(state(2))*m_params.dt + state(0);
    state(1) = input_vel*sin(state(2))*m_params.dt + state(1);
    state(2) = input_vel*tan(input_ang)*m_params.dt/m_params.bike_length + state(2);
    state(3) = m_accel*m_params.dt + state(3);
}

double Path::calculate_cost(const Eigen::Vector4d state, const double input_vel, const double input_ang){
    Eigen::Vector2d control = Eigen::Vector2d(input_vel, input_ang);
    Eigen::Vector4d state_diff = state-m_goal_state;
    
    double state_cost = state_diff.transpose()*m_params.Q*state_diff;
    double control_cost = control.transpose()*m_params.R*control;

    return state_cost + control_cost;
}

void Path::forward_rollout()
{
    for(int i = 0; i < m_params.steps; i++){
        // add sampling
        m_control_sequence(0,i) = 0.0;
        m_control_sequence(1,i) = 0.0;

        state_update(m_state, m_control_sequence(0,i), m_control_sequence(1,i));
        m_cost += calculate_cost(m_state, m_control_sequence(0,i), m_control_sequence(1,i));
    }
}

}