#include "mppi/path.hpp"

namespace mppi {

Path::Path(const pathParams pathParams, const Eigen::Vector4d goal_state, const Eigen::Vector4d init_state, const double m_accel):
    m_params(pathParams), m_controls_vel(pathParams.steps), m_controls_steer(pathParams.steps), m_cost(pathParams.steps), m_goal_state(goal_state), m_state(init_state){}

// state: x, y, theta, v
// control: v, steering angle 

double Path::calculate_cost(const Eigen::Vector4d state, const double input_vel, const double input_ang){
    Eigen::Vector2d control = Eigen::Vector2d(input_vel, input_ang);

    m_goal_state(3) = 5.0;

    Eigen::Vector4d state_diff = state-m_goal_state; //  validated
    
    double state_cost = state_diff.transpose()*m_params.Q*state_diff;
    double control_cost = control.transpose()*m_params.R*control;

    return state_cost/2 + control_cost/2;
}

void Path::forward_rollout()
{
    double mean_vel = 5.0;      // This will be the output of the mppi.control from the previous time step; the nominal input (probably) -- updated to a speed we know will move car fwds
    double mean_ang = 0.0;
    std::random_device rd;      // RNG for the sampling. Might wanna place this in the header file to keep it out of even the outer loop (number_rollouts)?
    std::mt19937 gen(rd());

    Eigen::Vector4d rollout_state = m_state;
    
    for(int i = 0; i < m_params.steps; i++){
        // Sampling controls from a gaussian -- perturbed controls
        std::normal_distribution<double> vel_distribution(0., m_params.vel_standard_deviation);
        std::normal_distribution<double> ang_distribution(0., m_params.ang_standard_deviation);

        m_controls_vel(i) = mean_vel + vel_distribution(gen);
        m_controls_steer(i) = mean_ang + ang_distribution(gen);

        m_controls_vel(i) = std::clamp(m_controls_vel(i), -3., mean_vel);
        m_controls_steer(i) = std::clamp(m_controls_steer(i), -1 * M_PI/6, M_PI/6);

        // moved state update here

        rollout_state(0) = m_controls_vel(i)*cos(rollout_state(2))*m_params.dt + rollout_state(0);
        rollout_state(1) = m_controls_vel(i)*sin(rollout_state(2))*m_params.dt + rollout_state(1);
        rollout_state(2) = m_controls_vel(i)*tan(m_controls_steer(i))*m_params.dt/m_params.bike_length + rollout_state(2);
        rollout_state(3) = m_controls_vel(i);

        m_cost(i) = calculate_cost(rollout_state, m_controls_vel(i), m_controls_steer(i)); // updated cost of step
    }
}

}