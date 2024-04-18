#include "mppi/path.hpp"

namespace mppi {

Path::Path(const pathParams pathParams, const Eigen::Vector4d goal_state, const Eigen::Vector4d init_state, const double m_accel):
    m_params(pathParams), m_control_sequence(2, pathParams.steps), m_cost(0.0), m_goal_state(goal_state), m_state(init_state) {}

// state: x, y, theta, v
// control: v, steering angle 
void Path::state_update(Eigen::Vector4d state, const double input_vel, const double input_ang)
{
    state(0) = input_vel*cos(state(2))*m_params.dt + state(0);
    state(1) = input_vel*sin(state(2))*m_params.dt + state(1);
    state(2) = input_vel*tan(input_ang)*m_params.dt/m_params.bike_length + state(2);
    state(3) = m_accel*m_params.dt + state(3);

}

double Path::calculate_cost(const Eigen::Vector4d state, const double input_vel, const double input_ang){
    Eigen::Vector2d control = Eigen::Vector2d(input_vel, input_ang);

    m_goal_state(3) = 5.0;

    Eigen::Vector4d state_diff = state-m_goal_state; //  validated
    
    double state_cost = state_diff.transpose()*m_params.Q*state_diff;
    double control_cost = control.transpose()*m_params.R*control;

    std::cout << "state cost:   \n" << state_cost << std::endl;
    std::cout << "control cost: \n" << control_cost << std::endl;

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

        m_control_sequence(0,i) = mean_vel + vel_distribution(gen);
        m_control_sequence(1,i) = mean_ang + ang_distribution(gen);

        m_control_sequence(0,i) = std::clamp(m_control_sequence(0,i), -3., mean_vel);
        m_control_sequence(1,i) = std::clamp(m_control_sequence(1,i), -1 * M_PI/6, M_PI/6);

        // moved state update here

        rollout_state(0) = m_control_sequence(0,i)*cos(rollout_state(2))*m_params.dt + rollout_state(0);
        rollout_state(1) = m_control_sequence(0,i)*sin(rollout_state(2))*m_params.dt + rollout_state(1);
        rollout_state(2) = m_control_sequence(0,i)*tan(m_control_sequence(1,i))*m_params.dt/m_params.bike_length + rollout_state(2);
        rollout_state(3) = m_control_sequence(0,i);

        m_cost += calculate_cost(rollout_state, m_control_sequence(0,i), m_control_sequence(1,i)); // trajectory cost
    }
}

}