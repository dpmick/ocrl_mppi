#include "mppi/path.hpp"

namespace mppi {

Path::Path(const pathParams pathParams, const Eigen::Vector4d goal_state, const Eigen::Vector4d init_state, const double m_accel, const Eigen::Vector2d u):
    m_params(pathParams), m_controls_vel(pathParams.steps), m_controls_ang(pathParams.steps), m_cost(pathParams.steps), m_goal_state(goal_state), m_state(init_state), latest_u(u){}


// state: x, y, theta, v
// control: v, steering angle 
void Path::state_update(Eigen::Vector4d &state, const double input_vel, const double input_ang)
{

    state(0) += input_vel * cos(state(2))*m_params.dt;
    state(1) += input_vel * sin(state(2))*m_params.dt;
    state(2) += input_vel * tan(input_ang)*m_params.dt/m_params.bike_length;
    state(3) = input_vel;

}

double Path::calculate_cost(const Eigen::Vector4d state, const double input_vel, const double input_ang, mppi::Costmap m_costmap){

    Eigen::Vector2d control = Eigen::Vector2d(input_vel, input_ang);

    m_goal_state(3) = 5.0;

    // Checking obstacles from costmap
    // if (static_cast<int>(m_costmap.vget(state(0), state(1)) == 100)){
    //     return 1e6;
    // }

    Eigen::Vector4d state_diff = state - m_goal_state;
    
    double state_cost = state_diff.transpose()*m_params.Q*state_diff;
    double control_cost = control.transpose()*m_params.R*control;

    return state_cost/2 + control_cost/2;
}

void Path::apply_constraints(double &input_vel, double &input_ang, const double prior_vel, const double prior_ang)
{
    double mean_vel = 5.0; 
    double max_vel_delta = 2.0 * m_params.dt; // max accel * dt
    double max_ang_delta = 4.0 * m_params.dt; // max w * dt

    // Constraints - First order
    input_vel = std::clamp(input_vel, -3., mean_vel);
    // input_ang = std::clamp(input_ang, -1 * M_PI/6, M_PI/6); // NOTE: Not considering wrap2pi

    // Constraints - Second order
    // input_vel = std::clamp(input_vel, prior_vel - max_vel_delta, prior_vel + max_vel_delta);
    // input_ang = std::clamp(input_ang, wrap2Pi(prior_ang - max_ang_delta), wrap2Pi(prior_ang + max_ang_delta));
}

void Path::forward_rollout(mppi::Costmap m_costmap, pcl::PointCloud<pcl::PointXYZI>::Ptr trajs, Eigen::Vector2d latest_u)
{
    wp_angle = wrap2Pi(atan2((m_goal_state(1) - m_state(1)), (m_goal_state(0) - m_state(0))) - m_state(2));
    
    // double mean_vel = latest_u(0);      // This will be the output of the mppi.control from the previous time step; the nominal input (probably)
    double mean_vel = 5.0;
    // double mean_ang = latest_u(1);
    double mean_ang = wp_angle; // Angle of line segment between car and goal waypoint
    std::random_device rd;      // RNG for the sampling. Might wanna place this in the header file to keep it out of even the outer loop (number_rollouts)?
    std::mt19937 gen(rd());

    double prior_vel = m_state(3);
    double prior_ang = m_state(2);

    Eigen::Vector4d rollout_state = m_state;

    for(int i = 0; i < m_params.steps; i++){
        // Sampling controls from a gaussian -- perturbed controls
        std::normal_distribution<double> vel_distribution(0., m_params.vel_standard_deviation);
        std::normal_distribution<double> ang_distribution(0., m_params.ang_standard_deviation);

        m_controls_vel(i) = mean_vel + vel_distribution(gen);
        m_controls_ang(i) = wrap2Pi(mean_ang + ang_distribution(gen));
        apply_constraints(m_controls_vel(i), m_controls_ang(i), prior_vel, prior_ang);
        
        state_update(rollout_state, m_controls_vel(i), m_controls_ang(i));

        wp_angle = wrap2Pi(atan2((m_goal_state(1) - rollout_state(1)), (m_goal_state(0) - rollout_state(0))) - rollout_state(2));
        mean_ang = wp_angle;

        m_cost(i) = calculate_cost(rollout_state, m_controls_vel(i), m_controls_ang(i), m_costmap); // updated cost of step

        prior_vel = m_controls_vel(i);
        prior_ang = m_controls_ang(i); 

        // To visualize the trajectories
        point.x = rollout_state(0);
        point.y = rollout_state(1);
        point.intensity = 1;
        trajs->points.push_back(point);        
    }
}

double Path::wrap2Pi(double reltheta)
{
    if (reltheta <= -M_PI)
    {
        reltheta += 2 * M_PI;
    }
    else if (reltheta > M_PI)
    {
        reltheta -= 2 * M_PI;
    }
    return reltheta;
}

}