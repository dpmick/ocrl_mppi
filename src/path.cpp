#include "mppi/path.hpp"

namespace mppi {

Path::Path(const pathParams pathParams, const Eigen::Vector4d goal_state, const Eigen::Vector4d init_state, const double m_accel, const double m_target_speed, const Eigen::Vector2d u):
    m_params(pathParams), m_controls_vel(pathParams.steps), m_controls_ang(pathParams.steps), m_cost(pathParams.steps), m_goal_state(goal_state), m_state(init_state), m_target_speed(m_target_speed), latest_u(u){}


// state: x, y, theta, v
// control: v, steering angle 
void Path::state_update(Eigen::Vector4d &state, const double input_vel, const double input_ang)
{

    state(0) += state(3) * cos(state(2))*m_params.dt;
    state(1) += state(3) * sin(state(2))*m_params.dt;
    state(2) += state(3) * tan(input_ang)*m_params.dt / m_params.bike_length;
    state(3) = input_vel;

}


double Path::calculate_cost(const Eigen::Vector4d state, const double input_vel, const double input_ang, mppi::Costmap m_costmap){

    Eigen::Vector2d control = Eigen::Vector2d(input_vel, input_ang);

    if (m_target_speed != 0){
        m_goal_state(3) = m_target_speed; // If target speed is given, the target speed is the given speed
    }
    else {
        m_goal_state(3) = state(3); // If no target speed is given, the target speed is the current speed
    }    

    Eigen::Vector4d state_diff = state - m_goal_state;
    
    double state_cost = state_diff.transpose()*m_params.Q*state_diff;
    double control_cost = control.transpose()*m_params.R*control;

    // cost on vehicle roll
    // double cent_a = pow(input_vel, 2) / (0.48 / tan(input_ang));
    // if (cent_a > 0.7) {
    //     control_cost += 1e10;
    // }

    // // Checking obstacles from costmap
    if (static_cast<int>(m_costmap.vget(state(0), state(1)) == 100)){
        state_cost += 1e64;
    }

    return state_cost/2 + control_cost/2;
}

void Path::apply_constraints(double &input_vel, double &input_ang)
{
    // setting default speed to 2.0
    if (m_target_speed == 0){
        m_target_speed = 2.; 
    }

    // first order constraints
    input_vel = std::clamp(input_vel, -10., abs(m_target_speed)); 
    input_ang = std::clamp(input_ang, -1 * M_PI/6, M_PI/6);

}


void Path::forward_rollout(mppi::Costmap m_costmap, pcl::PointCloud<pcl::PointXYZI>::Ptr trajs, Eigen::Vector2d latest_u)
{
    wp_angle = wrap2Pi(atan2((m_goal_state(1) - m_state(1)), (m_goal_state(0) - m_state(0))) - m_state(2));
    
    double mean_vel = latest_u(0);      // Starts sampling off of the most recently executed velocity
    double mean_ang = wp_angle; // Angle of line segment between car and goal waypoint
    std::random_device rd;      // RNG for the sampling. Might wanna place this in the header file to keep it out of even the outer loop (number_rollouts)?
    std::mt19937 gen(rd());
    
    double max_acc = 4.0;
    double max_deacc = 10.0;
    double max_steer_rate = 4.0;

    double prior_vel = m_state(3);
    double prior_ang = m_state(2);

    Eigen::Vector4d rollout_state = m_state;

    std::normal_distribution<double> throttle_distribution(0., m_params.throttle_standard_deviation);
    double throttle_effort = throttle_distribution(gen);
    throttle_effort = -std::clamp(throttle_effort, -1., 1.);

    Eigen::Vector4d rollout_state = m_state;
    
    for(int i = 0; i < m_params.steps; i++){
        // Sampling controls from a gaussian -- perturbed controls

        // What if we sampled about effort?
        
        std::normal_distribution<double> ang_distribution(0., m_params.ang_standard_deviation);


        double steer_effort = ang_distribution(gen);
        steer_effort = std::clamp(steer_effort, -1., 1.);

        // implicit 2nd order constraint
        if (throttle_effort <= 0){
            m_controls_vel(i) = mean_vel + max_deacc * throttle_effort * m_params.dt; //  4. * m_params.dt;
        }
        else{
            m_controls_vel(i) = mean_vel + max_acc * throttle_effort * m_params.dt; 
        }
        
        m_controls_ang(i) = wrap2Pi(mean_ang + max_steer_rate * steer_effort * m_params.dt);

        apply_constraints(m_controls_vel(i), m_controls_ang(i));

        state_update(rollout_state, m_controls_vel(i), m_controls_ang(i));

        wp_angle = wrap2Pi(atan2((m_goal_state(1) - rollout_state(1)), (m_goal_state(0) - rollout_state(0))) - rollout_state(2));

        mean_vel = m_controls_vel(i);
        mean_ang = m_controls_ang(i);

        m_cost(i) = calculate_cost(rollout_state, m_controls_vel(i), m_controls_ang(i), m_costmap); // updated cost of step

        if (m_cost(i) > 1e20){
            m_controls_vel(i) = 0.;
        }

        mean_vel = m_controls_vel(i);
        mean_ang = m_controls_ang(i);

        // std::cout << "cost " << m_cost(i) << std::endl;
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