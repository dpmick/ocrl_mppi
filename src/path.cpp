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

// Getting obstacles map from the voxel grid
m_costmap_sub = m_nh.subscribe("/" + m_systemid + "/local_mapping_lidar_node/voxel_grid/obstacle_map", 1, &Path::costmapCallback, this);

void Path::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    m_hascostmap = true;
    m_occupancyGrid = *msg;
}

void Path::updatemap()
{
    m_costmap = Costmap(m_occupancyGrid.info.origin.position.x,
                        m_occupancyGrid.info.origin.position.y,
                        m_occupancyGrid.info.resolution,
                        m_occupancyGrid.info.width,
                        m_occupancyGrid.info.height);

    for (auto &cell : m_occupancyGrid.data)
    {
        m_costmap.data.push_back(static_cast<int>(cell));
    }
}

double Path::calculate_cost(const Eigen::Vector4d state, const double input_vel, const double input_ang){
    // Getting obstacles from costmap
    // updatemap();
    // m_costmap.vget(); // TODO: Use the grid value to assign cost

    Eigen::Vector2d control = Eigen::Vector2d(input_vel, input_ang);
    Eigen::Vector4d state_diff = state-m_goal_state;
    
    double state_cost = state_diff.transpose()*m_params.Q*state_diff;
    double control_cost = control.transpose()*m_params.R*control;

    return state_cost + control_cost;
}

void Path::forward_rollout()
{
    double mean_vel = 1.0;      // This will be the output of the mppi.control from the previous time step; the nominal input (probably)
    double mean_ang = 0.0;
    std::random_device rd;      // RNG for the sampling. Might wanna place this in the header file to keep it out of even the outer loop (number_rollouts)?
    std::mt19937 gen(rd());

    for(int i = 0; i < m_params.steps; i++){
        // std::cout << "37\n";
        // Sampling controls from a gaussian
        std::normal_distribution<double> vel_distribution(mean_vel, m_params.vel_standard_deviation);
        std::normal_distribution<double> ang_distribution(mean_ang, m_params.ang_standard_deviation);

        m_control_sequence(0,i) = vel_distribution(gen);
        m_control_sequence(1,i) = std::clamp(ang_distribution(gen), -M_PI/6, M_PI/6);   // Clamping steering angle 

        state_update(m_state, m_control_sequence(0,i), m_control_sequence(1,i));
        m_cost += calculate_cost(m_state, m_control_sequence(0,i), m_control_sequence(1,i));

        mean_vel = m_control_sequence(0,i);
        mean_ang = m_control_sequence(1,i);
    }
}

}