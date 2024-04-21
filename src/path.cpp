#include "mppi/path.hpp"

namespace mppi {

Path::Path(const pathParams pathParams, const Eigen::Vector4d goal_state, const Eigen::Vector4d init_state, const double m_accel):
    m_params(pathParams), m_controls_vel(pathParams.steps), m_controls_steer(pathParams.steps), m_cost(pathParams.steps), m_goal_state(goal_state), m_state(init_state){}

        // m_nh.param<std::string>("vehicle_frame", m_vehicleframe, "cmu_rc1_base_link");
        // // m_nh.param<std::string>("map_frame", m_mapframe, "cmu_rc1_odom");
        // // rvizpub = m_nh.advertise<nav_msgs::Path>("/cmu_rc1/mppi/paths", 10);
        // rvizpub = m_nh.advertise<sensor_msgs::PointCloud2>("/cmu_rc1/mppi/paths", 10);
    //    
    // }

// state: x, y, theta, v
// control: v, steering angle 
void Path::state_update(Eigen::Vector4d &state, const double input_vel, const double input_ang)
{
    // std::cout << "Inside state_update" << std::endl;

    state(0) = input_vel*cos(state(2))*m_params.dt + state(0);
    state(1) = input_vel*sin(state(2))*m_params.dt + state(1);
    state(2) = input_vel*tan(input_ang)*m_params.dt/m_params.bike_length + state(2);
    state(3) = m_accel*m_params.dt + state(3);

}

double Path::calculate_cost(const Eigen::Vector4d state, const double input_vel, const double input_ang, mppi::Costmap m_costmap){

    Eigen::Vector2d control = Eigen::Vector2d(input_vel, input_ang);

    m_goal_state(3) = 5.0;

    // Checking obstacles from costmap
    if (m_costmap.vget(state(0), state(1)) == 100){

        // std::cout << "CANCELLING PATH" << std::endl;
        return 1e6;
    }
    
    // std::cout << "Inside calculate cost; path NOT cancelled" << std::endl;

    Eigen::Vector4d state_diff = state - m_goal_state;
    
    double state_cost = state_diff.transpose()*m_params.Q*state_diff;
    double control_cost = control.transpose()*m_params.R*control;

    // std::cout << "state cost:   \n" << state_cost << std::endl;
    // std::cout << "control cost: \n" << control_cost << std::endl;

    return state_cost/2 + control_cost/2;
}

void Path::forward_rollout(mppi::Costmap m_costmap, pcl::PointCloud<pcl::PointXYZI>::Ptr trajs)
{
    double mean_vel = 5.0;      // This will be the output of the mppi.control from the previous time step; the nominal input (probably)
    double mean_ang = 0.0;
    std::random_device rd;      // RNG for the sampling. Might wanna place this in the header file to keep it out of even the outer loop (number_rollouts)?
    std::mt19937 gen(rd());

    Eigen::Vector4d rollout_state = m_state;

    // std::cout << "initial state: \n" << m_state << std::endl;
    
    // To visualize the trajectories
    // traj->points.clear();
    // pathviz.header.frame_id = m_mapframe;
    // pathviz.header.stamp = ros::Time::now();
    // pose.header.frame_id = m_mapframe;
    // pose.header.stamp = ros::Time::now();

    // std::cout << "Inside forward rollout" << std::endl;

    for(int i = 0; i < m_params.steps; i++){
        // Sampling controls from a gaussian -- perturbed controls
        std::normal_distribution<double> vel_distribution(0., m_params.vel_standard_deviation);
        std::normal_distribution<double> ang_distribution(0., m_params.ang_standard_deviation);

        m_controls_vel(i) = mean_vel + vel_distribution(gen);
        m_controls_steer(i) = mean_ang + ang_distribution(gen);

        m_controls_vel(i) = std::clamp(m_controls_vel(i), -3., mean_vel);
        m_controls_steer(i) = std::clamp(m_controls_steer(i), -1 * M_PI/6, M_PI/6);

        // moved state update here

        rollout_state(2) += m_controls_vel(i)*tan(m_controls_steer(i))*m_params.dt/m_params.bike_length;
        rollout_state(0) += m_controls_vel(i)*cos(rollout_state(2))*m_params.dt;
        rollout_state(1) += m_controls_vel(i)*sin(rollout_state(2))*m_params.dt;
        rollout_state(3) = m_controls_vel(i);

        m_cost(i) = calculate_cost(rollout_state, m_controls_vel(i), m_controls_steer(i), m_costmap); // updated cost of step
        
        // To visualize the trajectories
        x = rollout_state(0);
        y = rollout_state(1);
        
        point.x = x;
        point.y = y;
        point.intensity = 1;
        trajs->points.push_back(point);
        
        // pose.pose.position.x = rollout_state(0);
        // pose.pose.position.y = rollout_state(1);
        // pathviz.poses.push_back(pose);

        // std::cout << "Pathviz pushed" << std::endl;
        
    }

    // To visualize the trajectories
    // std::cout << "TRAJ SIZE: " << trajs->points.size() << std::endl;
    // sensor_msgs::PointCloud2 output;
    // pcl::toROSMsg(*traj, output);

    // output.header.frame_id = m_vehicleframe;
    // output.header.stamp = ros::Time::now();

    // // std::cout << "OUTPUT SIZE: " << output.data.size() << std::endl;

    // rvizpub.publish(output);

    // rvizpub.publish(pathviz);
    // std::cout << "PUBLISHING" << std::endl;
}

}