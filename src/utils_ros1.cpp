#include "mppi/utils_ros1.hpp"

namespace mppi::ros1{

void getParam(ros::NodeHandle &node, const std::string &paramName,
     double &param) {
    if(!node.getParam(paramName,param)){
        std::cerr << "Error: Could not find " << paramName << std::endl;
        ros::requestShutdown();
    }
}

void getParam(ros::NodeHandle &node, const std::string &paramName,
     int &param) {
    if(!node.getParam(paramName,param)){
        std::cerr << "Error: Could not find " << paramName << std::endl;
        ros::requestShutdown();
    }
}

void getParam(ros::NodeHandle &node, const std::string &paramName,
    Eigen::Matrix4d &param) {
    std::vector<double> temp;
    if (!node.getParam(paramName, temp)) {
        std::cerr << "Error: Could not find " << paramName << std::endl;
        ros::requestShutdown();
    }

    int index = 0;
    for (int row = 0; row < 4; ++row){
        for (int col = 0; col < 4; ++col){
            param(row,col) = temp[index];
            ++index;
        }
    }
}

void getParam(ros::NodeHandle &node, const std::string &paramName,
    Eigen::Matrix2d &param) {
    std::vector<double> temp;
    if (!node.getParam(paramName, temp)) {
        std::cerr << "Error: Could not find " << paramName << std::endl;
        ros::requestShutdown();
    }

    int index = 0;
    for (int row = 0; row < 2; ++row){
        for (int col = 0; col < 2; ++col){
            param(row,col) = temp[index];
            ++index;
        }
    }
}

ROSParams getParams(ros::NodeHandle &node){
    ROSParams params;

    getParam(node, "horizon_steps", params.path_params.steps);
    getParam(node, "time_step", params.path_params.dt);
    getParam(node, "bike_length", params.path_params.bike_length);
    getParam(node, "Q", params.path_params.Q);
    getParam(node, "R", params.path_params.R);

    getParam(node, "rollout_number", params.mppi_params.number_rollouts);
    getParam(node, "lambda", params.mppi_params.lambda);

    getParam(node, "val_stdev", params.path_params.vel_standard_deviation);
    getParam(node, "ang_stdev", params.path_params.ang_standard_deviation);

    return params;
}

double roll, pitch, yaw;
bool odom_recieved = false;
nav_msgs::Odometry odom;
geometry_msgs::Quaternion geoQuat;

double roll, pitch, yaw;
bool odom_recieved = false;
nav_msgs::Odometry odom;
geometry_msgs::Quaternion geoQuat;

void odomMsgToState(const nav_msgs::Odometry::ConstPtr &odometry, Eigen::Vector4d &state){
     // state: x, y, theta, v

    odom = *odometry;
    odom_recieved = true;
    geoQuat = odom.pose.pose.orientation;
            tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
    
    state(0, 0) = odom.pose.pose.position.x;
    state(1, 0) = odom.pose.pose.position.y;
    state(2, 0) = yaw;
    state(3, 0) = odom.twist.twist.linear.x;
} 

void goalStateBuf(Eigen::Vector4d &state, Eigen::Vector4d &goal_state, Eigen::Vector4d m_goal_state_buf){
    ROSParams params;

    // this isnt right

    if (sqrt(pow(state(0, 0) - goal_state(0, 0), 2) + pow(state(1, 0) - goal_state(1, 0), 2)) < params.path_params.bike_length) {
        // if we're far from the goal state, keep pursuing goal
        m_goal_state_buf = goal_state;
    }
    else {
        // load odom to buffer
        m_goal_state_buf = state;
    }
}
}