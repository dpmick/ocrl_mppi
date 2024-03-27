#include <ros/ros.h>
#include "mppi/utils_ros1.hpp"
#include "mppi/path.hpp"
#include "mppi/mppi.hpp"
#include <deque>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "asdf");
    ros::NodeHandle publicNode;
    ros::NodeHandle privateNode("~");

    const auto system_params = mppi::ros1::getParams(publicNode);
    mppi::MPPI mppi(system_params.path_params, system_params.mppi_params);

    ros::Publisher cmdVelPublisher = publicNode.advertise<geometry_msgs::TwistStamped>("cmu_rc1/low_level_control/cmd_vel", 5);

    ros::Subscriber odomSubscriber = publicNode.subscribe<nav_msgs::Odometry>(
        "cmu_rc1/odom_to_base_link", 10,
        [&system_params, &mppi, &cmdVelPublisher, &steeringPublisher](const nav_msgs::Odometry::ConstPtr &odomMsg){
        Eigen::Vector4d current_state;
        mppi::ros1::odomMsgToState(odomMsg, current_state);
        Eigen::Vector4d goal_state; // here's the two issues

        // confirmed that goal and current state are compatible types
        
        Eigen::Vector2d control = mppi.control(current_state, goal_state, 0.0); // confirmed 4d
        
        std::cout << "current_state" << current_state << std::endl;
        
        std_msgs::Float32 cmdVel_msg;
        cmdVel_msg.data = control(0);
        cmdVelPublisher.publish(cmdVel_msg);
        std_msgs::Float32 steering_msg;
        steering_msg.data = control(1);
        steeringPublisher.publish(steering_msg);
    });

    ros::Subscriber goalStateSubscriber = publicNode.subscribe<nav_msgs::Odometry>(
        "command_interface/waypoint", 10,
        [&system_params, &mppi](const nav_msgs::Odometry::ConstPtr &goalMsg){
        
        Eigen::Vector4d goal_state;
        mppi::ros1::goalMsgToState(goalMsg, goal_state);

        mppi.registerGoalState(goal_state);

    });

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}