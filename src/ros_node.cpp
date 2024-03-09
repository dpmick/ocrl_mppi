#include <ros/ros.h>
#include "mppi/utils_ros1.hpp"
#include "mppi/path.hpp"
#include "mppi/mppi.hpp"

int main(int argc, char *argv[]){
    ros::init(argc, argv, "asdf");
    ros::NodeHandle publicNode;
    ros::NodeHandle privateNode("~");

    const auto system_params = mppi::ros1::getParams(publicNode);
    mppi::MPPI mppi(system_params.path_params, system_params.mppi_params);

    ros::Subscriber odomSubscriber = publicNode.subscribe<nav_msgs::Odometry>(
        "odometry topic", 10,
        [&system_params, &mppi](const nav_msgs::Odometry::ConstPtr &odomMsg){
        
        Eigen::Vector4d current_state;
        mppi::ros1::odomMsgToState(odomMsg, current_state);
        
        Eigen::Vector4d goal_state;
        Eigen::Vector2d control = mppi.control(current_state, goal_state, 0.0);

    });

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}