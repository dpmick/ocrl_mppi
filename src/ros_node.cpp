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
        [&system_params, &mppi, &cmdVelPublisher](const nav_msgs::Odometry::ConstPtr &odomMsg){
        
        Eigen::Vector4d current_state;
        mppi::ros1::odomMsgToState(odomMsg, current_state);
        
        Eigen::Vector4d goal_state(10.0,10.0,0.1,3.0);
        Eigen::Vector2d control = mppi.control(current_state, goal_state, 0.0);

        geometry_msgs::TwistStamped cmdMsg;
        mppi::ros1::controlToMsg(control, cmdMsg);
        cmdMsg.header.stamp = ros::Time::now();
        cmdVelPublisher.publish(cmdMsg);

    });

    ros::Subscriber goalStateSubscriber = publicNode.subscribe<geometry_msgs::PoseArray>(
        "/cmu_rc1/command_interface/waypoint", 10,
        [&system_params, &mppi](const geometry_msgs::PoseArray::ConstPtr &goalMsg){
        
        Eigen::Vector4d goal_state;
        mppi::ros1::goalMsgToState(goalMsg, goal_state);

        mppi.registerGoalState(goal_state);

    });

    // ros::Subscriber targetvelSubscriber = publicNode.subscribe<geometry_msgs::Pose2D>(
    //     "goal_state", 10,
    //     [&system_params, &mppi](const geometry_msgs::Pose2D::ConstPtr &goalMsg){
        
    //     Eigen::Vector4d goal_state;
    //     mppi::ros1::goalMsgToState(goalMsg, goal_state);

    //     mppi.registerGoalState(goal_state);

    // });

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}