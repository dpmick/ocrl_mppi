#include <ros/ros.h>
#include "mppi/utils_ros1.hpp"
#include "mppi/path.hpp"
#include "mppi/mppi.hpp"
#include <deque>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "asdf");
    ros::NodeHandle publicNode;
    ros::NodeHandle privateNode("~");

    // Params
    publicNode.setParam("horizon_steps", 300);
    publicNode.setParam("time_step", 0.1);
    publicNode.setParam("bike_length", 0.48);
    publicNode.setParam("Q", std::vector<double>{100, 0, 0, 0, 0, 150, 0, 0, 0, 0, 100, 0, 0, 0, 0, 30});
    publicNode.setParam("R", std::vector<double>{50, 0, 0, 10});
    publicNode.setParam("rollout_number", 100);
    publicNode.setParam("lambda", 1.0);

    const auto system_params = mppi::ros1::getParams(publicNode);
    mppi::MPPI mppi(system_params.path_params, system_params.mppi_params);

    ros::Publisher cmdVelPublisher = publicNode.advertise<geometry_msgs::TwistStamped>("cmu_rc1/low_level_control/cmd_vel", 5);

    ros::Subscriber odomSubscriber = publicNode.subscribe<nav_msgs::Odometry>(
        "odom_to_base_link", 10,
        [&system_params, &mppi, &cmdVelPublisher, &steeringPublisher](const nav_msgs::Odometry::ConstPtr &odomMsg){
        
        Eigen::Vector4d current_state;
        mppi::ros1::odomMsgToState(odomMsg, current_state);
        
        Eigen::Vector4d goal_state(10.0,10.0,0.1,3.0);
        Eigen::Vector2d control = mppi.control(current_state, goal_state, 0.0);

        geometry_msgs::TwistStamped cmdMsg;
        mppi::ros1::controlToMsg(control, cmdMsg);
        cmdMsg.header.stamp = ros::Time::now();
        cmdVelPublisher.publish(cmdMsg);

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