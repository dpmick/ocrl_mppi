#include <ros/ros.h>
#include "mppi/utils_ros1.hpp"
#include "mppi/path.hpp"
#include "mppi/mppi.hpp"
#include "mppi/costmap.hpp"
#include <deque>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "asdf");
    ros::NodeHandle publicNode;
    ros::NodeHandle privateNode("~");

    const auto system_params = mppi::ros1::getParams(publicNode);
    mppi::MPPI mppi(system_params.path_params, system_params.mppi_params);

    ros::Publisher cmdVelPublisher = publicNode.advertise<geometry_msgs::TwistStamped>("cmu_rc1/low_level_control/cmd_vel", 5);

    ros::Subscriber velSubscriber = publicNode.subscribe<std_msgs::Float32>(
        "/cmu_rc1/command_interface/target_speed", 1,
        [&system_params, &mppi](const std_msgs::Float32::ConstPtr &target_speed){

            mppi.m_target_speed = target_speed->data;

        });

    ros::Subscriber odomSubscriber = publicNode.subscribe<nav_msgs::Odometry>(
        "cmu_rc1/odom_to_base_link", 10,
        [&system_params, &mppi, &cmdVelPublisher](const nav_msgs::Odometry::ConstPtr &odomMsg){
        
        Eigen::Vector4d current_state;
        mppi::ros1::odomMsgToState(odomMsg, current_state);
        
        // Eigen::Vector4d goal_state(10.0,10.0,0.1,3.0);
        Eigen::Vector2d control = mppi.control(current_state, mppi.m_target_speed, 0.0);

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

    ros::Subscriber costmapSubscriber = publicNode.subscribe<nav_msgs::OccupancyGrid>(
        "/cmu_rc1//local_mapping_lidar_node/voxel_grid/obstacle_map", 10,
        [&system_params, &mppi](const nav_msgs::OccupancyGrid::ConstPtr &occMsg){

        mppi::Costmap costmap;
        mppi::ros1::occMsgtoMap(occMsg, costmap);
        mppi.m_costmap = costmap; //costmap in mppi class
    });

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}