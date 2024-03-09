#include <ros/ros.h>
#include "mppi/utils_ros1.hpp"
#include "mppi/path.hpp"
#include "mppi/mppi.hpp"

int main(int argc, char *argv[]){
    ros::init(argc, argv, "asdf");
    ros::NodeHandle publicNode;
    ros::NodeHandle privateNode("~");

    const auto system_params = mppi::ros1::getParams(publicNode);
    mppi::MPPI mppi2(system_params.path_params, system_params.mppi_params);

}