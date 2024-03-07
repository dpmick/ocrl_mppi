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

    return params;
}

}