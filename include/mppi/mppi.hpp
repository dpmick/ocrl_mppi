#pragma once

#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <iostream>
#include "mppi/path.hpp"
#include <deque>
#include <math.h>    

namespace mppi {

struct mppiParams{
    int number_rollouts;
    double lambda;
};

class MPPI{
public:
    MPPI(const pathParams pathParams, const mppiParams mppiParams);

    pathParams m_pathParams;
    mppiParams m_mppiParams;

    std::deque<Eigen::Vector4d> m_current_state_buf {Eigen::Vector4d(0.0,0.0,0.0,0.0)};

    Eigen::Vector2d control(Eigen::Vector4d goal_state, const double acceleration);

    void registerCurrentState(Eigen::Vector4d current_state);
};

}