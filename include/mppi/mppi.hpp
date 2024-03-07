#pragma once

#include <Eigen/Dense>
#include <iostream>
#include "mppi/path.hpp"

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

    Eigen::Vector2d control(Eigen::Vector4d state, Eigen::Vector4d goal_state, const double acceleration);
};

}