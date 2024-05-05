#include "mppi/mppi.hpp"
#include <math.h>
namespace mppi{

MPPI::MPPI(const pathParams pathParams, const mppiParams mppiParams):
    m_pathParams(pathParams), m_mppiParams(mppiParams), m_latest_u(Eigen::Vector2d::Zero()), m_trajs(new pcl::PointCloud<pcl::PointXYZI>()), m_selectedTraj(new pcl::PointCloud<pcl::PointXYZI>()){
    
}   
    
Eigen::Vector2d MPPI::control(Eigen::Vector4d curr_state, const double m_target_speed, const double acceleration = 0.0){

    Eigen::Vector2d u;

    goal_statedef = m_goal_state_buf.front();

    // std::cout << "m_goal_state_buf: " << m_goal_state_buf.size() << std::endl;

    // Checking distance to waypoint in order to pop it
    double dist = sqrt(pow(goal_statedef(0) - curr_state(0), 2) + pow(goal_statedef(1) - curr_state(1), 2));
    if (dist < 2.0){
        m_goal_state_buf.pop_front();
        // std::cout << "Waypoint POPPED" << std::endl;
    }

    if (m_goal_state_buf.size() < 1){
        u = Eigen::Vector2d(0.0,0.0);
        // std::cout << "LAST Waypoint" << std::endl;
        return u;

    } else{

        Eigen::MatrixXd du(2, m_pathParams.steps);
        Eigen::MatrixXd traj_weighted_combo(2, m_mppiParams.number_rollouts);
        Eigen::MatrixXd d_vel(m_mppiParams.number_rollouts, m_pathParams.steps);
        Eigen::MatrixXd d_steer(m_mppiParams.number_rollouts, m_pathParams.steps);

        Eigen::MatrixXd all_costs(m_mppiParams.number_rollouts, m_pathParams.steps);

        // eq. 34 on mppi paper
        m_trajs->points.clear();
        m_selectedTraj -> points.clear();

        du.setZero(); // clearing controls

        // storing relevant values from every rollout

        for(int k = 0; k < m_mppiParams.number_rollouts; k++){
            mppi::Path newPath(m_pathParams, goal_statedef, curr_state, acceleration, m_target_speed, m_latest_u);
            newPath.forward_rollout(m_costmap, m_trajs, m_latest_u);

            // row-wise rollouts, columnwise steps
            d_vel.row(k) = newPath.m_controls_vel;
            d_steer.row(k) = newPath.m_controls_ang;
            all_costs.row(k) = newPath.m_cost;
        }

        double min_cost = 0;
        Eigen::VectorXd weighted_cost(m_pathParams.steps);

        // weighted update rule (eq 34)

        for (int i = 0; i < m_pathParams.steps; i++){

            min_cost = all_costs.col(i).minCoeff(); // minimum cost incurred at this step
            all_costs.array().col(i) -= min_cost;

            weighted_cost = exp((-1.0/m_mppiParams.lambda) * all_costs.array().col(i)) + 1e-6; // exploration/exploitation of every step in traj

            weighted_cost = weighted_cost.array() / weighted_cost.sum();  // normalization

            du(0, i) += weighted_cost.dot(d_vel.col(i));
            du(1, i) += weighted_cost.dot(d_steer.col(i));
        }

        Eigen::Vector4d generatedPath;

        mppi::Path genPath(m_pathParams, goal_statedef, curr_state, acceleration, m_latest_u);

        // initializing position to start selected path projection 
        for (int i = 0; i < 4; i++){
            generatedPath(i) = curr_state(i);
        }

        // rolling out path for visualizer
        for (int i = 1; i < m_pathParams.steps; i++){
            genPath.state_update(generatedPath, du(0, i), du(1, i));

            selectedPoint.x = generatedPath(0);
            selectedPoint.y = generatedPath(1);
            
            m_selectedTraj->points.push_back(selectedPoint);
        }

        // To visualize the trajectories        
        u = du.col(0);
        m_latest_u = u;

        return u;         
    }
}

void MPPI::registerGoalState(std::deque<Eigen::Vector4d> goal_array){
    m_goal_state_buf = goal_array;
}
}