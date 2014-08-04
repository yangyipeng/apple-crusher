#include "kd_tree.h"

KDTree::KDTree(robot_model::RobotModelPtr& rmodel, const joint_values_t& low_bounds, const joint_values_t& high_bounds, const std::vector<std::size_t>& resolution)
{
    // Store pointer to robot model
    _rmodel = rmodel;

    // Extract dimension from model
    _dimension = _rmodel->getVariableCount();
    _bounds_low = low_bounds;
    _bounds_high = high_bounds;
    _resolution = resolution;

    return;
}
