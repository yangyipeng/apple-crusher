#ifndef KD_TREE_H
#define KD_TREE_H

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>

typedef struct {
    // We need both a moveit_msgs::RobotTrajectory and a valid RobotState
    moveit_msgs::RobotTrajectory trajectory;
    moveit_msgs::RobotState start_state;
    moveit_msgs::RobotState end_state;
    int start_target_index;
    int end_target_index;
    double duration; // seconds
    int num_wpts;
} ur5_motion_plan;

typedef std::vector<double> joint_values_t;

class KDTree
{
    // Robot model
    robot_model::RobotModelPtr _rmodel;

    // KD parameters
    std::size_t _dimension;
    joint_values_t _bounds_low;
    joint_values_t _bounds_high;
    std::vector<std::size_t> _resolution;

    // Data
    std::vector<ur5_motion_plan> _data;
    std::map< std::vector<int>, std::vector<int> > _grid_mapping;

    // Calculated proximity ordering
    std::vector<std::size_t> _proximity_ordering;

public:
    KDTree(robot_model::RobotModelPtr& rmodel, const joint_values_t& low_bounds, const joint_values_t& high_bounds, const std::vector<std::size_t>& resolution);

    void add(ur5_motion_plan& plan);

    void generatePriorityQueue(joint_values_t start_jvals, joint_values_t end_jvals, int depth);
    ur5_motion_plan& lookup(int hit);
};

#endif // KD_TREE_H
