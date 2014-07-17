#ifndef TRAJECTORY_LIBRARY_H
#define TRAJECTORY_LIBRARY_H

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>

#include "boost/scoped_ptr.hpp"
#include <iostream>

#define UR5_GROUP_NAME "manipulator"

typedef struct {
    // We need both a moveit_msgs::RobotTrajectory and a valid RobotState
    moveit_msgs::RobotTrajectory trajectory;
    moveit_msgs::RobotState start_state;
    moveit_msgs::RobotState end_state;
    unsigned int pick_loc_index;
    unsigned int place_loc_index;
} ur5_motion_plan;

typedef std::vector<double> joint_values_t;

typedef struct {
    double xlim_low;
    double xlim_high;
    int xres;
    double ylim_low;
    double ylim_high;
    int yres;
    double zlim_low;
    double zlim_high;
    int zres;

    // Assume single orientation for all positions
    geometry_msgs::Quaternion orientation;
} rect_grid;

class TrajectoryLibrary
{
    // Trajectories
    std::vector<ur5_motion_plan> _pick_trajects;
    std::vector<ur5_motion_plan> _place_trajects;
    std::size_t _num_trajects;

    // Pick and place target volumes
    rect_grid _pick_grid;
    rect_grid _place_grid;

    // Pick and place target joint values (from IK)
    std::vector<joint_values_t> _pick_jvals;
    std::vector<joint_values_t> _place_jvals;
    std::size_t _num_pick_targets;
    std::size_t _num_place_targets;

    // MoveIt variables
    robot_model_loader::RobotModelLoaderPtr _rmodel_loader;
    robot_model::RobotModelPtr _rmodel;
    const robot_model::JointModelGroup* _jmg;
    planning_scene::PlanningScenePtr _plan_scene;
    planning_interface::PlannerManagerPtr _planner;

    // Publisher
    ros::Publisher _pub;

    kinematics::KinematicsBasePtr _ik_solver;

    // Private methods
    std::size_t gridLinspace(std::vector<joint_values_t>& jvals, rect_grid& grid);
    bool planTrajectory(ur5_motion_plan& plan, std::vector<moveit_msgs::Constraints> constraints);

    void printPose(const geometry_msgs::Pose& pose);
    void printJointValues(const joint_values_t& jvals);

    moveit_msgs::Constraints genPoseConstraint(geometry_msgs::Pose pose_goal);
    moveit_msgs::Constraints genJointValueConstraint(joint_values_t jvals);

    bool ikValidityCallback(robot_state::RobotState* p_state, const robot_model::JointModelGroup* p_jmg, const double* jvals);

public:
    TrajectoryLibrary(ros::NodeHandle nh);

    void generateJvals(rect_grid& pick_grid, rect_grid& place_grid);
    int buildLibrary();

    inline std::size_t getNumTrajectories() { return _num_trajects; }

};

#endif // TRAJECTORY_LIBRARY_H
