#ifndef TRAJECTORY_LIBRARY_H
#define TRAJECTORY_LIBRARY_H

#include "kd_tree.h"

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <string.h>
#include <cmath>

#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>

#include "boost/scoped_ptr.hpp"
#include <iostream>

#define UR5_GROUP_NAME "manipulator"
#define DT_LOCAL_COLLISION_CHECK 0.05
#define MAX_IK_SOLUTIONS 1
#define MAX_PLANNER_ATTEMPTS 2

#define IK_COMP_MIN_DIST 3.0

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

typedef struct {
    rect_grid grid;
    bool allow_internal_paths;
} target_volume;

typedef struct {
    target_volume vol;
    std::vector<joint_values_t> jvals;
    int target_count;
} target_group;

enum target_groups {
    PICK_TARGET,
    PLACE_TARGET
};

class TrajectoryLibrary
{
    // Target positions
    int _num_target_groups;
    std::vector<target_group> _target_groups;

    // KD tree plan data structure
    KDTreePtr _kdtree;

    // MoveIt variables
    robot_model_loader::RobotModelLoaderPtr _rmodel_loader;
    robot_model::RobotModelPtr _rmodel;
    const robot_model::JointModelGroup* _jmg;
    collision_detection::AllowedCollisionMatrix _acm;
    planning_scene::PlanningScenePtr _plan_scene;
    planning_interface::PlannerManagerPtr _planner;
    planning_pipeline::PlanningPipelinePtr _planning_pipeline;
    boost::shared_ptr<trajectory_processing::IterativeParabolicTimeParameterization> _time_parametizer;
    trajectory_execution_manager::TrajectoryExecutionManagerPtr _execution_manager;

    // Publisher
    ros::Publisher _trajectory_publisher;
    ros::Publisher _plan_scene_publisher;
    ros::Publisher _robot_state_publisher;
    ros::Publisher _collision_object_publisher;

    // Gradient descent warp
    bool gradientDescentWarp(ur5_motion_plan& plan, const joint_values_t& jvals_start, const joint_values_t& jvals_end);
    void calculateGradients(double* gradient_array, robot_trajectory::RobotTrajectoryPtr traj);

    // Trajectory post-processing
    void optimizeTrajectory(robot_trajectory::RobotTrajectoryPtr traj_opt, robot_trajectory::RobotTrajectoryPtr traj);
    void timeWarpTrajectory(robot_trajectory::RobotTrajectoryPtr traj, double slow_factor);
    void computeVelocities(robot_trajectory::RobotTrajectoryPtr traj);

    // Private methods
    std::size_t gridLinspace(std::vector<joint_values_t>& jvals, rect_grid& grid);
    bool planTrajectory(ur5_motion_plan& plan, std::vector<moveit_msgs::Constraints> constraints);

    void printPose(const geometry_msgs::Pose& pose);
    void printJointValues(const joint_values_t& jvals);

    moveit_msgs::Constraints genPoseConstraint(geometry_msgs::Pose pose_goal);
    moveit_msgs::Constraints genJointValueConstraint(joint_values_t jvals);

    bool ikValidityCallback(const std::vector<joint_values_t>& comparison_values, robot_state::RobotState* p_state, const robot_model::JointModelGroup* p_jmg, const double* jvals);

    moveit_msgs::AttachedCollisionObject getAppleObjectMsg();

    // File write and read
    bool filewrite(const std::vector<ur5_motion_plan> &plans, const char* filename, bool debug);
    bool fileread(std::vector<ur5_motion_plan>& plans, const char* filename, bool debug);

public:
    TrajectoryLibrary(ros::NodeHandle& nh);

    void initWorld();
    void generateTargets(const std::vector<target_volume> & vols);
    void build();
    void demo();

    void fitPlan(ur5_motion_plan& plan, const joint_values_t &start_jvals, const joint_values_t &end_jvals);

    void exportToFile();
    void importFromFile();
};

#endif // TRAJECTORY_LIBRARY_H
