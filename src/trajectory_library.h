#ifndef TRAJECTORY_LIBRARY_H
#define TRAJECTORY_LIBRARY_H

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <string.h>

#include <eigen_conversions/eigen_msg.h>
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
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

typedef struct {
    // We need both a moveit_msgs::RobotTrajectory and a valid RobotState
    moveit_msgs::RobotTrajectory trajectory;
    moveit_msgs::RobotState start_state;
    moveit_msgs::RobotState end_state;
    int pick_loc_index;
    int place_loc_index;
    double duration; // seconds
    int num_wpts;
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
    boost::shared_ptr<trajectory_processing::IterativeParabolicTimeParameterization> _time_parametizer;

    // Publisher
    ros::Publisher _trajectory_publisher;
    ros::Publisher _plan_scene_publisher;

    // Private methods
    std::size_t gridLinspace(std::vector<joint_values_t>& jvals, rect_grid& grid);
    bool planTrajectory(ur5_motion_plan& plan, std::vector<moveit_msgs::Constraints> constraints);
    void optimizeTrajectory(robot_trajectory::RobotTrajectoryPtr traj_opt, robot_trajectory::RobotTrajectoryPtr traj);
    void timeWarpTrajectory(robot_trajectory::RobotTrajectoryPtr traj, double slow_factor);

    void printPose(const geometry_msgs::Pose& pose);
    void printJointValues(const joint_values_t& jvals);

    moveit_msgs::Constraints genPoseConstraint(geometry_msgs::Pose pose_goal);
    moveit_msgs::Constraints genJointValueConstraint(joint_values_t jvals);

    bool ikValidityCallback(robot_state::RobotState* p_state, const robot_model::JointModelGroup* p_jmg, const double* jvals);

    //write and read

    bool filewrite(std::vector<ur5_motion_plan> &Library, const char* filename, bool debug);
    bool fileread(std::vector<ur5_motion_plan> &Library, const char* filename, bool debug);

public:
    TrajectoryLibrary(ros::NodeHandle nh);

    void initWorld();
    void generateJvals(rect_grid& pick_grid, rect_grid& place_grid);
    int build();
    void demo();

    bool getPickPlan(ur5_motion_plan& plan, int place_start, int pick_end);
    bool getPlacePlan(ur5_motion_plan& plan, int pick_start, int place_end);

    inline std::size_t getNumTrajectories() { return _num_trajects; }

};

#endif // TRAJECTORY_LIBRARY_H
