// plans.h

#ifndef PLANS_H
#define PLANS_H

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include "boost/scoped_ptr.hpp"
#include <iostream>

#define UR5_NUM_JOINTS 6

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
} rect_grid;

int gridLinspace( std::vector<joint_values_t>& target_joint_vals, rect_grid& rg, geometry_msgs::Quaternion& orientation, robot_model::RobotModelPtr robot_model, planning_scene::PlanningSceneConstPtr plan_scene);
void printPose(const geometry_msgs::Pose& pose);
void printJointValues(const joint_values_t& jvals);
moveit_msgs::Constraints genJointValueConstraint(joint_values_t jvals, robot_model::RobotModelPtr robot_model);
void writePlansToFile(std::string filename);

#endif

