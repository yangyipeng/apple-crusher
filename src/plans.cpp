#include "plans.h"

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
#include <fstream>

#define STUB ROS_INFO("Line %d", __LINE__);

#define UR5_GROUP_NAME "manipulator"

std::vector<joint_values_t> pick_pose_joint_vals;
std::vector<joint_values_t> place_pose_joint_vals;


std::vector<ur5_motion_plan> pickTrajLibrary;
std::vector<ur5_motion_plan> placeTrajLibrary;

int gridLinspace( std::vector<joint_values_t>& target_joint_vals, rect_grid& rg, geometry_msgs::Quaternion& orientation, robot_model::RobotModelPtr robot_model, planning_scene::PlanningSceneConstPtr plan_scene)
{
    double di, dj, dk;
    di = dj = dk = 0.0;
    int num_poses;

    // Calculate grid spacings if xres != 1
    if (rg.xres != 1) {
        di = (rg.xlim_high - rg.xlim_low)/(rg.xres-1);
    }
    if (rg.yres != 1) {
        dj = (rg.ylim_high - rg.ylim_low)/(rg.yres-1);
    }
    if (rg.zres != 1) {
        dk = (rg.zlim_high - rg.zlim_low)/(rg.zres-1);
    }

    // Allocate
    num_poses = rg.xres * rg.yres * rg.zres;
    target_joint_vals.reserve(num_poses);

    geometry_msgs::Pose geo_pose;
    robot_state::RobotState state(robot_model);
    const robot_state::JointModelGroup* jmg = state.getJointModelGroup(UR5_GROUP_NAME);

    // Linspace
    int n = -1;
    for (int i = 0; i < rg.xres; i++)
    {
        for (int j = 0; j < rg.yres; j++)
        {
            for (int k = 0; k < rg.zres; k++)
            {
                n++;
                geo_pose.position.x = rg.xlim_low + i*di;
                geo_pose.position.y = rg.ylim_low + j*dj;
                geo_pose.position.z = rg.zlim_low + k*dk;
                geo_pose.orientation = orientation;
                bool ik_success;
                // Try fixed number of times to find valid, non-colliding solution
                int tries;
                for (tries = 0; tries < 3; tries++)
                {
                    // Do IK
                    ik_success = state.setFromIK(jmg, geo_pose, 5, 0.1);
                    if (!ik_success)
                    {
                        ROS_WARN("Could not solve IK for pose %d: Skipping.", n);
                        printPose(geo_pose);
                        break;
                    }
                    // If IK succeeded
                    joint_values_t jvals;
                    state.copyJointGroupPositions(UR5_GROUP_NAME, jvals);
                    // Now check for self-collisions
                    if (!plan_scene->isStateColliding(state, UR5_GROUP_NAME))
                    {
                        // Add to vector
                        ROS_INFO("Successfully generated joint values for pose %d.", n);
                        target_joint_vals.push_back(jvals);
                        break;
                    }
                }
                if (tries == 3)
                {
                    ROS_ERROR("Could not find non-collision joint state for pose %d: Skipping.", n);
                    printPose(geo_pose);
                }
            }
        }
    }
    return (int) target_joint_vals.size();
}

void printPose(const geometry_msgs::Pose& pose)
{
    printf("     Position (%f, %f, %f).\n", pose.position.x, pose.position.y, pose.position.z);
    printf("     Orientation (%f, %f, %f, %f).\n", pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    return;
}

void printJointValues(const joint_values_t& jvals)
{
    printf("Joint values: ");
    for (int i=0; i<jvals.size(); i++)
    {
        printf(" %f", jvals[i]);
    }
    printf("\n");
    return;
}

void initTargetJointVals(robot_model::RobotModelPtr robot_model, planning_scene::PlanningSceneConstPtr plan_scene)
{
    // Define pick volume as a 3D rectangular grid
    rect_grid pickVol;
    geometry_msgs::Quaternion pickOrient;
    pickOrient.w = 1;
    pickOrient.x = 0;
    pickOrient.y = 0;
    pickOrient.z = 0;
    pickVol.xlim_low = 0.30;
    pickVol.xlim_high = 0.50;
    pickVol.xres = 5;
    pickVol.ylim_high = -0.25;
    pickVol.ylim_low = -0.60;
    pickVol.yres = 4;
    pickVol.zlim_low = 0.40;
    pickVol.zlim_high = 0.60;
    pickVol.zres = 3;

    // Generate pickPoses array
    gridLinspace(pick_pose_joint_vals, pickVol, pickOrient, robot_model, plan_scene);
    ROS_INFO("Generated %d pick poses.", (int) pick_pose_joint_vals.size());

    // Define place volumes
    rect_grid placeVol;
    geometry_msgs::Quaternion placeOrient;
    placeOrient.w = 0;
    placeOrient.x = -1;
    placeOrient.y = 0;
    placeOrient.z = 0;
    placeVol.xlim_low = -0.2;
    placeVol.xlim_high = -0.3;
    placeVol.xres = 1;
    placeVol.ylim_low = 0.4;
    placeVol.ylim_high = 0.6;
    placeVol.yres = 3;
    placeVol.zlim_low = 0.3;
    placeVol.zlim_high = 0.50;
    placeVol.zres = 3;

    // Generate placePoses array
    gridLinspace(place_pose_joint_vals, placeVol, placeOrient, robot_model, plan_scene);
    ROS_INFO("Generated %d place poses.", (int) place_pose_joint_vals.size());

    return;
}

void getMsgFromRobotState(moveit_msgs::RobotState& state_msg, const robot_state::RobotState* robot_state)
{
    // First copy joint state
    sensor_msgs::JointState joint_state;
    size_t num_vars = robot_state->getVariableCount();
    joint_state.name = robot_state->getVariableNames();
    const double* pos = robot_state->getVariablePositions();
    const double* vel = robot_state->getVariableVelocities();
    const double* effort = robot_state->getVariableEffort();

    for (int i=0; i<num_vars; i++)
    {
        joint_state.position.push_back(pos[i]);
        joint_state.velocity.push_back(vel[i]);
        joint_state.effort.push_back(effort[i]);
    }

    state_msg.joint_state = joint_state;

    // No Multi-DOF joints

    // TODO: Attached objects

    // Is diff?
    state_msg.is_diff = false;

    return;
}

bool genTraj(ur5_motion_plan& plan, planning_scene::PlanningScenePtr plan_scene, planning_interface::PlannerManagerPtr planner_instance, std::vector<moveit_msgs::Constraints> constraints)
{
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    req.group_name = UR5_GROUP_NAME;

    // Add constraints
    req.goal_constraints = constraints;
    req.planner_id = "manipulator[RRTConnectkConfigDefault]";
    req.allowed_planning_time = 5.0;

    // Define workspace
    req.workspace_parameters.max_corner.x = 1.0;
    req.workspace_parameters.max_corner.y = 1.0;
    req.workspace_parameters.max_corner.z = 0.7;
    req.workspace_parameters.min_corner.x = -1.0;
    req.workspace_parameters.min_corner.y = -1.0;
    req.workspace_parameters.min_corner.z = 0.05;

    // Now prepare the planning context
    int tries = 0;
    while (tries < 3)
    {
        planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(plan_scene, req, res.error_code_);
        context->solve(res);
        if (res.error_code_.val == res.error_code_.SUCCESS)
        {
            moveit_msgs::MotionPlanResponse response;
            res.getMessage(response);

            getMsgFromRobotState(plan.start_state, &(plan_scene->getCurrentState()));
            getMsgFromRobotState(plan.end_state, &(res.trajectory_->getLastWayPoint()));
            plan.trajectory = response.trajectory;
            return true;
        }
        // else planner failed
        tries++;
    }
    return false;
}

moveit_msgs::Constraints genPoseConstraint(geometry_msgs::Pose pose_goal)
{
    geometry_msgs::PoseStamped pose_pkt;
    pose_pkt.header.frame_id = "world";
    pose_pkt.pose = pose_goal;
    // Position and orientation tolerances
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    // Create constraint from pose using IK
    return kinematic_constraints::constructGoalConstraints("ee_link", pose_pkt, tolerance_pose, tolerance_angle);
}

moveit_msgs::Constraints genJointValueConstraint(joint_values_t jvals, robot_model::RobotModelPtr robot_model)
{
    // Initialize state variable with joint values
    robot_state::RobotState state(robot_model);
    state.setJointGroupPositions(UR5_GROUP_NAME, jvals);
    // Get joint model group
    const robot_state::JointModelGroup* joint_model_group = state.getJointModelGroup(UR5_GROUP_NAME);
    if (!joint_model_group->satisfiesPositionBounds( jvals.data() ) )
    {
        ROS_ERROR("Joint value constraint does not satisfy position bounds.");
    }
    // Create constraint
    return kinematic_constraints::constructGoalConstraints(state, joint_model_group, 0.01);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "OfflinePlanGenerator");
    ros::NodeHandle nh("~");

    // Load up robot model
    ROS_INFO("Loading ur5 robot model.");
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    if (robot_model->getName() == "")
    {
        ROS_ERROR("RobotModel has not been initialized successfully.");
    }

    ROS_INFO("Loading kinematics plugin.");
    const kinematics_plugin_loader::KinematicsPluginLoaderPtr kine_plugin_loader = robot_model_loader.getKinematicsPluginLoader();

    ROS_INFO("Grabbing JointModelGroup.");
    robot_model::JointModelGroup* jmg = robot_model->getJointModelGroup(UR5_GROUP_NAME);

    ROS_INFO("Initializing kinematics solver.");
    kinematics::KinematicsBasePtr ik_solver = (kine_plugin_loader->getLoaderFunction())(    jmg);
    bool success = ik_solver->initialize("robot_description", UR5_GROUP_NAME, "world", "ee_link", 0.1);
    ROS_INFO("Initialize successful? : %d", (int) success);
    ROS_INFO("Default timeout = %f ", ik_solver->getDefaultTimeout());
    ROS_INFO("Get tip frame = %s ", ik_solver->getTipFrame().c_str());

    // Init planning scene
    ROS_INFO("Initializing PlanningScene from RobotModel");
    planning_scene::PlanningScenePtr plan_scene(new planning_scene::PlanningScene(robot_model));

    // Load the planner
    ROS_INFO("Loading the planner plugin.");
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    try
    {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
    }
    catch(pluginlib::PluginlibException& ex)
    {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }

    try
    {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance("ompl_interface/OMPLPlanner"));
        planner_instance->initialize(robot_model, nh.getNamespace());
    }
    catch(pluginlib::PluginlibException& ex)
    {
        const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (std::size_t i = 0 ; i < classes.size() ; ++i)
        ss << classes[i] << " ";
        ROS_ERROR_STREAM("Exception while loading planner: " << ex.what() << std::endl
                       << "Available plugins: " << ss.str());
    }

    // Create publisher for rviz
    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    // Initialize pickPoses
    ROS_INFO("Generating target joint values");
    initTargetJointVals(robot_model, plan_scene);


    // Now generate plans
    for (int n = 0; n < place_pose_joint_vals.size(); n++)
    {
        sleep(5);
        // Solve IK for place position n
        ROS_INFO("Jumping to place pose %d", n);
        robot_state::RobotState place_state(robot_model);
        place_state.setJointGroupPositions(UR5_GROUP_NAME, place_pose_joint_vals[n]);
        // Jump current state to place pose
        plan_scene->setCurrentState(place_state);

        for (int m = 0; m < pick_pose_joint_vals.size(); m++) {

            ////////////////// Pick target

            // Assume we are at a place pose
            // Set a pick target
            std::vector<moveit_msgs::Constraints> v_constraints;
            v_constraints.push_back(genJointValueConstraint(pick_pose_joint_vals[m], robot_model));
            // Generate trajectory
            ur5_motion_plan pick_traj;
            bool success = genTraj(pick_traj, plan_scene, planner_instance, v_constraints);
            if (!success)
            {
                ROS_ERROR("Planner failed to generate plan for pick target %d. Skipping.", m);
                continue;
            }

            // Now change state to our pick target pose
            plan_scene->setCurrentState(pick_traj.end_state);
            ROS_INFO("Successfully planned pick trajectory.");

            // TODO: Attach object (ie apple) for return trajectory



            //////////////////// Place target

            // Assume we are at a pick pose
            // Set a place target (in joint space)
            v_constraints.clear();
            v_constraints.push_back(genJointValueConstraint(place_pose_joint_vals[n], robot_model));;
            // Generate trajectory
            ur5_motion_plan place_traj;
            success = genTraj(place_traj, plan_scene, planner_instance, v_constraints);
            if (!success)
            {
                ROS_ERROR("Planner failed to generate plan for place target %d. Skipping.", m);
                continue;
            }
            // Add trajectory to display message and publish to Rviz
            moveit_msgs::DisplayTrajectory display_trajectory;
            display_trajectory.trajectory_start = pick_traj.start_state;
            display_trajectory.trajectory.push_back(pick_traj.trajectory);
            display_trajectory.trajectory.push_back(place_traj.trajectory);
            display_publisher.publish(display_trajectory);

            // Store trajectories in library
            pickTrajLibrary.push_back(pick_traj);
            placeTrajLibrary.push_back(place_traj);

            // Now change state back to our place target pose
            plan_scene->setCurrentState(place_state);

            // TODO: Detach object (apple) for next pick trajectory

            ROS_INFO("Successfully planned place trajectory.");
            ROS_INFO("Trajectory set %d saved.", m);

            /* Sleep a little to allow time for rviz to display path */
            ros::WallDuration sleep_time(1.0);
            //sleep_time.sleep();

        }
    }

    ROS_INFO("%d place trajectories created, %d pick trajectories created.", (int) pickTrajLibrary.size(), (int) placeTrajLibrary.size());
    // When we are done write them to file

    writePlansToFile(".plans");
    ros::shutdown();
    return 0;
}

void writePlansToFile(std::string filename)
{
//    std::string pick_filename = "pick" + filename;
//    std::string place_filename = "place" + filename;
//
//    std::ofstream pick_file(pick_filename.c_str(), std::ios::out | std::ios::binary | std::ios::ate);
//    pick_file << pickTrajLibrary;
//    pick_file.close();
//
//    std::ofstream place_file(place_filename.c_str(), std::ios::out | std::ios::binary | std::ios::ate);
//    place_file << placeTrajLibrary;
//    place_file.close();
}
