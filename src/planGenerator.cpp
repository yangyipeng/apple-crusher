#include "plans.h"

#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit/planning_scene/planning_scene.h"
#include "moveit/planning_interface/planning_interface.h"
#include "moveit/kinematic_constraints/kinematic_constraints.h"
#include "moveit_msgs/MotionPlanRequest.h"
#include "moveit_msgs/MotionPlanResponse.h"
#include "moveit_msgs/DisplayTrajectory.h"

#include "boost/scoped_ptr.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "OfflinePlanGenerator");
    ros::NodeHandle nh("~");

    // Load up robot model
    robot_model_loader::RobotModelLoader robot_model_loader("ur5");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    // Init planning scene
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    // Load the planner
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance("RRTConnectkConfigDefault"));
    planner_instance->initialize(robot_model, node_handle.getNamespace();

    /* Sleep a little to allow time to startup rviz, etc. */
    ros::WallDuration sleep_time(5.0);
    sleep_time.sleep();

    // Pose goal
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0.75;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    // Position and orientation tolerances
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    // Create constraint from pose using IK
    req.group_name = "right_arm";
    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("r_wrist_roll_link", pose, tolerance_pose, tolerance_angle);
    req.goal_constraints.push_back(pose_goal);
    
    // Now prepare the planning context
    planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    if(res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_INFO("Could not compute plan successfully");
      return 0;
    }

    // Create publisher for rviz
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    /* Visualize the trajectory */
    ROS_INFO("Visualizing the trajectory");
    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    display_publisher.publish(display_trajectory);

    sleep_time.sleep();

    ros::shutdown();
    return 0;
    
}
