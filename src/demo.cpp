#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <iostream>

//void writePlanToFile(moveit::planning_interface::MoveGroup::Plan& plan)
//{
    //std::ofstream file;
    //file.open("plans", std::ios::binary | std::ios::app | std::ios::out);

    //file << plan;

    //file.close();
    //return;
//}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ArmPlanDemo");
    ros::NodeHandle nh("~");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    //std::ofstream file;
    //file.open("plans");

    moveit::planning_interface::MoveGroup group("manipulator");
    group.setPlannerId("RRTConnectkConfigDefault");

    ROS_INFO("Setting pose target.");
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.orientation.x = 1;
    target_pose1.orientation.y = 0;
    target_pose1.orientation.z = 0;
    target_pose1.position.x = -0.1;
    target_pose1.position.y = -0.4;
    target_pose1.position.z = 0.6;

    bool success = group.setJointValueTarget(group.getRandomJointValues());

    if (!success)
    {
        ROS_INFO("Invalid target.");
        ros::shutdown();
        return 0;
    }

    ROS_INFO("Planning motion.");
    moveit::planning_interface::MoveGroup::Plan my_plan;
    success = group.plan(my_plan);

    if (success)
    {
        ROS_INFO("Executing.");
        group.execute(my_plan);
    }

    ROS_INFO("Done.");

    //file.close();
    ros::shutdown();
    return 0;
}
