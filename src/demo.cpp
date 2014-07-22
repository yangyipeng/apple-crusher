#include "trajectory_library.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "TrajectoryLibary");
    ros::NodeHandle nh("~");

    TrajectoryLibrary tlib(nh);
    tlib.initWorld();

    /* Read stored trajectories from file */
    // TODO: Make sure we can get number of pick and place locations

    ROS_INFO("Hit enter to begin demo.");
//    std::cin.ignore(100, '\n');
//    tlib.demo();

    ros::shutdown();
    return 0;
}
