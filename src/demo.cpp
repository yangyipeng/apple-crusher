#include "trajectory_library.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Demo Library");
    ros::NodeHandle nh("~");

    TrajectoryLibrary tlib(nh);
    tlib.initWorld();

    // TODO: Make sure we can get number of pick and place locations
    tlib.importFromFile(2);

    ROS_INFO("Hit enter to begin demo.");
    std::cin.ignore(100, '\n');
    tlib.demo();

    ros::shutdown();
    return 0;
}
