#include "trajectory_library.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Build Weeding Library");
    ros::NodeHandle nh("~");

    double BUSH_RADIUS;
    if (nh.hasParam("bush_radius"))
    {
        nh.getParam("bush_radius", BUSH_RADIUS);
    }

    TrajectoryLibrary tlib(nh);
    ROS_INFO("Initializing world.");
    tlib.initWorkspaceBounds();
    tlib.addSphereCollisionObject(BUSH_RADIUS);
    tlib.printCollisionWorldInfo(std::cout);

    ROS_INFO("Hit enter to begin building library.");
    std::cin.ignore(100, '\n');

    /* Generate trajectories */
    ROS_INFO("Building trajectory library.");
    tlib.build();

    tlib.exportToFile("plans_weeding.dat");

    ROS_INFO("Hit enter to begin demo.");
    std::cin.ignore(100, '\n');
    tlib.demo();

    ros::shutdown();
    return 0;
}
