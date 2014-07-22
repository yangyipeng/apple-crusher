#include "trajectory_library.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "TrajectoryLibary");
    ros::NodeHandle nh("~");

    TrajectoryLibrary tlib(nh);
    tlib.initWorld();

    /* Define pick and place volumes */
    rect_grid pickVol;
    pickVol.orientation.w = 1;
    pickVol.orientation.x = 0;
    pickVol.orientation.y = 0;
    pickVol.orientation.z = 0;
    pickVol.xlim_low = -0.60;
    pickVol.xlim_high = 0.60;
    pickVol.xres = 5;
    pickVol.ylim_high = -0.40;
    pickVol.ylim_low = 0.40;
    pickVol.yres = 4;
    pickVol.zlim_low = 0.30;
    pickVol.zlim_high = 0.35;
    pickVol.zres = 1;

    rect_grid placeVol;
    placeVol.orientation.w = 0;
    placeVol.orientation.x = -1;
    placeVol.orientation.y = 0;
    placeVol.orientation.z = 0;
    placeVol.xlim_low = -0.3;
    placeVol.xlim_high = -0.25;
    placeVol.xres = 1;
    placeVol.ylim_low = 0.7;
    placeVol.ylim_high = 0.7;
    placeVol.yres = 1;
    placeVol.zlim_low = 0.35;
    placeVol.zlim_high = 0.50;
    placeVol.zres = 1;

    /* Generate target joint values */
    ROS_INFO("Calculating target joint values.");
    tlib.generateJvals(pickVol, placeVol);
    ROS_INFO("Hit enter to begin building library.");
    std::cin.ignore(100, '\n');

    /* Generate trajectories */
    ROS_INFO("Building trajectory library.");
    tlib.build();

    ROS_INFO("Hit enter to begin demo.");
    std::cin.ignore(100, '\n');
    tlib.demo();

    ros::shutdown();
    return 0;
}
