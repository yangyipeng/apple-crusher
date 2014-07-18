#include "trajectory_library.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "TrajectoryLibary");
    ros::NodeHandle nh("~");

    TrajectoryLibrary tlib(nh);

    /* Define pick and place volumes */
    rect_grid pickVol;
    pickVol.orientation.w = 1;
    pickVol.orientation.x = 0;
    pickVol.orientation.y = 0;
    pickVol.orientation.z = 0;
    pickVol.xlim_low = 0.30;
    pickVol.xlim_high = 0.50;
    pickVol.xres = 5;
    pickVol.ylim_high = -0.25;
    pickVol.ylim_low = -0.60;
    pickVol.yres = 4;
    pickVol.zlim_low = 0.40;
    pickVol.zlim_high = 0.60;
    pickVol.zres = 3;

    rect_grid placeVol;
    placeVol.orientation.w = 0;
    placeVol.orientation.x = -1;
    placeVol.orientation.y = 0;
    placeVol.orientation.z = 0;
    placeVol.xlim_low = -0.2;
    placeVol.xlim_high = -0.3;
    placeVol.xres = 1;
    placeVol.ylim_low = 0.4;
    placeVol.ylim_high = 0.6;
    placeVol.yres = 3;
    placeVol.zlim_low = 0.3;
    placeVol.zlim_high = 0.50;
    placeVol.zres = 3;

    /* Generate target joint values */
    ROS_INFO("Calculating target joint values.");
    tlib.generateJvals(pickVol, placeVol);

    /* Generate trajectories */
    ROS_INFO("Building trajectory library.");
    tlib.build();

    ROS_INFO("Hit enter to begin demo.");
    std::cin.ignore();
    tlib.demo();

    ros::shutdown();
    return 0;
}
