#include "trajectory_library.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Build Library");
    ros::NodeHandle nh("~");

    TrajectoryLibrary tlib(nh);
    ROS_INFO("Initializing world.");
    tlib.initWorld();

    ROS_INFO("Hit enter to generate target joint values.");
    std::cin.ignore(100, '\n');

    /* Define pick and place volumes */
    rect_grid pickVol;
    pickVol.orientation.w = 1;
    pickVol.orientation.x = 0;
    pickVol.orientation.y = 0;
    pickVol.orientation.z = 0;
    pickVol.xlim_low = -0.25;
    pickVol.xlim_high = 0.25;
    pickVol.xres = 3;
    pickVol.ylim_high = -0.25;
    pickVol.ylim_low = 0.25;
    pickVol.yres = 3;
    pickVol.zlim_low = 0.70;
    pickVol.zlim_high = 0.80;
    pickVol.zres = 2;

    rect_grid placeVol;
    placeVol.orientation.w = 0;
    placeVol.orientation.x = -1;
    placeVol.orientation.y = 0;
    placeVol.orientation.z = 0;
    placeVol.xlim_low = -.10;
    placeVol.xlim_high = 0.10;
    placeVol.xres = 2;
    placeVol.ylim_low = -.10;
    placeVol.ylim_high = 0.10;
    placeVol.yres = 2;
    placeVol.zlim_low = 0.25;
    placeVol.zlim_high = 0.35;
    placeVol.zres = 2;

    std::vector<rect_grid> grids;
    grids.push_back(pickVol);
    grids.push_back(placeVol);

    /* Generate target joint values */
    ROS_INFO("Calculating target joint values.");
    tlib.generateTargets(grids);
    ROS_INFO("Hit enter to begin building library.");
    std::cin.ignore(100, '\n');

    /* Generate trajectories */
    ROS_INFO("Building trajectory library.");
    tlib.build();

    tlib.exportToFile();

    ROS_INFO("Hit enter to begin demo.");
    std::cin.ignore(100, '\n');
    tlib.demo();

    ros::shutdown();
    return 0;
}
