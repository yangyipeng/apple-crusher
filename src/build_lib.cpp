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
    target_volume pickVol;
    pickVol.grid.orientation.w = 1;
    pickVol.grid.orientation.x = 0;
    pickVol.grid.orientation.y = 0;
    pickVol.grid.orientation.z = 0;
    pickVol.grid.xlim_low = -0.35;
    pickVol.grid.xlim_high = 0.35;
    pickVol.grid.xres = 7;
    pickVol.grid.ylim_high = -0.35;
    pickVol.grid.ylim_low = 0.35;
    pickVol.grid.yres = 7;
    pickVol.grid.zlim_low = 0.80;
    pickVol.grid.zlim_high = 0.90;
    pickVol.grid.zres = 1;
    pickVol.allow_internal_paths = false;

    target_volume placeVol;
    placeVol.grid.orientation.w = 0;
    placeVol.grid.orientation.x = -1;
    placeVol.grid.orientation.y = 0;
    placeVol.grid.orientation.z = 0;
    placeVol.grid.xlim_low = -.10;
    placeVol.grid.xlim_high = 0.10;
    placeVol.grid.xres = 1;
    placeVol.grid.ylim_low = -.10;
    placeVol.grid.ylim_high = 0.10;
    placeVol.grid.yres = 1;
    placeVol.grid.zlim_low = 0.15;
    placeVol.grid.zlim_high = 0.35;
    placeVol.grid.zres = 1;
    placeVol.allow_internal_paths = false;

    std::vector<target_volume> t_vols;
    t_vols.push_back(pickVol);
    t_vols.push_back(placeVol);

    /* Generate target joint values */
    ROS_INFO("Calculating target joint values.");
    tlib.generateTargets(t_vols);
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
