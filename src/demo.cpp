#include "trajectory_library.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Demo Library");
    ros::NodeHandle nh("~");

    double BUSH_RADIUS;
    if (nh.hasParam("bush_radius"))
    {
        nh.getParam("bush_radius", BUSH_RADIUS);
    }

    TrajectoryLibrary tlib(nh);
    tlib.initWorkspaceBounds();
    tlib.addSphereCollisionObject(BUSH_RADIUS);
    tlib.printCollisionWorldInfo(std::cout);

    // Define weed soil volume grid
    target_volume weedSoilVol;
    weedSoilVol.type = GRID_RECT;
    weedSoilVol.grid.orientation.w = sqrt(0.5);
    weedSoilVol.grid.orientation.x = 0;
    weedSoilVol.grid.orientation.y = -sqrt(0.5);
    weedSoilVol.grid.orientation.z = 0;
    weedSoilVol.grid.xlim_low = -0.35;
    weedSoilVol.grid.xlim_high = 0.35;
    weedSoilVol.grid.xres = 4;
    weedSoilVol.grid.ylim_high = -0.25;
    weedSoilVol.grid.ylim_low = 0.25;
    weedSoilVol.grid.yres = 2;
    weedSoilVol.grid.zlim_low = 0.80;
    weedSoilVol.grid.zlim_high = 0.89;
    weedSoilVol.grid.zres = 1;
    weedSoilVol.allow_internal_paths = true;

    std::vector<target_volume> t_vols;
    t_vols.push_back(weedSoilVol);
    tlib.setTargetVolumes(t_vols);

    // Import plans
    tlib.importFromFile("plans_weeding.dat");

    ROS_INFO("Hit enter to begin demo.");
    std::cin.ignore(100, '\n');
    tlib.demo();

    ros::shutdown();
    return 0;
}
