#include "trajectory_library.h"

#define STUB ROS_INFO("LINE %d", __LINE__)

TrajectoryLibrary::TrajectoryLibrary(ros::NodeHandle nh)
{
    /* Load up robot model */
    ROS_INFO("Loading ur5 robot model.");
    _rmodel_loader.reset(new robot_model_loader::RobotModelLoader("robot_description"));
    _rmodel = _rmodel_loader->getModel();

    ROS_INFO("Loaded model %s.", _rmodel->getName().c_str());

    ROS_INFO("Grabbing JointModelGroup.");
    _jmg = _rmodel->getJointModelGroup(UR5_GROUP_NAME);

    /* Init planning scene */
    ROS_INFO("Initializing PlanningScene from RobotModel");
    _plan_scene = planning_scene::PlanningScenePtr(new planning_scene::PlanningScene(_rmodel));

    /* Load the planner */
    ROS_INFO("Loading the planner plugin.");
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
    try
    {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
    }
    catch(pluginlib::PluginlibException& ex)
    {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
        _planner.reset(planner_plugin_loader->createUnmanagedInstance("ompl_interface/OMPLPlanner"));
        _planner->initialize(_rmodel, nh.getNamespace());
    }
    catch(pluginlib::PluginlibException& ex)
    {
        const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (std::size_t i = 0 ; i < classes.size() ; ++i)
        ss << classes[i] << " ";
        ROS_ERROR_STREAM("Exception while loading planner: " << ex.what() << std::endl
                       << "Available plugins: " << ss.str());
    }

    // Create publisher for rviz
    _pub = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    return;
}

std::size_t TrajectoryLibrary::gridLinspace(std::vector<joint_values_t>& jvals, rect_grid& grid)
{
    double di, dj, dk;
    di = dj = dk = 0.0;
    int num_poses;

    // Calculate grid spacings if xres != 1
    if (grid.xres != 1) {
        di = (grid.xlim_high - grid.xlim_low)/(grid.xres-1);
    }
    if (grid.yres != 1) {
        dj = (grid.ylim_high - grid.ylim_low)/(grid.yres-1);
    }
    if (grid.zres != 1) {
        dk = (grid.zlim_high - grid.zlim_low)/(grid.zres-1);
    }

    // Reserve ahead of time the number of poses for speed
    num_poses = grid.xres * grid.yres * grid.zres;
    jvals.reserve(num_poses);
    ROS_INFO("Attempting to generate %d targets.", num_poses);

    geometry_msgs::Pose geo_pose;
    robot_state::RobotStatePtr state(new robot_state::RobotState(_rmodel));
    state->setToDefaultValues();
    const robot_state::JointModelGroup* jmg = state->getJointModelGroup(UR5_GROUP_NAME);
    kinematics::KinematicsQueryOptions ik_options;
    ik_options.lock_redundant_joints = false;
    ik_options.return_approximate_solution = false;

    // Linspace
    int n = -1;
    for (int i = 0; i < grid.xres; i++)
    {
        for (int j = 0; j < grid.yres; j++)
        {
            for (int k = 0; k < grid.zres; k++)
            {
                n++;
                geo_pose.position.x = grid.xlim_low + i*di;
                geo_pose.position.y = grid.ylim_low + j*dj;
                geo_pose.position.z = grid.zlim_low + k*dk;
                geo_pose.orientation = grid.orientation;
                bool ik_success;
                STUB;
                // Try fixed number of times to find valid, non-colliding solution
                int tries;
                for (tries = 0; tries < 3; tries++)
                {
                    // Do IK
                    ROS_INFO("Can do IK? : %d.", _jmg->canSetStateFromIK(UR5_GROUP_NAME));
                    STUB;
                    ik_success = state->setFromIK(_jmg, geo_pose, 5, 0.1);
                    STUB;
                    if (!ik_success)
                    {
                        ROS_WARN("Could not solve IK for pose %d: Skipping.", n);
                        printPose(geo_pose);
                        break;
                    }
                    // If IK succeeded
                    joint_values_t j;
                    state->copyJointGroupPositions(UR5_GROUP_NAME, j);
                    STUB;
                    // Now check for self-collisions
                    if (!_plan_scene->isStateColliding(*state, UR5_GROUP_NAME))
                    {
                        // Add to vector
                        ROS_INFO("Successfully generated joint values for pose %d.", n);
                        STUB;
                        jvals.push_back(j);
                        break;
                    }
                    ROS_INFO("State self-collision. Retrying.");
                }
                if (tries == 3)
                {
                    ROS_ERROR("Could not find non-collision joint state for pose %d: Skipping.", n);
                    printPose(geo_pose);
                }
            }
        }
    }
    return jvals.size();
}

bool TrajectoryLibrary::ikValidityCallback(robot_state::RobotState* p_state, const robot_model::JointModelGroup* p_jmg, const double* jvals)
{
    STUB;
    // Check for self-collisions
    p_state->setJointGroupPositions(p_jmg, jvals);
    return !(_plan_scene->isStateValid(*p_state, p_jmg->getName()));
}

void TrajectoryLibrary::generateJvals(rect_grid& pick_grid, rect_grid& place_grid)
{
    _pick_grid = pick_grid;
    ROS_INFO("Generating pick joint values.");
    _num_pick_targets = gridLinspace(_pick_jvals, _pick_grid);

    _place_grid = place_grid;
    ROS_INFO("Generating place joint values.");
    _num_place_targets = gridLinspace(_place_jvals, _place_grid);

    return;
}

void TrajectoryLibrary::printPose(const geometry_msgs::Pose& pose)
{
    printf("     Position (%f, %f, %f).\n", pose.position.x, pose.position.y, pose.position.z);
    printf("     Orientation (%f, %f, %f, %f).\n", pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    return;
}

void TrajectoryLibrary::printJointValues(const joint_values_t& jvals)
{
    printf("Joint values: ");
    for (int i=0; i<jvals.size(); i++)
    {
        printf(" %f", jvals[i]);
    }
    printf("\n");
    return;
}

bool TrajectoryLibrary::planTrajectory(ur5_motion_plan& plan, std::vector<moveit_msgs::Constraints> constraints)
{
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    req.group_name = UR5_GROUP_NAME;

    // Add constraints
    req.goal_constraints = constraints;
    req.planner_id = "manipulator[RRTConnectkConfigDefault]";
    req.allowed_planning_time = 5.0;

    // Define workspace
    req.workspace_parameters.max_corner.x = 1.0;
    req.workspace_parameters.max_corner.y = 1.0;
    req.workspace_parameters.max_corner.z = 0.7;
    req.workspace_parameters.min_corner.x = -1.0;
    req.workspace_parameters.min_corner.y = -1.0;
    req.workspace_parameters.min_corner.z = 0.05;

    // Now prepare the planning context
    int tries = 0;
    while (tries < 3)
    {
        planning_interface::PlanningContextPtr context = _planner->getPlanningContext(_plan_scene, req, res.error_code_);
        context->solve(res);
        if (res.error_code_.val == res.error_code_.SUCCESS)
        {
            moveit_msgs::MotionPlanResponse response;
            res.getMessage(response);

            moveit::core::robotStateToRobotStateMsg(_plan_scene->getCurrentState(), plan.start_state);
            moveit::core::robotStateToRobotStateMsg(res.trajectory_->getLastWayPoint(), plan.end_state);
            plan.trajectory = response.trajectory;
            return true;
        }
        // else planner failed
        tries++;
    }
    return false;
}

moveit_msgs::Constraints TrajectoryLibrary::genPoseConstraint(geometry_msgs::Pose pose_goal)
{
    geometry_msgs::PoseStamped pose_pkt;
    pose_pkt.header.frame_id = "world";
    pose_pkt.pose = pose_goal;
    // Position and orientation tolerances
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    // Create constraint from pose using IK
    return kinematic_constraints::constructGoalConstraints("ee_link", pose_pkt, tolerance_pose, tolerance_angle);
}

moveit_msgs::Constraints TrajectoryLibrary::genJointValueConstraint(joint_values_t jvals)
{
    // Initialize state variable with joint values
    robot_state::RobotState state(_rmodel);
    state.setJointGroupPositions(UR5_GROUP_NAME, jvals);
    // Verify joint values are within bounds
    if (!_jmg->satisfiesPositionBounds( jvals.data() ) )
    {
        ROS_ERROR("Joint value constraint does not satisfy position bounds.");
    }
    // Create constraint
    return kinematic_constraints::constructGoalConstraints(state, _jmg, 0.01);
}

int TrajectoryLibrary::buildLibrary()
{
    /* Check that target grids have been generated */
    if (_num_pick_targets == 0 || _num_place_targets == 0)
    {
        ROS_ERROR("No pick or place targets defined. Cannot build library.");
        return 0;
    }

    _num_trajects = 0;

    /* Iterate through place targets */
    for (int n = 0; n < _num_place_targets; n++)
    {
        sleep(5);
        // Solve IK for place position n
        ROS_INFO("Jumping to place pose %d", n);
        robot_state::RobotState place_state(_rmodel);
        place_state.setJointGroupPositions(UR5_GROUP_NAME, _place_jvals[n]);
        // Jump current state to place pose
        _plan_scene->setCurrentState(place_state);

        /* Iterate through pick targets */
        for (int m = 0; m < _num_pick_targets; m++) {

            ////////////////// Pick target

            // Assume we are at a place pose
            // Set a pick target
            std::vector<moveit_msgs::Constraints> v_constraints;
            v_constraints.push_back(genJointValueConstraint(_pick_jvals[m]));
            // Generate trajectory
            ur5_motion_plan pick_traj;
            bool success = planTrajectory(pick_traj, v_constraints);
            if (!success)
            {
                ROS_ERROR("Planner failed to generate plan for pick target %d. Skipping.", m);
                continue;
            }

            // Now change state to our pick target pose
            _plan_scene->setCurrentState(pick_traj.end_state);
            ROS_INFO("Successfully planned pick trajectory.");

            // TODO: Attach object (ie apple) for return trajectory



            //////////////////// Place target

            // Assume we are at a pick pose
            // Set a place target (in joint space)
            v_constraints.clear();
            v_constraints.push_back(genJointValueConstraint(_place_jvals[n]));;
            // Generate trajectory
            ur5_motion_plan place_traj;
            success = planTrajectory(place_traj, v_constraints);
            if (!success)
            {
                ROS_ERROR("Planner failed to generate plan for place target %d. Skipping.", m);
                continue;
            }
            // Add trajectory to display message and publish to Rviz
            moveit_msgs::DisplayTrajectory display_trajectory;
            display_trajectory.trajectory_start = pick_traj.start_state;
            display_trajectory.trajectory.push_back(pick_traj.trajectory);
            display_trajectory.trajectory.push_back(place_traj.trajectory);
            _pub.publish(display_trajectory);

            // Store trajectories in library
            _pick_trajects.push_back(pick_traj);
            _place_trajects.push_back(place_traj);

            // Now change state back to our place target pose
            _plan_scene->setCurrentState(place_state);

            // TODO: Detach object (apple) for next pick trajectory

            ROS_INFO("Successfully planned place trajectory.");
            _num_trajects++;
            ROS_INFO("Trajectory set %d saved.", (int) _num_trajects);

            /* Sleep a little to allow time for rviz to display path */
            sleep(1);
        }
    }

    return _num_trajects;
}
