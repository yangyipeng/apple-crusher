#include "trajectory_library.h"

#define STUB ROS_INFO("LINE %d", __LINE__)

TrajectoryLibrary::TrajectoryLibrary(ros::NodeHandle& nh)
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
    _acm = _plan_scene->getAllowedCollisionMatrixNonConst();

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

    // Initialize time parameterizer
    _time_parametizer.reset(new trajectory_processing::IterativeParabolicTimeParameterization());

    // Initialize trajectory manager
    _execution_manager.reset(new trajectory_execution_manager::TrajectoryExecutionManager(_rmodel));
    if (!_execution_manager->ensureActiveControllersForGroup(UR5_GROUP_NAME))
    {
        ROS_ERROR("Controllers for joint group are not active.");
    }

    // Create publisher for rviz
    _trajectory_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    _plan_scene_publisher = nh.advertise<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", 1, true);
    _robot_state_publisher = nh.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 1, true);
    _collision_object_publisher = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 1);

    _num_plan_groups = 0;
    _num_target_groups = 0;

    return;
}

void TrajectoryLibrary::initWorld()
{
    collision_detection::WorldPtr world = _plan_scene->getWorldNonConst();

    moveit_msgs::CollisionObject object_msg;

    // Define workspace limits with planes
    shapes::ShapeConstPtr ground_plane(new shapes::Plane(0,0,1,0));
    geometry_msgs::Pose pose;
    pose.orientation.w = 1;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0;
    Eigen::Affine3d eigen_pose;
    tf::poseMsgToEigen(pose, eigen_pose);
    world->addToObject("workspace_bounds", ground_plane, eigen_pose);
    _acm.setEntry("workspace_bounds", "world", true);
    _acm.setEntry("workspace_bounds", "base_link", true);

    // Publish object
    shape_msgs::Plane plane;
    plane.coef[0] = 0;
    plane.coef[1] = 0;
    plane.coef[2] = 1;
    plane.coef[3] = 0;
    object_msg.id = "workspace_bounds";
    object_msg.header.frame_id = "world";
    object_msg.planes.push_back(plane);
    object_msg.primitive_poses.push_back(pose);
    object_msg.operation = object_msg.ADD;
    _collision_object_publisher.publish(object_msg);

    // Add obstructo-sphere in middle of workspace
//    shapes::ShapeConstPtr obstructo(new shapes::Sphere(0.2));
//    pose.position.x = 0;
//    pose.position.y = 0;
//    pose.position.z = 0.7;
//    tf::poseMsgToEigen(pose, eigen_pose);
//    world->addToObject("obstructo_sphere", obstructo, eigen_pose);
//    _acm.setEntry("obstructo_sphere", "world", true);
//    _acm.setEntry("obstructo_sphere", "base_link", true);
//    _acm.setEntry("obstructo_sphere", "shoulder_link", true);

//    // Publish object
//    shape_msgs::SolidPrimitive primitive;
//    primitive.type = primitive.SPHERE;
//    primitive.dimensions.resize(1);
//    primitive.dimensions[0] = 0.2;
//    object_msg.id = "obstructo_sphere";
//    object_msg.header.frame_id = "world";
//    object_msg.primitives.push_back(primitive);
//    object_msg.primitive_poses.push_back(pose);
//    object_msg.operation = object_msg.ADD;
//    _collision_object_publisher.publish(object_msg);

    // Publish updated planning scene
    moveit_msgs::PlanningScene scene_msg;
    _plan_scene->getPlanningSceneMsg(scene_msg);
    _plan_scene_publisher.publish(scene_msg);

    _plan_scene->printKnownObjects(std::cout);
    std::vector<std::string> collision_detector_names;
    _plan_scene->getCollisionDetectorNames(collision_detector_names);
    for (int i=0; i < collision_detector_names.size(); i++)
    {
        std::cout << "CD: " << collision_detector_names[i] << std::endl;
    }

    return;
}

std::size_t TrajectoryLibrary::gridLinspace(std::vector<joint_values_t>& jvals, rect_grid& grid)
{
    double di, dj, dk;
    di = dj = dk = 0.0;
    int num_poses;
    jvals.clear();

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

    geometry_msgs::Pose geo_pose;
    robot_state::RobotStatePtr state(new robot_state::RobotState(_rmodel));

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
                // Do IK
                ik_success = state->setFromIK(_jmg, geo_pose, 10, 0.2, boost::bind(&TrajectoryLibrary::ikValidityCallback, this, _1, _2, _3));
                if (!ik_success)
                {
                    ROS_WARN("Could not solve IK for pose %d: Skipping.", n);
                    printPose(geo_pose);
                    continue;
                }
                // If IK succeeded
                joint_values_t j;
                state->copyJointGroupPositions(_jmg, j);
                // Add to vector
                ROS_INFO("Successfully generated joint values for pose %d.", n);
                jvals.push_back(j);
            }
        }
    }
    return jvals.size();
}

bool TrajectoryLibrary::ikValidityCallback(robot_state::RobotState* p_state, const robot_model::JointModelGroup* p_jmg, const double* jvals)
{
    // Check for self-collisions
    p_state->setJointGroupPositions(p_jmg, jvals);
    p_state->update(true);
    return _plan_scene->isStateValid(*p_state, p_jmg->getName());
}

void TrajectoryLibrary::generateTargets(const std::vector<rect_grid> grids)
{
    target_volume vol;
    for (int i = 0; i < grids.size(); i++)
    {
        ROS_INFO("Generating joint value targets for group %d.", i);
        vol.grid = grids[i];
        vol.target_count = gridLinspace(vol.jvals, vol.grid);
        ROS_INFO("Generated %d of %d possible targets.", vol.target_count, vol.grid.xres*vol.grid.yres*vol.grid.zres);
        _target_groups.push_back(vol);
    }

    _num_target_groups = grids.size();
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
    //req.planner_id = "manipulator[LBKPIECEkConfigDefault]";
    req.planner_id = "manipulator[RRTConnectkConfigDefault]";
    //req.planner_id = "manipulator[RRTstarkConfigDefault]";
    req.allowed_planning_time = 5.0;

    // Define workspace
    req.workspace_parameters.max_corner.x = 1.0;
    req.workspace_parameters.max_corner.y = 1.0;
    req.workspace_parameters.max_corner.z = 0.7;
    req.workspace_parameters.min_corner.x = -1.0;
    req.workspace_parameters.min_corner.y = -1.0;
    req.workspace_parameters.min_corner.z = 0.25;

    // Now prepare the planning context
    int tries = 0;
    while (tries < 3)
    {
        planning_interface::PlanningContextPtr context = _planner->getPlanningContext(_plan_scene, req, res.error_code_);
        context->solve(res);
        if (res.error_code_.val == res.error_code_.SUCCESS)
        {
            robot_trajectory::RobotTrajectoryPtr traj(res.trajectory_);

            if (!_plan_scene->isPathValid(*traj, UR5_GROUP_NAME))
            {
                ROS_ERROR("Path invalid.");
                tries++;
                continue;
            }

            // Do optimization
            robot_trajectory::RobotTrajectoryPtr traj_opt(new robot_trajectory::RobotTrajectory(_rmodel, UR5_GROUP_NAME));
            optimizeTrajectory(traj_opt, traj);

            // Now slow it down for safety
            // timeWarpTrajectory(traj_opt, 3);

            // Pack motion plan struct
            moveit::core::robotStateToRobotStateMsg(_plan_scene->getCurrentState(), plan.start_state);
            moveit::core::robotStateToRobotStateMsg(traj_opt->getLastWayPoint(), plan.end_state);
            plan.num_wpts = traj_opt->getWayPointCount();
            plan.duration = traj_opt->getWaypointDurationFromStart(plan.num_wpts-1);
            traj_opt->getRobotTrajectoryMsg(plan.trajectory);

            ROS_INFO("Duration = %f.", plan.duration);
            return true;
        }
        // else planner failed
        tries++;
    }
    return false;
}

void TrajectoryLibrary::timeWarpTrajectory(robot_trajectory::RobotTrajectoryPtr traj, double slow_factor)
{
    std::size_t wpt_count = traj->getWayPointCount();
    for (std::size_t i = 1; i < wpt_count; i++)
    {
        double t = traj->getWayPointDurationFromPrevious(i);
        t = slow_factor*t;
        traj->setWayPointDurationFromPrevious(i, t);
    }
    return;
}

void TrajectoryLibrary::optimizeTrajectory(robot_trajectory::RobotTrajectoryPtr traj_opt, const robot_trajectory::RobotTrajectoryPtr traj)
{
    // Make a copy
    *traj_opt = *traj;

    std::size_t wpt_count = traj->getWayPointCount();
    ROS_INFO("Optimize starts with %d waypoints.", (int) wpt_count);

    // Iterate forwards from the first waypoint
    for (std::size_t i=0; i < wpt_count; i++)
    {
        // Do time parameterization
        _time_parametizer->computeTimeStamps(*traj_opt);

        // Check for shortcut to other waypoint
        // Starting from the end and going backwards
        for (std::size_t j=(wpt_count-1); j > i; j--)
        {
            // Define shortcut as straight line in joint space between waypoints i and j
            robot_state::RobotState wpt_i = traj_opt->getWayPoint(i);
            robot_state::RobotState wpt_j = traj_opt->getWayPoint(j);

            // Generate intermediate states and check each for collisions
            double duration = traj_opt->getWaypointDurationFromStart(j) - traj_opt->getWaypointDurationFromStart(i);
            std::size_t step_count = duration / DT_LOCAL_COLLISION_CHECK;
            double step_scaled;
            robot_state::RobotState step_state(_rmodel);
            bool shortcut_invalid = false;

            for (std::size_t s=1; s < step_count; s++)
            {
                step_scaled = s / ((float) step_count);
                // Calculate intermediate state at step s
                wpt_i.interpolate(wpt_j, step_scaled, step_state);
                step_state.update(true);
                // Check state for collision
                if (!_plan_scene->isStateValid(step_state, UR5_GROUP_NAME) )
                {
                    // ROS_INFO("Collision found at %f of the way between waypoints %d and %d.", step_scaled, (int) i, (int) j);
                    shortcut_invalid = true;
                    break;
                }
            }

            // If the shortcut is valid
            if (!shortcut_invalid)
            {
                //ROS_INFO("Shortcut found between nodes %d and %d.", (int) i, (int) j);
                // Create new trajectory object with only neccessary endpoints
                robot_trajectory::RobotTrajectory temp_traj(_rmodel, UR5_GROUP_NAME);
                // Copy in waypoints before the shortcut
                for (std::size_t n=0; n <= i; n++)
                {
                    robot_state::RobotStatePtr state = traj_opt->getWayPointPtr(n);
                    temp_traj.insertWayPoint(n, state, 0.0);
                }
                // Copy in waypoints after the shortcut
                for (std::size_t n=j; n < wpt_count; n++)
                {
                    robot_state::RobotStatePtr state = traj_opt->getWayPointPtr(n);
                    temp_traj.insertWayPoint(n - (j-i-1), state, 0.0);
                }
                ROS_ASSERT(temp_traj.getWayPointCount() == (wpt_count - (j-i-1)));

                // Copy temporary trajectory
                *traj_opt = temp_traj;
                wpt_count = traj_opt->getWayPointCount();
                break; // break out of j loop
            }

            // Otherwise shortcut is invalid, so move on
        }
    }

    // Do time parameterization again
    _time_parametizer->computeTimeStamps(*traj_opt);

    ROS_INFO("Successfully trimmed %d nodes.", (int) (traj->getWayPointCount() - traj_opt->getWayPointCount()) );
    return;
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

void TrajectoryLibrary::build()
{
    /* Check that target groups have been generated */
    if (_num_target_groups < 2)
    {
        ROS_ERROR("%d target groups defined. Need at least 2.", _num_target_groups);
        return;
    }

    _num_plan_groups = 0;

    /* Iterate through target groups for trajectory start location */
    for (int i = 0; i < _num_target_groups; i++)
    {
        ROS_INFO("START GROUP: %d", i);

        /* Iterate through target groups again for end location */
        for (int j = 0; j < _num_target_groups; j++)
        {
            if (j == i)
            {
                // We don't want to generate paths between targets in the same group
                continue;
            }
            ROS_INFO("END GROUP: %d", j);

            plan_group p_group;
            p_group.start_group = i;
            p_group.end_group = j;
            p_group.plan_count = 0;

            /* Pick particular start target */
            for (int n = 0; n < _target_groups[i].target_count; n++)
            {
                ROS_INFO("START TARGET: %d", n);

                // Construct trajectory start state
                robot_state::RobotState start_state(_rmodel);
                start_state.setJointGroupPositions(UR5_GROUP_NAME, _target_groups[i].jvals[n]);
                // TODO: Attach object if neccessary

                // Update planning scene with start state
                _plan_scene->setCurrentState(start_state);
                // Publish planning scene
                moveit_msgs::PlanningScene scene_msg;
                _plan_scene->getPlanningSceneDiffMsg(scene_msg);
                _plan_scene_publisher.publish(scene_msg);

                /* Now pick particular end target */
                for (int m = 0; m < _target_groups[j].target_count; m++)
                {
                    ROS_INFO("END TARGET: %d", m);

                    // Generate constraint from target joint values
                    std::vector<moveit_msgs::Constraints> v_constraints;
                    v_constraints.push_back( genJointValueConstraint( _target_groups[j].jvals[m] ) );
                    // Generate trajectory
                    ur5_motion_plan plan;
                    bool success = planTrajectory(plan, v_constraints);
                    if (!success)
                    {
                        ROS_ERROR("Planner failed to generate plan for end target %d. Skipping.", m);
                        continue;
                    }

                    // Now record start and stop locations
                    plan.start_target_index = n;
                    plan.end_target_index = m;

                    // Store trajectory in plan group
                    p_group.plans.push_back(plan);
                    p_group.plan_count++;
                }
            }

            // Store plan group
            _plan_groups.push_back(p_group);
            _num_plan_groups++;
        }
    }

    return;
}

void TrajectoryLibrary::demo()
{
    srand(0);

    // Pick random trajectory to base our first search off of
    int i = rand() % _num_plan_groups;
    int n = rand() % _plan_groups[i].plan_count;
    ur5_motion_plan* plan = _plan_groups[i].plans.data() + n;

    // Now extract end state and initialize end state to equal start state
    robot_state::RobotState start_state(_rmodel);
    robot_state::RobotState end_state(_rmodel);
    moveit::core::robotStateMsgToRobotState(plan->end_state, end_state);

    int prev_end_group = _plan_groups[i].end_group;
    int prev_end_target = plan->end_target_index;

    ROS_INFO("%d plan groups.", _num_plan_groups);
    ROS_INFO("Starting at group %d target %d.", prev_end_group, prev_end_target);

    while (1)
    {
        ///////////// Search randomly for next trajectory
        std::vector<bool> tried;
        tried.assign(_num_plan_groups, false);
        bool success = false;
        // First find valid plan group, until you've exhausted the list
        while (std::find(tried.begin(), tried.end(), false) != tried.end())
        {
            i = rand() % _num_plan_groups;
            if (tried[i])
            {
                continue;
            }
            if (_plan_groups[i].start_group == prev_end_group)
            {
                success = true;
                break;
            }
            tried[i] = true;
        }

        if (!success)
        {
            ROS_INFO("No valid plan found. Random trajectory selected.");
            i = rand() % _num_plan_groups;
            n = rand() % _plan_groups[i].plan_count;
        }
        else
        {
            // Assuming we have valid plan group, now try to find valid plan
            success = false;
            tried.clear();
            tried.assign(_plan_groups[i].plan_count, false);
            while (std::find(tried.begin(), tried.end(), false) != tried.end())
            {
                n = rand() % _plan_groups[i].plan_count;
                if (tried[n])
                {
                    continue;
                }
                if (_plan_groups[i].plans[n].start_target_index == prev_end_target)
                {
                    success = true;
                    break;
                }
                tried[n] = true;
            }

            if (!success)
            {
                ROS_INFO("No valid plan found. Random trajectory selected.");
                i = rand() % _num_plan_groups;
                n = rand() % _plan_groups[i].plan_count;
            }
        }

        // Use i and n to assign plan
        plan = _plan_groups[i].plans.data() + n;

        ROS_INFO("Moving from group %d target %d to group %d target %d.", _plan_groups[i].start_group, plan->start_target_index,
                 _plan_groups[i].end_group, plan->end_target_index);

        // Update previous end target data
        prev_end_group = _plan_groups[i].end_group;
        prev_end_target = plan->end_target_index;

        ////////////// Now demo the trajectory

        // Extract start state
        moveit::core::robotStateMsgToRobotState(plan->start_state, start_state);
        ROS_INFO("Distance between start state and previous end state is %f.", start_state.distance(end_state));

        // Extract end state
        moveit::core::robotStateMsgToRobotState(plan->end_state, end_state);

        // Set current start state
        _plan_scene->setCurrentState(start_state);

        // Publish planning scene and end state as separate messages
        moveit_msgs::PlanningScene scene_msg;
        _plan_scene->getPlanningSceneDiffMsg(scene_msg);
        _plan_scene_publisher.publish(scene_msg);

        moveit_msgs::DisplayRobotState state_msg;
        state_msg.state = plan->end_state;
        _robot_state_publisher.publish(state_msg);

        // Get trajectory data
        robot_trajectory::RobotTrajectory traj(_rmodel, UR5_GROUP_NAME);
        traj.setRobotTrajectoryMsg(start_state, plan->trajectory);
        int num_wpts = plan->num_wpts;
        double duration = traj.getWaypointDurationFromStart(num_wpts-1);
        ROS_INFO("Trajectory has %d nodes and takes %f seconds.", num_wpts, duration);

        // Publish trajectory
        moveit_msgs::DisplayTrajectory display_trajectory;
        display_trajectory.trajectory_start = plan->start_state;
        display_trajectory.trajectory.push_back(plan->trajectory);
        _trajectory_publisher.publish(display_trajectory);

        // Execute trajectory
        _execution_manager->pushAndExecute(plan->trajectory);

        // Now delay for duration of trajectory
//        ros::WallDuration sleep_time(duration);
//        sleep_time.sleep();
        _execution_manager->waitForExecution();
    }

    return;
}

bool TrajectoryLibrary::fetchPlan(ur5_motion_plan& plan, int start_group, int start_index, int end_group, int end_index)
{
    // First find plan group
    bool success = false;
    plan_group* p_group;
    for (int i=0; i < _num_plan_groups; i++)
    {
        p_group = &(_plan_groups[i]);
        if (p_group->start_group == start_group && p_group->end_group == end_group)
        {
            success = true;
            break;
        }
    }

    if (!success)
    {
        // We couldn't find corresponding plan group
        return false;
    }

    // Now search for plan within group
    for (int i=0; i < p_group->plan_count; i++)
    {
        if (p_group->plans[i].start_target_index == start_index)
        {
            if (p_group->plans[i].end_target_index == end_index)
            {
                plan = p_group->plans[i];
                return true;
            }
        }
    }
    // If no match
    return false;
}

bool TrajectoryLibrary::filewrite(const plan_group &p_group, const char* filename, bool debug)
{
    bool check;
    int itersize = p_group.plan_count;
    int nodesize = 0;
    double radius = 0.05;
    int8_t ADD = 0;

    std::ofstream file;
    file.open (filename, std::ofstream::out | std::ofstream::binary);
    if(file.is_open())
    {
        file.write((char *)(&itersize),sizeof(itersize));
        if (debug == 1) ROS_INFO("%d",itersize);
        for (size_t n = 0; n < itersize; n++)
        {
            //RobotTrajectory -> JointTrajectory -> JointTrajectoryPoints
            nodesize = p_group.plans[n].trajectory.joint_trajectory.points.size();
            ROS_INFO("%d",nodesize);
            file.write((char *)(&nodesize),sizeof(nodesize));
            for (size_t idx = 0; idx < nodesize; idx++)
            {
                for (size_t i=0; i < 6; i++) file.write((char *)(&p_group.plans[n].trajectory.joint_trajectory.points[idx].positions[i]),sizeof(double));
                file.write((char *)(&p_group.plans[n].trajectory.joint_trajectory.points[idx].time_from_start),sizeof(ros::Duration));
            }

            //RobotTrajectory -> JointTrajectory -> Header
            file.write((char *)(&p_group.plans[n].trajectory.joint_trajectory.header.seq),sizeof(uint32_t));
            file.write((char *)(&p_group.plans[n].trajectory.joint_trajectory.header.stamp),sizeof(ros::Time));
            file << p_group.plans[n].trajectory.joint_trajectory.header.frame_id << '\n';

            //RobotTrajectory -> JointTrajectory -> joint_names
            for (size_t i=0; i < 6; i++) file << p_group.plans[n].trajectory.joint_trajectory.joint_names[i] << '\n';


            //start_state
            //RobotState -> JointState -> Header
            file.write((char *)(&p_group.plans[n].start_state.joint_state.header.seq),sizeof(uint32_t));
            file.write((char *)(&p_group.plans[n].start_state.joint_state.header.stamp),sizeof(ros::Time));
            file << p_group.plans[n].start_state.joint_state.header.frame_id << '\n';
            //RobotState -> JointState -> stirng & position
            for (size_t j = 0; j < 6; j++)
            {
                file << p_group.plans[n].start_state.joint_state.name[j] << '\n';
                file.write((char *)(&p_group.plans[n].start_state.joint_state.position[j]),sizeof(double));
            }

            //end_state
            //RobotState -> JointState -> Header
            file.write((char *)(&p_group.plans[n].end_state.joint_state.header.seq),sizeof(uint32_t));
            file.write((char *)(&p_group.plans[n].end_state.joint_state.header.stamp),sizeof(ros::Time));
            file << p_group.plans[n].end_state.joint_state.header.frame_id << '\n';
            //RobotState -> JointState -> stirng & position
            for (size_t j = 0; j < 6; j++)
            {
                file << p_group.plans[n].end_state.joint_state.name[j] << '\n';
                file.write((char *)(&p_group.plans[n].end_state.joint_state.position[j]),sizeof(double));
            }

            //index
            file.write((char *)(&p_group.plans[n].start_target_index),sizeof(unsigned int));
            file.write((char *)(&p_group.plans[n].end_target_index),sizeof(unsigned int));

            //write an apple
            file << "ee_link"  << '\n' << "apple" << '\n';
            file.write((char *)(&radius),sizeof(double));


        }

//        //string test;
//        p_group.plans[0].trajectory.joint_trajectory.header.frame_id = "This is a Test. Also Im Hungry!!";
//        std::cout << p_group.plans[0].trajectory.joint_trajectory.header.frame_id << '\n';
//        file << p_group.plans[0].trajectory.joint_trajectory.header.frame_id  << '\n';

        check = 1;
        file.close();
    }
    else check = 0;

    return check;
}

bool TrajectoryLibrary::fileread(plan_group& p_group, const char* filename, bool debug)
{
    bool check;
    int wpt_count;
    int plan_count;

    //int nodeset = 10;
    double temp_double;
    std::string temp_string;
    std::string blank_string;
//    uint32_t temp_uint32;
//    ros::Time temp_time;
    ros::Duration temp_duration;

    ur5_motion_plan ur5;
    ur5_motion_plan empty;
    std::vector<int> temp;
    trajectory_msgs::JointTrajectoryPoint temp_points;
    trajectory_msgs::JointTrajectoryPoint blank;

    std::ifstream info;
    info.open(filename,std::ifstream::in | std::ofstream::binary);

    if(info.is_open())
    {
        // Iterate through trajectories
        info.read((char*)(&plan_count),sizeof(plan_count));

        if (debug == 1) ROS_INFO("%d",plan_count);
        for (size_t n = 0; n < plan_count; n++)
        {
            // Iterate through waypoints
            info.read((char*)(&wpt_count),sizeof(wpt_count));
            temp.push_back(wpt_count);
            if (debug == 1) ROS_INFO("%d",wpt_count);
            for (size_t idx = 0; idx < wpt_count; idx++)
            {
                for (size_t i=0; i<6; i++)
                {
                    info.read((char *)(&temp_double),sizeof(temp_double));
                    temp_points.positions.push_back(temp_double);
                }
                info.read((char *)(&temp_duration),sizeof(temp_duration));
                //ros::Duration d(z);
                temp_points.time_from_start = temp_duration;
                ur5.trajectory.joint_trajectory.points.push_back(temp_points);
                temp_points = blank;

            }

            //RobotTrajectory -> JointTrajectory -> Header
            info.read((char *)(&ur5.trajectory.joint_trajectory.header.seq),sizeof(uint32_t));
            info.read((char *)(&ur5.trajectory.joint_trajectory.header.stamp),sizeof(ros::Time));
            getline (info,ur5.trajectory.joint_trajectory.header.frame_id);

            //RobotTrajectory -> JointTrajectory -> joint_names
            for (size_t i=0; i < 6; i++)
            {
                getline(info,temp_string);
                ur5.trajectory.joint_trajectory.joint_names.push_back(temp_string);
                temp_string = blank_string;
            }

            //start_state
            //RobotState -> JointState -> Header
            info.read((char *)(&ur5.start_state.joint_state.header.seq),sizeof(uint32_t));
            info.read((char *)(&ur5.start_state.joint_state.header.stamp),sizeof(ros::Time));
            getline (info,ur5.start_state.joint_state.header.frame_id);

            //RobotState -> JointState -> stirng & position
            for (size_t j = 0; j < 6; j++)
            {
                getline(info,temp_string);
                ur5.start_state.joint_state.name.push_back(temp_string);
                temp_string = blank_string;

                info.read((char *)(&temp_double),sizeof(temp_double));
                ur5.start_state.joint_state.position.push_back(temp_double);
            }

            //end_state
            //RobotState -> JointState -> Header
            info.read((char *)(&ur5.end_state.joint_state.header.seq),sizeof(uint32_t));
            info.read((char *)(&ur5.end_state.joint_state.header.stamp),sizeof(ros::Time));
            getline (info,ur5.end_state.joint_state.header.frame_id);

            //RobotState -> JointState -> stirng & position
            for (size_t j = 0; j < 6; j++)
            {
                getline(info,temp_string);
                ur5.end_state.joint_state.name.push_back(temp_string);
                temp_string = blank_string;

                info.read((char *)(&temp_double),sizeof(temp_double));
                ur5.end_state.joint_state.position.push_back(temp_double);

            }

            //Index
            info.read((char *)(&ur5.start_target_index),sizeof(unsigned int));
            info.read((char *)(&ur5.end_target_index),sizeof(unsigned int));

            //read an apple
            /* Define the attached object message*/
            getline(info,temp_string);
            std::cout << temp_string << '\n';
            ur5.end_state.attached_collision_objects.resize(1);
            ur5.end_state.attached_collision_objects[0].link_name = temp_string;
            ur5.end_state.attached_collision_objects[0].object.header.frame_id = temp_string;
            temp_string = blank_string;
            getline(info,temp_string);
            ur5.end_state.attached_collision_objects[0].object.id = temp_string;
            temp_string = blank_string;

            /* Define a box to be attached */
            geometry_msgs::Pose pose;
            shape_msgs::SolidPrimitive primitive;
            ROS_INFO("TESTING");
            info.read((char *)(&temp_double),sizeof(double));
            pose.position.x = temp_double;
            primitive.type = primitive.SPHERE;
            primitive.dimensions.resize(1);
            primitive.dimensions[0] = temp_double;

            ur5.end_state.attached_collision_objects[0].object.primitives.push_back(primitive);
            ur5.end_state.attached_collision_objects[0].object.primitive_poses.push_back(pose);

            /* An attach operation requires an ADD */
            ur5.end_state.attached_collision_objects[0].object.operation = ur5.end_state.attached_collision_objects[0].object.ADD;

            // Other parameters
            ur5.num_wpts = wpt_count;

            p_group.plans.push_back(ur5);
            ur5 = empty;
        }

//        //string test;
//        std::string James;
//        getline (info,James);
//        std::cout << James << '\n';

        check = 1;
        info.close();
    }
    else check = 0;

    p_group.plan_count = plan_count;

    return check;

}

void TrajectoryLibrary::exportToFile()
{
    // SAVE DATA TO .bin FILE
    ROS_INFO("--------------SAVING!!!!-------------------");
    for (int i = 0; i < _num_plan_groups; i++)
    {
        std::string filename = "plangroup" + boost::lexical_cast<std::string>(i) + ".bin";
        filewrite(_plan_groups[i], filename.c_str(), false);
    }
    return;
}

void TrajectoryLibrary::importFromFile(int num_plan_groups)
{
    _num_plan_groups = 0;
    //LOAD DATA FROM .bin FILE
    ROS_INFO("--------------LOADING!!!!-------------------");
    for (int i = 0; i < num_plan_groups; i++)
    {
        std::string filename = "plangroup" + boost::lexical_cast<std::string>(i) + ".bin";
        plan_group p_group;
        if ( fileread(p_group, filename.c_str(), false) )
        {
            _num_plan_groups++;
            _plan_groups.push_back(p_group);
        }
    }
    return;
}
