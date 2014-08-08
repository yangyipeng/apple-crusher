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

    // Load up the planning pipeline
    _planning_pipeline.reset(new planning_pipeline::PlanningPipeline(_rmodel, nh, "/planning_plugin", "/planning_adapters"));

    // Initialize time parameterizer
    _time_parametizer.reset(new trajectory_processing::IterativeParabolicTimeParameterization());

    // Create publisher for rviz
    _trajectory_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    _plan_scene_publisher = nh.advertise<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", 1, true);
    _robot_state_publisher = nh.advertise<moveit_msgs::DisplayRobotState>("/display_robot_state", 1, true);
    _collision_object_publisher = nh.advertise<moveit_msgs::CollisionObject>("/collision_object", 1);

    _num_target_groups = 0;

    // Initialize KD Tree
    std::vector<double> low_bounds(12,-M_PI-0.10);
    std::vector<double> high_bounds(12, M_PI+0.10);
    std::vector<std::size_t> res(12, 10);
    _kdtree.reset(new KDTree(_rmodel, low_bounds, high_bounds, res));

    return;
}

void TrajectoryLibrary::initWorkspaceBounds()
{
    moveit_msgs::CollisionObject object_msg;
    geometry_msgs::Pose pose;

    ///////////////// Define workspace bounds as set of 6 planes
    object_msg.header.frame_id = "world";
    object_msg.operation = object_msg.ADD;

    // Zlow plane
    object_msg.id = "zlow_plane";
    shape_msgs::Plane plane;
    pose.orientation.w = 1;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0;
    plane.coef[0] = 0;
    plane.coef[1] = 0;
    plane.coef[2] = 1;
    plane.coef[3] = 0;
    object_msg.planes.push_back(plane);
    object_msg.plane_poses.push_back(pose);
    _plan_scene->processCollisionObjectMsg(object_msg);
    _acm.setEntry("zlow_plane", "world", true);
    _acm.setEntry("zlow_plane", "base_link", true);
    _acm.setEntry("zlow_plane", "shoulder_link", true);

    // Zhigh plane
    object_msg.id = "zhigh_plane";
    pose.position.z = 0.9;
    object_msg.planes[0] = plane;
    object_msg.plane_poses[0] = pose;
    _plan_scene->processCollisionObjectMsg(object_msg);

    // Ylow plane
    object_msg.id = "ylow_plane";
    plane.coef[0] = 0;
    plane.coef[1] = 1;
    plane.coef[2] = 0;
    plane.coef[3] = 0;
    pose.position.x = 0;
    pose.position.y = -0.40;
    pose.position.z = 0;
    object_msg.planes[0] = plane;
    object_msg.plane_poses[0] = pose;
    _plan_scene->processCollisionObjectMsg(object_msg);

    // Yhigh plane
    object_msg.id = "yhigh_plane";
    pose.position.y = 0.4;
    object_msg.planes[0] = plane;
    object_msg.plane_poses[0] = pose;
    _plan_scene->processCollisionObjectMsg(object_msg);

    // Xlow plane
    object_msg.id = "xlow_plane";
    plane.coef[0] = 1;
    plane.coef[1] = 0;
    plane.coef[2] = 0;
    plane.coef[3] = 0;
    pose.position.x = -0.4;
    pose.position.y = 0;
    pose.position.z = 0;
    object_msg.planes[0] = plane;
    object_msg.plane_poses[0] = pose;
    _plan_scene->processCollisionObjectMsg(object_msg);

    // Xhigh plane
    object_msg.id = "xhigh_plane";
    pose.position.x = 0.4;
    object_msg.planes[0] = plane;
    object_msg.plane_poses[0] = pose;
    _plan_scene->processCollisionObjectMsg(object_msg);

    // Publish updated planning scene
    moveit_msgs::PlanningScene scene_msg;
    _plan_scene->getPlanningSceneMsg(scene_msg);
    _plan_scene_publisher.publish(scene_msg);

    return;
}

void TrajectoryLibrary::addSphereCollisionObject(double radius)
{
    /////////////////// Obstructo sphere
    moveit_msgs::CollisionObject object_msg;
    object_msg.header.frame_id = "world";
    object_msg.operation = object_msg.ADD;

    // Publish object
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.SPHERE;
    primitive.dimensions.resize(1);
    primitive.dimensions[0] = radius;
    geometry_msgs::Pose pose;
    pose.orientation.w = 1;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0.9 - radius*0.6; // place sphere on "ground" (robot is upside down on Ladybird, ~0.9 m above the ground)
    object_msg.id = "obstructo_sphere";
    object_msg.header.frame_id = "world";
    object_msg.primitives.clear();
    object_msg.primitives.push_back(primitive);
    object_msg.primitive_poses.clear();
    object_msg.primitive_poses.push_back(pose);
    object_msg.operation = object_msg.ADD;
    _plan_scene->processCollisionObjectMsg(object_msg);

    _acm.setEntry("obstructo_sphere", "world", true);
    _acm.setEntry("obstructo_sphere", "base_link", true);
    _acm.setEntry("obstructo_sphere", "shoulder_link", true);

    // Publish updated planning scene
    moveit_msgs::PlanningScene scene_msg;
    _plan_scene->getPlanningSceneMsg(scene_msg);
    _plan_scene_publisher.publish(scene_msg);

    return;
}

void TrajectoryLibrary::printCollisionWorldInfo(std::ostream& cout)
{
    _plan_scene->printKnownObjects(cout);
    _acm.print(cout);

    return;
}

std::size_t TrajectoryLibrary::rectLinspace(std::vector<joint_values_t>& jvals, grid_rect& grid)
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

                // We want to generate a number of joint value targets for each geo pose
                std::vector<joint_values_t> solutions;
                bool success = doIK(solutions, geo_pose);

                if (success)
                {
                    // Now we push any values contained in geo_jvals into our jvals vector
                    for (int m = 0; m < solutions.size(); m++)
                    {
                        jvals.push_back(solutions[m]);
                    }
                    ROS_INFO("Generated %d solutions for geo_pose %d.", (int) solutions.size(), n);
                }

                // If we had none to push
                if (solutions.size() == 0)
                {
                    printPose(geo_pose);
                    ROS_WARN("Could not solve IK for pose %d: Skipping.", n);
                }
            }
        }
    }
    return jvals.size();
}

std::size_t TrajectoryLibrary::sphereLinspace(std::vector<joint_values_t> &jvals, grid_sphere &sphere)
{
//    double lat;
//    double longitude;
//    std::size_t target_count = 0;

//    double dlong = M_PI/sphere.long_res;





//    return target_count;
}

bool TrajectoryLibrary::doIK(std::vector<joint_values_t>& solutions, const geometry_msgs::Pose& geo_pose)
{
    // We want to generate a number of joint value targets for each geo pose
    bool ik_success;
    robot_state::RobotState state(_rmodel);
    for (int tries = 0; tries < MAX_IK_SOLUTIONS; tries++)
    {
        // Do IK
        ik_success = state.setFromIK(_jmg, geo_pose, 5, 0.4, boost::bind(&TrajectoryLibrary::ikValidityCallback, this, solutions, _1, _2, _3));
        if (!ik_success)
        {
            break;
        }

        // If IK succeeded
        joint_values_t j;
        state.copyJointGroupPositions(_jmg, j);
        // Add to vector
        solutions.push_back(j);
    }

    // If we had none to push
    if (solutions.size() == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool TrajectoryLibrary::ikValidityCallback(const std::vector<joint_values_t>& comparison_values, robot_state::RobotState* p_state, const robot_model::JointModelGroup* p_jmg, const double* jvals)
{
    // ROS_INFO("IK Validity checker...");
    // Construct state from given joint values
    p_state->setJointGroupPositions(p_jmg, jvals);
    p_state->update(true);

    // Check if state is valid
    if (!_plan_scene->isStateValid(*p_state, p_jmg->getName(), false))
    {
        return false;
    }

    // Now make sure state is not too similar to the comparison values
    robot_state::RobotState comp_state(*p_state);
    for (int c = 0; c < comparison_values.size(); c++)
    {
        comp_state.setJointGroupPositions(p_jmg, comparison_values[c]);
        // Calculate distance in joint space
        double dist = p_state->distance(comp_state);
        // ROS_INFO("Dist = %f", dist);
        // If this is below our threshold distance for any comparison state
        if (dist < IK_COMP_MIN_DIST)
        {
            return false;
        }
    }

    // If we made it this far
    return true;
}

void TrajectoryLibrary::setTargetVolumes(const std::vector<target_volume> &vols)
{
    target_group t_group;

    _num_target_groups = vols.size();

    for (int i = 0; i < _num_target_groups; i++)
    {
        t_group.vol = vols[i];
        _target_groups.push_back(t_group);
    }

    return;
}

void TrajectoryLibrary::generateTargets()
{
    for (int i = 0; i < _num_target_groups; i++)
    {
        target_group & t_group = _target_groups[i];
        ROS_INFO("Generating joint value targets for group %d.", i);
        if (t_group.vol.type == GRID_RECT)
        {
            t_group.target_count = rectLinspace(t_group.jvals, t_group.vol.grid);
        }
        if (t_group.vol.type == GRID_SPHERE)
        {
        }
        ROS_INFO("Generated %d targets.", t_group.target_count);
    }
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

    moveit::core::robotStateToRobotStateMsg(_plan_scene->getCurrentState(), req.start_state);

    // Add constraints
    req.goal_constraints = constraints;
    //req.planner_id = "manipulator[LBKPIECEkConfigDefault]";
    req.planner_id = "manipulator[RRTConnectkConfigDefault]";
    //req.planner_id = "manipulator[RRTstarkConfigDefault]";
    req.allowed_planning_time = 20.0;

    req.num_planning_attempts = 3;


    // Define workspace
    req.workspace_parameters.header.frame_id = "world";
    req.workspace_parameters.max_corner.x = 0.38;
    req.workspace_parameters.max_corner.y = 0.38;
    req.workspace_parameters.max_corner.z = 1.0;
    req.workspace_parameters.min_corner.x = -0.38;
    req.workspace_parameters.min_corner.y = -0.38;
    req.workspace_parameters.min_corner.z = 0.05;

    // Now prepare the planning context
    int tries = 0;
    while (tries < MAX_PLANNER_ATTEMPTS)
    {
        _planning_pipeline->generatePlan(_plan_scene, req, res);
        if (res.error_code_.val == res.error_code_.SUCCESS)
        {
            robot_trajectory::RobotTrajectoryPtr traj(res.trajectory_);

            std::vector<std::size_t> invalid_index;
            if (!_plan_scene->isPathValid(*traj, UR5_GROUP_NAME, true, &invalid_index))
            {
                ROS_ERROR("Path invalid.");
                for (int i = 0; i < invalid_index.size(); i++)
                {
                    ROS_ERROR("Invalid index %d", (int) invalid_index[i]);
                }
                tries++;
                continue;
            }

            for (int n = 0; n < traj->getWayPointCount(); n++)
            {
                robot_state::RobotState wpt = traj->getWayPoint(n);
                if (!wpt.hasVelocities())
                {
                    ROS_ERROR("Waypoint %d does not have velocities.", n);
                }
            }

            // Do optimization
            robot_trajectory::RobotTrajectoryPtr traj_opt(new robot_trajectory::RobotTrajectory(_rmodel, UR5_GROUP_NAME));
            optimizeTrajectory(traj_opt, traj);

            // Do time parameterization on optimized trajectory
            _time_parametizer->computeTimeStamps(*traj_opt);

            // Now generate velocities
            computeVelocities(traj);

            // Check validity one last time
            if (!_plan_scene->isPathValid(*traj_opt, UR5_GROUP_NAME, true, &invalid_index))
            {
                ROS_ERROR("Post-processed path invalid.");
                for (int i = 0; i < invalid_index.size(); i++)
                {
                    ROS_ERROR("Invalid index %d", (int) invalid_index[i]);
                }
                tries++;
                continue;
            }

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

bool TrajectoryLibrary::segmentValid(const robot_state::RobotState &start, const robot_state::RobotState &end, int res)
{
    robot_state::RobotState inter_state(_rmodel);
    double t = 0;
    double dt = 1.0/res;
    for (int i=0; i < res; i++)
    {
        t += dt;
        start.interpolate(end, t, inter_state);
        inter_state.update(true);
        if (!_plan_scene->isStateValid(inter_state, UR5_GROUP_NAME))
        {
            return false;
        }
    }
    return true;
}

bool TrajectoryLibrary::pathValid(const robot_trajectory::RobotTrajectoryPtr traj, int res)
{
    robot_state::RobotStateConstPtr seg_start;
    robot_state::RobotStateConstPtr seg_end;
    for (int i=1; i < traj->getWayPointCount(); i++)
    {
        seg_start = traj->getWayPointPtr(i-1);
        seg_end = traj->getWayPointPtr(i);
        if (!segmentValid(*seg_start, *seg_end, res))
        {
            return false;
        }
    }
    return true;
}

void TrajectoryLibrary::optimizeTrajectory(robot_trajectory::RobotTrajectoryPtr traj_opt, const robot_trajectory::RobotTrajectoryPtr traj)
{
    // Make a copy
    *traj_opt = *traj;
//  return;

    std::size_t wpt_count = traj->getWayPointCount();
    ROS_INFO("Optimize starts with %d waypoints.", (int) wpt_count);

    robot_trajectory::RobotTrajectory shortcut(_rmodel, UR5_GROUP_NAME);

    // Iterate forwards from the first waypoint
    for (std::size_t i=0; i < wpt_count; i++)
    {
        // Get shortcut start waypoint
        robot_state::RobotState wpt_i = traj_opt->getWayPoint(i);

        // Starting from the end and going backwards
        for (std::size_t j=(wpt_count-1); j > (i+1); j--)
        {
            // Get shortcut end waypoint
            robot_state::RobotState wpt_j = traj_opt->getWayPoint(j);
            if (segmentValid(wpt_i, wpt_j, PATH_VALIDITY_CHECKER_RES))
            {
                ROS_INFO("Shortcut found between nodes %d and %d.", (int) i, (int) j);
                // Create new trajectory object with only neccessary endpoints
                // Copy in waypoints before the shortcut
                shortcut.clear();
                for (std::size_t n=0; n <= i; n++)
                {
                    robot_state::RobotStatePtr state = traj_opt->getWayPointPtr(n);
                    shortcut.insertWayPoint(n, state, 0);
                }
                // Copy in waypoints after the shortcut
                for (std::size_t n=j; n < wpt_count; n++)
                {
                    robot_state::RobotStatePtr state = traj_opt->getWayPointPtr(n);
                    shortcut.insertWayPoint(n - (j-i-1), state, 0);
                }
                ROS_ASSERT(shortcut.getWayPointCount() == (wpt_count - (j-i-1)));

                // Copy temporary trajectory
                *traj_opt = shortcut;
                wpt_count = traj_opt->getWayPointCount();
                break; // break out of j loop
            }

            // Otherwise shortcut is invalid, so move on
        }
    }

    ROS_INFO("Successfully trimmed %d nodes.", (int) (traj->getWayPointCount() - traj_opt->getWayPointCount()) );
    return;
}

void TrajectoryLibrary::computeVelocities(robot_trajectory::RobotTrajectoryPtr traj)
{
    std::size_t num_wpts = traj->getWayPointCount();

    for (int i=1; i < num_wpts; i++)
    {
        robot_state::RobotStatePtr wpt = traj->getWayPointPtr(i);
        robot_state::RobotStatePtr wpt_prev = traj->getWayPointPtr(i-1);

        double duration = traj->getWayPointDurationFromPrevious(i);

        // Iterate through joints
        std::size_t num_joints = wpt->getVariableCount();
        for (int j=0; j < num_joints; j++)
        {
            // Calculate joint movement distance
            double joint_dist = wpt->getVariablePosition(j) - wpt_prev->getVariablePosition(j);
            // Calculate joint velocity given desired duration
            double vel = joint_dist / duration;
            // Set joint velocity for waypoint
            // ROS_INFO("Waypoint %d joint %d with duration %f has velocity %f.", i, j, duration, wpt->getVariableVelocity(j));
            wpt->setVariableVelocity(j, vel);
            // ROS_INFO("Assigned waypoint %d joint %d with duration %f a velocity of %f.", i, j, duration, vel);
        }
    }

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

double TrajectoryLibrary::calculateGradients(double* gradient_array, robot_trajectory::RobotTrajectoryPtr traj)
{
    int num_wpts = traj->getWayPointCount();
    int num_joints = _rmodel->getVariableCount();
    double delta = 0.001;
    double max = 0;

    double duration_old = traj->getWaypointDurationFromStart(num_wpts-1);
    double duration_new;

    robot_state::RobotStatePtr wpt;
    double* gtstate;
    // Loop through waypoints
    for (int i=0; i < num_wpts; i++)
    {
        // Get waypoint
        wpt = traj->getWayPointPtr(i);
        // Get joint position pointer
        gtstate = wpt->getVariablePositions();
        // Loop through joints
        for (int j=0; j < num_joints; j++)
        {
            // Perturb joint j by delta
            gtstate[j] += delta;
            wpt->update(true);

            // Calculate new path duration
            _time_parametizer->computeTimeStamps(*traj);
            duration_new = traj->getWaypointDurationFromStart(num_wpts-1);

            // Now calculate gradient
            double gradient = (duration_new - duration_old) / delta;
            if (max < abs(gradient))
            {
                max = abs(gradient);
            }
            printf("Calculated gradient with old dur %f and new dur %f to be %f.\n", duration_old, duration_new, gradient);
            *(gradient_array + (i*num_joints) + j) = gradient;

            // Now reset joint j
            gtstate[j] -= delta;
        }
    }

    return max;
}

bool TrajectoryLibrary::gradientDescentWarp(ur5_motion_plan &plan, const joint_values_t &jvals_start, const joint_values_t &jvals_end)
{
    // Intialize RobotTrajectory object
    robot_trajectory::RobotTrajectoryPtr traj(new robot_trajectory::RobotTrajectory(_rmodel, UR5_GROUP_NAME));
    robot_state::RobotState start_state(_rmodel);
    moveit::core::robotStateMsgToRobotState(plan.start_state, start_state);
    traj->setRobotTrajectoryMsg(start_state, plan.trajectory);

    // Collect useful data
    int num_wpts = traj->getWayPointCount();
    int num_joints = _rmodel->getVariableCount();

    // Now replace start and end points to target start and end
    robot_state::RobotStatePtr wpt_start = traj->getFirstWayPointPtr();
    robot_state::RobotStatePtr wpt_end = traj->getLastWayPointPtr();
    wpt_start->setVariablePositions(jvals_start);
    wpt_start->update(true);
    wpt_end->setVariablePositions(jvals_end);
    wpt_end->update(true);

    // If path invalid
    if (!pathValid(traj, PATH_VALIDITY_CHECKER_RES))
    {
        ROS_WARN("Gradient descent failed.");
        return false;
    }

    // Now do gradient descent to smooth the rest of the path
    robot_trajectory::RobotTrajectoryPtr traj_temp(new robot_trajectory::RobotTrajectory(*traj));
    double* gradient_field = new double[num_wpts * num_joints];
    robot_state::RobotStatePtr wpt;
    double* gtstate;

    // Compute path duration
    _time_parametizer->computeTimeStamps(*traj_temp);
    double old_duration;
    double new_duration = traj_temp->getWaypointDurationFromStart(num_wpts-1);

    do
    {
        std::cout << "GDW cycle.\n";

        // Update previous duration
        old_duration = new_duration;

        // Calculate gradients
        double grad_max = calculateGradients(gradient_field, traj_temp);
        printf("Max gradient is %f.\n", grad_max);

        // For each waypoint besides the start and end
        for (int i=1; i < num_wpts-1; i++)
        {
            wpt = traj_temp->getWayPointPtr(i);
            gtstate = wpt->getVariablePositions();

            // For each joint
            for (int j=0; j < num_joints; j++)
            {
                // Move joint according to negative gradient
                double gradient = gradient_field[ (i*num_joints) + j ];
                double move = -(gradient / grad_max) * 0.001;
                printf("Gradient %f. Moving wpt %d joint %d by %f.\n", gradient, i, j, move);
                gtstate[j] += move;
            }
            wpt->update(true);
        }

        // Make sure path is valid
        if (!pathValid(traj_temp, PATH_VALIDITY_CHECKER_RES))
        {
            ROS_ERROR("Made invalid path in GDW.");
            break;
        }

        // Compute path duration
        _time_parametizer->computeTimeStamps(*traj_temp);
        new_duration = traj_temp->getWaypointDurationFromStart(num_wpts-1);

        // Update trajectory
        *traj = *traj_temp;

        ROS_INFO("Improved duration by %f sec.", old_duration - new_duration);

    } while (old_duration - new_duration > 0.01);

    std::cout << "Broke out of GDW loop.\n";

    delete[] gradient_field;

    // Now we have smoothed and fitted path
    // timeWarpTrajectory(traj, 3);
    computeVelocities(traj);
    traj->getRobotTrajectoryMsg(plan.trajectory);
    plan.duration = new_duration;
    plan.num_wpts = num_wpts;
    moveit::core::robotStateToRobotStateMsg(traj->getFirstWayPoint(), plan.start_state);
    moveit::core::robotStateToRobotStateMsg(traj->getLastWayPoint(), plan.end_state);

    return true;
}

bool TrajectoryLibrary::fitPlan(ur5_motion_plan& plan, const joint_values_t &start_jvals, const joint_values_t &end_jvals)
{
    // First find similar plan in database
    _kdtree->setTargets(start_jvals, end_jvals);
    bool success;
    bool lookup_success;
    int proximity_index = 0;
    do
    {
        std::cout << "Looking up plan at position " << proximity_index << " in priority queue." << std::endl;
        lookup_success = _kdtree->lookup(plan, proximity_index);
        if (!lookup_success)
        {
            ROS_ERROR("All plans failed.");
            return false;
        }
        success = gradientDescentWarp(plan, start_jvals, end_jvals);
        ++proximity_index;
    } while (!success);

    std::cout << "Found plan.\n";
    return true;
}

void TrajectoryLibrary::build()
{
    /* Check that target groups have been generated */
    if (_num_target_groups == 0)
    {
        ROS_ERROR("No target groups defined.");
        return;
    }

    /* Iterate through target groups for trajectory start location */
    for (int i = 0; i < _num_target_groups; i++)
    {
        ROS_INFO("START GROUP: %d", i);

        /* Iterate through target groups again for end location */
        for (int j = 0; j < _num_target_groups; j++)
        {
            if (j == i)
            {
                if (!_target_groups[i].vol.allow_internal_paths)
                {
                    // We don't want to generate paths between targets in the same group
                    continue;
                }
                else
                {
                    ROS_INFO("INTERNAL");
                }
            }
            else
            {
                ROS_INFO("END GROUP: %d", j);
            }

            // APPLE: If we are moving from place target to pick target, we need to attach an apple
//            moveit_msgs::AttachedCollisionObject aco_msg;
//            if (i == PLACE_TARGET && j == PICK_TARGET)
//            {
//                aco_msg = getAppleObjectMsg();
//            }
//            else
//            {
//                aco_msg = getAppleObjectMsg();
//                aco_msg.object.operation = aco_msg.object.REMOVE;
//            }
//            _plan_scene->processAttachedCollisionObjectMsg(getAppleObjectMsg());

            /* Pick particular start target */
            for (int n = 0; n < _target_groups[i].target_count; n++)
            {
                ROS_INFO("START TARGET: %d", n);

                // Construct trajectory start state
                robot_state::RobotState start_state(_rmodel);
                start_state.setJointGroupPositions(UR5_GROUP_NAME, _target_groups[i].jvals[n]);

                // Update planning scene with start state
                _plan_scene->setCurrentState(start_state);
                // Publish planning scene
                moveit_msgs::PlanningScene scene_msg;
                _plan_scene->getPlanningSceneMsg(scene_msg);
                _plan_scene_publisher.publish(scene_msg);

                /* Now pick particular end target */
                for (int m = 0; m < _target_groups[j].target_count; m++)
                {
                    if (i == j && n == m)
                    {
                        // The start and end targets are the same.
                        continue;
                    }
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

                    // Publish trajectory
                    moveit_msgs::DisplayTrajectory display_trajectory;
                    display_trajectory.trajectory.push_back(plan.trajectory);
                    display_trajectory.trajectory_start = plan.start_state;
                    _trajectory_publisher.publish(display_trajectory);

                    // Now record start and stop locations
                    plan.start_target_index = n;
                    plan.end_target_index = m;

                    // Store trajectory in KD tree
                    try { _kdtree->add(plan); }
                    catch (std::string& s)
                    {
                        ROS_ERROR("KDTree::add() exception: %s.", s.c_str());
                    }
                }
            }
        }
    }

    _kdtree->printInfo(std::cout);

    return;
}

void TrajectoryLibrary::demo()
{
    // Initialize trajectory manager
    _execution_manager.reset(new trajectory_execution_manager::TrajectoryExecutionManager(_rmodel));
    if (!_execution_manager->ensureActiveControllersForGroup(UR5_GROUP_NAME))
    {
        ROS_ERROR("Controllers for joint group are not active.");
    }

    robot_state::RobotState start_state(_rmodel);
    robot_state::RobotState end_state(_rmodel);

    ROS_INFO("Generating start target.");
    // Generate last end target
    joint_values_t start_jvals;
    joint_values_t end_jvals;
    //////////////////////
    /// This is specific to WEEDING with single target group
    generateRandomJointTarget(end_jvals, _target_groups[0].vol);
    //////////////////////
    end_state.setJointGroupPositions(_jmg, end_jvals);

    while (1)
    {
        // Update start state
        start_state = end_state;

        // Generate end target
        generateRandomJointTarget(end_jvals, _target_groups[0].vol);
        end_state.setJointGroupPositions(_jmg, end_jvals);
        end_state.update(true);

        // Extract start state joint values
        start_state.copyJointGroupPositions(_jmg, start_jvals);

        // Computation time
        boost::posix_time::ptime compute_time_start = boost::posix_time::microsec_clock::universal_time();

        ROS_INFO("Running gradient fit.");
        ur5_motion_plan plan;
        bool success = fitPlan(plan, start_jvals, end_jvals);
        if (!success)
        {
            ROS_ERROR("Plan fitting failed. Trying new target.");
            continue;
        }

        // Computation time
        boost::posix_time::ptime compute_time_end = boost::posix_time::microsec_clock::universal_time();
        ROS_INFO("KD lookup time: = %d usec.", (int) (compute_time_end - compute_time_start).total_microseconds());

        // Get RobotTrajectory object from msg
        robot_trajectory::RobotTrajectoryPtr traj(new robot_trajectory::RobotTrajectory(_rmodel, UR5_GROUP_NAME));
        traj->setRobotTrajectoryMsg(start_state, plan.trajectory);

        // Make sure path is valid
        if (!pathValid(traj, PATH_VALIDITY_CHECKER_RES))
        {
            ROS_ERROR("Path invalid. Skipping.");
            continue;
        }

        ////////////// Now demo the trajectory
        // Set current start state
        _plan_scene->setCurrentState(start_state);

        // Publish planning scene and end state as separate messages
        moveit_msgs::PlanningScene scene_msg;
        _plan_scene->getPlanningSceneMsg(scene_msg);
        _plan_scene_publisher.publish(scene_msg);

        moveit_msgs::DisplayRobotState state_msg;
        state_msg.state = plan.end_state;
        _robot_state_publisher.publish(state_msg);

        // Get trajectory data
        int num_wpts = plan.num_wpts;
        double duration = traj->getWaypointDurationFromStart(num_wpts-1);
        if (duration - plan.duration > 0.05)
        {
            ROS_ERROR("Duration fields do not match. %f - %f = %f.", duration, plan.duration, duration - plan.duration);
        }
        ROS_INFO("Trajectory has %d nodes and takes %f seconds.", num_wpts, plan.duration);

        // Publish trajectory
        moveit_msgs::DisplayTrajectory display_trajectory;
        display_trajectory.trajectory_start = plan.start_state;
        display_trajectory.trajectory.push_back(plan.trajectory);
        _trajectory_publisher.publish(display_trajectory);

        ROS_INFO("Starting from joint values: ");
        printJointValues(start_jvals);
        ROS_INFO("Press enter when ready.");
        while (std::cin.get() != '\n');

        // Execute trajectory
        _execution_manager->pushAndExecute(plan.trajectory);

        // Now delay for duration of trajectory
        ROS_INFO("Waiting for execution.");
        ros::WallDuration sleep_time(plan.duration);
        sleep_time.sleep();
        ROS_INFO("Done.");
        // _execution_manager->waitForExecution();
    }

    return;
}

bool TrajectoryLibrary::filewrite(const std::vector<ur5_motion_plan>& plans, const char* filename, bool debug = false)
{
    int plan_count = plans.size();
    int node_count;

    std::ofstream file;
    file.open (filename, std::ofstream::out | std::ofstream::binary);

    // If file didn't open correctly
    if (!file.is_open())
    {
        return false;
    }

    // First write plan_group meta info
    file.write((char *)(&plan_count),sizeof(plan_count));                   // number of plans

    if (debug == 1) ROS_INFO("%d",plan_count);
    for (size_t n = 0; n < plan_count; n++)
    {
        const ur5_motion_plan& plan = plans[n];

        //RobotTrajectory -> JointTrajectory -> JointTrajectoryPoints
        node_count = plan.trajectory.joint_trajectory.points.size();
        ROS_INFO("%d",node_count);
        file.write((char *)(&node_count),sizeof(node_count));
        for (size_t idx = 0; idx < node_count; idx++)
        {
            for (size_t i=0; i < 6; i++)
            {
                file.write((char *)(&plan.trajectory.joint_trajectory.points[idx].positions[i]),sizeof(double));
                file.write((char *)(&plan.trajectory.joint_trajectory.points[idx].velocities[i]),sizeof(double));
            }
            file.write((char *)(&plan.trajectory.joint_trajectory.points[idx].time_from_start),sizeof(ros::Duration));
        }

        //RobotTrajectory -> JointTrajectory -> Header
        file.write((char *)(&plan.trajectory.joint_trajectory.header.seq),sizeof(uint32_t));
        file.write((char *)(&plan.trajectory.joint_trajectory.header.stamp),sizeof(ros::Time));
        file << plan.trajectory.joint_trajectory.header.frame_id << '\n';

        //RobotTrajectory -> JointTrajectory -> joint_names
        for (size_t i=0; i < 6; i++) file << plan.trajectory.joint_trajectory.joint_names[i] << '\n';


        //start_state
        //RobotState -> JointState -> Header
        file.write((char *)(&plan.start_state.joint_state.header.seq),sizeof(uint32_t));
        file.write((char *)(&plan.start_state.joint_state.header.stamp),sizeof(ros::Time));
        file << plan.start_state.joint_state.header.frame_id << '\n';
        //RobotState -> JointState -> string & position
        for (size_t j = 0; j < 6; j++)
        {
            file << plan.start_state.joint_state.name[j] << '\n';
            file.write((char *)(&plan.start_state.joint_state.position[j]),sizeof(double));
        }

        //end_state
        //RobotState -> JointState -> Header
        file.write((char *)(&plan.end_state.joint_state.header.seq),sizeof(uint32_t));
        file.write((char *)(&plan.end_state.joint_state.header.stamp),sizeof(ros::Time));
        file << plan.end_state.joint_state.header.frame_id << '\n';
        //RobotState -> JointState -> stirng & position
        for (size_t j = 0; j < 6; j++)
        {
            file << plan.end_state.joint_state.name[j] << '\n';
            file.write((char *)(&plan.end_state.joint_state.position[j]),sizeof(double));
        }

        //index
        file.write((char *)(&plan.start_target_index),sizeof(unsigned int));
        file.write((char *)(&plan.end_target_index),sizeof(unsigned int));

        //duration
        file.write((char *) &plan.duration, sizeof(ur5_motion_plan::duration));

        //write an apple
        if (plan.start_state.attached_collision_objects.size() > 0)
        {
            file << "apple\n";
        }
        else
        {
            file << "no apple\n";
        }
    }

    file.close();

    return true;
}

bool TrajectoryLibrary::fileread(std::vector<ur5_motion_plan>& plans, const char* filename, bool debug = false)
{
    int wpt_count;
    int plan_count;

    //int nodeset = 10;
    double temp_double;
    std::string temp_string;
    std::string blank_string;
    ros::Duration temp_duration;

    ur5_motion_plan temp_plan;
    ur5_motion_plan empty;
    std::vector<int> temp;
    trajectory_msgs::JointTrajectoryPoint temp_points;
    trajectory_msgs::JointTrajectoryPoint blank;

    std::ifstream info;
    info.open(filename,std::ifstream::in | std::ofstream::binary);

    if(!info.is_open())
    {
        return false;
    }

    // First read meta data
    info.read((char*)(&plan_count), sizeof(plan_count));                    // Number of plans
    plans.reserve(plan_count);

    if (debug == 1) ROS_INFO("%d",plan_count);
    for (size_t n = 0; n < plan_count; n++)
    {
        // Iterate through waypoints
        info.read((char*)(&wpt_count),sizeof(wpt_count));
        temp.push_back(wpt_count);
        if (debug == 1) ROS_INFO("%d",wpt_count);
        for (size_t wpt_idx = 0; wpt_idx < wpt_count; wpt_idx++)
        {
            for (size_t i=0; i<6; i++)
            {
                info.read((char *)(&temp_double),sizeof(temp_double));
                temp_points.positions.push_back(temp_double);
                info.read((char *)(&temp_double),sizeof(temp_double));
                temp_points.velocities.push_back(temp_double);
            }
            info.read((char *)(&temp_duration),sizeof(temp_duration));
            //ros::Duration d(z);
            temp_points.time_from_start = temp_duration;
            temp_plan.trajectory.joint_trajectory.points.push_back(temp_points);
            temp_points = blank;
        }

        //RobotTrajectory -> JointTrajectory -> Header
        info.read((char *)(&temp_plan.trajectory.joint_trajectory.header.seq),sizeof(uint32_t));
        info.read((char *)(&temp_plan.trajectory.joint_trajectory.header.stamp),sizeof(ros::Time));
        getline (info,temp_plan.trajectory.joint_trajectory.header.frame_id);

        //RobotTrajectory -> JointTrajectory -> joint_names
        for (size_t i=0; i < 6; i++)
        {
            getline(info,temp_string);
            temp_plan.trajectory.joint_trajectory.joint_names.push_back(temp_string);
            temp_string = blank_string;
        }

        //start_state
        //RobotState -> JointState -> Header
        info.read((char *)(&temp_plan.start_state.joint_state.header.seq),sizeof(uint32_t));
        info.read((char *)(&temp_plan.start_state.joint_state.header.stamp),sizeof(ros::Time));
        getline(info, temp_plan.start_state.joint_state.header.frame_id);

        //RobotState -> JointState -> stirng & position
        for (size_t j = 0; j < 6; j++)
        {
            getline(info, temp_string);
            temp_plan.start_state.joint_state.name.push_back(temp_string);
            temp_string = blank_string;

            info.read((char *)(&temp_double),sizeof(temp_double));
            temp_plan.start_state.joint_state.position.push_back(temp_double);
        }

        //end_state
        //RobotState -> JointState -> Header
        info.read((char *)(&temp_plan.end_state.joint_state.header.seq),sizeof(uint32_t));
        info.read((char *)(&temp_plan.end_state.joint_state.header.stamp),sizeof(ros::Time));
        getline (info,temp_plan.end_state.joint_state.header.frame_id);

        //RobotState -> JointState -> stirng & position
        for (size_t j = 0; j < 6; j++)
        {
            getline(info,temp_string);
            temp_plan.end_state.joint_state.name.push_back(temp_string);
            temp_string = blank_string;

            info.read((char *)(&temp_double),sizeof(temp_double));
            temp_plan.end_state.joint_state.position.push_back(temp_double);

        }

        //Index
        info.read((char *)(&temp_plan.start_target_index),sizeof(unsigned int));
        info.read((char *)(&temp_plan.end_target_index),sizeof(unsigned int));

        // Duration
        info.read((char*) &(temp_plan.duration), sizeof(ur5_motion_plan::duration));

        // Now check for an apple
        /* Define the attached object message*/
        getline(info,temp_string);
        if (debug) { std::cout << temp_string << '\n'; }
        if (temp_string == "apple")
        {
            std::cout << "Adding apple for this trajectory.\n";
            temp_plan.start_state.attached_collision_objects.push_back(getAppleObjectMsg());
            temp_plan.end_state.attached_collision_objects.push_back(getAppleObjectMsg());
        }
        else
        {
            ROS_ASSERT(temp_string == "no apple");
        }

        // Other parameters
        temp_plan.num_wpts = wpt_count;

        plans.push_back(temp_plan);
        temp_plan = empty;
    }

    info.close();

    return true;

}

void TrajectoryLibrary::exportToFile(const char* filename)
{
    // SAVE DATA TO .dat FILE
    ROS_INFO("--------------SAVING!!!!-------------------");
    if ( filewrite(_kdtree->getPlanData(), filename, false) )
    {
        ROS_INFO("Trajectories written to file.");
    }
    else
    {
        ROS_ERROR("Trajectories not saved to file.");
    }
    return;
}

void TrajectoryLibrary::importFromFile(const char *filename)
{
    //LOAD DATA FROM .dat FILE
    ROS_INFO("--------------LOADING!!!!-------------------");
    std::vector<ur5_motion_plan> plans;
    if ( fileread(plans, filename, false) )
    {
        for (int i=0; i < plans.size(); i++)
        {
            try { _kdtree->add(plans[i]); }
            catch (std::string& s)
            {
                ROS_ERROR("Exception: %s.", s.c_str());
            }
        }
    }
    else
    {
        ROS_ERROR("File import failed.");
    }

    _kdtree->printInfo(std::cout);
    return;
}

 moveit_msgs::AttachedCollisionObject TrajectoryLibrary::getAppleObjectMsg()
{
    moveit_msgs::AttachedCollisionObject apple;
    apple.link_name = "ee_link";
    apple.object.header.frame_id = "ee_link";
    apple.object.id = "apple";

    geometry_msgs::Pose pose;
    shape_msgs::SolidPrimitive primitive;
    pose.position.x = 0.05;
    primitive.type = primitive.SPHERE;
    primitive.dimensions.resize(1);
    primitive.dimensions[0] = 0.05;

    apple.object.primitives.push_back(primitive);
    apple.object.primitive_poses.push_back(pose);
    apple.object.operation = apple.object.ADD;
    /* Publish and sleep (to view the visualized results)*/

    return apple;
}

 void TrajectoryLibrary::generateRandomJointTarget(joint_values_t& jvals, const target_volume& vol)
 {
     if (vol.type == GRID_RECT)
     {
         double dx = (vol.grid.xlim_high - vol.grid.xlim_low) / 1000.0;
         double dy = (vol.grid.ylim_high - vol.grid.ylim_low) / 1000.0;
         double dz = (vol.grid.zlim_high - vol.grid.zlim_low) / 1000.0;

         bool success = false;
         std::vector<joint_values_t> solutions;
         while (!success)
         {
             geometry_msgs::Pose pose;
             pose.orientation = vol.grid.orientation;
             pose.position.x = (dx * (rand() % 1000)) + vol.grid.xlim_low;
             pose.position.y = (dy * (rand() % 1000)) + vol.grid.ylim_low;
             pose.position.z = (dz * (rand() % 1000)) + vol.grid.zlim_low;

             success = doIK(solutions, pose);
         }

         jvals = joint_values_t(solutions[0]);

     }

     else
     {
         ROS_ERROR("Bad target volume type. Generate random joint target failed.");
     }

     return;
 }
