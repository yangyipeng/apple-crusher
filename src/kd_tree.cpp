#include "kd_tree.h"

//////////////// Cell Class definitions

Cell::Cell(const std::vector<std::size_t> & coords)
{
    _coords = coords;
    _dimension = _coords.size();
    return;
}

bool operator== (const Cell& lhs, const Cell& rhs)
{
    if (lhs._dimension != rhs._dimension)
    {
        return false;
    }

    for (int i=0; i < lhs._dimension; i++)
    {
        if (lhs._coords[i] != rhs._coords[i])
        {
            return false;
        }
    }
    return true;
}

bool operator!= (const Cell& lhs, const Cell& rhs)
{
    return !(lhs == rhs);
}

std::size_t Cell::rectDistFrom(const cell_coords_t& coords)
{
    if (coords.size() != _dimension)
    {
        throw "Dimension mismatch.";
        return 0;
    }

    std::size_t max_dist = 0;
    for (int i=0; i < _dimension; i++)
    {
        std::size_t dist = abs(coords[i] - _coords[i]);
        if (dist > max_dist)
        {
            max_dist = dist;
        }
    }

    return max_dist;
}

////////////////// KDTree Class definitions

KDTree::KDTree(robot_model::RobotModelPtr& rmodel, const std::vector<double>& low_bounds, const std::vector<double>& high_bounds, const std::vector<std::size_t>& resolution)
{
    // Store pointer to robot model
    _rmodel = rmodel;

    // Extract dimension from model
    // Represent start and end joint-space positions as single point (dim = 2*numJoints)
    _dimension = 2 * _rmodel->getVariableCount();
    if (low_bounds.size() != _dimension)
    {
        throw std::string("Dimension mismatch.");
    }
    if (high_bounds.size() != _dimension)
    {
        throw std::string("Dimension mismatch.");
    }
    if (resolution.size() != _dimension)
    {
        throw std::string("Dimension mismatch.");
    }

    _bounds_low = low_bounds;
    _bounds_high = high_bounds;
    _resolution = resolution;

    // Calculate index increments
    for (int i = 0; i < _dimension; i++)
    {
        _cell_increments.push_back( (_bounds_high[i] - _bounds_low[i]) / _resolution[i] );
    }

    _plan_count = 0;
    _cell_count = 0;

    return;
}

void KDTree::printInfo(std::ostream &cout)
{
    cout << "KDTree: " << std::endl;
    cout << "  Dimensions: " << _dimension << std::endl;
    cout << "  Low bounds: ";
    for (int i=0; i < _dimension; i++)
    {
        cout << _bounds_low[i] << ' ';
    }
    cout << "\n  High bounds: ";
    for (int i=0; i < _dimension; i++)
    {
        cout << _bounds_high[i] << ' ';
    }
    cout << std::endl;
    cout << "  Number of plans: " << _plan_count << std::endl;
    cout << "  Number of populated cells: " << _cell_count << std::endl;
    return;
}

cell_coords_t KDTree::calcCoords(const joint_values_t &jvals)
{
    cell_coords_t coords;
    if (jvals.size() > _dimension)
    {
        throw "Cannot calculate cell coordinates of joint value list with size higher than _dimension.";
    }

    for (int i = 0; i < jvals.size(); i++)
    {
        double f_index = floor( (jvals[i] - _bounds_low[i]) / _cell_increments[i] );
        coords.push_back((coord_t) f_index);
    }
    return coords;
}

void KDTree::searchCellsAtNextDistance()
{
    /* Build plan search pool */
    std::vector<std::size_t> pool;

    // Increment latest search depth counter
    _search_depth++;
    // std::cout << "Expanding search to level " << _search_depth << std::endl;

    // Search through our cell map for cells at the correct distance from target
    for (int c=0; c < _cell_count; c++)
    {
        if (_cells[c].rectDistFrom(_target_coords) == _search_depth)
        {
            std::vector<std::size_t> plan_indices = _cells[c].getValues();
            for (int i=0; i < plan_indices.size(); i++)
            {
                pool.push_back(plan_indices[i]);
            }
        }
    }

    // Now sort the pool
    linearSort(pool);
}

void KDTree::add(const ur5_motion_plan & plan)
{
    // First make sure plan start state and end state are within range
    for (int i=0; i < (_dimension/2); i++)
    {
        if (plan.start_state.joint_state.position[i] < _bounds_low[i] || plan.start_state.joint_state.position[i] > _bounds_high[i])
        {
            throw std::string("Plan start state out of bounds. Cannot add plan to KDTree.");
        }
        if (plan.end_state.joint_state.position[i] < _bounds_low[i + (_dimension/2)] || plan.end_state.joint_state.position[i] > _bounds_high[i + (_dimension/2)])
        {
            throw std::string("Plan end state out of bounds. Cannot add plan to KDTree.");
        }
    }

    // Add plan to data vector
    std::size_t plan_num = _plan_count;
    _plans.push_back(plan);
    _plan_count++;

    // Combine joint values for start and end states into single vector
    std::vector<double> jvals;
    jvals.reserve(2 * plan.start_state.joint_state.position.size());

    jvals = plan.start_state.joint_state.position;
    for (int i=0; i < plan.end_state.joint_state.position.size(); i++)
    {
        jvals.push_back(plan.end_state.joint_state.position[i]);
    }

    // Calculate cell coordinates
    cell_coords_t coords = calcCoords(jvals);

    // Search for cell in our vector
    bool found = false;
    for (std::size_t i = 0; i < _cell_count; i++)
    {
        if (_cells[i].getCoords() == coords)
        {
            found = true;
            _cells[i].addValue(plan_num);
        }
    }
    // If not found
    if (!found)
    {
        // Create new cell
        Cell cell(coords);
        cell.addValue(plan_num);
        _cells.push_back(cell);
        _cell_count++;
    }
    return;
}

const ur5_motion_plan& KDTree::getRandomPlan()
{
    // Get random plan
    std::size_t plan_idx = rand() % _plans.size();

    return _plans[plan_idx];
}

const ur5_motion_plan& KDTree::getRandomPlanStartingNear(const moveit_msgs::RobotState& start_state, double dist_max)
{
    // First calculate cell subspace with this starting location
    cell_coords_t start_coords = calcCoords(start_state.joint_state.position);
    // This only includes the start state -- i.e. we have a 6 dimensional subspace to search still

    robot_state::RobotState state(_rmodel);
    robot_state::RobotState comp_state(_rmodel);
    moveit::core::robotStateMsgToRobotState(start_state, comp_state);

    // Now search for cells in this subspace
    for (int j=0; j < _cell_count; j++)
    {
        cell_coords_t cell_start_coords = _cells[j].getCoords();
        cell_start_coords.resize(start_coords.size());

        if (cell_start_coords == start_coords)
        {
            const std::vector<std::size_t>& values = _cells[j].getValues();
            for (int i = 0; i < values.size(); i++)
            {
                const ur5_motion_plan& plan = _plans[ values[i] ];
                moveit::core::robotStateMsgToRobotState(plan.start_state, state);
                double dist = comp_state.distance(state);

                if (dist < dist_max)
                {
                    std::cout << "Found plan distance " << dist << " away.\n";
                    return plan;
                }
            }
        }
    }

    throw "Could not find plan nearby.";
}

void KDTree::setTargets(const joint_values_t &start_jvals, const joint_values_t &end_jvals)
{
    _target_point.reserve(2 * start_jvals.size());
    _target_point = start_jvals;
    for (int i=0; i < end_jvals.size(); i++)
    {
        _target_point.push_back(end_jvals[i]);
    }

    _target_coords = calcCoords(_target_point);

    // Now reset proximity list
    _proximity_ordering.clear();
    _search_depth = 0;

    // Start promiximity ordering by searching plans in target cell
    for (std::size_t i=0; i < _cell_count; i++)
    {
        if (_cells[i].getCoords() == _target_coords)
        {
            int num_plans = _cells[i].getValues().size();
            std::cout << "Coords match. Cell has " << num_plans << " plans." << std::endl;
            if (num_plans > 0)
            {
                linearSort(_cells[i].getValues());
            }
            break;
        }
    }

    return;
}

bool KDTree::lookup(ur5_motion_plan& plan, int hit)
{
    while (1)
    {
        if (hit >= _plan_count)
        {
            return false;
        }
        // Check if we have already found a plan at this hit number
        if (hit < _proximity_ordering.size())
        {
            plan = _plans[ _proximity_ordering[hit] ];
            // Todo: Replace with actual distance value
            return true;
        }

        // Otherwise we need to expand our search
        searchCellsAtNextDistance();
    }
}

void KDTree::linearSort(const std::vector<std::size_t>& plan_pool)
{
    int pool_size = plan_pool.size();

    // std::cout << "Performing linear sort on " << pool_size << " plans." << std::endl;

    if (pool_size == 0)
    {
        return;
    }
    if (pool_size == 1)
    {
        _proximity_ordering.push_back(plan_pool[0]);
        return;
    }

    std::vector<std::size_t> index_vect;
    std::vector<double> d_vect;
    robot_state::RobotState target_start(_rmodel);
    robot_state::RobotState target_end(_rmodel);

    // Initialize target start and end states
    target_start.setVariablePositions(_target_point.data());
    target_end.setVariablePositions(_target_point.data() + 6);

    robot_state::RobotState state(_rmodel);
    double distance;

    // std::cout << "Calculating distances." << std::endl;
    for (int i=0; i < pool_size; i++)
    {
        state.setVariablePositions(_plans[ plan_pool[i] ].start_state.joint_state.position);
        distance = target_start.distance(state);

        state.setVariablePositions(_plans[ plan_pool[i] ].end_state.joint_state.position);
        distance += target_end.distance(state);

        // std::cout << distance << std::endl;
        index_vect.push_back(plan_pool[i]);
        d_vect.push_back(distance);
    }

    // Find 1st, then 2nd, then 3rd lowest distance, and so on
    while (d_vect.size() > 0)
    {
        double min = 10000;
        std::size_t min_index = 0;
        int min_j = 0;

        for (int j=0; j < d_vect.size(); j++)
        {
            if (d_vect[j] < min)
            {
                // Then we have a new minimum
                min = d_vect[j];
                min_index = index_vect[j];
                min_j = j;
            }
        }

        // Now add to proximity queue
        _proximity_ordering.push_back(min_index);
        std::cout << "Added " << min_index << " with distance of " << min << " to priority queue." << std::endl;
        // And remove entry from distance and index vectors
        std::vector<double>::iterator d_it = d_vect.begin() + min_j;
        std::vector<std::size_t>::iterator i_it = index_vect.begin() + min_j;
        d_vect.erase(d_it);
        index_vect.erase(i_it);
    }

    return;
}
