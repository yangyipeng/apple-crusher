#ifndef KD_TREE_H
#define KD_TREE_H

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <boost/shared_ptr.hpp>

#include <cmath>

typedef struct {
    moveit_msgs::RobotTrajectory trajectory;
    moveit_msgs::RobotState start_state;
    moveit_msgs::RobotState end_state;
    int start_target_index;
    int end_target_index;
    double duration; // seconds
    int num_wpts;
} ur5_motion_plan;

typedef std::vector<double> joint_values_t;

typedef std::size_t coord_t;
typedef std::vector<coord_t> cell_coords_t;

class Cell
{
protected:
    std::size_t _dimension;
    cell_coords_t _coords;
    std::vector<std::size_t> _values;
public:
    Cell(const std::vector<std::size_t>& coords);

    inline const std::vector<std::size_t>& getCoords() { return _coords; }
    inline const std::vector<std::size_t>& getValues() { return _values; }

    std::size_t rectDistFrom(const cell_coords_t &coords);

    inline void addValue(std::size_t val) { _values.push_back(val); }

    friend bool operator== (const Cell& lhs, const Cell& rhs);
    friend bool operator!= (const Cell& lhs, const Cell& rhs);
};

class KDTree
{
    // Robot model
    robot_model::RobotModelPtr _rmodel;

    // KD parameters
    std::size_t _dimension;
    std::vector<double> _bounds_low;
    std::vector<double> _bounds_high;
    std::vector<coord_t> _resolution;
    std::vector<double> _cell_increments;

    // Data
    std::vector<ur5_motion_plan> _plans;
    std::size_t _plan_count;
    std::vector<Cell> _cells;
    std::size_t _cell_count;

    // Proximity Queue data
    joint_values_t _target_point;
    cell_coords_t _target_coords;
    std::vector<std::size_t> _proximity_ordering;
    int _search_depth;                              // distance of furthest cells included in proximity ordering so far

    // Helper functions
    std::vector<std::size_t> calcCoords(const joint_values_t& jvals);
    void searchCellsAtNextDistance();
    void linearSort(const std::vector<std::size_t>& plan_pool);
    void rectDistFrom(const cell_coords_t& coords);

public:
    KDTree(robot_model::RobotModelPtr& rmodel, const std::vector<double>& low_bounds, const std::vector<double>& high_bounds, const std::vector<std::size_t>& resolution);

    void add(const ur5_motion_plan &plan);

    void setTargets(const joint_values_t& start_jvals, const joint_values_t& end_jvals);
    double lookup(ur5_motion_plan& plan, int hit);

};

typedef boost::shared_ptr<KDTree> KDTreePtr;

#endif // KD_TREE_H
