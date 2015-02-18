#ifndef PATH_PLANNING_HPP
#define PATH_PLANNING_HPP
#include <deque>
#include <vector>
#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "math/point.hpp"

struct breadth_point{
    bool visited;
    eecs467::Point<int> prev_coord;
};

class path_planning{
public:
    path_planning(){}
    path_planning(eecs467::OccupancyGrid *g);
    void update_grid(eecs467::OccupancyGrid *g, int wall_expansion);
    std::vector<eecs467::Point<int>> find_frontier(eecs467::Point<float> best);
	eecs467::OccupancyGrid grid;
private:
    inline int convertTo1D(eecs467::Point<int> p);
    bool check_gray_range(eecs467::Point<int> p,int range);
    void enlarge_obstacle(int wall_expansion);
};


#endif
