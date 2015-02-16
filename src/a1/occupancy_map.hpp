#ifndef OCCUPANCY_MAP_HPP
#define OCCUPANCY_MAP_HPP

#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"
#include "maebot_data.hpp"
#include <vector>
class occupancy_map{
public:
    occupancy_map(float wInMeters=5, float hInMeters=5, float mPerCell=0.05, float s_rate = 1);
    void update(std::vector<maebot_laser> lasers);
    eecs467::OccupancyGrid& get_grid();
    eecs467::OccupancyGrid grid;
private:
    float sampling_rate; 
};
#endif
