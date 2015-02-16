#ifndef SENSOR_MODEL_HPP
#define SENSOR_MODEL

#include <stdint.h>

#include "mapping/occupancy_grid.hpp"
#include "lcmtypes/maebot_laser_scan_t.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "maebot_data.hpp"
class sensor_model{
public:
    sensor_model(){};
    sensor_model(eecs467::OccupancyGrid *g){
        grid = g;
    }
    void update_grid(eecs467::OccupancyGrid *g){
        grid = g;
    }
    float calc_weight(maebot_laser_scan_t msg, maebot_pose_t &start, maebot_pose_t &end);  
    std::vector<maebot_laser> get_processed_laser_scan(maebot_laser_scan_t msg, maebot_pose_t &start, maebot_pose_t &end);
private:
    eecs467::OccupancyGrid* grid;
    
};

#endif
