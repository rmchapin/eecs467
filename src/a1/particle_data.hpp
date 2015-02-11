#ifndef PARTICLE_DATA_HPP
#define PARTICLE_DATA_HPP
#include <deque>
#include <math/point.hpp>
#include <math/gsl_util_rand.h>
#include <lcmtypes/maebot_pose_t.hpp>
#include <lcmtypes/maebot_laser_scan_t.hpp>
#include <mapping/occupancy_grid.hpp>

typedef maebot_pose_t maebot_pose_delta_t;

class particle_data{
public:
    particle_data();
    particle_data(int numb, maebot_pose_t starting_loc);
    particle_data(const particle_data& other);
    
    maebot_pose_t get_pose(int index);
    float get_weight(int index);
    int get_size();

    void translate(maebot_pose_delta_t deltas);
    void calc_weight(eecs467::OccupancyGrid & grid, const maebot_laser_scan_t & lasers);
    maebot_pose_t get_best();
    float* get_particle_coords();
    ~particle_data(){};
    std::deque<maebot_pose_t> pose;
    std::deque<float> weight;
    int number;
    
};

#endif