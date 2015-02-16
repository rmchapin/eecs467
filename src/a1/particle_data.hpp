#ifndef PARTICLE_DATA_HPP
#define PARTICLE_DATA_HPP
#include <vector>
#include <math/point.hpp>
#include <math/gsl_util_rand.h>
#include <lcmtypes/maebot_pose_t.hpp>
#include <lcmtypes/maebot_laser_scan_t.hpp>
#include <lcmtypes/maebot_motor_feedback_t.hpp>
#include <mapping/occupancy_grid.hpp>
#include "maebot_data.hpp"
#include "sensor_model.hpp"
#include "action_model.hpp"
#include "odometry_matcher.hpp"

typedef maebot_pose_t maebot_pose_delta_t;

class particle_data{
public:
    particle_data(){}
    particle_data(int numb, maebot_pose_t starting_loc,eecs467::OccupancyGrid *g);
    void push_odo(maebot_motor_feedback_t msg);
    void push_scan(maebot_laser_scan_t msg); 
    void update();
    void resample();
    void normalize();
    bool ready();
    maebot_pose_t get_pose(int index);
    float get_weight(int index);
    int get_size();
    maebot_laser_scan_t get_scan();
    maebot_pose_t get_best();
    float* get_particle_coords();
    ~particle_data(){};
    bool processing;
    bool has_scan;
    bool has_map;
    sensor_model s_model;

private:   
    std::vector<maebot_pose_t> pose;
    std::vector<float> weight;
    std::vector<maebot_pose_t> old_pose;
    std::vector<float> old_weight;
    int number;
    gsl_rng* rand_gen;
    action_model a_model;
    odometry_matcher odo_matcher;
    maebot_laser_scan_t scan;
};

#endif
