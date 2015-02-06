#ifndef LASER_MATCHER_HPP
#define LASER_MATCHER_HPP

#include <deque>
#include <vector>
#include "maebot_data.hpp"
#include <lcmtypes/maebot_laser_scan_t.hpp>
#include <lcmtypes/maebot_pose_t.hpp>


class laser_matcher{
public:
    laser_matcher(){}
    bool push_laser(const maebot_laser_scan_t* msg);
    bool push_pose(const maebot_pose_t* msg);
    void process();
    std::deque<maebot_laser> get_processed_laser();    
    bool get_processed_laser(std::deque<maebot_laser> &lasers);
private:
    std::deque<maebot_laser> to_process_laser;
    std::deque<maebot_laser> processed_laser;
    std::deque<maebot_pose_t> poses;
};

#endif
