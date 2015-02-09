#ifndef ODOMETRY_MATCHER_HPP
#define ODOMETRY_MATCHER_HPP

#include "lcmtypes/maebot_motor_feedback_t.hpp"

class odometry_matcher{
public:
    odometry_matcher(maebot_motor_feedback_t &msg);
    void update(maebot_motor_feedback_t &msg,int64_t laser_time);
    maebot_motor_feedback_t get_curr_motor_feedback();   
    maebot_motor_feedback_t get_prev_motor_feedback();

private:
    maebot_motor_feedback_t prev;
    maebot_motor_feedback_t curr;
    maebot_motor_feedback_t next;
};

#endif
