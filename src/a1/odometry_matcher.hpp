#ifndef ODOMETRY_MATCHER_HPP
#define ODOMETRY_MATCHER_HPP

#include "lcmtypes/maebot_motor_feedback_t.hpp"

class odometry_matcher{
public:
    odometry_matcher();
    odometry_matcher(maebot_motor_feedback_t msg);
    void update(maebot_motor_feedback_t msg);
    void set_prev(maebot_motor_feedback_t msg);
    maebot_motor_feedback_t get_interpolate_motor_feedback(int64_t time);
    maebot_motor_feedback_t get_curr_motor_feedback();   
    maebot_motor_feedback_t get_prev_motor_feedback();

    maebot_motor_feedback_t prev;
    maebot_motor_feedback_t curr;
};

#endif
