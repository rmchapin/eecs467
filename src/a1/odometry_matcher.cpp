#include "odometry_matcher.hpp"

odometry_matcher::odometry_matcher(maebot_motor_feedback_t &msg){
    prev = msg;
    //curr = msg;
    next = msg;
}

void odometry_matcher::update(maebot_motor_feedback_t &msg){
    prev = next;
    /*float time_ratio = (float)(laser_time - next.utime) / (float)(msg.utime - next.utime);
    curr.utime = laser_time;
    curr.encoder_right_ticks = next.encoder_right_ticks + time_ratio*(float)(msg.encoder_right_ticks - next.encoder_right_ticks);
    curr.encoder_left_ticks = next.encoder_left_ticks + time_ratio*(float)(msg.encoder_left_ticks - next.encoder_left_ticks);
    */
    next = msg;
}

maebot_motor_feedback_t odometry_matcher::get_next_motor_feedback(){
    return next;
}

maebot_motor_feedback_t odometry_matcher::get_prev_motor_feedback(){
    return prev;
}
