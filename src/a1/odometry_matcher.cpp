#include "odometry_matcher.hpp"

odometry_matcher::odometry_matcher(){
    prev.encoder_left_ticks = 0;
    prev.encoder_right_ticks = 0;
    prev.utime = -1;
    curr.encoder_left_ticks = 0;
    curr.encoder_right_ticks = 0;
    curr.utime = -1;
}


odometry_matcher::odometry_matcher(maebot_motor_feedback_t msg){
    prev = msg;
    curr = msg;
}

void odometry_matcher::set_prev(maebot_motor_feedback_t msg){
    prev = msg;
}

void odometry_matcher::update(maebot_motor_feedback_t msg){
    if(prev.utime == -1){
        prev = msg;
    }
    else{
        curr = msg;
    }
    //printf("odo matcher updated\n");
    //printf("msg: %d\n",msg.encoder_left_ticks);
    //printf("prev: %d\n",prev.encoder_left_ticks);
    //printf("curr: %d\n", curr.encoder_left_ticks);
}

maebot_motor_feedback_t odometry_matcher::get_curr_motor_feedback(){
    return curr;
}

maebot_motor_feedback_t odometry_matcher::get_prev_motor_feedback(){
    return prev;
}

maebot_motor_feedback_t odometry_matcher::get_interpolate_motor_feedback(int64_t time){
    float time_elapsed = (float)(time - prev.utime);
    float time_diff = (float)(curr.utime - prev.utime);
    float time_ratio = time_elapsed / time_diff;
    maebot_motor_feedback_t ret;
    ret.utime = time;
    ret.encoder_left_ticks = prev.encoder_left_ticks + time_ratio*(curr.encoder_left_ticks - prev.encoder_left_ticks);
    ret.encoder_right_ticks = prev.encoder_right_ticks + time_ratio*(curr.encoder_right_ticks - prev.encoder_right_ticks);
    return ret;
}
