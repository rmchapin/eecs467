#include "action_model.hpp"
#include <chrono>
#include <math.h>

action_model::action_model():
    rand_gen(std::chrono::system_clock::now().time_since_epoch().count()),
    nor_dist(0,1),k1(1.0),k2(0.4){}
  

action_model::action_model(float e1, float e2):
    rand_gen(std::chrono::system_clock::now().time_since_epoch().count()),
    nor_dist(0,1),k1(k1),k2(e2){}

// in the localization main file, we will use the odometry_matcher to return the
// prev and curr odometry data

void action_model::init_model(maebot_motor_feedback_t prev,maebot_motor_feedback_t curr){
    float delta_right = (curr.encoder_right_ticks - prev.encoder_right_ticks)/4800.0;
    float delta_left  = (curr.encoder_left_ticks - prev.encoder_left_ticks)/4800.0;
    dis = (delta_right+delta_left) / 2;
    dtheta = (delta_right - delta_left) / 0.08;
    alpha = dtheta/2;
    curr_timestamp = curr.utime;
}
//prev_pose is the current estimated maebot position
//get the new pose based on current motor feedback data
maebot_pose_t action_model::get_new_pose(maebot_pose_t prev_pose){
    float dx = dis*cosf(prev_pose.theta + alpha);
    float dy = dis*sinf(prev_pose.theta + alpha);
    float delta_s = sqrt(dx*dx + dy*dy);
    float a = atan2(dy,dx) - prev_pose.theta;
    float rot1 = nor_dist(rand_gen)*k1*a;
    float trans = nor_dist(rand_gen)*k2*delta_s;
    float rot2 = nor_dist(rand_gen)*k1*(dtheta-a);
    maebot_pose_t new_pose;
    new_pose.x = prev_pose.x + trans*cosf(prev_pose.theta+rot1);
    new_pose.y = prev_pose.y + trans*sinf(prev_pose.theta+rot1);
    new_pose.theta = prev_pose.theta + rot1 + rot2;
    new_pose.utime = curr_timestamp;
    return new_pose; 
}
