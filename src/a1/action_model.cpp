#include "action_model.hpp"
#include <math.h>
#include <math/angle_functions.hpp>
action_model::action_model():k1(5.0),k2(0.5){
    rand_gen = gslu_rand_rng_alloc(); 
}


action_model::action_model(float e1, float e2):k1(e1),k2(e2){
    rand_gen = gslu_rand_rng_alloc();
}

void action_model::init_model(maebot_motor_feedback_t prev_odo,maebot_motor_feedback_t curr_odo){
    float delta_right = (curr_odo.encoder_right_ticks - prev_odo.encoder_right_ticks)*0.032*M_PI/480.0;
    float delta_left  = (curr_odo.encoder_left_ticks - prev_odo.encoder_left_ticks)*0.032*M_PI/480.0;
    dis = (delta_right+delta_left) / 2;
    dtheta = (delta_right - delta_left) / 0.08;
    alpha = dtheta/2;
    curr_timestamp = curr_odo.utime;
}

maebot_pose_t action_model::gen_pose(maebot_pose_t prev_pose)
{
    // printf("odometry compare in action model:%d %d\n",curr_odo.encoder_left_ticks,prev_odo.encoder_left_ticks);
    //printf("dis:%f dtheta:%f alpha:%f\n",dis,dtheta,alpha);

    float dx = dis*cosf(prev_pose.theta + alpha);
    float dy = dis*sinf(prev_pose.theta + alpha);
    float delta_s = sqrt(dx*dx + dy*dy);
    float a = eecs467::wrap_to_pi(atan2(dy,dx) - prev_pose.theta);

    //std::normal_distribution<float> nor_dist1(a,k1*a);
    //std::normal_distribution<float> nor_dist2(delta_s,k2*delta_s);
    //std::normal_distribution<float> nor_dist3(dtheta-a,k1*(dtheta-a));
    //printf("k1%f k2%f",k1,k2);
    /*float rot1 = gslu_rand_normal(rand_gen)*sqrt(fabs(a*k1));
    float trans = gslu_rand_normal(rand_gen)*sqrt(fabs(delta_s*k2));
    float rot2 = gslu_rand_normal(rand_gen)*sqrt(fabs(eecs467::wrap_to_pi(dtheta-a)*k1));*/
    float rot1 = gslu_rand_gaussian(rand_gen,0,sqrt(fabs(a*k1)));
    float trans = gslu_rand_gaussian(rand_gen,0,sqrt(fabs(delta_s*k2)));
    float rot2 = gslu_rand_gaussian(rand_gen,0,sqrt(fabs(eecs467::wrap_to_pi(dtheta-a)*k1)));
    //printf("rot1: %f trans: %f rot2: %f\n",rot1,trans,rot2);*/
    maebot_pose_t ret_pose;
    ret_pose.x = prev_pose.x + fabs(trans)*cosf(prev_pose.theta + rot1);
    ret_pose.y = prev_pose.y + fabs(trans)*sinf(prev_pose.theta + rot1);
    ret_pose.theta = eecs467::wrap_to_pi(prev_pose.theta + rot1 + rot2);
    ret_pose.utime = curr_timestamp;
    //printf("gen_pose here with: %f %f %f\n",ret_pose.x,ret_pose.y,ret_pose.theta);
    return ret_pose; 
}
