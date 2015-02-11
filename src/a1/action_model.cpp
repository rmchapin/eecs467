#include "action_model.hpp"
#include <math/angle_functions.hpp>
action_model::action_model():
    rand_gen(std::chrono::system_clock::now().time_since_epoch().count()),
    k1(0.0),k2(0.0){}
  

action_model::action_model(float e1, float e2):
    rand_gen(std::chrono::system_clock::now().time_since_epoch().count()),
    k1(e1),k2(e2){}

maebot_pose_t action_model::gen_pose(maebot_pose_t prev_pose, maebot_motor_feedback_t prev_odo, maebot_motor_feedback_t curr_odo)
{

   // printf("odometry compare in action model:%d %d\n",curr_odo.encoder_left_ticks,prev_odo.encoder_left_ticks);
    float delta_right = (curr_odo.encoder_right_ticks - prev_odo.encoder_right_ticks) / 4800.0;
    float delta_left  = (curr_odo.encoder_left_ticks - prev_odo.encoder_left_ticks) / 4800.0;
    float dis = (delta_right + delta_left) / 2.0;
    float dtheta = (delta_right - delta_left) / 0.08;
    float alpha = dtheta / 2.0;
    printf("%f %f %f %f %f\n",delta_right,delta_left,dis,dtheta,alpha);

    float dx = dis*cosf(prev_pose.theta + alpha);
    float dy = dis*sinf(prev_pose.theta + alpha);
    float delta_s = sqrt(dx*dx + dy*dy);
    float a = atan2(dy,dx) - prev_pose.theta;

    std::normal_distribution<float> nor_dist1(a,k1*a);
    std::normal_distribution<float> nor_dist2(delta_s,k2*delta_s);
    std::normal_distribution<float> nor_dist3(dtheta-a,k1*(dtheta-a));
    //printf("k1%f k2%f",k1,k2);
    float rot1 = nor_dist1(rand_gen);
    float trans = nor_dist2(rand_gen);
    float rot2 = nor_dist3(rand_gen);

    maebot_pose_t ret_pose;
    ret_pose.x = prev_pose.x + trans*cosf(prev_pose.theta + rot1);
    ret_pose.y = prev_pose.y + trans*sinf(prev_pose.theta + rot1);
    ret_pose.theta = eecs467::wrap_to_2pi(prev_pose.theta + rot1 + rot2);
    ret_pose.utime = curr_odo.utime;
   // printf("gen_pose here with: %f %f %f\n",ret_pose.x,ret_pose.y,ret_pose.theta);
    return ret_pose; 
}
