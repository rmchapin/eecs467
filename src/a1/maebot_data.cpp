#include "maebot_data.hpp"
#include <math.h>

maebot_laser::maebot_laser(){
    utime = 0;
    range = 0;
    theta = 0;
    intensity = 0;
    x_pos = 0;
    y_pos = 0;
}

maebot_laser::maebot_laser(int64_t time, float r,float th,float inten,float x,float y){
    utime = time;
    range = r;
    theta = th;
    intensity = inten;
    x_pos = x;
    y_pos = y; 
}

int64_t maebot_laser::get_timestamp(){
    return utime;
}

float maebot_laser::get_range(){
    return range;
}

float maebot_laser::get_theta(){
    return theta;
}

float maebot_laser::get_intensity(){
    return intensity;
}

float maebot_laser::get_x_pos(){
    return x_pos;
}

float maebot_laser::get_y_pos(){
    return y_pos;
}

float maebot_laser::get_x_end_pos(){
    return x_pos+cosf(theta)*range; 
}

float maebot_laser::get_y_end_pos(){
    return y_pos+sinf(theta)*range;
}

maebot_pose_delta::maebot_pose_delta(){
    delta_x = 0;
    delta_y = 0;
    delta_theta = 0;
}

maebot_pose_delta::maebot_pose_delta(float dx,float dy, float dt){
    delta_x = dx;
    delta_y = dy;
    delta_theta = dt;
}

float maebot_pose_delta::get_delta_x(){
    return delta_x;
}

float maebot_pose_delta::get_delta_y(){
    return delta_y;
}

float maebot_pose_delta::get_delta_theta(){
    return delta_theta;
}



