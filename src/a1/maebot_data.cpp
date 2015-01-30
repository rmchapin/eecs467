#include "maebot_data.hpp"
#include <math.h>
maebot_pose_data::maebot_pose_data(){
    pose_time = 0;
    pose_x_curr = 0.0;
    pose_y_curr = 0.0;
    pose_theta_curr = 0.0;
}

maebot_pose_data::maebot_pose_data(const maebot_pose_data& other):
    pose_time(other.pose_time),pose_x_curr(other.pose_x_curr),
    pose_y_curr(other.pose_y_curr),pose_theta_curr(other.pose_theta_curr){}


maebot_pose_data::maebot_pose_data(int64_t utime,float x,float y, float theta){
    pose_time = utime;
    pose_x_curr = x;
    pose_y_curr = y;
    pose_theta_curr = theta;
}

int64_t maebot_pose_data::get_timestamp(){
    return pose_time;
}

float maebot_pose_data::get_x_pos(){
    return pose_x_curr;
}

float maebot_pose_data::get_y_pos(){
    return pose_y_curr;
}

float maebot_pose_data::get_theta(){
    return pose_theta_curr;
}

maebot_laser_data::maebot_laser_data(){
    laser_time = 0;
    num_ranges = 0;
    curr_pose = maebot_pose_data(); 
}

maebot_laser_data::maebot_laser_data(int64_t utime,int32_t nranges,std::vector<float> rs,
        std::vector<float> ths,std::vector<int64_t> ts,
        std::vector<float> is){
    laser_time = utime;
    num_ranges = nranges;
    ranges = rs;
    thetas = ths;
    times = ts;
    intensities = is;
    curr_pose = maebot_pose_data();
}

maebot_laser_data::maebot_laser_data(int64_t utime,int32_t nranges,std::vector<float> rs,
        std::vector<float> ths,std::vector<int64_t> ts,
        std::vector<float> is, maebot_pose_data pose){
    laser_time = utime;
    num_ranges = nranges;
    ranges = rs;
    thetas = ths;
    times = ts;
    intensities = is;
    curr_pose = pose;
    calc_end_points();
}

void maebot_laser_data::calc_end_points(){
    for(int i = 0; i < num_ranges;++i){
        float x = curr_pose.get_x_pos()+cosf(thetas[i]-curr_pose.get_theta())*ranges[i];
        float y = curr_pose.get_y_pos()+sinf(thetas[i]-curr_pose.get_theta())*ranges[i];
        end_points.push_back(eecs467::Point<float>(x,y));
    } 
}

int64_t maebot_laser_data::get_timestamp(){
    return laser_time;
}

int32_t maebot_laser_data::get_num_ranges(){
    return num_ranges;
}

std::vector<float> maebot_laser_data::get_ranges(){
    return ranges;
}

std::vector<float> maebot_laser_data::get_thetas(){
    return thetas;
}

std::vector<int64_t> maebot_laser_data::get_times(){
    return times;
}

std::vector<float> maebot_laser_data::get_intensities(){
    return intensities;
}

maebot_pose_data maebot_laser_data::get_curr_pose(){
    return curr_pose;
}

std::vector<eecs467::Point<float>> maebot_laser_data::get_end_points(){
    return end_points;
} 





