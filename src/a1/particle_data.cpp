#include <deque>
#include <math.h>
#include <math/point.hpp>
#include "particle_data.hpp"
#include <math/gsl_util_rand.h>
#include <mapping/occupancy_grid_utils.hpp>
#include <iostream>
#include <string>
#include <math/angle_functions.hpp>

particle_data::particle_data(int numb, maebot_pose_t starting_loc,eecs467::OccupancyGrid *g){
    number = numb;
    rand_gen = gslu_rand_rng_alloc();
    pose.reserve(number);
    weight.reserve(number);
    old_weight.reserve(number);
    old_pose.reserve(number);
    for(int i=0; i < number ; ++i){
        old_pose.push_back(starting_loc);
        pose.push_back(starting_loc);
        float temp = (float)(1.0/number);
        old_weight.push_back(temp);
        weight.push_back(temp);
    }
    //printf("old_pose size: %d\n",old_pose.size());
    //printf("old_weight size: %d\n",old_weight.size());
    s_model = sensor_model(g); 
    a_model = action_model();
    odo_matcher = odometry_matcher();
    processing = false;
    has_scan = false;
    has_map = false;
}

void particle_data::push_odo(maebot_motor_feedback_t msg){
    //printf("pushed a odo\n");
    odo_matcher.update(msg);
}

void particle_data::push_scan(maebot_laser_scan_t msg){
    //printf("pushed a scan\n");
    scan = msg;
    has_scan = true;
}

void particle_data::update(){
    //call it in the scan handler
    //printf("processing\n");
    processing = true;
    resample();
    //printf("resampled\n");
    int64_t curr_time = (scan.times[0] + scan.times.back())/2;
    //int64_t curr_time = scan.times.back();
    maebot_motor_feedback_t prev = odo_matcher.get_prev_motor_feedback();
    maebot_motor_feedback_t curr = odo_matcher.get_interpolate_motor_feedback(curr_time);
    //printf("time; %lld \n",curr_time);
    //printf("prev odo: %d %d\n",prev.encoder_left_ticks,prev.encoder_right_ticks);
    //printf("interpolated odo: %d %d\n",curr.encoder_left_ticks,curr.encoder_right_ticks);
    //printf("interpolated\n");
    bool isMove = true;
    if(prev.encoder_left_ticks == curr.encoder_left_ticks && prev.encoder_right_ticks == curr.encoder_right_ticks){
        isMove = false;
    }
    if(isMove){
        a_model.init_model(prev,curr);
        //printf("model init\n");
        odo_matcher.set_prev(curr);
        for(int i = 0;i<number;++i){
            maebot_pose_t prev_pose = pose[i];
            //update pos
            pose[i] = a_model.gen_pose(prev_pose);
            //printf("gen new pose\n");
            //if(pose[i].x == 0.000000 && pose[i].y == 0.000000)
            //printf("%f %f\n",pose[i].x,pose[i].y);
            //update weight
            weight[i] = s_model.calc_weight(scan,prev_pose,pose[i]); 
            //printf("sensor_model applied\n");
            //printf("%f\n",weight[i]);
        }
        normalize();
    }
    else{
        for(int i = 0;i<number;++i){
            pose[i].utime = curr_time;
        }     
    }
    processing = false;
    has_scan = false;
}

void particle_data::resample(){
    //printf("old weight size: %d\n",old_weight.size());
    //printf("old pose size: %d\n",old_pose.size());
    //printf("weight size: %d\n",weight.size());
    //printf("pose size: %d \n",pose.size());
    for(int i=0;i<number;++i){
        float alpha = gslu_rand_uniform(rand_gen);
        float weight_sum = 0.0;
        int k = 0;
        //printf("get alpha: %f\n",alpha);
        while(weight_sum < alpha){
            //printf("old_weight %d %f %f\n",k,alpha,weight_sum);
            weight_sum += old_weight[k];
            ++k;
        }
        //printf("alpha: %f weight_sum: %f\n",alpha,weight_sum);
        if(k!=0){
            pose[i] = old_pose[k-1];
            weight[i] = old_weight[k-1];
        }
        else{
            pose[i] = old_pose[k];
            weight[i] = old_weight[k];
        }
        k = 0;
        weight_sum = 0.0;
    }
}

void particle_data::normalize(){
    float max_weight = *(std::max_element(weight.begin(),weight.end()));
    float weight_sum = 0.0;
    for(int i=0;i<number;++i){
        weight[i] -= max_weight;
        weight[i] = exp(weight[i]);
        weight_sum += weight[i];
    }
    for(int i=0;i<number;++i){
        weight[i] /= weight_sum;
        old_weight[i] = weight[i];
        old_pose[i] = pose[i];
    }
}

bool particle_data::ready(){
    maebot_motor_feedback_t prev = odo_matcher.get_prev_motor_feedback();
    maebot_motor_feedback_t curr = odo_matcher.get_curr_motor_feedback();
    if(has_scan && prev.utime != -1 && curr.utime != -1 && curr.utime > scan.times.back()){
        //printf("ready!\n");
        return true;
    }
    //printf("not ready\n");
    return false;
}

maebot_pose_t particle_data::get_pose(int index){
    return old_pose[index];
}

float particle_data::get_weight(int index){
    return old_weight[index];
}

int particle_data::get_size(){
    return old_weight.size();
}

maebot_laser_scan_t particle_data::get_scan(){
    return scan;
}

maebot_pose_t particle_data::get_best(){
    int highest_index=0;
    for(int i=0; i< number; ++i){
        if(old_weight[i] > old_weight[highest_index] ){
            highest_index = i;
        }
    }

    //printf("highest index: %d\n", highest_index);
    return old_pose[highest_index];
}

float* particle_data::get_particle_coords(){
    float pts[3*number];
    for(int i=0;i<old_pose.size();++i){
        pts[3*i+0] = old_pose[i].x*15;
        pts[3*i+1] = old_pose[i].y*15;
        pts[3*i+2] = 0;
    }
    return pts;
}
