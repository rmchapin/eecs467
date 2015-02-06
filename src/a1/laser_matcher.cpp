#include "laser_matcher.hpp"
#include "math/angle_functions.hpp"
#include <math.h>

bool laser_matcher::push_laser(const maebot_laser_scan_t *msg){
    for(int32_t i = 0; i< msg->num_ranges;++i){
        maebot_laser l = maebot_laser(msg->times[i],
                                        msg->ranges[i],
                                        msg->thetas[i],
                                        msg->intensities[i],
                                        0,
                                        0);
        to_process_laser.push_back(l);
        //printf("%f %f\n",msg->ranges[i],msg->thetas[i]);
    }
    return true;
}

bool laser_matcher::push_pose(const maebot_pose_t *msg){
    poses.push_back(*msg);
    return true;
}

void laser_matcher::process(){
    if(to_process_laser.empty()){
        return;
    }
    if(poses.empty() || (poses.front().utime > to_process_laser.back().get_timestamp())){
        to_process_laser.clear();
        return;
    }
    while(!to_process_laser.empty()){
        maebot_laser l = to_process_laser.front();
        //printf("%f %f\n",l.get_range(),l.get_theta());
        //printf("%d\n",to_process_laser.size());
        maebot_pose_t curr;
        while(!poses.empty() && (poses.front().utime < l.get_timestamp())){
            curr = poses.front();
            poses.pop_front();
        }
        if(poses.empty()){
            poses.push_front(curr);
            return;
        }
        float angle_elapsed = eecs467::angle_diff(poses.front().theta,curr.theta);
        if(fabs(angle_elapsed) > 2){
            to_process_laser.clear();
            return;
        }
        float time_diff = (float)(l.get_timestamp() - curr.utime);
        float time_elapsed = (float)(poses.front().utime - curr.utime);
        float time_ratio = time_diff / time_elapsed;
        float x_pos = curr.x + time_ratio*(poses.front().x - curr.x);
        float y_pos = curr.y + time_ratio*(poses.front().y - curr.y);
        float theta = curr.theta + time_ratio*angle_elapsed;
        maebot_laser post_l = maebot_laser(l.get_timestamp(),
                                            l.get_range(),
                                            eecs467::wrap_to_2pi(theta-l.get_theta()),
                                            l.get_intensity(),
                                            x_pos,
                                            y_pos); 
        processed_laser.push_back(post_l);
        poses.push_front(curr);
        to_process_laser.pop_front();
    }
}

bool laser_matcher::get_processed_laser(std::deque<maebot_laser> &laser){
    if(processed_laser.empty()){
        return false;
    }
    laser = processed_laser;
    processed_laser.clear();
    return true;
}

std::deque<maebot_laser> laser_matcher::get_processed_laser(){
    std::deque<maebot_laser> lasers = processed_laser;
    //printf("not empty with size %d %d\n",processed_laser.size(),lasers.size());
    //printf("%d\n",lasers.size());
    processed_laser.clear();
    return lasers;
}

