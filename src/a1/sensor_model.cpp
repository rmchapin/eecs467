#include "sensor_model.hpp"
#include "mapping/occupancy_grid_utils.hpp"
#include "math/angle_functions.hpp"
#include "math/point.hpp"
std::vector<maebot_laser> sensor_model::get_processed_laser_scan(maebot_laser_scan_t msg, maebot_pose_t &start, maebot_pose_t &end){
    std::vector<maebot_laser> processed_laser_scan;
    for(int i=0;i<msg.num_ranges;++i){
        float time_diff = (float)(msg.times[i] - start.utime);
        float time_elapsed = (float)(end.utime - start.utime);
        float time_ratio = time_diff / time_elapsed;
        float x_pos = start.x + time_ratio*(end.x - start.x);
        float y_pos = start.y + time_ratio*(end.y - start.y);
        float theta = eecs467::wrap_to_pi(start.theta + eecs467::wrap_to_pi(time_ratio*eecs467::angle_diff(eecs467::wrap_to_pi(end.theta),eecs467::wrap_to_pi(start.theta))));
        maebot_laser l = maebot_laser(msg.times[i],
                                            msg.ranges[i],
                                            eecs467::wrap_to_pi(theta-msg.thetas[i]),
                                            msg.intensities[i],
                                            x_pos,
                                            y_pos); 
        processed_laser_scan.push_back(l);
    }
    return processed_laser_scan;
}


float sensor_model::calc_weight(maebot_laser_scan_t msg, maebot_pose_t &begin, maebot_pose_t &back){
    std::vector<maebot_laser> processed_laser_scan = get_processed_laser_scan(msg,begin,back);
    eecs467::Point<float> start,end;
    float weight = 0.0;
    for(int i=0;i<processed_laser_scan.size();++i){
        start.x = processed_laser_scan[i].get_x_pos();
        start.y = processed_laser_scan[i].get_y_pos();
        end.x = processed_laser_scan[i].get_x_end_pos();
        end.y = processed_laser_scan[i].get_y_end_pos();
        //printf("start: %f %f end: %f %f\n",start.x,start.y,end.x,end.y);
        eecs467::Point<int> start_cell = eecs467::global_position_to_grid_cell(start,*grid);
        if(grid->isCellInGrid(start_cell.x,start_cell.y) == false){
            weight -= 30;
        }
        eecs467::Point<int> end_cell = eecs467::global_position_to_grid_cell(end,*grid);
        if(grid->isCellInGrid(end_cell.x,end_cell.y) == false){
            weight -= 8.0;
        }
        else if(grid->logOdds(end_cell.x,end_cell.y) > 0){
            weight -= 4.0;
        }
        else if(grid->logOdds(end_cell.x,end_cell.y) < 0){
            weight -= 6.0;
        }
        else{
            weight -= 8.0;
        }
    }
    //printf("start:%f %f end:%f %f\n",processed_laser_scan[0].get_x_pos(),processed_laser_scan[0].get_y_pos(),
    //                                processed_laser_scan[0].get_x_end_pos(),processed_laser_scan[0].get_y_end_pos()); 
    return weight;
} 

