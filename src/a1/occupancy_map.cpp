#include "occupancy_map.hpp"
#include <math.h>
#include <math/point.hpp>

occupancy_map::occupancy_map(float wInMeters, float hInMeters, float mPerCell, float s_rate){
    grid = eecs467::OccupancyGrid(wInMeters,hInMeters,mPerCell);
    sampling_rate = s_rate;
}

void occupancy_map::update(std::deque<maebot_laser> lasers){
    //printf("update map: %d\n",lasers.size());
    float SAMPLE_SPACING = sampling_rate * grid.metersPerCell();
    for(int i=0;i<lasers.size();++i){
        maebot_laser l = lasers[i];
        //printf("%f %f\n",l.get_range(),l.get_theta());
        if(l.get_intensity() == 0){
            continue;
        }
        float theta = l.get_theta();
        float sample_x = l.get_x_pos();
        float sample_y = l.get_y_pos();
        float dx = SAMPLE_SPACING * cosf(theta);
        float dy = SAMPLE_SPACING * sinf(theta);
        eecs467::Point<int> sample_cell;        
        for(float sample_m = 0; sample_m < l.get_range(); sample_m += SAMPLE_SPACING){
            sample_cell = global_position_to_grid_cell(eecs467::Point<float>(sample_x,sample_y),grid);
            if(grid.isCellInGrid(sample_cell.y,sample_cell.x)){
                int8_t odds = grid.logOdds(sample_cell.y,sample_cell.x); 
                if(odds < (-128+2)){
                    grid.setLogOdds(sample_cell.y,sample_cell.x,-128);
                }
                else{
                    odds -= 2;
                    grid.setLogOdds(sample_cell.y,sample_cell.x,odds);
                }
            } 
            sample_x += dx;
            sample_y += dy;
        }       
        //get bound
        sample_x = l.get_x_pos() + l.get_range()*cosf(theta);
        sample_y = l.get_y_pos() + l.get_range()*sinf(theta);
        sample_cell = global_position_to_grid_cell(eecs467::Point<float>(sample_x,sample_y),grid);
        if(grid.isCellInGrid(sample_cell.y,sample_cell.x)){
            int8_t odds = grid.logOdds(sample_cell.y,sample_cell.x); 
            if(odds > (127-2)){
                grid.setLogOdds(sample_cell.y,sample_cell.x,127);
            }
            else{
                odds += 2;
                grid.setLogOdds(sample_cell.y,sample_cell.x,odds);
            }
        } 

    }

}

eecs467::OccupancyGrid& occupancy_map::get_grid(){
    return grid;
}

