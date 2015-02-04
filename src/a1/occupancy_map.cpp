#include "occupancy_map.hpp"
#include "math/point.hpp"
#include <math.h>
#include <math/point.hpp>

occupancy_map::occupancy_map(float wInMeters, float hInMeters, float mPerCell, float s_rate){
    grid = eecs467::OccupancyGrid(wInMeters,hInMeters,mPerCell);
    sampling_rate = s_rate;
}

void occupancy_map::update(std::deque<maebot_laser> &lasers){
    eecs467::Point<float> global_pos;
    for(int i=0;i<lasers.size();++i){
        maebot_laser l = lasers[i];
        global_pos.x = l.get_x_pos();
        global_pos.y = l.get_y_pos();
        float dx = sampling_rate * grid.metersPerCell() * cosf(l.get_theta());
        float dy = sampling_rate * grid.metersPerCell() * sinf(l.get_theta());
        int times = l.get_range() / (sampling_rate*grid.metersPerCell());
        eecs467::Point<int> cell_pos;
        for(int i = 0;i<times;++i){
            cell_pos = global_position_to_grid_cell(global_pos,grid);
            if(!grid.isCellInGrid(cell_pos.y,cell_pos.x)){
                break;
            }

            if(grid(cell_pos.y,cell_pos.x) < (-128+2)){
                grid(cell_pos.y,cell_pos.x) = -128;
            }
            else{
                grid(cell_pos.y,cell_pos.x) -= 2;
            }
            global_pos.x += dx;
            global_pos.y += dy;
        }
        //get bound
        global_pos.x = l.get_x_pos() + l.get_range()*cosf(l.get_theta());
        global_pos.y = l.get_y_pos() + l.get_range()*sinf(l.get_theta());
        cell_pos = global_position_to_grid_cell(global_pos,grid);
        if(!grid.isCellInGrid(cell_pos.y,cell_pos.x)){
            continue;
        }
        if(grid(cell_pos.y,cell_pos.x) > (127-1)){
            grid(cell_pos.y,cell_pos.x) = 127;
        }
        else{
            grid(cell_pos.y,cell_pos.x) += 1;
        }
    }
}

eecs467::OccupancyGrid& occupancy_map::get_grid(){
    return grid;
}

