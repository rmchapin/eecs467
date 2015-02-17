#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <lcm/lcm-cpp.hpp>
#include <vector>
#include <deque>
#include <iostream>

// core api
#include "vx/vx.h"
#include "vx/vx_util.h"

#include "vx/gtk/vx_gtk_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

#include "common/getopt.h"
#include "common/timestamp.h"
#include "common/timestamp.h"
#include "imagesource/image_u32.h"
#include "imagesource/image_util.h"
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_sensor_data_t.hpp"
#include "particle_data.hpp"
#include "action_model.hpp"
#include "pose_tracker.hpp"
#include "path_planning.hpp"
#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"
#include <math/point.hpp>
#include "occupancy_map.hpp"
#include <math/gsl_util_rand.h>

static const char* MAP_TO_READ;
occupancy_map map;

void read_map()
{
    FILE *fp;
    uint8_t temp;
    fp = fopen(MAP_TO_READ,"r");
    fscanf(fp,"%d\n",&temp);
    if(temp != map.grid.heightInCells()){
        std::cout << "Height not match\n";
        exit(1);
    }
    fscanf(fp,"%d\n",&temp);
    if(temp != map.grid.widthInCells()){
        std::cout << "Width not match\n";
        exit(1);
    }
    map = occupancy_map(5.0,5.0,0.05,1.0); //if s_rate changes here, correction in samples class needs adjustment
    for(size_t y = 0; y < map.grid.heightInCells();y++){
        for(size_t x = 0; x < map.grid.widthInCells(); x++){
            fscanf(fp,"%d ",&temp);
            map.grid.setLogOdds(x,y,temp);
        }
    }
    fclose(fp);
}



int main(int argc, char ** argv)
{
    MAP_TO_READ = argv[1];
    printf("%s\n",MAP_TO_READ);
    read_map();
    path_planning p_plan = path_planning(&map.grid);
    std::vector<eecs467::Point<int>> result;
    eecs467::Point<float> start;
    start.x = 0;
    start.y = 0;
    //eecs467::Point<int> s = eecs467::global_position_to_grid_cell(start,map.grid);
    result = p_plan.find_frontier(start);
    printf("size: %d\n",result.size());
    if(!result.empty()){
        for(int i = 0;i<result.size();++i){
            printf("%d %d\n",result[i].x,result[i].y);
        }
    }
}
