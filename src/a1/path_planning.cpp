#include "path_planning.hpp"


path_planning::path_planning(eecs467::OccupancyGrid *g){
    grid = *g;
    //enlarge_obstacle();
}

void path_planning::update_grid(eecs467::OccupancyGrid *g, int wall_expansion){
    //grid = eecs467::OccupancyGrid(g->widthInMeters(),g->heightInMeters(),g->metersPerCell());
	grid = *g;
	enlarge_obstacle(wall_expansion);	
	/*for(int y=0;y<g->widthInCells();++y){
        for(int x=0;x<g->heightInCells();++x){
			uint8_t odds = g->logOdds(x,y);       
			grid.setLogOdds(x,y,odds);
            if(odds > 8){
                grid.setLogOdds(x-1,y,127);
                grid.setLogOdds(x+1,y,127);
                grid.setLogOdds(x,y-1,127);
                grid.setLogOdds(x,y+1,127);
                //tmp.setLogOdds(x-1,y-1,127);
                //tmp.setLogOdds(x+1,y+1,127);
                //tmp.setLogOdds(x-1,y+1,127);
                //tmp.setLogOdds(x+1,y-1,127);
            } 
        }
    }*/
}

void path_planning::enlarge_obstacle(int wall_expansion){
    eecs467::OccupancyGrid tmp(grid.widthInMeters(),grid.heightInMeters(),grid.metersPerCell());
    for(int y=0;y<grid.widthInCells();++y){
        for(int x=0;x<grid.heightInCells();++x){
            tmp.setLogOdds(x,y,grid.logOdds(x,y));
            if(grid.logOdds(x,y) > 8){
				for (int p=-wall_expansion; p <= wall_expansion; p++)
				{
					for (int k=-wall_expansion; k<=wall_expansion; k++)
					{					
						tmp.setLogOdds(x+k, y+p, 127);
					}
				}			
            } 
        }
    }
    for(int y=0;y<grid.widthInCells();++y){
        for(int x=0;x<grid.heightInCells();++x){
            grid.setLogOdds(x,y,tmp.logOdds(x,y));
        }
    }
}

std::vector<eecs467::Point<int>> path_planning::find_frontier(eecs467::Point<float> best){
    eecs467::Point<int> start = eecs467::global_position_to_grid_cell(best,grid);
    std::deque<eecs467::Point<int>> q;
    std::vector<eecs467::Point<int>> path;
    breadth_point tmp;
    tmp.visited = 0;
    tmp.prev_coord.x = -1;
    tmp.prev_coord.y = -1;
    std::vector<breadth_point> isVisted(grid.widthInCells()*grid.heightInCells(),tmp);
    q.push_back(start);
    isVisted[convertTo1D(start)].visited = 1;
    isVisted[convertTo1D(start)].prev_coord.x = -1; //RMC -1,-1 is a valid grid cell
    isVisted[convertTo1D(start)].prev_coord.y = -1;
    while(!q.empty()){
        eecs467::Point<int> curr = q.front();
        q.pop_front();
        //printf("front: %d %d\n",curr.x,curr.y);
        //here to modify unexplore qualification
        if(abs(grid.logOdds(curr.x,curr.y)) <= 8 && check_gray_range(curr,3)){
            path.push_back(curr);
            //printf("find result at: %d %d \n",curr.x,curr.y);
            break;
        }
        for(int i =0; i<8 ; ++i){
            int n=0;
            int m=0;
            switch (i){
                case 0:
                    n = -1;
                    break;
                case 1:
                    m = 1;
                    break;
                case 2:
                    n = 1;
                    break;
                case 3:
                    m = -1;
                    break;
				case 4:
					n = -1;
					m = 1;
					break;
				case 5:
					n = -1;
					m = -1;
					break;
				case 6:
					n = 1;
					m = 1;
					break;
				case 7:
					n = 1;
					m = -1;
					break;
            }
            eecs467::Point<int> new_point;
            new_point.x = curr.x+n;
            new_point.y = curr.y+m;
            //printf("here\n");
            if(grid.isCellInGrid(new_point.x,new_point.y)){
                //printf("isVisted.size(): %d convertTo1D: %d\n",isVisted.size(),convertTo1D(new_point));
                //printf("visited: %d\n",isVisted[convertTo1D(new_point)].visited);
                //here to modify path qualification

                //RMC - where does 8 come from?
                if(grid.logOdds(new_point.x,new_point.y) <= 3 && isVisted[convertTo1D(new_point)].visited == 0){
                    q.push_back(new_point);
                    //printf("add: %d %d \n",new_point.x,new_point.y);
                    isVisted[convertTo1D(new_point)].prev_coord = curr;
                    isVisted[convertTo1D(new_point)].visited=1;
                }
            }
        }
    }
    if(path.empty()){
        return path;
    }
    //unwind
    while(path.back() != start){
        eecs467::Point<int> next = path.back();
        path.push_back(isVisted[convertTo1D(next)].prev_coord);
    }   
    return path;
}

/* to slow
   bool path_planning::check_gray_range(eecs467::Point<int> p,int range){
//check the 10x10 area if there are enough gray to explore
int x_left = p.x-range;
if(x_left < 0){
return false;
}
int y_up = p.y-range;
if(y_up < 0){
return false;
}
int x_right = p.x+range;
if(x_right > grid->widthInCells()){
return false;
}
int y_down = p.y+range;
if(y_down > grid->heightInCells()){
return false;
}
//printf("runs here\n");
int total_size = (2*range+1)*(2*range+1);
int x_offset = -(p.x - range);
int y_offset = -(p.y - range);
std::vector<int> visited(total_size,0);
int gray_counter = 0;
std::deque<eecs467::Point<int>> q;
q.push_back(p);
visited[convertTo1D(eecs467::transform(p,x_offset,y_offset,0))] = 1;
while(!q.empty()){
eecs467::Point<int> curr = q.front();
q.pop_front();
if(abs(grid->logOdds(curr.x,curr.y)) <= 20){
++gray_counter;
visited[convertTo1D(eecs467::transform(curr,x_offset,y_offset,0))] = 1;
for(int i =0; i<4 ; ++i){
int n=0;
int m=0;
switch (i){
case 0:
n = -1;
break;
case 1:
m = 1;
break;
case 2:
n = 1;
break;
case 3:
m = -1;
break;
}
eecs467::Point<int> new_point;
new_point.x = curr.x+n;
new_point.y = curr.y+m;
if(new_point.x >= x_left && 
new_point.x <= x_right && 
new_point.y >= y_up && 
new_point.y <= y_down && 
visited[convertTo1D(eecs467::transform(new_point,x_offset,y_offset,0))] == 0){
q.push_back(new_point);       
}
}
}
}
if(gray_counter >= total_size/2){
return true;
}
return false;
}
*/

bool path_planning::check_gray_range(eecs467::Point<int> p,int range){
    int x = -range;
    int y = -range;
    int gray_counter = 0;
    while(y <= range){
        while(x <= range){
            if(abs(grid.logOdds(p.x+x,p.y+y)) < 8){
                ++gray_counter;
            }
            ++x;
        }
        x = -range;
        ++y;
    }
    int total_size = (range*2+1)*(range*2+1);
    if(gray_counter >= (total_size/3)-2){
        return true;
    }
    return false;
}


inline int path_planning::convertTo1D(eecs467::Point<int> p){
    return p.y*grid.widthInCells()+p.x;
} 
