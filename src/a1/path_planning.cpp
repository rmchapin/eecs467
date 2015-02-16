#include "path_planning.hpp"


//based on grid index
std::vector<eecs467::Point<int>> path_planning::search(eecs467::Point<int> start,eecs467::Point<int> end){
    std::unordered_map<eecs467::Point<int>, eecs467::Point<int>> came_from;
    std::unordered_map<eecs467::Point<int>, int> cost_so_far;
    //std::priority_queue<a_star_point,a_star_point_comparator> frontier;
    PriorityQueue<eecs467::Point<int>> frontier;
    frontier.put(start,0);
    came_from[start] = start;
    cost_so_far[start] = 0;
    while(!frontier.empty()){
        auto current = frontier.get();
        if(current == end){
            break;
        }
        std::deque<eecs467::Point<int>> neighbor = get_neighbor(current);
        for(auto next:neighbor){
            int new_cost = cost_so_far[current] + get_cost(current,next);
            if(!cost_so_far.count(next) || new_cost < cost_so_far[next]){
                cost_so_far[next] = new_cost;
                int priority = new_cost + heuristic_cost_estimation(next,goal);
                frontier.put(next,priority);
                came_from[next] = current;
            }
        }
    }
    return reconstruct_path(came_from,current,start);
}

std::vector<eecs467::Point<int>> path_planning::get_neighbor(eecs467::Point<int> p){
    std::vector<eecs467::Point<int>> neighbor;
    if(grid.isCellInGrid(p.x-1,p.y) && grid.logOdds(p.x-1,p.y)<0){
        neighbor.push_back(eecs467::Point<int>(p.x-1,p.y));
    }
    if(grid.isCellInGrid(p.x+1,p.y) && grid.logOdds(p.x+1,p.y)<0){
        neighbor.push_back(eecs467::Point<int>(p.x+1,p.y));
    }
    if(grid.isCellInGrid(p.x,p.y-1) && grid.logOdds(p.x,p.y-1)<0){
        neighbor.push_back(eecs467::Point<int>(p.x,p.y-1));
    }
    if(grid.isCellInGrid(p.x,p.y+1) && grid.logOdds(p.x,p.y+1)<0){
        neighbor.push_back(eecs467::Point<int>(p.x,p.y+1));
    }
}

//distance is typically a heuristic cost estimation
float path_planning::heuristic_cost_estimation(eecs467::Point<int> p1, eecs467::Point<int> p2){
    return abs(p1.x-p2.x)+abs(p1.y-p2.y);
}

std::vector<eecs467::Point<int>> path_planning::reconstruct_path(std::unordered_map<eecs467::Point<int>,eecs467::Point<int>> &came_from,eecs467::Point<int> current,eecs467::Point<int> start){
    std::vector<eecs467::Point<int>> path;
    path.push_back(current);
    while(current != start){
        current = came_from[current];
        path.push_back(current);
    }    
    return path;
}
eecs467::OccupancyGrid path_planning::get_grid(){
    return grid;
}

void path_planning::updateGrid(eecs467::OccupancyGrid g){
    grid = g;
}

//this is the moving cost
int path_planning::get_cost(eecs467::Point<int> p){
    //shift the log Odds and times 0.5 to give a cost on position
    int cost = (grid.logOdds(p.x,p.y)+128)/2;
    if(grid.isCellInGrid(p.x,p.y)){
        cost += 1;
    }
    else{
        //give a large cost to prevent from moving to the out side
        cost += 100;
    }
    return ;
}
