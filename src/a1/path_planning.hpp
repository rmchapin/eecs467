#ifndef A_STAR_ALG_HPP
#define A_STAR_ALG_HPP
#include <queue>
#include <vector>
#include <unordered_map>
#include <utility>
#include "mapping/occupancy_grid.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "math/point.hpp"


class path_planning{
public:
    std::vector<eecs467::Point<int>> search(eecs467::Point<int> start,eecs467::Point<int> end);
    std::vector<eecs467::Point<int>> get_neighbor(eecs467::Point<int> p);
    float heuristic_cost_estimation(eecs467::Point<int> p1, eecs467::Point<int> p2);
    std::vector<eecs467::Point<int>> reconstruct_path(std::unordered_map<eecs467::Point<int>,eecs467::Point<int>> &came_from,eecs467::Point<int> current, eecs467::Point<int> start);
    eecs467::OccupancyGrid get_grid();
    void updateGrid(eecs467::OccupancyGrid g);
    int get_cost(eecs467::Point<int> p);
private:
    eecs467::OccupancyGrid grid;
};

/*class a_star_point{
public:    
    a_star_point(){
        point.x = 0;
        point.y = 0;
        cost = 0;
    }
    a_star_point(eecs467::Point<int> p, int c){
        point = p;
        cost = c;
    }
    eecs467::Point<int> point;
    int cost;
};

class a_star_point_comparator()
{
    bool operator(const a_star_point &p1, const a_star_point &p2) const{
        return p1.cost < p2.cost;
    }
}*/

template<typename T, typename Number=int>
struct PriorityQueue {
  typedef std::pair<Number, T> PQElement;
  std::priority_queue<PQElement, std::vector<PQElement>,
                 std::greater<PQElement>> elements;

  inline bool empty() { return elements.empty(); }

  inline void put(T item, Number priority) {
    elements.emplace(priority, item);
  }

  inline T get() {
    T best_item = elements.top().second;
    elements.pop();
    return best_item;
  }
};

#endif
