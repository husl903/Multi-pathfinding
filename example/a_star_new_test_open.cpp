#include <fstream>
#include <iostream>
#include <queue>
#include <rnd/stonesngems.h>
#include <random>
#include <algorithm>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/resource.h>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <libMultiRobotPlanning/a_star_t.hpp>
#include <libMultiRobotPlanning/a_star_t_1.hpp>
#include <libMultiRobotPlanning/timer.hpp>
#include <libMultiRobotPlanning/vectorCache.hpp>
#include <libMultiRobotPlanning/queueCache.hpp>

#define GemGoal

#define DEBUG

using namespace stonesngems;
using namespace stonesngems::util;
using libMultiRobotPlanning::AStar;
using libMultiRobotPlanning::AStarT;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;

GameParameters params = kDefaultGameParams;
RNDGameState state_game;
static vectorCache<int8_t> gridCache;
static vectorCache<int> indexCache;
std::vector<size_t> rand_x;
std::vector<size_t> rand_y;
std::vector<std::vector<int>> random_act;
int MODEL_MOVE = 1;
// std::vector<int8_t> grid_temp;
// std::vector<std::vector<int8_t>> goal_locations;
// int num_state = 0;

struct State {
  State(int x, int y, int time, std::vector<int8_t>&grid, std::vector<int>&need_update_index) : x(x), y(y), 
         time(time), grid(grid), need_update_index(need_update_index) {}  
  State(int x, int y, int time, uint64_t zorb_hash, std::vector<int8_t>&grid, std::vector<int>&need_update_index) : x(x), y(y), 
         time(time), grid(grid), zorb_hash(zorb_hash), need_update_index(need_update_index) {}
  State(int x, int y, int time, uint64_t zorb_hash) : x(x), y(y), 
         time(time), zorb_hash(zorb_hash) {}
  ~State(){}
                    
  // State(int x, int y, int time, std::vector<int8_t>&grid) : x(x), y(y), time(time), grid(grid) {}           
  // State(int x, int y) : x(x), y(y) {}

  State(const State&) = default;
  State(State&&) = default;
  State& operator=(const State&) = default;
  State& operator=(State&&) = default;

  bool operator==(const State& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << "(" << s.x << "," << s.y << ")";
  }

  unsigned int dir = 0xff;
  int x;
  int y;
  int time;
  uint64_t zorb_hash;
  std::vector<int8_t> grid;
  std::vector<int> need_update_index;
  LocalState localstate;
  bool is_falling = false;
  bool canRollLeft = false;
  bool canRollRight = false;
  bool is_wait = false;
  bool goal_roll = false;
  int index_gem = -1;
  int gem_x = -1;
  int gem_y = -1;
  int f = -1;
  int h = -1;
};

namespace std {
template <>
struct hash<State> {
  size_t operator()(const State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    // seed^=rand_x[s.x];
    // seed^=rand_y[s.y];
    // boost::hash_combine(seed, s.time);
//    boost::hash_combine(seed, s.zorb_hash);
    boost::hash_combine(seed, s.gem_x);
    boost::hash_combine(seed, s.gem_y);
    return seed;
  }
};
}  // namespace std

struct Location {
  Location(int x, int y) : x(x), y(y) {}
  int x;
  int y;

  bool operator<(const Location& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator!=(const Location& other) const {
	 return std::tie(x, y) != std::tie(other.x, other.y);
  }
  bool operator==(const Location& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const Location& c) {
    return os << "(" << c.x << "," << c.y << ")";
  }
};

struct StateLight {
  StateLight(int x, int y) : x(x), y(y) {}
  StateLight(int x, int y, int time, unsigned int dir) : x(x), y(y), time(time), dir(dir){}
  StateLight(int x, int y, int time) : x(x), y(y), time(time){}

  int x;
  int y;
  int time;
  unsigned int dir = 0x00;
  int f = 0;
  int move_h = 0;
  uint64_t zorb_hash;
  // std::vector<int8_t> grid;

  bool operator<(const StateLight& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator!=(const StateLight& other) const {
	 return std::tie(x, y) != std::tie(other.x, other.y);
  }
  bool operator==(const StateLight& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const StateLight& c) {
    return os << "(" << c.x << "," << c.y << ")";
  }
};

namespace std {
template <>
struct hash<Location> {
  size_t operator()(const Location& s) const {
    size_t seed = 0;
    // boost::hash_combine(seed, s.x);
    // boost::hash_combine(seed, s.y);
    seed^=rand_x[s.x];
    seed^=rand_y[s.y];
    return seed;
  }
};
}  // namespace std

namespace std {
template <>
struct hash<StateLight> {
  size_t operator()(const StateLight& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    // seed^=rand_x[s.x];
    // seed^=rand_y[s.y];
    return seed;
  }
};
}  // namespace std

enum class Action {
  Up,
  Down,
  Left,
  Right,
  Wait,
};

std::ostream& operator<<(std::ostream& os, const Action& a) {
  switch (a) {
    case Action::Up:
      os << "Up";
      break;
    case Action::Down:
      os << "Down";
      break;
    case Action::Left:
      os << "Left";
      break;
    case Action::Right:
      os << "Right";
      break;
    case Action::Wait:
      os << "Wait";
      break;      
  }
  return os;
}

class Environment {
 public:
  Environment(size_t dimx, size_t dimy, std::unordered_set<State> obstacles,
              Location goal, std::vector<std::vector<int>> m_eHeuristic)
              // std::vector<std::vector<StateLight>> m_goal_location_table)
      : m_dimx(dimx),
        m_dimy(dimy),
        m_obstacles(std::move(obstacles)),
        m_goal(std::move(goal)),
        m_eHeuristic(std::move(m_eHeuristic))
        // m_goal_location_table(std::move(m_goal_location_table))  // NOLINT
  {}
    Environment(size_t dimx, size_t dimy,
              Location goal, std::vector<std::vector<int>> m_eHeuristic)
      : m_dimx(dimx),
        m_dimy(dimy),
        m_goal(std::move(goal)),
        m_eHeuristic(std::move(m_eHeuristic))  // NOLINT
  {}

    Environment(size_t dimx, size_t dimy,
              Location goal, std::vector<std::vector<int>> m_eHeuristic,
              std::vector<std::vector<int>>m_eHeuristic_overall_borders, 
              std::vector<std::vector<int>>m_eHeuristic_overall_border_location, 
              std::vector<std::vector<int>>m_eHeuristic_goalArea,
              std::vector<std::vector<std::vector<int>>>m_border_distance, 
              std::vector<std::vector<std::vector<int>>>m_goal_distance,
              std::vector<std::vector<int>>m_goal_index, 
              std::vector<Location>m_border_loc,
              std::vector<int8_t> m_grid)
//              std::vector<std::vector<StateLight>> m_goal_location_table)
      : m_dimx(dimx),
        m_dimy(dimy),
        m_goal(std::move(goal)),
        m_eHeuristic(std::move(m_eHeuristic)),
        m_eHeuristic_overall_borders(std::move(m_eHeuristic_overall_borders)),
        m_eHeuristic_overall_border_location(std::move(m_eHeuristic_overall_border_location)),
        m_eHeuristic_goalArea(std::move(m_eHeuristic_goalArea)),
        m_border_distance(std::move(m_border_distance)),
        m_goal_distance(std::move(m_goal_distance)),        
        m_goal_index(std::move(m_goal_index)),
        m_border_loc(std::move(m_border_loc)),
        m_grid(std::move(m_grid))
//        m_goal_location_table(std::move(m_goal_location_table))  // NOLINT
  {}  
    Environment(size_t dimx, size_t dimy,
              Location goal, std::vector<std::vector<int>> m_eHeuristic, std::vector<int8_t> m_grid)
//              std::vector<std::vector<StateLight>> m_goal_location_table)
      : m_dimx(dimx),
        m_dimy(dimy),
        m_goal(std::move(goal)),
        m_eHeuristic(std::move(m_eHeuristic)),
        m_grid(std::move(m_grid))
//        m_goal_location_table(std::move(m_goal_location_table))  // NOLINT
  {}    
  int admissibleHeuristic(const StateLight& s) {
    return 0;
  }

  int admissibleHeuristic(const State& s) {
//        else return std::abs(s.x - s.gem_x) + std::abs(s.y - s.gem_y) + 1;
// #ifdef GemGoal    
//     if(s.is_falling){
//        if(s.x >= s.gem_x){is_roll_fall = true; std::cout << ",roll," << "\n"; 
//     }else if(s.canRollLeft) { 
//       if(s.y <= s.gem_y){is_roll_fall = true;  std::cout << ",roll," << "\n"; return std::abs(s.x - s.gem_x) + std::abs(s.y - s.gem_y)/2;}
//       else return std::abs(s.x - s.gem_x) + 1 + std::abs(s.y - s.gem_y);
//     }else if(s.canRollRight) { 
//       if(s.y >= s.gem_y) {is_roll_fall = true; std::cout << ",roll," << "\n"; return std::abs(s.x - s.gem_x)  + std::abs(s.y - s.gem_y)/2;}
//       else return std::abs(s.x - s.gem_x) + 1 + std::abs(s.y - s.gem_y);
//     }else{
//       if(s.gem_x == m_goal.x && s.gem_y == m_goal.y && isExact) return m_eHeuristic[s.x][s.y];
//       else return std::abs(s.x - s.gem_x) + std::abs(s.y - s.gem_y);
//     }
// #endif    
//     return std::abs(s.x - m_goal.x) + std::abs(s.y - m_goal.y);

#ifdef GemGoal

    int time_step = s.time;
    // std::cout <<"Size " << m_goal_location_table.size() << "\n";
    /*if(s.time < m_goal_location_table.size()){
      int min_heuristic = 0;
      for(int ii = 0; ii < m_goal_location_table[s.time].size(); ii++){
        int md = (std::abs(s.x - m_goal_location_table[s.time][ii].x) + std::abs(s.y - m_goal_location_table[s.time][ii].y));
        if(ii == 0) min_heuristic = md;
        else if(md < min_heuristic) min_heuristic = md;
      }
      // std::cout << "Heuristic \n";
      return min_heuristic;
    }else*/
    if(is_manhattan_distance == 1) //manhattan distance
    {
      int md = (std::abs(s.x - s.gem_x) + std::abs(s.y - s.gem_y));

        int border_x = m_eHeuristic_overall_border_location[s.x][s.y] / m_dimy;
        int border_y = m_eHeuristic_overall_border_location[s.x][s.y] % m_dimy;

        int dist_1 = abs(m_goal.x - border_x) + abs(m_goal.y - border_y);
        int dist_2 = abs(s.gem_x - border_x) + abs(s.gem_y - border_y);
        int temp_res = 0;
        if((m_eHeuristic_goalArea[border_x- 1][border_y] != -1 && m_eHeuristic_goalArea[border_x + 1][border_y]
          && m_eHeuristic_goalArea[border_x][border_y + 1] != -1 && m_eHeuristic_goalArea[border_x][border_y - 1] != -1) 
          || (border_x == m_goal.x && border_y == m_goal.y)){
            temp_res = md;
        }else{
          temp_res = m_eHeuristic_overall_borders[s.x][s.y] + dist_2 - dist_1;
        }
        int temp = abs(s.gem_x - m_goal.x) + abs(s.gem_y - m_goal.y);
        // std::cout << "border x,y " << border_x << ", " << border_y << ", dist1, " << dist_1 << ", dist2, " << dist_2 << " \n";
        // std::cout <<"state Man, x: " << s.x << ", y: " << s.y <<", oriG, " << m_goal.x << ", " << m_goal.y << ", currG, " << s.gem_x << ", " << s.gem_y << ",dist1," << dist_1 << ",dist2," << dist_2 << ", res, "  << (md + 1)/2  << ", overall borders, " << (temp_res + 1)/2 << ", "<< m_eHeuristic_overall_borders[s.x][s.y]  << ", move, " << temp << "\n";
      return (md +1)/2;
      if(md%2 == 1) return md/2+1;
      else return md/2;      
    }else if(is_manhattan_distance == 2){// full border

      int md = (std::abs(s.x - s.gem_x) + std::abs(s.y - s.gem_y));
      // std::cout << "s.x, s.y " << s.x << ", " << s.y << ",1D, " << m_eHeuristic_goalArea.size() << ", 2D, " << m_eHeuristic_goalArea[0].size() << "\n" ;
      if(m_eHeuristic_goalArea[s.x][s.y] != -1){
            
            int temp = (abs(m_eHeuristic_goalArea[s.gem_x][s.gem_y] - m_eHeuristic_goalArea[s.x][s.y]) + 1) / 2;
            for(int i = 0; i < m_border_distance.size(); i++){
              if(m_border_distance[i][s.x][s.y] != -1)
              temp = std::max(temp, (abs(m_border_distance[i][s.gem_x][s.gem_y] - m_border_distance[i][s.x][s.y]) + 1)/2);
            }
            return temp;
      }else{
          int res_min = 1000000000;
          int index = -1;
          for(int i = 0; i < m_border_distance.size(); i++){
            if(m_border_distance[i][s.x][s.y] != -1){
              if(index == -1){
                int temp = m_border_distance[i][s.x][s.y] + std::max(0, (m_border_distance[i][s.gem_x][s.gem_y] - m_border_distance[i][s.x][s.y] + 1)/2);
	//std::cout <<"index " << i << ", fullb， temp," << temp << ",res_min, " << res_min << ", " << m_border_distance[i][s.x][s.y] << ", " << m_border_distance[i][s.gem_x][s.gem_y] << " \n";
                res_min = temp;
                index = i;
              }else{
                int temp = m_border_distance[i][s.x][s.y] + std::max(0, (m_border_distance[i][s.gem_x][s.gem_y] - m_border_distance[i][s.x][s.y] + 1)/2);
                if(temp < res_min) {
                  res_min = temp;
                  index = i;
	//std::cout <<"Index" << i << ", fullb， temp," << temp << ",res_min, " << res_min << ", " << m_border_distance[i][s.x][s.y] << ", " << m_border_distance[i][s.gem_x][s.gem_y] << " \n";
                }
              }
            }
          }
//          std::cout << ", Man, " << (md+1)/2 << ",NewH, " << res_min << ",x,y" << s.x << ", " << s.y << "," << m_border_distance[index][s.gem_x][s.gem_y] << ", " << m_border_distance[index][s.x][s.y] << " Testhere\n";
          return res_min;
      }
    }else if(is_manhattan_distance == 3){ //max(TD(s,b), MD(s,g)/2)
      if(m_eHeuristic_goalArea[s.x][s.y] != -1){
          return (abs(m_eHeuristic_goalArea[s.gem_x][s.gem_y] - m_eHeuristic_goalArea[s.x][s.y]) + 1) / 2;
      }else{
        int closest_s_b = -1;
        int index = -1;
        for(int i = 0; i < m_border_distance.size(); i++){
          if(m_border_distance[i][s.x][s.y] != -1){
            if(closest_s_b == -1) closest_s_b = m_border_distance[i][s.x][s.y];
            else if(m_border_distance[i][s.x][s.y] < closest_s_b) closest_s_b = m_border_distance[i][s.x][s.y];
          }
        }
//       if(index == -1 || closest_s_b == -1) return 1000000000;
        int md = (std::abs(s.x - s.gem_x) + std::abs(s.y - s.gem_y));
        return std::max((md + 1)/2, closest_s_b);
      }
    }else if(is_manhattan_distance == 4){
      // printf("\n");
      // std::cout << "s.x, s.y " << s.x << ", " << s.y << ",1D, " << m_eHeuristic_goalArea.size() << ", 2D, " << m_eHeuristic_goalArea[0].size() << "\n" ;
      if(m_eHeuristic_goalArea[s.x][s.y] != -1){ // max(TD(s,b),TDBD)
          // printf("1111\n");
          return (abs(m_eHeuristic_goalArea[s.gem_x][s.gem_y] - m_eHeuristic_goalArea[s.x][s.y]) + 1) / 2;
      }else{
        // printf("222 %d %d index_goal %d %d, m_border_distance, %d %d %d\n", s.gem_x, s.gem_y, m_goal_index.size(), m_goal_index[0].size(), m_border_distance.size(), m_border_distance[0].size(), m_border_distance[0][0].size());

        int closest_s_b = -1;
        int index = -1;
        int index_goal_curr = m_goal_index[s.gem_x][s.gem_y];

        if(index_goal_curr == -1){
          printf("****************  %d %d *******************\n", s.gem_x, s.gem_y);
        }
        assert(index_goal_curr != -1);        
        int closest_s_b_goal = -1;
        //  printf("333 %d %d index_goal %d %d, m_border_distance, %d %d %d %d\n", s.gem_x, s.gem_y, m_goal_index.size(), m_goal_index[0].size(), m_border_distance.size(), m_border_distance[0].size(), m_border_distance[0][0].size(), index_goal_curr);

        for(int i = 0; i < m_border_distance.size(); i++){
//          printf("Here %d\n", m_border_loc.size());
          if (m_border_distance[i].empty()) continue;
          if(s.x < 0 || s.x >= m_border_distance[i].size()) { printf("Test \n");continue;}
          if(s.y < 0 || s.y >= m_border_distance[i][s.x].size()) {printf("Test 2\n"); continue;}
            if(m_border_distance[i][s.x][s.y] != -1){
              if(closest_s_b == -1){
                closest_s_b =  m_border_distance[i][s.x][s.y];
                index = i;
              }else if(m_border_distance[i][s.x][s.y] < closest_s_b){
                closest_s_b =  m_border_distance[i][s.x][s.y];
                index = i;
              }else if(m_border_distance[i][s.x][s.y] == closest_s_b){
                if(m_border_distance[index][s.gem_x][s.gem_y] > m_border_distance[i][s.gem_x][s.gem_y]) index = i;
              }
            }
//          printf("333 %d %d index_goal %d %d, m_border_distance, %d %d %d\n", s.gem_x, s.gem_y, m_goal_index.size(), m_goal_index[0].size(), m_border_distance.size(), m_border_distance[0].size(), m_border_distance[0][0].size());
            if(i < 0 || i >= m_border_loc.size()) continue;
            // if(index_goal_curr < 0 || index_goal_curr >= m_goal_distance.size()) continue;
            int distance_goal_border = m_goal_distance[index_goal_curr][m_border_loc[i].x][m_border_loc[i].y];
            if(distance_goal_border != -1 && (distance_goal_border < closest_s_b_goal || closest_s_b_goal == -1)){
              closest_s_b_goal = distance_goal_border;
            }
        }
        if(index == -1 || closest_s_b == -1) return 1000000000;
        // if(s.x == 10 && s.y == 23){
        //   printf("\n index %d, bx,y, %d, %d, closeb: %d \n", index, m_border_loc[index].x, m_border_loc[index].y, closest_s_b);
        // }
        if(closest_s_b_goal < closest_s_b ) return closest_s_b;
        else return closest_s_b + (closest_s_b_goal - closest_s_b + 1)/2;
        // if(closest_s_b > distance_goal_border){
        //   return closest_s_b;
        // }else{ return (closest_s_b + (distance_goal_border - closest_s_b + 1)/2);}
      }
    }else if(is_manhattan_distance == 5){ //max(TD(s,b), TD(s,g)/2)
      if(m_eHeuristic_goalArea[s.x][s.y] != -1){
          return (abs(m_eHeuristic_goalArea[s.gem_x][s.gem_y] - m_eHeuristic_goalArea[s.x][s.y]) + 1) / 2;
      }else{
        int closest_s_b = -1;
        int index = -1;
        for(int i = 0; i < m_border_distance.size(); i++){
          if(m_border_distance[i][s.x][s.y] != -1){
            if(closest_s_b == -1) closest_s_b = m_border_distance[i][s.x][s.y];
            else if(m_border_distance[i][s.x][s.y] < closest_s_b){
              closest_s_b = m_border_distance[i][s.x][s.y];
            }
          }
        }
//       if(index == -1 || closest_s_b == -1) return 1000000000;
        int index_goal_curr = m_goal_index[s.gem_x][s.gem_y];
        int distance_goal_s = m_goal_distance[index_goal_curr][s.x][s.y];
        return std::max(closest_s_b, (distance_goal_s + 1)/2);
      }
    }
    
#endif    
    return std::abs(s.x - m_goal.x)/2 + std::abs(s.y - m_goal.y)/2;    
  }

  int admissibleHeuristicRe(const State& s, int gem_x, int gem_y, int& flag) {
    if(flag == 1){
       if(s.x >= gem_x) return std::abs(s.x - gem_x)/2 + std::abs(s.y - gem_y);
       else return std::abs(s.x - gem_x) + std::abs(s.y - gem_y) + 1;      
    }else if(flag == 2){
      if(s.y <= gem_y) return std::abs(s.x - gem_x) + std::abs(s.y - gem_y)/2;
      else return std::abs(s.x - gem_x) + 1 + std::abs(s.y - gem_y);      
    }else if(flag == 3){
      if(s.y >= gem_y) return std::abs(s.x - gem_x)  + std::abs(s.y - gem_y)/2;
      else return std::abs(s.x - gem_x) + 1 + std::abs(s.y - gem_y);      
    }
    return 0;
  }      

  bool isSolution(const StateLight& s) { 
    return false;
  }

  bool isSolution(const State& s) { 
#ifdef GemGoal
    // return s.x == m_goal.x && s.y == m_goal.y; 
    return s.x == s.gem_x && s.y == s.gem_y; 
    return s.x == s.index_gem / m_dimy && s.y == s.index_gem % m_dimy; 
#endif
    return s.x == m_goal.x && s.y == m_goal.y; 
  }

//whether needs const
  void getNeighbors(State& s,
                    std::vector<Neighbor<State, Action, int> >& neighbors, int f_value) {
    neighbors.clear();
    // if(num_expand > 12000) return;
#ifdef DEBUG    
    std::cout << "Current state "<< s.x <<", " << s.y  <<",time, " << s.time <<",h, " << ", hash, " << s.zorb_hash << ", size, " << s.grid.size() << "s.gemx, " << s.gem_x << ",s.gemy," << s.gem_y << "----------------------------------"<< std::endl;
    // << ", f, " << f_value << ", g , " << s.time << ", h , " << f_value - s.time <<
#endif
    // if( s.x == 3 && s.y == 25)
    // {
    //   // std::cout << "Current state "<< s.x <<", " << s.y << ", time " << s.time << ", " << s.grid.size()  << "----------------------------------"<< std::endl;
    //   for (int h = 0; h < state_game.board.rows; ++h)
    //   {
    //     for (int w = 0; w < state_game.board.cols; ++w) 
    //     {
    //       std::cout << kCellTypeToElement[s.grid[h * state_game.board.cols + w] + 1].id;
    //       // std::cout << kCellTypeToElement.at(state_game.board.grid[h * state_game.board.cols + w]).id;
    //     }
    //     std::cout << std::endl;
    //   }
    // }
    // if(s.zorb_hash != zorb_hash_temp) std::cout << "ERROR \n";
    // std::cout << "Current state "<< s.x <<", " << s.y << ", time " << s.time << ", hash, " << s.zorb_hash  << ", new-hash " << zorb_hash_temp << ", size, " << s.grid.size()  << "----------------------------------"<< std::endl;

    int index = s.x * m_dimy + s.y;
//    sort(s.need_update_index.begin(), s.need_update_index.end());

    // state_game.board.grid.assign(s.grid.begin(), s.grid.end());
    // state_game.board.need_update_index.clear();
    // state_game.board.need_update_index.assign(s.need_update_index.begin(), s.need_update_index.end());
    // state_game.resetLocalState(s.localstate);
    // state_game.board.agent_pos = index;
    // state_game.board.agent_idx = index;
    // state_game.curr_gem_index = s.index_gem;
    // state_game.board.zorb_hash = s.zorb_hash;
    uint64_t zorb_hash_temp = s.zorb_hash;
//    state_game.apply_action(0);
    // int new_gem_x = s.gem_x, new_gem_y = s.gem_y;
    int gem_act = random_act[s.gem_x][s.gem_y] + 1; //0, wait, 1, up, 2, down, 3 left, 4, right;
    if(gem_act >= 5) gem_act = 4;
    // printf("gem_act %d\n", gem_act);
    // if(gem_act == 1 && random_act[s.gem_x - 1][s.gem_y] != -1){
    //   state_game.board.grid[state_game.board.cols * s.gem_x + s.gem_y] = 1;
    //   state_game.board.grid[state_game.board.cols * (s.gem_x - 1) + s.gem_y] = 5;
    //   new_gem_x = s.gem_x - 1;
    // }else if(gem_act == 2 && random_act[s.gem_x + 1][s.gem_y] != -1){
    //   state_game.board.grid[state_game.board.cols * s.gem_x + s.gem_y] = 1;
    //   state_game.board.grid[state_game.board.cols * (s.gem_x + 1) + s.gem_y] = 5;
    //   new_gem_x = s.gem_x + 1;
    // }else if(gem_act == 3 && random_act[s.gem_x][s.gem_y - 1] != -1){
    //   state_game.board.grid[state_game.board.cols * s.gem_x + s.gem_y] = 1;
    //   state_game.board.grid[state_game.board.cols * s.gem_x + s.gem_y - 1] = 5;
    //   new_gem_y = s.gem_y - 1;
    // }else if(gem_act == 4 && random_act[s.gem_x][s.gem_y + 1] != -1){
    //   state_game.board.grid[state_game.board.cols * s.gem_x + s.gem_y] = 1;
    //   state_game.board.grid[state_game.board.cols * s.gem_x + s.gem_y + 1] = 5;
    //   new_gem_y = s.gem_y + 1;
    // }
    // std::cout << "new_gemx, new_gemy " << new_gem_x << ", " <<new_gem_y << "\n";
    // state_game.curr_gem_index = new_gem_x * state_game.board.cols + new_gem_y;
    // state_game.init_hash();
    if(1){
      // printf("Wait\n");
      State wait(s.x, s.y, s.time + 1, 0);
//      wait.grid.assign(state_game.board.grid.begin(), state_game.board.grid.end());
      // wait.need_update_index.assign(state_game.board.need_update_index.begin(), state_game.board.need_update_index.end());
      // wait.localstate = state_game.local_state;
#ifdef GemGoal      
      wait.index_gem =  s.gem_x * state_game.board.cols + s.gem_y;
      wait.gem_x = s.gem_x;
      wait.gem_y = s.gem_y;
      // wait.canRollLeft = state_game.CanRollLeft(wait.index_gem);
      // wait.canRollRight = state_game.CanRollRight(wait.index_gem);
      // wait.is_falling = state_game.IsType(wait.index_gem, kElEmpty, Directions::kDown);
      if(s.index_gem != wait.index_gem && (abs(s.x - s.gem_x) + abs(s.y - s.gem_y)) != 1){
        wait.goal_roll = true;
        //   std::cout << s.x << ", " << s.y << ", time " << s.time << ",wait gemxy, " << s.gem_x << ", " << s.gem_y<< "\n";
        // for (int h = 0; h < state_game.board.rows; ++h) {
        //   for (int w = 0; w < state_game.board.cols; ++w) {
        //     std::cout << kCellTypeToElement[wait.grid[h * state_game.board.cols + w] + 1].id;
        //     // std::cout << kCellTypeToElement.at(state_game.board.grid[h * state_game.board.cols + w]).id;
        //   }
        //   std::cout << std::endl;
        // }       
      }
      else wait.goal_roll = false;
#endif      
      // if(wait.zorb_hash != zorb_hash_temp)
      wait.is_wait = true;
      neighbors.emplace_back(Neighbor<State, Action, int>(wait, Action::Wait, 1));
      num_generated++;
#ifdef DEBUG         
      std::cout << "Neighbor  "<<", " << wait.zorb_hash << ", " << wait.x <<", " << wait.y << ", time " << wait.time << ",h, " <<admissibleHeuristic(wait)  << ", manh, " << (abs(s.gem_x - wait.x) + abs(s.gem_y - wait.y) + 1)/2 << ",goal, " << wait.gem_x << ", y, " << wait.gem_y << ",bGoal," << s.gem_x << ", " << s.gem_y <<", "<< ", " << s.grid.size() << std::endl;
#endif
      // num_state++;
    }

    if(m_grid[index - m_dimy] != 18 && m_grid[index - m_dimy] != 19){
      // state_game.board.need_update_index.clear();
      // state_game.board.need_update_index.assign(s.need_update_index.begin(), s.need_update_index.end());
      // state_game.board.agent_pos = index;
      // state_game.board.agent_idx = index;
      // state_game.resetLocalState(s.localstate);
      // state_game.board.grid[s.x * state_game.board.cols + s.y] = 1;
      // state_game.board.grid[(s.x - 1) * state_game.board.cols + s.y] = 0;
      int new_gem_x = s.gem_x, new_gem_y = s.gem_y;
      if(random_act[s.gem_x - 1][s.gem_y] != -1 && MODEL_MOVE == 1){
//        state_game.board.grid[s.gem_x * state_game.board.cols + s.gem_y] = 1;
//        state_game.board.grid[(s.gem_x - 1) * state_game.board.cols + s.gem_y] = 5;
          new_gem_x = s.gem_x - 1;
          new_gem_y = s.gem_y;
      }else if(random_act[s.gem_x + 1][s.gem_y] != -1 && MODEL_MOVE == 2){
        new_gem_x = s.gem_x + 1;
        new_gem_y = s.gem_y;
      }else if(random_act[s.gem_x][s.gem_y - 1] != -1 && MODEL_MOVE == 3){
        new_gem_x = s.gem_x;
        new_gem_y = s.gem_y - 1;
      }else if(random_act[s.gem_x][s.gem_y + 1] != -1 && MODEL_MOVE == 4){
        new_gem_x = s.gem_x;
        new_gem_y = s.gem_y + 1;
      }
      // state_game.init_hash();
//      state_game.board.zorb_hash = zorb_hash_temp;
//      state_game.apply_action(1);
      // if ((index - m_dimy) == state_game.board.agent_pos || state_game.board.agent_pos == kAgentPosExit) {
        State up(s.x - 1, s.y, s.time + 1, 0);
        // up.grid.assign(state_game.board.grid.begin(), state_game.board.grid.end());
        // up.need_update_index.assign(state_game.board.need_update_index.begin(), state_game.board.need_update_index.end());
        // up.localstate = state_game.local_state;
  #ifdef GemGoal            
        up.index_gem =  new_gem_x * state_game.board.cols + new_gem_y;
        up.gem_x = new_gem_x;
        up.gem_y = new_gem_y;
        // up.canRollLeft = state_game.CanRollLeft(up.index_gem);
        // up.canRollRight = state_game.CanRollRight(up.index_gem);
        // up.is_falling = state_game.IsType(up.index_gem, kElEmpty, Directions::kDown);
        if(s.index_gem != up.index_gem && (abs(s.x - s.gem_x) + abs(s.y - s.gem_y)) != 1) {
          up.goal_roll = true;
          // std::cout << s.x << ", " << s.y << ", time " << s.time << ",up gemxy, " << s.gem_x << ", " << s.gem_y<< "\n";
          // for (int h = 0; h < state_game.board.rows; ++h) {
          //   for (int w = 0; w < state_game.board.cols; ++w) {
          //     std::cout << kCellTypeToElement[up.grid[h * state_game.board.cols + w] + 1].id;
          //   // std::cout << kCellTypeToElement.at(state_game.board.grid[h * state_game.board.cols + w]).id;
          //   }
          //   std::cout << std::endl;
          // }              
        }
        else up.goal_roll = false;        
  #endif      
        neighbors.emplace_back(Neighbor<State, Action, int>(up, Action::Up, 1));
        num_generated++;
        // std::cout << state_game.local_state.gems_collected << " gems " << std::endl;
#ifdef DEBUG          
        std::cout << "Neighbor  " << ", " << up.zorb_hash << ", " << up.x <<", " << up.y << ", time " << up.time << ",h, " <<admissibleHeuristic(up)  << ", manh, " << (abs(s.gem_x - up.x) + abs(s.gem_y - up.y) + 1)/2 << ",goal, " << up.gem_x << ", y, " << up.gem_y << ",bGoal," << s.gem_x << ", " << s.gem_y <<", " << ", " << s.grid.size() << std::endl;
#endif        
        // num_state++;
      // }
      // state_game.board.grid[s.x * state_game.board.cols + s.y] = 0;
      // state_game.board.grid[(s.x - 1) * state_game.board.cols + s.y] = 1;
      // if(random_act[s.gem_x - 1][s.gem_y] != -1){
      //   state_game.board.grid[s.gem_x * state_game.board.cols + s.gem_y] = 5;
      //   state_game.board.grid[(s.gem_x - 1) * state_game.board.cols + s.gem_y] = 1;        
      // }      
    }

    if(m_grid[index + m_dimy] != 18 && m_grid[index + m_dimy] != 19){
//      if(s.zorb_hash == 7598063330544506766) std::cout << "Down 111\n";
      // state_game.board.grid.clear();
      // state_game.board.grid.assign(s.grid.begin(), s.grid.end());
      // state_game.board.need_update_index.clear();
      // state_game.board.need_update_index.assign(s.need_update_index.begin(), s.need_update_index.end());
      // state_game.board.agent_pos = index;
      // state_game.board.agent_idx = index;
      // state_game.resetLocalState(s.localstate);
      // state_game.board.grid[s.x * state_game.board.cols + s.y] = 1;
      // state_game.board.grid[(s.x + 1) * state_game.board.cols + s.y] = 0;
      int new_gem_x = s.gem_x, new_gem_y = s.gem_y;
      if(random_act[s.gem_x + 1][s.gem_y] != -1 && MODEL_MOVE == 1){
        // state_game.board.grid[s.gem_x * state_game.board.cols + s.gem_y] = 1;
        // state_game.board.grid[(s.gem_x + 1) * state_game.board.cols + s.gem_y] = 5;
        new_gem_x = s.gem_x + 1;
        new_gem_y = s.gem_y;        
      }else if(random_act[s.gem_x - 1][s.gem_y] != -1 && MODEL_MOVE == 2){
        new_gem_x = s.gem_x - 1;
        new_gem_y = s.gem_y;
      }else if(random_act[s.gem_x][s.gem_y + 1] != -1 && MODEL_MOVE == 3){
        new_gem_x = s.gem_x;
        new_gem_y = s.gem_y + 1;
      }else if(random_act[s.gem_x][s.gem_y - 1] != -1 && MODEL_MOVE == 4){
        new_gem_x = s.gem_x;
        new_gem_y = s.gem_y - 1;
      }
      // state_game.init_hash();
      // state_game.apply_action(3);
      // if (index + m_dimy == state_game.board.agent_pos || state_game.board.agent_pos == kAgentPosExit) {
        State down(s.x + 1, s.y, s.time + 1, state_game.board.zorb_hash);
        // down.grid.assign(state_game.board.grid.begin(), state_game.board.grid.end());
        // down.need_update_index.assign(state_game.board.need_update_index.begin(), state_game.board.need_update_index.end());
        // down.localstate = state_game.local_state;
#ifdef GemGoal      
        down.index_gem = new_gem_y * state_game.board.cols + new_gem_y;
        down.gem_x = new_gem_x;
        down.gem_y = new_gem_y;      
        // down.canRollLeft = state_game.CanRollLeft(down.index_gem);
        // down.canRollRight = state_game.CanRollRight(down.index_gem); 
        // down.is_falling = state_game.IsType(down.index_gem, kElEmpty, Directions::kDown);
        if(s.index_gem != down.index_gem && (abs(s.x - s.gem_x) + abs(s.y - s.gem_y)) != 1){
          down.goal_roll = true;
          // std::cout << s.x << ", " << s.y << ", time " << s.time << ",down gemxy, " << s.gem_x << ", " << s.gem_y<< "\n";
          // for (int h = 0; h < state_game.board.rows; ++h) {
          //   for (int w = 0; w < state_game.board.cols; ++w) {
          //     std::cout << kCellTypeToElement[down.grid[h * state_game.board.cols + w] + 1].id;
          //   // std::cout << kCellTypeToElement.at(state_game.board.grid[h * state_game.board.cols + w]).id;
          //   }
          //   std::cout << std::endl;
          // }            
        }  
        else down.goal_roll = false;
#endif            
        neighbors.emplace_back(Neighbor<State, Action, int>(down, Action::Down, 1));
        num_generated++;
#ifdef DEBUG         
        std::cout << "Neighbor  " << ", " << down.zorb_hash << ", " << down.x <<", " << down.y << ", time " << down.time << ",h, " <<admissibleHeuristic(down)  << ", manh, " << (abs(s.gem_x - down.x) + abs(s.gem_y - down.y) + 1)/2 << ",goal, " << down.gem_x << ", y, " << down.gem_y << ",bGoal," << s.gem_x << ", " << s.gem_y <<", " << ", " << s.grid.size() << std::endl;
#endif
        // num_state++;   
      // }
      // state_game.board.grid[s.x * state_game.board.cols + s.y] = 0;
      // state_game.board.grid[(s.x + 1) * state_game.board.cols + s.y] = 1;      
      // if(random_act[s.gem_x + 1][s.gem_y] != -1){
      //   state_game.board.grid[s.gem_x * state_game.board.cols + s.gem_y] = 5;
      //   state_game.board.grid[(s.gem_x + 1) * state_game.board.cols + s.gem_y] = 1;
      // }        
    }

    if(m_grid[index - 1] != 18 && m_grid[index - 1] != 19){

      // state_game.board.need_update_index.clear();
      // state_game.board.need_update_index.assign(s.need_update_index.begin(), s.need_update_index.end());
      // state_game.board.agent_pos = index;
      // state_game.board.agent_idx = index;
      // state_game.resetLocalState(s.localstate);
      // state_game.board.grid[s.x * state_game.board.cols + s.y] = 1;
      // state_game.board.grid[ s.x * state_game.board.cols + s.y - 1] = 0;
      int new_gem_x = s.gem_x, new_gem_y = s.gem_y;
      if(random_act[s.gem_x][s.gem_y - 1] != -1 && MODEL_MOVE == 1){
        state_game.board.grid[s.gem_x * state_game.board.cols + s.gem_y] = 1;
        state_game.board.grid[s.gem_x * state_game.board.cols + s.gem_y - 1] = 5;
        new_gem_y = s.gem_y - 1;
      }else if(random_act[s.gem_x][s.gem_y + 1] != -1 && MODEL_MOVE == 2){
        new_gem_y = s.gem_y + 1;
        new_gem_x = s.gem_x;
      }else if(random_act[s.gem_x + 1][s.gem_y] != -1 && MODEL_MOVE == 3){
        new_gem_x = s.gem_x + 1;
        new_gem_y = s.gem_y;
      }else if(random_act[s.gem_x - 1][s.gem_y] != -1 && MODEL_MOVE == 4){
        new_gem_x = s.gem_x - 1;
        new_gem_y = s.gem_y;
      }
      // state_game.init_hash();
//      state_game.apply_action(4);
      // if (index - 1 == state_game.board.agent_pos || state_game.board.agent_pos == kAgentPosExit) {
        State left(s.x, s.y - 1, s.time + 1, state_game.board.zorb_hash);
        // left.grid.assign(state_game.board.grid.begin(), state_game.board.grid.end());
        // left.need_update_index.assign(state_game.board.need_update_index.begin(), state_game.board.need_update_index.end());
        // left.localstate = state_game.local_state;
#ifdef GemGoal      
        left.index_gem = new_gem_x * state_game.board.cols + new_gem_y;
        left.gem_x = new_gem_x;
        left.gem_y = new_gem_y;
        // left.canRollLeft = state_game.CanRollLeft(left.index_gem);
        // left.canRollRight = state_game.CanRollRight(left.index_gem);
        // left.is_falling = state_game.IsType(left.index_gem, kElEmpty, Directions::kDown);
        if(s.index_gem != left.index_gem && (abs(s.x - s.gem_x) + abs(s.y - s.gem_y)) != 1){
          left.goal_roll = true;
          // std::cout << s.x << ", " << s.y << ", time " << s.time << ",left gemxy, " << s.gem_x << ", " << s.gem_y<< "\n";
          // for (int h = 0; h < state_game.board.rows; ++h) {
          //   for (int w = 0; w < state_game.board.cols; ++w) {
          //     std::cout << kCellTypeToElement[left.grid[h * state_game.board.cols + w] + 1].id;
          //   // std::cout << kCellTypeToElement.at(state_game.board.grid[h * state_game.board.cols + w]).id;
          //   }
          //   std::cout << std::endl;
          // }            
        }
        else left.goal_roll = false;        
#endif      
        neighbors.emplace_back(Neighbor<State, Action, int>(left, Action::Left, 1));
        num_generated++;
#ifdef DEBUG         
        std::cout << "Neighbor  " << "," << left.zorb_hash << "," << left.x <<", " << left.y << ", time " << left.time << ",h, " <<admissibleHeuristic(left)  << ", manh, " << (abs(s.gem_x - left.x) + abs(s.gem_y - left.y) + 1)/2 << ",goal, " << left.gem_x << ", y, " << left.gem_y << ",bGoal," << s.gem_x << ", " << s.gem_y <<", " << s.grid.size() << std::endl;
#endif
        // num_state++;
      // }
      // state_game.board.grid[s.x * state_game.board.cols + s.y] = 0;
      // state_game.board.grid[ s.x * state_game.board.cols + s.y - 1] = 1;
      // if(random_act[s.gem_x][s.gem_y - 1] != -1){
      //   state_game.board.grid[s.gem_x * state_game.board.cols + s.gem_y] = 5;
      //   state_game.board.grid[s.gem_x * state_game.board.cols + s.gem_y - 1] = 1;
      // }      
    }

    if(m_grid[index + 1] != 18 && m_grid[index + 1] != 19){
      // state_game.board.need_update_index.assign(s.need_update_index.begin(), s.need_update_index.end());
      // state_game.board.agent_pos = index;
      // state_game.board.agent_idx = index;
      // state_game.resetLocalState(s.localstate);
      // state_game.board.grid[s.x * state_game.board.cols + s.y] = 1;
      // state_game.board.grid[ s.x * state_game.board.cols + s.y + 1] = 0;
      int new_gem_x = s.gem_x, new_gem_y = s.gem_y;
      if(random_act[s.gem_x][s.gem_y + 1] != -1 && MODEL_MOVE == 1){
        state_game.board.grid[s.gem_x * state_game.board.cols + s.gem_y] = 1;
        state_game.board.grid[s.gem_x * state_game.board.cols + s.gem_y + 1] = 5;
        new_gem_y = s.gem_y + 1;
      }else if(random_act[s.gem_x][s.gem_y - 1] != -1 && MODEL_MOVE == 2){
        new_gem_x = s.gem_x;
        new_gem_y = s.gem_y - 1;
      }else if(random_act[s.gem_x - 1][s.gem_y] != -1 && MODEL_MOVE == 3){
        new_gem_x = s.gem_x - 1;
        new_gem_y = s.gem_y;
      }else if(random_act[s.gem_x + 1][s.gem_y] != -1 && MODEL_MOVE == 4){
        new_gem_x = s.gem_x + 1;
        new_gem_y = s.gem_y;
      }
      // state_game.init_hash();
//      state_game.apply_action(2);
      // if (index + 1 == state_game.board.agent_pos || state_game.board.agent_pos == kAgentPosExit) {
        State right(s.x, s.y + 1, s.time + 1, state_game.board.zorb_hash);
        // right.grid.assign(state_game.board.grid.begin(), state_game.board.grid.end());
        // right.need_update_index.assign(state_game.board.need_update_index.begin(), state_game.board.need_update_index.end());
        // right.localstate = state_game.local_state;
#ifdef GemGoal      
        right.index_gem =  new_gem_x * state_game.board.cols + new_gem_y;
        right.gem_x = new_gem_x;
        right.gem_y = new_gem_y;
        // right.canRollLeft = state_game.CanRollLeft(right.index_gem);
        // right.canRollRight = state_game.CanRollRight(right.index_gem);
        // right.is_falling = state_game.IsType(right.index_gem, kElEmpty, Directions::kDown);
        if(s.index_gem != right.index_gem && (abs(s.x - s.gem_x) + abs(s.y - s.gem_y)) != 1) {
          right.goal_roll = true;
          // std::cout << s.x << ", " << s.y << ", time " << s.time << ",right gemxy, " << s.gem_x << ", " << s.gem_y<< "\n";
          // for (int h = 0; h < state_game.board.rows; ++h) {
          //   for (int w = 0; w < state_game.board.cols; ++w) {
          //     std::cout << kCellTypeToElement[right.grid[h * state_game.board.cols + w] + 1].id;
          //   // std::cout << kCellTypeToElement.at(state_game.board.grid[h * state_game.board.cols + w]).id;
          //   }
          //   std::cout << std::endl;
          // }              
        }
        else right.goal_roll = false;
#endif      
        neighbors.emplace_back(Neighbor<State, Action, int>(right, Action::Right, 1));
        num_generated++;
#ifdef DEBUG         
        std::cout << "Neighbor  " << "," << right.zorb_hash << "," << right.x <<", " << right.y << ", time " << right.time << ",h, " <<admissibleHeuristic(right)  << ", manh, " << (abs(s.gem_x - right.x) + abs(s.gem_y - right.y) + 1)/2 << ",goal, " << right.gem_x << ", y, " << right.gem_y << ",bGoal," << s.gem_x << ", " << s.gem_y <<", " << s.grid.size() << std::endl;
#endif
        if(right.zorb_hash == 9097503194969756300){
          for (int h = 0; h < state_game.board.rows; ++h) {
            for (int w = 0; w < state_game.board.cols; ++w) {
              std::cout << kCellTypeToElement[right.grid[h * state_game.board.cols + w] + 1].id;
                // std::cout << kCellTypeToElement.at(state_game.board.grid[h * state_game.board.cols + w]).id;
            }
            std::cout << std::endl;
          } 
        }
        // num_state++;
      // }
      // state_game.board.grid[s.x * state_game.board.cols + s.y] = 0;
      // state_game.board.grid[ s.x * state_game.board.cols + s.y + 1] = 1; 
      // if(random_act[s.gem_x][s.gem_y + 1] != -1){
      //   state_game.board.grid[s.gem_x * state_game.board.cols + s.gem_y] = 5;
      //   state_game.board.grid[s.gem_x * state_game.board.cols + s.gem_y + 1] = 1;
      // }                 
    }
  }
  
void getNeighborsT_temp(StateLight& s,
                    std::vector<Neighbor<StateLight, Action, int> >& neighbors, int f_value) {
    neighbors.clear();
#ifdef DEBUG    
    std::cout << "Current state "<< s.x <<", " << s.y  << ", hash, " << s.zorb_hash << ", size, " << "----------------------------------"<< std::endl;
    // << ", f, " << f_value << ", g , " << s.time << ", h , " << f_value - s.time <<
#endif

    int index = s.x * m_dimy  + s.y;
    if(s.dir & 0x10){
      StateLight succ_temp(s.x, s.y, s.time + 1);
      succ_temp.dir = 0xff;
      neighbors.emplace_back(Neighbor<StateLight, Action, int>(succ_temp, Action::Wait, 1));      
    }
    if(s.dir & 0x02){ //down
      if(m_grid[(s.x + 1) * m_dimy + s.y] != 18 && m_grid[(s.x + 1) * m_dimy + s.y] != 19){
        StateLight succ_temp(s.x + 1, s.y, s.time + 1);
        succ_temp.dir = 0x10;
        if(m_grid[(s.x + 2) * m_dimy + s.y] != 18 && m_grid[(s.x + 2) * m_dimy + s.y] != 19) 
          succ_temp.dir |= 0x02;
        if(m_grid[(s.x + 1) * m_dimy + s.y - 1] != 18 && m_grid[(s.x + 1) * m_dimy + s.y - 1] != 19)
          succ_temp.dir |= 0x04;
        if(m_grid[(s.x + 1) * m_dimy + s.y + 1] != 18 && m_grid[(s.x + 1) * m_dimy + s.y + 1] != 19)
          succ_temp.dir |= 0x08;
        neighbors.emplace_back(Neighbor<StateLight, Action, int>(succ_temp, Action::Down, 1));
      }
    }
    if(s.dir & 0x04){//left
      if(m_grid[(s.x ) * m_dimy + s.y - 1] != 18 && m_grid[(s.x) * m_dimy + s.y -1] != 19){
        StateLight succ_temp(s.x,  s.y - 1, s.time + 1);
        succ_temp.dir = 0x00;
        if(s.move_h > 0) s.move_h--;
        if(m_grid[(s.x + 1) * m_dimy + s.y - 1] != 18 && m_grid[(s.x + 1) * m_dimy + s.y - 1] != 19){
          succ_temp.dir = 0x02;
        }

        /*if(s.y -2 >= 0){
          if(m_grid[s.x * m_dimy + s.y - 2] != 18 && m_grid[s.x * m_dimy + s.y - 2] != 19){
            int num_move = 0;
            int ob_move = -1;
            bool flag_move  = false;
            for(int row_index = s.x + 1; row_index < m_dimx; row_index++){
              int temp_index = row_index * m_dimy + s.y - 1;
              if(!flag_move && (m_grid[temp_index] == 3 || m_grid[temp_index] == 4 || 
                m_grid[temp_index] == 5 || m_grid[temp_index] == 6 ||
                m_grid[temp_index] == 39 || m_grid[temp_index] == 40 ||
                m_grid[temp_index] == 3 || m_grid[temp_index] == 4 || 
                m_grid[temp_index] == 5 || m_grid[temp_index] == 6 ||
                m_grid[temp_index] == 39 || m_grid[temp_index] == 40)){
                num_move++;
              }else flag_move = true;
              if(m_grid[temp_index] == 18 || m_grid[temp_index] == 19){
                ob_move = (temp_index - s.x);
                break;
              }
            }

            if(num_move > 0 || s.move_h > 0){
              succ_temp.dir |= 0x04;
              if(ob_move == -1){
                if(num_move != 0) succ_temp.move_h = num_move;
                else succ_temp.move_h = s.move_h;
              }else{
                if(num_move != 0) succ_temp.move_h = num_move;
                else succ_temp.move_h = std::min(ob_move, s.move_h);
              }
              // if(num_move > s.move_h) succ_temp.move_h = num_move;
              // else succ_temp.move_h = s.move_h;
              // succ_temp.move_h = num_move;
            }
            // if(m_grid[(s.x + 1) * m_dimy + s.y - 1] == 3 || m_grid[(s.x + 1) * m_dimy + s.y - 1] == 4 || 
            //   m_grid[(s.x + 1) * m_dimy + s.y - 1] == 5 || m_grid[(s.x + 1) * m_dimy + s.y - 1] == 6 ||
            //   m_grid[(s.x + 1) * m_dimy + s.y - 1] == 39 || m_grid[(s.x + 1) * m_dimy + s.y - 1] == 40 ||
            //   m_grid[(s.x + 1) * m_dimy + s.y] == 3 || m_grid[(s.x + 1) * m_dimy + s.y] == 4 || 
            //   m_grid[(s.x + 1) * m_dimy + s.y] == 5 || m_grid[(s.x + 1) * m_dimy + s.y] == 6 ||
            //   m_grid[(s.x + 1) * m_dimy + s.y] == 39 || m_grid[(s.x + 1) * m_dimy + s.y] == 40)
            //   succ_temp.dir |= 0x04;
          }
        }*/
        neighbors.emplace_back(Neighbor<StateLight, Action, int>(succ_temp, Action::Left, 1));
      }
    }
    if(s.dir & 0x08){//right
      if(m_grid[(s.x ) * m_dimy + s.y + 1] != 18 && m_grid[(s.x) * m_dimy + s.y + 1] != 19){
        StateLight succ_temp(s.x,  s.y + 1, s.time + 1);
        if(m_grid[(s.x + 1) * m_dimy + s.y + 1] != 18 && m_grid[(s.x + 1) * m_dimy + s.y + 1] != 19){
          succ_temp.dir = 0x02;     
        }
        /*if(s.y + 2 < m_dimy){
          if(m_grid[s.x * m_dimy + s.y + 2] != 18 && m_grid[s.x * m_dimy + s.y + 2] != 19){

            int num_move = 0;
            int ob_move = -1;
            bool flag_move  = false;         
            for(int row_index = s.x + 1; row_index < m_dimx; row_index++){
              int temp_index = row_index * m_dimy + s.y + 1;
              if(!flag_move && (m_grid[temp_index] == 3 || m_grid[temp_index] == 4 || 
                m_grid[temp_index] == 5 || m_grid[temp_index] == 6 ||
                m_grid[temp_index] == 39 || m_grid[temp_index] == 40 ||
                m_grid[temp_index] == 3 || m_grid[temp_index] == 4 || 
                m_grid[temp_index] == 5 || m_grid[temp_index] == 6 ||
                m_grid[temp_index] == 39 || m_grid[temp_index] == 40)){
                num_move++;
              }else flag_move = true;
              if(m_grid[temp_index] == 18 || m_grid[temp_index] == 19){
                ob_move = (temp_index - s.x);
                break;
              }            
            }

            if(num_move > 0 || s.move_h > 0){
              succ_temp.dir |= 0x08;
              if(ob_move == -1){
                if(num_move != 0) succ_temp.move_h = num_move;
                else succ_temp.move_h = s.move_h;
              }else{
                if(num_move != 0) succ_temp.move_h = num_move;
                else succ_temp.move_h = std::min(ob_move, s.move_h);
              }
              // succ_temp.move_h = num_move;
              // if(num_move > succ_temp.move_h) succ_temp.move_h = num_move;
              // else succ_temp.move_h = s.move_h;
            }
            // if(m_grid[(s.x + 1) * m_dimy + s.y + 1] == 3 || m_grid[(s.x + 1) * m_dimy + s.y + 1] == 4 || 
            //   m_grid[(s.x + 1) * m_dimy + s.y + 1] == 5 || m_grid[(s.x + 1) * m_dimy + s.y + 1] == 6 ||
            //   m_grid[(s.x + 1) * m_dimy + s.y + 1] == 39 || m_grid[(s.x + 1) * m_dimy + s.y + 1] == 40 ||
            //   m_grid[(s.x + 1) * m_dimy + s.y] == 3 || m_grid[(s.x + 1) * m_dimy + s.y] == 4 || 
            //   m_grid[(s.x + 1) * m_dimy + s.y] == 5 || m_grid[(s.x + 1) * m_dimy + s.y] == 6 ||
            //   m_grid[(s.x + 1) * m_dimy + s.y] == 39 || m_grid[(s.x + 1) * m_dimy + s.y] == 40)
            //   succ_temp.dir |= 0x08;       
          } 
        }*/
        neighbors.emplace_back(Neighbor<StateLight, Action, int>(succ_temp, Action::Right, 1));
      }
    }

  }


void getNeighborsT(StateLight& s,
                    std::vector<Neighbor<StateLight, Action, int> >& neighbors, int f_value) {
    neighbors.clear();
#ifdef DEBUG    
    std::cout << "Current state "<< s.x <<", " << s.y  << ", hash, " << s.zorb_hash << ", size, " << "----------------------------------"<< std::endl;
#endif

    int index = s.x * m_dimy  + s.y;
    if(m_grid[(s.x - 1) * m_dimy + s.y] != 18 && m_grid[(s.x - 1) * m_dimy + s.y] != 19){ //up
      StateLight succ_temp(s.x - 1, s.y, s.time + 1);
      succ_temp.dir = 0xff;
      neighbors.emplace_back(Neighbor<StateLight, Action, int>(succ_temp, Action::Up, 1));            
    }
    if(m_grid[(s.x + 1) * m_dimy + s.y] != 18 && m_grid[(s.x + 1) * m_dimy + s.y] != 19){ //down
       StateLight succ_temp(s.x + 1, s.y, s.time + 1);
       neighbors.emplace_back(Neighbor<StateLight, Action, int>(succ_temp, Action::Down, 1));
    }

    if(m_grid[(s.x ) * m_dimy + s.y - 1] != 18 && m_grid[(s.x) * m_dimy + s.y -1] != 19){ //left
        StateLight succ_temp(s.x,  s.y - 1, s.time + 1);
        neighbors.emplace_back(Neighbor<StateLight, Action, int>(succ_temp, Action::Down, 1));      
    }

    if(m_grid[(s.x ) * m_dimy + s.y + 1] != 18 && m_grid[(s.x) * m_dimy + s.y + 1] != 19){ //right
        StateLight succ_temp(s.x,  s.y + 1, s.time + 1);
        neighbors.emplace_back(Neighbor<StateLight, Action, int>(succ_temp, Action::Right, 1));
    }
    
  }

  void onExpandNode(const State& /*s*/, int /*fScore*/, int /*gScore*/) {
    num_expand++;
  }

  void onDiscover(const State& /*s*/, int /*fScore*/, int /*gScore*/) {}

 public:
  bool stateValid(const State& s) {
    return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
           m_obstacles.find(s) == m_obstacles.end();
  }
  int num_expand = 0;
  int num_generated = 0;
  bool isExact = false;
  bool is_roll_fall = false;
  int is_manhattan_distance = 5;

 private:
  int m_dimx;
  int m_dimy;
  std::unordered_set<State> m_obstacles;
  Location m_goal;
  std::vector<std::vector<int>> m_eHeuristic;
  std::vector<std::vector<int>> m_eHeuristic_overall_borders;
  std::vector<std::vector<int>> m_eHeuristic_overall_border_location;
  std::vector<std::vector<int>> m_eHeuristic_goalArea;
  std::vector<std::vector<std::vector<int>>> m_border_distance;
  std::vector<std::vector<std::vector<int>>> m_goal_distance;  
   std::vector<std::vector<int>> m_goal_index;
   std::vector<Location> m_border_loc;

//  std::vector<std::vector<StateLight>> m_goal_location_table;
  std::vector<int8_t> m_grid;
};


void getShortestPathHeuristic(std::vector<std::vector<int>> &eHeuristic, const std::vector<std::vector<bool>> map_obstacle, int goalX, int goalY, int dimx, int dimy)
{
	int xx[5] = {0, 0, -1, 1};
	int yy[5] = {1, -1, 0, 0};
  for(int i = 0; i < eHeuristic.size(); i++){
    for(int j = 0; j < eHeuristic[i].size(); j++){
      eHeuristic[i][j] = -1;
    }
  }
//  std::cout <<eHeuristic.size() <<  "Test 11111111 " << goalX << ", " << goalY << std::endl;
	eHeuristic[goalX][goalY] = 0;
  Location goal(goalX, goalY);
	std::queue<Location> que;
	que.push(goal);
	while (true)
	{
		int queSize = que.size();
		if (queSize == 0)
			break;
		for (int i = 0; i < queSize; i++)
		{
			Location curr = que.front();
			int currValue = eHeuristic[curr.x][curr.y];
			que.pop();
			for (int ii = 0; ii < 4; ii++)
			{
				Location nei(curr.x + xx[ii], curr.y + yy[ii]);
				if (curr.x + xx[ii] < 0 || curr.y + yy[ii] < 0 || curr.x + xx[ii] >= dimx || curr.y + yy[ii] >= dimy || map_obstacle[nei.x][nei.y])
					continue;
				if (eHeuristic[nei.x][nei.y] == -1)
				{
					eHeuristic[nei.x][nei.y] = currValue + 1;
					que.push(nei);
				}
			}
		}
	}
}


int getShortestPathHeuristic(std::vector<std::vector<int>> &eHeuristic, const std::vector<std::vector<int>> map_obstacle, int goalX, int goalY, int dimx, int dimy)
{
	int xx[5] = {0, 0, -1, 1};
	int yy[5] = {1, -1, 0, 0};

  std::cout <<eHeuristic.size() <<  "Test222222 " << goalX << ", " << goalY  <<", " << map_obstacle[goalX][goalY] << std::endl;
  for(int i = 0; i < eHeuristic.size(); i++){
    for(int j = 0; j < eHeuristic[0].size(); j++){
      eHeuristic[i][j] = -1;
    }
    //     std::cout << map_obstacle[i][j] << ", ";
    // std::cout << "\n";
  }
	eHeuristic[goalX][goalY] = 0;
  Location goal(goalX, goalY);
	std::queue<Location> que;
	que.push(goal);
	while (true)
	{
		int queSize = que.size();
		if (queSize == 0)
			break;
		for (int i = 0; i < queSize; i++)
		{
			Location curr = que.front();
			int currValue = eHeuristic[curr.x][curr.y];
			que.pop();
			for (int ii = 0; ii < 4; ii++)
			{
				Location nei(curr.x + xx[ii], curr.y + yy[ii]);
				if (curr.x + xx[ii] < 0 || curr.y + yy[ii] < 0 || curr.x + xx[ii] >= dimx || curr.y + yy[ii] >= dimy || map_obstacle[nei.x][nei.y] == -1)
					continue;
				if (eHeuristic[nei.x][nei.y] == -1)
				{
          if(nei.x == 0 || nei.x == dimx - 1 || nei.y == 0 || nei.y == dimy - 1 ||
            map_obstacle[nei.x + 1][nei.y] == -1 || map_obstacle[nei.x - 1][nei.y] == -1 ||
            map_obstacle[nei.x][nei.y + 1] == -1 || map_obstacle[nei.x][nei.y - 1] ==- 1){
              std::cout << currValue + 1 << "Wrong----- TESSSSSSSSSSSSSSSSSSSSSSSS\n";
              return currValue + 1;
            }
					eHeuristic[nei.x][nei.y] = currValue + 1;
					que.push(nei);
				}
			}
		}
	}
  return 0;
}

void PowerLocation(int goalX, int goalY, const std::vector<int8_t>& grid, int dimx, int dimy)
{
  int indexGoal = goalX * dimy + goalY;
  if( indexGoal < dimx * dimy)
  {
    const Element ele = kCellTypeToElement[grid[indexGoal + dimy] + 1];
    std::cout <<  kCellTypeToElement[grid[indexGoal] + 1].id << "\n";
    if(ele.properties & ElementProperties::kRounded)
    {
      int temp_index = indexGoal;
      while( temp_index - 1 > 0 && temp_index < dimx * dimy && temp_index - 1 + dimy > 0 && temp_index - 1 + dimy < dimx * dimy 
             && temp_index - 1 + 2*dimy > 0 && temp_index - 1 + 2*dimy < dimx * dimy){
        if(grid[temp_index - 1] == 1 && (kCellTypeToElement[grid[temp_index - 1 + dimy] + 1].properties & ElementProperties::kRounded) 
           && (kCellTypeToElement[grid[temp_index - 1 + 2 * dimy] + 1].properties & ElementProperties::kRounded)){
          temp_index = temp_index - 1 + dimy;
          std::cout << "Test ------------\n";
          continue;
        }else {
          // if()
          break;
        }
      }
      std::cout << "Round\n";
    }
    std::cout << ele.properties << "\n";
    // if(ele.properties )
  }
}

void StimulateTest(std::vector<std::vector<StateLight>> &goal_location_table, int goalX, int goalY, int m_dimy, int m_dimx, const std::vector<int8_t>& grid){
 
//  std::vector<std::vector<StateLight>> goal_location_table;
  std::vector<StateLight> current_time_step;
  int time_step = 0;
  current_time_step.push_back(StateLight(goalX, goalY, 0, 0xff));
  goal_location_table.push_back(current_time_step);
  current_time_step.clear();
  while(1){
    for(int i = 0; i < goal_location_table[time_step].size(); i++){
      StateLight temp = goal_location_table[time_step][i];
      // std::cout <<"Current   :" <<  temp.x << ", " << temp.y << ", " << ",time," << temp.time << " ******************************\n";
      // if(temp.dir & 0x10){ // wait
      //   StateLight succ_temp(temp.x, temp.y, temp.time + 1);
      //   succ_temp.dir = 0xff;
      //   current_time_step.push_back(succ_temp);
      //   // std::cout <<"Successor  :" <<  succ_temp.x << ", " << succ_temp.y << ", " << ",time," << succ_temp.time << "\n";
      // }
      if(temp.dir & 0x02){ // down
        StateLight succ_temp(temp.x + 1,  temp.y, temp.time + 1);
        // succ_temp.dir = 0x10;
        if(grid[(temp.x + 2) * m_dimy + temp.y] != 18 && grid[(temp.x + 1) * m_dimy + temp.y] != 19) 
        succ_temp.dir |= 0x02;
        if(grid[(temp.x + 1) * m_dimy + temp.y - 1] != 18 && grid[(temp.x + 1) * m_dimy + temp.y] != 19)
        succ_temp.dir |= 0x04;
        if(grid[(temp.x + 1) * m_dimy + temp.y + 1] != 18 && grid[(temp.x + 1) * m_dimy + temp.y] != 19)
        succ_temp.dir |= 0x08;
        current_time_step.push_back(succ_temp);     
        // std::cout <<"Successor  :" <<  succ_temp.x << ", " << succ_temp.y << ", " << ",time," << succ_temp.time << ", dir, " << succ_temp.dir << "\n";
      }
      if(temp.dir & 0x04){ // left
        StateLight succ_temp(temp.x,  temp.y - 1, temp.time + 1);
        succ_temp.dir = 0x02;
        current_time_step.push_back(succ_temp);  
        // std::cout <<"Successor  :" <<  succ_temp.x << ", " << succ_temp.y << ", " << ",time," << succ_temp.time << "\n";
      }
      if(temp.dir & 0x08){ // right
        StateLight succ_temp(temp.x,  temp.y + 1, temp.time + 1);
        succ_temp.dir = 0x02;
        current_time_step.push_back(succ_temp);   
        // std::cout <<"Successor  :" <<  succ_temp.x << ", " << succ_temp.y << ", " << ",time," << succ_temp.time << "\n";
      }      
    }
    goal_location_table.push_back(current_time_step);
    current_time_step.clear();
    time_step++;
    if(time_step == 10) break;
  }

  // for(int i = 0; i < goal_location_table.size(); i++){
  //   for(int ii = 0; ii < goal_location_table[i].size(); ii++){
  //     std::cout << goal_location_table[i][ii].x << ", " << goal_location_table[i][ii].y << ", time, " << goal_location_table[i][ii].time << std::endl;
  //   }
  // }

}

void StimulateTest2(std::vector<std::vector<int>> &eHeuristic, int goalX, int goalY, int m_dimy, int m_dimx, const std::vector<int8_t>& grid){
 
//  std::vector<std::vector<StateLight>> goal_location_table;
  std::cout <<eHeuristic.size() <<  "Test" << goalX << ", " << goalY << std::endl;
  std::cout << "StimulateTest2\n";
	eHeuristic[goalX][goalY] = 0;
  StateLight goal(goalX, goalY, 0, 0xff);
	std::queue<StateLight> que;
	que.push(goal);

  while(!que.empty()){
    StateLight temp = que.front();
    que.pop();
      std::cout << temp.x << ", " << temp.y << ", time, " << temp.time << ", " << temp.dir << "\n";
      if(temp.dir & 0x02){ // down
        StateLight succ_temp(temp.x + 1,  temp.y, temp.time + 1);
        if(eHeuristic[temp.x + 1][temp.y] != -1) continue;
        eHeuristic[temp.x + 1][temp.y] = temp.time + 1;
        std::cout << eHeuristic[temp.x + 1][temp.y] <<  ", " << temp.x + 1 << temp.y << " Test\n";
        if(grid[(temp.x + 2) * m_dimy + temp.y] != 18 && grid[(temp.x + 2) * m_dimy + temp.y] != 19) 
        succ_temp.dir |= 0x02;
        if(grid[(temp.x + 1) * m_dimy + temp.y - 1] != 18 && grid[(temp.x + 1) * m_dimy + temp.y - 1] != 19)
        succ_temp.dir |= 0x04;
        if(grid[(temp.x + 1) * m_dimy + temp.y + 1] != 18 && grid[(temp.x + 1) * m_dimy + temp.y + 1] != 19)
        succ_temp.dir |= 0x08;

        que.push(succ_temp);     
        // std::cout <<"Successor  :" <<  succ_temp.x << ", " << succ_temp.y << ", " << ",time," << succ_temp.time << ", dir, " << succ_temp.dir << "\n";
      }
      if(temp.dir & 0x04){ // left
        StateLight succ_temp(temp.x,  temp.y - 1, temp.time + 1);
        if(eHeuristic[temp.x][temp.y - 1] != -1) continue;
        eHeuristic[temp.x][temp.y - 1] = temp.time + 1;      
        std::cout << eHeuristic[temp.x][temp.y - 1] << " Test\n";  
        if(grid[(temp.x + 1) * m_dimy + temp.y - 1] != 18 && grid[(temp.x + 1) * m_dimy + temp.y - 1] != 19) 
        succ_temp.dir |= 0x02;
        que.push(succ_temp);  
        // std::cout <<"Successor  :" <<  succ_temp.x << ", " << succ_temp.y << ", " << ",time," << succ_temp.time << "\n";
      }
      if(temp.dir & 0x08){ // right
        StateLight succ_temp(temp.x,  temp.y + 1, temp.time + 1);
        if(eHeuristic[temp.x][temp.y + 1] != -1) continue;
        eHeuristic[temp.x][temp.y + 1] = temp.time + 1;        
        std::cout << eHeuristic[temp.x][temp.y + 1] << " Test\n";
        if(grid[(temp.x + 1) * m_dimy + temp.y + 1] != 18 && grid[(temp.x + 1) * m_dimy + temp.y - 1] != 19) 
        succ_temp.dir |= 0x02;
        que.push(succ_temp);
        // std::cout <<"Successor  :" <<  succ_temp.x << ", " << succ_temp.y << ", " << ",time," << succ_temp.time << "\n";
      }      
    }

    for(int i = 0; i < eHeuristic.size(); i++){
      for(int j = 0; j < eHeuristic[i].size(); j++){
        std::cout << eHeuristic[i][j] << ", ";
      }
      std::cout<< "\n";
    }

  // for(int i = 0; i < goal_location_table.size(); i++){
  //   for(int ii = 0; ii < goal_location_table[i].size(); ii++){
  //     std::cout << goal_location_table[i][ii].x << ", " << goal_location_table[i][ii].y << ", time, " << goal_location_table[i][ii].time << std::endl;
  //   }
  // }

}

int main(int argc, char* argv[]) {

    struct rusage r_usage;
   	getrusage(RUSAGE_SELF, &r_usage);
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    int goalX, goalY;
    int goalXD, goalYD;
    int startXD, startYD;
    int heuris = 1;
    int is_weighted =0;
    int wa_version = 0;
    std::string filename;
    std::string f;
    desc.add_options()("help,h", "produce help message")(
      "goalXD,x", po::value<int>(&goalXD)->required(), "goal position x-component")(
      "goalYD,y", po::value<int>(&goalYD)->required(), "goal position y-component")(
      "startXD,r", po::value<int>(&startXD)->required(), "start position x-component" )(
      "startYD,c", po::value<int>(&startYD)->required(), "start position y-component")(
      "heuristic,e", po::value<int>(&heuris)->required(), "heuristic option")(
      "weight,w", po::value<int>(&is_weighted)->required(), "weighted")(
      "weightver,v", po::value<int>(&wa_version)->required(), "weighted version")
      ("filename,f", po::value<std::string>(&filename)->required(), "file name (TXT)");

    std::vector<std::vector<bool>> map_obstacle;
	  try
	  {
		  po::variables_map vm;
		  po::store(po::parse_command_line(argc, argv, desc), vm);
		  po::notify(vm);

		  if (vm.count("help") != 0u)
		  {
			  std::cout << desc << "\n";
			  return 0;
		  }
	  }

	  catch (po::error &e)
	  {
		  std::cerr << e.what() << std::endl
			  	  << std::endl;
		  std::cerr << desc << std::endl;
		  return 1;
	  }

    
      
    std::cout << "Begin test \n";
    std::ifstream infile(filename.c_str());
    assert(infile.is_open());
    std::string board_str;
    getline(infile,board_str);
    params["game_board_str"] = GameParameter(board_str);
    RNDGameState state_p(params);
    // state_p.board.grid[0] = 0;
    state_game = state_p;
    
    std::cout << state_p.board.rows << std::endl;
    std::vector<int8_t> grid;
    grid.swap(state_p.board.grid);

    std::cout << state_p.board.grid.size() << "end\n";
    std::random_device rd;  // 用于获取随机种子（硬件熵源）
    std::mt19937 gen(rd()); // 用随机种子初始化引擎    
    std::uniform_int_distribution<int> dist(0, 4);
    random_act.resize(state_p.board.rows);
    for(int h = 0; h < state_p.board.rows; h++){
      random_act[h].resize(state_p.board.cols);
      for(int w = 0; w < state_p.board.cols; w++){
        if(h == 0 || h == state_p.board.rows - 1 || w == 0 || w == state_p.board.cols - 1) random_act[h][w] = -1;
        else if(w == 49 || w == 150) random_act[h][w] =-1;
        else if(h == 51) random_act[h][w] = -1;
        else random_act[h][w] = dist(gen);
        printf("%d ", random_act[h][w]);
      }
      printf("\n");
    }

    int startX, startY;  
    std::vector<Location> goals_loc;
    map_obstacle.resize(state_p.board.rows);
    for(int h = 0; h < state_p.board.rows; ++h){
      map_obstacle[h].resize(state_p.board.cols);
    }
    for (int h = 0; h < state_p.board.rows; ++h) 
    {
      for (int w = 0; w < state_p.board.cols; ++w) 
      {
        if(grid[h * state_p.board.cols + w] == 0)
        {
          startX = h;
          startY = w;
        }
        if(grid[h * state_p.board.cols + w] == 5){
          // std::cout <<  "||" << h << ", " << w;
          // goals_loc.push_back(Location(h, w));
        }
        if(grid[h * state_p.board.cols + w] == 19 || grid[h * state_p.board.cols + w] == 18 ){
          // std::cout << "h," << h << ", w, " << w << std::endl;
          // std::cout << map_obstacle.size() << ", " << map_obstacle[h].size() << std::endl;
          map_obstacle[h][w] = 1;
        }
        std::cout << kCellTypeToElement[grid[h * state_p.board.cols + w] + 1].id;
      }
      std::cout << std::endl;
    }
    std::cout << "startx, " << startX << ", " << startY << ",xy, " << startXD << ", " << startYD <<"\n";
    // grid[startX * state_p.board.cols + startY] = 1;
    // grid[startXD * state_p.board.cols + startYD] = 0;
    // startX = startXD; startY = startYD;
    // grid.swap(state_p.board.grid);
    for (int h = 0; h < state_p.board.rows; ++h) 
    {
      for (int w = 0; w < state_p.board.cols; ++w){
        if(grid[h * state_p.board.cols + w] == 0){
          startX = h;
          startY = w;
        }
        if(grid[h * state_p.board.cols + w] == 5){
          // std::cout <<  "||" << h << ", " << w;
          goals_loc.push_back(Location(h, w));
        }
        if(grid[h * state_p.board.cols + w] == 19 || grid[h * state_p.board.cols + w] == 18 ){
          // std::cout << "h," << h << ", w, " << w << std::endl;
          // std::cout << map_obstacle.size() << ", " << map_obstacle[h].size() << std::endl;
          map_obstacle[h][w] = 1;
        }
        std::cout << kCellTypeToElement[grid[h * state_p.board.cols + w] + 1].id;
      }
      std::cout << std::endl;
    }
    // return 0;
    //PowerLocation(goalXD, goalYD, grid, state_p.board.rows, state_p.board.cols);

    std::vector <PlanResult<State, Action, int>>  solutions;
    LocalState localstate;
    if(goals_loc.size() == 0){
      std::cout << "Initially, there is no diamond\n";
      return 0;
    }
    int next_index = goals_loc[0].x * state_p.board.cols + goals_loc[0].y;
    int index_g = 0;
    bool is_falling = false;
    std::vector<int> faild;
    Timer total;
    State start_temp(startX, startY, 0, state_game.board.zorb_hash);
    start_temp.grid.assign(grid.begin(), grid.end());
    start_temp.localstate = localstate;
    start_temp.need_update_index = state_p.board.need_update_index;

    while(index_g < goals_loc.size()){
      Timer total_time;
      Timer preprocess;
      std::cout << index_g << "," << solutions.size() << "----\n";
      if(next_index == -1) break;
      int current_index = next_index;
      next_index = goals_loc[index_g].x * state_p.board.cols + goals_loc[index_g].y;
      // if(index_g == 1) break;
      // if(index_g != 0 ){
      //   startX = solutions[index_g - 1].states[0].first.x;
      //   startY = solutions[index_g - 1].states[0].first.y;
      // }
      // if(index_g == 0) state_game.board.grid.assign(grid.begin(), grid.end());
      // else state_game.board.grid.assign(solutions[index_g - 1].states[0].first.grid.begin(), solutions[index_g - 1].states[0].first.grid.end());

      state_game.board.grid.assign(grid.begin(), grid.end()); //the same initial map
      state_game.init_hash();
      state_game.num_apply_action = 0;
      state_game.is_hash_dirt = false;

      State start(startX, startY, 0, state_game.board.zorb_hash);
      start.grid.assign(grid.begin(), grid.end());
      start.localstate = localstate;
      start.need_update_index = start_temp.need_update_index;

      // if(index_g == 0){
      //   start.grid.assign(grid.begin(), grid.end());
      //   start.need_update_index = state_p.board.need_update_index;
      //   start.localstate = localstate;
      // }else{
      //   start.grid.assign(solutions[index_g - 1].states[0].first.grid.begin(), solutions[index_g - 1].states[0].first.grid.end());
      //   start.need_update_index.assign(solutions[index_g - 1].states[0].first.need_update_index.begin(), solutions[index_g - 1].states[0].first.need_update_index.end());
      //   start.localstate = solutions[index_g - 1].states[0].first.localstate;
      // }
      std::cout << "Start size " << start.need_update_index.size() << " ----------\n";
      for (int h = 0; h < state_game.board.rows; ++h)
      {
        for (int w = 0; w < state_game.board.cols; ++w) 
        {
          std::cout << kCellTypeToElement[start.grid[h * state_game.board.cols + w] + 1].id;
          // std::cout << kCellTypeToElement.at(state_game.board.grid[h * state_game.board.cols + w]).id;
        }
        std::cout << std::endl;
      }      
      // start.grid.assign(state_game.board.grid.begin(), state_game.board.grid.end());
      // start.need_update_index = state_p.board.need_update_index;
      // start.localstate = localstate;
      bool success = false;
      goalX = next_index/state_p.board.cols;
      goalY = next_index%state_p.board.cols;
      
      // if(index_g == 0){
      //   std::cout << "Test 111\n";
      //   goalX = goalXD;
      //   goalY = goalYD; 
      //   next_index = goalX * state_p.board.cols + goalY;
      // } else 
      // if(!(goalX == goalXD && goalY == goalYD)){
      //   std::cout << "Test 222\n";
      //   index_g++;
      //   continue;
      // }
      std::cout << startX << ", " << startY << ", " << goalX << ", " << goalY << " \n";
      // index_g ++;
      // continue;
      // std::vector<std::vector<StateLight>> goal_location_table;
      // StimulateTest(goal_location_table, goalXD, goalYD, state_p.board.cols, state_p.board.rows, grid);

      std::vector<std::vector<int>> eHeuristicGoal(state_p.board.rows, std::vector<int>(state_p.board.cols + 1, -1));
  
      std::cout << "Goal " << goalX << ", " << goalY << std::endl;
      std::vector<int8_t> grid_border;
      grid_border.assign(grid.begin(), grid.end());

      for(int h = 0; h < state_p.board.rows; h++){
        for(int w = 0; w < state_p.board.cols; w++){
          if(h == 0 || h == state_p.board.rows - 1 || w == 0 || w == state_p.board.cols - 1) grid_border[h * state_game.board.cols + w] = 19;
          else if(w == 49 || w == 150) grid_border[h * state_game.board.cols + w] = 19;
          else if(h == 51) grid_border[h * state_game.board.cols + w] = 19;
         std::cout << kCellTypeToElement[grid_border[h * state_game.board.cols + w] + 1].id;
        }
        printf("\n");
      } 

      Environment env(state_p.board.rows, state_p.board.cols, Location(goalX, goalY), eHeuristicGoal, grid_border);
      AStarT<StateLight, Action, int, Environment, vectorCache<int8_t>, vectorCache<int>> astar_1(env, gridCache, indexCache);
      PlanResult<StateLight, Action, int> solution_t;
      StateLight start_t(goalX, goalY, 0, 0xff);
      std::vector<std::vector<int>> eHeuristicGoalArea(state_p.board.rows, std::vector<int>(state_p.board.cols + 1, -1));
      std::unordered_set<StateLight, std::hash<StateLight>> closedSet;

      int num_move = 0;
      // for(int row_index =goalX + 1; row_index < state_p.board.rows; row_index++){
      //   int temp_index = row_index * state_p.board.cols + goalY;
      //   if(grid[temp_index] == 3 || grid[temp_index] == 4 || 
      //     grid[temp_index] == 5 || grid[temp_index] == 6 ||
      //     grid[temp_index] == 39 || grid[temp_index] == 40 ||
      //     grid[temp_index] == 3 || grid[temp_index] == 4 || 
      //     grid[temp_index] == 5 || grid[temp_index] == 6 ||
      //     grid[temp_index] == 39 || grid[temp_index] == 40){
      //     num_move++;
      //    }else break;
      // }
      std::cout << num_move << ", Start Rolling Under goal \n";
      start_t.move_h = num_move;
      std::cout <<start_t.x << ", y, " << start_t.y << ",move, " <<  num_move << ", Start Rolling Under goal \n";

      astar_1.search(start_t, solution_t, closedSet);

      std::vector<std::vector<int>> min_heuristic_from_border_target(state_p.board.rows, std::vector<int>(state_p.board.cols + 1, -1));
      std::vector<std::vector<int>> min_heuristic_overall_border(state_p.board.rows, std::vector<int>(state_p.board.cols + 1, -1));
      std::vector<std::vector<int>> min_heuristic_overall_border_location(state_p.board.rows, std::vector<int>(state_p.board.cols + 1, -1));
      std::vector<std::vector<int>> min_heuristic_closest_border_location(state_p.board.rows, std::vector<int>(state_p.board.cols + 1, -1));
      std::vector<std::vector<int>> min_heuristic_closest_border(state_p.board.rows, std::vector<int>(state_p.board.cols + 1, -1));
      std::vector<std::vector<int>> min_heuristic_diff_tb_sb(state_p.board.rows, std::vector<int>(state_p.board.cols + 1, -1));
      std::vector<std::vector<int>> min_heuristic_closest_border_goalarea(state_p.board.rows, std::vector<int>(state_p.board.cols + 1, -1));
      std::vector<std::vector<std::vector<int>>> border_distance;
      std::vector<Location> border_loc;
      std::vector<std::vector<std::vector<int>>> distance_goal_loc;
      std::vector<std::vector<int>> goal_area_index(state_p.board.rows, std::vector<int>(state_p.board.cols + 1, -1));
      
      int index_goal = 0;
      std::cout << distance_goal_loc.size() << "\n";
      Timer preprocess_1;
      for(auto it = closedSet.begin(); it != closedSet.end(); it++){
        eHeuristicGoalArea[(*it).x][(*it).y] = (*it).time;
        std::vector<std::vector<int>> min_heuristic_from_state_border(state_p.board.rows, std::vector<int>(state_p.board.cols + 1, -1));
        getShortestPathHeuristic(min_heuristic_from_state_border, map_obstacle, (*it).x, (*it).y, state_p.board.rows, state_p.board.cols);
        distance_goal_loc.push_back(min_heuristic_from_state_border);
        goal_area_index[(*it).x][(*it).y] = index_goal; 
        index_goal++;
      }
      preprocess_1.stop();

      Timer preprocess_2;
      int first_flag = 0;
      for(auto it = closedSet.begin(); it != closedSet.end(); it++){
        //  std::cout << (*it).x << ", " << (*it).y << ", time, " << (*it).time << "----\n";
          std::vector<std::vector<int>> min_heuristic_from_state_border(state_p.board.rows, std::vector<int>(state_p.board.cols + 1, -1));
          min_heuristic_overall_border[(*it).x][(*it).y] = (*it).time;
          min_heuristic_overall_border_location[(*it).x][(*it).y] = (*it).x * state_p.board.cols + (*it).y;
          min_heuristic_from_border_target[(*it).x][(*it).y] = (*it).time;
          // border_distance.push_back(min_heuristic_diff_tb_sb);
          // std::cout << (*it).x << ", " << (*it).y << ", "<< eHeuristicGoalArea[(*it).x][(*it).y] << "\n";
          //choose the border, exclude the cells inside the goal area
          if(eHeuristicGoalArea[(*it).x - 1][(*it).y] != -1 && eHeuristicGoalArea[(*it).x + 1][(*it).y] != -1 
              && eHeuristicGoalArea[(*it).x][(*it).y + 1] != -1 && eHeuristicGoalArea[(*it).x][(*it).y - 1] != -1){
            continue;
          }
          std::cout << "[" << (*it).x + 1 << ", " << (*it).y + 1 << "], ";
  //        min_heuristic_closest_border_goalarea[(*it).x][(*it).y] = 0;
          if(first_flag == 0){
            getShortestPathHeuristic(min_heuristic_from_state_border, map_obstacle, (*it).x, (*it).y, state_p.board.rows, state_p.board.cols);
            border_distance.push_back(min_heuristic_from_state_border);
            border_loc.push_back(Location((*it).x, (*it).y));
            for(int i = 0; i < min_heuristic_from_state_border.size(); i++){
              for(int j = 0; j < min_heuristic_from_state_border[i].size(); j++){
                if(min_heuristic_from_state_border[i][j] != -1) {
//                   min_heuristic_from_border_target[i][j] = (*it).time;
                   if((*it).time - min_heuristic_from_state_border[i][j] > 0){
                      min_heuristic_overall_border[i][j] = min_heuristic_from_state_border[i][j] + ((*it).time - min_heuristic_from_state_border[i][j]) / 2;
                   }else{
                      min_heuristic_overall_border[i][j] = min_heuristic_from_state_border[i][j];
                   }
                  //  min_heuristic_overall_border[i][j] = min_heuristic_from_state_border[i][j] + (*it).time;
                   min_heuristic_overall_border_location[i][j] = (*it).x * state_p.board.cols + (*it).y;
                   min_heuristic_closest_border[i][j] = min_heuristic_from_state_border[i][j];
                   
                   int temp_diff = ((*it).time - min_heuristic_from_state_border[i][j])/2;
                   if(temp_diff > 0){
                     if(min_heuristic_diff_tb_sb[i][j] == -1 || min_heuristic_diff_tb_sb[i][j] < temp_diff){
                      min_heuristic_diff_tb_sb[i][j] = temp_diff;

                      // min_heuristic_overall_border_location[i][j] = (*it).x * state_p.board.cols + (*it).y;
                     }
                   }
                }
                // min_heuristic_from_location[i][j] = (*it).time;
                // min_heuristic_from_goalarea[i][j] += (*it).time;
              }
              // std::cout <<"\n";
            }
          }else{
              getShortestPathHeuristic(min_heuristic_from_state_border, map_obstacle, (*it).x, (*it).y, state_p.board.rows, state_p.board.cols);
              border_distance.push_back(min_heuristic_from_state_border);
              border_loc.push_back(Location((*it).x, (*it).y));

              for(int i = 0; i < min_heuristic_from_state_border.size(); i++){
                for(int j = 0; j < min_heuristic_from_state_border[i].size(); j++){
                  if(min_heuristic_from_state_border[i][j] != -1){
                    int current_border_res;
                    if((*it).time - min_heuristic_from_state_border[i][j] > 0){
                      current_border_res = min_heuristic_from_state_border[i][j] + ((*it).time - min_heuristic_from_state_border[i][j]) / 2;
                    }else{
                      current_border_res = min_heuristic_from_state_border[i][j];
                    }
                    if(current_border_res < min_heuristic_overall_border[i][j] || min_heuristic_overall_border[i][j] == -1){
                      min_heuristic_overall_border[i][j] = current_border_res;
                      min_heuristic_overall_border_location[i][j] = (*it).x * state_p.board.cols + (*it).y;
                      min_heuristic_closest_border[i][j] = min_heuristic_from_state_border[i][j];
                    }
                    
                    int temp = min_heuristic_from_state_border[i][j] + (*it).time;
                    // if(temp < min_heuristic_overall_border[i][j]) {
                    //   // min_heuristic_overall_border[i][j] = temp;
                    //   min_heuristic_overall_border_location[i][j] = (*it).x * state_p.board.cols + (*it).y;
                    // }
                    if(min_heuristic_from_state_border[i][j] < min_heuristic_closest_border[i][j] 
                       || (min_heuristic_closest_border[i][j] == -1 && min_heuristic_from_border_target[i][j] != -1) ){
                      // min_heuristic_closest_border[i][j] = min_heuristic_from_state_border[i][j];
                      min_heuristic_closest_border_location[i][j] = (*it).x * state_p.board.cols + (*it).y;                      
                    }
                    int temp_diff = ((*it).time - min_heuristic_from_state_border[i][j])/2;
                    if(temp_diff > 0 ){
                      if(min_heuristic_diff_tb_sb[i][j] == -1 || min_heuristic_diff_tb_sb[i][j] < temp_diff){
                        min_heuristic_diff_tb_sb[i][j] = temp_diff;
                        if(i == 10 && j == 17) std::cout << (*it).x << ", " << (*it).y << ",time, " << (*it).time << ",sb, " <<  min_heuristic_from_state_border[i][j] << " , 10, 17 \n";
                        // min_heuristic_overall_border_location[i][j] = (*it).x * state_p.board.cols + (*it).y;
                      }
                    }
                  }
                  // if(((min_heuristic_from_state_border[i][j] > eHeuristic[i][j]) || 
                  //     ( min_heuristic_from_state_border[i][j] == eHeuristic[i][j] && min_heuristic_from_border_target[i][j] > (*it).time) )
                  //     && eHeuristic[i][j] != -1){
                  //   if(i == 10 && j == 17) std::cout << "goal L, " << (*it).x << ", " << (*it).y << ",d, " << (*it).time <<", shortest d, " << eHeuristic[i][j] << "\n";
                  //   min_heuristic_from_state_border[i][j] = eHeuristic[i][j];
                  //   min_heuristic_from_border_target[i][j] = (*it).time;
                  // }
                }
              }              

          }

//           std::cout << "--------------------------\n";
//          printf("%d \n", border_distance.size());
//          for(int iiii = 0; iiii <  min_heuristic_from_state_border.size(); iiii++){
//            for(int jjjj = 0; jjjj < min_heuristic_from_state_border[iiii].size(); jjjj++)
//            std::cout <<  std::setw(2) << min_heuristic_from_state_border[iiii][jjjj] << ",";
//            std::cout << "\n";
//          }
//           std::cout << "--------------------------\n";

          first_flag++;
          min_heuristic_from_border_target[(*it).x][(*it).y] = (*it).time;
      }
 std::cout << " end border\n" ;
      preprocess_2.stop();
      for(auto it = closedSet.begin(); it != closedSet.end(); it++){
//          std::cout << (*it).x << ", " << (*it).y << ", time, " << (*it).time << "----\n";
//          min_heuristic_from_state_border[(*it).x][(*it).y] =  (*it).time;
//          min_heuristic_from_border_target[(*it).x][(*it).y] = 0;
          min_heuristic_overall_border[(*it).x][(*it).y] = (*it).time;
          min_heuristic_closest_border[(*it).x][(*it).y] = (*it).time;
      }

      // for(int i = 0; i < eHeuristicGoalArea.size(); i++){
      //   for(int j = 0; j < eHeuristicGoalArea[i].size(); j++){
      //     std::cout <<  std::setw(2) << min_heuristic_overall_border[i][j] << ",";
      //     // if(min_heuristic_overall_border_location[i][j] == -1 && min_heuristic_closest_border_location[i][j] != -1){
      //     //   min_heuristic_overall_border_location[i][j] = min_heuristic_closest_border_location[i][j];
      //     // }
      //     if(min_heuristic_diff_tb_sb[i][j] == -1) min_heuristic_diff_tb_sb[i][j] = 0;
      //     // if(min_heuristic_closest_border[i][j] > (min_heuristic_overall_border[i][j] + 1)/2){
      //     //   min_heuristic_overall_border[i][j] = min_heuristic_closest_border[i][j] * 2;
      //     //   min_heuristic_overall_border_location[i][j] = min_heuristic_closest_border_location[i][j];
      //     // }
      //   }
      //   std::cout <<"\n";
      // }

      // std::cout << "--------------------------\n";
      // for(int i = 0; i < border_distance.size(); i++){
      //   std::cout << "Location " << border_loc[i].x << ", " << border_loc[i].y << " \n";
      //   for(int j = 0; j < border_distance[i].size(); j++){
      //     for(int k = 0; k < border_distance[i][j].size(); k++){
      //       std::cout <<std::setw(2) << border_distance[i][j][k] << ",";
      //     }
      //     std::cout << "\n";
      //   }
      // }


    //   for(int i = 0; i < eHeuristicGoalArea.size(); i++){
    //     for(int j = 0; j < eHeuristicGoalArea[i].size(); j++){
    //       std::cout <<  std::setw(2) << min_heuristic_closest_border[i][j] << ",";
    //       // if(min_heuristic_overall_border_location[i][j] == -1 && min_heuristic_closest_border_location[i][j] != -1){
    //       //   min_heuristic_overall_border_location[i][j] = min_heuristic_closest_border_location[i][j];
    //       // }
    //       if(min_heuristic_diff_tb_sb[i][j] == -1) min_heuristic_diff_tb_sb[i][j] = 0;
    //       // if(min_heuristic_closest_border[i][j] > (min_heuristic_overall_border[i][j] + 1)/2){
    //       //   min_heuristic_overall_border[i][j] = min_heuristic_closest_border[i][j] * 2;
    //       //   min_heuristic_overall_border_location[i][j] = min_heuristic_closest_border_location[i][j];
    //       // }
    //     }
    //     std::cout <<"\n";
    //   }         
    //  std::cout << "--------------------------\n";

      // std::cout << "Goal Area--------------------------\n";
      // for(int i = 0; i < eHeuristicGoalArea.size(); i++){
      //   for(int j = 0; j < eHeuristicGoalArea[i].size(); j++){
      //     std::cout <<  std::setw(2) << min_heuristic_closest_border_goalarea[i][j] << ",";
      //     // if(min_heuristic_overall_border_location[i][j] == -1 && min_heuristic_closest_border_location[i][j] != -1){
      //     //   min_heuristic_overall_border_location[i][j] = min_heuristic_closest_border_location[i][j];
      //     // }
      //     // if(min_heuristic_diff_tb_sb[i][j] == -1) min_heuristic_diff_tb_sb[i][j] = 0;
      //     // if(min_heuristic_closest_border[i][j] > (min_heuristic_overall_border[i][j] + 1)/2){
      //     //   min_heuristic_overall_border[i][j] = min_heuristic_closest_border[i][j] * 2;
      //     //   min_heuristic_overall_border_location[i][j] = min_heuristic_closest_border_location[i][j];
      //     // }
      //   }
      //   std::cout <<"\n";
      // }         
     std::cout << "--------------------------\n";     
      for(int i = 0; i < eHeuristicGoalArea.size(); i++){
        for(int j = 0; j < eHeuristicGoalArea[i].size(); j++){
          std::cout <<  std::setw(2) << eHeuristicGoalArea[i][j] << ",";
        }
        std::cout <<"\n";
      } 
      std::cout << "--------------------------\n";
    //   for(int i = 0; i < eHeuristicGoalArea.size(); i++){
    //     for(int j = 0; j < eHeuristicGoalArea[i].size(); j++){
    //       std::cout <<  std::setw(2) << min_heuristic_overall_border[i][j] << ",";
    //     }
    //     std::cout <<"\n";
    //   }                
    //   std::cout << "111111111111111111111111--------------------------\n";


      // for(int i = 0; i < eHeuristicGoalArea.size(); i++){
      //   for(int j = 0; j < eHeuristicGoalArea[i].size(); j++){
      //     std::cout <<  std::setw(2) << min_heuristic_from_border_target[i][j] << ",";
      //   }
      //   std::cout <<"\n";
      // }         
      // std::cout << "--------------------------\n";

      // for(int i = 0; i < eHeuristicGoalArea.size(); i++){
      //   for(int j = 0; j < eHeuristicGoalArea[i].size(); j++){
      //     std::cout <<  std::setw(2) << eHeuristicGoalArea[i][j] << ",";
      //   }
      //   std::cout <<"\n";
      // }         
      // std::cout << "--------------------------\n";

      // return 0;
          // for(int kkk = 0; kkk < distance_goal_loc.size(); kkk++){
          //   for(int i = 0; i < distance_goal_loc[kkk].size(); i++){
          //     for(int j = 0; j < distance_goal_loc[kkk][i].size(); j++){
          //       std::cout <<  std::setw(2) << distance_goal_loc[kkk][i][j] << ",";
          //     }
          //     std::cout <<"\n";
          //   }
          //   std::cout << kkk << "------------------------------------------------------------------\n";
          // } 

      Environment env_1(state_p.board.rows, state_p.board.cols, Location(goalX, goalY), 
                        min_heuristic_closest_border, min_heuristic_overall_border, min_heuristic_overall_border_location, eHeuristicGoalArea, border_distance, distance_goal_loc, goal_area_index, border_loc, grid);
      env_1.is_manhattan_distance = heuris;

      AStar<State, Action, int, Environment, vectorCache<int8_t>, vectorCache<int>> astar(env_1, gridCache, indexCache);


      start.index_gem = next_index;
      start.gem_x = goalX;
      start.gem_y = goalY;
      state_game.board.grid.assign(start.grid.begin(), start.grid.end());
      start.is_falling = state_game.IsType(next_index, kElEmpty, Directions::kDown);;
      start.canRollLeft = state_game.CanRollLeft(next_index);
      start.canRollRight = state_game.CanRollRight(next_index);
      // if(start.is_falling || start.canRollLeft || start.canRollRight) start.goal_roll = true;
    
      std::cout << next_index / state_p.board.cols << ", " << next_index % state_p.board.cols << std::endl;

      PlanResult<State, Action, int> solution;
      Timer timer;
      if (env.stateValid(start)) {
        state_game.is_hash_dirt = true;
        // astar.is_second = false;
        env_1.num_expand = 0; 
        env_1.num_generated = 0;
        Timer timerSolve;
        success = astar.search(start, solution, is_weighted, wa_version);
        timerSolve.stop();
        total_time.stop();
        if(success){
          std::cout << ",Pathfinding success !,";
          if(solution.states[0].first.x == goalX && solution.states[0].first.y == goalY) std::cout << ",same,";
          else std::cout << ",diff,";
          getrusage(RUSAGE_SELF, &r_usage);
          std::cout << "," << env_1.is_manhattan_distance << ", heuristic,";
          if(env_1.is_manhattan_distance == 1) std::cout << ", " << env_1.is_manhattan_distance << ",ManhattanDistance,";
          else if(env_1.is_manhattan_distance == 2) std::cout  << ", " << env_1.is_manhattan_distance<< ",FullBNewHeuristic,";
          else if(env_1.is_manhattan_distance == 3) std::cout << ", " << env_1.is_manhattan_distance << ",Max(TDSB, MD2),";
          else if(env_1.is_manhattan_distance == 4) std::cout << ", " << env_1.is_manhattan_distance << ",Max(TDSB,BDTD/2),";
          else if(env_1.is_manhattan_distance == 5) std::cout << ", " << env_1.is_manhattan_distance <<", Max(TDSB, TDSG/2)";
          if(success) std::cout <<filename <<  ", success, cost, " << solution.cost <<"," << env.is_roll_fall << ",start, " << startX << ", " << startY << ", goal, " << goalX <<", " << goalY << ", memory, " << r_usage.ru_maxrss  << ",preprocess time1, "<< preprocess_1.elapsedSeconds() << ", preprocess2, " << preprocess_2.elapsedSeconds() << ", solve time, " << timerSolve.elapsedSeconds() <<", total time, " <<  total_time.elapsedSeconds() <<  ", Expansion, " << env_1.num_expand << ", generation, " << env_1.num_generated << ",num_action," << state_game.num_apply_action << ", bordersize, " << border_loc.size()<<std::endl;    
          else std::cout << filename <<  ", not success, ,"<< env.is_roll_fall << ", ,start, " << startX << ", " << startY << ", goal, " << goalX <<", " << goalY << ", memory, " << r_usage.ru_maxrss  << ",preprocess time 1, "<< preprocess_1.elapsedSeconds() << ", preprocess2, " << preprocess_2.elapsedSeconds()<< ", solve time, " << timerSolve.elapsedSeconds() <<", total time, " <<  total_time.elapsedSeconds() << ", Expansion, " << env_1.num_expand << ", generation, " << env_1.num_generated << ",num_action," << state_game.num_apply_action << ", bordersize, " << border_loc.size() <<std::endl;    
        }else{
          if(env_1.is_manhattan_distance == 1) std::cout << ", " << env_1.is_manhattan_distance << ",ManhattanDistance,";
          else if(env_1.is_manhattan_distance == 2) std::cout  << ", " << env_1.is_manhattan_distance<< ",FullBNewHeuristic,";
          else if(env_1.is_manhattan_distance == 3) std::cout << ", " << env_1.is_manhattan_distance << ",Max(TDSB, MD2),";
          else if(env_1.is_manhattan_distance == 4) std::cout << ", " << env_1.is_manhattan_distance << ",Max(TDSB,BDTD/2),";
          else if(env_1.is_manhattan_distance == 5) std::cout << ", " << env_1.is_manhattan_distance <<", Max(TDSB, TDSG/2)";          
          std::cout << filename <<  ", fail not success, ,"<< env.is_roll_fall << ", ,start, " << startX << ", " << startY << ", goal, " << goalX <<", " << goalY << ", memory, " << r_usage.ru_maxrss   << ",preprocess time1, " << preprocess_1.elapsedSeconds() << ", preprocess2, " << preprocess_2.elapsedSeconds() << ", solve time, " << timerSolve.elapsedSeconds() <<", total time, " <<  total_time.elapsedSeconds() << ", Expansion, " << env_1.num_expand << ", generation, " << env_1.num_generated << ",num_action," << state_game.num_apply_action << ", bordersize, " << border_loc.size() <<std::endl;           
          //std::cout<<filename << ",start, " << startX << ", " << startY << ", goal, " << goalX <<", " << goalY << "Pathfinding not success\n";
        }/*else if(astar.is_goal_move){
          // getrusage(RUSAGE_SELF, &r_usage);
          // std::cout << "The goal is move, ";
          std::cout << filename <<  ", not success, ,"<< env.is_roll_fall << ", ,start, " << startX << ", " << startY << ", goal, " << goalX <<", " << goalY << ", memory, " << r_usage.ru_maxrss  << ", time, " <<  timer.elapsedSeconds() <<  ", Expansion, " << env.num_expand << ", generation, " << env.num_generated << ",num_action," << state_game.num_apply_action <<std::endl; 
          state_game.is_hash_dirt = true;
          state_game.board.grid.assign(grid.begin(), grid.end());
          state_game.init_hash();
          state_game.num_apply_action = 0;
          Environment env1(state_p.board.rows, state_p.board.cols, Location(goalX, goalY), eHeuristicGoal);
          AStar<State, Action, int, Environment, vectorCache<int8_t>, vectorCache<int>> astar1(env1, gridCache, indexCache);
          astar1.is_second = true;

          State start(startX, startY, 0, state_game.board.zorb_hash, *gridCache.getItem(), *indexCache.getItem());
          start.grid.assign(grid.begin(), grid.end());
          start.need_update_index = state_p.board.need_update_index;
          start.localstate = localstate;          
          start.index_gem = next_index;
          start.gem_x = goalX;
          start.gem_y = goalY;
          state_game.board.grid.assign(start.grid.begin(), start.grid.end());
          start.is_falling = state_game.IsType(next_index, kElEmpty, Directions::kDown);;
          start.canRollLeft = state_game.CanRollLeft(next_index);
          start.canRollRight = state_game.CanRollRight(next_index);
          Timer timer2;
          success = astar1.search(start, solution);
          timer2.stop();
          if(success){
            std::cout << ",Pathfinding success !,";            
          }
          getrusage(RUSAGE_SELF, &r_usage);
          if(success) std::cout <<filename <<  ", success, cost, " << solution.cost <<"," << env.is_roll_fall << ",start, " << startX << ", " << startY << ", goal, " << goalX <<", " << goalY << ", memory, " << r_usage.ru_maxrss  << ", time, " <<  timer2.elapsedSeconds() <<  ", Expansion, " << env1.num_expand << ", generation, " << env1.num_generated << ",num_action," << state_game.num_apply_action <<std::endl;    
          else std::cout << filename <<  ", not success, ,"<< env.is_roll_fall << ", ,start, " << startX << ", " << startY << ", goal, " << goalX <<", " << goalY << ", memory, " << r_usage.ru_maxrss  << ", time, " <<  timer2.elapsedSeconds() <<  ", Expansion, " << env.num_expand << ", generation, " << env.num_generated << ",num_action," << state_game.num_apply_action <<std::endl;    

        }*/
      }
      // timer.stop();
      

      index_g++;
      if(success){
        for (int i = solution.actions.size() - 1; i > 0; i--) {
          if(i == solution.actions.size() - 1)  std::cout << solution.states.back().second << ": "
              << solution.states.back().first <<  "->" << solution.actions[i].first
              << "(cost: " << solution.actions[i].second << ")" << std::endl;
          std::cout << solution.states[i].second << ": " << solution.states[i].first << "->" << solution.actions[i - 1].first
                  << "(cost: " << solution.actions[i - 1].second << ")"  << ",hash, " << solution.states[i].first.zorb_hash << std::endl;                  
        }
        std::cout << solution.states[0].second << ": " << solution.states[0].first;       
        std::cout << std::endl;

        for (int i = solution.actions.size() - 1; i > 0; i--) {
            if(i == solution.actions.size() - 1) {
              std::cout << solution.states.back().second << ": "
                << solution.states.back().first <<  "->" << solution.actions[i].first
                  << "(cost: " << solution.actions[i].second << ")" << std::endl;
              // for (int h = 0; h < state_p.board.rows; ++h) {
              //   for (int w = 0; w < state_p.board.cols; ++w) {
              //     std::cout << kCellTypeToElement[solution.states.back().first.grid[h * state_p.board.cols + w] + 1].id;
              //     // if(h==0) grid[h * state_p.board.cols + w] = 100;
              //   }
              //   std::cout << std::endl;
              // }                    
            }
            std::cout << solution.states[i].second << ": " << solution.states[i].first << "->" << solution.actions[i - 1].first
                  << "(cost: " << solution.actions[i - 1].second << ")" << std::endl;
          //  for (int h = 0; h < state_p.board.rows; ++h) {
          //   for (int w = 0; w < state_p.board.cols; ++w) {
          //     std::cout << kCellTypeToElement[solution.states[i].first.grid[h * state_p.board.cols + w] + 1].id;
          //     // if(h==0) grid[h * state_p.board.cols + w] = 100;
          //   }
          //   std::cout << std::endl;
          // }                        
        }
        std::cout << solution.states[0].second << ": " << solution.states[0].first;       
        std::cout << std::endl;
          //  for (int h = 0; h < state_p.board.rows; ++h) {
          //   for (int w = 0; w < state_p.board.cols; ++w) {
          //     std::cout << kCellTypeToElement[solution.states[0].first.grid[h * state_p.board.cols + w] + 1].id;
          //     // if(h==0) grid[h * state_p.board.cols + w] = 100;
          //   }
          //   std::cout << std::endl;
          // }                        
      
      }
      solutions.push_back(solution);      
      continue;
      
      next_index  = -1;
      if (success) {
        for(int iii = 0; iii < faild.size(); iii++){
          if(faild[iii] == current_index) {
            faild[iii] = -1;
            break;
          }
        }        
        std::cout << filename << "Planning successful! Total cost: " << solution.cost << ", time, " << timer.elapsedSeconds() << ", expand," << env.num_expand
                << std::endl;
        // std::cout << solution.states.back().second << ": "
        //         << solution.states.back().first;
        // std::cout << solution.actions.size() - 1 << std::endl;
        for (int i = solution.actions.size() - 1; i > 0; i--) {
          if(i == solution.actions.size() - 1)  std::cout << solution.states.back().second << ": "
              << solution.states.back().first <<  "->" << solution.actions[i].first
              << "(cost: " << solution.actions[i].second << ")" << std::endl;
          std::cout << solution.states[i].second << ": " << solution.states[i].first << "->" << solution.actions[i - 1].first
                  << "(cost: " << solution.actions[i - 1].second << ")" << std::endl;                  
        }
        std::cout << solution.states[0].second << ": " << solution.states[0].first;       
        std::cout << std::endl;

        for (int i = solution.actions.size() - 1; i > 0; i--) {
          if(i == solution.actions.size() - 1) {
            std::cout << solution.states.back().second << ": "
             << solution.states.back().first <<  "->" << solution.actions[i].first
             << "(cost: " << solution.actions[i].second << ")" << std::endl;
            // for (int h = 0; h < state_p.board.rows; ++h) {
            //   for (int w = 0; w < state_p.board.cols; ++w) {
            //     std::cout << kCellTypeToElement[solution.states.back().first.grid[h * state_p.board.cols + w] + 1].id;
            //       // if(h==0) grid[h * state_p.board.cols + w] = 100;
            //   }
            //     std::cout << std::endl;
            // }                    
          }
          std::cout << solution.states[i].second << ": " << solution.states[i].first << "->" << solution.actions[i - 1].first
            << "(cost: " << solution.actions[i - 1].second << ")" << std::endl;
          // for (int h = 0; h < state_p.board.rows; ++h) {
          //   for (int w = 0; w < state_p.board.cols; ++w) {
          //     std::cout << kCellTypeToElement[solution.states[i].first.grid[h * state_p.board.cols + w] + 1].id;
          //     // if(h==0) grid[h * state_p.board.cols + w] = 100;
          //   }
          //   std::cout << std::endl;
          // }                        
        }
        std::cout << solution.states[0].second << ": " << solution.states[0].first;       
        std::cout << std::endl;
        // std::cout << "---------------------------------\n";               
        // for (size_t i = 0; i < solution.actions.size(); ++i) {
        //   std::cout << solution.states[i].second << ": " << solution.states[i].first
        //           << "->" << solution.actions[i].first
        //           << "(cost: " << solution.actions[i].second << ")" << std::endl;
        // }
        // std::cout << solution.states.back().second << ": "
        //         << solution.states.back().first << std::endl;
        solutions.push_back(solution);

        std::cout << static_cast<unsigned int>(solutions[index_g].states[0].first.localstate.gems_collected) << std::endl;
        state_p.board.grid.assign(solutions[index_g].states[0].first.grid.begin(), solutions[index_g].states[0].first.grid.end());
        // if(solutions[index_g].states[0].first.localstate.gems_collected >= state_p.board.gems_required){
        //   next_index = 678;
        //   index_g++;
        //   continue; 
        // }
        
        int gem_num = 0;
        is_falling = false;
        sort(solutions[index_g].states[0].first.need_update_index.begin(), solutions[index_g].states[0].first.need_update_index.end());
        for(int gem_i = 0; gem_i < solutions[index_g].states[0].first.need_update_index.size(); gem_i++){
          int index_gem = solutions[index_g].states[0].first.need_update_index[gem_i];
          switch (state_p.board.item(index_gem))
          {
            case static_cast<std::underlying_type_t<HiddenCellType>>(HiddenCellType::kDiamond):
              if(gem_num == 0) next_index = index_gem;
              gem_num++;
              break;
            case static_cast<std::underlying_type_t<HiddenCellType>>(HiddenCellType::kDiamondFalling):
              if(gem_num == 0) next_index = index_gem;
              is_falling = true;
              gem_num++;
              break;
            default:
              break;
          }
        }
        index_g++;
        std::cout << "Num_gem " << gem_num << " \n";        
      } else {
        std::cout << filename << startY << ", goal, " << goalX <<", " << goalY << ",id " << index_g << ",index," << current_index <<  ", Planning NOT successful!" << std::endl;
        faild.push_back(current_index);
        int gem_num = 0;
        is_falling = false;
        sort(solutions[index_g - 1].states[0].first.need_update_index.begin(), solutions[index_g - 1].states[0].first.need_update_index.end());
        for(int gem_i = 0; gem_i < solutions[index_g - 1].states[0].first.need_update_index.size(); gem_i++){
          int index_gem = solutions[index_g - 1].states[0].first.need_update_index[gem_i];
          switch (state_p.board.item(index_gem))
          {
            case static_cast<std::underlying_type_t<HiddenCellType>>(HiddenCellType::kDiamond):
              if(next_index == -1 && index_gem != current_index) {
                bool flag_faild = false;
                for(int iii = 0; iii < faild.size(); iii++){
                  if(faild[iii] == index_gem) flag_faild = true;
                }
                if(!flag_faild) next_index = index_gem;
              }
              gem_num++;
              break;
            case static_cast<std::underlying_type_t<HiddenCellType>>(HiddenCellType::kDiamondFalling):
              if(next_index == -1 && index_gem != current_index) {
                bool flag_faild = false;
                for(int iii = 0; iii < faild.size(); iii++){
                  if(faild[iii] == index_gem) flag_faild = true;
                }
                if(!flag_faild) next_index = index_gem;                
              }
              is_falling = true;
              gem_num++;
              break;
            default:
              break;
          }
        }
        // break;
      }

    }
    total.stop();
    std::cout << total.elapsedSeconds() << "  -------"<<std::endl;    

  // namespace po = boost::program_options;
  // Declare the supported options.
  // po::options_description desc("Allowed options");
  // int startX, startY, goalX, goalY;
  // std::string mapFile;
  // std::string outputFile;
  // desc.add_options()("help", "produce help message")(
      // "startX", po::value<int>(&startX)->required(),
      // "start position x-component")("startY",
                                    // po::value<int>(&startY)->required(),
                                    // "start position y-component")(
      // "goalX", po::value<int>(&goalX)->required(), "goal position x-component")(
      // "goalY", po::value<int>(&goalY)->required(), "goal position y-component")(
      // "map,m", po::value<std::string>(&mapFile)->required(), "input map (txt)")(
      // "output,o", po::value<std::string>(&outputFile)->required(),
      // "output file (YAML)");

  // try {
  //   po::variables_map vm;
  //   po::store(po::parse_command_line(argc, argv, desc), vm);
  //   po::notify(vm);

  //   if (vm.count("help") != 0u) {
  //     std::cout << desc << "\n";
  //     return 0;
  //   }
  // } catch (po::error& e) {
  //   std::cerr << e.what() << std::endl << std::endl;
  //   std::cerr << desc << std::endl;
  //   return 1;
  // }

  // std::unordered_set<State> obstacles;

  // std::ifstream map(mapFile);
  // int dimX = 0;
  // int y = 0;
  // while (map.good()) {
  //   std::string line;
  //   std::getline(map, line);
  //   int x = 0;
  //   for (char c : line) {
  //     if (c == '#') {
  //       obstacles.insert(State(x, y));
  //     }
  //     ++x;
  //   }
  //   dimX = std::max(dimX, x);
  //   ++y;
  // }
  // std::cout << dimX << " " << y << std::endl;

  // bool success = false;

  // State goal(goalX, goalY);
  // State start(startX, startY);
  // Environment env(dimX, y - 1, obstacles, goal);

  // AStar<State, Action, int, Environment> astar(env);

  // PlanResult<State, Action, int> solution;

  // if (env.stateValid(start)) {
  //   success = astar.search(start, solution);
  // }

  // std::ofstream out(outputFile);
  // if (success) {
  //   std::cout << "Planning successful! Total cost: " << solution.cost
  //             << std::endl;
  //   for (size_t i = 0; i < solution.actions.size(); ++i) {
  //     std::cout << solution.states[i].second << ": " << solution.states[i].first
  //               << "->" << solution.actions[i].first
  //               << "(cost: " << solution.actions[i].second << ")" << std::endl;
  //   }
  //   std::cout << solution.states.back().second << ": "
  //             << solution.states.back().first << std::endl;

  //   out << "schedule:" << std::endl;
  //   out << "  agent1:" << std::endl;
  //   for (size_t i = 0; i < solution.states.size(); ++i) {
  //     out << "    - x: " << solution.states[i].first.x << std::endl
  //         << "      y: " << solution.states[i].first.y << std::endl
  //         << "      t: " << i << std::endl;
  //   }
  // } else {
  //   std::cout << "Planning NOT successful!" << std::endl;
  // }

  return 0;
}