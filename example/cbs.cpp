#include <fstream>
#include <iostream>
#include <algorithm>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include <libMultiRobotPlanning/cbs.hpp>
#include <libMultiRobotPlanning/cbs_astar.hpp>
#include <libMultiRobotPlanning/cbs_jpst_astar.hpp>
#include <libMultiRobotPlanning/cbs_caastar.hpp>
#include <libMultiRobotPlanning/cbs_sipp.hpp>
#include <libMultiRobotPlanning/gridmap.hpp>
#include <libMultiRobotPlanning/jpst_gridmap.hpp>

//#include "timer.hpp"
using namespace std;

using libMultiRobotPlanning::CBS;
using libMultiRobotPlanning::CBSJPSTAstar;
using libMultiRobotPlanning::CBSSIPP;
using libMultiRobotPlanning::CBSAstar;
using libMultiRobotPlanning::CBSCAstar;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using libMultiRobotPlanning::gridmap;
using libMultiRobotPlanning::jpst_gridmap;

struct State {
  State(int time, int x, int y) : time(time), x(x), y(y) {}

  bool operator==(const State& s) const {
    return time == s.time && x == s.x && y == s.y;
  }

  bool equalExceptTime(const State& s) const { return x == s.x && y == s.y; }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << s.time << ": (" << s.x << "," << s.y << ")";
    // return os << "(" << s.x << "," << s.y << ")";
  }

  int time;
  int x;
  int y;
  int nc_cat = 0;
  unsigned int dir = 0xff;
  unsigned int dir_p = 0xff;
};

namespace std {
template <>
struct hash<State> {
  size_t operator()(const State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

///
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

///

struct Conflict {
  enum Type {
    Vertex,
    Edge,
  };

  int time;
  size_t agent1;
  size_t agent2;
  Type type;

  int x1;
  int y1;
  int x2;
  int y2;

  friend std::ostream& operator<<(std::ostream& os, const Conflict& c) {
    switch (c.type) {
      case Vertex:
        return os << c.time << ": Vertex(" << c.x1 << "," << c.y1 << ")";
      case Edge:
        return os << c.time << ": Edge(" << c.x1 << "," << c.y1 << "," << c.x2
                  << "," << c.y2 << ")";
    }
    return os;
  }
};

struct VertexConstraint {
  VertexConstraint(int time, int x, int y) : time(time), x(x), y(y) {}
  int time;
  int x;
  int y;

  bool operator<(const VertexConstraint& other) const {
    return std::tie(time, x, y) < std::tie(other.time, other.x, other.y);
  }

  bool operator==(const VertexConstraint& other) const {
    return std::tie(time, x, y) == std::tie(other.time, other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const VertexConstraint& c) {
    // return os << "VC(" << c.time << "," << c.x << "," << c.y << ")";
    return os << c.time << " " << c.x << " " << c.y << "  VC";

  }
};

namespace std {
template <>
struct hash<VertexConstraint> {
  size_t operator()(const VertexConstraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

struct EdgeConstraint {
  EdgeConstraint(int time, int x1, int y1, int x2, int y2)
      : time(time), x1(x1), y1(y1), x2(x2), y2(y2) {}
  int time;
  int x1;
  int y1;
  int x2;
  int y2;

  bool operator<(const EdgeConstraint& other) const {
    return std::tie(time, x1, y1, x2, y2) <
           std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
  }

  bool operator==(const EdgeConstraint& other) const {
    return std::tie(time, x1, y1, x2, y2) ==
           std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
  }

  friend std::ostream& operator<<(std::ostream& os, const EdgeConstraint& c) {
    // return os << "EC(" << c.time << "," << c.x1 << "," << c.y1 << "," << c.x2
    //           << "," << c.y2 << ")";
    return os <<  c.time << " " << c.x1 << " " << c.y1 << " " << c.x2
              << " " << c.y2 << " EC";              
  }
};

namespace std {
template <>
struct hash<EdgeConstraint> {
  size_t operator()(const EdgeConstraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x1);
    boost::hash_combine(seed, s.y1);
    boost::hash_combine(seed, s.x2);
    boost::hash_combine(seed, s.y2);
    return seed;
  }
};
}  // namespace std

struct Constraints {
  std::unordered_set<VertexConstraint> vertexConstraints;
  std::unordered_set<EdgeConstraint> edgeConstraints;

  void add(const Constraints& other) {
    vertexConstraints.insert(other.vertexConstraints.begin(),
                             other.vertexConstraints.end());
    edgeConstraints.insert(other.edgeConstraints.begin(),
                           other.edgeConstraints.end());
  }

  bool overlap(const Constraints& other) {
    std::vector<VertexConstraint> vertexIntersection;
    std::vector<EdgeConstraint> edgeIntersection;
    std::set_intersection(vertexConstraints.begin(), vertexConstraints.end(),
                          other.vertexConstraints.begin(),
                          other.vertexConstraints.end(),
                          std::back_inserter(vertexIntersection));
    std::set_intersection(edgeConstraints.begin(), edgeConstraints.end(),
                          other.edgeConstraints.begin(),
                          other.edgeConstraints.end(),
                          std::back_inserter(edgeIntersection));
    return !vertexIntersection.empty() || !edgeIntersection.empty();
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraints& c) {
    os << "Vertex " << c.vertexConstraints.size() << std::endl;
    for (const auto& vc : c.vertexConstraints) {
      os << vc << std::endl;
    }
    os << "Edge " << c.edgeConstraints.size() << std::endl;
    for (const auto& ec : c.edgeConstraints) {
      os << ec << std::endl;
    }
    return os;
  }
};

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

namespace std {
template <>
struct hash<Location> {
  size_t operator()(const Location& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

///
class Environment {
 public:
  Environment(size_t dimx, size_t dimy, std::unordered_set<Location> obstacles,
              std::vector<Location> goals, Location goal, std::vector<std::vector<bool>>m_obstacle_matrix,
			  std::vector<std::vector<bool>>temporal_obstacle,
			  std::vector<std::vector<int>>temporal_edge_constraint,
			  std::vector<std::vector<bool>>jump_point_map_m,
			  std::vector<std::vector<int>>last_ob_g,
  	  	  	  std::vector<std::vector<int>>nei_ob_g,
			  std::unordered_map<Location, std::vector<std::vector<int>>> eHeuristic,
        std::vector<double>preTime1,
        jpst_gridmap *mmap_
			  )
      : m_dimx(dimx),
        m_dimy(dimy),
        m_obstacles(std::move(obstacles)),
        m_goals(std::move(goals)),
		m_goal(goal),
		m_obstacles_m(std::move(m_obstacle_matrix)),
		m_temporal_obstacle(std::move(temporal_obstacle)),
		m_temporal_edge_constraint(std::move(temporal_edge_constraint)),
		jump_point_map(std::move(jump_point_map_m)),
		m_last_ob_g(std::move(last_ob_g)),
		m_nei_ob_g(std::move(nei_ob_g)),
		m_eHeuristic(std::move(eHeuristic)),
    preTime(std::move(preTime1)),
    jpst_gm_(mmap_),
        m_agentIdx(0),
        m_constraints(nullptr),
        m_lastGoalConstraint(-1),
        m_highLevelExpanded(0),
        m_lowLevelExpanded(0),
		m_lowLevelGeneration(0){
		  goalID = m_goal.y * m_dimx + m_goal.x;
    // computeHeuristic();
  }

  Environment(const Environment&) = delete;
  Environment& operator=(const Environment&) = delete;

  void setLowLevelContext(size_t agentIdx, const Constraints* constraints) {
    assert(constraints);  // NOLINT
    m_agentIdx = agentIdx;
    m_goal = m_goals[agentIdx];
    m_constraints = constraints;
    m_lastGoalConstraint = -1;
    m_lowLevelGeneration = 0;
    for (const auto& vc : constraints->vertexConstraints) {
      if (vc.x == m_goals[m_agentIdx].x && vc.y == m_goals[m_agentIdx].y) {
        m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
      }
    }
  }

  int admissibleHeuristic(const State& s) {
    // std::cout << "H: " <<  s << " " << m_heuristic[m_agentIdx][s.x + m_dimx *
    // s.y] << std::endl;
    // return m_heuristic[m_agentIdx][s.x + m_dimx * s.y];
	if(isExact){
		if(m_eHeuristic[m_goals[m_agentIdx]][s.x][s.y] == -1) return INT_MAX;
		else return m_eHeuristic[m_goals[m_agentIdx]][s.x][s.y];
	} else return std::abs(s.x - m_goals[m_agentIdx].x) +
	           std::abs(s.y - m_goals[m_agentIdx].y);

/*    return std::abs(s.x - m_goals[m_agentIdx].x) +
           std::abs(s.y - m_goals[m_agentIdx].y);*/
  }


  int admissibleHeuristic(const Location& s) {
    // std::cout << "H: " <<  s << " " << m_heuristic[m_agentIdx][s.x + m_dimx *
    // s.y] << std::endl;
    // return m_heuristic[m_agentIdx][s.x + m_dimx * s.y];
	if(isExact){
		if(m_eHeuristic[m_goals[m_agentIdx]][s.x][s.y] == -1) return INT_MAX;
		else return m_eHeuristic[m_goals[m_agentIdx]][s.x][s.y];
	} else return std::abs(s.x - m_goals[m_agentIdx].x) +
           std::abs(s.y - m_goals[m_agentIdx].y);
  }

  int admissibleHeuristic(const Location& s, unsigned int dir) {
    // std::cout << "H: " <<  s << " " << m_heuristic[m_agentIdx][s.x + m_dimx *
    // s.y] << std::endl;
    // return m_heuristic[m_agentIdx][s.x + m_dimx * s.y];
	if(isExact){
		if(m_eHeuristic[m_goals[m_agentIdx]][s.x][s.y] == -1) return INT_MAX;
		else return m_eHeuristic[m_goals[m_agentIdx]][s.x][s.y];
	} else return std::abs(s.x - m_goals[m_agentIdx].x) +
           std::abs(s.y - m_goals[m_agentIdx].y);
  }

  // bool isSolution(const State& s) {
  //   return s.x == m_goals[m_agentIdx].x && s.y == m_goals[m_agentIdx].y &&
  //          s.time > m_lastGoalConstraint;
  // }
  bool isSolution(const State& s) {
    if(!isSeg) return s.x == m_goal.x && s.y == m_goal.y && s.time > m_lastGoalConstraint;
    if(isSeg) return s.x == m_goal.x && s.y == m_goal.y;
  }  
  bool isSolution(const Location& s) {
//	  std::cout << " Goals " << m_goal.x << " -- " << m_goal.y << "\n";
	  return s == m_goal;
  }

  bool isSameXY(const State& s){return (s.x == m_goal.x || s.y == m_goal.y);}
  bool isSameXY(const Location& s) {return s == m_goal;}

	void getNeighbors(const Location& s,
                  std::vector<Neighbor<Location, Action, int> >& neighbors) {
		neighbors.clear();

		Location left(s.x - 1, s.y);
		if (stateValid(left)) {
			neighbors.emplace_back(
					Neighbor<Location, Action, int>(left, Action::Left, 1));
		}
		Location right(s.x + 1, s.y);
		if (stateValid(right)) {
			neighbors.emplace_back(
					Neighbor<Location, Action, int>(right, Action::Right, 1));
		}

		Location up(s.x, s.y + 1);
		if (stateValid(up)) {
			neighbors.emplace_back(Neighbor<Location, Action, int>(up, Action::Up, 1));
		}

		Location down(s.x, s.y - 1);
		if (stateValid(down)) {
			neighbors.emplace_back(
					Neighbor<Location, Action, int>(down, Action::Down, 1));
		}
	}

  void getNeighbors(const State& s,
                    std::vector<Neighbor<State, Action, int> >& neighbors) {
    // std::cout << "#VC " << constraints.vertexConstraints.size() << std::endl;
    // for(const auto& vc : constraints.vertexConstraints) {
    //   std::cout << "  " << vc.time << "," << vc.x << "," << vc.y <<
    //   std::endl;
    // }
//	  if(isOutput) std::cout << "Current state : " << s.x << ", " << s.y << " time " << s.time << " f " << admissibleHeuristic(s) << "  -----------\n";
    neighbors.clear();
   if(isTemporalEdgeConstraintAfterT(Location(s.x + 1, s.y), s.time) || isTemporalEdgeConstraintAfterT(Location(s.x - 1, s.y), s.time)
    		|| isTemporalEdgeConstraintAfterT(Location(s.x, s.y - 1), s.time) || isTemporalEdgeConstraintAfterT(Location(s.x, s.y + 1), s.time)
			|| isTemporalEdgeConstraintAfterT(Location(s.x + 1, s.y - 1), s.time) || isTemporalEdgeConstraintAfterT(Location(s.x - 1, s.y - 1), s.time)
			|| isTemporalEdgeConstraintAfterT(Location(s.x + 1, s.y + 1), s.time) || isTemporalEdgeConstraintAfterT(Location(s.x - 1, s.y + 1), s.time)
			|| isTemporalObstacleAfterT(Location(s.x + 1, s.y), s.time) || isTemporalObstacleAfterT(Location(s.x - 1, s.y), s.time)
			|| isTemporalObstacleAfterT(Location(s.x, s.y - 1), s.time) || isTemporalObstacleAfterT(Location(s.x, s.y + 1), s.time)
			|| isTemporalObstacleAfterT(Location(s.x + 1, s.y - 1), s.time) || isTemporalObstacleAfterT(Location(s.x - 1, s.y - 1), s.time)
			|| isTemporalObstacleAfterT(Location(s.x + 1, s.y + 1), s.time) || isTemporalObstacleAfterT(Location(s.x - 1, s.y + 1), s.time))
      //  if(isTemporalEdgeConstraint(Location(s.x + 1, s.y)) || isTemporalEdgeConstraint(Location(s.x - 1, s.y))
      //   		|| isTemporalEdgeConstraint(Location(s.x, s.y - 1)) || isTemporalEdgeConstraint(Location(s.x, s.y + 1))
    	// 		|| isTemporalEdgeConstraint(Location(s.x + 1, s.y - 1)) || isTemporalEdgeConstraint(Location(s.x - 1, s.y - 1))
    	// 		|| isTemporalEdgeConstraint(Location(s.x + 1, s.y + 1)) || isTemporalEdgeConstraint(Location(s.x - 1, s.y + 1))
    	// 		|| isTemporalObstacle(Location(s.x + 1, s.y)) || isTemporalObstacle(Location(s.x - 1, s.y))
    	// 		|| isTemporalObstacle(Location(s.x, s.y - 1)) || isTemporalObstacle(Location(s.x, s.y + 1))
    	// 		|| isTemporalObstacle(Location(s.x + 1, s.y - 1)) || isTemporalObstacle(Location(s.x - 1, s.y - 1))
    	// 		|| isTemporalObstacle(Location(s.x + 1, s.y + 1)) || isTemporalObstacle(Location(s.x - 1, s.y + 1)))
    {
      State n(s.time + 1, s.x, s.y);
      m_lowLevelGeneration++;
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Wait, 1));
//        if(isOutput) std::cout << "Succ---- : " << n.x << ", " << n.y << " time " << s.time + 1 << " f " << admissibleHeuristic(n) << "  \n";
      }
    }
    {
      State n(s.time + 1, s.x - 1, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Left, 1));
//        if(isOutput) std::cout << "Succ---- : " << n.x << ", " << n.y << " time " << s.time + 1 << " f " << admissibleHeuristic(n) << "  \n";
      }
    }
    {
      State n(s.time + 1, s.x + 1, s.y);
      m_lowLevelGeneration++;
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Right, 1));
//        if(isOutput) std::cout << "Succ---- : " << n.x << ", " << n.y << " time " << s.time + 1 << " f " << admissibleHeuristic(n) << "  \n";
      }
    }
    {
      State n(s.time + 1, s.x, s.y + 1);
      m_lowLevelGeneration++;
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(Neighbor<State, Action, int>(n, Action::Up, 1));
//        if(isOutput) std::cout << "Succ---- : " << n.x << ", " << n.y << " time " << s.time + 1 << " f " << admissibleHeuristic(n) << "  \n";
      }
    }
    {
      State n(s.time + 1, s.x, s.y - 1);
      m_lowLevelGeneration++;
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Down, 1));
//        if(isOutput) std::cout << "Succ---- : " << n.x << ", " << n.y << " time " << s.time + 1 << " f " << admissibleHeuristic(n) << "  \n";
      }
    }
  }

  typedef struct PathPoint
  {
    PathPoint(int m_x, int m_y, int m_delta_t, int m_init_t, Action m_ac, int m_path_id, int m_point_id){
      x = m_x; y = m_y; delta_t = m_delta_t; init_t = m_init_t; ac = m_ac; path_id = m_path_id; point_id = m_point_id;
    }
    int x, y;
    int delta_t;
    int init_t;
    Action ac;    
    int path_id;
    int point_id;
    
    /* data */
  }PathPoint;

  static bool comparsion(PathPoint p_a, PathPoint p_b){
    return p_a.init_t < p_b.init_t;
  }

 bool CheckValid(
      std::vector<PlanResult<Location, Action, int> >& solution){

    std::vector<PlanResult<Location, Action, int>> solution_path(solution.size());
    
    int max_t = 0;
    for(size_t i = 0; i < solution.size(); i++){
      int tt = 0;
      Location a(-1, -1), b(-1, -1); 
      int time_a, time_b;
      for(size_t jump_point_id = 0; jump_point_id < solution[i].states.size(); jump_point_id++){

        if(jump_point_id == solution[i].states.size() - 1){
          solution_path[i].states.push_back(solution[i].states[jump_point_id]);
          tt++;
         if(tt > max_t) max_t = tt;          
          continue;
        }
        a = solution[i].states[jump_point_id].first;
        b = solution[i].states[jump_point_id + 1].first;
        time_a = solution[i].states[jump_point_id].second;
        time_b = solution[i].states[jump_point_id + 1].second;
        int delta_t = time_b - time_a;
        int flag_y = 1;
        Action ac_c;
        if(a.y > b.y) { flag_y = -1; ac_c = Action::Down;}
        else { flag_y = 1; ac_c = Action::Up;}

        for(int temp_y = 0; temp_y < abs(a.y - b.y); temp_y++){ //from 0 insure the first location is added
          Location temp_loc(a.x, a.y+flag_y*temp_y);
          solution_path[i].states.push_back(std::make_pair<>(temp_loc, time_a + temp_y));
          solution_path[i].actions.push_back(std::make_pair<>(ac_c, 1));
          if(isObstacle(temp_loc)){
            return false;
          }
          tt++;
        }
        if(a.x != b.x){
          Location temp_loc(a.x, a.y+flag_y*abs(a.y-b.y));
          solution_path[i].states.push_back(std::make_pair<>(temp_loc, time_a + abs(a.y - b.y)));
          solution_path[i].actions.push_back(std::make_pair<>(ac_c, 1));
          if(isObstacle(temp_loc)){
            return false;
          }
          tt++;        
        }
        int flag_x = 1;
        if(a.x <= b.x){flag_x = 1; ac_c = Action::Right;}
        else{flag_x = -1; ac_c = Action::Left;}
        for(int temp_x = 1; temp_x < abs(a.x - b.x); temp_x++){// from 1 insure the last location is not added
          Location temp_loc(a.x + flag_x*temp_x, b.y);
          solution_path[i].states.push_back(std::make_pair<>(temp_loc, time_a + abs(a.y - b.y)+temp_x-1));
          solution_path[i].actions.push_back(std::make_pair<>(ac_c, 1));
          if(isObstacle(temp_loc)){
            return false;
          }
          tt++;
        } 
        if(delta_t != abs(a.x - b.x) + abs(a.y - b.y)){
          Location temp_loc(-1, -1);
          if(a.x == b.x){ temp_loc.x = a.x, temp_loc.y = b.y - flag_y;}
          else{temp_loc.x = b.x - flag_x; temp_loc.y = b.y;}
          if(a.x == b.x && a.y == b.y) temp_loc = a;
          if(isObstacle(temp_loc)){
            return false;
          }
        	// 	for (size_t ii = 0; ii < solution[i].actions.size(); ++ii) {
        	// 		std::cout << solution[i].states[ii].second << ": " <<
        	// 					solution[i].states[ii].first << "->" << solution[i].actions[ii].first
					// 			<< "(cost: " << solution[i].actions[ii].second << ")" << std::endl;
        	// 	}
        	// 	std::cout << solution[i].states.back().second << ": " <<
        	// 	  		   solution[i].states.back().first << std::endl;

          // }
          int timed = abs(a.x - b.x) + abs(a.y - b.y);
          for(int temp_w = 0; temp_w  < delta_t - timed; temp_w++){
            solution_path[i].states.push_back(std::make_pair<>(temp_loc, time_a + timed - 1 +temp_w));
            solution_path[i].actions.push_back(std::make_pair<>(Action::Wait, 1));
            tt++;
          }
        }
      }
      if(tt - 1 != solution[i].cost){
           std::cout << "recover path is not correct\n";
        return false;
      }
    }

    Conflict result;
    std::vector<std::unordered_set<int>> jump_point(solution.size());
    for (int t = 0; t < max_t; ++t) {
      for (size_t i = 0; i < solution_path.size(); ++i) {
        Location state1 = getState(i, solution_path, t);
        for (size_t j = i + 1; j < solution_path.size(); ++j) {
          Location state2 = getState(j, solution_path, t);
          if (state1 == state2) {
            return false;
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Vertex;
            result.x1 = state1.x;
            result.y1 = state1.y;
          }
        }
      }
      // if(is_restart) continue;
      // drive-drive edge (swap)
      for (size_t i = 0; i < solution_path.size(); ++i) {
        Location state1a = getState(i, solution_path, t);
        Location state1b = getState(i, solution_path, t + 1);
        for (size_t j = i + 1; j < solution_path.size(); ++j) {
          Location state2a = getState(j, solution_path, t);
          Location state2b = getState(j, solution_path, t + 1);
          if (state1a == state2b && state1b == state2a) {
            return false;
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Edge;
            result.x1 = state1a.x;
            result.y1 = state1a.y;
            result.x2 = state1b.x;
            result.y2 = state1b.y;
            
          }
        }
      }
      
    }
    return true;    
  }


  bool getFirstConflict(
      std::vector<PlanResult<Location, Action, int> >& solution,
      Conflict& result, int& jump_id) {
    bool isprint = false;
    std::vector<PathPoint> point_t; //store the line points of all paths 
    std::vector<PathPoint> pool_k; //for each path store the current point,the size is the number of paths
    int jump_id_conf = -1;
    //Construct the line points
    for(size_t i = 0; i < solution.size(); i++){
      Location a(-1, -1), b(-1, -1); 
      int time_a, time_b;
      int delta_t;
      Action ac;
      bool is_first = true; //put the first point into the state pool
      for(int j = 0; j < solution[i].states.size() - 1; j++){
        a = solution[i].states[j].first;
        b = solution[i].states[j + 1].first;
        time_a = solution[i].states[j].second;
        time_b = solution[i].states[j + 1].second;
        delta_t = time_b - time_a;
        if(a.x == b.x){
          int temp_y = b.y;
          if(a.y > b.y){ ac = Action::Down; temp_y = b.y + 1;}
          else{ ac = Action::Up; temp_y = b.y - 1;}
          if(abs(a.y - b.y) < delta_t){
            if(abs(a.y - b.y) > 1) point_t.emplace_back(a.x, a.y, abs(a.y - b.y) - 1, time_a, ac, i, j);
            point_t.emplace_back(a.x, temp_y, delta_t - abs(a.y - b.y), time_a + abs(a.y-b.y) - 1, Action::Wait, i, j);
            point_t.emplace_back(a.x, temp_y, 1, time_b - 1, ac, i, j);
            if(is_first){
              pool_k.emplace_back(a.x, a.y, abs(a.y - b.y) - 1, time_a, ac, i, j);
              is_first = false;
            }             
          }else{
            point_t.emplace_back(a.x, a.y, delta_t, time_a, ac, i, j);
            if(is_first){
              pool_k.emplace_back(a.x, a.y, delta_t, time_a, ac, i, j);
              is_first = false;
            }
          }
          if(isprint) std::cout << "(" << a.x << ", " << a.y << "),  " << i << ", "<< ac << ", " << time_a << ", " << delta_t<< "  11****************\n";
        }else if(a.y == b.y){
          int temp_x= b.x;
          if(a.x > b.x) {ac = Action::Left; temp_x = b.x + 1;}
          else {ac = Action::Right; temp_x = b.x - 1;}

          if(abs(a.x - b.x) < delta_t){
            if(abs(a.x - b.x) > 1) point_t.emplace_back(a.x, a.y, abs(a.x-b.x) - 1, time_a, ac, i, j);
            point_t.emplace_back(temp_x, a.y, delta_t - abs(a.x - b.x), time_a + abs(a.x - b.x) - 1, Action::Wait, i, j);
            point_t.emplace_back(temp_x, a.y, 1, time_b - 1, ac, i, j);
            if(is_first){
              pool_k.emplace_back(a.x, a.y, abs(a.x-b.x) - 1, time_a, ac, i, j);
              is_first = false;
            }            
          }else{
            point_t.emplace_back(a.x, a.y, delta_t, time_a, ac, i, j);
            if(is_first){
              pool_k.emplace_back(a.x, a.y, delta_t, time_a, ac, i, j);
              is_first = false;
            }            
          }
          if(isprint) std::cout << "(" << a.x << ", " << a.y << "),  " << i << ", "<< ac << ", " << time_a << ", " << delta_t<< " 22****************\n";
        }else{
          if(a.y > b.y) ac = Action::Down;
          else ac = Action::Up;
          int delta_t_1 = abs(a.y - b.y);
          point_t.emplace_back(a.x, a.y, delta_t_1, time_a, ac, i, j);
          if(isprint) std::cout << "(" << a.x << ", " << a.y << "),  " << i << ", "<< ac << ", " << time_a << ", " << delta_t_1 << " 33****************\n";
          if(is_first){
            pool_k.emplace_back(a.x, a.y, delta_t_1, time_a, ac, i, j);
            is_first = false;
          }
          int temp_x = b.x;
          if(a.x > b.x){ ac = Action::Left; temp_x = b.x + 1;
          }else{ temp_x = b.x - 1; ac=  Action::Right;}

          if(abs(a.x - b.x) + abs(a.y - b.y) < delta_t){
            if(abs(a.x -b.x) > 1) point_t.emplace_back(a.x, b.y, abs(a.x - b.x) - 1, time_a + abs(a.y - b.y), ac, i, j);
            point_t.emplace_back(temp_x, b.y, delta_t - abs(a.x - b.x) - abs(a.y - b.y), time_a + abs(a.y - b.y) + abs(a.x - b.x) - 1, Action::Wait, i, j);
            point_t.emplace_back(temp_x, b.y, 1, time_b - 1, ac, i, j);
          }else {
            point_t.emplace_back(a.x, b.y, abs(a.x - b.x), time_a + abs(a.y - b.y), ac, i, j);
          } 
          if(is_first){
            pool_k.emplace_back(a.x, b.y, delta_t, delta_t_1, ac, i, j);
            is_first = false;
          }
        }
      }
      point_t.emplace_back(solution[i].states.back().first.x, solution[i].states.back().first.y, 
                 std::numeric_limits<int>::max(), solution[i].states.back().second, Action::Wait, i, -1);
      if(is_first){
        pool_k.emplace_back(solution[i].states.back().first.x, solution[i].states.back().first.y, 
                 std::numeric_limits<int>::max(), solution[i].states.back().second, Action::Wait, i, -1);
        is_first = false;
      }
      if(isprint) std::cout << "(" << solution[i].states.back().first.x << ", " << solution[i].states.back().first.y << "),  " 
      << i << ", "<< Action::Wait << ", " << solution[i].states.back().second << ", " << std::numeric_limits<int>::max() << " 55****************\n";
    }
    const Location temp_t(0,1);
    int cost_t = 10;
    std::sort(point_t.begin(), point_t.end(), comparsion);

    if(isprint) std::cout << point_t.size() << ", pool " << pool_k.size() << ", After Ssort ------------------------------\n";

    // std::cout << "Find the conflict *****************************************************************************************************\n";
    result.time = std::numeric_limits<int>::max();
    for(size_t ii = 0; ii < point_t.size(); ii++){
      PathPoint current_p = point_t[ii];
      if(current_p.init_t >= result.time) break;
      for(size_t jj = 0; jj < pool_k.size(); jj++){
        if(current_p.path_id == jj){
          pool_k[jj] = current_p;
          continue;
        }
        if(pool_k[jj].init_t > result.time) continue;

        int time_diff = current_p.init_t - pool_k[jj].init_t;
        if(pool_k[jj].ac == Action::Down){
          // if(isprint) std::cout << pool_k[jj].x << ", " << pool_k[jj].y <<", curr " << current_p.x << ", " << current_p.y <<  " -----------------------down \n";
          pool_k[jj].init_t = current_p.init_t;
          pool_k[jj].y = pool_k[jj].y - time_diff;
          pool_k[jj].delta_t = pool_k[jj].delta_t - time_diff;
          int min_delta_t = std::min(current_p.delta_t, pool_k[jj].delta_t);
          if(current_p.ac == Action::Down){
            if(pool_k[jj].x == current_p.x && pool_k[jj].y == current_p.y && current_p.init_t < result.time){
              result.time = current_p.init_t;
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              result.type = Conflict::Vertex;
              result.x1 = pool_k[jj].x;
              result.y1 = pool_k[jj].y;       
              jump_id_conf = current_p.point_id;  
              if(isprint) std::cout << " test 11 \n";    
            }
          } else if(current_p.ac == Action::Up){
            int dis = abs(pool_k[jj].y - current_p.y);
            if(pool_k[jj].x == current_p.x && pool_k[jj].y >= current_p.y && 2*min_delta_t >= dis && current_p.init_t + dis/2 < result.time){
              result.time = current_p.init_t + dis/2;
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              jump_id_conf = current_p.point_id;
              if(dis%2 == 0){
                result.type = Conflict::Vertex;
                result.x1 = pool_k[jj].x;
                result.y1 = pool_k[jj].y - dis/2;
                if(isprint) std::cout << " test 22 \n";                        
              }else{
                result.type = Conflict::Edge;
                result.x1 = current_p.x;
                result.y1 = current_p.y + dis/2;
                result.x2 = pool_k[jj].x;
                result.y2 = pool_k[jj].y - dis/2;    
                if(isprint) std::cout << current_p.x << ", " << current_p.y << ", " << pool_k[jj].x << ", " << pool_k[jj].y << ", deleta " << current_p.delta_t << ", "<< pool_k[jj].delta_t << " test 33 \n";                
              }
            } 
          }else if(current_p.ac == Action::Left){
            if(abs(current_p.x - pool_k[jj].x) == abs(current_p.y- pool_k[jj].y) 
               && current_p.x >= pool_k[jj].x && current_p.y <= pool_k[jj].y
               && min_delta_t >= abs(current_p.y - pool_k[jj].y)
               && current_p.init_t + abs(current_p.x - pool_k[jj].x)< result.time){
              result.time = current_p.init_t + abs(current_p.x - pool_k[jj].x);
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              jump_id_conf = current_p.point_id;

              result.type = Conflict::Vertex;
              result.x1 = pool_k[jj].x;
              result.y1 = current_p.y;
              if(isprint) std::cout << " test 44 -1 \n";                   
            }
          }else if(current_p.ac == Action::Right){
            if(abs(current_p.x - pool_k[jj].x) == abs(current_p.y- pool_k[jj].y) 
               && current_p.x <= pool_k[jj].x && current_p.y <= pool_k[jj].y
               && min_delta_t >= abs(current_p.y - pool_k[jj].y)
               && current_p.init_t + abs(current_p.x - pool_k[jj].x)< result.time){
              result.time = current_p.init_t + abs(current_p.x - pool_k[jj].x);
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              jump_id_conf = current_p.point_id;
              result.type = Conflict::Vertex;
              result.x1 = pool_k[jj].x;
              result.y1 = current_p.y;
              if(isprint) std::cout << " test 44 -2\n";                   
            }
          }else if(current_p.ac == Action::Wait){
            if(current_p.x == pool_k[jj].x && pool_k[jj].y >= current_p.y 
               && pool_k[jj].y -  current_p.y <= min_delta_t
               && current_p.init_t + abs(current_p.y - pool_k[jj].y) < result.time){
              result.time = current_p.init_t + abs(current_p.y - pool_k[jj].y);
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              jump_id_conf = current_p.point_id;
              result.type = Conflict::Vertex;
              result.x1 = current_p.x;
              result.y1 = current_p.y;       
              if(isprint) std::cout << " test 55 \n";                
            }
          }
        }

        if(pool_k[jj].ac == Action::Up){
          // if(isprint) std::cout << pool_k[jj].x << ", " << pool_k[jj].y <<", curr " << current_p.x << ", " << current_p.y <<  " -----------------------up \n";
          pool_k[jj].init_t = current_p.init_t;
          pool_k[jj].y = pool_k[jj].y + time_diff;
          pool_k[jj].delta_t = pool_k[jj].delta_t - time_diff;
          int min_delta_t = std::min(current_p.delta_t, pool_k[jj].delta_t);
          if(current_p.ac == Action::Up){
            if(pool_k[jj].x == current_p.x && pool_k[jj].y == current_p.y && current_p.init_t < result.time){
              result.time = current_p.init_t;
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              jump_id_conf = current_p.point_id;
              result.type = Conflict::Vertex;
              result.x1 = pool_k[jj].x;
              result.y1 = pool_k[jj].y;    
              if(isprint) std::cout << " test 66 \n";              
            }
          } else if(current_p.ac == Action::Down){
            int dis = abs(pool_k[jj].y - current_p.y);
            if(pool_k[jj].x == current_p.x && pool_k[jj].y <= current_p.y && 2*min_delta_t >= dis && current_p.init_t + dis/2 < result.time){
              result.time = current_p.init_t + dis/2;
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              jump_id_conf = current_p.point_id;
              if(dis%2 == 0){
                result.type = Conflict::Vertex;
                result.x1 = pool_k[jj].x;
                result.y1 = pool_k[jj].y + dis/2;    
                if(isprint) std::cout << pool_k[jj].x << "," << pool_k[jj].y + dis/2 << ", " << result.time << " test 77 \n";                    
              }else{
                result.type = Conflict::Edge;
                result.x1 = current_p.x;
                result.y1 = current_p.y - dis/2;
                result.x2 = pool_k[jj].x;
                result.y2 = pool_k[jj].y + dis/2;   
                if(isprint) std::cout << current_p.x  << ", " << current_p.y  << ", " << pool_k[jj].x  << ", " << pool_k[jj].y << ", " << current_p.delta_t << ", " << pool_k[jj].delta_t << " test 88 \n";                 
              }
            } 
          }else if(current_p.ac == Action::Left){
            if(abs(current_p.x - pool_k[jj].x) == abs(current_p.y- pool_k[jj].y)
               && (current_p.x >= pool_k[jj].x && current_p.y >= pool_k[jj].y) 
               && min_delta_t>= abs(current_p.x - pool_k[jj].x)
               && current_p.init_t + abs(current_p.x - pool_k[jj].x) < result.time){
              result.time = current_p.init_t + abs(current_p.x - pool_k[jj].x);
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              jump_id_conf = current_p.point_id;
              result.type = Conflict::Vertex;
              result.x1 = pool_k[jj].x;
              result.y1 = current_p.y;         
              if(isprint) std::cout << current_p.x << ", "<< current_p.y << ", pool " << pool_k[jj].x  << ", " << pool_k[jj].y << ", delta " << current_p.delta_t << ", " << pool_k[jj].delta_t <<  ", "" test 99 -1 \n";          
            }
          }else if(current_p.ac == Action::Right){
            if(abs(current_p.x - pool_k[jj].x) == abs(current_p.y- pool_k[jj].y)
               && (current_p.x <= pool_k[jj].x && current_p.y >= pool_k[jj].y) 
               && min_delta_t>= abs(current_p.x - pool_k[jj].x)
               && current_p.init_t + abs(current_p.x - pool_k[jj].x) < result.time){
              result.time = current_p.init_t + abs(current_p.x - pool_k[jj].x);
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              jump_id_conf = current_p.point_id;
              result.type = Conflict::Vertex;
              result.x1 = pool_k[jj].x;
              result.y1 = current_p.y;         
              if(isprint) std::cout << current_p.x << ", "<< current_p.y << ", pool " << pool_k[jj].x  << ", " << pool_k[jj].y << ", delta " << current_p.delta_t << ", " << pool_k[jj].delta_t <<  ", "" test 99 -2\n";          
            }
          }else if(current_p.ac == Action::Wait){
            if(current_p.x == pool_k[jj].x && pool_k[jj].y <= current_p.y 
               && min_delta_t >= abs(current_p.y - pool_k[jj].y)
               && current_p.init_t + abs(current_p.y - pool_k[jj].y) < result.time){
              result.time = current_p.init_t + abs(current_p.y - pool_k[jj].y);
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              jump_id_conf = current_p.point_id;
              result.type = Conflict::Vertex;
              result.x1 = current_p.x;
              result.y1 = current_p.y;       
              if(isprint) std::cout << " test 1010 \n";                
            }
          }
        }

        if(pool_k[jj].ac == Action::Left){           
          pool_k[jj].init_t = current_p.init_t;
          pool_k[jj].x = pool_k[jj].x - time_diff;
          pool_k[jj].delta_t = pool_k[jj].delta_t - time_diff;
          int min_delta_t = std::min(current_p.delta_t, pool_k[jj].delta_t);
          if(current_p.ac == Action::Left){
            if(pool_k[jj].x == current_p.x && pool_k[jj].y == current_p.y && current_p.init_t < result.time){
              result.time = current_p.init_t;
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              jump_id_conf = current_p.point_id;
              result.type = Conflict::Vertex;
              result.x1 = pool_k[jj].x;
              result.y1 = pool_k[jj].y;
              if(isprint) std::cout << current_p.x << ", " << current_p.y << ", pool " 
               << pool_k[jj].x << ", " << pool_k[jj].y << " delta " << current_p.delta_t << ", "<< pool_k[jj].delta_t << ", " << 
              " init " << current_p.init_t << ", " << pool_k[jj].init_t << ", res "<<result.x1 << ", " << result.x2 << ", time  " << result.time << ", agent " << result.agent1 << ", " << result.agent2 << " test 11111\n";             
              // if(isprint) std::cout << " test 1111 \n";         
            }
          } else if(current_p.ac == Action::Right){
            int dis = abs(pool_k[jj].x - current_p.x);
            if( pool_k[jj].y == current_p.y && pool_k[jj].x >= current_p.x && 2*min_delta_t >= dis && current_p.init_t + dis/2 < result.time){
              result.time = current_p.init_t + dis/2;
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              jump_id_conf = current_p.point_id;
              if(dis%2 == 0){
                result.type = Conflict::Vertex;
                result.x1 = pool_k[jj].x - dis/2;
                result.y1 = pool_k[jj].y;   
                if(isprint) std::cout << " test 1212 \n";                     
              }else{
                result.type = Conflict::Edge;
                result.x1 = current_p.x + dis/2;
                result.y1 = current_p.y;
                result.x2 = pool_k[jj].x - dis/2;
                result.y2 = pool_k[jj].y;       
                if(isprint) std::cout << current_p.x  << ", " << current_p.y << ", " << pool_k[jj].x << ", " << pool_k[jj].y << ", delta " <<  current_p.delta_t << ", " << pool_k[jj].delta_t <<  " test 1313 \n";             
              }
            } 
          }else if(current_p.ac == Action::Up){
            if(abs(current_p.x - pool_k[jj].x) == abs(current_p.y- pool_k[jj].y)
               && current_p.x <= pool_k[jj].x && current_p.y <= pool_k[jj].y 
               && min_delta_t>= abs(current_p.x - pool_k[jj].x)
               && current_p.init_t + abs(current_p.x - pool_k[jj].x) < result.time){
              result.time = current_p.init_t + abs(current_p.x - pool_k[jj].x);
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              jump_id_conf = current_p.point_id;
              result.type = Conflict::Vertex;
              result.x1 = current_p.x;
              result.y1 = pool_k[jj].y;        
              if(isprint) std::cout << pool_k[jj].x << ", " << pool_k[jj].y << ", " << current_p.x << ", " << current_p.y << ", "
              << " init " << pool_k[jj].init_t << ", " << current_p.init_t << ", delta " << pool_k[jj].delta_t <<", " << current_p.delta_t << " test 1414 -1\n";           
            }
          }else if(current_p.ac == Action::Down){
            if(abs(current_p.x - pool_k[jj].x) == abs(current_p.y- pool_k[jj].y)
               && current_p.x <= pool_k[jj].x && current_p.y >= pool_k[jj].y 
               && min_delta_t>= abs(current_p.x - pool_k[jj].x)
               && current_p.init_t + abs(current_p.x - pool_k[jj].x) < result.time){
              result.time = current_p.init_t + abs(current_p.x - pool_k[jj].x);
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              jump_id_conf = current_p.point_id;
              result.type = Conflict::Vertex;
              result.x1 = current_p.x;
              result.y1 = pool_k[jj].y;        
              if(isprint) std::cout << " test 1414 -2\n";           
            }
          }else if(current_p.ac == Action::Wait){
            if(current_p.y == pool_k[jj].y && pool_k[jj].x >= current_p.x 
               && min_delta_t >= abs(current_p.x - pool_k[jj].x)
               && current_p.init_t + abs(current_p.x- pool_k[jj].x) < result.time){
              result.time = current_p.init_t + abs(current_p.x- pool_k[jj].x);
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              jump_id_conf = current_p.point_id;
              result.type = Conflict::Vertex;
              result.x1 = current_p.x;
              result.y1 = current_p.y;      
              if(isprint) std::cout << current_p.x  << ", " << current_p.y << ", " << pool_k[jj].x << ", " << pool_k[jj].y << ", delta " <<  current_p.delta_t << ", " << pool_k[jj].delta_t << " test 1515 \n";                 
            }
          }
        }  

        if(pool_k[jj].ac == Action::Right){
          // if(isprint) std::cout << pool_k[jj].x << ", " << pool_k[jj].y <<", curr " << current_p.x << ", " << current_p.y <<  " ---------------------------Rigth \n";
          pool_k[jj].init_t = current_p.init_t;
          pool_k[jj].x = pool_k[jj].x + time_diff;
          pool_k[jj].delta_t = pool_k[jj].delta_t - time_diff;
          int min_delta_t = std::min(current_p.delta_t, pool_k[jj].delta_t);
          if(current_p.ac == Action::Right){
            if(pool_k[jj].x == current_p.x && pool_k[jj].y == current_p.y && current_p.init_t < result.time){
              result.time = current_p.init_t;
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              jump_id_conf = current_p.point_id;
              result.type = Conflict::Vertex;
              result.x1 = pool_k[jj].x;
              result.y1 = pool_k[jj].y; 
              if(isprint) std::cout << " test 1616 \n";                 
            }
          } else if(current_p.ac == Action::Left){
            int dis = abs(pool_k[jj].x - current_p.x);
            if(pool_k[jj].y == current_p.y && pool_k[jj].x <= current_p.x
               && 2*min_delta_t >= dis && current_p.init_t + dis/2 < result.time){
              result.time = current_p.init_t + dis/2;
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              jump_id_conf = current_p.point_id;
              if(dis%2 == 0){
                result.type = Conflict::Vertex;
                result.x1 = pool_k[jj].x + dis/2;
                result.y1 = pool_k[jj].y;     
                if(isprint) std::cout << pool_k[jj].x << ", " << pool_k[jj].y << ", " << pool_k[jj].init_t << ", " << current_p.x << ", " << current_p.y << ", " << current_p.init_t << " test 1717 \n";                   
              }else{
                result.type = Conflict::Edge;
                result.x1 = current_p.x - dis/2;
                result.y1 = current_p.y;
                result.x2 = pool_k[jj].x + dis/2;
                result.y2 = pool_k[jj].y;   
                if(isprint) std::cout << " test 1818 \n";                 
              }
            } 
          }else if(current_p.ac == Action::Up){
            if(abs(current_p.x - pool_k[jj].x) == abs(current_p.y- pool_k[jj].y) 
               && current_p.x >= pool_k[jj].x && current_p.y <= pool_k[jj].y
               && min_delta_t>= abs(current_p.x - pool_k[jj].x)
               && current_p.init_t + abs(current_p.x - pool_k[jj].x) < result.time){
              result.time = current_p.init_t + abs(current_p.x - pool_k[jj].x);
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              jump_id_conf = current_p.point_id;
              result.type = Conflict::Vertex;
              result.x1 = current_p.x;
              result.y1 = pool_k[jj].y;     
              if(isprint) std::cout << " test 1919 -1\n";              
            }
          }else if(current_p.ac == Action::Down){
            if(abs(current_p.x - pool_k[jj].x) == abs(current_p.y- pool_k[jj].y) 
               && current_p.x >= pool_k[jj].x && current_p.y >= pool_k[jj].y
               && min_delta_t>= abs(current_p.x - pool_k[jj].x)
               && current_p.init_t + abs(current_p.x - pool_k[jj].x) < result.time){
              result.time = current_p.init_t + abs(current_p.x - pool_k[jj].x);
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              jump_id_conf = current_p.point_id;
              result.type = Conflict::Vertex;
              result.x1 = current_p.x;
              result.y1 = pool_k[jj].y;     
              if(isprint) std::cout << result.x1 << ", " << result.y1 << ", " << result.time <<" test 1919 -2\n";              
            }
          }else if(current_p.ac == Action::Wait){
            if(current_p.y == pool_k[jj].y && pool_k[jj].x <= current_p.x 
               && min_delta_t >= abs(current_p.x - pool_k[jj].x)
               && current_p.init_t + abs(current_p.x- pool_k[jj].x) < result.time){
              result.time = current_p.init_t + abs(current_p.x- pool_k[jj].x);
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              jump_id_conf = current_p.point_id;
              result.type = Conflict::Vertex;
              result.x1 = current_p.x;
              result.y1 = current_p.y;   
              if(isprint) std::cout << current_p.x  << ", " << current_p.y  << ", " << pool_k[jj].x  << ", " << pool_k[jj].y << ", delta " << current_p.delta_t << ", " << pool_k[jj].delta_t << " test 2020 \n";                    
            }
          }
        }          

        if(pool_k[jj].ac == Action::Wait){
          // if(isprint) std::cout << pool_k[jj].x << ", " << pool_k[jj].y <<", curr " << current_p.x << ", " << current_p.y <<  " wait ----------\n";
          pool_k[jj].init_t = current_p.init_t;
          pool_k[jj].delta_t = pool_k[jj].delta_t - time_diff;
          int min_delta_t = std::min(current_p.delta_t, pool_k[jj].delta_t);
          if(current_p.ac == Action::Right){
            if(pool_k[jj].x >= current_p.x && pool_k[jj].x - current_p.x <= min_delta_t 
               && pool_k[jj].y == current_p.y && current_p.init_t + abs(pool_k[jj].x - current_p.x) < result.time){
              result.time = current_p.init_t + abs(pool_k[jj].x - current_p.x);
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              jump_id_conf = current_p.point_id;
              result.type = Conflict::Vertex;
              result.x1 = pool_k[jj].x;
              result.y1 = pool_k[jj].y;        
              if(isprint) std::cout << " test 2121 \n";          
            }
          } else if(current_p.ac == Action::Left){
            if(pool_k[jj].x <= current_p.x && pool_k[jj].y == current_p.y 
               && abs(pool_k[jj].x - current_p.x) <= min_delta_t 
               && current_p.init_t + abs(pool_k[jj].x - current_p.x) < result.time){
              result.time = current_p.init_t + abs(pool_k[jj].x - current_p.x);
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              jump_id_conf = current_p.point_id;
              result.type = Conflict::Vertex;
              result.x1 = pool_k[jj].x;
              result.y1 = pool_k[jj].y;
              if(isprint) std::cout << " test 2222 \n";             
            }

          }else if(current_p.ac == Action::Up){
            if(pool_k[jj].y >= current_p.y && pool_k[jj].x == current_p.x
               && min_delta_t>= abs(current_p.y - pool_k[jj].y)
               && current_p.init_t + abs(current_p.y - pool_k[jj].y) < result.time){
              result.time = current_p.init_t + abs(current_p.y - pool_k[jj].y);
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              jump_id_conf = current_p.point_id;
              result.type = Conflict::Vertex;
              result.x1 = pool_k[jj].x;
              result.y1 = pool_k[jj].y;      
              if(isprint) std::cout << current_p.x  << ", " << current_p.y  << ", " << pool_k[jj].x  << ", " << pool_k[jj].y << " delta " << current_p.delta_t << ", " << pool_k[jj].delta_t << " test 2323 -1\n";             
            }
          }else if(current_p.ac == Action::Down){
            if(pool_k[jj].y <= current_p.y && pool_k[jj].x == current_p.x
               && min_delta_t >= abs(current_p.y - pool_k[jj].y)
               && current_p.init_t + abs(current_p.y - pool_k[jj].y) < result.time){
              result.time = current_p.init_t + abs(current_p.y - pool_k[jj].y);
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              jump_id_conf = current_p.point_id;
              result.type = Conflict::Vertex;
              result.x1 = pool_k[jj].x;
              result.y1 = pool_k[jj].y;      
              if(isprint) std::cout << current_p.x << ", " << current_p.y << ", pool " 
               << pool_k[jj].x << ", " << pool_k[jj].y << " delta " << current_p.delta_t << ", "<< pool_k[jj].delta_t << ", " << 
              " init " << current_p.init_t << ", " << pool_k[jj].init_t << ", res "<<result.x1 << ", " << result.x2 << ", time  " << result.time << ", " << result.agent1 << ", " << result.agent2 << " test 2323 -2\n";             
            }
          }else if(current_p.ac == Action::Wait){
            if(current_p.x == pool_k[jj].x && pool_k[jj].y == current_p.y
               && current_p.init_t < result.time){
              result.time = current_p.init_t;
              result.agent1 = current_p.path_id;
              result.agent2 = pool_k[jj].path_id;
              jump_id_conf = current_p.point_id;
              result.type = Conflict::Vertex;
              result.x1 = current_p.x;
              result.y1 = current_p.y;      
              if(isprint) std::cout << " test 2424 \n";                 
            }
          }
        }          
        // if(result.time != std::numeric_limits<int>::max()){
        //   std::cout << current_p.point_id << ", " << pool_k[jj].point_id << ", path id " << current_p.path_id << ", " << pool_k[jj].path_id << " \n";
        // }
      }


    }

    jump_id = jump_id_conf;
    if(result.time != std::numeric_limits<int>::max()) {
      if(isprint) std::cout << " conflict " << result.x1 << ", " << result.y1  << ", time, " << result.time << ", agent, " << result.agent1  << ", " << result.agent2 << ", " << result.type<< " !!!!!!\n";
      return true;
    }else return false;
    
  }

  bool getFirstConflict(
      const std::vector<PlanResult<State, Action, int> >& solution,
      Conflict& result) {
    int max_t = 0;
    for (const auto& sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }

    for (int t = 0; t < max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1 = getState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2 = getState(j, solution, t);
          if (state1.equalExceptTime(state2)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Vertex;
            result.x1 = state1.x;
            result.y1 = state1.y;
            // std::cout << "VC " << t << "," << state1.x << "," << state1.y <<
            // std::endl;
            return true;
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1a = getState(i, solution, t);
        State state1b = getState(i, solution, t + 1);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2a = getState(j, solution, t);
          State state2b = getState(j, solution, t + 1);
          if (state1a.equalExceptTime(state2b) &&
              state1b.equalExceptTime(state2a)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Edge;
            result.x1 = state1a.x;
            result.y1 = state1a.y;
            result.x2 = state1b.x;
            result.y2 = state1b.y;
            return true;
          }
        }
      }
    }

    return false;
  }  

  bool getFirstConflict(
      const std::vector<PlanResult<Location, Action, int> >& solution,
      Conflict& result, bool isFlag) {
    int max_t = 0;
    for (const auto& sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }

    for (int t = 0; t < max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        Location state1 = getState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          Location state2 = getState(j, solution, t);
          if (state1 == state2) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Vertex;
            result.x1 = state1.x;
            result.y1 = state1.y;
            // std::cout << "VC " << t << "," << state1.x << "," << state1.y <<
            // std::endl;
            return true;
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t i = 0; i < solution.size(); ++i) {
        Location state1a = getState(i, solution, t);
        Location state1b = getState(i, solution, t + 1);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          Location state2a = getState(j, solution, t);
          Location state2b = getState(j, solution, t + 1);
          if (state1a == state2b &&
              state1b == state2a) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Edge;
            result.x1 = state1a.x;
            result.y1 = state1a.y;
            result.x2 = state1b.x;
            result.y2 = state1b.y;
            return true;
          }
        }
      }
    }

    return false;
  }  


  void createConstraintsFromConflict(
      const Conflict& conflict, std::map<size_t, Constraints>& constraints) {
    if (conflict.type == Conflict::Vertex) {
      Constraints c1;
      c1.vertexConstraints.emplace(
          VertexConstraint(conflict.time, conflict.x1, conflict.y1));
      constraints[conflict.agent1] = c1;
      constraints[conflict.agent2] = c1;
    } else if (conflict.type == Conflict::Edge) {
      Constraints c1;
      c1.edgeConstraints.emplace(EdgeConstraint(
          conflict.time, conflict.x1, conflict.y1, conflict.x2, conflict.y2));
      constraints[conflict.agent1] = c1;
      Constraints c2;
      c2.edgeConstraints.emplace(EdgeConstraint(
          conflict.time, conflict.x2, conflict.y2, conflict.x1, conflict.y1));
      constraints[conflict.agent2] = c2;
    }
  }

  void createConstraintsFromV(
      int time_h, int xx, int yy, Constraints& constraints) {

      constraints.vertexConstraints.emplace(
          VertexConstraint(time_h, xx, yy));
  }  
  void createConstraintsFromE(
      int time_h, int xx1, int yy1, int xx2, int yy2, Constraints& constraints) {
      
      constraints.edgeConstraints.emplace(
          EdgeConstraint(time_h, xx1, yy1, xx2, yy2));
  }    

  void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded++; }

  void onExpandLowLevelNode(const State& /*s*/, int /*fScore*/,
                            int /*gScore*/) {
    m_lowLevelExpanded++;
  }

  int highLevelExpanded() { return m_highLevelExpanded; }

  int lowLevelExpanded() const { return m_lowLevelExpanded; }
  int lowLevelGenerated() const { return m_lowLevelGeneration; }
 private:
  State getState(size_t agentIdx,
                 const std::vector<PlanResult<State, Action, int> >& solution,
                 size_t t) {
    assert(agentIdx < solution.size());
    if (t < solution[agentIdx].states.size()) {
      return solution[agentIdx].states[t].first;
    }
    assert(!solution[agentIdx].states.empty());
    return solution[agentIdx].states.back().first;
  }
  Location getState(size_t agentIdx,
                 const std::vector<PlanResult<Location, Action, int> >& solution,
                 size_t t) {
    assert(agentIdx < solution.size());
    if (t < solution[agentIdx].states.size()) {
      return solution[agentIdx].states[t].first;
    }
    assert(!solution[agentIdx].states.empty());
    return solution[agentIdx].states.back().first;
  }  
 public:
  void onExpandNode(const Location& /*s*/, int /*fScore*/, int /*gScore*/) {
		// std::cout << "expand: " << s << "g: " << gScore << std::endl;
  }

  void onDiscover(const Location& /*s*/, int /*fScore*/, int /*gScore*/) {
		// std::cout << "  discover: " << s << std::endl;
  }
  bool isCommandValid(
		  const Location& /*s1*/, const Location& /*s2*/, const Action& /*a*/,
		  int earliestStartTime,      // can start motion at this time
		  int /*latestStartTime*/,    // must have left s by this time
		  int earliestArrivalTime,    // can only arrive at (s+cmd)
		  int /*latestArrivalTime*/,  // needs to arrive by this time at (s+cmd)
		  int& t, const int& cost_c) {
	  	 t = std::max<int>(earliestArrivalTime, earliestStartTime + cost_c);

		// TODO(whoenig): need to check for swaps here...
		// return t - 1 <= latestStartTime;
		return true;
  }

  bool isCommandValid(
		  const Location& /*s1*/, const Location& /*s2*/, const Action& /*a*/,
		  int earliestStartTime,      // can start motion at this time
		  int /*latestStartTime*/,    // must have left s by this time
		  int earliestArrivalTime,    // can only arrive at (s+cmd)
		  int /*latestArrivalTime*/,  // needs to arrive by this time at (s+cmd)
		  int& t) {
		t = std::max<int>(earliestArrivalTime, earliestStartTime + 1);

		// TODO(whoenig): need to check for swaps here...
		// return t - 1 <= latestStartTime;
		return true;
  }

  Location getLocation(const Location& s) { return s; }

  int getIndex(const Location& s){
	  return (s.x*m_dimy + s.y);
  }

  bool stateValid(const Location& location){
	  return location.x >= 0 && location.x < m_dimx && location.y >= 0 && location.y < m_dimy &&
				!m_obstacles_m[location.x][location.y];
  }

  bool stateValid(const State& s) {
    assert(m_constraints);
    const auto& con = m_constraints->vertexConstraints;
    return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
           m_obstacles.find(Location(s.x, s.y)) == m_obstacles.end() &&
           con.find(VertexConstraint(s.time, s.x, s.y)) == con.end();
  }

  bool transitionValid(const State& s1, const State& s2) {
    assert(m_constraints);
    const auto& con = m_constraints->edgeConstraints;
    return con.find(EdgeConstraint(s1.time, s1.x, s1.y, s2.x, s2.y)) ==
           con.end();
  }

  bool isBorder(const Location& s){
	return s.x == 0 || s.x == m_dimx - 1 || s.y == 0 || s.y == m_dimy - 1;
  }

  bool isObstacle(const Location& location){
		return location.x >= 0 && location.x < m_dimx && location.y >= 0 && location.y < m_dimy &&
	           m_obstacles_m[location.x][location.y];
  }
  bool isTemporalObstacle(const Location& s){
		return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
			  m_temporal_obstacle[s.x][s.y];
  }

  bool isTemporalEdgeConstraint(const Location& s){
		return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
			  m_temporal_edge_constraint[s.x][s.y] >= 0;
  }

  bool isTemporalEdgeConstraintAfterT(const Location& s, int T){
//	  std::cout << " IsSetEdge--- " << s.x <<", " << s.y << ", T "<< T  << ", va " << m_temporal_edge_constraint[s.x][s.y] <<"\n";
		return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
			  m_temporal_edge_constraint[s.x][s.y]>=T;
  }

  bool isEdgeConstraintAtT(const Location s1, Location s2, int T){
		if(s1.x >= 0 && s1.x < m_dimx && s1.y >= 0 && s1.y < m_dimy){
      assert(m_constraints);
      const auto& con = m_constraints->edgeConstraints;
      return con.find(EdgeConstraint(T, s1.x, s1.y, s2.x, s2.y)) !=
           con.end();    
    }
    else return false;
  }    

  bool isTemporalObstacleAfterT(const Location& s, int T){
		return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy
			  && m_last_ob_g[s.x][s.y] >= T;
  }
  bool isTemporalObstacleAtT(const Location& s, int T){
    assert(m_constraints);
    const auto& con = m_constraints->vertexConstraints;    
		return con.find(VertexConstraint(T, s.x, s.y)) != con.end();
  }    
  void setTemporalEdgeConstraint(const Location& s){
	  m_temporal_edge_constraint[s.x][s.y] = true;
  }

  void setTemporalEdgeConstraint(const Location& s, int T){
//	  std::cout << " SetEdge--- " << s.x <<", " << s.y << ", "<< T <<"\n";
	    m_temporal_edge_constraint[s.x][s.y] = std::max(T, m_temporal_edge_constraint[s.x][s.y]);
      int xx[5] = {0, 1, -1};
      int yy[5] = {0, 1, -1};
      for(int xx_1 = 0; xx_1 < 3; xx_1++){
    	  for(int yy_1 = 0; yy_1 < 3; yy_1++){
    		  if(xx_1 == 0 && yy_1 == 0) continue;
    		  Location temp_state(s.x + xx[xx_1], s.y + yy[yy_1]);
    		  if(stateValid(temp_state)){
    			  m_nei_ob_g[temp_state.x][temp_state.y] = std::max(m_nei_ob_g[temp_state.x][temp_state.y], T);
    		  }
    	  }
      }
  }

  void setTemporalObstacle(const Location& s){
		m_temporal_obstacle[s.x][s.y] = true;
  }

  void setTemporalObstacle(const Location& s, int T){
		m_temporal_obstacle[s.x][s.y] = true;
		m_last_ob_g[s.x][s.y] = std::max(m_last_ob_g[s.x][s.y], T);
        int xx[5] = {0, 1, -1};
        int yy[5] = {0, 1, -1};
		for(int xx_1 = 0; xx_1 < 3; xx_1++){
			for(int yy_1 = 0; yy_1 < 3; yy_1++){
				if(xx_1 == 0 && yy_1 == 0) continue;
				Location temp_state(s.x + xx[xx_1], s.y + yy[yy_1]);
				if(stateValid(temp_state)){
					m_nei_ob_g[temp_state.x][temp_state.y] = std::max(m_nei_ob_g[temp_state.x][temp_state.y], T);
				}
			 }
		 }
		return ;
  }

  void resetTemporalObstacle(){
	  int len = m_temporal_obstacle[0].size();
	  for(int i = 0; i < m_temporal_obstacle.size(); i++){
		  for(int j = 0; j < m_temporal_obstacle[i].size(); j++){
			  m_temporal_obstacle[i][j] = false;
			  m_temporal_edge_constraint[i][j] = -1;
			  m_last_ob_g[i][j] = -1;
			  m_nei_ob_g[i][j] = -1;
		  }
	  }
  }
  bool isJumpPoint(const Location& s) {
		return jump_point_map[s.x][s.y];
  }

  bool isJumpPoint(const Location& s, int time) {
		return jump_point_map[s.x][s.y] || m_nei_ob_g[s.x][s.y] >= time - 1;
  }

  void setJumpPoint(const Location& s){
		jump_point_map[s.x][s.y] = true;
  }

  void setCAT(bool isCAT_tt){
    isCAT = isCAT_tt;
  }
  
  void setBP(bool isBP_tt){
    isBP = isBP_tt;
  }

  void setJPS(){
		is_jps = true;
  }
  void setNoJPS(){
		is_jps = false;
  }

  bool isJPS(){
		return is_jps;
  }

  void Reset(){
		num_generation = 0;
		num_expansion = 0;
  }
Location  setGoal(int agentId){
	  m_goal = m_goals[agentId];
	  m_agentIdx = agentId;
	  Location goal = m_goal;
	  isOutput = true;
    goalID = getNodeId(m_goal);
	  return goal;
  }
  void setExactHeuristTrue(){
	  isExact = true;
  }
  void setExactHeuristFalse(){
	  isExact = false;
  }
  void setIsSegPlanning(bool isSeg_t){
	  isSeg = isSeg_t;
  }


#if 0
  // We use another A* search for simplicity
  // we compute the shortest path to each goal by using the fact that our getNeighbor function is
  // symmetric and by not terminating the AStar search until the queue is empty
  void computeHeuristic()
  {
    class HeuristicEnvironment
    {
    public:
      HeuristicEnvironment(
        size_t dimx,
        size_t dimy,
        const std::unordered_set<Location>& obstacles,
        std::vector<int>* heuristic)
        : m_dimx(dimx)
        , m_dimy(dimy)
        , m_obstacles(obstacles)
        , m_heuristic(heuristic)
      {
      }

      int admissibleHeuristic(
        const Location& s)
      {
        return 0;
      }

      bool isSolution(
        const Location& s)
      {
        return false;
      }

      void getNeighbors(
        const Location& s,
        std::vector<Neighbor<Location, Action, int> >& neighbors)
      {
        neighbors.clear();

        {
          Location n(s.x-1, s.y);
          if (stateValid(n)) {
            neighbors.emplace_back(Neighbor<Location, Action, int>(n, Action::Left, 1));
          }
        }
        {
          Location n(s.x+1, s.y);
          if (stateValid(n)) {
            neighbors.emplace_back(Neighbor<Location, Action, int>(n, Action::Right, 1));
          }
        }
        {
          Location n(s.x, s.y+1);
          if (stateValid(n)) {
            neighbors.emplace_back(Neighbor<Location, Action, int>(n, Action::Up, 1));
          }
        }
        {
          Location n(s.x, s.y-1);
          if (stateValid(n)) {
            neighbors.emplace_back(Neighbor<Location, Action, int>(n, Action::Down, 1));
          }
        }
      }

      void onExpandNode(
        const Location& s,
        int fScore,
        int gScore)
      {
      }

      void onDiscover(
        const Location& s,
        int fScore,
        int gScore)
      {
        (*m_heuristic)[s.x + m_dimx * s.y] = gScore;
      }

    private:
      bool stateValid(
        const Location& s)
      {
        return    s.x >= 0
               && s.x < m_dimx
               && s.y >= 0
               && s.y < m_dimy
               && m_obstacles.find(Location(s.x, s.y)) == m_obstacles.end();
      }

    private:
      int m_dimx;
      int m_dimy;
      const std::unordered_set<Location>& m_obstacles;
      std::vector<int>* m_heuristic;

    };

    m_heuristic.resize(m_goals.size());

    std::vector< Neighbor<State, Action, int> > neighbors;

    for (size_t i = 0; i < m_goals.size(); ++i) {
      m_heuristic[i].assign(m_dimx * m_dimy, std::numeric_limits<int>::max());
      HeuristicEnvironment henv(m_dimx, m_dimy, m_obstacles, &m_heuristic[i]);
      AStar<Location, Action, int, HeuristicEnvironment> astar(henv);
      PlanResult<Location, Action, int> dummy;
      astar.search(m_goals[i], dummy);
      m_heuristic[i][m_goals[i].x + m_dimx * m_goals[i].y] = 0;
    }
  }
#endif

	bool isFCheck()
	{
		return isFI;
	}

  	uint32_t getGoalId()
	{
		return goalID;
		return getNodeId(m_goal);
	}

	int getNodeId(const Location &s)
	{
		return (s.y * m_dimx + s.x);
	}

	void setGoal(Location goal){
		m_goal = goal;
    goalID = getNodeId(m_goal);    
    isOutput = true;
	}

	void setGoal(Location goal, int agentId){
		m_goal = goal;
    m_agentIdx = agentId;
    goalID = getNodeId(m_goal);    
    isOutput = true;
	}

  double getPreTime(int i){
    return preTime[i];
  }

	int getDimX() { return m_dimx; }
	int getDimY() { return m_dimy; }

public:
  int num_generation = 0;
  int num_expansion = 0;
  int limit_jump = 32;
  int m_dimx;
  int m_dimy;
 private:
  std::unordered_set<Location> m_obstacles;
  std::vector<Location> m_goals;
  Location m_goal;
  uint32_t goalID;


  size_t m_agentIdx;
  const Constraints* m_constraints;
  int m_lastGoalConstraint;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;
  int m_lowLevelGeneration;
  bool is_limit = false;
  bool is_jps = true;
  bool isOutput = false;
  bool isExact = true;
  bool isFI = true;

  std::vector<std::vector<bool>> m_obstacles_m;
  std::vector<std::vector<bool>> m_temporal_obstacle;
  std::vector<std::vector<int>> m_temporal_edge_constraint;
  std::vector<std::vector<bool>> jump_point_map;
  std::vector<std::vector<int>> m_last_ob_g;
  std::vector<std::vector<int>> m_nei_ob_g;
  std::unordered_map<Location, std::vector<std::vector<int>>> m_eHeuristic;
  std::vector<double> preTime;

 public:
  jpst_gridmap *jpst_gm_;
  bool isDebug = false;
  bool isSeg = false;
  bool isBP = true;
  bool isCAT = true; 
};


void getExactHeuristic(std::vector<std::vector<int>>& eHeuristic, std::vector<std::vector<bool>>map_obstacle, Location goal, int dimx, int dimy){
    int xx[5] = {0, 0, -1, 1};
    int yy[5] = {1, -1, 0, 0};

	eHeuristic[goal.x][goal.y] = 0;
	std::queue<Location> que;
	que.push(goal);

	while(true){
		int queSize = que.size();
		if(queSize == 0) break;
		for(int i = 0; i < queSize; i++){
			Location curr = que.front();
			int currValue = eHeuristic[curr.x][curr.y];
			que.pop();
			for(int ii = 0; ii < 4; ii++){
				Location nei(curr.x + xx[ii], curr.y + yy[ii]);
				if(curr.x + xx[ii] < 0 || curr.y + yy[ii] < 0 || curr.x + xx[ii] >= dimx || curr.y + yy[ii] >= dimy
						|| map_obstacle[nei.x][nei.y]) continue;
				if(eHeuristic[nei.x][nei.y] == -1){
					eHeuristic[nei.x][nei.y] = currValue + 1;
					que.push(nei);
				}
			}
		}
	}
}


int main(int argc, char* argv[]) {
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  std::string solver;
  int numAgent = INT_MAX;
  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input file (YAML)")
      ("output,o", po::value<std::string>(&outputFile)->required(), "output file (YAML)")
			("agents,a", po::value<int>(&numAgent)->required(), "number agents")
      ("solver,s", po::value<string>(&solver)->required(), "solver name");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  std::cout << "File " << inputFile << " \n";

  YAML::Node config = YAML::LoadFile(inputFile);

  std::unordered_set<Location> obstacles;
  std::vector<Location> goals;
  std::vector<State> startStates;

  const auto& dim = config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  std::vector<std::vector<bool>> map_jump_point(dimx+1, std::vector<bool>(dimy+1));
  std::vector<std::vector<bool>> map_obstacle(dimx+1, std::vector<bool>(dimy+1));
  std::vector<std::vector<bool>> map_temporal_obstacle(dimx, std::vector<bool>(dimy+1));
  std::vector<std::vector<int>> map_temporal_edge_constraint(dimx, std::vector<int>(dimy+1));
  std::vector<std::vector<int>> last_ob_g(dimx+1, std::vector<int>(dimy+1));
  std::vector<std::vector<int>> nei_ob_g(dimx+1, std::vector<int>(dimy+1));

  gridmap gm(dimy, dimx);
  for (const auto& node : config["map"]["obstacles"]) {
    obstacles.insert(Location(node[0].as<int>(), node[1].as<int>()));
    map_obstacle[node[0].as<int>()][node[1].as<int>()] = true;
  }

	for (int i = 0; i < dimx; i++)
	{
		for (int j = 0; j < dimy; j++)
		{
			if (map_obstacle[i][j])
			{
				gm.set_label(gm.to_padded_id(i, j), 0);
			}
			else
			{
				gm.set_label(gm.to_padded_id(i, j), 1);
			}
		}
	}
	jpst_gridmap jpst_gm_(&gm);

  for (const auto& ob:obstacles){
	  Location temp1 = ob,temp2 = ob;
	  temp1.x = ob.x + 1;
	  temp2.y = ob.y + 1;

	  if(temp1.x >= 0 && temp1.x <= dimx && temp2.y >=0 && temp2.y <= dimy
			  && obstacles.find(temp1)==obstacles.end() && obstacles.find(temp2) == obstacles.end()){
		  map_jump_point[ob.x + 1][ob.y + 1] = true;
	  }
	  temp1 = ob; temp2 = ob;
	  temp1.x = ob.x + 1;
	  temp2.y = ob.y - 1;
	  if(temp1.x >= 0 && temp1.x <= dimx && temp2.y >=0 && temp2.y <= dimy
			  && obstacles.find(temp1)==obstacles.end() && obstacles.find(temp2) == obstacles.end()){
		  map_jump_point[ob.x + 1][ob.y - 1] = true;
	  }

	  temp1 = ob; temp2 = ob;
	  temp1.x = ob.x - 1;
	  temp2.y = ob.y + 1;
	  if(temp1.x >= 0 && temp1.x <= dimx && temp2.y >=0 && temp2.y <= dimy
			  && obstacles.find(temp1)==obstacles.end() && obstacles.find(temp2) == obstacles.end()){
		  map_jump_point[ob.x - 1][ob.y + 1] = true;
	  }

	  temp1 = ob; temp2 = ob;
	  temp1.x = ob.x - 1;
	  temp2.y = ob.y - 1;
	  if(temp1.x >= 0 && temp1.x <= dimx && temp2.y >=0 && temp2.y <= dimy
			  && obstacles.find(temp1)==obstacles.end() && obstacles.find(temp2) == obstacles.end()){
			map_jump_point[ob.x - 1][ob.y - 1] = true;
	  }

  }

  int num_agent = 0;
  std::unordered_map<Location, std::vector<std::vector<int>>> eHeuristic;
  std::vector<double> preTime;
  int agent_limit = numAgent;
  numAgent = 100;
  preTime.resize(numAgent+1);
  Timer t;
  for (const auto& node : config["agents"]) {
    const auto& start = node["start"];
    const auto& goal = node["goal"];
    startStates.emplace_back(State(0, start[0].as<int>(), start[1].as<int>()));
    // std::cout << "s: " << startStates.back() << std::endl;
    goals.emplace_back(Location(goal[0].as<int>(), goal[1].as<int>()));

    t.reset();
    std::vector<std::vector<int>> eHeuristicGoal(dimx+1, std::vector<int>(dimy+1, -1));
    getExactHeuristic(eHeuristicGoal, map_obstacle, goals[num_agent], dimx, dimy);
    eHeuristic[goals[num_agent]] = eHeuristicGoal;
    t.stop();
    double preT1 = t.elapsedSeconds();
    preTime[num_agent] = preT1;
    std::cout << numAgent <<  "(" << startStates[num_agent].x << " " << startStates[num_agent].y <<
    		")" << " , " <<  "(" << goals[num_agent].x << " " << goals[num_agent].y <<
    		")" << ", " << preT1 << "\n";
    num_agent++;
    if(num_agent > numAgent) break;
  }
  t.stop();
  std::cout << "Here \n";
  double preT = t.elapsedSeconds();

  std::cout << " size " << goals.size() << " numAgent " << numAgent + 1 << " PreTime " << preT << " \n";
  Environment mapf(dimx, dimy, obstacles, goals, goals[0], map_obstacle,
		  map_temporal_obstacle, map_temporal_edge_constraint, map_jump_point, 
      last_ob_g, nei_ob_g, eHeuristic, preTime, &jpst_gm_);
  CBS<State, Location, Action, int, Conflict, Constraints, Environment> cbs(mapf);
  std::vector<PlanResult<Location, Action, int> > solution;

  CBSJPSTAstar<State, Location, Action, int, Conflict, Constraints, Environment> cbs_jpsta(mapf);
  std::vector<PlanResult<Location, Action, int> > solution_jpsta;


  CBSSIPP<State, Location, Action, int, Conflict, Constraints, Environment> cbs_sipp(mapf);
  std::vector<PlanResult<Location, Action, int> > solution_sipp;


  CBSAstar<State, Location, Action, int, Conflict, Constraints, Environment> cbs_astar(mapf);
  std::vector<PlanResult<State, Action, int> > solution_astar;

  CBSCAstar<State, Location, Action, int, Conflict, Constraints, Environment> cbs_castar(mapf);
  std::vector<PlanResult<State, Action, int> > solution_castar;
  Timer timer;
  // bool success1 = cbs_jpsta.search(startStates, solution_jpsta);
  // timer.stop();

  // if(success1) std::cout << inputFile << " Planning successful! time " << timer.elapsedSeconds() << std::endl;
  
  //  mapf.setCAT(false);

  // Timer timer2;
  // bool success2 = cbs_astar.search(startStates, solution_astar);
  // timer2.stop();   
  // if(success1) std::cout << inputFile << " Planning successful! time " << timer2.elapsedSeconds() << std::endl;
  // return 0;

  // std::cout << startStates.size() << " Here \n";
  mapf.setBP(false);
  bool successJpst = true, successSipp = true, successA = true, successCA = true;
  bool successJpstA = true, successSippNoCat = true, successANoCAT = true, successCANoCAT = true, successJpstNoBP = true;
  int num_agent_iter = 0;
  std::vector<State> startStates_temp;

  string solver_jpst = "JPST";
  string solver_jpsta = "JPSTA";
  string solver_sipp = "SIPP";
  string solver_astar = "Astar";
  string solver_castar = "CAstar";
  string solver_jpstnobp = "JPSTNoBP";
  string solver_ancat = "AstarNoCAT";
  string solver_sippncat = "SIPPNoCAT";
  string solver_cancat = "CastarNoCAT";
 
  mapf.setBP(true);
  while(true)
  // || successSipp || successA || successCA || successJpstA || successSippNoCat || successANoCAT || successCANoCAT || successJpstNoBP)
  {
  //  || successSipp || successA || successA || successJpstA || successSippNoCat || successANoCAT || successJpstNoBP){

    startStates_temp.push_back(startStates[num_agent_iter]);

    if(solver == solver_jpst){
      Timer timer_t1;
      std::cout << "JPST-J, " << num_agent_iter << ", ";
      solution.clear();
      successJpst = cbs.search(startStates_temp, solution);
      timer_t1.stop();
      if(successJpst) std::cout << " Planning successful! Yeah time " << timer_t1.elapsedSeconds() <<  ", " << inputFile << std::endl;
      else std::cout << " Planning NOT successful! time " << timer_t1.elapsedSeconds() << " ," << inputFile  << std::endl;
    }
    if(solver == solver_sipp){
      Timer timer_t2;
      std::cout << "SIPPCAT, " << num_agent_iter << ", ";
      successSipp = cbs_sipp.search(startStates_temp, solution_sipp);
      timer_t2.stop();
      if(successSipp) std::cout << " Planning successful! Yeah time " << timer_t2.elapsedSeconds()  << ", " << inputFile <<  std::endl;
      else std::cout << " Planning NOT successful! time " << timer_t2.elapsedSeconds() << ", " << inputFile <<  std::endl;
    }

    if(solver == solver_astar){
      Timer timer_t3;
      std::cout << "AstarCAT-1, "<< num_agent_iter << ", ";
      successA = cbs_astar.search(startStates_temp, solution_astar);
      timer_t3.stop();
      if(successA) std::cout << " Planning successful! Yeah time " << timer_t3.elapsedSeconds() << ", " << inputFile <<  std::endl;
      else std::cout << " Planning NOT successful! time " << timer_t3.elapsedSeconds() << ", " << inputFile << std::endl;
    }

    if(solver == solver_castar){
      Timer timer_t4;
      std::cout << "CAstarCAT, " << num_agent_iter << ", ";
      successCA= cbs_castar.search(startStates_temp, solution_castar);
      timer_t4.stop();
      if(successCA) std::cout << " Planning successful! Yeah time " << timer_t4.elapsedSeconds() << ", " << inputFile <<  std::endl;
      else std::cout << " Planning NOT successful! time " << timer_t4.elapsedSeconds() << ", " << inputFile <<  std::endl;
    }
    
    if(solver == solver_jpsta){
      Timer timer_t8;
      std::cout << "JpstAstar, " << num_agent_iter << ", ";
      solution_jpsta.clear();
      successJpstA = cbs_jpsta.search(startStates_temp, solution_jpsta);
      timer_t8.stop();
      if(successJpstA) std::cout << " Planning successful! Yeah time " << timer_t8.elapsedSeconds() << ", "<< inputFile << std::endl;
      else std::cout << " Planning NOT successful! time " << timer_t8.elapsedSeconds() << ", " << inputFile << std::endl;
    }

    // mapf.setBP(false);

    if(solver == solver_sippncat){
      Timer timer_t5;
      mapf.setCAT(false);
      std::cout << "SIPPNoCAT, " << num_agent_iter << ", ";
      successSippNoCat = cbs_sipp.search(startStates_temp, solution_sipp);
      timer_t5.stop();
      if(successSippNoCat) std::cout << " Planning successful! Yeah time " << timer_t5.elapsedSeconds() << ", "<< inputFile <<  std::endl;
      else std::cout << " Planning NOT successful! time " << timer_t5.elapsedSeconds() << ", " << inputFile << std::endl;
      mapf.setCAT(true);
    }

    if(solver == solver_ancat){
      Timer timer_t6;
      mapf.setCAT(false);
      std::cout << "AstarNoCAT-1, "<< num_agent_iter << ", ";
      successANoCAT = cbs_astar.search(startStates_temp, solution_astar);
      timer_t6.stop();
      if(successANoCAT) std::cout  << " Planning successful! Yeah time " << timer_t6.elapsedSeconds() << ", "<< inputFile << std::endl;
      else std::cout << " Planning NOT successful! time " << timer_t6.elapsedSeconds() << ", " << inputFile << std::endl;
      mapf.setCAT(true);
    }

    if(solver == solver_cancat){
      Timer timer_t9;
      mapf.setCAT(false);
      std::cout << "CAstarNoCAT, "<< num_agent_iter << ", ";
      successCANoCAT = cbs_castar.search(startStates_temp, solution_castar);
      timer_t9.stop();
      if(successCANoCAT) std::cout  << " Planning successful! Yeah time " << timer_t9.elapsedSeconds() << ", "<< inputFile << std::endl;
      else std::cout << " Planning NOT successful! time " << timer_t9.elapsedSeconds() << ", " << inputFile << std::endl;
      mapf.setCAT(true);
    }

    if(solver == solver_jpstnobp){
      Timer timer_t7;
      mapf.setCAT(false);
      std::cout << "JPST-NoBP, " << num_agent_iter << ", ";
      solution.clear();
      successJpstNoBP = cbs.search(startStates_temp, solution);
      timer_t7.stop();
      if(successJpstNoBP) std::cout << " Planning successful! Yeah time " << timer_t7.elapsedSeconds() << ", " << inputFile << std::endl;
      else std::cout << " Planning NOT successful! time " << timer_t7.elapsedSeconds() << ", " << inputFile  << std::endl;
      mapf.setCAT(true);
    }
    // mapf.setBP(true);
    // sleep(5);
    std::cout.flush();
    num_agent_iter++;
    if(num_agent_iter > startStates.size() - 1 || num_agent_iter > agent_limit) break;
  } 

  return 0;

  // Timer timer;
  // std::cout << "jpst " << ", ";
  // bool success = cbs.search(startStates, solution);
  // timer.stop();
  // std::cout << inputFile << " Planning successful! time " << timer.elapsedSeconds() << std::endl;

  // std::cout << "sipp " << ", ";
  // Timer tsipp;
  // bool successSipp = cbs_sipp.search(startStates, solution_sipp);
  // tsipp.stop();
  // std::cout << inputFile << " Planning successful! time " << tsipp.elapsedSeconds() << std::endl;

  // std::cout << "astar " << ", ";
  // bool successA = cbs_astar.search(startStates, solution_astar);

  // std::cout << "castar " << ", ";
  // bool successCA= cbs_castar.search(startStates, solution_castar);


  if (successJpst) {
    return 0;
    std::cout << inputFile << " Planning successful! time " << timer.elapsedSeconds() << std::endl;
    int cost = 0;
    int makespan = 0;
    for (const auto& s : solution) {
      cost += s.cost;
      makespan = std::max<int>(makespan, s.cost);
    }

    std::ofstream out(outputFile);
    out << "statistics:" << std::endl;
    out << "  cost: " << cost << std::endl;
    out << "  makespan: " << makespan << std::endl;
    out << "  runtime: " << timer.elapsedSeconds() << std::endl;
    out << "  highLevelExpanded: " << mapf.highLevelExpanded() << std::endl;
    out << "  lowLevelExpanded: " << mapf.lowLevelExpanded() << std::endl;
    out << "schedule:" << std::endl;
    for (size_t a = 0; a < solution.size(); ++a) {
      std::cout << "Solution for: " << a << std::endl;
      for (size_t i = 0; i < solution[a].actions.size(); ++i) {
        std::cout << solution[a].states[i].second << ": " <<
        solution[a].states[i].first << "->" << solution[a].actions[i].first
        << "(cost: " << solution[a].actions[i].second << ")" << std::endl;
      }
      std::cout << solution[a].states.back().second << ": " <<
      solution[a].states.back().first << std::endl;

      out << "  agent" << a << ":" << std::endl;
      for (const auto& state : solution[a].states) {
        out << "    - x: " << state.first.x << std::endl
            << "      y: " << state.first.y << std::endl
            << "      t: " << state.second << std::endl;
      }
    }
  } else {
    std::cout << inputFile << " Planning NOT successful!" << std::endl;
  }

  return 0;
}
