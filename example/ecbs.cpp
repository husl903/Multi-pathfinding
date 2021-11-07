#include <fstream>
#include <iostream>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include <libMultiRobotPlanning/ecbs.hpp>
#include <libMultiRobotPlanning/ecbs_jpst.hpp>
#include <libMultiRobotPlanning/ecbs_sipp.hpp>
#include <libMultiRobotPlanning/gridmap.hpp>
#include <libMultiRobotPlanning/jpst_gridmap.hpp>

//#include "timer.hpp"

using libMultiRobotPlanning::ECBS;
using libMultiRobotPlanning::ECBSJPST;
using libMultiRobotPlanning::ECBSSIPP;
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

  unsigned int dir = 0xff;
  int time;
  int x;
  int y;
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
    return os << "VC(" << c.time << "," << c.x << "," << c.y << ")";
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
    return os << "EC(" << c.time << "," << c.x1 << "," << c.y1 << "," << c.x2
              << "," << c.y2 << ")";
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
    for (const auto& vc : c.vertexConstraints) {
      os << vc << std::endl;
    }
    for (const auto& ec : c.edgeConstraints) {
      os << ec << std::endl;
    }
    return os;
  }
};

struct Location {
  Location(int x, int y) : x(x), y(y) {}
  Location(){x = -1; y = -1;}
  int x;
  int y;

  bool operator<(const Location& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
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
              std::vector<Location> goals, std::vector<std::vector<bool>>m_obstacle_matrix,
			  std::vector<std::vector<bool>>temporal_obstacle,
			  std::vector<std::vector<int>>temporal_edge_constraint,
			  std::vector<std::vector<bool>>jump_point_map_m,
			  std::vector<std::vector<int>>last_ob_g,
  	  	  	  std::vector<std::vector<int>>nei_ob_g,
			  std::unordered_map<Location, std::vector<std::vector<int>>> eHeuristic,
        jpst_gridmap *mmap_
      )
      : m_dimx(dimx),
        m_dimy(dimy),
        m_obstacles(std::move(obstacles)),
        m_goals(std::move(goals)),
        m_obstacles_m(std::move(m_obstacle_matrix)),
		    m_temporal_obstacle(std::move(temporal_obstacle)),
		    m_temporal_edge_constraint(std::move(temporal_edge_constraint)),
		    jump_point_map(std::move(jump_point_map_m)),
		    m_last_ob_g(std::move(last_ob_g)),
		    m_nei_ob_g(std::move(nei_ob_g)),
		    m_eHeuristic(std::move(eHeuristic)),
        jpst_gm_(mmap_),
        m_agentIdx(0),
        m_constraints(nullptr),
        m_lastGoalConstraint(-1),
        m_highLevelExpanded(0),
        m_lowLevelExpanded(0) {}

  Environment(const Environment&) = delete;
  Environment& operator=(const Environment&) = delete;

  void setLowLevelContext(size_t agentIdx, const Constraints* constraints) {
    assert(constraints);
    m_agentIdx = agentIdx;
    m_constraints = constraints;
    m_lastGoalConstraint = -1;
    for (const auto& vc : constraints->vertexConstraints) {
      if (vc.x == m_goals[m_agentIdx].x && vc.y == m_goals[m_agentIdx].y) {
        m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
      }
    }
  }

  int admissibleHeuristic(const State& s) {
    if(isPerfectH){
      if(!isSeg){
  		  if(m_eHeuristic[m_goals[m_agentIdx]][s.x][s.y] == -1) return INT_MAX;
	  	  else return m_eHeuristic[m_goals[m_agentIdx]][s.x][s.y];        
      }else{
        if(m_eHeuristic[m_goals[m_agentIdx]][s.x][s.y] == -1) return INT_MAX;
        else{
          return abs(m_eHeuristic[m_goals[m_agentIdx]][s.x][s.y] - m_eHeuristic[m_goals[m_agentIdx]][m_goal.x][m_goal.y]);
        }
      }
    }
    else return std::abs(s.x - m_goals[m_agentIdx].x) +
           std::abs(s.y - m_goals[m_agentIdx].y);
  }

  int admissibleHeuristic(const Location& s) {
    // std::cout << "H: " <<  s << " " << m_heuristic[m_agentIdx][s.x + m_dimx *
    // s.y] << std::endl;
    // return m_heuristic[m_agentIdx][s.x + m_dimx * s.y];
	  if(isPerfectH){
		  if(m_eHeuristic[m_goals[m_agentIdx]][s.x][s.y] == -1) return INT_MAX;
		  else return m_eHeuristic[m_goals[m_agentIdx]][s.x][s.y];
	  } else return std::abs(s.x - m_goals[m_agentIdx].x) +
           std::abs(s.y - m_goals[m_agentIdx].y);
  }


  // low-level
  int focalStateHeuristic(
      const State& s, int /*gScore*/,
      const std::vector<PlanResult<State, Action, int> >& solution) {
    int numConflicts = 0;
    for (size_t i = 0; i < solution.size(); ++i) {
      if (i != m_agentIdx && !solution[i].states.empty()) {
        State state2 = getState(i, solution, s.time);
        if (s.equalExceptTime(state2)) {
          ++numConflicts;
        }
      }
    }
    return numConflicts;
  }

  // low-level
  int focalTransitionHeuristic(
      const State& s1a, const State& s1b, int /*gScoreS1a*/, int /*gScoreS1b*/,
      const std::vector<PlanResult<State, Action, int> >& solution) {
    int numConflicts = 0;
    for (size_t i = 0; i < solution.size(); ++i) {
      if (i != m_agentIdx && !solution[i].states.empty()) {
        State s2a = getState(i, solution, s1a.time);
        State s2b = getState(i, solution, s1b.time);
        if (s1a.equalExceptTime(s2b) && s1b.equalExceptTime(s2a)) {
          ++numConflicts;
        }
      }
    }
    return numConflicts;
  }

  int focalStateHeuristic(
      const Location& s, int gScore,
      const std::vector<PlanResult<Location, Action, int> >& solution, bool is_jpst) {
    int numConflicts = 0;
    if(is_jpst){
      for (size_t i = 0; i < solution.size(); ++i) {
        if (i != m_agentIdx && !solution[i].states.empty()) {
          for(size_t location_t = 0; location_t < solution[i].states.size(); location_t++){
            if(solution[i].states[location_t].second > gScore) break;
            if(solution[i].states[location_t].first == s
               && solution[i].states[location_t].second == gScore
               ) numConflicts++;          
          }
        }
      }
    }else{
      for (size_t i = 0; i < solution.size(); ++i) {
        if (i != m_agentIdx && !solution[i].states.empty()) {
          Location state2 = getState(i, solution, gScore);
          if (s == state2) {
            ++numConflicts;
          }
        }
      }      
    }
    return numConflicts;
  }

  int focalTransitionHeuristic(
      const Location& s1a, const Location& s1b, int gScoreS1a, int gScoreS1b,
      const std::vector<PlanResult<Location, Action, int> >& solution) {
    int numConflicts = 0;
    for (size_t i = 0; i < solution.size(); ++i) {
      if (i != m_agentIdx && !solution[i].states.empty()) {
        Location s2a = getState(i, solution, gScoreS1a);
        Location s2b = getState(i, solution, gScoreS1b);
        if (s1a == s2b && s1b == s2a) {
          ++numConflicts;
        }
      }
    }
    return numConflicts;
  }  
  // Count all conflicts
  int focalHeuristic(
      const std::vector<PlanResult<State, Action, int> >& solution) {
    int numConflicts = 0;

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
            ++numConflicts;
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
            ++numConflicts;
          }
        }
      }
    }
    return numConflicts;
  }

  bool isSolution(const State& s) {
    if(!isSeg) return s.x == m_goals[m_agentIdx].x && s.y == m_goals[m_agentIdx].y &&
           s.time > m_lastGoalConstraint;
    if(isSeg) return s.x == m_goal.x && s.y == m_goal.y;
  }

  bool isSolution(const Location& s) {
//	  std::cout << " Goals " << m_goal.x << " -- " << m_goal.y << "\n";
	  return s == m_goal;
  }

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
    neighbors.clear();
    {
      State n(s.time + 1, s.x, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Wait, 1));
      }
    }
    {
      State n(s.time + 1, s.x - 1, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Left, 1));
      }
    }
    {
      State n(s.time + 1, s.x + 1, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Right, 1));
      }
    }
    {
      State n(s.time + 1, s.x, s.y + 1);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(Neighbor<State, Action, int>(n, Action::Up, 1));
      }
    }
    {
      State n(s.time + 1, s.x, s.y - 1);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Down, 1));
      }
    }
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
          solution_path[i].states.push_back(std::make_pair<>(temp_loc, time_a + abs(a.y - b.y)+temp_x));
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
            solution_path[i].states.push_back(std::make_pair<>(temp_loc, time_a + timed +temp_w));
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

    // for (size_t a = 0; a < solution_path.size(); ++a) {
    //   std::cout << "Solution for: " << a << std::endl;
    //   for (size_t i = 0; i < solution_path[a].actions.size(); ++i) {
    //     std::cout << solution_path[a].states[i].second << ": " <<
    //     solution_path[a].states[i].first << "->" << solution_path[a].actions[i].first
    //     << "(cost: " << solution_path[a].actions[i].second << ")" << std::endl;
    //   }
    //   std::cout << solution_path[a].states.back().second << ": " <<
    //   solution_path[a].states.back().first << std::endl;
    // }    

    Conflict result;
    std::vector<std::unordered_set<int>> jump_point(solution.size());
    for (int t = 0; t < max_t; ++t) {
      for (size_t i = 0; i < solution_path.size(); ++i) {
        Location state1 = getState(i, solution_path, t);
        for (size_t j = i + 1; j < solution_path.size(); ++j) {
          Location state2 = getState(j, solution_path, t);
          if (state1 == state2) {
            std::cout << "VertexConflict " << i << ", " << j << " xy " << state1.x << ", " << state1.y <<  ", time " << t << " \n";
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
            std::cout << "EdgeConflict " << i << ", " << j << " xy " << state1a.x << ", " << state1a.y << ", " << state1b.x << ", " << state1b.y  << ", " << t << " \n";
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

 bool  getAllConflicts(
      std::vector<PlanResult<Location, Action, int> >& solution,
      std::vector<Conflict>& conflicts_all){
    std::vector<PlanResult<Location, Action, int>> solution_path(solution.size());
    std::vector<std::vector<int>> point_id(solution.size());
    std::vector<std::vector<int>> point_st(solution.size());
    Conflict result;
    int max_t = 0;
    int num_cft = 0;
    for(size_t i = 0; i < solution.size(); i++){
      int tt = 0;
      Location a(-1, -1), b(-1, -1); 
      int time_a, time_b;
      for(size_t jump_point_id = 0; jump_point_id < solution[i].states.size(); jump_point_id++){
        if(jump_point_id == solution[i].states.size() - 1){
          solution_path[i].states.push_back(solution[i].states[jump_point_id]);
          point_id[i].push_back(jump_point_id);         
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

        point_st[i].push_back(tt);
        for(int temp_y = 0; temp_y < abs(a.y - b.y); temp_y++){
          Location temp_loc(a.x, a.y+flag_y*temp_y);
          solution_path[i].states.push_back(std::make_pair<>(temp_loc, time_a + temp_y));
          solution_path[i].actions.push_back(std::make_pair<>(ac_c, 1));
          point_id[i].push_back(jump_point_id);
          tt++;
        }
        if(a.x != b.x){
          Location temp_loc(a.x, a.y+flag_y*abs(a.y-b.y));
          solution_path[i].states.push_back(std::make_pair<>(temp_loc, time_a + abs(a.y - b.y)));
          solution_path[i].actions.push_back(std::make_pair<>(ac_c, 1));
          point_id[i].push_back(jump_point_id); 
          tt++;        
        }
        int flag_x = 1;
        if(a.x <= b.x){flag_x = 1; ac_c = Action::Right;}
        else{flag_x = -1; ac_c = Action::Left;}
        for(int temp_x = 1; temp_x < abs(a.x - b.x); temp_x++){
          Location temp_loc(a.x + flag_x*temp_x, b.y);
          solution_path[i].states.push_back(std::make_pair<>(temp_loc, time_a + abs(a.y - b.y)+temp_x));
          solution_path[i].actions.push_back(std::make_pair<>(ac_c, 1));
          point_id[i].push_back(jump_point_id);
          tt++;
        } 
        if(delta_t != abs(a.x - b.x) + abs(a.y - b.y)){
          Location temp_loc(-1, -1);
          if(a.x == b.x){ temp_loc.x = a.x, temp_loc.y = b.y - flag_y;}
          else{temp_loc.x = b.x - flag_x; temp_loc.y = b.y;}
          if(a.x == b.x && a.y == b.y) temp_loc = a;
          int timed = abs(a.x - b.x) + abs(a.y - b.y);
          for(int temp_w = 0; temp_w  < delta_t - timed; temp_w++){
            solution_path[i].states.push_back(std::make_pair<>(temp_loc, time_a + timed + temp_w));
            solution_path[i].actions.push_back(std::make_pair<>(Action::Wait, 1));
            point_id[i].push_back(jump_point_id);
            tt++;
          }
        }
      }
      if(tt - 1 != solution[i].cost){
        std::cout <<"Agent " <<  i << "recover path is not correct " << tt - 1 << solution[i].cost << std::endl;
        return false;
      }
    }

    // for (size_t a = 0; a < solution.size(); ++a) {
    //   std::cout << "Solution for: " << a << std::endl;
    //   for (size_t i = 0; i < solution[a].actions.size(); ++i) {
    //     std::cout << solution[a].states[i].second << ": " <<
    //     solution[a].states[i].first << "->" << solution[a].actions[i].first
    //     << "(cost: " << solution[a].actions[i].second << ")" << std::endl;
    //   }
    //   std::cout << solution[a].states.back().second << ": " <<
    //   solution[a].states.back().first << std::endl;
    // }

    // for (size_t a = 0; a < solution_path.size(); ++a) {
    //   std::cout << "Solution for: " << a << std::endl;
    //   for (size_t i = 0; i < solution_path[a].actions.size(); ++i) {
    //     std::cout << solution_path[a].states[i].second << ": " <<
    //     solution_path[a].states[i].first << "->" << solution_path[a].actions[i].first
    //     << "(cost: " << solution_path[a].actions[i].second << ")" << std::endl;
    //   }
    //   std::cout << solution_path[a].states.back().second << ": " <<
    //   solution_path[a].states.back().first << std::endl;
      
    // }

    bool is_restart = false;
    std::vector<std::unordered_set<int>> jump_point(solution.size());
    for (int t = 0; t < max_t; ++t) {
      is_restart = false;
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution_path.size(); ++i) {
        Location state1 = getState(i, solution_path, t);
        for (size_t j = i + 1; j < solution_path.size(); ++j) {
          Location state2 = getState(j, solution_path, t);
          if (state1 == state2) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Vertex;
            result.x1 = state1.x;
            result.y1 = state1.y;
            conflicts_all.push_back(result);
            num_cft++;
            // if(t >= point_id[i].size()) jump_id = -1;
            // else jump_id = point_id[i][t];
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
            if(state1a.x == state1b.x && state1a.y == state1b.y) continue;
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Edge;
            result.x1 = state1a.x;
            result.y1 = state1a.y;
            result.x2 = state1b.x;
            result.y2 = state1b.y;
            num_cft++;
            // if(t >= point_id[i].size()) jump_id = -1;
            // else jump_id = point_id[i][t];
            conflicts_all.push_back(result);
          }
        }
      }
    }
    if(conflicts_all.size() > 0) return true;
    else return false;    
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

  void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded++; }

  void onExpandLowLevelNode(const State& /*s*/, int /*fScore*/,
                            int /*gScore*/) {
    m_lowLevelExpanded++;
  }
  Location getLocation(const Location& s) { return s; }
  size_t getAgentId(){ return m_agentIdx;}

  int highLevelExpanded() { return m_highLevelExpanded; }

  int lowLevelExpanded() const { return m_lowLevelExpanded; }

  int getDimX() {return m_dimx;}
  int getDimY() {return m_dimy;}

  int getIndex(const Location& s){
	  return (s.x*m_dimy + s.y);
  }

	int getNodeId(const Location &s)
	{
		return (s.y * m_dimx + s.x);
	}

  uint32_t getGoalId()
	{
		return goalID;
		return getNodeId(m_goal);
	}

  Location  setGoal(int agentId){
	  m_agentIdx = agentId;
    m_goal = m_goals[agentId];
    goalID = getNodeId(m_goal);
    return m_goal;
  }

	void setGoal(Location goal){
		m_goal = goal;
    goalID = getNodeId(m_goal);    
	}

	void setGoal(Location goal, int agentId){
		m_goal = goal;
    m_agentIdx = agentId;
    goalID = getNodeId(m_goal);
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

  // void setCAT(bool isCAT_tt){
  //   isCAT = isCAT_tt;
  // }
  
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

	bool isFCheck()
	{
		return isFI;
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
  bool stateValid(const Location& location){
	  return location.x >= 0 && location.x < m_dimx && location.y >= 0 && location.y < m_dimy &&
				!m_obstacles_m[location.x][location.y];
  }

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

 private:
  int m_dimx;
  int m_dimy;
  std::unordered_set<Location> m_obstacles;
  std::vector<Location> m_goals;
  std::vector<std::vector<bool>> m_obstacles_m;
  std::vector<std::vector<bool>> m_temporal_obstacle;
  std::vector<std::vector<int>> m_temporal_edge_constraint;
  std::vector<std::vector<bool>> jump_point_map;
  std::vector<std::vector<int>> m_last_ob_g;
  std::vector<std::vector<int>> m_nei_ob_g;
  std::unordered_map<Location, std::vector<std::vector<int>>> m_eHeuristic;
  size_t m_agentIdx;
  const Constraints* m_constraints;
  int m_lastGoalConstraint;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;
 public:
  jpst_gridmap *jpst_gm_;
 private:
  Location m_goal;
  uint32_t goalID;
 public:
  int num_generation = 0;
  int num_expansion = 0;
  int limit_jump = 32;
  bool isDebug = false;
  bool isSeg = false;
  bool isBP = true;
  // bool isCAT = true;
  bool is_jps = true;
  bool isPerfectH = false;
  bool isFI = true;
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
  float w;
  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input file (YAML)")("output,o",
                           po::value<std::string>(&outputFile)->required(),
                           "output file (YAML)")(
      "suboptimality,w", po::value<float>(&w)->default_value(1.0),
      "suboptimality bound");

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


  int num_agent = 100;
  int index_agent = 0;
  std::unordered_map<Location, std::vector<std::vector<int>>> eHeuristic;
  std::vector<double> preTime;
  preTime.resize(num_agent+1);
  Timer t;

  for (const auto& node : config["agents"]) {
    const auto& start = node["start"];
    const auto& goal = node["goal"];
    startStates.emplace_back(State(0, start[0].as<int>(), start[1].as<int>()));
    // std::cout << "s: " << startStates.back() << std::endl;
    goals.emplace_back(Location(goal[0].as<int>(), goal[1].as<int>()));
    std::vector<std::vector<int>> eHeuristicGoal(dimx+1, std::vector<int>(dimy+1, -1));
    getExactHeuristic(eHeuristicGoal, map_obstacle, goals[index_agent], dimx, dimy);
    eHeuristic[goals[index_agent]] = eHeuristicGoal;
    t.stop();
    double preT1 = t.elapsedSeconds();
    preTime[index_agent] = preT1;
    std::cout << index_agent <<  "(" << startStates[index_agent].x << " " << startStates[index_agent].y <<
    		")" << " , " <<  "(" << goals[index_agent].x << " " << goals[index_agent].y <<
    		")" << ", " << preT1 << "\n";

    index_agent++;
    if(index_agent > num_agent) break;
  }

  Environment mapf(dimx, dimy, obstacles, goals, map_obstacle,
		  map_temporal_obstacle, map_temporal_edge_constraint, map_jump_point , 
      last_ob_g, nei_ob_g, eHeuristic, &jpst_gm_);

  // Environment mapf(dimx, dimy, obstacles, goals);
  ECBS<State, Action, int, Conflict, Constraints, Environment> cbs(mapf, w);
  
  ECBSJPST<State, Location, Action, int, Conflict, Constraints, Environment> cbs_jpst(mapf, w);
  ECBSSIPP<State, Location, Action, int, Conflict, Constraints, Environment> cbs_sipp(mapf, w);
  

  std::vector<PlanResult<State, Action, int> > solution;
  std::vector<PlanResult<Location, Action, int> > solution_jpst;
  std::vector<PlanResult<Location, Action, int> > solution_sipp;

  Timer timer1;
  bool success = cbs_jpst.search(startStates, solution_jpst);
  timer1.stop();
  std::cout << "cbs_jpst time1 " << timer1.elapsedSeconds() << "----------------------------------------------------------\n";

  Timer timer2;
  success = cbs_sipp.search(startStates, solution_sipp);
  timer2.stop();
  std::cout << "cbs_sipp time1 " << timer2.elapsedSeconds() << "----------------------------------------------------------\n";

  Timer timer;
  success = cbs.search(startStates, solution);
  timer.stop();
  std::cout << "cbs_jpst time2 " << timer.elapsedSeconds() << std::endl;
  if (success) {
    std::cout << "Planning successful! " << std::endl;
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
      // std::cout << "Solution for: " << a << std::endl;
      // for (size_t i = 0; i < solution[a].actions.size(); ++i) {
      //   std::cout << solution[a].states[i].second << ": " <<
      //   solution[a].states[i].first << "->" << solution[a].actions[i].first
      //   << "(cost: " << solution[a].actions[i].second << ")" << std::endl;
      // }
      // std::cout << solution[a].states.back().second << ": " <<
      // solution[a].states.back().first << std::endl;

      out << "  agent" << a << ":" << std::endl;
      for (const auto& state : solution[a].states) {
        out << "    - x: " << state.first.x << std::endl
            << "      y: " << state.first.y << std::endl
            << "      t: " << state.second << std::endl;
      }
    }
  } else {
    std::cout << "Planning NOT successful!" << std::endl;
  }

  return 0;
}
