#include <fstream>
#include <iostream>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include <libMultiRobotPlanning/cbs.hpp>
#include "timer.hpp"

using libMultiRobotPlanning::CBS;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;

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
  	  	  	  std::vector<std::vector<int>>nei_ob_g
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
        m_agentIdx(0),
        m_constraints(nullptr),
        m_lastGoalConstraint(-1),
        m_highLevelExpanded(0),
        m_lowLevelExpanded(0) {
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
    return std::abs(s.x - m_goals[m_agentIdx].x) +
           std::abs(s.y - m_goals[m_agentIdx].y);
  }
  int admissibleHeuristic(const Location& s) {
    // std::cout << "H: " <<  s << " " << m_heuristic[m_agentIdx][s.x + m_dimx *
    // s.y] << std::endl;
    // return m_heuristic[m_agentIdx][s.x + m_dimx * s.y];
    return std::abs(s.x - m_goals[m_agentIdx].x) +
           std::abs(s.y - m_goals[m_agentIdx].y);
  }

  bool isSolution(const State& s) {
    return s.x == m_goals[m_agentIdx].x && s.y == m_goals[m_agentIdx].y &&
           s.time > m_lastGoalConstraint;
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

  int highLevelExpanded() { return m_highLevelExpanded; }

  int lowLevelExpanded() const { return m_lowLevelExpanded; }

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
			  m_temporal_edge_constraint[s.x][s.y] > 0;
  }

  bool isTemporalEdgeConstraintAfterT(const Location& s, int T){
//	  std::cout << " IsSetEdge--- " << s.x <<", " << s.y << ", T "<< T  << ", va " << m_temporal_edge_constraint[s.x][s.y] <<"\n";
		return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
			  m_temporal_edge_constraint[s.x][s.y]>=T;
  }

  bool isTemporalObstacleAfterT(const Location& s, int T){
		return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy
			  && m_last_ob_g[s.x][s.y] >= T;
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
			  m_last_ob_g[i][j] = 0;
			  m_nei_ob_g[i][j] = 0;
		  }
	  }
  }
  bool isJumpPoint(const Location& s) {
		return jump_point_map[s.x][s.y];
  }

  bool isJumpPoint(const Location& s, int time) {
		return jump_point_map[s.x][s.y] || m_nei_ob_g[s.x][s.y]>=time;
  }

  void setJumpPoint(const Location& s){
		jump_point_map[s.x][s.y] = true;
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
		num_expansion =0;
  }
  void setGoal(int agentId){
	  m_goal = m_goals[agentId];
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

  size_t m_agentIdx;
  const Constraints* m_constraints;
  int m_lastGoalConstraint;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;
  bool is_limit = false;
  bool is_jps = true;

  std::vector<std::vector<bool>> m_obstacles_m;
  std::vector<std::vector<bool>> m_temporal_obstacle;
  std::vector<std::vector<int>> m_temporal_edge_constraint;
  std::vector<std::vector<bool>> jump_point_map;
  std::vector<std::vector<int>> m_last_ob_g;
  std::vector<std::vector<int>> m_nei_ob_g;

};

int main(int argc, char* argv[]) {
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input file (YAML)")("output,o",
                           po::value<std::string>(&outputFile)->required(),
                           "output file (YAML)");

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

  for (const auto& node : config["map"]["obstacles"]) {
    obstacles.insert(Location(node[0].as<int>(), node[1].as<int>()));
    map_obstacle[node[0].as<int>()][node[1].as<int>()] = true;
  }

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

  for (const auto& node : config["agents"]) {
    const auto& start = node["start"];
    const auto& goal = node["goal"];
    startStates.emplace_back(State(0, start[0].as<int>(), start[1].as<int>()));
    // std::cout << "s: " << startStates.back() << std::endl;
    goals.emplace_back(Location(goal[0].as<int>(), goal[1].as<int>()));
  }

  Environment mapf(dimx, dimy, obstacles, goals, goals[0], map_obstacle,
		  map_temporal_obstacle, map_temporal_edge_constraint, map_jump_point, last_ob_g, nei_ob_g);
  CBS<State, Location, Action, int, Conflict, Constraints, Environment> cbs(mapf);
  std::vector<PlanResult<State, Action, int> > solution;

  Timer timer;
  bool success = cbs.search(startStates, solution);
  timer.stop();

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
//       std::cout << "Solution for: " << a << std::endl;
//       for (size_t i = 0; i < solution[a].actions.size(); ++i) {
//         std::cout << solution[a].states[i].second << ": " <<
//         solution[a].states[i].first << "->" << solution[a].actions[i].first
//         << "(cost: " << solution[a].actions[i].second << ")" << std::endl;
//       }
//       std::cout << solution[a].states.back().second << ": " <<
//       solution[a].states.back().first << std::endl;

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
