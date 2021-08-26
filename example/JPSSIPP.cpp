#include <fstream>
#include <iostream>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/resource.h>
// #include <crtdbg.h>

#define   RUSAGE_SELF     0
#define   RUSAGE_CHILDREN     -1

#include <yaml-cpp/yaml.h>

#include <libMultiRobotPlanning/JPSSIPP.hpp>
#include <libMultiRobotPlanning/JPSSIPP_BIT.hpp>
#include <libMultiRobotPlanning/JPSSIPP_BITCBS.hpp>
#include <libMultiRobotPlanning/JPSSIPPNEWGEN.hpp>
#include <libMultiRobotPlanning/JPSSIPPAN.hpp>
#include <libMultiRobotPlanning/sipp.hpp>
#include <libMultiRobotPlanning/gridmap.hpp>
#include <libMultiRobotPlanning/jpst_gridmap.hpp>
#include <libMultiRobotPlanning/timer.hpp>

// #include <libMultiRobotPlanning/online_jump_point_locator2.hpp>

using libMultiRobotPlanning::gridmap;
using libMultiRobotPlanning::JPSSIPP;
using libMultiRobotPlanning::JPSSIPP_BIT;
using libMultiRobotPlanning::JPSSIPPNEWGEN;
using libMultiRobotPlanning::JPSSIPPAN;
using libMultiRobotPlanning::jpst_gridmap;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using libMultiRobotPlanning::SIPP;

enum class Action
{
	Up,
	Down,
	Left,
	Right,
	Wait,
	All,
};

struct State
{
	State(int x, int y) : x(x), y(y) {}

	bool operator==(const State &other) const
	{
		return std::tie(x, y) == std::tie(other.x, other.y);
	}

	bool operator!=(const State &other) const
	{
		return std::tie(x, y) != std::tie(other.x, other.y);
	}

	bool operator<(const State &other) const
	{
		return std::tie(x, y) < std::tie(other.x, other.y);
	}

	friend std::ostream &operator<<(std::ostream &os, const State &s)
	{
		return os << "(" << s.x << "," << s.y << ")";
	}
//	int time;
	int x;
	int y;
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

namespace std
{
	template <>
	struct hash<State>
	{
		size_t operator()(const State &s) const
		{
			size_t seed = 0;
			boost::hash_combine(seed, s.x);
			boost::hash_combine(seed, s.y);
			return seed;
		}
	};
} // namespace std

std::ostream &operator<<(std::ostream &os, const Action &a)
{
	switch (a)
	{
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
	case Action::All:
		os << "All";
	}
	return os;
}

class Environment
{
public:
	Environment(size_t dimx, size_t dimy, std::vector<std::vector<bool>> obstacles, std::vector<std::vector<bool>> t_obstacle,
				std::vector<std::vector<bool>> jump_point_map, std::vector<std::vector<int>> last_ob_g,
				std::vector<std::vector<int>> nei_ob_g,
				std::unordered_map<State, std::vector<std::vector<int>>> m_eHeuristic, State goal, jpst_gridmap *mmap_)
		: m_dimx(dimx),
		  m_dimy(dimy),
		  m_obstacles(std::move(obstacles)),
		  m_temporal_obstacle(std::move(t_obstacle)),
		  jump_point_map(std::move(jump_point_map)),
		  last_ob_g(std::move(last_ob_g)),
		  nei_ob_g(std::move(nei_ob_g)),
		  m_eHeuristic(std::move(m_eHeuristic)),
		  m_goal(goal),
		  jpst_gm_(mmap_) {
			  goalID = m_goal.y * m_dimx + m_goal.x;
		}

	~Environment() {}

	float admissibleHeuristic(const State &s)
	{
		if (!stateValid(s))
			return INT_MAX;
		if (isExact)
		{
			if (m_eHeuristic[m_goal][s.x][s.y] == -1)
				return INT_MAX;
			else
				return m_eHeuristic[m_goal][s.x][s.y];
		}
		else
			return std::abs(s.x - m_goal.x) +
				   std::abs(s.y - m_goal.y);
	}

	/*	float admissibleHeuristic(const State& s, unsigned int dir) {

		if(!stateValid(s)) return INT_MAX;
		if(isExact){
			if(m_eHeuristic[m_goal][s.x][s.y] == -1) return INT_MAX;
			else{
				return m_eHeuristic[m_goal][s.x][s.y];
			}
		} else {
			float hvalue = std::abs(s.x - m_goal.x) + std::abs(s.y - m_goal.y);
			if(s.y < m_goal.y ){
				if(s.x == m_goal.x && !(dir & 0x04)){
					if(isTemporalObstacle(State(s.x, s.y + 1)))  return hvalue + 1;
					else return hvalue + 2;
//					if(s.x - 1 >= 0 && isObstacle(State(s.x - 1, s.y + 1))) return hvalue + 4;
//					if(s.x - 1 >= 0 && isTemporalObstacle(State(s.x - 1, s.y + 1))) return hvalue + 2;
				}
				if(s.x > m_goal.x && !(dir & 0x01) && !(dir & 0x04)){
					if(isTemporalObstacle(State(s.x - 1, s.y)) || isTemporalObstacle(State(s.x, s.y + 1))) return hvalue + 1;
					else return hvalue + 2;
				}
				if(s.x > m_goal.x && (dir & 0x01) && !(dir & 0x04)){
					return hvalue;
					int xx;
					for(xx = m_goal.x; xx <= s.x; xx++){
						if(isTemporalObstacle(State(xx, s.y + 1)) || isObstacle(State(xx, s.y+1))) return hvalue;
					}
					if(xx == s.x + 1) return hvalue + 1;
				}

				if(s.x > m_goal.x && (dir & 0x02) && !(dir & 0x04)){
					if(!isTemporalObstacle(State(s.x, s.y + 1))) return hvalue + 2;
					else return hvalue + 1;
				}

				if(s.x < m_goal.x && !(dir & 0x02) && !(dir & 0x04)){
					if(isTemporalObstacle(State(s.x + 1, s.y)) || isTemporalObstacle(State(s.x, s.y + 1))) return hvalue + 1;
					else return hvalue + 2;
				}
				if(s.x < m_goal.x && (dir & 0x02) && !(dir & 0x04)){
					return hvalue;
					int xx;
					for(xx = s.x; xx <= m_goal.x; xx++){
						if(isTemporalObstacle(State(xx, s.y + 1)) || isObstacle(State(xx, s.y + 1))) return hvalue;
					}
					if(xx == m_goal.x + 1) return hvalue + 1;
				}
				if(s.x < m_goal.x && (dir & 0x01) && !(dir & 0x04)){
					if(!isTemporalObstacle(State(s.x, s.y + 1))) return hvalue + 2;
					else return hvalue + 1;
				}
			}

			if(s.y == m_goal.y){
				if(s.x > m_goal.x && !(dir & 0x01)) {
					if(isTemporalObstacle(State(s.x - 1, s.y))) return hvalue + 1;
					else return hvalue + 2;
				}
				if(s.x < m_goal.x && !(dir & 0x02)) {
					if(isTemporalObstacle(State(s.x + 1, s.y))) return hvalue + 1;
					else return hvalue + 2;
				}
			}

			if(s.y > m_goal.y ){
				if(s.x == m_goal.x && !(dir & 0x08)){
					if(isTemporalObstacle(State(s.x, s.y - 1)))  return hvalue+ 1;
					else return hvalue + 2;
//					if(s.x + 1 < m_dimx && isObstacle(State(s.x + 1, s.y - 1))) return hvalue + 4;
//					if(s.x + 1 < m_dimx && isTemporalObstacle(State(s.x + 1, s.y - 1))) return hvalue + 1;
				}
				if(s.x > m_goal.x && !(dir & 0x01) && !(dir & 0x08)){
					if(isTemporalObstacle(State(s.x - 1, s.y)) || isTemporalObstacle(State(s.x, s.y - 1))) return hvalue + 1;
					else return hvalue + 2;
				}

				if(s.x > m_goal.x && (dir & 0x01) && !(dir & 0x08)){
					return hvalue;
					int xx;
					for(xx = m_goal.x; xx <= s.x; xx++){
						if(isTemporalObstacle(State(xx, s.y - 1)) || isObstacle(State(xx, s.y - 1))) return hvalue;
					}
					if(xx == s.x + 1) return hvalue + 1;
				}

				if(s.x > m_goal.x && (dir & 0x02) && !(dir & 0x08)){
					if(!isTemporalObstacle(State(s.x, s.y - 1))) return hvalue + 2;
					else return hvalue + 1;
				}

				if(s.x < m_goal.x && !(dir & 0x02) && !(dir & 0x08)){
					if(isTemporalObstacle(State(s.x + 1, s.y)) || isTemporalObstacle(State(s.x, s.y - 1))) return hvalue + 1;
					else return hvalue + 2;
				}
				if(s.x < m_goal.x && (dir & 0x02) && !(dir & 0x08)){
					return hvalue;
					int xx;
					for(xx = s.x; xx <= m_goal.x; xx++){
						if(isTemporalObstacle(State(xx, s.y - 1)) || isObstacle(State(xx, s.y - 1))) return hvalue;
					}
					if(xx == m_goal.x + 1) return hvalue + 1;
				}
				if(s.x < m_goal.x && (dir & 0x01) && !(dir & 0x08)){
					if(!isTemporalObstacle(State(s.x, s.y - 1))) return hvalue + 2;
					else return hvalue + 1;
				}

			}
			return hvalue;
		}
	}

	float admissibleHeuristic(const State& s, unsigned int dir, int g_cost) {

		if(!stateValid(s)) return INT_MAX;
		if(isExact){
			if(m_eHeuristic[m_goal][s.x][s.y] == -1) return INT_MAX;
			else{
				return m_eHeuristic[m_goal][s.x][s.y];
			}
		} else {
			float hvalue = std::abs(s.x - m_goal.x) + std::abs(s.y - m_goal.y);
			if(s.y < m_goal.y ){
				if(s.x == m_goal.x && !(dir & 0x04)){
					if(isTemporalObstacleAfterT(State(s.x, s.y + 1), g_cost))  return hvalue + 1;
//					else return hvalue + 2;
					if(s.x - 1 >= 0 && isObstacle(State(s.x - 1, s.y + 1)) && !(dir & 0x02)) return hvalue + 4;
					if(s.x + 1 < m_dimx && isObstacle(State(s.x + 1, s.y + 1)) && !(dir & 0x01)) return hvalue + 4;
					return hvalue + 2;
//					if(s.x - 1 >= 0 && isTemporalObstacleAfterT(State(s.x - 1, s.y + 1), g_cost + 1)) return hvalue + 2;
//					if(s.x + 1 < m_dimx && isTemporalObstacleAfterT(State(s.x + 1, s.y + 1), g_cost + 1)) return hvalue + 2;
				}

				if(s.x > m_goal.x && !(dir & 0x01) && !(dir & 0x04)){
					if(isTemporalObstacleAfterT(State(s.x - 1, s.y), g_cost) || isTemporalObstacleAfterT(State(s.x, s.y + 1), g_cost)) return hvalue + 1;
					else return hvalue + 2;
				}
				if(s.x > m_goal.x && (dir & 0x01) && !(dir & 0x04)){
//					if(s.x == m_goal.x + 1 && isObstacle(State(s.x - 1, s.y + 1))) return hvalue + 2;
					return hvalue;
					int xx;
					for(xx = m_goal.x; xx <= s.x; xx++){
						if(isTemporalObstacleAfterT(State(xx, s.y + 1), g_cost) || isObstacle(State(xx, s.y+1))) return hvalue;
					}
					if(xx == s.x + 1) return hvalue + 1;
				}
				if(s.x > m_goal.x && (dir & 0x02) && !(dir & 0x04)){
					if(!isTemporalObstacleAfterT(State(s.x, s.y + 1), g_cost)) return hvalue + 2;
					else return hvalue + 1;
				}

				if(s.x < m_goal.x && !(dir & 0x02) && !(dir & 0x04)){
					if(isTemporalObstacleAfterT(State(s.x + 1, s.y), g_cost) || isTemporalObstacleAfterT(State(s.x, s.y + 1), g_cost)) return hvalue + 1;
					else return hvalue + 2;
				}
				if(s.x < m_goal.x && (dir & 0x02) && !(dir & 0x04)){
//					if(s.x == m_goal.x - 1 && isObstacle(State(s.x + 1, s.y + 1))) return hvalue + 2;
					return hvalue;
					int xx;
					for(xx = s.x; xx <= m_goal.x; xx++){
						if(isTemporalObstacleAfterT(State(xx, s.y + 1), g_cost) || isObstacle(State(xx, s.y + 1))) return hvalue;
					}
					if(xx == m_goal.x + 1) return hvalue + 1;
				}
				if(s.x < m_goal.x && (dir & 0x01) && !(dir & 0x04)){
					if(!isTemporalObstacleAfterT(State(s.x, s.y + 1), g_cost)) return hvalue + 2;
					else return hvalue + 1;
				}
			}

			if(s.y == m_goal.y){
				if(s.x > m_goal.x && !(dir & 0x01)) {
					if(isTemporalObstacleAfterT(State(s.x - 1, s.y), g_cost)) return hvalue + 1;
					else return hvalue + 2;
				}
				if(s.x < m_goal.x && !(dir & 0x02)) {
					if(isTemporalObstacleAfterT(State(s.x + 1, s.y), g_cost)) return hvalue + 1;
					else return hvalue + 2;
				}
			}

			if(s.y > m_goal.y ){
				if(s.x == m_goal.x && !(dir & 0x08)){
					if(isTemporalObstacleAfterT(State(s.x, s.y - 1), g_cost))  return hvalue + 1;
//					else return hvalue + 2;
					if(s.x + 1 < m_dimx && isObstacle(State(s.x + 1, s.y - 1)) && !(dir & 0x01)) return hvalue + 4;
					if(s.x - 1 >= 0 && isObstacle(State(s.x - 1, s.y - 1)) && !(dir & 0x02)) return hvalue + 4;
					return hvalue + 2;
				}
				if(s.x > m_goal.x && !(dir & 0x01) && !(dir & 0x08)){
					if(isTemporalObstacleAfterT(State(s.x - 1, s.y), g_cost) || isTemporalObstacleAfterT(State(s.x, s.y - 1), g_cost)) return hvalue + 1;
					else return hvalue + 2;
				}

				if(s.x > m_goal.x && (dir & 0x01) && !(dir & 0x08)){
//					if(s.x == m_goal.x + 1 && isObstacle(State(s.x - 1, s.y - 1))) return hvalue + 2;
					return hvalue;
				}

				if(s.x > m_goal.x && (dir & 0x02) && !(dir & 0x08)){
					if(!isTemporalObstacleAfterT(State(s.x, s.y - 1), g_cost)) return hvalue + 2;
					else return hvalue + 1;
				}

				if(s.x < m_goal.x && !(dir & 0x02) && !(dir & 0x08)){
					if(isTemporalObstacleAfterT(State(s.x + 1, s.y), g_cost) || isTemporalObstacleAfterT(State(s.x, s.y - 1), g_cost)) return hvalue + 1;
					else return hvalue + 2;
				}
				if(s.x < m_goal.x && (dir & 0x02) && !(dir & 0x08)){
//					if(s.x == m_goal.x -1 && isObstacle(State(s.x + 1, s.y - 1))) return hvalue + 2;
					return hvalue;
				}
				if(s.x < m_goal.x && (dir & 0x01) && !(dir & 0x08)){
					if(!isTemporalObstacleAfterT(State(s.x, s.y - 1), g_cost)) return hvalue + 2;
					else return hvalue + 1;
				}
			}
			return hvalue;
		}
	}*/

	bool isSolution(const State &s) { return s == m_goal; }

	uint32_t getGoalId()
	{
		return goalID;
		return getNodeId(m_goal);
	}

	int getNodeId(const State &s)
	{
		return (s.y * m_dimx + s.x);
	}

	bool isSameXY(const State &s) { return (s.x == m_goal.x || s.y == m_goal.y); }
	//	bool isSameXY(const Location& s) {return s == m_goal;}

	State getLocation(const State &s) { return s; }
	int getIndex(const State &s)
	{
		return (s.x * m_dimy + s.y);
	}

	void getNeighbors(const State &s,
					  std::vector<Neighbor<State, Action, int>> &neighbors)
	{
		neighbors.clear();

		State left(s.x - 1, s.y);
		if (stateValid(left))
		{
			neighbors.emplace_back(
				Neighbor<State, Action, int>(left, Action::Left, 1));
		}
		State right(s.x + 1, s.y);
		if (stateValid(right))
		{
			neighbors.emplace_back(
				Neighbor<State, Action, int>(right, Action::Right, 1));
		}

		State up(s.x, s.y + 1);
		if (stateValid(up))
		{
			neighbors.emplace_back(Neighbor<State, Action, int>(up, Action::Up, 1));
		}
		State down(s.x, s.y - 1);
		if (stateValid(down))
		{
			neighbors.emplace_back(
				Neighbor<State, Action, int>(down, Action::Down, 1));
		}
	}

	void onExpandNode(const State & /*s*/, int /*fScore*/, int /*gScore*/)
	{
		// std::cout << "expand: " << s << "g: " << gScore << std::endl;
	}

	void onDiscover(const State & /*s*/, int /*fScore*/, int /*gScore*/)
	{
		// std::cout << "  discover: " << s << std::endl;
	}

	bool isCommandValid(
		const State & /*s1*/, const State & /*s2*/, const Action & /*a*/,
		int earliestStartTime,	   // can start motion at this time
		int /*latestStartTime*/,   // must have left s by this time
		int earliestArrivalTime,   // can only arrive at (s+cmd)
		int /*latestArrivalTime*/, // needs to arrive by this time at (s+cmd)
		int &t, const int &cost_c)
	{
		t = std::max<int>(earliestArrivalTime, earliestStartTime + cost_c);

		// TODO(whoenig): need to check for swaps here...
		// return t - 1 <= latestStartTime;
		return true;
	}

	bool isCommandValid(
		const State & /*s1*/, const State & /*s2*/, const Action & /*a*/,
		int earliestStartTime,	   // can start motion at this time
		int /*latestStartTime*/,   // must have left s by this time
		int earliestArrivalTime,   // can only arrive at (s+cmd)
		int /*latestArrivalTime*/, // needs to arrive by this time at (s+cmd)
		int &t)
	{
		t = std::max<int>(earliestArrivalTime, earliestStartTime + 1);

		// TODO(whoenig): need to check for swaps here...
		// return t - 1 <= latestStartTime;
		return true;
	}

public:
	bool stateValid(const State &s)
	{
		return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
			   !m_obstacles[s.x][s.y];
	}
	bool isBorder(const State &s)
	{
		return s.x == 0 || s.x == m_dimx - 1 || s.y == 0 || s.y == m_dimy - 1;
	}
	bool isObstacle(const State &s)
	{
		return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
			   m_obstacles[s.x][s.y];
	}

	bool isTemporalObstacle(const State &s)
	{
		return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
			   m_temporal_obstacle[s.x][s.y];
	}

	bool isTemporalEdgeConstraint(const State &s)
	{
		return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
			   m_temporal_obstacle[s.x][s.y];
	}

	bool isTemporalEdgeConstraintAfterT(const State &s, int T)
	{
		return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
			   m_temporal_obstacle[s.x][s.y] >= T;
	}

	bool isTemporalObstacleAfterT(const State &s, int T)
	{
		return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy && last_ob_g[s.x][s.y] >= T;
	}

	void setTemporalObstacle(const State &s)
	{
		m_temporal_obstacle[s.x][s.y] = true;
	}

	bool isJumpPoint(const State &s)
	{
		return jump_point_map[s.x][s.y];
	}

	bool isJumpPoint(const State &s, int time)
	{
		return jump_point_map[s.x][s.y] || nei_ob_g[s.x][s.y] >= time;
	}

	void setJumpPoint(const State &s)
	{
		jump_point_map[s.x][s.y] = true;
	}

	void setJPS()
	{
		is_jps = true;
	}
	void setNoJPS()
	{
		is_jps = false;
	}
	bool isJPS()
	{
		return is_jps;
	}

	bool isFCheck()
	{
		return isFI;
	}

	void Reset()
	{
		num_generation = 0;
		num_expansion = 0;
	}
	void setExactHeuristTrue()
	{
		isExact = true;
	}

	void setExactHeuristFalse()
	{
		isExact = false;
	}
	void setJumpLimit(int jumpStep)
	{
		limit_jump = jumpStep;
	}
	void setFI(bool isF)
	{
		isFI = isF;
	}

	void setGoal(State goal){
		m_goal = goal;
	}

	int getDimX() { return m_dimx; }
	int getDimY() { return m_dimy; }

public:
	int num_generation = 0;
	int num_expansion = 0;
	int limit_jump = 8;

private:
	int m_dimx;
	int m_dimy;
	std::vector<std::vector<bool>> m_obstacles;
	std::vector<std::vector<bool>> m_temporal_obstacle;
	std::vector<std::vector<bool>> jump_point_map;
	std::vector<std::vector<int>> last_ob_g;
	std::vector<std::vector<int>> nei_ob_g;
	std::unordered_map<State, std::vector<std::vector<int>>> m_eHeuristic;
	State m_goal;
	uint32_t goalID;
	bool is_limit = false;
	bool is_jps = true;
	bool isExact = false;
	bool isFI = true;
	

public:
	jpst_gridmap *jpst_gm_;
	bool isDebug = false;
	bool isCAT = false;
};



void getExactHeuristic(std::vector<std::vector<int>> &eHeuristic, std::vector<std::vector<bool>> map_obstacle, State goal, int dimx, int dimy)
{
	int xx[5] = {0, 0, -1, 1};
	int yy[5] = {1, -1, 0, 0};

	eHeuristic[goal.x][goal.y] = 0;
	std::queue<State> que;
	que.push(goal);

	while (true)
	{
		int queSize = que.size();
		if (queSize == 0)
			break;
		for (int i = 0; i < queSize; i++)
		{
			State curr = que.front();
			int currValue = eHeuristic[curr.x][curr.y];
			que.pop();
			for (int ii = 0; ii < 4; ii++)
			{
				State nei(curr.x + xx[ii], curr.y + yy[ii]);
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

int main(int argc, char *argv[])
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	std::string inputFile;
	std::string outputFile;
	std::string res;
	//	std::string mapfile;
	int num_path = 50; //goals.size();
	int jumpLimit = 8;
	int jumpLimit2 = 256;
	bool isF = true;
	desc.add_options()("help", "produce help message")(
		"input,i", po::value<std::string>(&inputFile)->required(),
		"input file (YAML)")("output,o",
							 po::value<std::string>(&outputFile)->required(),
							 "output file (YAML)")
		//						   ("map,m", po::value<std::string>(&mapfile)->required(), "map file (MAP)")
		("results,r", po::value<std::string>(&res)->required(), "results file (TXT)")
		("path num,N", po::value<int>(&num_path)->required(), "num path (int)")
		("jump limit,J", po::value<int>(&jumpLimit)->required(), "jump limit (int)")
		("jump limit2,L", po::value<int>(&jumpLimit2)->required(), "jump limit (int)")
		("is F inscreasing,F", po::value<bool>(&isF)->required(), "is F inscreasing")
		// ("url",
		// po::value<std::string>(&url)->default_value("http://0.0.0.0:8080"),
		// "server URL")
		;
	// jumpLimit2 = jumpLimit;
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

	std::fstream res_sta(res, std::ios::app);
	// std::fstream res_Good(res + "_good", std::ios::app);
	std::fstream res_Constraint("constraint.txt", std::ios::app);

	// Configure SIPP based on config file
	YAML::Node config = YAML::LoadFile(inputFile);

	std::unordered_set<State> obstacles;

	std::vector<State> goals;
	std::vector<State> startStates;
	std::vector<std::vector<bool>> map_jump_point;
	std::vector<std::vector<bool>> map_obstacle;
	std::vector<std::vector<bool>> map_temporal_obstacle;
	std::vector<std::vector<int>> last_ob_g;
	std::vector<std::vector<int>> nei_ob_g;

	const auto &dim = config["map"]["dimensions"];
	int dimx = dim[0].as<int>();
	int dimy = dim[1].as<int>();

	//	const char *mapfile_ = mapfile.c_str();

	map_jump_point.resize(dimx + 2);
	map_obstacle.resize(dimx + 2);
	map_temporal_obstacle.resize(dimx + 2);
	last_ob_g.resize(dimx + 2);
	nei_ob_g.resize(dimx + 2);
	for (size_t i = 0; i < map_jump_point.size(); i++)
	{
		map_jump_point[i].resize(dimy + 2);
		map_obstacle[i].resize(dimy + 2);
		map_temporal_obstacle[i].resize(dimy + 2);
		last_ob_g[i].resize(dimy + 2);
		nei_ob_g[i].resize(dimy + 2);
	}

	std::cout << "dimy dimx " << dimy << " " << dimx;
	gridmap gm(dimy, dimx);
	for (const auto &node : config["map"]["obstacles"])
	{
		obstacles.insert(State(node[0].as<int>(), node[1].as<int>()));
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

	for (const auto &ob : obstacles)
	{
		// if(ob.x >=666 && ob.x <= 671 && ob.y >= 84 && ob.y <= 88) std::cout << " Obst " << ob.x << " " << ob.y << " \n";
		State temp1 = ob, temp2 = ob;
		temp1.x = ob.x + 1;
		temp2.y = ob.y + 1;

		if (temp1.x >= 0 && temp1.x <= dimx && temp2.y >= 0 && temp2.y <= dimy && obstacles.find(temp1) == obstacles.end() && obstacles.find(temp2) == obstacles.end())
		{
			map_jump_point[ob.x + 1][ob.y + 1] = true;
		}

		temp1 = ob;
		temp2 = ob;
		temp1.x = ob.x + 1;
		temp2.y = ob.y - 1;
		if (temp1.x >= 0 && temp1.x <= dimx && temp2.y >= 0 && temp2.y <= dimy && obstacles.find(temp1) == obstacles.end() && obstacles.find(temp2) == obstacles.end())
		{
			map_jump_point[ob.x + 1][ob.y - 1] = true;
		}

		temp1 = ob;
		temp2 = ob;
		temp1.x = ob.x - 1;
		temp2.y = ob.y + 1;
		if (temp1.x >= 0 && temp1.x <= dimx && temp2.y >= 0 && temp2.y <= dimy && obstacles.find(temp1) == obstacles.end() && obstacles.find(temp2) == obstacles.end())
		{
			map_jump_point[ob.x - 1][ob.y + 1] = true;
		}

		temp1 = ob;
		temp2 = ob;
		temp1.x = ob.x - 1;
		temp2.y = ob.y - 1;
		if (temp1.x >= 0 && temp1.x <= dimx && temp2.y >= 0 && temp2.y <= dimy && obstacles.find(temp1) == obstacles.end() && obstacles.find(temp2) == obstacles.end())
		{
			map_jump_point[ob.x - 1][ob.y - 1] = true;
		}
	}

	for (const auto &node : config["agents"])
	{
		const auto &start = node["start"];
		const auto &goal = node["goal"];
		startStates.emplace_back(State(start[0].as<int>(), start[1].as<int>()));
		goals.emplace_back(State(goal[0].as<int>(), goal[1].as<int>()));
	}
	typedef JPSSIPP_BIT<State, State, Action, int, Environment> jps_sipp;
	typedef JPSSIPP<State, State, Action, int, Environment> jpst_old1;
	typedef JPSSIPPNEWGEN<State, State, Action, int, Environment> jpst_old2;

	typedef SIPP<State, State, Action, int, Environment> sipp_t;

	std::ofstream out(outputFile);
	out << "schedule:" << std::endl;

	// Plan (sequentially)
	std::map<State, std::vector<jps_sipp::interval>> allCollisionIntervals;
	std::map<State, std::vector<jps_sipp::edgeCollision>> allEdgeCollisions;

	std::map<State, std::vector<jpst_old1::interval>> allCollisionIntervals_jpstold1;
	std::map<State, std::vector<jpst_old1::edgeCollision>> allEdgeCollisions_jpstold1;

	std::map<State, std::vector<jpst_old2::interval>> allCollisionIntervals_jpstold2;
	std::map<State, std::vector<jpst_old2::edgeCollision>> allEdgeCollisions_jpstold2;


	std::map<State, std::vector<sipp_t::interval>> allCollisionIntervals_sipp;
	std::map<State, std::vector<sipp_t::edgeCollision>> allEdgeCollisions_sipp;


    if (num_path == -1)
		num_path = goals.size();
	long cost = 0;
	int num_temporal_obstacle = 0;
	Timer t;

	struct rusage r_usage;
	getrusage(RUSAGE_SELF, &r_usage);

	std::unordered_map<State, std::vector<std::vector<int>>> eHeuristic;
	// Environment env(dimx, dimy, map_obstacle, map_temporal_obstacle, map_jump_point, last_ob_g, nei_ob_g, eHeuristic, goals[0], &jpst_gm_);

	for (int i = 0; i < (int)goals.size(); ++i)
	{
		// if(i > 1) break;
		getrusage(RUSAGE_SELF, &r_usage);
		std::cout << "Memory usage: " << r_usage.ru_maxrss << " kilobytes\n";

		std::cout << "Planning for agent " << i << std::endl;
		out << "  agent" << i << ":" << std::endl;
		t.reset();

		std::unordered_map<State, std::vector<std::vector<int>>> eHeuristic;
		std::vector<std::vector<int>> eHeuristicGoal(dimx + 1, std::vector<int>(dimy + 1, -1));
		getExactHeuristic(eHeuristicGoal, map_obstacle, goals[i], dimx, dimy);
		eHeuristic[goals[i]] = eHeuristicGoal;
		t.stop();
		double preTime = t.elapsedSeconds();

		Environment env(dimx, dimy, map_obstacle, map_temporal_obstacle, map_jump_point, last_ob_g, nei_ob_g, eHeuristic, goals[i], &jpst_gm_);
		env.setExactHeuristTrue();
		env.setJumpLimit(jumpLimit2);
		env.setFI(isF);
		jps_sipp jpssipp(env);
		jpst_old1 jpstold1(env);
		jpst_old2 jpstold2(env);
		std::vector<PlanResult<State, Action, int> > cat;
		sipp_t sipp(env, cat);

		jpssipp.setEdgeCollisionSize(dimx, dimy);
		jpstold1.setEdgeCollisionSize(dimx, dimy);
		jpstold2.setEdgeCollisionSize(dimx, dimy);
		sipp.setEdgeCollisionSize(dimx, dimy);
		for (const auto &collisionIntervals : allCollisionIntervals)
		{
			jpssipp.setCollisionIntervals(collisionIntervals.first, collisionIntervals.second);
		}

		for (const auto &ec : allEdgeCollisions)
		{
			jpssipp.setEdgeCollisions(ec.first, ec.second);
		}

		for (const auto &collisionIntervals : allCollisionIntervals_jpstold1)
		{
			jpstold1.setCollisionIntervals(collisionIntervals.first, collisionIntervals.second);
		}

		for (const auto &ec : allEdgeCollisions_jpstold1)
		{
			jpstold1.setEdgeCollisions(ec.first, ec.second);
		}

		for (const auto &collisionIntervals : allCollisionIntervals_jpstold2)
		{
			jpstold2.setCollisionIntervals(collisionIntervals.first, collisionIntervals.second);
		}

		for (const auto &ec : allEdgeCollisions_jpstold2)
		{
			jpstold2.setEdgeCollisions(ec.first, ec.second);
		}


		for (const auto &collisionIntervals_sipp : allCollisionIntervals_sipp)
		{
			sipp.setCollisionIntervals(collisionIntervals_sipp.first, collisionIntervals_sipp.second);
		}
		for (const auto &ec : allEdgeCollisions_sipp)
		{
			sipp.setEdgeCollisions(ec.first, ec.second);
		}

		// Plan
		env.Reset();
		PlanResult<State, Action, int> solution;

		t.reset();
		bool success = jpssipp.search(startStates[i], Action::Wait, solution);
		t.stop();
		std::cout << t.elapsedSeconds() << std::endl;
		int num_expansion1 = env.num_expansion;
		int num_generation1 = env.num_generation;
		double time1 = t.elapsedSeconds();
		if (success)
		{
			std::cout << "JPS Planning successful! Total cost: " << solution.cost << " Expansion:"
					  << env.num_expansion << " Generation: " << env.num_generation
					  << std::endl;

			// print solution
			for (size_t i = 0; i < solution.actions.size(); ++i)
			{
				std::cout << solution.states[i].second << ": " << solution.states[i].first
						  << "->" << solution.actions[i].first
						  << "(cost: " << solution.actions[i].second << ")" << std::endl;
			}
			std::cout << solution.states.back().second << ": "
					  << solution.states.back().first << std::endl;

			for (size_t i = 0; i < solution.states.size(); ++i)
			{
				out << "    - x: " << solution.states[i].first.x << std::endl
					<< "      y: " << solution.states[i].first.y << std::endl
					<< "      t: " << solution.states[i].second << std::endl;
			}
		}
		else
		{
			std::cout << "Planning NOT successful!" << std::endl;
			out << "    []" << std::endl;
		}

		env.setJumpLimit(jumpLimit);
		env.Reset();
		PlanResult<State, Action, int> solutionJpstOld;
		t.reset();
		// env.setJumpLimit(32);
		success = jpstold1.search(startStates[i], Action::Wait, solutionJpstOld);
		t.stop();
		std::cout << t.elapsedSeconds() << std::endl;
		int num_expansionJpstOld = env.num_expansion;
		int num_generationJpstOld = env.num_generation;
		double timeJpstOld = t.elapsedSeconds();

		env.Reset();
		PlanResult<State, Action, int> solutionJpstOld2;
		t.reset();
		// env.setJumpLimit(32);
		success = jpstold2.search(startStates[i], Action::Wait, solutionJpstOld2);
		t.stop();
		std::cout << t.elapsedSeconds() << std::endl;
		int num_expansionJpstOld2 = env.num_expansion;
		int num_generationJpstOld2 = env.num_generation;
		double timeJpstOld2 = t.elapsedSeconds();


		if(success){
			std::cout << "JPST-old path \n";
			// print solution
			for (size_t i = 0; i < solutionJpstOld.actions.size(); ++i)
			{
				std::cout << solutionJpstOld.states[i].second << ": " << solutionJpstOld.states[i].first
						  << "->" << solutionJpstOld.actions[i].first
						  << "(cost: " << solutionJpstOld.actions[i].second << ")" << std::endl;
			}
			std::cout << solutionJpstOld.states.back().second << ": "
					  << solutionJpstOld.states.back().first << std::endl;

		}

		getrusage(RUSAGE_SELF, &r_usage);
		std::cout << "Memory usage: " << r_usage.ru_maxrss << " kilobytes\n";


		// env.setJumpLimit(jumpLimit);
		env.Reset();
		env.setNoJPS();
		PlanResult<State, Action, int> solution2;
		t.reset();
		bool success_temp = sipp.search(startStates[i], Action::Wait, solution2);
		t.stop();

		int num_expansion2 = env.num_expansion;
		int num_generation2 = env.num_generation;

		double time2 = t.elapsedSeconds();
		std::cout << t.elapsedSeconds() << std::endl;

/*		env.Reset();
		env.setNoJPS();
		env.setExactHeuristTrue();
		PlanResult<State, Action, int> solution2;
		t.reset();
		sipp.search(startStates[i], Action::Wait, solution2);
		t.stop();
*/
		if (success_temp)
		{
			std::cout << "NoJPS Planning successful! Total cost: " << solution2.cost << " Expansion:"
					  << env.num_expansion << " Generation: " << env.num_generation
					  << std::endl;

			// update collision intervals
			auto lastState = solution2.states[0];
			int xx[5] = {0, 1, -1};
			int yy[5] = {0, 1, -1};
			if (i < num_path)
			{	//goals.size()
				//    	  std::cout << "VC " << i << std::endl;
				//    	  res_Constraint << " -1 -1 -1"<< std::endl;
				for (size_t i = 1; i < solution2.states.size(); ++i)
				{
					if (solution2.states[i].first != lastState.first)
					{
						for (int timeStart = lastState.second; timeStart <= solution2.states[i].second - 1; timeStart++)
						{
							if (timeStart == 0)
								continue;
						}	

						jpst_gm_.add_obstacle((uint32_t)lastState.first.x, (uint32_t)lastState.first.y);

						allCollisionIntervals[lastState.first].push_back(
							jps_sipp::interval(lastState.second, solution2.states[i].second - 1));
						allCollisionIntervals_jpstold1[lastState.first].push_back(
							jpst_old1::interval(lastState.second, solution2.states[i].second - 1));
						allCollisionIntervals_jpstold2[lastState.first].push_back(
							jpst_old2::interval(lastState.second, solution2.states[i].second - 1));

						allCollisionIntervals_sipp[lastState.first].push_back(
							sipp_t::interval(lastState.second, solution2.states[i].second - 1));
						int tt = std::max(last_ob_g[lastState.first.x][lastState.first.y],
										  solution2.states[i].second - 1);
						last_ob_g[lastState.first.x][lastState.first.y] = tt;
						for (int xx_1 = 0; xx_1 < 3; xx_1++)
						{
							for (int yy_1 = 0; yy_1 < 3; yy_1++)
							{
								if (xx_1 == 0 && yy_1 == 0)
									continue;
								State temp_state(lastState.first.x + xx[xx_1], lastState.first.y + yy[yy_1]);
								if (env.stateValid(temp_state))
								{
									nei_ob_g[temp_state.x][temp_state.y] = std::max(nei_ob_g[temp_state.x][temp_state.y], tt);
								}
							}
						}

						if (!map_temporal_obstacle[lastState.first.x][lastState.first.y])
						{
							map_temporal_obstacle[lastState.first.x][lastState.first.y] = true;
							num_temporal_obstacle++;
						}
						lastState = solution2.states[i];
					}
				}
				allCollisionIntervals[solution2.states.back().first].push_back(
					jps_sipp::interval(solution2.states.back().second, std::numeric_limits<int>::max()));
				allCollisionIntervals_jpstold1[solution2.states.back().first].push_back(
					jpst_old1::interval(solution2.states.back().second, std::numeric_limits<int>::max()));
				allCollisionIntervals_jpstold2[solution2.states.back().first].push_back(
					jpst_old2::interval(solution2.states.back().second, std::numeric_limits<int>::max()));

				allCollisionIntervals_sipp[solution2.states.back().first].push_back(
					sipp_t::interval(solution2.states.back().second, std::numeric_limits<int>::max()));
				
				jpst_gm_.add_obstacle((uint32_t)solution2.states.back().first.x, (uint32_t)solution2.states.back().first.y);
				
				//    	  std::cout << lastState.first.x << " " << lastState.first.y << " " << std::numeric_limits<int>::max() << " \n";
				//    	  res_Constraint << lastState.first.x << " " << lastState.first.y << " " << std::numeric_limits<int>::max() << " \n";
				//    	  res_Constraint << "-1 -1 -1" << std::endl;
				for (int xx_1 = 0; xx_1 < 3; xx_1++)
				{
					for (int yy_1 = 0; yy_1 < 3; yy_1++)
					{
						if (xx_1 == 0 && yy_1 == 0)
							continue;
						State temp_state(lastState.first.x + xx[xx_1], lastState.first.y + yy[yy_1]);
						if (env.stateValid(temp_state))
						{
							nei_ob_g[temp_state.x][temp_state.y] = std::numeric_limits<int>::max();
						}
					}
				}
				if (!map_temporal_obstacle[lastState.first.x][lastState.first.y])
				{
					map_temporal_obstacle[lastState.first.x][lastState.first.y] = true;
					num_temporal_obstacle++;
				}
			}

			// update statistics
			cost += solution2.cost;
			// print solution
			if (i < num_path)
			{	//goals.size()
				//    	  std::cout << "EC " << i << std::endl;
				//    	  res_Constraint << "-1 -1 -1 -1 -1 " << std::endl;
				for (size_t i = 0; i < solution2.actions.size(); ++i)
				{
					if (solution2.actions[i].first != Action::Wait)
					{
						allEdgeCollisions[solution2.states[i].first].push_back(jps_sipp::edgeCollision(solution2.states[i].second, solution2.actions[i].first));
						allEdgeCollisions_jpstold1[solution2.states[i].first].push_back(jpst_old1::edgeCollision(solution2.states[i].second, solution2.actions[i].first));
						allEdgeCollisions_jpstold2[solution2.states[i].first].push_back(jpst_old2::edgeCollision(solution2.states[i].second, solution2.actions[i].first));

						assert(i + 1 <= solution2.states.size());
						//   			  std::cout << solution2.states[i].second << " " << solution2.states[i].first.x << " " <<  solution2.states[i].first.y <<
						//   					  " " << solution2.states[i+1].first.x << " " << solution2.states[i+1].first.y << std::endl;
						//    			  res_Constraint << solution2.states[i].second << " " << solution2.states[i].first.x << " " <<  solution2.states[i].first.y <<
						//    					  " " << solution2.states[i+1].first.x << " " << solution2.states[i+1].first.y << std::endl;
					}
					if (solution2.actions[i].first != Action::Wait)
					{
						allEdgeCollisions_sipp[solution2.states[i].first].push_back(sipp_t::edgeCollision(solution2.states[i].second, solution2.actions[i].first));
					}
					std::cout << solution2.states[i].second << ": " << solution2.states[i].first
							  << "->" << solution2.actions[i].first
							  << "(cost: " << solution2.actions[i].second << ")" << std::endl;
				}
				//    	  res_Constraint << "-1 -1 -1 -1 -1" << std::endl;
			}
			else
			{
				for (size_t i = 0; i < solution2.actions.size(); ++i)
				{
					std::cout << solution2.states[i].second << ": " << solution2.states[i].first
							  << "->" << solution2.actions[i].first
							  << "(cost: " << solution2.actions[i].second << ")" << std::endl;
				}
				std::cout << solution2.states.back().second << ": "
						  << solution2.states.back().first << std::endl;
			}

			//   for (size_t i = 0; i < solution2.states.size(); ++i) {
			//     out << "    - x: " << solution2.states[i].first.x << std::endl
			//         << "      y: " << solution2.states[i].first.y << std::endl
			//         << "      t: " << solution2.states[i].second << std::endl;
			//   }
		}
		else
		{
			std::cout << inputFile << " SIPP Planning NOT successful from Agent: " << i << std::endl;
			//   out << "    []" << std::endl;
			res_sta << inputFile << " Agent " << i << " JPSSIPP: "
					<< " cost: "
					<< " -1 "
					<< " -1 "
					<< " -1 "
					<< " -1 "
					<< " -1 \n";

			res_sta << inputFile << " Agent " << i << " SIPP: "
					<< " cost: "
					<< " -1 "
					<< " -1 "
					<< " -1 "
					<< " -1 "
					<< " -1 \n";

			continue;
		}

		if (solution.cost != solution2.cost || solutionJpstOld.cost != solution2.cost || solutionJpstOld2.cost != solution2.cost)
		{
			std::cout << inputFile << "Agent " << i << ": Not equal1" << std::endl;
			res_sta << inputFile << " Agent, " << i << ", Not equal" << std::endl;
			// jpst_gm_.gm_->print(std::cout);
			// jpst_gm_.t_gm_->print(std::cout);
			break;
		}

		if (num_expansion1 > num_expansion2)
		{
			std::cout << inputFile << " Not Expansion-1 Agent " << i << " " << num_expansion1 << " " << num_expansion2 << " " << num_expansion1 - num_expansion2 << "\n";
		}

		if (num_generation1 > 5 * num_generation2)
		{
			std::cout << inputFile << " Not Generation-1 Agent " << i << " " << num_generation1 << " " << num_generation2 << " " << num_generation1 - num_generation2 << "\n";
		}

		std::cout << inputFile << " All-Generation Agent " << i << " " << num_generation1 << " " << num_generation2 << "\n";
		std::cout << inputFile << " All-Expansion Agent " << i << " " << num_expansion1 << " " << num_expansion2 << "\n";

		double speedup = time2 / time1;
		double speedupJpstOld = time2 / timeJpstOld;
		double speedupJpstOld2 = time2 / timeJpstOld2;
		res_sta << inputFile << " Agent, " << i << ", " << num_temporal_obstacle << ", " << preTime
				<< ", JPSTOld1 cost, " << solutionJpstOld.cost << ", " << timeJpstOld << ", " << num_expansionJpstOld << ", " << num_generationJpstOld
				<< ", JPSTOld2 cost, " << solutionJpstOld2.cost << ", " << timeJpstOld2 << ", " << num_expansionJpstOld2 << ", " << num_generationJpstOld2
				<< ", JPSTNew cost, " << solution.cost << ", " << time1 << ", " << num_expansion1 << ", " << num_generation1
				<< ", SIPP cost, " << solution2.cost << ", " << time2 << ", " << num_expansion2 << ", " << num_generation2 
				<< ", " << speedup << ", " << speedupJpstOld << ", " << speedupJpstOld2 << " \n";

		// if (time1 < time2)
		// {
		// 	res_Good << inputFile << " Agent " << i << " JPSSIPP: "
		// 			 << " cost " << solution.cost << " " << time1 << " " << num_expansion1 << " " << num_generation1 << "\n";
		// 	res_Good << inputFile << " Agent " << i << " SIPP: "
		// 			 << " cost " << solution.cost << " " << time2 << " " << num_expansion2 << " " << num_generation2 << "\n";
		// }
	}

	out << "statistics:" << std::endl;
	out << "  cost: " << cost << std::endl;

	return 0;
}
