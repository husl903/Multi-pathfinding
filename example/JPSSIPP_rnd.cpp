#include <fstream>
#include <iostream>
#include <queue>
#include <rnd/stonesngems.h>
#include <random>
#include <algorithm>

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
#include <libMultiRobotPlanning/JPSSIPP_BIT_rnd.hpp>
#include <libMultiRobotPlanning/JPSSIPP_BITCBS.hpp>
#include <libMultiRobotPlanning/JPSSIPPNEWGEN.hpp>
#include <libMultiRobotPlanning/JPSSIPPAN.hpp>
#include <libMultiRobotPlanning/sipp.hpp>
#include <libMultiRobotPlanning/gridmap.hpp>
#include <libMultiRobotPlanning/jpst_gridmap.hpp>
#include <libMultiRobotPlanning/timer.hpp>

#include <libMultiRobotPlanning/vectorCache.hpp>
#include <libMultiRobotPlanning/queueCache.hpp>

// #include <libMultiRobotPlanning/online_jump_point_locator2.hpp>

using namespace stonesngems;
using namespace stonesngems::util;

using libMultiRobotPlanning::gridmap;
using libMultiRobotPlanning::JPSSIPP;
using libMultiRobotPlanning::JPSSIPP_BIT;
using libMultiRobotPlanning::JPSSIPPNEWGEN;
using libMultiRobotPlanning::JPSSIPPAN;
using libMultiRobotPlanning::jpst_gridmap;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using libMultiRobotPlanning::SIPP;

GameParameters params = kDefaultGameParams;
RNDGameState state_game;
static vectorCache<int8_t> gridCache;
static vectorCache<int> indexCache;

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
	State(int x, int y, int time) : x(x), y(y), time(time) {}
	State(int x, int y) : x(x), y(y){}	
	// State(int x, int y, int time, std::vector<int8_t>&grid, std::vector<int>&need_update_index) : x(x), y(y), 
        // time(time), grid(grid), need_update_index(need_update_index) {}  

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
	int x;
	int y;
	int time;
	std::vector<int8_t> grid;
	std::vector<int> need_update_index;
	LocalState localstate;
	bool is_falling = false;
	bool canRollLeft = false;
	bool canRollRight = false;
	int index_gem = -1;
	int gem_x = -1;
	int gem_y = -1;
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
			// boost::hash_combine(seed, s.time)
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

	size_t getAgentId(){
	 	return -1;
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
	bool isDebug = true;
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

    int goalX, goalY;
    desc.add_options()("help", "produce help message")(
      "goalX,x", po::value<int>(&goalX)->required(), "goal position x-component")(
      "goalY,y", po::value<int>(&goalY)->required(), "goal position y-component");

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
		std::cerr << e.what() << std::endl;
		std::cerr << desc << std::endl;
		return 1;
	}
    std::cout << "Begin test \n";
    const std::string filename="./levels/bd_01_1.txt";
    std::ifstream infile(filename.c_str());
    assert(infile.is_open());
    std::string board_str;
    getline(infile,board_str);
    params["game_board_str"] = GameParameter(board_str);
    RNDGameState state_p(params);
    state_game = state_p;
    
    std::cout << state_p.board.rows << std::endl;
    std::vector<int8_t> grid;
    grid.swap(state_p.board.grid);

    std::cout << state_p.board.grid.size() << "end\n";
    int startX, startY;  
    for (int h = 0; h < state_p.board.rows; ++h) {
      for (int w = 0; w < state_p.board.cols; ++w) {
        if(grid[h * state_p.board.cols + w] == 0){
          startX = w;
          startY = h;
        }
          std::cout << kCellTypeToElement[grid[h * state_p.board.cols + w] + 1].id;
      }
      std::cout << std::endl;
    }

    for (int h = 0; h < state_p.board.rows; ++h) {
      for (int w = 0; w < state_p.board.cols; ++w) {
        //printf("%d ", grid[h * state_p.board.cols + w]);
        if(grid[h * state_p.board.cols + w] == 0){
          startX = w;
          startY = h;
          std::cout <<  "||" << h << ", " << w;
        }
        if(grid[h * state_p.board.cols + w] == 5){
          std::cout <<  "||" << h << ", " << w;
        }
        std::cout << kCellTypeToElement[grid[h * state_p.board.cols + w] + 1].id;
        // if(h==0) grid[h * state_p.board.cols + w] = 100;
      }
      std::cout << std::endl;
    }	
    std::vector<Location> goals_loc;
    goals_loc.push_back(Location(10, 1));
    goals_loc.push_back(Location(2, 22));
    goals_loc.push_back(Location(8, 9));
    goals_loc.push_back(Location(8, 27));
    goals_loc.push_back(Location(8, 30));
    goals_loc.push_back(Location(9, 3));
    // goals_loc.push_back(Location(16, 38));    
    goals_loc.push_back(Location(11, 32));
    goals_loc.push_back(Location(15, 16));
    goals_loc.push_back(Location(17, 28));
    goals_loc.push_back(Location(18, 6));
    goals_loc.push_back(Location(18, 28));
    goals_loc.push_back(Location(19, 28));
    goals_loc.push_back(Location(20, 2));
    goals_loc.push_back(Location(16, 38));


	std::unordered_set<State> obstacles;
	std::vector<State> goals;
	std::vector<State> startStates;
	std::vector<std::vector<bool>> map_jump_point;
	std::vector<std::vector<bool>> map_obstacle;
	std::vector<std::vector<bool>> map_temporal_obstacle;
	std::vector<std::vector<int>> last_ob_g;
	std::vector<std::vector<int>> nei_ob_g;

	gridmap gm(state_p.board.rows, state_p.board.cols);
	map_jump_point.resize(state_p.board.cols + 2);
	map_obstacle.resize(state_p.board.cols + 2);
	map_temporal_obstacle.resize(state_p.board.cols + 2);
	last_ob_g.resize(state_p.board.cols + 2);
	nei_ob_g.resize(state_p.board.cols + 2);

	for (size_t i = 0; i < map_jump_point.size(); i++)
	{
		map_jump_point[i].resize(state_p.board.rows + 2);
		map_obstacle[i].resize(state_p.board.rows + 2);
		map_temporal_obstacle[i].resize(state_p.board.rows + 2);
		last_ob_g[i].resize(state_p.board.rows + 2);
		nei_ob_g[i].resize(state_p.board.rows + 2);
	}
	std::cout << "Test jpst-sipp-2\n";
	int dimx, dimy;

    for (int h = 0; h < state_p.board.rows; ++h) {
      for (int w = 0; w < state_p.board.cols; ++w) {
        if(grid[h * state_p.board.cols + w] != 0 && 
		   grid[h * state_p.board.cols + w] != -1 &&
		   grid[h * state_p.board.cols + w] != 1 &&
		   grid[h * state_p.board.cols + w] != 2 &&
		   grid[h * state_p.board.cols + w] != 5 &&
		   grid[h * state_p.board.cols + w] != 6){
		   map_obstacle[w][h] = true;
		   gm.set_label(gm.to_padded_id(w, h), 0);
        }else{
			map_obstacle[w][h] = false;
			gm.set_label(gm.to_padded_id(w, h), 1);
		}
        // if(grid[h * state_p.board.cols + w] == 5){
        //   std::cout <<  "||" << h << ", " << w;
        // }
        std::cout << kCellTypeToElement[grid[h * state_p.board.cols + w] + 1].id;
      }
      std::cout << std::endl;
    }	
	jpst_gridmap jpst_gm_(&gm);

	// return 0;


	for (int h = 0; h < state_p.board.cols; ++h)
	{
		for(int w = 0; w < state_p.board.rows; ++w){
			if(map_obstacle[h][w]){
				if(h + 1 < state_p.board.cols && w - 1 >=0 && 
					!map_obstacle[h + 1][w] && !map_obstacle[h][w - 1] && !map_obstacle[h + 1][ w - 1]){
					map_jump_point[h + 1][ w - 1] = true;
				} 
				if(h + 1 < state_p.board.cols && w + 1 < state_p.board.rows && 
					!map_obstacle[h + 1][w] && !map_obstacle[h][w + 1] && !map_obstacle[h + 1][w + 1]){
					map_jump_point[h + 1][ w + 1] = true;
				} 
				if(h - 1 >= 0 && w + 1 < state_p.board.rows && 
					!map_obstacle[h - 1][w] && !map_obstacle[h][w + 1] && !map_obstacle[h - 1][w + 1]){
					map_jump_point[h - 1][ w + 1] = true;
				} 
				if(h - 1 >= 0 && w - 1 >= 0 && 
					!map_obstacle[h - 1][w] && !map_obstacle[h][w - 1] && !map_obstacle[h - 1][w - 1]){
					map_jump_point[h - 1][ w - 1] = true;
				} 
			}
		}
	}

	// for (const auto &node : config["agents"])
	// {
	// 	const auto &start = node["start"];
	// 	const auto &goal = node["goal"];
	// 	startStates.emplace_back(State(start[0].as<int>(), start[1].as<int>()));
	// 	goals.emplace_back(State(goal[0].as<int>(), goal[1].as<int>()));
	// }
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
	
	std::cout << " Test size " << std::endl;
	std::cout << goals.size() << std::endl;
	Timer total;
    std::vector <PlanResult<State, Action, int>>  solutions;
    LocalState localstate;
    int next_index = goals_loc[0].x * state_p.board.cols + goals_loc[0].y;
    int index_g = 0;
    bool is_falling = false;
	while(1){
		if(next_index == -1) break;
		if(index_g != 0){
			startX = solutions[index_g - 1].states[0].first.x;
        	startY = solutions[index_g - 1].states[0].first.y;
		}

		State start(startX, startY, 0);
		if(index_g == 0){
			start.grid.assign(grid.begin(), grid.end());
			start.need_update_index = state_p.board.need_update_index;
			start.localstate = localstate;
		}else{
			start.grid.assign(solutions[index_g - 1].states[0].first.grid.begin(), solutions[index_g - 1].states[0].first.grid.end());
	        start.need_update_index.assign(solutions[index_g - 1].states[0].first.need_update_index.begin(), solutions[index_g - 1].states[0].first.need_update_index.end());
    	    start.localstate = solutions[index_g - 1].states[0].first.localstate;
		}
	    goalX = next_index/state_p.board.cols;
    	goalY = next_index%state_p.board.cols;
		start.index_gem = next_index;
		start.gem_x = goalX;
		start.gem_y = goalY;
	    state_game.board.grid.assign(start.grid.begin(), start.grid.end());
    	start.is_falling = is_falling;
      	start.canRollLeft = state_game.CanRollLeft(next_index);
      	start.canRollRight = state_game.CanRollRight(next_index);

      	std::cout << next_index / state_p.board.cols << ", " << next_index % state_p.board.cols << std::endl;

		std::unordered_map<State, std::vector<std::vector<int>>> eHeuristic;
		std::vector<std::vector<int>> eHeuristicGoal(dimx + 1, std::vector<int>(dimy + 1, -1));
		// getExactHeuristic(eHeuristicGoal, map_obstacle, goals[i], dimx, dimy);
		// eHeuristic[goals[i]] = eHeuristicGoal;		
		Environment env(state_p.board.cols, state_p.board.rows, map_obstacle, map_temporal_obstacle, map_jump_point, last_ob_g, nei_ob_g, eHeuristic, State(goalX, goalY, 0), &jpst_gm_);
		env.setExactHeuristFalse();
		env.setJumpLimit(jumpLimit2);
		env.setFI(isF);
		jps_sipp jpssipp(env);
		PlanResult<State, Action, int>  solution;
		Timer timer;
	    bool success = jpssipp.search(start, Action::Wait, solution);
		timer.stop();
		solutions.push_back(solution);

      	if (success) {
        	std::cout << "Planning successful! Total cost: " << solution.cost << ", " << timer.elapsedSeconds() << ", " 
			// << env.num_expand
               	 << std::endl;
        	for (size_t i = 0; i < solution.actions.size(); ++i) {
          		std::cout << solution.states[i].second << ": " << solution.states[i].first
                  << "->" << solution.actions[i].first
                  << "(cost: " << solution.actions[i].second << ")" << std::endl;
        	}
        	std::cout << solution.states.back().second << ": "
                << solution.states.back().first << std::endl;

        	solutions.push_back(solution);
        	if(goalX * state_p.board.cols + goalY == 678)
        	{

          		for (int h = 0; h < state_p.board.rows; ++h) {
            		for (int w = 0; w < state_p.board.cols; ++w) {
              			std::cout << kCellTypeToElement[solutions[index_g].states[0].first.grid[h * state_p.board.cols + w] + 1].id;
              			// if(h==0) grid[h * state_p.board.cols + w] = 100;
            		}
            		std::cout << std::endl;
          		}
          		std::cout << static_cast<unsigned int>(solutions[index_g].states[0].first.localstate.gems_collected) << std::endl;
       		}
        	std::cout << static_cast<unsigned int>(solutions[index_g].states[0].first.localstate.gems_collected) << std::endl;
        	state_p.board.grid.assign(solutions[index_g].states[0].first.grid.begin(), solutions[index_g].states[0].first.grid.end());
        	if(goalX * state_p.board.cols + goalY == 678) break;
        	if(solutions[index_g].states[0].first.localstate.gems_collected >= state_p.board.gems_required){
          		next_index = 678;
         		index_g++;
          		continue; 
        	}
        
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
        std::cout << "Planning NOT successful!" << std::endl;
        break;
      }		
		next_index = -1;
		index_g++;
	}

	out << "statistics:" << std::endl;
	out << "  cost: " << cost << std::endl;

	return 0;
}
