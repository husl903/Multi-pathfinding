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
#include <libMultiRobotPlanning/timer.hpp>
#include <libMultiRobotPlanning/vectorCache.hpp>
#include <libMultiRobotPlanning/queueCache.hpp>

#define GemGoal

using namespace stonesngems;
using namespace stonesngems::util;

using libMultiRobotPlanning::AStar;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;

GameParameters params = kDefaultGameParams;
RNDGameState state_game;
static vectorCache<int8_t> gridCache;
static vectorCache<int> indexCache;
std::vector<size_t> rand_x;
std::vector<size_t> rand_y;
std::vector<int8_t> grid_temp;

int num_state = 0;

struct State {
  State(int x, int y, int time, std::vector<int8_t>&grid, std::vector<int>&need_update_index) : x(x), y(y), 
         time(time), grid(grid), need_update_index(need_update_index) {}  
  State(int x, int y, int time, uint64_t zorb_hash, std::vector<int8_t>&grid, std::vector<int>&need_update_index) : x(x), y(y), 
         time(time), grid(grid), zorb_hash(zorb_hash), need_update_index(need_update_index) {}           
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
  std::vector<int8_t> &grid;
  std::vector<int> &need_update_index;
  LocalState localstate;
  bool is_falling = false;
  bool canRollLeft = false;
  bool canRollRight = false;
  bool is_wait = false;
  int index_gem = -1;
  int gem_x = -1;
  int gem_y = -1;
  int f = -1;
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
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.zorb_hash);
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
      : m_dimx(dimx),
        m_dimy(dimy),
        m_obstacles(std::move(obstacles)),
        m_goal(std::move(goal)),
        m_eHeuristic(std::move(m_eHeuristic))  // NOLINT
  {}
    Environment(size_t dimx, size_t dimy,
              Location goal, std::vector<std::vector<int>> m_eHeuristic)
      : m_dimx(dimx),
        m_dimy(dimy),
        m_goal(std::move(goal)),
        m_eHeuristic(std::move(m_eHeuristic))  // NOLINT
  {}

  int admissibleHeuristic(const State& s) {
// #ifdef GemGoal    
//     if(s.is_falling){
//        if(s.x >= s.gem_x){is_roll_fall = true; std::cout << ",roll," << "\n"; return std::abs(s.x - s.gem_x)/2 + std::abs(s.y - s.gem_y);}
//        else return std::abs(s.x - s.gem_x) + std::abs(s.y - s.gem_y) + 1;
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
    if(!isExact) return std::abs(s.x - s.gem_x)/2 + std::abs(s.y - s.gem_y)/2;       
    else return abs(m_eHeuristic[s.x][s.y] - m_eHeuristic[s.gem_x][s.gem_y]);
    if(s.is_falling){
       if(s.x >= s.gem_x) return std::abs(s.x - s.gem_x)/2 + std::abs(s.y - s.gem_y);
       else return std::abs(s.x - s.gem_x) + std::abs(s.y - s.gem_y) + 1;
    }else if(s.canRollLeft) { 
      if(s.y <= s.gem_y && s.x >= s.gem_x) return std::abs(s.x - s.gem_x)/2 + std::abs(s.y - s.gem_y)/2;
      else if(s.y <= s.gem_y) return std::abs(s.x - s.gem_x) + std::abs(s.y - s.gem_y)/2;
      else if(s.x >= s.gem_x) return std::abs(s.x - s.gem_x)/2 + std::abs(s.y - s.gem_y);
      else return std::abs(s.x - s.gem_x) + 1 + std::abs(s.y - s.gem_y);
    }else if(s.canRollRight) {
      if(s.y >= s.gem_y && s.x >= s.gem_x) return std::abs(s.x - s.gem_x)/2  + std::abs(s.y - s.gem_y)/2;
      else if(s.y >= s.gem_y) return std::abs(s.x - s.gem_x)  + std::abs(s.y - s.gem_y)/2;
      else if(s.x >= s.gem_x) return std::abs(s.x - s.gem_x)/2 + std::abs(s.y - s.gem_y);
      else return std::abs(s.x - s.gem_x) + 1 + std::abs(s.y - s.gem_y);
    }else{
      if(s.gem_x == m_goal.x && s.gem_y == m_goal.y && isExact) return m_eHeuristic[s.x][s.y];
      else return std::abs(s.x - s.gem_x) + std::abs(s.y - s.gem_y);
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

  bool isSolution(const State& s) { 
#ifdef GemGoal
    return s.x == s.index_gem / m_dimy && s.y == s.index_gem % m_dimy; 
#endif
    return s.x == m_goal.x && s.y == m_goal.y; 
  }

//whether needs const
  void getNeighbors(State& s,
                    std::vector<Neighbor<State, Action, int> >& neighbors) {
    neighbors.clear();
    std::cout << "Current state "<< s.x <<", " << s.y << ", time " << s.time << ", hash, " << s.zorb_hash << ", size, " << s.grid.size()  << "----------------------------------"<< std::endl;
    // if((s.x == 12 && s.y == 21 &&  s.time == 1) || (s.x == 12 && s.y == 21 &&  s.time == 2) || (s.x == 8 && s.y == 26 &&  s.time == 8))
    // {
    //   // std::cout << "Current state "<< s.x <<", " << s.y << ", time " << s.time << ", " << s.grid.size()  << "----------------------------------"<< std::endl;
    //   for (int h = 0; h < state_game.board.rows; ++h) {
    //     for (int w = 0; w < state_game.board.cols; ++w) {
    //       std::cout << kCellTypeToElement[s.grid[h * state_game.board.cols + w] + 1].id;
    //         // std::cout << kCellTypeToElement.at(state_game.board.grid[h * state_game.board.cols + w]).id;
    //     }
    //     std::cout << std::endl;
    //   } 
    // }
    // if(s.zorb_hash != zorb_hash_temp) std::cout << "ERROR \n";
    // std::cout << "Current state "<< s.x <<", " << s.y << ", time " << s.time << ", hash, " << s.zorb_hash  << ", new-hash " << zorb_hash_temp << ", size, " << s.grid.size()  << "----------------------------------"<< std::endl;

    int index = s.x * m_dimy + s.y;
    sort(s.need_update_index.begin(), s.need_update_index.end());

    state_game.board.grid.assign(s.grid.begin(), s.grid.end());
    state_game.board.need_update_index.clear();
    state_game.board.need_update_index.assign(s.need_update_index.begin(), s.need_update_index.end());
    state_game.resetLocalState(s.localstate);
    state_game.board.agent_pos = index;
    state_game.board.agent_idx = index;
    state_game.curr_gem_index = s.index_gem;
    state_game.board.zorb_hash = s.zorb_hash;
    uint64_t zorb_hash_temp = s.zorb_hash;
    state_game.apply_action(0);
    if(index == state_game.board.agent_pos || state_game.board.agent_pos == kAgentPosExit){
      State wait(s.x, s.y, s.time + 1, state_game.board.zorb_hash, *gridCache.getItem(), *indexCache.getItem());
      wait.grid.assign(state_game.board.grid.begin(), state_game.board.grid.end());
      wait.need_update_index.assign(state_game.board.need_update_index.begin(), state_game.board.need_update_index.end());
      wait.localstate = state_game.local_state;
#ifdef GemGoal      
      wait.index_gem =  state_game.curr_gem_index;
      wait.gem_x = state_game.curr_gem_index / m_dimy;
      wait.gem_y = state_game.curr_gem_index % m_dimy;
      wait.canRollLeft = state_game.CanRollLeft(wait.index_gem);
      wait.canRollRight = state_game.CanRollRight(wait.index_gem);
      wait.is_falling = state_game.IsType(wait.index_gem, kElEmpty, Directions::kDown);   
#endif      
      // if(wait.zorb_hash != zorb_hash_temp)
      wait.is_wait = true;
      neighbors.emplace_back(Neighbor<State, Action, int>(wait, Action::Wait, 1));
      num_generated++;
      std::cout << "Neighbor  "<<", " << wait.zorb_hash << ", " << wait.x <<", " << wait.y << ", time " << wait.time << ", " << s.grid.size() << std::endl;
      // num_state++;
    }

    if(s.grid[index - m_dimy] != 18 && s.grid[index - m_dimy] != 19){
      state_game.board.grid.clear();
      state_game.board.grid.assign(s.grid.begin(), s.grid.end());
      state_game.board.need_update_index.clear();
      state_game.board.need_update_index.assign(s.need_update_index.begin(), s.need_update_index.end());
      state_game.board.agent_pos = index;
      state_game.board.agent_idx = index;
      state_game.resetLocalState(s.localstate);
      state_game.curr_gem_index = s.index_gem;
      // state_game.init_hash();
      state_game.board.zorb_hash = zorb_hash_temp;
      state_game.apply_action(1);
      if ((index - m_dimy) == state_game.board.agent_pos || state_game.board.agent_pos == kAgentPosExit) {
        State up(s.x - 1, s.y, s.time + 1, state_game.board.zorb_hash, *gridCache.getItem(), *indexCache.getItem());
        up.grid.assign(state_game.board.grid.begin(), state_game.board.grid.end());
        up.need_update_index.assign(state_game.board.need_update_index.begin(), state_game.board.need_update_index.end());
        up.localstate = state_game.local_state;
  #ifdef GemGoal            
        up.index_gem =  state_game.curr_gem_index;
        up.gem_x = state_game.curr_gem_index / m_dimy;
        up.gem_y = state_game.curr_gem_index % m_dimy;
        up.canRollLeft = state_game.CanRollLeft(up.index_gem);
        up.canRollRight = state_game.CanRollRight(up.index_gem);
        up.is_falling = state_game.IsType(up.index_gem, kElEmpty, Directions::kDown);
  #endif      
        neighbors.emplace_back(Neighbor<State, Action, int>(up, Action::Up, 1));
        num_generated++;
        // std::cout << state_game.local_state.gems_collected << " gems " << std::endl;
        std::cout << "Neighbor  " << ", " << up.zorb_hash << ", " << up.x <<", " << up.y << ", time " << up.time << ", " << s.grid.size() << std::endl;
        // num_state++;
      }

    }

    if(s.grid[index + m_dimy] != 18 && s.grid[index + m_dimy] != 19){
      if(s.zorb_hash == 7598063330544506766) std::cout << "Down 111\n";
      state_game.board.grid.clear();
      state_game.board.grid.assign(s.grid.begin(), s.grid.end());
      state_game.board.need_update_index.clear();
      state_game.board.need_update_index.assign(s.need_update_index.begin(), s.need_update_index.end());
      state_game.board.agent_pos = index;
      state_game.board.agent_idx = index;
      state_game.resetLocalState(s.localstate);
      state_game.curr_gem_index = s.index_gem;
      state_game.board.zorb_hash = zorb_hash_temp;
      state_game.apply_action(3);
      if (index + m_dimy == state_game.board.agent_pos || state_game.board.agent_pos == kAgentPosExit) {
        if(s.zorb_hash == 7598063330544506766) std::cout << "Down 222\n";        
        State down(s.x + 1, s.y, s.time + 1, state_game.board.zorb_hash, *gridCache.getItem(), *indexCache.getItem());
        down.grid.assign(state_game.board.grid.begin(), state_game.board.grid.end());
        down.need_update_index.assign(state_game.board.need_update_index.begin(), state_game.board.need_update_index.end());
        down.localstate = state_game.local_state;
#ifdef GemGoal      
        down.index_gem =  state_game.curr_gem_index;
        down.gem_x = state_game.curr_gem_index / m_dimy;
        down.gem_y = state_game.curr_gem_index % m_dimy;      
        down.canRollLeft = state_game.CanRollLeft(down.index_gem);
        down.canRollRight = state_game.CanRollRight(down.index_gem); 
        down.is_falling = state_game.IsType(down.index_gem, kElEmpty, Directions::kDown);
#endif            
        neighbors.emplace_back(Neighbor<State, Action, int>(down, Action::Down, 1));
        num_generated++;
        std::cout << "Neighbor  " << ", " << down.zorb_hash << ", " << down.x <<", " << down.y << ", time " << down.time << ", " << s.grid.size() << std::endl;
        // num_state++;   
      }
    }

    if(s.grid[index - 1] != 18 && s.grid[index - 1] != 19){
      state_game.board.grid.clear();
      state_game.board.grid.assign(s.grid.begin(), s.grid.end());
      state_game.board.need_update_index.clear();
      state_game.board.need_update_index.assign(s.need_update_index.begin(), s.need_update_index.end());
      state_game.board.agent_pos = index;
      state_game.board.agent_idx = index;
      state_game.resetLocalState(s.localstate);
      state_game.curr_gem_index = s.index_gem;
      state_game.board.zorb_hash = zorb_hash_temp;
      state_game.apply_action(4);
      if (index - 1 == state_game.board.agent_pos || state_game.board.agent_pos == kAgentPosExit) {
        State left(s.x, s.y - 1, s.time + 1, state_game.board.zorb_hash, *gridCache.getItem(), *indexCache.getItem());
        left.grid.assign(state_game.board.grid.begin(), state_game.board.grid.end());
        left.need_update_index.assign(state_game.board.need_update_index.begin(), state_game.board.need_update_index.end());
        left.localstate = state_game.local_state;
#ifdef GemGoal      
        left.index_gem =  state_game.curr_gem_index;
        left.gem_x = state_game.curr_gem_index / m_dimy;
        left.gem_y = state_game.curr_gem_index % m_dimy;
        left.canRollLeft = state_game.CanRollLeft(left.index_gem);
        left.canRollRight = state_game.CanRollRight(left.index_gem);
        left.is_falling = state_game.IsType(left.index_gem, kElEmpty, Directions::kDown);
#endif      
        neighbors.emplace_back(Neighbor<State, Action, int>(left, Action::Left, 1));
        num_generated++;
        std::cout << "Neighbor  " << ", " << left.zorb_hash << ", "<< left.x <<", " << left.y << ", time " << left.time << ", " << s.grid.size() << std::endl;
        // num_state++;
      }
    }

    if(s.grid[index + 1] != 18 && s.grid[index + 1] != 19){
      state_game.board.grid.assign(s.grid.begin(), s.grid.end());
      state_game.board.need_update_index.clear();
      state_game.board.need_update_index.assign(s.need_update_index.begin(), s.need_update_index.end());
      state_game.board.agent_pos = index;
      state_game.board.agent_idx = index;
      state_game.resetLocalState(s.localstate);
      state_game.curr_gem_index = s.index_gem;
      state_game.board.zorb_hash = zorb_hash_temp;
      state_game.apply_action(2);
      if (index + 1 == state_game.board.agent_pos || state_game.board.agent_pos == kAgentPosExit) {
        State right(s.x, s.y + 1, s.time + 1, state_game.board.zorb_hash, *gridCache.getItem(), *indexCache.getItem());
        right.grid.assign(state_game.board.grid.begin(), state_game.board.grid.end());
        right.need_update_index.assign(state_game.board.need_update_index.begin(), state_game.board.need_update_index.end());
        right.localstate = state_game.local_state;
#ifdef GemGoal      
        right.index_gem =  state_game.curr_gem_index;
        right.gem_x = state_game.curr_gem_index / m_dimy;
        right.gem_y = state_game.curr_gem_index % m_dimy;
        right.canRollLeft = state_game.CanRollLeft(right.index_gem);
        right.canRollRight = state_game.CanRollRight(right.index_gem);
        right.is_falling = state_game.IsType(right.index_gem, kElEmpty, Directions::kDown);
#endif      
        neighbors.emplace_back(Neighbor<State, Action, int>(right, Action::Right, 1));
        num_generated++;
        std::cout << "Neighbor  " << "," << right.zorb_hash << "," << right.x <<", " << right.y << ", time " << right.time << ", " << s.grid.size() << std::endl;
        // num_state++;
      }
    }

    indexCache.returnItem(&s.need_update_index);
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
 int  num_expand = 0;
 int num_generated = 0;
 bool isExact = true;
 bool is_roll_fall = false;
 private:
  int m_dimx;
  int m_dimy;
  std::unordered_set<State> m_obstacles;
  Location m_goal;
  std::vector<std::vector<int>> m_eHeuristic;
};


void getExactHeuristic(std::vector<std::vector<int>> &eHeuristic, const std::vector<std::vector<bool>> map_obstacle, int goalX, int goalY, int dimx, int dimy)
{
	int xx[5] = {0, 0, -1, 1};
	int yy[5] = {1, -1, 0, 0};

  std::cout <<eHeuristic.size() <<  "Test" << goalX << ", " << goalY << std::endl;
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

int main(int argc, char* argv[]) {

    struct rusage r_usage;
   	getrusage(RUSAGE_SELF, &r_usage);
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    int goalX, goalY;
    int goalXD, goalYD;
    std::string filename;
    std::string f;
    desc.add_options()("help,h", "produce help message")(
      "goalXD,x", po::value<int>(&goalXD)->required(), "goal position x-component")(
      "goalYD,y", po::value<int>(&goalYD)->required(), "goal position y-component")
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
//    std::cout << board_str << std::endl;
//    getline(infile,board_str);
//    std::cout << board_str << std::endl;

    params["game_board_str"] = GameParameter(board_str);
    RNDGameState state_p(params);
    state_game = state_p;
    
    std::cout << state_p.board.rows << std::endl;
    std::vector<int8_t> grid;
    grid.swap(state_p.board.grid);

    std::cout << state_p.board.grid.size() << "end\n";
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

    Timer total;
    std::vector <PlanResult<State, Action, int>>  solutions;
    LocalState localstate;
    int next_index = goals_loc[0].x * state_p.board.cols + goals_loc[0].y;
    int index_g = 0;
    bool is_falling = false;
    std::vector<int> faild;
    while(index_g < goals_loc.size()){
      std::cout << "----\n";
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

      state_game.board.grid.assign(grid.begin(), grid.end());
      state_game.init_hash();
      state_game.num_apply_action = 0;

      State start(startX, startY, 0, state_game.board.zorb_hash, *gridCache.getItem(), *indexCache.getItem());
      // if(index_g == 0){
      //   start.grid.assign(grid.begin(), grid.end());
      //   start.need_update_index = state_p.board.need_update_index;
      //   start.localstate = localstate;
      // }else{
      //   start.grid.assign(solutions[index_g - 1].states[0].first.grid.begin(), solutions[index_g - 1].states[0].first.grid.end());
      //   start.need_update_index.assign(solutions[index_g - 1].states[0].first.need_update_index.begin(), solutions[index_g - 1].states[0].first.need_update_index.end());
      //   start.localstate = solutions[index_g - 1].states[0].first.localstate;
      // }
        start.grid.assign(grid.begin(), grid.end());
        start.need_update_index = state_p.board.need_update_index;
        start.localstate = localstate;

      bool success = false;
      goalX = next_index/state_p.board.cols;
      goalY = next_index%state_p.board.cols;
      if(!(goalX == goalXD && goalY == goalYD)){
        index_g++;
        continue;
      }
      // std::cout << startX << ", " << startY << ", " << goalX << ", " << goalY << " \n";

      std::vector<std::vector<int>> eHeuristicGoal(state_p.board.rows, std::vector<int>(state_p.board.cols + 1, -1));
      getExactHeuristic(eHeuristicGoal, map_obstacle, goalX, goalY, state_p.board.rows, state_p.board.cols);
      // std::cout << "goal " << goalX << ", " << goalY << std::endl;
      // for(int test_i = 0; test_i < eHeuristicGoal.size(); test_i++){
      //   for(int test_j = 0; test_j < eHeuristicGoal[test_i].size(); test_j++){
      //     std::cout << eHeuristicGoal[test_i][test_j] << ", ";
      //   }
      //   std::cout << std::endl;
      // }
      std::cout << "Goal " << goalX << ", " << goalY << std::endl;
      Environment env(state_p.board.rows, state_p.board.cols, Location(goalX, goalY), eHeuristicGoal);
      AStar<State, Action, int, Environment, vectorCache<int8_t>, vectorCache<int>> astar(env, gridCache, indexCache);
      start.index_gem = next_index;
      start.gem_x = goalX;
      start.gem_y = goalY;
      state_game.board.grid.assign(start.grid.begin(), start.grid.end());
      start.is_falling = state_game.IsType(next_index, kElEmpty, Directions::kDown);;
      start.canRollLeft = state_game.CanRollLeft(next_index);
      start.canRollRight = state_game.CanRollRight(next_index);
    
      std::cout << next_index / state_p.board.cols << ", " << next_index % state_p.board.cols << std::endl;

      PlanResult<State, Action, int> solution;
      Timer timer;       

      if (env.stateValid(start)) {
        success = astar.search(start, solution);
        if(success){
          std::cout << ",Pathfinding success !,";
          if(solution.states[0].first.x == goalX && solution.states[0].first.y == goalY) std::cout << ",same,";
          else std::cout << ",diff,";
        }
      }
      timer.stop();
      
      getrusage(RUSAGE_SELF, &r_usage);
      if(success) std::cout <<filename <<  ", success, cost, " << solution.cost <<"," << env.is_roll_fall << ",start, " << startX << ", " << startY << ", goal, " << goalX <<", " << goalY << ", memory, " << r_usage.ru_maxrss  << ", time, " <<  timer.elapsedSeconds() <<  ", Expansion, " << env.num_expand << ", generation, " << env.num_generated << ",num_action," << state_game.num_apply_action <<std::endl;    
      else std::cout << filename <<  ", not success, "<< env.is_roll_fall << ", ,start, " << startX << ", " << startY << ", goal, " << goalX <<", " << goalY << ", memory, " << r_usage.ru_maxrss  << ", time, " <<  timer.elapsedSeconds() <<  ", Expansion, " << env.num_expand << ", generation, " << env.num_generated << ",num_action," << state_game.num_apply_action <<std::endl;    

      index_g++;
      if(success){
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
              for (int h = 0; h < state_p.board.rows; ++h) {
                for (int w = 0; w < state_p.board.cols; ++w) {
                  std::cout << kCellTypeToElement[solution.states.back().first.grid[h * state_p.board.cols + w] + 1].id;
                  // if(h==0) grid[h * state_p.board.cols + w] = 100;
                }
                std::cout << std::endl;
              }                    
            }
            std::cout << solution.states[i].second << ": " << solution.states[i].first << "->" << solution.actions[i - 1].first
                  << "(cost: " << solution.actions[i - 1].second << ")" << std::endl;
           for (int h = 0; h < state_p.board.rows; ++h) {
            for (int w = 0; w < state_p.board.cols; ++w) {
              std::cout << kCellTypeToElement[solution.states[i].first.grid[h * state_p.board.cols + w] + 1].id;
              // if(h==0) grid[h * state_p.board.cols + w] = 100;
            }
            std::cout << std::endl;
          }                        
        }        
      }      
      continue;
      
      next_index  = -1;
      if (success) {
        for(int iii = 0; iii < faild.size(); iii++){
          if(faild[iii] == current_index) {
            faild[iii] = -1;
            break;
          }
        }        
        std::cout << "Planning successful! Total cost: " << solution.cost << ", time, " << timer.elapsedSeconds() << ", expand," << env.num_expand
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
            for (int h = 0; h < state_p.board.rows; ++h) {
              for (int w = 0; w < state_p.board.cols; ++w) {
                std::cout << kCellTypeToElement[solution.states.back().first.grid[h * state_p.board.cols + w] + 1].id;
                  // if(h==0) grid[h * state_p.board.cols + w] = 100;
              }
                std::cout << std::endl;
            }                    
          }
          std::cout << solution.states[i].second << ": " << solution.states[i].first << "->" << solution.actions[i - 1].first
            << "(cost: " << solution.actions[i - 1].second << ")" << std::endl;
          for (int h = 0; h < state_p.board.rows; ++h) {
            for (int w = 0; w < state_p.board.cols; ++w) {
              std::cout << kCellTypeToElement[solution.states[i].first.grid[h * state_p.board.cols + w] + 1].id;
              // if(h==0) grid[h * state_p.board.cols + w] = 100;
            }
            std::cout << std::endl;
          }                        
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
        std::cout << "id " << index_g << ",index," << current_index <<  ", Planning NOT successful!" << std::endl;
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
    std::cout << total.elapsedSeconds() <<std::endl;    

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