#include <fstream>
#include <iostream>
#include <queue>
#include <rnd/stonesngems.h>
#include <random>
#include <algorithm>

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

int num_state = 0;

struct State {
  State(int x, int y, int time, std::vector<int8_t>&grid, std::vector<int>&need_update_index) : x(x), y(y), 
         time(time), grid(grid), need_update_index(need_update_index) {}  
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
  std::vector<int8_t> &grid;
  std::vector<int> &need_update_index;
  LocalState localstate;
  bool is_falling = false;
  bool canRollLeft = false;
  bool canRollRight = false;
  int index_gem = -1;
  int gem_x = -1;
  int gem_y = -1;
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
              Location goal)
      : m_dimx(dimx),
        m_dimy(dimy),
        m_obstacles(std::move(obstacles)),
        m_goal(std::move(goal))  // NOLINT
  {}
    Environment(size_t dimx, size_t dimy,
              Location goal)
      : m_dimx(dimx),
        m_dimy(dimy),
        m_goal(std::move(goal))  // NOLINT
  {}

  int admissibleHeuristic(const State& s) {
#ifdef GemGoal    
    if(s.is_falling){
       if(s.y >= s.gem_y) return std::abs(s.x - s.gem_x) + std::abs(s.y - s.gem_y)/2;
       else return std::abs(s.x - s.gem_x) + std::abs(s.y - s.gem_y) + 1;
    }else if(s.canRollLeft) { 
      if(s.x <= s.gem_x) return std::abs(s.x - s.gem_x)/2 + std::abs(s.y - s.gem_y);
      else return std::abs(s.x - s.gem_x) + 1 + std::abs(s.y - s.gem_y);
    }else if(s.canRollRight) { 
      if(s.x >= s.gem_x) return std::abs(s.x - s.gem_x)/2 + std::abs(s.y - s.gem_y);
      else return std::abs(s.x - s.gem_x) + 1 + std::abs(s.y - s.gem_y);
    }
#endif    
    return std::abs(s.x - m_goal.x) + std::abs(s.y - m_goal.y);
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
    // std::cout << "Current state "<< s.x <<", " << s.y << ", time " << s.time << ", " << s.grid.size()  << "----------------------------------"<< std::endl;
    // if(s.time  <=3 )
    // {
    // std::cout << "Current state "<< s.x <<", " << s.y << ", time " << s.time << ", " << s.grid.size()  << "----------------------------------"<< std::endl;
    // for (int h = 0; h < state_game.board.rows; ++h) {
    //     for (int w = 0; w < state_game.board.cols; ++w) {
    //       std::cout << kCellTypeToElement[state_game.board.grid[h * state_game.board.cols + w] + 1].id;
    //         // std::cout << kCellTypeToElement.at(state_game.board.grid[h * state_game.board.cols + w]).id;
    //     }
    //     std::cout << std::endl;
    // } 
    // }

    neighbors.clear();
    state_game.board.grid.assign(s.grid.begin(), s.grid.end());
    state_game.board.need_update_index.clear();
    sort(s.need_update_index.begin(), s.need_update_index.end());
    state_game.board.need_update_index.assign(s.need_update_index.begin(), s.need_update_index.end());
    state_game.resetLocalState(s.localstate);

    int index = s.x * m_dimy + s.y;
    state_game.board.agent_pos = index;
    state_game.board.agent_idx = index;
    state_game.curr_gem_index = s.index_gem;
    state_game.apply_action(0);
    if(index == state_game.board.agent_pos || state_game.board.agent_pos == kAgentPosExit){
      State wait(s.x, s.y, s.time + 1, *gridCache.getItem(), *indexCache.getItem());
      wait.grid.swap(state_game.board.grid);
      wait.need_update_index.swap(state_game.board.need_update_index);
      wait.localstate = state_game.local_state;
#ifdef GemGoal      
      wait.index_gem =  state_game.curr_gem_index;
      wait.gem_x = state_game.curr_gem_index / m_dimy;
      wait.gem_y = state_game.curr_gem_index % m_dimy;
      wait.canRollLeft = state_game.CanRollLeft(wait.index_gem);
      wait.canRollRight = state_game.CanRollRight(wait.index_gem);
      wait.is_falling = state_game.IsType(wait.index_gem, kElEmpty, Directions::kDown);
#endif      
      neighbors.emplace_back(Neighbor<State, Action, int>(wait, Action::Wait, 1));
      // std::cout << "Neighbor  "<< wait.x <<", " << wait.y << ", time " << wait.time << ", " << s.grid.size() << std::endl;
      // num_state++;
    }

    state_game.board.grid.assign(s.grid.begin(), s.grid.end());
    state_game.board.need_update_index.clear();
    state_game.board.need_update_index.assign(s.need_update_index.begin(), s.need_update_index.end());
    state_game.board.agent_pos = index;
    state_game.board.agent_idx = index;
    state_game.resetLocalState(s.localstate);
    state_game.curr_gem_index = s.index_gem;
    state_game.apply_action(1);
    if ((index - m_dimy) == state_game.board.agent_pos || state_game.board.agent_pos == kAgentPosExit) {
      State up(s.x - 1, s.y, s.time + 1, *gridCache.getItem(), *indexCache.getItem());
      up.grid.swap(state_game.board.grid);
      up.need_update_index.swap(state_game.board.need_update_index);
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
      // std::cout << state_game.local_state.gems_collected << " gems " << std::endl;
      // std::cout << "Neighbor  "<< up.x <<", " << up.y << ", time " << up.time << ", " << s.grid.size() << std::endl;
      // num_state++;
    }

    state_game.board.grid.assign(s.grid.begin(), s.grid.end());
    state_game.board.need_update_index.clear();
    state_game.board.need_update_index.assign(s.need_update_index.begin(), s.need_update_index.end());
    state_game.board.agent_pos = index;
    state_game.board.agent_idx = index;
    state_game.resetLocalState(s.localstate);
    state_game.curr_gem_index = s.index_gem;
    state_game.apply_action(3);
    if (index + m_dimy == state_game.board.agent_pos || state_game.board.agent_pos == kAgentPosExit) {
      State down(s.x + 1, s.y, s.time + 1, *gridCache.getItem(), *indexCache.getItem());
      down.grid.swap(state_game.board.grid);
      down.need_update_index.swap(state_game.board.need_update_index);
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
      // std::cout << "Neighbor  "<< down.x <<", " << down.y << ", time " << down.time << ", " << s.grid.size() << std::endl;
      // num_state++;   
    }

    state_game.board.grid.assign(s.grid.begin(), s.grid.end());
    state_game.board.need_update_index.clear();
    state_game.board.need_update_index.assign(s.need_update_index.begin(), s.need_update_index.end());
    state_game.board.agent_pos = index;
    state_game.board.agent_idx = index;
    state_game.resetLocalState(s.localstate);
    state_game.curr_gem_index = s.index_gem;
    state_game.apply_action(4);
    if (index - 1 == state_game.board.agent_pos || state_game.board.agent_pos == kAgentPosExit) {
      State left(s.x, s.y - 1, s.time + 1, *gridCache.getItem(), *indexCache.getItem());
      left.grid.swap(state_game.board.grid);
      left.need_update_index.swap(state_game.board.need_update_index);
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
      // std::cout << "Neighbor  "<< left.x <<", " << left.y << ", time " << left.time << ", " << s.grid.size() << std::endl;
      // num_state++;
    }

    state_game.board.grid.assign(s.grid.begin(), s.grid.end());
    state_game.board.need_update_index.clear();
    state_game.board.need_update_index.assign(s.need_update_index.begin(), s.need_update_index.end());
    state_game.board.agent_pos = index;
    state_game.board.agent_idx = index;
    state_game.resetLocalState(s.localstate);
    state_game.curr_gem_index = s.index_gem;
    state_game.apply_action(2);
    if (index + 1 == state_game.board.agent_pos || state_game.board.agent_pos == kAgentPosExit) {
      State right(s.x, s.y + 1, s.time + 1, *gridCache.getItem(), *indexCache.getItem());
      right.grid.swap(state_game.board.grid);
      right.need_update_index.swap(state_game.board.need_update_index);
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
      // std::cout << "Neighbor  "<< right.x <<", " << right.y << ", time " << right.time << ", " << s.grid.size() << std::endl;
      // num_state++;
    }

    // gridCache.returnItem(&s.grid);
    indexCache.returnItem(&s.need_update_index);
    // return 0;
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
 private:
  int m_dimx;
  int m_dimy;
  std::unordered_set<State> m_obstacles;
  Location m_goal;
};

int main(int argc, char* argv[]) {

    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
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
		  std::cerr << e.what() << std::endl
			  	  << std::endl;
		  std::cerr << desc << std::endl;
		  return 1;
	  }
      
    std::cout << "Begin test \n";
    const std::string filename="./levels/bd_01_1.txt";
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

    for (int h = 0; h < state_p.board.rows; ++h) {
      for (int w = 0; w < state_p.board.cols; ++w) {
        if(grid[h * state_p.board.cols + w] == 0){
          startX = h;
          startY = w;
        }
          std::cout << kCellTypeToElement[grid[h * state_p.board.cols + w] + 1].id;
      }
      std::cout << std::endl;
    }

    for (int h = 0; h < state_p.board.rows; ++h) {
      for (int w = 0; w < state_p.board.cols; ++w) {
        //printf("%d ", grid[h * state_p.board.cols + w]);
        if(grid[h * state_p.board.cols + w] == 0){
          startX = h;
          startY = w;
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
    goals_loc.push_back(Location(1, 10));
    // goals_loc.push_back(Location(2, 22));
    // goals_loc.push_back(Location(8, 9));
    // goals_loc.push_back(Location(8, 27));
    // goals_loc.push_back(Location(8, 30));
    // goals_loc.push_back(Location(9, 3));
    // goals_loc.push_back(Location(16, 38));    
    // goals_loc.push_back(Location(11, 32));
    // goals_loc.push_back(Location(15, 16));
    // goals_loc.push_back(Location(17, 28));
    // goals_loc.push_back(Location(18, 6));
    // goals_loc.push_back(Location(18, 28));
    // goals_loc.push_back(Location(19, 28));
    // goals_loc.push_back(Location(20, 2));
    // goals_loc.push_back(Location(16, 38));

    Timer total;
    std::vector <PlanResult<State, Action, int>>  solutions;
    LocalState localstate;
    int next_index = goals_loc[0].x * state_p.board.cols + goals_loc[0].y;
    int index_g = 0;
    bool is_falling = false;
    while(1){
      if(next_index == -1) break;
      if(index_g  == 2) break;
      if(index_g != 0 ){
        startX = solutions[index_g - 1].states[0].first.x;
        startY = solutions[index_g - 1].states[0].first.y;
      }
      State start(startX, startY, 0, *gridCache.getItem(), *indexCache.getItem());
      if(index_g == 0){
        start.grid.assign(grid.begin(), grid.end());
        start.need_update_index = state_p.board.need_update_index;
        start.localstate = localstate;
      }else{
        start.grid.assign(solutions[index_g - 1].states[0].first.grid.begin(), solutions[index_g - 1].states[0].first.grid.end());
        start.need_update_index.assign(solutions[index_g - 1].states[0].first.need_update_index.begin(), solutions[index_g - 1].states[0].first.need_update_index.end());
        start.localstate = solutions[index_g - 1].states[0].first.localstate;
      }

      bool success = false;
      goalX = next_index/state_p.board.cols;
      goalY = next_index%state_p.board.cols;
      Environment env(state_p.board.rows, state_p.board.cols, Location(goalX, goalY));
      AStar<State, Action, int, Environment, vectorCache<int8_t>, vectorCache<int>> astar(env, gridCache, indexCache);
      start.index_gem = next_index;
      start.gem_x = goalX;
      start.gem_y = goalY;
      state_game.board.grid.assign(start.grid.begin(), start.grid.end());
      start.is_falling = is_falling;
      start.canRollLeft = state_game.CanRollLeft(next_index);
      start.canRollRight = state_game.CanRollRight(next_index);
    
      std::cout << next_index / state_p.board.cols << ", " << next_index % state_p.board.cols << std::endl;

      PlanResult<State, Action, int> solution;
      Timer timer;

        for (int h = 0; h < state_p.board.rows; ++h) {
          for (int w = 0; w < state_p.board.cols; ++w) {

              std::cout << kCellTypeToElement[start.grid[h * state_p.board.cols + w] + 1].id;
              // if(h==0) grid[h * state_p.board.cols + w] = 100;
          }
          std::cout << std::endl;
        }           

      if (env.stateValid(start)) {
        success = astar.search(start, solution);
        if(success){
          std::cout << "Pathfinding success !\n";
        }
      }
      timer.stop();
      std::cout << timer.elapsedSeconds() <<  ", " << env.num_expand <<std::endl;      
      next_index  = -1;
      if (success) {
        std::cout << "Planning successful! Total cost: " << solution.cost << ", " << timer.elapsedSeconds() << ", " << env.num_expand
                << std::endl;
        // std::cout << solution.states.back().second << ": "
        //         << solution.states.back().first;
        // std::cout << solution.actions.size() - 1 << std::endl;
         for (int i = solution.actions.size() - 1; i > 0; i--) {
          
             if(i == solution.actions.size() - 1)   std::cout << solution.states.back().second << ": "
                << solution.states.back().first <<  "->" << solution.actions[i].first
                  << "(cost: " << solution.actions[i].second << ")" << std::endl;
            std::cout << solution.states[i].second << ": " << solution.states[i].first << "->" << solution.actions[i - 1].first
                  << "(cost: " << solution.actions[i - 1].second << ")" << std::endl;                  
        }
     std::cout << solution.states[0].second << ": "
                << solution.states[0].first;       
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
     std::cout << solution.states[0].second << ": "
                << solution.states[0].first;       
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