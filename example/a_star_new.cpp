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
  // State(int x, int y, int time) : x(x), y(y), time(time) {}
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
    return std::abs(s.x - m_goal.x) + std::abs(s.y - m_goal.y);
  }

  bool isSolution(const State& s) { return s.x == m_goal.x && s.y == m_goal.y; }

//whether needs const
  void getNeighbors(State& s,
                    std::vector<Neighbor<State, Action, int> >& neighbors) {
    // std::cout << "Current state "<< s.x <<", " << s.y << ", time " << s.time << ", " << s.grid.size() << std::endl;

    neighbors.clear();
    state_game.board.grid.assign(s.grid.begin(), s.grid.end());
    state_game.board.need_update_index.clear();
    sort(s.need_update_index.begin(), s.need_update_index.end());
    state_game.board.need_update_index.assign(s.need_update_index.begin(), s.need_update_index.end());
    // for (int h = 0; h < state_game.board.rows; ++h) {
    //     for (int w = 0; w < state_game.board.cols; ++w) {
    //       std::cout << kCellTypeToElement[state_game.board.grid[h * state_game.board.cols + w] + 1].id;
    //         // std::cout << kCellTypeToElement.at(state_game.board.grid[h * state_game.board.cols + w]).id;
    //     }
    //     std::cout << std::endl;
    // } 
    // }
    // if((s.x == 13 && s.y == 31 && s.time == 39) 
    // // || 
    // // (s.x == 3 && s.y == 3 && s.time == 1) || (s.x == 4 && s.y == 3 && s.time == 2) || 
    // // (s.x == 4 && s.y == 4 && s.time == 3) || (s.x == 4 && s.y == 5 && s.time == 4) ||
    // // (s.x == 3 && s.y == 5 && s.time == 5) || (s.x == 2 && s.y == 5 && s.time == 6) ||
    // // (s.x == 2 && s.y == 4 && s.time == 7) || (s.x == 1 && s.y == 4 && s.time == 8))
    // )
    // {
    //   for(int xx = 0; xx < s.need_update_index.size(); xx ++){
    //     std::cout << s.need_update_index[xx] << ", ";
    //   }
    //   std::cout << std::endl;
    // }

    int index = s.x * m_dimy + s.y;
    int index1 = -1;
    state_game.board.agent_pos = index;
    state_game.board.agent_idx = index;

    state_game.apply_action(0);
    if(index == state_game.board.agent_pos){
      State wait(s.x, s.y, s.time + 1, *gridCache.getItem(), *indexCache.getItem());
      wait.grid.swap(state_game.board.grid);
      wait.need_update_index.swap(state_game.board.need_update_index);
      neighbors.emplace_back(Neighbor<State, Action, int>(wait, Action::Wait, 1));
      // std::cout << "Neighbor  "<< wait.x <<", " << wait.y << ", time " << wait.time << ", " << s.grid.size() << std::endl;
      // num_state++;
    } 

    state_game.board.grid.assign(s.grid.begin(), s.grid.end());
    state_game.board.need_update_index.clear();
    state_game.board.need_update_index.assign(s.need_update_index.begin(), s.need_update_index.end());
    state_game.board.agent_pos = index;
    state_game.board.agent_idx = index;
    state_game.apply_action(1);
    if ((index - m_dimy) == state_game.board.agent_pos) {
      State up(s.x - 1, s.y, s.time + 1, *gridCache.getItem(), *indexCache.getItem());
      up.grid.swap(state_game.board.grid);
      up.need_update_index.swap(state_game.board.need_update_index);
      neighbors.emplace_back(Neighbor<State, Action, int>(up, Action::Up, 1));
      // std::cout << "Neighbor  "<< up.x <<", " << up.y << ", time " << up.time << ", " << s.grid.size() << std::endl;
      // num_state++;
    }

    state_game.board.grid.assign(s.grid.begin(), s.grid.end());
    state_game.board.need_update_index.clear();
    state_game.board.need_update_index.assign(s.need_update_index.begin(), s.need_update_index.end());
    state_game.board.agent_pos = index;
    state_game.board.agent_idx = index;
    state_game.apply_action(3);
    if (index + m_dimy == state_game.board.agent_pos) {
      State down(s.x + 1, s.y, s.time + 1, *gridCache.getItem(), *indexCache.getItem());
      down.grid.swap(state_game.board.grid);
      down.need_update_index.swap(state_game.board.need_update_index);
      neighbors.emplace_back(Neighbor<State, Action, int>(down, Action::Down, 1));
      // std::cout << "Neighbor  "<< down.x <<", " << down.y << ", time " << down.time << ", " << s.grid.size() << std::endl;
      // num_state++;   
    }

    state_game.board.grid.assign(s.grid.begin(), s.grid.end());
    state_game.board.need_update_index.clear();
    state_game.board.need_update_index.assign(s.need_update_index.begin(), s.need_update_index.end());
    state_game.board.agent_pos = index;
    state_game.board.agent_idx = index;
    state_game.apply_action(4);
    if (index - 1 == state_game.board.agent_pos) {
      State left(s.x, s.y - 1, s.time + 1, *gridCache.getItem(), *indexCache.getItem());
      left.grid.swap(state_game.board.grid);
      left.need_update_index.swap(state_game.board.need_update_index);
      neighbors.emplace_back(Neighbor<State, Action, int>(left, Action::Left, 1));
      // std::cout << "Neighbor  "<< left.x <<", " << left.y << ", time " << left.time << ", " << s.grid.size() << std::endl;
      // num_state++;
    }

    state_game.board.grid.assign(s.grid.begin(), s.grid.end());
    state_game.board.need_update_index.clear();
    state_game.board.need_update_index.assign(s.need_update_index.begin(), s.need_update_index.end());
    state_game.board.agent_pos = index;
    state_game.board.agent_idx = index;
    state_game.apply_action(2);
    if (index + 1 == state_game.board.agent_pos) {
      State right(s.x, s.y + 1, s.time + 1, *gridCache.getItem(), *indexCache.getItem());
      right.grid.swap(state_game.board.grid);
      right.need_update_index.swap(state_game.board.need_update_index);
      neighbors.emplace_back(Neighbor<State, Action, int>(right, Action::Right, 1));
      // std::cout << "Neighbor  "<< right.x <<", " << right.y << ", time " << right.time << ", " << s.grid.size() << std::endl;
      // num_state++;
    }
    gridCache.returnItem(&s.grid);
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


    State start(startX, startY, 0, *gridCache.getItem(), *indexCache.getItem());
    start.grid.assign(grid.begin(), grid.end());
    start.need_update_index = state_p.board.need_update_index;

    for (int h = 0; h < state_p.board.rows; ++h) {
        for (int w = 0; w < state_p.board.cols; ++w) {
            //printf("%d ", grid[h * state_p.board.cols + w]);
            if(start.grid[h * state_p.board.cols + w] == 0){
              startX = h;
              startY = w;
              std::cout <<  "||" << h << ", " << w;
            }
           if(start.grid[h * state_p.board.cols + w] == 5){
             std::cout <<  "||" << h << ", " << w;
           }

            std::cout << kCellTypeToElement[start.grid[h * state_p.board.cols + w] + 1].id;
            // if(h==0) grid[h * state_p.board.cols + w] = 100;
        }
        std::cout << std::endl;
    }
    
    std::cout << goalX << ", " << goalY << " goal location\n";
    Location goal(goalX, goalY);
    bool success = false;

    // std::mt19937 g2; 
    // for (int x = 0; x < state_p.board.rows; x++) {rand_x.push_back(g2()); std::cout << rand_x[x] << ", ";} std::cout << "end " <<  std::endl; 
    // for (int y = 0; y < state_p.board.cols; y++) {rand_y.push_back(g2()); std::cout << rand_y[y] << ", ";} std::cout << std::endl;

    Environment env(state_p.board.rows, state_p.board.cols, goal);

    AStar<State, Action, int, Environment, vectorCache<int8_t>, vectorCache<int>> astar(env, gridCache, indexCache);

    PlanResult<State, Action, int> solution;
    Timer timer;

    if (env.stateValid(start)) {
      success = astar.search(start, solution);
      if(success){
        std::cout << "Pathfinding success !\n";
      }
    }
    timer.stop();
    std::cout << timer.elapsedSeconds() <<  ", " << env.num_expand <<std::endl;

    if (success) {
      std::cout << "Planning successful! Total cost: " << solution.cost << ", " << timer.elapsedSeconds() << ", " << env.num_expand
                << std::endl;
      for (size_t i = 0; i < solution.actions.size(); ++i) {
        std::cout << solution.states[i].second << ": " << solution.states[i].first
                  << "->" << solution.actions[i].first
                  << "(cost: " << solution.actions[i].second << ")" << std::endl;
      }
      std::cout << solution.states.back().second << ": "
                << solution.states.back().first << std::endl;

      // out << "schedule:" << std::endl;
      // out << "  agent1:" << std::endl;
      // for (size_t i = 0; i < solution.states.size(); ++i) {
      //   out << "    - x: " << solution.states[i].first.x << std::endl
      //       << "      y: " << solution.states[i].first.y << std::endl
      //       << "      t: " << i << std::endl;
      // }
    } else {
      std::cout << "Planning NOT successful!" << std::endl;
    }

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