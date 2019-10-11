#include <fstream>
#include <iostream>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include <libMultiRobotPlanning/JPSSIPP.hpp>
#include <libMultiRobotPlanning/JPSSIPPAN.hpp>
#include <libMultiRobotPlanning/sipp.hpp>
#include "timer.hpp"

using libMultiRobotPlanning::JPSSIPP;
using libMultiRobotPlanning::JPSSIPPAN;
using libMultiRobotPlanning::SIPP;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;


enum class Action {
  Up  ,
  Down  ,
  Left  ,
  Right ,
  Wait  ,
  All   ,
};

struct State {
  State(int x, int y) : x(x), y(y){}

  bool operator==(const State& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  bool operator!=(const State& other) const {
    return std::tie(x, y) != std::tie(other.x, other.y);
  }

  bool operator<(const State& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << "(" << s.x << "," << s.y << ")";
  }

  int x;
  int y;
//  Action a;
};



namespace std {
template <>
struct hash<State> {
  size_t operator()(const State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

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
    case Action::All:
    	os << "All";
  }
  return os;
}

class Environment {
 public:
  Environment(size_t dimx, size_t dimy, std::unordered_set<State> obstacles, std::unordered_set<State> m_jumppoint,
              std::vector<std::vector<bool>>jump_point_map, State goal)
      : m_dimx(dimx),
        m_dimy(dimy),
        m_obstacles(std::move(obstacles)),
		m_jumppoint(std::move(m_jumppoint)),
		jump_point_map(std::move(jump_point_map)),
        m_goal(goal) {}

  float admissibleHeuristic(const State& s) {
    return std::abs(s.x - m_goal.x) + std::abs(s.y - m_goal.y);
  }

  bool isSolution(const State& s) { return s == m_goal; }

  State getLocation(const State& s) { return s; }

  void getNeighbors(const State& s,
                    std::vector<Neighbor<State, Action, int> >& neighbors) {
    neighbors.clear();

    State left(s.x - 1, s.y);
    if (stateValid(left)) {
      neighbors.emplace_back(
          Neighbor<State, Action, int>(left, Action::Left, 1));
    }
    State right(s.x + 1, s.y);
    if (stateValid(right)) {
      neighbors.emplace_back(
          Neighbor<State, Action, int>(right, Action::Right, 1));
    }

    State up(s.x, s.y + 1);
    if (stateValid(up)) {
      neighbors.emplace_back(Neighbor<State, Action, int>(up, Action::Up, 1));
    }
    State down(s.x, s.y - 1);
    if (stateValid(down)) {
      neighbors.emplace_back(
          Neighbor<State, Action, int>(down, Action::Down, 1));
    }
  }

  void onExpandNode(const State& /*s*/, int /*fScore*/, int /*gScore*/) {
    // std::cout << "expand: " << s << "g: " << gScore << std::endl;
  }

  void onDiscover(const State& /*s*/, int /*fScore*/, int /*gScore*/) {
    // std::cout << "  discover: " << s << std::endl;
  }

  bool isCommandValid(
      const State& /*s1*/, const State& /*s2*/, const Action& /*a*/,
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
      const State& /*s1*/, const State& /*s2*/, const Action& /*a*/,
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

 public:
  bool stateValid(const State& s) {
    return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
           m_obstacles.find(s) == m_obstacles.end();
  }

  bool isObstacle(const State& s){
	  return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
	           m_obstacles.find(s) != m_obstacles.end();
  }

  bool isJumpPoint(const State& s) {
	  return jump_point_map[s.x][s.y];
  }

  void setJumpPoint(const State& s){
	  jump_point_map[s.x][s.y] = true;
	  m_jumppoint.insert(s);
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
 public:
  int num_generation = 0;
  int num_expansion = 0;
  int limit_jump = 5;
  std::vector<std::vector<bool>> jump_point_map;
  bool is_limit = false;

 private:
  int m_dimx;
  int m_dimy;
  std::unordered_set<State> m_obstacles;
  std::unordered_set<State> m_jumppoint;
  State m_goal;
  bool is_jps = true;
};

int main(int argc, char* argv[]) {
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  std::string res;
  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input file (YAML)")("output,o",
                           po::value<std::string>(&outputFile)->required(),
                           "output file (YAML)")
						   //("results, txt", po::value<std::string>(&res)->required(), "results file (TXT)")
      // ("url",
      // po::value<std::string>(&url)->default_value("http://0.0.0.0:8080"),
      // "server URL")
      ;

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

  std::fstream res_sta("Berlin_all_17", std::ios::app);
  std::fstream res_Good("Berlin_all_good_17", std::ios::app);


  // Configure SIPP based on config file
  YAML::Node config = YAML::LoadFile(inputFile);

  std::unordered_set<State> obstacles;
  std::unordered_set<State> jumppoint;
  std::vector<State> goals;
  std::vector<State> startStates;
  std::vector<std::vector<bool>> map_1;

  const auto& dim = config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  for (const auto& node : config["map"]["obstacles"]) {
     obstacles.insert(State(node[0].as<int>(), node[1].as<int>()));
  }

  map_1.resize(dimx + 2);
  for(int i = 0; i < map_1.size();i++){
	  map_1[i].resize(dimy + 2);
  }

  for (const auto& ob:obstacles){
	  State temp1 = ob,temp2 = ob;
	  temp1.x = ob.x + 1;
	  temp2.y = ob.y + 1;

	  if(temp1.x >= 0 && temp1.x <= dimx && temp2.y >=0 && temp2.y <= dimy
			  && obstacles.find(temp1)==obstacles.end() && obstacles.find(temp2) == obstacles.end()){
		  jumppoint.insert(State(ob.x + 1, ob.y + 1));
		  map_1[ob.x + 1][ob.y + 1] = true;
	  }

	  temp1 = ob; temp2 = ob;
	  temp1.x = ob.x + 1;
	  temp2.y = ob.y - 1;
	  if(temp1.x >= 0 && temp1.x <= dimx && temp2.y >=0 && temp2.y <= dimy
			  && obstacles.find(temp1)==obstacles.end() && obstacles.find(temp2) == obstacles.end()){
		  jumppoint.insert(State(ob.x + 1, ob.y - 1));
		  map_1[ob.x + 1][ob.y - 1] = true;
	  }

	  temp1 = ob; temp2 = ob;
	  temp1.x = ob.x - 1;
	  temp2.y = ob.y + 1;
	  if(temp1.x >= 0 && temp1.x <= dimx && temp2.y >=0 && temp2.y <= dimy
			  && obstacles.find(temp1)==obstacles.end() && obstacles.find(temp2) == obstacles.end()){
		  jumppoint.insert(State(ob.x - 1, ob.y + 1));
		  map_1[ob.x - 1][ob.y + 1] = true;
	  }

	  temp1 = ob; temp2 = ob;
	  temp1.x = ob.x - 1;
	  temp2.y = ob.y - 1;
	  if(temp1.x >= 0 && temp1.x <= dimx && temp2.y >=0 && temp2.y <= dimy
			  && obstacles.find(temp1)==obstacles.end() && obstacles.find(temp2) == obstacles.end()){
		  jumppoint.insert(State(ob.x - 1, ob.y - 1));
		  map_1[ob.x - 1][ob.y - 1] = true;
	  }

  }

  for (const auto& node : config["agents"]) {
	  const auto& start = node["start"];
	  const auto& goal = node["goal"];
	  startStates.emplace_back(State(start[0].as<int>(), start[1].as<int>()));
	  goals.emplace_back(State(goal[0].as<int>(), goal[1].as<int>()));
  }
  typedef JPSSIPP<State, State, Action, int, Environment> jps_sipp;
  typedef JPSSIPPAN<State, State, Action, int, Environment> jps_an_sipp;
  typedef SIPP<State, State, Action, int, Environment> sipp_t;

  std::ofstream out(outputFile);
  out << "schedule:" << std::endl;

  // Plan (sequentially)
  std::map<State, std::vector<jps_sipp::interval>> allCollisionIntervals;
  std::map<State, std::vector<jps_sipp::edgeCollision>> allEdgeCollisions;

  std::map<State, std::vector<jps_an_sipp::interval>> allCollisionIntervals_jpsan;
  std::map<State, std::vector<jps_an_sipp::edgeCollision>> allEdgeCollisions_jpsan;

  std::map<State, std::vector<sipp_t::interval>> allCollisionIntervals_sipp;
  std::map<State, std::vector<sipp_t::edgeCollision>> allEdgeCollisions_sipp;

//  allCollisionIntervals[State(1,1)].push_back(jps_sipp::interval(5, 5));
//  allCollisionIntervals[State(1,1)].push_back(jps_sipp::interval(10, 15));
//  allCollisionIntervals[State(2,1)].push_back(jps_sipp::interval(10, 16));

//  allCollisionIntervals_sipp[State(1,1)].push_back(sipp_t::interval(5, 5));
//  allCollisionIntervals_sipp[State(1,1)].push_back(sipp_t::interval(10, 15));
//  allCollisionIntervals_sipp[State(2,1)].push_back(sipp_t::interval(10, 16));


  long cost = 0;
  for (size_t i = 0; i < goals.size(); ++i) {
    std::cout << "Planning for agent " << i << std::endl;
    out << "  agent" << i << ":" << std::endl;

    Environment env(dimx, dimy, obstacles, jumppoint, map_1, goals[i]);
    jps_sipp jpssipp(env);
    jps_an_sipp jps_an_sipp(env);
    sipp_t sipp(env);

    for (const auto& collisionIntervals : allCollisionIntervals) {
      jpssipp.setCollisionIntervals(collisionIntervals.first, collisionIntervals.second);
      State current_state = collisionIntervals.first;
/*      if(env.stateValid(State(current_state.x + 1, current_state.y))){
    	  env.setJumpPoint(State(current_state.x + 1, current_state.y));
      }

      if(env.stateValid(State(current_state.x - 1, current_state.y))){
    	  env.setJumpPoint(State(current_state.x - 1, current_state.y));
      }

      if(env.stateValid(State(current_state.x, current_state.y - 1))){
    	  env.setJumpPoint(State(current_state.x, current_state.y - 1));
      }

      if(env.stateValid(State(current_state.x, current_state.y + 1))){
    	  env.setJumpPoint(State(current_state.x, current_state.y + 1));
      }*/

      if(env.stateValid(State(current_state.x + 1, current_state.y - 1))){
    	  env.setJumpPoint(State(current_state.x + 1, current_state.y - 1));
      }

      if(env.stateValid(State(current_state.x - 1, current_state.y - 1))){
    	  env.setJumpPoint(State(current_state.x - 1, current_state.y - 1));
      }

      if(env.stateValid(State(current_state.x + 1, current_state.y + 1))){
    	  env.setJumpPoint(State(current_state.x + 1, current_state.y + 1));
      }

      if(env.stateValid(State(current_state.x - 1, current_state.y + 1))){
    	  env.setJumpPoint(State(current_state.x - 1, current_state.y + 1));
      }
    }


    for (const auto& ec: allEdgeCollisions) {
    	jpssipp.setEdgeCollisions(ec.first, ec.second);
    }

    for (const auto& collisionIntervals_jpsan : allCollisionIntervals_jpsan) {
       jps_an_sipp.setCollisionIntervals(collisionIntervals_jpsan.first, collisionIntervals_jpsan.second);
    }
    for (const auto& ec: allEdgeCollisions_jpsan) {
    	jps_an_sipp.setEdgeCollisions(ec.first, ec.second);
    }
    for (const auto& collisionIntervals_sipp : allCollisionIntervals_sipp) {
       sipp.setCollisionIntervals(collisionIntervals_sipp.first, collisionIntervals_sipp.second);
    }
    for (const auto& ec: allEdgeCollisions_sipp) {
    	sipp.setEdgeCollisions(ec.first, ec.second);
    }


    env.Reset();
    // Plan
    PlanResult<State, Action, int> solution;
    Timer t;
    t.reset();
    bool success = jpssipp.search(startStates[i], Action::Wait, solution);
    t.stop();
    std::cout<< t.elapsedSeconds() << std::endl;
    int num_expansion1 = env.num_expansion;
    int num_generation1 = env. num_generation;
    double time1 = t.elapsedSeconds();
    if (success) {
      std::cout << "JPS Planning successful! Total cost: " << solution.cost << " Expansion:"
    		    << env.num_expansion << " Generation: " << env.num_generation
                << std::endl;

      // print solution
      for (size_t i = 0; i < solution.actions.size(); ++i) {
        std::cout << solution.states[i].second << ": " << solution.states[i].first
                  << "->" << solution.actions[i].first
                  << "(cost: " << solution.actions[i].second << ")" << std::endl;
      }
      std::cout << solution.states.back().second << ": "
                << solution.states.back().first << std::endl;

      for (size_t i = 0; i < solution.states.size(); ++i) {
        out << "    - x: " << solution.states[i].first.x << std::endl
            << "      y: " << solution.states[i].first.y << std::endl
            << "      t: " << solution.states[i].second << std::endl;
      }
    } else {
      std::cout << "Planning NOT successful!" << std::endl;
      out << "    []" << std::endl;
    }


    env.Reset();
    // Plan
    PlanResult<State, Action, int> solution3;
    t.reset();
    bool success3 = jps_an_sipp.search(startStates[i], Action::Wait, solution3);
    t.stop();
    std::cout<< t.elapsedSeconds() << std::endl;
    int num_expansion3 = env.num_expansion;
    int num_generation3 = env. num_generation;
    double time3 = t.elapsedSeconds();
/*    if (success3) {
      std::cout << "JPSAN Planning successful! Total cost: " << solution3.cost << " Expansion:"
    		    << env.num_expansion << " Generation: " << env.num_generation
                << std::endl;

      // print solution
      for (size_t i = 0; i < solution3.actions.size(); ++i) {
        std::cout << solution3.states[i].second << ": " << solution3.states[i].first
                  << "->" << solution3.actions[i].first
                  << "(cost: " << solution3.actions[i].second << ")" << std::endl;
      }
      std::cout << solution3.states.back().second << ": "
                << solution3.states.back().first << std::endl;

      for (size_t i = 0; i < solution3.states.size(); ++i) {
        out << "    - x: " << solution3.states[i].first.x << std::endl
            << "      y: " << solution3.states[i].first.y << std::endl
            << "      t: " << solution3.states[i].second << std::endl;
      }
    } else {
      std::cout << "Planning NOT successful!" << std::endl;
      out << "    []" << std::endl;
    }
*/

    env.Reset();
    env.setNoJPS();
    PlanResult<State, Action, int> solution2;
    t.reset();
    bool success_temp = sipp.search(startStates[i], Action::Wait, solution2);
    t.stop();

     int num_expansion2 = env.num_expansion;
     int num_generation2 = env.num_generation;

     double time2 = t.elapsedSeconds();
     std::cout<< t.elapsedSeconds() << std::endl;

    if (success_temp) {
      std::cout << "NoJPS Planning successful! Total cost: " << solution2.cost << " Expansion:"
    		    << env.num_expansion << " Generation: " << env.num_generation
                << std::endl;

      // update collision intervals
      auto lastState = solution2.states[0];
      for (size_t i = 1; i < solution2.states.size(); ++i) {
        if (solution2.states[i].first != lastState.first) {
          allCollisionIntervals[lastState.first].push_back(
            jps_sipp::interval(lastState.second, solution2.states[i].second - 1));
          allCollisionIntervals_sipp[lastState.first].push_back(
            sipp_t::interval(lastState.second, solution2.states[i].second - 1));
          allCollisionIntervals_jpsan[lastState.first].push_back(
            jps_an_sipp::interval(lastState.second, solution2.states[i].second - 1));

          lastState = solution2.states[i];
        }
      }
      allCollisionIntervals[solution2.states.back().first].push_back(
            jps_sipp::interval(solution2.states.back().second, std::numeric_limits<int>::max()));
      allCollisionIntervals_sipp[solution2.states.back().first].push_back(
            sipp_t::interval(solution2.states.back().second, std::numeric_limits<int>::max()));

      allCollisionIntervals_jpsan[solution2.states.back().first].push_back(
            jps_an_sipp::interval(solution2.states.back().second, std::numeric_limits<int>::max()));
//      std::cout << ""
      // update statistics
      cost += solution2.cost;
      // print solution
      for (size_t i = 0; i < solution2.actions.size(); ++i) {
    	  if(solution2.actions[i].first != Action::Wait ){
    		  allEdgeCollisions[solution2.states[i].first].push_back(jps_sipp::edgeCollision(solution2.states[i].second,solution2.actions[i].first));
    	  }
    	  if(solution2.actions[i].first != Action::Wait ){
    	      allEdgeCollisions_sipp[solution2.states[i].first].push_back(sipp_t::edgeCollision(solution2.states[i].second,solution2.actions[i].first));
    	   }

    	  if(solution2.actions[i].first != Action::Wait ){
    	      allEdgeCollisions_jpsan[solution2.states[i].first].push_back(jps_an_sipp::edgeCollision(solution2.states[i].second,solution2.actions[i].first));
    	   }
        std::cout << solution2.states[i].second << ": " << solution2.states[i].first
                  << "->" << solution2.actions[i].first
                  << "(cost: " << solution2.actions[i].second << ")" << std::endl;
      }
      std::cout << solution2.states.back().second << ": "
                << solution2.states.back().first << std::endl;

      for (size_t i = 0; i < solution2.states.size(); ++i) {
        out << "    - x: " << solution2.states[i].first.x << std::endl
            << "      y: " << solution2.states[i].first.y << std::endl
            << "      t: " << solution2.states[i].second << std::endl;
      }
    } else {
      std::cout << inputFile  <<  " SIPP Planning NOT successful from Agent: " << i << std::endl;
      out << "    []" << std::endl;
      break;
    }


    if(solution.cost != solution2.cost){
    	std::cout << inputFile <<  "Agent " << i << ": Not equal1" << std::endl;
    	break;
    }

    if(solution.cost != solution3.cost){
    	std::cout << inputFile <<  "Agent " << i << ": Not equal" << std::endl;
    	break;
    }

    if(num_expansion1 > num_expansion2){
    	std::cout << inputFile << " Not Expansion-1 Agent " << i << " "<< num_expansion1 - num_expansion2 << "\n";
    }

    if(num_expansion3 > num_expansion2){
    	std::cout << inputFile << " Not Expansion-3 Agent " << i << " "<< num_expansion3 - num_expansion2 << "\n";
    }

//    if(num_generation1 - num_generation3 > 20){
//    	std::cout << inputFile << " Not Generation-1 Agent " << i << " "<< num_generation1 - num_generation3 << "\n";
//    }

/*    if(num_expansion1 > num_expansion3){
    	std::cout << inputFile << " Not Expansion-3 Agent " << i << " "<< num_expansion1 - num_expansion3 << "\n";
    }


    if(num_expansion1 < num_expansion3){
    	std::cout << inputFile << " Not Expansion-4 Agent " << i << " "<< num_expansion1 - num_expansion3 << "\n";
    }


    if(num_generation1 > num_generation2){
    	std::cout << inputFile << " Not Generation-1 Agent " << i << " "<< num_generation1 - num_generation2 << "\n";
    }
*/
    res_sta << inputFile << " Agent " << i << " JPSSIPP: " << " cost: " << solution.cost << " " << time1 << " " << num_expansion1 << " " << num_generation1 <<"\n";
    res_sta << inputFile << " Agent " << i << " JPSSIPPAN: " << " cost: " << solution3.cost << " " << time3 << " " << num_expansion3 << " " << num_generation3 <<"\n";
    res_sta << inputFile << " Agent " << i << " SIPP: " << " cost: " << solution2.cost << " " <<time2 << " " << num_expansion2 << " " << num_generation2 <<"\n";

    if (time1 < time2){
        res_Good << inputFile << " Agent " << i << " JPSSIPP: "  <<" cost " << solution.cost << " " << time1 << " " << num_expansion1 << " " << num_generation1 <<"\n";
        res_Good << inputFile << " Agent " << i << " SIPP: "  <<" cost " << solution.cost << " "<< time2 << " " << num_expansion2 << " " << num_generation2 <<"\n";
    }
//   if(i == 49) break;
//    break;
  }

  out << "statistics:" << std::endl;
  out << "  cost: " << cost << std::endl;
  // out << "  makespan: " << makespan << std::endl;
  // out << "  runtime: " << timer.elapsedSeconds() << std::endl;




  // for (const auto& node : config["environment"]["collisionIntervals"]) {
  //   State state(node["location"][0].as<int>(), node["location"][1].as<int>());

  //   std::vector<jps_sipp::interval> collisionIntervals;

  //   for (const auto& interval : node["intervals"]) {
  //     collisionIntervals.emplace_back(
  //         jps_sipp::interval(interval[0].as<int>(), interval[1].as<int>()));
  //   }
  //   sipp.setCollisionIntervals(state, collisionIntervals);
  // }




  return 0;
}
