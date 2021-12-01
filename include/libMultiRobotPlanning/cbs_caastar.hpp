
#pragma once

#include <map>
#include <time.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/resource.h>


#include "a_star.hpp"
#include "sipp.hpp"
#include "JPSSIPPCBS.hpp"
#include "JPSSIPP_BITCBS.hpp"
#include "planresult.hpp"
#include "timer.hpp"
#include "canonical_astar.hpp"

namespace libMultiRobotPlanning {

/*!
  \example cbs.cpp Example that solves the Multi-Agent Path-Finding (MAPF)
  problem in a 2D grid world with up/down/left/right
  actions
*/

/*! \brief Conflict-Based-Search (CBS) algorithm to solve the Multi-Agent
Path-Finding (MAPF) problem

This class implements the Conflict-Based-Search (CBS) algorithm.
This algorithm can find collision-free path for multiple agents with start and
goal locations
given for each agent.
CBS is a two-level search. On the low-level, A* is used to find paths for
individual agents (ideally using a perfect heuristic).
The high-level is a tree-search that resolves conflicts between agents as they
occur, earliest conflict-time first.
CBS is optimal with respect to the sum-of-individual costs.

Details of the algorithm can be found in the following paper:\n
Guni Sharon, Roni Stern, Ariel Felner, Nathan R. Sturtevant:\n
"Conflict-based search for optimal multi-agent pathfinding". Artif. Intell. 219:
40-66 (2015)\n
https://doi.org/10.1016/j.artint.2014.11.006

The underlying A* can either use a fibonacci heap, or a d-ary heap.
The latter is the default. Define "USE_FIBONACCI_HEAP" to use the fibonacci heap
instead.

\tparam State Custom state for the search. Needs to be copy'able
\tparam Action Custom action for the search. Needs to be copy'able
\tparam Cost Custom Cost type (integer or floating point types)
\tparam Conflict Custom conflict description. A conflict needs to be able to be
transformed into a constraint.
\tparam Constraints Custom constraint description. The Environment needs to be
able to search on the low-level while taking the constraints into account.
\tparam Environment This class needs to provide the custom logic. In particular,
it needs to support the following functions:
  - `void setLowLevelContext(size_t agentIdx, const Constraints* constraints)`\n
    Set the current context to a particular agent with the given set of
constraints

  - `Cost admissibleHeuristic(const State& s)`\n
    Admissible heuristic. Needs to take current context into account.

  - `bool isSolution(const State& s)`\n
    Return true if the given state is a goal state for the current agent.

  - `void getNeighbors(const State& s, std::vector<Neighbor<State, Action, int>
>& neighbors)`\n
    Fill the list of neighboring state for the given state s and the current
agent.

  - `bool getFirstConflict(const std::vector<PlanResult<State, Action, int> >&
solution, Conflict& result)`\n
    Finds the first conflict for the given solution for each agent. Return true
if a conflict was found and false otherwise.

  - `void createConstraintsFromConflict(const Conflict& conflict,
std::map<size_t, Constraints>& constraints)`\n
    Create a list of constraints for the given conflict.

  - `void onExpandHighLevelNode(Cost cost)`\n
    This function is called on every high-level expansion and can be used for
statistical purposes.

  - `void onExpandLowLevelNode(const State& s, Cost fScore, Cost gScore)`\n
    This function is called on every low-level expansion and can be used for
statistical purposes.
*/
template <typename State, typename Location, typename Action, typename Cost, typename Conflict,
          typename Constraints, typename Environment>
class CBSCAstar {
 public:
  CBSCAstar(Environment& environment) : m_env(environment) {}

  bool search(const std::vector<State>& initialStates,
              std::vector<PlanResult<State, Action, Cost> >& solution) {
    HighLevelNode start;
    start.solution.resize(initialStates.size());
    start.constraints.resize(initialStates.size());
    start.cost = 0;
    start.id = 0;

    for (size_t i = 0; i < initialStates.size(); ++i) {
      // if (   i < solution.size()
      //     && solution[i].states.size() > 1) {
      //   start.solution[i] = solution[i];
      //   std::cout << "use existing solution for agent: " << i << std::endl;
      // } else {
      m_env.setLowLevelContext(i, &start.constraints[i]);
      canonical_astar can_astar(m_env, start.solution);
      bool success = can_astar.search(initialStates[i], start.solution[i]);
        
      if (!success) {
        return false;
      }
      // }
      start.cost += start.solution[i].cost;
    }


/*    std::vector<HighLevelNode> high_levle_node;
    std::ifstream file_res("lak303d-highnode-1.txt");
    high_levle_node.resize(1000);
    int high_node_num = 548;
    for(int h_node = 0; h_node < high_node_num; h_node++){
      int id_i, agent_id_i, cost_i;
      high_levle_node[h_node].solution.resize(initialStates.size());
      high_levle_node[h_node].constraints.resize(initialStates.size());
      std::string str1;
      file_res >> str1  >>id_i  >> str1 >> agent_id_i >> str1 >> cost_i;
      // std::cout << id_i << agent_id_i << cost_i;
      high_levle_node[h_node].id = id_i;
      high_levle_node[h_node].agent_id = agent_id_i;
      high_levle_node[h_node].cost = cost_i;
      for(int num_agent = 0; num_agent < initialStates.size(); num_agent++){
        std::string str;
        file_res >> str;
        int v_constraint_num = 0;
        int e_constraint_num = 0;
        file_res >> str >> v_constraint_num;
        // std::cout << str << v_constraint_num<< std::endl;
        for(int vv = 0; vv < v_constraint_num; vv++){
          Constraints c1;
          int vc_time, vc_x1, vc_y1;
          file_res >> vc_time >> vc_x1 >> vc_y1 >> str;
          m_env.createConstraintsFromV(vc_time, vc_x1, vc_y1, c1);
          // std::cout << vc_time << ", (" << vc_x1 << ", " << vc_y1 << ")"<< c1 << " Here \n";
          
          high_levle_node[h_node].constraints[num_agent].add(c1);
        }
        file_res >> str >> e_constraint_num;
        // std::cout << str << e_constraint_num<< std::endl;
        for(int ee = 0; ee < e_constraint_num; ee++){
          Constraints c1;
          int ec_time, ec_x1, ec_y1, ec_x2, ec_y2;
          file_res >> ec_time >> ec_x1 >> ec_y1 >> ec_x2 >> ec_y2 >> str;
          // std::cout << ec_time << ", (" << ec_x1 << ", " << ec_y1 << ")" << " (" << ec_x2 << ", " << ec_y2 << ")" << c1 << " Here \n";
          m_env.createConstraintsFromE(ec_time, ec_x1, ec_y1, ec_x2, ec_y2, c1);
          high_levle_node[h_node].constraints[num_agent].add(c1);          
        }
      }
    }

    Timer timerJpstbit;
    timerJpstbit.reset();
    for(int h_node = 0; h_node < high_node_num; h_node++){
        size_t i = high_levle_node[h_node].agent_id;

        if(i == -1) continue;
        bool is_first_constraint_v = true;
        bool is_first_constraint_e = true;
        m_env.resetTemporalObstacle();
        
        for(auto & constraint : high_levle_node[h_node].constraints[i].vertexConstraints){
        	Location location(constraint.x, constraint.y);
        	m_env.setTemporalObstacle(location, constraint.time);
        }
        for(auto & constraint : high_levle_node[h_node].constraints[i].edgeConstraints){
        	Location location(constraint.x2, constraint.y2);
        	m_env.setTemporalEdgeConstraint(location, constraint.time);

        }

        Location goal = m_env.setGoal(i);
        m_env.Reset();
        Location startNode(-1, -1);
        startNode.x = initialStates[i].x;
        startNode.y = initialStates[i].y;
         m_env.setExactHeuristTrue();


        // LowLevelEnvironment llenv(m_env, i, high_levle_node[h_node].constraints[i]);
        // LowLevelSearch_t lowLevel(llenv);
        // PlanResult<State, Action, int> solutiontemp4;        
        // bool successTA = lowLevel.search(initialStates[i], solutiontemp4);
     

        m_env.setLowLevelContext(i, &high_levle_node[h_node].constraints[i]);
        PlanResult<State, Action, int> solutiontempCa;  
        canonical_astar can_astar(m_env);
        bool successCA = can_astar.search(initialStates[i], solutiontempCa);

        // if(solutiontemp4.cost != solutiontempCa.cost){
        //   std::cout << "Error " << " \n";
        //   return false;
        // }
// ?        std::cout << solutionSipp.cost << ", " << solutiontempJps.cost << " \n";
        
    }
    timerJpstbit.stop();
    double tSipp = timerJpstbit.elapsedSeconds();
    std::cout << "time " << tSipp << " \n";*/

    // std::priority_queue<HighLevelNode> open;
    struct rusage r_usage;
   	getrusage(RUSAGE_SELF, &r_usage);

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >
        open;
    // while(!start.conflicts_all.empty()) start.conflicts_all.pop();
    std::vector<Conflict> empty_1;
    start.conflicts_all.swap(empty_1);
    m_env.getAllConflicts(start.solution, start.conflicts_all, start.num_conflict);
    std::cout <<",Num conflict," << start.conflicts_all.size() << ",";    

    auto handle = open.push(start);
    (*handle).handle = handle;

    solution.clear();
    int id = 1;
    // clock_t startTime, endTime;
    // clock_t startTotal, endTotal;
    // startTotal = clock();

    Timer timer;
    timer.reset();
    int num_node = 0;
    int gen_node = 0;
    while (!open.empty()) {
      num_node++;
      timer.stop();
      double duration1 = timer.elapsedSeconds();
      if(duration1 > 300){
        std::cout << " ,done, time-out fail" << ", num_node,  " << num_node << " , gen_node, " << gen_node << ", " << " num_open, " << id << ", ";
    	  return false;
      }
      // if(num_node % 100 == 0){
      //   getrusage(RUSAGE_SELF, &r_usage);
      //   // std::cout << r_usage.ru_maxrss << " memory \n"; 
      //   if(r_usage.ru_maxrss > 15204352){
      //     std::cout << " ,done, memory-out fail" << ", num_node, " << num_node << " , gen_node, " << gen_node << ", " << " num_open, " << id << ", ";          
      //     return false;
      //   }
      // }      

      HighLevelNode P = open.top();
      m_env.onExpandHighLevelNode(P.cost);
      open.pop();
      Conflict conflict;
      if(P.conflicts_all.size() == 0){
        m_env.getAllConflicts(P.solution, P.conflicts_all, P.num_conflict);     
        if(P.num_conflict == 0){
          std::cout << ", done, " << P.cost << ", " << " num_node, " << num_node << ", " 
          << "gen_node, " << gen_node << ", " << " num_open, " << id << ", ";
          solution = P.solution;
          return true;
        }else{
          std::cout << " Final resulsts is not correct 1111\n";
          return false; 
        }
      }

      bool foundBypass = true;
      while(foundBypass){
        if(P.conflicts_all.size() == 0){
          m_env.getAllConflicts(P.solution, P.conflicts_all, P.num_conflict);     
          if(P.num_conflict == 0){
            std::cout << ", done, " << P.cost << ", " << " num_node, " << num_node << ", " 
            << "gen_node, " << gen_node << ", " << " num_open, " << id << ", ";
            solution = P.solution;
            return true;
          }else{
            std::cout << " Final resulsts is not correct 2222\n";
            return false;
          }
          // std::cout << ", done, " << P.cost << ", " << " num_node, " << num_node << ", " << "gen_node, " << gen_node << ", " << " num_open, " << id << ", ";
          // solution = P.solution;
          // return true;
        }

        // Conflict conflict_temp = P.conflicts_all.front();
        // P.conflicts_all.pop();
        if(P.conflicts_all.size() == 0) return true;
        // int random_index = rand()%P.conflicts_all.size();
        int random_index = 0;
        Conflict conflict_temp = P.conflicts_all[random_index];

        HighLevelNode NewChild[2];
        bool is_solved[2] = {false, false};
        std::map<size_t, Constraints> constraints;
        m_env.createConstraintsFromConflict(conflict_temp, constraints);
        int child_id = 0;
        foundBypass = false;
        for(const auto& c : constraints){
          size_t i = c.first;
          NewChild[child_id].solution = P.solution;
          NewChild[child_id].constraints = P.constraints;
          NewChild[child_id].cost = P.cost;
          NewChild[child_id].id = id;

          assert(!NewChild[child_id].constraints[i].overlap(c.second));
          NewChild[child_id].constraints[i].add(c.second);
          NewChild[child_id].cost -= NewChild[child_id].solution[i].cost;
          m_env.resetTemporalObstacle();
          for(auto & constraint : NewChild[child_id].constraints[i].vertexConstraints){
        	  Location location(constraint.x, constraint.y);
        	  m_env.setTemporalObstacle(location, constraint.time);
          }
          for(auto & constraint : NewChild[child_id].constraints[i].edgeConstraints){
        	  Location loc(constraint.x2, constraint.y2);
        	  m_env.setTemporalEdgeConstraint(loc, constraint.time);
          }
          m_env.Reset();
          m_env.setExactHeuristTrue();

          m_env.setLowLevelContext(i, &NewChild[child_id].constraints[i]);        
          canonical_astar can_astar(m_env, NewChild[child_id].solution);
          Timer timerCAstar;
          timerCAstar.reset();
          is_solved[child_id] = can_astar.search(initialStates[i], NewChild[child_id].solution[i]);
          timerCAstar.stop();
          double tCAstar = timerCAstar.elapsedSeconds();

          if(!is_solved[child_id]) continue;
          // while(!NewChild[child_id].conflicts_all.empty()) NewChild[child_id].conflicts_all.pop();
          if(!NewChild[child_id].conflicts_all.empty()) {
            NewChild[child_id].conflicts_all.clear();
            NewChild[child_id].conflicts_all.swap(empty_1);
          }
          m_env.getAllConflicts(NewChild[child_id].solution, NewChild[child_id].conflicts_all, NewChild[child_id].num_conflict);
          gen_node++;
          if(m_env.isBP && NewChild[child_id].solution[i].cost == P.solution[i].cost 
             && NewChild[child_id].num_conflict < P.num_conflict){
            foundBypass = true;
            P.solution[i] = NewChild[child_id].solution[i];
            P.num_conflict = NewChild[child_id].num_conflict;
            std::vector<Conflict>().swap(P.conflicts_all);            
            // while(!P.conflicts_all.empty()) P.conflicts_all.pop();
            // P.conflicts_all.clear();
            // P.conflicts_all.swap(empty_1);
            P.conflicts_all = NewChild[child_id].conflicts_all;
            break;
          }
          NewChild[child_id].cost += NewChild[child_id].solution[i].cost;
          child_id++;
        }

        if(!foundBypass){
          for(int ii = 0; ii < 2; ii++){
            if(is_solved[ii]){
              NewChild[ii].id = id;
              auto handle = open.push(NewChild[ii]);
              (*handle).handle = handle;
              id++;
            }
          }
        }

      }
      continue;  

      std::map<size_t, Constraints> constraints;
      m_env.createConstraintsFromConflict(conflict, constraints);
      for (const auto& c : constraints) {
        // std::cout << "Add HL node for " << c.first << std::endl;
        size_t i = c.first;
        HighLevelNode newNode = P;
        newNode.id = id;
        // (optional) check that this constraint was not included already
        // std::cout << newNode.constraints[i] << std::endl;
        // std::cout << c.second << std::endl;
        assert(!newNode.constraints[i].overlap(c.second));
        newNode.constraints[i].add(c.second);
        newNode.cost -= newNode.solution[i].cost;
        m_env.resetTemporalObstacle();

        for(auto & constraint : newNode.constraints[i].vertexConstraints){
        	Location location(constraint.x, constraint.y);
        	m_env.setTemporalObstacle(location, constraint.time);
        }

        for(auto & constraint : newNode.constraints[i].edgeConstraints){
        	Location loc(constraint.x2, constraint.y2);
        	m_env.setTemporalEdgeConstraint(loc, constraint.time);
        }

        
        m_env.setExactHeuristTrue();
        m_env.setLowLevelContext(i, &newNode.constraints[i]);        
        canonical_astar can_astar(m_env, newNode.solution);
        Timer timerCAstar;
        timerCAstar.reset();
        bool successCA = can_astar.search(initialStates[i], newNode.solution[i]);
        timerCAstar.stop();
        double tCAstar = timerCAstar.elapsedSeconds();
        newNode.cost += newNode.solution[i].cost;
/*        std::cout successCA<< i << ", Start, (" << initialStates[i].x << " " << initialStates[i].y <<
        		"), Goal, (" << goal.x << " " << goal.y <<
				"), Cost jps , " << solutiontemp.cost << " , VertexConstraint ," << newNode.constraints[i].vertexConstraints.size() <<
				", EdgeConstraint , " << newNode.constraints[i].edgeConstraints.size() <<
				", Time , " << tAstar << " , " << tSipp << " , " << tJps << " , " << tSippM << " , " << tJpsM <<
				", Exp , " << ExpAstar << " , " << ExpSipp << " , " << ExpJps << " , " << ExpSippM << " , " << ExpJpsM <<
				", Gen , " << GenAstar << " , " << GenSipp << " , " << GenJps <<  " , " << GenSippM << " , " << GenJpsM <<
				" \n";*/

                // std::cout << i << ", Start, (" << initialStates[i].x << " " << initialStates[i].y <<
                // 		"), Goal, (" << goal.x << " " << goal.y <<
        				// "), Cost jps , " << solutiontemp.cost << " , VertexConstraint ," << newNode.constraints[i].vertexConstraints.size() <<
        				// ", EdgeConstraint , " << newNode.constraints[i].edgeConstraints.size() <<
                // ", preTime, " << m_env.getPreTime(i) << 
        				// ", Time , " << tAstar << " , " << tSipp << " , " << tJps << ", " << tJpstbit << ", " << tCAstar <<
        				// ", Exp , " << ExpAstar << " , " << ExpSipp << " , " << ExpJps <<
        				// ", Gen , " << GenAstar << " , " << GenSipp << " , " << GenJps <<
        				// " \n";

        if (successCA) {
            
          auto handle = open.push(newNode);
          (*handle).handle = handle;
          gen_node++;
        }

        ++id;
      }
    }

    return false;
  }

 private:
  struct HighLevelNode {
    std::vector<PlanResult<State, Action, Cost> > solution;
    std::vector<Constraints> constraints;

    Cost cost;

    int id;

    int agent_id = -1;
    int num_conflict = 0;
    std::vector<Conflict> conflicts_all;
    // Conflict first_conflict;    

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >::handle_type
        handle;

    bool operator<(const HighLevelNode& n) const {
      // if (cost != n.cost)
      return cost > n.cost;
      // return id > n.id;
    }

    friend std::ostream& operator<<(std::ostream& os, const HighLevelNode& c) {
      os << "id: " << c.id << " cost: " << c.cost << std::endl;
      for (size_t i = 0; i < c.solution.size(); ++i) {
        os << "Agent: " << i << std::endl;
        os << " States:" << std::endl;
        for (size_t t = 0; t < c.solution[i].states.size(); ++t) {
          os << "  " << c.solution[i].states[t].first << std::endl;
        }
        os << " Constraints:" << std::endl;
        os << c.constraints[i];
        os << " cost: " << c.solution[i].cost << std::endl;
      }
      return os;
    }
  };

  struct LowLevelEnvironment {
    LowLevelEnvironment(Environment& env, std::vector<PlanResult<State, Action, Cost>>& cat,
                        size_t agentIdx, const Constraints& constraints)
        : m_env(env), m_cat(cat)
    // , m_agentIdx(agentIdx)
    // , m_constraints(constraints)
    {
      m_env.setLowLevelContext(agentIdx, &constraints);
    }

    Cost admissibleHeuristic(const State& s) {
      return m_env.admissibleHeuristic(s);
    }

    bool isSolution(const State& s) { return m_env.isSolution(s); }

    void getNeighbors(const State& s,
                      std::vector<Neighbor<State, Action, Cost> >& neighbors) {
      m_env.getNeighbors(s, neighbors);
      // std::cout << "current state xy " << s.x << ", " << s.y << " " << s.time << "------------------" << std::endl;
      if(m_env.isCAT){
      for(size_t nei = 0; nei < neighbors.size(); nei++){
        neighbors[nei].state.nc_cat = 0;
        int current_time = s.time + 1;
        State temp_s(-1, -1, -1), temp_s_p(-1, -1, -1);
        for(size_t agent_id = 0; agent_id < m_cat.size(); agent_id++){
          if(m_cat[agent_id].states.empty()) continue;
          if(m_env.getAgentId() == agent_id) continue;
          if (current_time < m_cat[agent_id].states.size()) {
            temp_s = m_cat[agent_id].states[current_time].first;
          }else{
            temp_s = m_cat[agent_id].states.back().first;     
          }
          if(temp_s == neighbors[nei].state){
            neighbors[nei].state.nc_cat++;
          }
          if(current_time - 1 >= 0){
            if(current_time - 1 < m_cat[agent_id].states.size()){
              temp_s_p = m_cat[agent_id].states[current_time - 1].first;
              if(temp_s_p.x == neighbors[nei].state.x && temp_s_p.y == neighbors[nei].state.y
                 && temp_s.x == s.x && temp_s.y == s.y){
                 neighbors[nei].state.nc_cat++;
               }
             }
          }
        }
      }
      }
    }

    void onExpandNode(const State& s, Cost fScore, Cost gScore) {
      // std::cout << "LL expand: " << s << std::endl;
      m_env.onExpandLowLevelNode(s, fScore, gScore);
    }

    void onDiscover(const State& /*s*/, Cost /*fScore*/, Cost /*gScore*/) {
      // std::cout << "LL discover: " << s << std::endl;
      // m_env.onDiscoverLowLevel(s, m_agentIdx, m_constraints);
    }

   private:
    Environment& m_env; 
    std::vector<PlanResult<State, Action, Cost>>& m_cat;
    // size_t m_agentIdx;
    // const Constraints& m_constraints;
  };

 private:
  Environment& m_env;
  typedef AStar<State, Action, Cost, LowLevelEnvironment> LowLevelSearch_t;
  typedef JPSSIPP<Location, Location, Action, int, Environment> jps_sipp;
  typedef SIPP<Location, Location, Action, int, Environment> sipp_t;
  typedef JPSSIPP_BITCBS<Location, Location, Action, int, Environment> jpst_bit;
  typedef CANAstar<State, Location, Action, Cost, Environment> canonical_astar;
//  jps_sipp.setEdgeCollisionSize(10, 10);
//  std::map<Location, std::vector<jps_sipp::interval>> allCollisionIntervals;
//  std::map<Location, std::vector<JPSSIPP::edgeCollision>> allEdgeCollisions;

};

}  // namespace libMultiRobotPlanning
