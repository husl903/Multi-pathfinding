
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
//using libMultiRobotPlanning::SIPP;
//using libMultiRobotPlanning::JPSSIPP;

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
class CBSAstar {
 public:
  CBSAstar(Environment& environment) : m_env(environment) {}

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
        // m_env.setLowLevelContext(i, &start.constraints[i]);
        // canonical_astar can_astar(m_env, start.solution);
        // bool success = can_astar.search(initialStates[i], start.solution[i]);
        
      LowLevelEnvironment llenv(m_env, start.solution, i, start.constraints[i]);
      LowLevelSearch_t lowLevel(llenv);
      bool success = lowLevel.search(initialStates[i], start.solution[i]);
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

    std::vector<Conflict> empty_1;
    start.conflicts_all.swap(empty_1);
    // while(!start.conflicts_all.empty()) start.conflicts_all.pop();
    m_env.getAllConflicts(start.solution, start.conflicts_all, start.num_conflict);

    std::cout <<" Num conflict " << start.conflicts_all.size() << std::endl;
    for(int iiii = 0; iiii < start.conflicts_all.size(); iiii++)
      std::cout << start.conflicts_all[iiii] << std::endl;
    solution = start.solution;
    return true;

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
        std::cout << " ,done, time-out fail" << ", num_node, " << num_node 
        << " , gen_node, " << gen_node << ", " << " num_open, " << id << ", ";
    	  return false;
      }
      if(num_node % 100 == 0){
        getrusage(RUSAGE_SELF, &r_usage);
        // std::cout << r_usage.ru_maxrss << " memory \n"; 
        if(r_usage.ru_maxrss > 15204352){
          std::cout << " ,done, memory-out fail" << ", num_node, " << num_node 
          << " , gen_node, " << gen_node << ", " << " num_open, " << id << ", ";          
          return false;
        }
      }

      HighLevelNode P = open.top();
      m_env.onExpandHighLevelNode(P.cost);
      open.pop();
      Conflict conflict;
//      if (!m_env.getFirstConflict(P.solution, conflict, P.num_conflict)) {
      if(P.conflicts_all.size() == 0){
        if(!m_env.getFirstConflict(P.solution, conflict)){
          std::cout << ", done, " << P.cost << ", " << " num_node, " << num_node << ", " 
          << "gen_node, " << gen_node << ", " << " num_open, " << id << ", ";
          solution = P.solution;
          return true;
        }else{
          std::cout << " Final resulsts is not correct\n";
          return false; 
        }
      }

      bool foundBypass = true;
      while(foundBypass){
        // std::cout << "Bypass \n";
        if(P.conflicts_all.size() == 0){
          std::cout << ", done," << P.cost << ", " << " num_node, " << num_node << ", " 
          << "gen_node, " << gen_node << ", " << " num_open, " << id << ", ";
          solution = P.solution;
          return true;
        }

        // while(!P.conflicts_all.empty()) P.conflicts_all.pop();
        // m_env.getAllConflicts(P.solution, P.conflicts_all, P.num_conflict);
        if(P.conflicts_all.size() == 0) return true;
        int random_index = rand()%P.conflicts_all.size();
        Conflict conflict_temp = P.conflicts_all[random_index];
        // Conflict conflict_temp = P.conflicts_all.front();
        // P.conflicts_all.pop();

        HighLevelNode NewChild[2];
        bool is_solved[2] = {false, false};
        std::map<size_t, Constraints> constraints;
        m_env.createConstraintsFromConflict(conflict_temp, constraints);
        int child_id = 0;
        foundBypass = false;
        for(const auto& c : constraints){
          size_t i = c.first;
          // NewChild[child_id] = P;
          NewChild[child_id].solution = P.solution;
          NewChild[child_id].constraints = P.constraints;
          NewChild[child_id].cost = P.cost;

          // NewChild[child_id].solution = P.solution;
          
          NewChild[child_id].id = id;
          // if(NewChild[child_id].constraints[i].overlap(c.second)){
          //   std::cout << " \n";
          //   for(auto &ct : NewChild[child_id].constraints[i].vertexConstraints){
          //     std::cout << ct << std::endl;
          //   }

          //   for(auto & constraint : NewChild[child_id].constraints[i].edgeConstraints){
          //     std::cout << constraint << std::endl;
          //   }
          // }
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
          m_env.setGoal(i);
          m_env.Reset();
          m_env.setExactHeuristTrue();
          LowLevelEnvironment llenv(m_env, NewChild[child_id].solution, i, NewChild[child_id].constraints[i]);
          LowLevelSearch_t lowLevel(llenv);

          Timer timerAstar;
          timerAstar.reset();
          int ExpA =  m_env.lowLevelExpanded();
          is_solved[child_id] = lowLevel.search(initialStates[i], NewChild[child_id].solution[i]);
          timerAstar.stop();
          int ExpAstar = m_env.lowLevelExpanded() - ExpA;
          int GenAstar = m_env.lowLevelGenerated();
          double tAstar = timerAstar.elapsedSeconds();
          
          if(!is_solved[child_id]) continue;

          if(!NewChild[child_id].conflicts_all.empty()) {
            NewChild[child_id].conflicts_all.clear();
            NewChild[child_id].conflicts_all.swap(empty_1);
          }          
          m_env.getAllConflicts(NewChild[child_id].solution, NewChild[child_id].conflicts_all, NewChild[child_id].num_conflict);
          gen_node++;
          if(m_env.isBP && NewChild[child_id].solution[i].cost == P.solution[i].cost 
             && NewChild[child_id].num_conflict < P.num_conflict){
            // std::cout << "Here \n";
            foundBypass = true;
            P.solution[i] = NewChild[child_id].solution[i];
            P.num_conflict = NewChild[child_id].num_conflict;
            P.conflicts_all.clear();
            P.conflicts_all.swap(empty_1);
            P.conflicts_all = NewChild[child_id].conflicts_all;
            break;
          }
          NewChild[child_id].cost += NewChild[child_id].solution[i].cost;

          child_id++;
        }

        if(!foundBypass){
          // std::cout << "--------------------------------------------------------------------\n";
          //     std::priority_queue<Conflict> temp = P.conflicts_all;
          //     while(!temp.empty()){
          //       Conflict tt = temp.top();
          //       temp.pop();
          //       std::cout << tt << " Paretnt " << std::endl;
          //     }
          for(int ii = 0; ii < 2; ii++){
            if(is_solved[ii]){
              NewChild[ii].id = id;
              auto handle = open.push(NewChild[ii]);

              // std::priority_queue<Conflict> temp1 = NewChild[ii].conflicts_all;
              // while(!temp1.empty()){
              //   Conflict tt = temp1.top();
              //   temp1.pop();
              //   std::cout << tt << " chilid " << ii << std::endl;
              // }

              (*handle).handle = handle;
              id++;
            }
          }
          // std::cout << "--------------------------------------------------------------------\n";

        }

      }
      continue;  

        // Conflict conflict;
        if (!m_env.getFirstConflict(P.solution, conflict)) {
          std::cout << ", done, cost, " << P.cost << ", " << " num_node, " << num_node << ", " 
          << "gen_node, " << gen_node << ", " << " num_open, " << id << ", ";
          solution = P.solution;
          return true;
        }

      // if(m_env.isBP){
      //   if(TryBypassAstar(conflict, P)){
      //     auto handle = open.push(P);
      //     (*handle).handle = handle;
      //       continue;
      //   }
      // }


      // create additional nodes to resolve conflict
      // std::cout << "Found conflict: " << conflict << std::endl;
      // std::cout << "Found conflict at t=" << conflict.time << " type: " <<
      // conflict.type << std::endl;

        // for(size_t jj = 0; jj < P.solution.size(); jj++){        
        // 		for (size_t ii = 0; ii < P.solution[jj].actions.size(); ++ii) {
        // 			std::cout << P.solution[jj].states[ii].second << ": " <<
        // 						P.solution[jj].states[ii].first << "->" << P.solution[jj].actions[ii].first
				// 				<< "(cost: " << P.solution[jj].actions[ii].second << ")" << std::endl;
        // 		}
        // 		std::cout << P.solution[jj].states.back().second << ": " <<
        // 		  		   P.solution[jj].states.back().first << std::endl;
        // }

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

        // jps_sipp jps(m_env);
        // jpst_bit jpstbit(m_env);
        // jps.setEdgeCollisionSize(m_env.m_dimx, m_env.m_dimy);
        // jpstbit.setEdgeCollisionSize(m_env.m_dimx, m_env.m_dimy);

        PlanResult<Location, Action, int> solutiontemp;
        // PlanResult<Location, Action, int> solutiontemp2;
        bool is_first_constraint_v = true;
        bool is_first_constraint_e = true;        
        for(auto & constraint : newNode.constraints[i].vertexConstraints){
        	Location location(constraint.x, constraint.y);
        	m_env.setTemporalObstacle(location, constraint.time);
          // std::cout << constraint << " \n";
        	// if(is_first_constraint_v){
        	// // 	jps.setCollisionVertex(location, constraint.time, constraint.time, true);
        	// 	jpstbit.setCollisionVertex(location, constraint.time, constraint.time, true);            
        	// 	is_first_constraint_v = false;
        	// }else{
        	// 	// jps.setCollisionVertex(location, constraint.time, constraint.time, false);
        	// 	jpstbit.setCollisionVertex(location, constraint.time, constraint.time, false);            
        	// }
        }

        for(auto & constraint : newNode.constraints[i].edgeConstraints){
        	Location loc(constraint.x2, constraint.y2);
        	m_env.setTemporalEdgeConstraint(loc, constraint.time);
          // std::cout << constraint << " \n";
        	// if(constraint.x1 == constraint.x2){
        	// 	if(constraint.y1 == constraint.y2 - 1){
        	// 		// jps.setEdgeConstraint(loc, constraint.time, Action::Down, is_first_constraint_e);
        	// 		jpstbit.setEdgeConstraint(loc, constraint.time, Action::Down, is_first_constraint_e);
        	// 	}else if(constraint.y1 == constraint.y2 + 1){
        	// 		// jps.setEdgeConstraint(loc, constraint.time, Action::Up, is_first_constraint_e);
        	// 		jpstbit.setEdgeConstraint(loc, constraint.time, Action::Up, is_first_constraint_e);
        	// 	}
        	// }else{
        	// 	if(constraint.x1 == constraint.x2 - 1){
        	// 		// jps.setEdgeConstraint(loc, constraint.time, Action::Left, is_first_constraint_e);
        	// 		jpstbit.setEdgeConstraint(loc, constraint.time, Action::Left, is_first_constraint_e);
        	// 	}else if(constraint.x1 == constraint.x2 + 1){
        	// 		// jps.setEdgeConstraint(loc, constraint.time, Action::Right, is_first_constraint_e);
        	// 		jpstbit.setEdgeConstraint(loc, constraint.time, Action::Right, is_first_constraint_e);
        	// 	}
        	// }
        	// if(is_first_constraint_e){
        	// 	is_first_constraint_e = false;
        	// }
        }
        // jps.sortCollisionVertex();
        // jps.sortCollisionEdgeConstraint();
        // jpstbit.sortCollisionVertex();
        // jpstbit.sortCollisionEdgeConstraint();

        Location goal = m_env.setGoal(i);
        m_env.Reset();
        Location startNode(-1, -1);
        startNode.x = initialStates[i].x;
        startNode.y = initialStates[i].y;

        // m_env.setExactHeuristTrue();
        // Timer timerJps;
        // timerJps.reset();
        // bool isJpsSucc = jps.search(startNode, Action::Wait, solutiontemp, 0, true);
        // timerJps.stop();
        // double tJps = timerJps.elapsedSeconds();
        // int ExpJps = m_env.num_expansion;
        // int GenJps = m_env.num_generation;

        Timer timerJpstbit;
        // timerJpstbit.reset();
        // m_env.setExactHeuristTrue();
        // bool isJpstbit = jpstbit.search(startNode, Action::Wait, solutiontemp, 0);
        // timerJpstbit.stop();
        // double tJpstbit = timerJpstbit.elapsedSeconds();
        // int ExpJps1 = m_env.num_expansion;
        // int GenJps1 = m_env.num_generation;
/*        m_env.setExactHeuristFalse();
        timerJps.reset();
        bool isJpsSuccM = jps.search(startNode, Action::Wait, solutiontemp, 0, true);
        timerJps.stop();
        double tJpsM = timerJps.elapsedSeconds();
        int ExpJpsM = m_env.num_expansion;
        int GenJpsM = m_env.num_generation;
*/
        // sipp_t sipp(m_env);
        // sipp.setEdgeCollisionSize(m_env.m_dimx, m_env.m_dimy);
        // PlanResult<Location, Action, int> solutionSipp;
        // is_first_constraint_v = true;
        // for(auto & constraint : newNode.constraints[i].vertexConstraints){
        // 	Location location(constraint.x, constraint.y);
        // 	// std::cout << " Vertex Constraint " << constraint.x <<  " " <<constraint.y << " " << constraint.time << " --\n";
        // 	if(is_first_constraint_v){
        // 		sipp.setCollisionVertex(location, constraint.time, constraint.time, true);
        // 		is_first_constraint_v = false;
        // 	}else{
        // 		sipp.setCollisionVertex(location, constraint.time, constraint.time, false);
        // 	}
        // }


        // is_first_constraint_e = true;
        // for(auto & constraint : newNode.constraints[i].edgeConstraints){
        // 	// std::cout << " Edge Constraint " << constraint.x1 << " " << constraint.y1 << " second " << constraint.x2 << " " <<constraint.y2 << " " << constraint.time << " --\n";
        // 	Location loc(constraint.x2, constraint.y2);
        // 	if(constraint.x1 == constraint.x2){
        // 		if(constraint.y1 == constraint.y2 - 1){
        // 			sipp.setEdgeConstraint(loc, constraint.time, Action::Down, is_first_constraint_e);
        // 		}else if(constraint.y1 == constraint.y2 + 1){
        // 			sipp.setEdgeConstraint(loc, constraint.time, Action::Up, is_first_constraint_e);
        // 		}
        // 	}else{
        // 		if(constraint.x1 == constraint.x2 - 1){
        // 			sipp.setEdgeConstraint(loc, constraint.time, Action::Left, is_first_constraint_e);
        // 		}else if(constraint.x1 == constraint.x2 + 1){
        // 			sipp.setEdgeConstraint(loc, constraint.time, Action::Right, is_first_constraint_e);
        // 		}
        // 	}

        // 	if(is_first_constraint_e){
        // 		is_first_constraint_e = false;
        // 	}
        // }

        // sipp.sortCollisionVertex();
        // sipp.sortCollisionEdgeConstraint();
        // goal = m_env.setGoal(i);
        // m_env.Reset();
        // startNode.x = initialStates[i].x;
        // startNode.y = initialStates[i].y;

        // Timer timerSipp;
        // timerSipp.reset();
        // m_env.setExactHeuristTrue();
        // bool isSippSucc = sipp.search(startNode, Action::Wait, solutionSipp, 0);
        // timerSipp.stop();
        // double tSipp = timerSipp.elapsedSeconds();
        // int ExpSipp = m_env.num_expansion;
        // int GenSipp = m_env.num_generation;

/*        timerSipp.reset();
        m_env.setExactHeuristFalse();
        bool isSippSuccM = sipp.search(startNode, Action::Wait, solutionSipp, 0);
        timerSipp.stop();
        double tSippM = timerSipp.elapsedSeconds();
        int ExpSippM = m_env.num_expansion;
        int GenSippM = m_env.num_generation;
*/
        // std::cout << "===================================================================\n";

        m_env.setExactHeuristTrue();
        LowLevelEnvironment llenv(m_env, newNode.solution, i, newNode.constraints[i]);
        LowLevelSearch_t lowLevel(llenv);

        Timer timerAstar;
        timerAstar.reset();
        int ExpA =  m_env.lowLevelExpanded();
        PlanResult<State, Action, int> solutiontemp4;        
        bool successA = lowLevel.search(initialStates[i], newNode.solution[i]);
        timerAstar.stop();
        int ExpAstar = m_env.lowLevelExpanded() - ExpA;
        int GenAstar = m_env.lowLevelGenerated();
        double tAstar = timerAstar.elapsedSeconds();

        // m_env.setExactHeuristTrue();
        // LowLevelEnvironment llenvP(m_env, i, newNode.constraints[i]);
        // LowLevelSearch_t lowLevelP(llenvP);

        // Timer timerAstarP;
        // timerAstarP.reset();
        // int ExpAP =  m_env.lowLevelExpanded();
        // bool successP = lowLevelP.search(initialStates[i], newNode.solution[i]);
        // timerAstarP.stop();
        // int ExpAstarP = m_env.lowLevelExpanded() - ExpAP;
        // int GenAstarP = m_env.lowLevelGenerated();
        // double tAstarP = timerAstarP.elapsedSeconds();
        
        // m_env.setExactHeuristTrue();
        // m_env.setLowLevelContext(i, &newNode.constraints[i]);        
        // canonical_astar can_astar(m_env, newNode.solution);
        // Timer timerCAstar;
        // timerCAstar.reset();
        // bool successCA = can_astar.search(initialStates[i], newNode.solution[i]);
        // timerCAstar.stop();
        // double tCAstar = timerCAstar.elapsedSeconds();
        newNode.cost += newNode.solution[i].cost;

        // std::cout << "Herehrere \n";
        // 		       for (size_t ii = 0; ii < newNode.solution[i].actions.size(); ++ii) {
        // 		         std::cout << newNode.solution[i].states[ii].second << ": " <<
        // 		        		 newNode.solution[i].states[ii].first << "->" << newNode.solution[i].actions[ii].first
        // 		         << "(cost: " << newNode.solution[i].actions[ii].second << ")" << std::endl;
        // 		       }
        // 		       std::cout << newNode.solution[i].states.back().second << ": " <<
        // 		    		   newNode.solution[i].states.back().first << std::endl;


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

        // if(sucessCA && success){
        //   if(solutiontemp4.cost != newNode.solution[i].cost){
        //     std::cout << "canonical astar is not equal " << solutiontemp4.cost << ", " << newNode.solution[i].cost << " \n";
        //     return false;
        //   }
        // }else if(!sucessCA && success){
        //   std::cout << " canonical astar is not equal 22\n";
        // 		for (size_t ii = 0; ii < newNode.solution[i].actions.size(); ++ii) {
        // 			std::cout << newNode.solution[i].states[ii].second << ": " <<
        // 						newNode.solution[i].states[ii].first << "->" << newNode.solution[i].actions[ii].first
				// 				<< "(cost: " << newNode.solution[i].actions[ii].second << ")" << std::endl;
        // 		}
        // 		std::cout << newNode.solution[i].states.back().second << ": " <<
        // 		  		   newNode.solution[i].states.back().first << std::endl;          
        //   return false;
        // }

//         if(isSippSucc && success){
//         	if(solutionSipp.cost != newNode.solution[i].cost){
//         		std::cout << "Sipp is not equal \n";
// /*        		       for (size_t ii = 0; ii < newNode.solution[i].actions.size(); ++ii) {
//         		         std::cout << newNode.solution[i].states[ii].second << ": " <<
//         		        		 newNode.solution[i].states[ii].first << "->" << newNode.solution[i].actions[ii].first
//         		         << "(cost: " << newNode.solution[i].actions[ii].second << ")" << std::endl;
//         		       }
//         		       std::cout << newNode.solution[i].states.back().second << ": " <<
//         		    		   newNode.solution[i].states.back().first << std::endl;
// */
//         		return false;
//         	}
//         }


        // for(auto & constraint : newNode.constraints[i].vertexConstraints){
        // 	Location location(constraint.x, constraint.y);
        //   jpstbit.clearObstacle(location);
        // }
        // for(auto & constraint : newNode.constraints[i].edgeConstraints){
        // 	Location loc(constraint.x2, constraint.y2);
        //   jpstbit.clearObstacle(loc);
        // }


        // if(isJpsSucc && success){
        	// if(solutiontemp.cost != newNode.solution[i].cost){

 /*       		for (size_t ii = 0; ii < newNode.solution[i].actions.size(); ++ii) {
        			std::cout << newNode.solution[i].states[ii].second << ": " <<
        						newNode.solution[i].states[ii].first << "->" << newNode.solution[i].actions[ii].first
								<< "(cost: " << newNode.solution[i].actions[ii].second << ")" << std::endl;
        		}
        		std::cout << newNode.solution[i].states.back().second << ": " <<
        		  		   newNode.solution[i].states.back().first << std::endl;

                for (size_t ii = 0; ii < solutiontemp.actions.size(); ++ii) {
                	std::cout << solutiontemp.states[ii].second << ": " <<
        		         		 solutiontemp.states[ii].first << "->" << solutiontemp.actions[ii].first
        		       		         << "(cost: " << solutiontemp.actions[ii].second << ")" << std::endl;
        		}
        		std::cout << solutiontemp.states.back().second << ": " <<
        		    		   solutiontemp.states.back().first << std::endl;*/

        		// std::cout << "Jps is not equal \n";
        		// return false;
        	/*}else{
        		for (size_t ii = 0; ii < newNode.solution[i].actions.size(); ++ii) {
        			std::cout << newNode.solution[i].states[ii].second << ": " <<
        						newNode.solution[i].states[ii].first << "->" << newNode.solution[i].actions[ii].first
								<< "(cost: " << newNode.solution[i].actions[ii].second << ")" << std::endl;
        		}
        		std::cout << newNode.solution[i].states.back().second << ": " <<
        		  		   newNode.solution[i].states.back().first << std::endl;

                for (size_t ii = 0; ii < solutiontemp.actions.size(); ++ii) {
                	std::cout << solutiontemp.states[ii].second << ": " <<
        		         		 solutiontemp.states[ii].first << "->" << solutiontemp.actions[ii].first
        		       		         << "(cost: " << solutiontemp.actions[ii].second << ")" << std::endl;
        		}
        		std::cout << solutiontemp.states.back().second << ": " <<
        		    		   solutiontemp.states.back().first << std::endl;
        	}*/
        // } else if(!isJpsSucc && success){
    		// std::cout << "Jps is not equal 111\n";
    		// return false;
        // }


        // if(isJpstbit && success){
        // 	if(solutiontemp2.cost != newNode.solution[i].cost){

        // 		for (size_t ii = 0; ii < newNode.solution[i].actions.size(); ++ii) {
        // 			std::cout << newNode.solution[i].states[ii].second << ": " <<
        // 						newNode.solution[i].states[ii].first << "->" << newNode.solution[i].actions[ii].first
				// 				<< "(cost: " << newNode.solution[i].actions[ii].second << ")" << std::endl;
        // 		}
        // 		std::cout << newNode.solution[i].states.back().second << ": " <<
        // 		  		   newNode.solution[i].states.back().first << std::endl;

        //     for (size_t ii = 0; ii < solutiontemp2.actions.size(); ++ii) {
        //         	std::cout << solutiontemp2.states[ii].second << ": " <<
        // 		         		 solutiontemp2.states[ii].first << "->" << solutiontemp2.actions[ii].first
        // 		       		         << "(cost: " << solutiontemp2.actions[ii].second << ")" << std::endl;
        // 		}
        // 		std::cout << solutiontemp2.states.back().second << ": " <<
        // 		    		   solutiontemp2.states.back().first << std::endl;

        // 		std::cout << "Jpstbit is not equal 2222\n";
        // 		return false;
        /*	}/*else{
        		for (size_t ii = 0; ii < newNode.solution[i].actions.size(); ++ii) {
        			std::cout << newNode.solution[i].states[ii].second << ": " <<
        						newNode.solution[i].states[ii].first << "->" << newNode.solution[i].actions[ii].first
								<< "(cost: " << newNode.solution[i].actions[ii].second << ")" << std::endl;
        		}
        		std::cout << newNode.solution[i].states.back().second << ": " <<
        		  		   newNode.solution[i].states.back().first << std::endl;

                for (size_t ii = 0; ii < solutiontemp.actions.size(); ++ii) {
                	std::cout << solutiontemp.states[ii].second << ": " <<
        		         		 solutiontemp.states[ii].first << "->" << solutiontemp.actions[ii].first
        		       		         << "(cost: " << solutiontemp.actions[ii].second << ")" << std::endl;
        		}
        		std::cout << solutiontemp.states.back().second << ": " <<
        		    		   solutiontemp.states.back().first << std::endl;
        	}*/
        // } else if(!isJpstbit && success){
    		// std::cout << "Jpstbit is not equal 1111\n";
    		// return false;
        // }

        // for(size_t jj = 0; jj < newNode.solution.size(); jj++){        
        // 		for (size_t ii = 0; ii < newNode.solution[jj].actions.size(); ++ii) {
        // 			std::cout << newNode.solution[jj].states[ii].second << ": " <<
        // 						newNode.solution[jj].states[ii].first << "->" << newNode.solution[jj].actions[ii].first
				// 				<< "(cost: " << newNode.solution[jj].actions[ii].second << ")" << std::endl;
        // 		}
        // 		std::cout << newNode.solution[jj].states.back().second << ": " <<
        // 		  		   newNode.solution[jj].states.back().first << std::endl;
        // }

        if (successA) {
        //   if(newNode.solution[i].cost != solutiontemp.cost){
        //     std::cout << "Not equal\n";
        //     return false;
        //   }
        //  if(newNode.solution[i].cost != solutiontemp4.cost){
        //      std::cout << newNode.solution[i].cost << ", " << solutiontemp4.cost << "Not equal 222\n";
        //     return false;
        //  }

             
        		// for (size_t ii = 0; ii < solutiontemp4.actions.size(); ++ii) {
        		// 	std::cout << solutiontemp4.states[ii].second << ": " <<
        		// 				solutiontemp4.states[ii].first << "->" << solutiontemp4.actions[ii].first
						// 		<< "(cost: " << solutiontemp4.actions[ii].second << ")" << std::endl;
        		// }
        		// std::cout << solutiontemp4.states.back().second << ": " <<
        		//   		   solutiontemp4.states.back().first << std::endl;
            
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

  bool TryBypassAstar(Conflict cft, HighLevelNode& CurNode){
    int i = cft.agent1;
    PlanResult<State, Action, int> solutionA;

    m_env.setExactHeuristTrue();
    m_env.resetTemporalObstacle();
    for(auto & constraint : CurNode.constraints[i].vertexConstraints){
     	Location location(constraint.x, constraint.y);
      m_env.setTemporalObstacle(location, constraint.time);
    }
    bool is_first_constraint_e = true;
    for(auto & constraint : CurNode.constraints[i].edgeConstraints){
     	Location loc(constraint.x2, constraint.y2);
       m_env.setTemporalEdgeConstraint(loc, constraint.time);
    }

    Constraints ct;
    if(cft.type == Conflict::Vertex){
      Location state1(cft.x1, cft.y1);
      m_env.setTemporalObstacle(state1, cft.time);
      m_env.createConstraintsFromV(cft.time, cft.x1, cft.y1, ct);
      CurNode.constraints[i].add(ct);
    }
    if(cft.type == Conflict::Edge){
      Location state2(cft.x2, cft.y2);
      m_env.setTemporalObstacle(state2, cft.time);
      m_env.createConstraintsFromE(cft.time, cft.x1, cft.y1, cft.x2, cft.y2, ct);
      CurNode.constraints[i].add(ct);
    }        

    LowLevelEnvironment llenv(m_env, CurNode.solution, i, CurNode.constraints[i]);
    LowLevelSearch_t lowLevel(llenv);

    m_env.setGoal(i);
    m_env.Reset();
    State startNode = CurNode.solution[i].states[0].first;
    startNode.time = 0;
    bool isSuccA = lowLevel.search(startNode, solutionA);

    if(cft.type == Conflict::Vertex){
      for(auto& vc : ct.vertexConstraints){
        auto iter = CurNode.constraints[i].vertexConstraints.find(vc);
        if(iter != CurNode.constraints[i].vertexConstraints.end())
        CurNode.constraints[i].vertexConstraints.erase(iter);
      }

    }
    if(cft.type == Conflict::Edge){
      for(auto& ec : ct.edgeConstraints){
        auto iter = CurNode.constraints[i].edgeConstraints.find(ec);
        if(iter != CurNode.constraints[i].edgeConstraints.end())
        CurNode.constraints[i].edgeConstraints.erase(iter);
      }      
    }   

    if(isSuccA && solutionA.cost == CurNode.solution[i].cost){
      // std::cout << " --- \n";
      CurNode.solution[i] = solutionA;
      return true;
    }else return false;
  }



 private:

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
          neighbors[nei].state.nc_cat = s.nc_cat;
          int current_time = s.time + 1;
          State temp_s(-1, -1, -1), temp_s_p(-1, -1, -1);
          for(size_t agent_id = 0; agent_id < m_cat.size(); agent_id++){
            if(m_cat[agent_id].states.empty()) continue;
            if(agent_id == m_env.getAgentId()) continue;
            if(m_env.getAgentId() == agent_id) continue;
            if (current_time < m_cat[agent_id].states.size()) {
              temp_s = m_cat[agent_id].states[current_time].first;
            }else{
              temp_s = m_cat[agent_id].states.back().first;     
            }
            if(temp_s.x == neighbors[nei].state.x && temp_s.y == neighbors[nei].state.y){
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