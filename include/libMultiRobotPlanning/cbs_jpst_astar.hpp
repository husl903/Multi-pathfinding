
#pragma once

#include <map>
#include <time.h>
#include <iostream>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/resource.h>


#include "a_star.hpp"
#include "sipp.hpp"
#include "JPSSIPPCBS.hpp"
#include "JPSSIPP_BITCBS.hpp"
#include "planresult.hpp"
#include "timer.hpp"

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
          
class CBSJPSTAstar {
  public:
 
 public:
  CBSJPSTAstar(Environment& environment) : m_env(environment) {}

  bool search(const std::vector<State>& initialStates,
              std::vector<PlanResult<Location, Action, Cost> >& solution) {
                
    HighLevelNodeJps startJps;
    startJps.solution.resize(initialStates.size());
    startJps.constraints.resize(initialStates.size());
    startJps.cost = 0;
    startJps.id = 0;
    m_env.Reset();
    m_env.resetTemporalObstacle();

    for (size_t i = 0; i < initialStates.size(); ++i) {

        jpst_bit jps1(m_env);
        jps1.setEdgeCollisionSize(m_env.m_dimx, m_env.m_dimy);      
        Location goal = m_env.setGoal(i);
        Location startNode(-1, -1);
        startNode.x = initialStates[i].x;
        startNode.y = initialStates[i].y;       
        bool isJpsSucc = jps1.search(startNode, Action::Wait, startJps.solution[i], 0);
        if(!isJpsSucc){
          return false;
        }
        startJps.cost += startJps.solution[i].cost;
    }
    solution = startJps.solution;
    // std::cout << " Initialize finished " << startJps.cost  << ", " << start.cost << " --------------------------------------------------------------" << std::endl;

    // std::vector<HighLevelNodeJps> high_levle_node;
    // std::ifstream file_res("Berlin-highnode-1.txt");
    // high_levle_node.resize(1000);
    // int high_node_num = 54;
    // for(int h_node = 0; h_node < high_node_num; h_node++){
    //   int id_i, agent_id_i, cost_i;
    //   high_levle_node[h_node].solution.resize(initialStates.size());
    //   high_levle_node[h_node].constraints.resize(initialStates.size());
    //   std::string str1;
    //   file_res >> str1  >>id_i  >> str1 >> agent_id_i >> str1 >> cost_i;
    //   // std::cout << id_i << agent_id_i << cost_i;
    //   high_levle_node[h_node].id = id_i;
    //   high_levle_node[h_node].agent_id = agent_id_i;
    //   high_levle_node[h_node].cost = cost_i;
    //   for(int num_agent = 0; num_agent < initialStates.size(); num_agent++){
    //     std::string str;
    //     file_res >> str;
    //     int v_constraint_num = 0;
    //     int e_constraint_num = 0;
    //     file_res >> str >> v_constraint_num;
    //     // std::cout << str << v_constraint_num<< std::endl;
    //     for(int vv = 0; vv < v_constraint_num; vv++){
    //       Constraints c1;
    //       int vc_time, vc_x1, vc_y1;
    //       file_res >> vc_time >> vc_x1 >> vc_y1 >> str;
    //       m_env.createConstraintsFromV(vc_time, vc_x1, vc_y1, c1);
    //       // std::cout << vc_time << ", (" << vc_x1 << ", " << vc_y1 << ")"<< c1 << " Here \n";
          
    //       high_levle_node[h_node].constraints[num_agent].add(c1);
    //     }
    //     file_res >> str >> e_constraint_num;
    //     // std::cout << str << e_constraint_num<< std::endl;
    //     for(int ee = 0; ee < e_constraint_num; ee++){
    //       Constraints c1;
    //       int ec_time, ec_x1, ec_y1, ec_x2, ec_y2;
    //       file_res >> ec_time >> ec_x1 >> ec_y1 >> ec_x2 >> ec_y2 >> str;
    //       // std::cout << ec_time << ", (" << ec_x1 << ", " << ec_y1 << ")" << " (" << ec_x2 << ", " << ec_y2 << ")" << c1 << " Here \n";
    //       m_env.createConstraintsFromE(ec_time, ec_x1, ec_y1, ec_x2, ec_y2, c1);
    //       high_levle_node[h_node].constraints[num_agent].add(c1);          
    //     }
    //   }
    // }

    // Timer timerJpstbit;
    // timerJpstbit.reset();
    // for(int h_node = 0; h_node < high_node_num; h_node++){
    //     size_t i = high_levle_node[h_node].agent_id;
        
    //     if(i == -1) continue;
    //     bool is_first_constraint_v = true;
    //     bool is_first_constraint_e = true;
    //     m_env.resetTemporalObstacle();
    //     Timer timerJps;       
    //     Location goal = m_env.setGoal(i);
    //     m_env.Reset();
    //     Location startNode(-1, -1);
    //     startNode.x = initialStates[i].x;
    //     startNode.y = initialStates[i].y;
    //     m_env.setExactHeuristTrue();

    //     // jpst_bit jpstbit(m_env);
    //     // jpstbit.setEdgeCollisionSize(m_env.m_dimx, m_env.m_dimy);
        
    //     // for(auto & constraint : high_levle_node[h_node].constraints[i].vertexConstraints){
    //     // 	Location location(constraint.x, constraint.y);
    //     // 	m_env.setTemporalObstacle(location, constraint.time);
    //     // 	if(is_first_constraint_v){
    //     // 		jpstbit.setCollisionVertex(location, constraint.time, constraint.time, true);            
    //     // 		is_first_constraint_v = false;
    //     // 	}else{
    //     // 		jpstbit.setCollisionVertex(location, constraint.time, constraint.time, false);            
    //     // 	}
    //     // }
    //     // for(auto & constraint : high_levle_node[h_node].constraints[i].edgeConstraints){
    //     // 	Location location(constraint.x2, constraint.y2);
    //     // 	m_env.setTemporalEdgeConstraint(location, constraint.time);
    //     // 	if(constraint.x1 == constraint.x2){
    //     // 		if(constraint.y1 == constraint.y2 - 1){
    //     // 			jpstbit.setEdgeConstraint(location, constraint.time, Action::Down, is_first_constraint_e);
    //     // 		}else if(constraint.y1 == constraint.y2 + 1){
    //     // 			jpstbit.setEdgeConstraint(location, constraint.time, Action::Up, is_first_constraint_e);
    //     // 		}
    //     // 	}else{
    //     // 		if(constraint.x1 == constraint.x2 - 1){
    //     // 			jpstbit.setEdgeConstraint(location, constraint.time, Action::Left, is_first_constraint_e);
    //     // 		}else if(constraint.x1 == constraint.x2 + 1){
    //     // 			jpstbit.setEdgeConstraint(location, constraint.time, Action::Right, is_first_constraint_e);
    //     // 		}
    //     // 	}
    //     // 	if(is_first_constraint_e){
    //     // 		is_first_constraint_e = false;
    //     // 	}
    //     // }
    //     // jpstbit.sortCollisionVertex();
    //     // jpstbit.sortCollisionEdgeConstraint();
    //     // PlanResult<Location, Action, int> solutiontempJps;

    //     // timerJps.reset();
    //     // bool isJpstbit = jpstbit.search(startNode, Action::Wait, solutiontempJps, 0);
    //     // timerJps.stop();
    //     // double tJps = timerJps.elapsedSeconds();
    //     // int ExpJps = m_env.num_expansion;
    //     // int GenJps = m_env.num_generation;     

    //     PlanResult<Location, Action, int> solutionSipp;
    //     sipp_t sipp(m_env);
    //     sipp.setEdgeCollisionSize(m_env.m_dimx, m_env.m_dimy);        
    //     is_first_constraint_v = true;
    //     for(auto & constraint : high_levle_node[h_node].constraints[i].vertexConstraints){
    //     	Location location(constraint.x, constraint.y);
    //       m_env.setTemporalObstacle(location, constraint.time);
    //     	if(is_first_constraint_v){
    //     		sipp.setCollisionVertex(location, constraint.time, constraint.time, true);
    //     		is_first_constraint_v = false;
    //     	}else{
    //     		sipp.setCollisionVertex(location, constraint.time, constraint.time, false);
    //     	}
    //     }

    //     is_first_constraint_e = true;
    //     for(auto & constraint : high_levle_node[h_node].constraints[i].edgeConstraints){
    //     	Location loc(constraint.x2, constraint.y2);
    //       m_env.setTemporalEdgeConstraint(loc, constraint.time);
    //       //  std::cout << " Edge Constraint " << constraint.x1 << " " << constraint.y1 << ", second " << constraint.x2 << " " <<constraint.y2 << " " << constraint.time << " --\n";
    //     	if(constraint.x1 == constraint.x2){
    //     		if(constraint.y1 == constraint.y2 - 1){
    //     			sipp.setEdgeConstraint(loc, constraint.time, Action::Down, is_first_constraint_e);
    //     		}else if(constraint.y1 == constraint.y2 + 1){
    //     			sipp.setEdgeConstraint(loc, constraint.time, Action::Up, is_first_constraint_e);
    //     		}
    //     	}else{
    //     		if(constraint.x1 == constraint.x2 - 1){
    //     			sipp.setEdgeConstraint(loc, constraint.time, Action::Left, is_first_constraint_e);
    //     		}else if(constraint.x1 == constraint.x2 + 1){
    //     			sipp.setEdgeConstraint(loc, constraint.time, Action::Right, is_first_constraint_e);
    //     		}
    //     	}
    //     	if(is_first_constraint_e){
    //     		is_first_constraint_e = false;
    //     	}
    //     }
    //     sipp.sortCollisionVertex();
    //     sipp.sortCollisionEdgeConstraint();
    //     goal = m_env.setGoal(i);
    //     m_env.Reset();
    //     startNode.x = initialStates[i].x;
    //     startNode.y = initialStates[i].y;
    //     m_env.setExactHeuristTrue();
    //     timerJps.reset();
    //     bool isSippSucc = sipp.search(startNode, Action::Wait, solutionSipp, 0);
    //     timerJps.stop();
    //     double tSipp = timerJps.elapsedSeconds();
        
    //     // int ExpSipp = m_env.num_expansion;
    //     // int GenSipp = m_env.num_generation;
    //     // if(solutionSipp.cost != solutiontempJps.cost){
    //     //   std::cout << "Error " << " \n";
    //     //   return false;
    //     // }
    //   //  std::cout << solutionSipp.cost << ", " << solutiontempJps.cost << " time " << tJps << ", " << tSipp << ", " << tSipp/tJps << " \n";
    //    std::cout << solutionSipp.cost << " \n";
        
    // }
    // timerJpstbit.stop();
    // double tSipp = timerJpstbit.elapsedSeconds();
    // std::cout << "time " << tSipp << " \n";
    // return true;
    struct rusage r_usage;
   	getrusage(RUSAGE_SELF, &r_usage);

    typename boost::heap::d_ary_heap<HighLevelNodeJps, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >
        openJps;    
    auto handleJps = openJps.push(startJps);
    (*handleJps).handle = handleJps;

    solution.clear();
    int id = 1;    
    Timer timer;
    timer.reset();
    int num_node = 0;
    int gen_node = 0;
    while(!openJps.empty()){
      num_node++;
      // if(num_node > 50) return false;
      timer.stop();
      double duration1 = timer.elapsedSeconds();
      if(duration1 > 300){
        std::cout << " ,done, time-out fail" << ", num_node, " << num_node << " , gen_node, " << gen_node << ", ";
    	  return false;
      }
      
      // if(num_node % 100 == 0){
      //   getrusage(RUSAGE_SELF, &r_usage);
      //   if(r_usage.ru_maxrss > 13631488){
      //     std::cout << " ,done, memory-out fail" << ", num_node, " << num_node << " , gen_node, " << gen_node << ", ";
      //     return false;
      //   }
      // }      

      HighLevelNodeJps PJps = openJps.top();
      m_env.onExpandHighLevelNode(PJps.cost);
      openJps.pop();

    // std::cout << "***************************************************************************\n";
    //   // std::cout << PJps;
    //     for(size_t jj = 0; jj < PJps.solution.size(); jj++){ 
    //         std::cout << " solution for " << jj << std::endl;       
    //     		for (size_t ii = 0; ii < PJps.solution[jj].actions.size(); ++ii) {
    //     			std::cout << PJps.solution[jj].states[ii].second << ": " <<
    //     						PJps.solution[jj].states[ii].first << "->" << PJps.solution[jj].actions[ii].first
		// 						<< "(cost: " << PJps.solution[jj].actions[ii].second << ")" << std::endl;
    //     		}
    //     		std::cout << PJps.solution[jj].states.back().second << ": " <<
    //     		  		   PJps.solution[jj].states.back().first << std::endl;
    //     }
    // std::cout << "***************************************************************************\n";

      Conflict conflict;
      int jump_id_clf = -1;
      int return_value = getFirstConflict(PJps.solution, conflict, PJps);
      if(return_value  == 0){
        solution = PJps.solution;
        int jump_id_clf = -1;
        if(!m_env.CheckValid(PJps.solution)){
          std::cout << "Check solution falis \n";
          return false;
        }
        // if(m_env.getFirstConflict(PJps.solution, conflict, jump_id_clf)) std::cout << "Not equal\n";
        std::cout << " ,done, cost, " << PJps.cost << ", num_node, " << num_node << " , gen_node, " << gen_node << ", ";
        return true;
      }

      // if(m_env.isBP){
      //   if(return_value == 1){
      //     if(TryBypassJpst(conflict, PJps, jump_id_clf)) {
      //       auto handle = openJps.push(PJps);
      //       (*handle).handle = handle;
      //       continue;
      //     }
      //   }
      // }

 
      std::map<size_t, Constraints> constraints;
      m_env.createConstraintsFromConflict(conflict, constraints);
      for (const auto& c : constraints) {
        size_t i = c.first;
        HighLevelNodeJps newNodeJps = PJps;
        newNodeJps.id = id;
        newNodeJps.agent_id = i;
        
        bool is_debug_print = false;
        // if(is_debug_print){
        // for(size_t jj = 0; jj < newNodeJps.solution.size(); jj++){        
        // 		for (size_t ii = 0; ii < newNodeJps.solution[jj].actions.size(); ++ii) {
        // 			std::cout << newNodeJps.solution[jj].states[ii].second << ": " <<
        // 						newNodeJps.solution[jj].states[ii].first << "->" << newNodeJps.solution[jj].actions[ii].first
				// 				<< "(cost: " << newNodeJps.solution[jj].actions[ii].second << ")" << std::endl;
        // 		}
        // 		std::cout << newNodeJps.solution[jj].states.back().second << ": " <<
        // 		  		   newNodeJps.solution[jj].states.back().first << std::endl;
        // }
        // if(newNodeJps.constraints[i].overlap(c.second)){
        //   std::cout << c.first << ", " << c.second <<"OVERLAP \n";

        // }
        // }
        assert(!newNodeJps.constraints[i].overlap(c.second));
        newNodeJps.constraints[i].add(c.second);
        newNodeJps.cost -= newNodeJps.solution[i].cost;

        m_env.resetTemporalObstacle();
        bool is_first_constraint_v = true;
        bool is_first_constraint_e = true;

       jpst_bit jpstbit(m_env);
       jpstbit.setEdgeCollisionSize(m_env.m_dimx, m_env.m_dimy);

        for(auto & constraint : newNodeJps.constraints[i].vertexConstraints){
        	Location location(constraint.x, constraint.y);
        	m_env.setTemporalObstacle(location, constraint.time);
        	if(is_first_constraint_v){
//        		jps.setCollisionVertex(location, constraint.time, constraint.time, true);
        		jpstbit.setCollisionVertex(location, constraint.time, constraint.time, true);            
        		is_first_constraint_v = false;
        	}else{
//        		jps.setCollisionVertex(location, constraint.time, constraint.time, false);
        		jpstbit.setCollisionVertex(location, constraint.time, constraint.time, false);            
        	}
        }
        
        for(auto & constraint : newNodeJps.constraints[i].edgeConstraints){
        	Location location(constraint.x2, constraint.y2);
        	m_env.setTemporalEdgeConstraint(location, constraint.time);
        	if(constraint.x1 == constraint.x2){
        		if(constraint.y1 == constraint.y2 - 1){
//        			jps.setEdgeConstraint(location, constraint.time, Action::Down, is_first_constraint_e);
        			jpstbit.setEdgeConstraint(location, constraint.time, Action::Down, is_first_constraint_e);
        		}else if(constraint.y1 == constraint.y2 + 1){
//        			jps.setEdgeConstraint(location, constraint.time, Action::Up, is_first_constraint_e);
        			jpstbit.setEdgeConstraint(location, constraint.time, Action::Up, is_first_constraint_e);
        		}
        	}else{
        		if(constraint.x1 == constraint.x2 - 1){
//        			jps.setEdgeConstraint(location, constraint.time, Action::Left, is_first_constraint_e);
        			jpstbit.setEdgeConstraint(location, constraint.time, Action::Left, is_first_constraint_e);
        		}else if(constraint.x1 == constraint.x2 + 1){
//        			jps.setEdgeConstraint(location, constraint.time, Action::Right, is_first_constraint_e);
        			jpstbit.setEdgeConstraint(location, constraint.time, Action::Right, is_first_constraint_e);
        		}
        	}
        	if(is_first_constraint_e){
        		is_first_constraint_e = false;
        	}
        }

        jpstbit.sortCollisionVertex();
        jpstbit.sortCollisionEdgeConstraint();
        PlanResult<Location, Action, int> solutiontempJps;
        Location goal = m_env.setGoal(i);
        m_env.Reset();
        Location startNode(-1, -1);
        startNode.x = initialStates[i].x;
        startNode.y = initialStates[i].y;
        
        Timer timerJpstbit;
        timerJpstbit.reset();
        m_env.setExactHeuristTrue();
        bool isJpstbit = jpstbit.search(startNode, Action::Wait, newNodeJps.solution[i], 0);
        // newNodeJps.solution[i] = solutiontempJps;
        timerJpstbit.stop();
        double tJpstbit = timerJpstbit.elapsedSeconds();
        int ExpJps1 = m_env.num_expansion;
        int GenJps1 = m_env.num_generation;

        // PlanResult<Location, Action, int> solutionSipp;
        // sipp_t sipp(m_env, PJps.solution);
        // sipp.setEdgeCollisionSize(m_env.m_dimx, m_env.m_dimy);
        // m_env.resetTemporalObstacle();
        // is_first_constraint_v = true;
        // for(auto & constraint : newNodeJps.constraints[i].vertexConstraints){
        // 	Location location(constraint.x, constraint.y);
        //   m_env.setTemporalObstacle(location, constraint.time);
        //   if(is_debug_print) 
        //   std::cout << " Vertex " << constraint << " \n";
        // 	if(is_first_constraint_v){
        // 		sipp.setCollisionVertex(location, constraint.time, constraint.time, true);
        // 		is_first_constraint_v = false;
        // 	}else{
        // 		sipp.setCollisionVertex(location, constraint.time, constraint.time, false);
        // 	}
        // }
        // is_first_constraint_e = true;
        // for(auto & constraint : newNodeJps.constraints[i].edgeConstraints){
        //  	if(is_debug_print) 
        //    std::cout << " Edge Constraint " << constraint.x1 << " " << constraint.y1 << ", second " << constraint.x2 << " " <<constraint.y2 << " " << constraint.time << " --\n";
        // 	Location loc(constraint.x2, constraint.y2);
        //   m_env.setTemporalEdgeConstraint(loc, constraint.time);
        //   //m_env.setTemporalObstacle(loc, constraint.time);          
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
        // bool isSippSucc = sipp.search(startNode, Action::Wait, newNodeJps.solution[i], 0);
        // timerSipp.stop();
        // double tSipp = timerSipp.elapsedSeconds();
        // int ExpSipp = m_env.num_expansion;
        // int GenSipp = m_env.num_generation;        

        // m_env.setExactHeuristTrue();
        // LowLevelEnvironment llenv(m_env, i, newNodeJps.constraints[i]);
        // LowLevelSearch_t lowLevel(llenv);

        // Timer timerAstar;
        // timerAstar.reset();
        // PlanResult<State, Action, int> solutionAstar;
        // int ExpA =  m_env.lowLevelExpanded();
        // bool success = lowLevel.search(initialStates[i], solutionAstar);
        // timerAstar.stop();
        // int ExpAstar = m_env.lowLevelExpanded() - ExpA;
        // int GenAstar = m_env.lowLevelGenerated();
        // double tAstar = timerAstar.elapsedSeconds();


        // m_env.setExactHeuristTrue();
        // LowLevelEnvironment llenvP(m_env, i, newNodeJps.constraints[i]);
        // LowLevelSearch_t lowLevelP(llenvP);

        // Timer timerAstarP;
        // PlanResult<State, Action, int> solutionAstarP;
        // timerAstarP.reset();
        // int ExpAP =  m_env.lowLevelExpanded();
        // bool successP = lowLevelP.search(initialStates[i], solutionAstarP);
        // timerAstarP.stop();
        // int ExpAstarP = m_env.lowLevelExpanded() - ExpAP;
        // int GenAstarP = m_env.lowLevelGenerated();
        // double tAstarP = timerAstarP.elapsedSeconds();

                // std::cout << i << ", Start, (" << initialStates[i].x << " " << initialStates[i].y <<
                // 		"), Goal, (" << goal.x << " " << goal.y <<
        				// "), Cost jps , " << newNodeJps.solution[i].cost<< " , VertexConstraint ," << newNodeJps.constraints[i].vertexConstraints.size() <<
        				// ", EdgeConstraint , " << newNodeJps.constraints[i].edgeConstraints.size() <<
                // ", preTime, " << m_env.getPreTime(i) << 
        				// ", Time , " << tAstar << " , " << tSipp << " , " << tJps << ", " << tJpstbit <<
        				// ", Exp , " << ExpAstar << " , " << ExpSipp << " , " << ExpJps <<
        				// ", Gen , " << GenAstar << " , " << GenSipp << " , " << GenJps <<
        				// " \n";
        // for(auto & constraint : newNodeJps.constraints[i].vertexConstraints){
        // 	Location location(constraint.x, constraint.y);
        //   jpstbit.clearObstacle(location);
        // }
        // for(auto & constraint : newNodeJps.constraints[i].edgeConstraints){
        // 	Location loc(constraint.x2, constraint.y2);
        //   jpstbit.clearObstacle(loc);
        // }

      //  if(isSippSucc && isJpstbit){
      //   	if(solutionSipp.cost != newNodeJps.solution[i].cost){
      //   		std::cout << "Sipp is not equal \n";
      //   		return false;
      //   	}
      //   	for (size_t ii = 0; ii < solutionSipp.actions.size(); ++ii) {
      //   	    std::cout << solutionSipp.states[ii].second << ": " <<
      //   		  		 solutionSipp.states[ii].first << "->" << solutionSipp.actions[ii].first
      //   		         << "(cost: " << solutionSipp.actions[ii].second << ")" << std::endl;
      //   	}
      //   	std::cout << solutionSipp.states.back().second << ": " <<
      //   		solutionSipp.states.back().first << std::endl;          
      //   }

//        if(isJpsSucc && success){
//        	if(solutionAstar.cost != solutiontempJps.cost){
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

//        		std::cout << "Jps is not equal \n";
//        		return false;
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
//        } else if(!isJpsSucc && success){
//      		std::cout << "Jps is not equal 111\n";
      		// return false;
        // }

/*        if(isJpstbit && success){
        	if(newNodeJps.solution[i].cost != solutionAstar.cost){
        		for (size_t ii = 0; ii < solutionAstar.actions.size(); ++ii) {
        			std::cout << solutionAstar.states[ii].second << ": " <<
        						solutionAstar.states[ii].first << "->" << solutionAstar.actions[ii].first
								<< "(cost: " << solutionAstar.actions[ii].second << ")" << std::endl;
        		}
        		std::cout << solutionAstar.states.back().second << ": " <<
        		  		   solutionAstar.states.back().first << std::endl;

            for (size_t ii = 0; ii < newNodeJps.solution[i].actions.size(); ++ii) {
                	std::cout << newNodeJps.solution[i].states[ii].second << ": " <<
        		         		 newNodeJps.solution[i].states[ii].first << "->" << newNodeJps.solution[i].actions[ii].first
        		       		         << "(cost: " << newNodeJps.solution[i].actions[ii].second << ")" << std::endl;
        		}
        		std::cout << newNodeJps.solution[i].states.back().second << ": " <<
        		    		   newNodeJps.solution[i].states.back().first << std::endl;

        		std::cout << newNodeJps.solution[i].cost << ", " << solutionAstar.cost << " Jpstbit is not equal 2222\n";
        		return false;
        	}*/ /*else{
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
    	  //   	std::cout << "Jpstbit is not equal 1111\n";
    		//     return false;
        // }
        // if(solutiontempJps.cost != newNodeJps.solution[i].cost){

        //   std::cout << solutiontempJps.cost << ", " << newNodeJps.solution[i].cost <<" Solution " << "Cost error\n";
        //   return false;
        // }
        newNodeJps.cost += newNodeJps.solution[i].cost;
        if (isJpstbit) {
          auto handle = openJps.push(newNodeJps);
          (*handle).handle = handle;
          newNodeJps.agent_id = i;
          // if(newNodeJps.solution[i].cost != solutionSipp.cost) {
          //   std::cout << "Sipp is not equal to jpst\n";
          //   return false;
          // }
          // std::cout << newNodeJps;
          // std::cout << newNodeJps.solution[i].cost  << ", jpst " << solutiontempJps.cost << " i " << newNodeJps.agent_id << std::endl;
          gen_node++;
        }
        ++id;
      }     
    }

    return false;
  }

private:
  typedef struct HighLevelNodeJps {
    std::vector<PlanResult<Location, Action, Cost> > solution;
    std::vector<Constraints> constraints;

    Cost cost;

    int id;
    int agent_id = -1;

    typename boost::heap::d_ary_heap<HighLevelNodeJps, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >::handle_type
        handle;

    bool operator<(const HighLevelNodeJps& n) const {
      // if (cost != n.cost)
      return cost > n.cost;
      // return id > n.id;
    }

    friend std::ostream& operator<<(std::ostream& os, const HighLevelNodeJps& c) {
      os << "id: " << c.id <<  " agentId "<< c.agent_id << " cost: " << c.cost << std::endl;
      for (size_t i = 0; i < c.solution.size(); ++i) {
        // os << "Agent: " << i << std::endl;
        // os << " States:" << std::endl;
        // for (size_t t = 0; t < c.solution[i].states.size(); ++t) {
        //   os << "  " << c.solution[i].states[t].first << std::endl;
        // }
        os << " Constraints:" << std::endl;
        os << c.constraints[i];
        // os << " cost: " << c.solution[i].cost << std::endl;
      }
      return os;
    }
  };  

  bool TryBypassJpst(Conflict cft, HighLevelNodeJps& CurNode, int& jump_id){
    
    int next_jump_id = jump_id + 1;
    int agentId = cft.agent1;
    if(next_jump_id >= CurNode.solution[agentId].states.size()) return false; //overflow
    Location start = CurNode.solution[agentId].states[jump_id].first;
    Location goalLoc = CurNode.solution[agentId].states[next_jump_id].first;
    int time_a = CurNode.solution[agentId].states[jump_id].second;
    int time_b = CurNode.solution[agentId].states[next_jump_id].second;
    int cost_t = time_b - time_a;
    if(abs(start.x - goalLoc.x) + abs(start.y - goalLoc.y) == 1) return false; // adjacent grids
    if(start.x == goalLoc.x || start.y == goalLoc.y) return false; // the stright line
    if(cft.x1 == start.x && cft.y1 == start.y) return false;
    if(cft.x2 == start.x && cft.y2 == start.y) return false;
    if(cft.x1 == goalLoc.x && cft.y1 == goalLoc.y) return false;
    if(cft.x2 == goalLoc.x && cft.y2 == goalLoc.y) return false;

    m_env.Reset();
    m_env.resetTemporalObstacle();
    m_env.setGoal(goalLoc, agentId);
    m_env.setExactHeuristTrue();
    m_env.setIsSegPlanning(true);
    jpst_bit jpstbit(m_env);
    bool is_first_constraint_v = true;
    bool is_first_constraint_e = true;
    bool isV = true;
    jpstbit.setEdgeCollisionSize(m_env.m_dimx, m_env.m_dimy);

    if(cft.type == Conflict::Vertex){
      Location state1(cft.x1, cft.y1);
      m_env.setTemporalObstacle(state1, cft.time);
      jpstbit.setCollisionVertex(state1, cft.time, cft.time, is_first_constraint_v, isV);
      is_first_constraint_v = false;
    }
    if(cft.type == Conflict::Edge){
      Location state2(cft.x2, cft.y2);
      m_env.setTemporalObstacle(state2, cft.time);
      Action ac_temp;
      if(cft.x1 == cft.x2){
        if(cft.y1 == cft.y2 - 1) ac_temp = Action::Down;
        else ac_temp = Action::Up;
      }else{
        if(cft.x1 == cft.x2 - 1) ac_temp = Action::Left;
        else ac_temp = Action::Right;
      }
      jpstbit.setEdgeConstraint(state2, cft.time, ac_temp, is_first_constraint_e);
      is_first_constraint_e = false;
    }
            
    for(auto & constraint : CurNode.constraints[agentId].vertexConstraints){
      Location location(constraint.x, constraint.y);
      m_env.setTemporalObstacle(location, constraint.time);
      jpstbit.setCollisionVertex(location, constraint.time, constraint.time, is_first_constraint_v, isV);  
      if(is_first_constraint_v) is_first_constraint_v = false;
    }
        
    for(auto & constraint : CurNode.constraints[agentId].edgeConstraints){
      Location location(constraint.x2, constraint.y2);
      m_env.setTemporalEdgeConstraint(location, constraint.time);
      if(constraint.x1 == constraint.x2){
        if(constraint.y1 == constraint.y2 - 1){
        	jpstbit.setEdgeConstraint(location, constraint.time, Action::Down, is_first_constraint_e);
        }else if(constraint.y1 == constraint.y2 + 1){
        	jpstbit.setEdgeConstraint(location, constraint.time, Action::Up, is_first_constraint_e);
        }
      }else{
        if(constraint.x1 == constraint.x2 - 1){
        	jpstbit.setEdgeConstraint(location, constraint.time, Action::Left, is_first_constraint_e);
        }else if(constraint.x1 == constraint.x2 + 1){
        	jpstbit.setEdgeConstraint(location, constraint.time, Action::Right, is_first_constraint_e);
  	    }
      }
      if(is_first_constraint_e){
        is_first_constraint_e = false;
      }
    }
    jpstbit.sortCollisionVertex();
    jpstbit.sortCollisionEdgeConstraint();
 
    PlanResult<Location, Action, int>segmentPathJPS;
    bool isJpsSucc = jpstbit.search(start, Action::Wait, segmentPathJPS, time_a);
    m_env.setIsSegPlanning(false);

    if(segmentPathJPS.cost == cost_t && isJpsSucc){
                    
      // for (size_t ii = 0; ii < solution[i].actions.size(); ++ii) {
      //   std::cout << solution[i].states[ii].second << ": " <<
      // 			solution[i].states[ii].first << "->" << solution[i].actions[ii].first
			// 	  << "(cost: " << solution[i].actions[ii].second << ")" << std::endl;
      // }
      // std::cout << solution[i].states.back().second << ": " <<
      // 		   solution[i].states.back().first << std::endl;

      // for (size_t iii = 0; iii < segmentPathJPS.actions.size(); ++iii) {
      //   std::cout << segmentPathJPS.states[iii].second << ": " <<
      //   segmentPathJPS.states[iii].first << "->" << segmentPathJPS.actions[iii].first
      //   << "(cost: " << segmentPathJPS.actions[iii].second << ")" << std::endl;
      // }
      // std::cout << segmentPathJPS.states.back().second << ": " <<
      // segmentPathJPS.states.back().first << std::endl;  
      // for(auto & constraint : CurNode.constraints[i].vertexConstraints){
      //   std::cout << constraint << "\n";
      // }
      // for(auto & constraint : CurNode.constraints[i].edgeConstraints){
      //   std::cout << constraint << "\n";
      // }
      auto it = CurNode.solution[agentId].states.begin();
      auto it_ac = CurNode.solution[agentId].actions.begin();

      CurNode.solution[agentId].states[jump_id] = segmentPathJPS.states[0];;
      CurNode.solution[agentId].actions[jump_id] = segmentPathJPS.actions[0];

      // for(auto it = segmentPathJPS.states.begin() + 1; it != segmentPathJPS.states.end(); it++){
      //   std::cout << (*it).first << " here\n";
      // }
      CurNode.solution[agentId].states.insert(it + jump_id + 1, segmentPathJPS.states.begin() + 1,
              segmentPathJPS.states.end() - 1);

      CurNode.solution[agentId].actions.insert(it_ac + jump_id + 1, segmentPathJPS.actions.begin() + 1,
              segmentPathJPS.actions.end()); 

      return true;
     }


    return false;
  }

  bool TryBypassSipp(Conflict cft, HighLevelNodeJps& CurNode){
        int i = cft.agent1;
        PlanResult<Location, Action, int> solutionSipp;
        sipp_t sipp(m_env, CurNode.solution);
        sipp.setEdgeCollisionSize(m_env.m_dimx, m_env.m_dimy);
        m_env.resetTemporalObstacle();
        bool is_first_constraint_v = true;
        for(auto & constraint : CurNode.constraints[i].vertexConstraints){
        	Location location(constraint.x, constraint.y);
          m_env.setTemporalObstacle(location, constraint.time);
        	if(is_first_constraint_v){
        		sipp.setCollisionVertex(location, constraint.time, constraint.time, true);
        		is_first_constraint_v = false;
        	}else{
        		sipp.setCollisionVertex(location, constraint.time, constraint.time, false);
        	}
        }
        bool is_first_constraint_e = true;
        for(auto & constraint : CurNode.constraints[i].edgeConstraints){
        	Location loc(constraint.x2, constraint.y2);
          m_env.setTemporalEdgeConstraint(loc, constraint.time);
        	if(constraint.x1 == constraint.x2){
        		if(constraint.y1 == constraint.y2 - 1){
        			sipp.setEdgeConstraint(loc, constraint.time, Action::Down, is_first_constraint_e);
        		}else if(constraint.y1 == constraint.y2 + 1){
        			sipp.setEdgeConstraint(loc, constraint.time, Action::Up, is_first_constraint_e);
        		}
        	}else{
        		if(constraint.x1 == constraint.x2 - 1){
        			sipp.setEdgeConstraint(loc, constraint.time, Action::Left, is_first_constraint_e);
        		}else if(constraint.x1 == constraint.x2 + 1){
        			sipp.setEdgeConstraint(loc, constraint.time, Action::Right, is_first_constraint_e);
        		}
        	}
        	if(is_first_constraint_e){
        		is_first_constraint_e = false;
        	}
        }

    if(cft.type == Conflict::Vertex){
      Location state1(cft.x1, cft.y1);
      m_env.setTemporalObstacle(state1, cft.time);
      sipp.setCollisionVertex(state1, cft.time, cft.time, is_first_constraint_v);
      is_first_constraint_v = false;
    }
    if(cft.type == Conflict::Edge){
      Location state2(cft.x2, cft.y2);
      m_env.setTemporalObstacle(state2, cft.time);
      Action ac_temp;
      if(cft.x1 == cft.x2){
        if(cft.y1 == cft.y2 - 1) ac_temp = Action::Down;
        else ac_temp = Action::Up;
      }else{
        if(cft.x1 == cft.x2 - 1) ac_temp = Action::Left;
        else ac_temp = Action::Right;
      }
      sipp.setEdgeConstraint(state2, cft.time, ac_temp, is_first_constraint_e);
      is_first_constraint_e = false;
    }        
    sipp.sortCollisionVertex();
    sipp.sortCollisionEdgeConstraint();
    m_env.setGoal(i);
    m_env.Reset();
    Location startNode = CurNode.solution[i].states[0].first;

    PlanResult<Location, Action, int> tempsolution;
    bool isSippSucc = sipp.search(startNode, Action::Wait, tempsolution, 0);

    if(tempsolution.cost == CurNode.solution[i].cost){
      CurNode.solution[i] = tempsolution;
      return true;
    }else return false;
  }

 bool TryBypassAstar(Conflict cft, HighLevelNodeJps& CurNode, int& jump_id){
    
    int next_jump_id = jump_id + 1;
    int agentId = cft.agent1;
    if(next_jump_id >= CurNode.solution[agentId].states.size()) return false; //overflow
    Location start = CurNode.solution[agentId].states[jump_id].first;
    Location goalLoc = CurNode.solution[agentId].states[next_jump_id].first;
    int time_a = CurNode.solution[agentId].states[jump_id].second;
    int time_b = CurNode.solution[agentId].states[next_jump_id].second;
    int cost_t = time_b - time_a;
    if(abs(start.x - goalLoc.x) + abs(start.y - goalLoc.y) == 1) return false; // adjacent grids
    if(start.x == goalLoc.x || start.y == goalLoc.y) return false; // the stright line
    if(cft.x1 == start.x && cft.y1 == start.y) return false;
    if(cft.x2 == start.x && cft.y2 == start.y) return false;
    if(cft.x1 == goalLoc.x && cft.y1 == goalLoc.y) return false;
    if(cft.x2 == goalLoc.x && cft.y2 == goalLoc.y) return false;

    State initialState(-1, -1, time_a);
    LowLevelEnvironment llenv(m_env, agentId, goalLoc, CurNode.constraints[agentId]);
    LowLevelSearch_t lowLevel(llenv);        
    PlanResult<State, Action, int>segmentPath;
    bool success = lowLevel.search(initialState, segmentPath);

    m_env.Reset();
    m_env.resetTemporalObstacle();
    m_env.setGoal(goalLoc, agentId);
    m_env.setExactHeuristTrue();
    m_env.setIsSegPlanning(true);
    jpst_bit jpstbit(m_env);
    bool is_first_constraint_v = true;
    bool is_first_constraint_e = true;
    bool isV = true;
    jpstbit.setEdgeCollisionSize(m_env.m_dimx, m_env.m_dimy);

    if(cft.type == Conflict::Vertex){
      Location state1(cft.x1, cft.y1);
      m_env.setTemporalObstacle(state1, cft.time);
      jpstbit.setCollisionVertex(state1, cft.time, cft.time, is_first_constraint_v, isV);
      is_first_constraint_v = false;
    }
    if(cft.type == Conflict::Edge){
      Location state2(cft.x2, cft.y2);
      m_env.setTemporalObstacle(state2, cft.time);
      Action ac_temp;
      if(cft.x1 == cft.x2){
        if(cft.y1 == cft.y2 - 1) ac_temp = Action::Down;
        else ac_temp = Action::Up;
      }else{
        if(cft.x1 == cft.x2 - 1) ac_temp = Action::Left;
        else ac_temp = Action::Right;
      }
      jpstbit.setEdgeConstraint(state2, cft.time, ac_temp, is_first_constraint_e);
      is_first_constraint_e = false;
    }
            
    for(auto & constraint : CurNode.constraints[agentId].vertexConstraints){
      Location location(constraint.x, constraint.y);
      m_env.setTemporalObstacle(location, constraint.time);
      jpstbit.setCollisionVertex(location, constraint.time, constraint.time, is_first_constraint_v, isV);  
      if(is_first_constraint_v) is_first_constraint_v = false;
    }
        
    for(auto & constraint : CurNode.constraints[agentId].edgeConstraints){
      Location location(constraint.x2, constraint.y2);
      m_env.setTemporalEdgeConstraint(location, constraint.time);
      if(constraint.x1 == constraint.x2){
        if(constraint.y1 == constraint.y2 - 1){
        	jpstbit.setEdgeConstraint(location, constraint.time, Action::Down, is_first_constraint_e);
        }else if(constraint.y1 == constraint.y2 + 1){
        	jpstbit.setEdgeConstraint(location, constraint.time, Action::Up, is_first_constraint_e);
        }
      }else{
        if(constraint.x1 == constraint.x2 - 1){
        	jpstbit.setEdgeConstraint(location, constraint.time, Action::Left, is_first_constraint_e);
        }else if(constraint.x1 == constraint.x2 + 1){
        	jpstbit.setEdgeConstraint(location, constraint.time, Action::Right, is_first_constraint_e);
  	    }
      }
      if(is_first_constraint_e){
        is_first_constraint_e = false;
      }
    }
    jpstbit.sortCollisionVertex();
    jpstbit.sortCollisionEdgeConstraint();
 
    PlanResult<Location, Action, int>segmentPathJPS;
    bool isJpsSucc = jpstbit.search(start, Action::Wait, segmentPathJPS, time_a);
    m_env.setIsSegPlanning(false);

    if(segmentPathJPS.cost == cost_t && isJpsSucc){
                    
      // for (size_t ii = 0; ii < solution[i].actions.size(); ++ii) {
      //   std::cout << solution[i].states[ii].second << ": " <<
      // 			solution[i].states[ii].first << "->" << solution[i].actions[ii].first
			// 	  << "(cost: " << solution[i].actions[ii].second << ")" << std::endl;
      // }
      // std::cout << solution[i].states.back().second << ": " <<
      // 		   solution[i].states.back().first << std::endl;

      // for (size_t iii = 0; iii < segmentPathJPS.actions.size(); ++iii) {
      //   std::cout << segmentPathJPS.states[iii].second << ": " <<
      //   segmentPathJPS.states[iii].first << "->" << segmentPathJPS.actions[iii].first
      //   << "(cost: " << segmentPathJPS.actions[iii].second << ")" << std::endl;
      // }
      // std::cout << segmentPathJPS.states.back().second << ": " <<
      // segmentPathJPS.states.back().first << std::endl;  
      // for(auto & constraint : CurNode.constraints[i].vertexConstraints){
      //   std::cout << constraint << "\n";
      // }
      // for(auto & constraint : CurNode.constraints[i].edgeConstraints){
      //   std::cout << constraint << "\n";
      // }
      auto it = CurNode.solution[agentId].states.begin();
      auto it_ac = CurNode.solution[agentId].actions.begin();

      CurNode.solution[agentId].states[jump_id] = segmentPathJPS.states[0];;
      CurNode.solution[agentId].actions[jump_id] = segmentPathJPS.actions[0];

      // for(auto it = segmentPathJPS.states.begin() + 1; it != segmentPathJPS.states.end(); it++){
      //   std::cout << (*it).first << " here\n";
      // }
      CurNode.solution[agentId].states.insert(it + jump_id + 1, segmentPathJPS.states.begin() + 1,
              segmentPathJPS.states.end() - 1);

      CurNode.solution[agentId].actions.insert(it_ac + jump_id + 1, segmentPathJPS.actions.begin() + 1,
              segmentPathJPS.actions.end()); 

      return true;
     }


    return false;
  }

  bool getFirstConflict(
      std::vector<PlanResult<Location, Action, int> >& solution,
      Conflict& result, HighLevelNodeJps& CurNode){
    std::vector<PlanResult<Location, Action, int>> solution_path(solution.size());
    std::vector<std::vector<int>> point_id(solution.size());
    std::vector<std::vector<int>> point_st(solution.size());
    // std::cout << point_id.size() << ", " << point_st.size() << ", " << solution_path.size() << " \n";
    
    int max_t = 0;
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
        for(int temp_y = 0; temp_y < abs(a.y - b.y); temp_y++){ //from 0 insure the first location is added
          Location temp_loc(a.x, a.y+flag_y*temp_y);
          solution_path[i].states.push_back(std::make_pair<>(temp_loc, time_a + temp_y));
          solution_path[i].actions.push_back(std::make_pair<>(ac_c, 1));
          point_id[i].push_back(jump_point_id);
          if(m_env.isObstacle(temp_loc)){
            std::cout << "Obstacle is here " << "X, Y " << temp_loc.x << ", " << temp_loc.y << " jump_id " <<  a.x << ", " << a.y
             << " jump_id + 1 " << b.x << ", " << b.y << " 111 "<< std::endl;
          }
          tt++;
        }
        if(a.x != b.x){
          Location temp_loc(a.x, a.y+flag_y*abs(a.y-b.y));
          solution_path[i].states.push_back(std::make_pair<>(temp_loc, time_a + abs(a.y - b.y)));
          solution_path[i].actions.push_back(std::make_pair<>(ac_c, 1));
          point_id[i].push_back(jump_point_id); 
          // if(m_env.isObstacle(temp_loc)){
          //   std::cout << "Obstacle is here " << "X, Y " << temp_loc.x << ", " << temp_loc.y << " jump_id " <<  a.x << ", " << a.y
          //    << " jump_id + 1 " << b.x << ", " << b.y   << " 222 "<< std::endl;
          // }
          tt++;        
        }
        int flag_x = 1;
        if(a.x <= b.x){flag_x = 1; ac_c = Action::Right;}
        else{flag_x = -1; ac_c = Action::Left;}
        for(int temp_x = 1; temp_x < abs(a.x - b.x); temp_x++){// from 1 insure the last location is not added
          Location temp_loc(a.x + flag_x*temp_x, b.y);
          solution_path[i].states.push_back(std::make_pair<>(temp_loc, time_a + abs(a.y - b.y)+temp_x-1));
          solution_path[i].actions.push_back(std::make_pair<>(ac_c, 1));
          point_id[i].push_back(jump_point_id);
          if(m_env.isObstacle(temp_loc)){
            std::cout << "Obstacle is here " << "X, Y " << temp_loc.x << ", " << temp_loc.y << " jump_id " <<  a.x << ", " << a.y
             << " jump_id + 1 " << b.x << ", " << b.y   << " 3333 "<< std::endl;
          }
          tt++;
        } 
        if(delta_t != abs(a.x - b.x) + abs(a.y - b.y)){
          Location temp_loc(-1, -1);
          if(a.x == b.x){ temp_loc.x = a.x, temp_loc.y = b.y - flag_y;}
          else{temp_loc.x = b.x - flag_x; temp_loc.y = b.y;}
          if(a.x == b.x && a.y == b.y) temp_loc = a;
          if(m_env.isObstacle(temp_loc)){
            std::cout << "Obstacle is here " << "X, Y " << temp_loc.x << ", " << temp_loc.y << " jump_id " <<  a.x << ", " << a.y
             << " jump_id + 1 " << b.x << ", " << b.y  << " 4444"<< std::endl;
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
            solution_path[i].states.push_back(std::make_pair<>(temp_loc, time_a + timed - 1 +temp_w));
            solution_path[i].actions.push_back(std::make_pair<>(Action::Wait, 1));
            point_id[i].push_back(jump_point_id);
            tt++;
          }
        }
      }
      if(tt - 1 != solution[i].cost){
           std::cout << "recover path is not correct\n";
        		// for (size_t ii = 0; ii < solution[i].actions.size(); ++ii) {
        		// 	std::cout << solution[i].states[ii].second << ": " <<
        		// 				solution[i].states[ii].first << "->" << solution[i].actions[ii].first
						// 		<< "(cost: " << solution[i].actions[ii].second << ")" << std::endl;
        		// }
        		// std::cout << solution[i].states.back().second << ": " <<
        		//   		   solution[i].states.back().first << std::endl;
            // std::cout << "------------------------------------------------------------------------------------\n";

        		// for (size_t ii = 0; ii < solution_path[i].actions.size(); ++ii) {
        		// 	std::cout << solution_path[i].states[ii].second << ": " <<
        		// 				solution_path[i].states[ii].first << "->" << solution_path[i].actions[ii].first
						// 		<< "(cost: " << solution_path[i].actions[ii].second << ")" << std::endl;
        		// }
        		// std::cout << solution_path[i].states.back().second << ": " <<
        		//   		   solution_path[i].states.back().first << std::endl;
        return false;
      }
    }

        		// for (size_t ii = 0; ii < solution[19].actions.size(); ++ii) {
        		// 	std::cout << solution[19].states[ii].second << ": " <<
        		// 				solution[19].states[ii].first << "->" << solution[19].actions[ii].first
						// 		<< "(cost: " << solution[19].actions[ii].second << ")" << std::endl;
        		// }
        		// std::cout << solution[19].states.back().second << ": " <<
        		//   		   solution[19].states.back().first << std::endl;
            // std::cout << " id :: ";
            // for(int ii = 0; ii < point_id[19].size(); ii++){
            //   std::cout << " id " << point_id[19][ii];
            // }
            // std::cout << "\n";

    // for(size_t i = 0; i < solution.size(); i++){
    //   std::cout << "Agent " << i << " ----------------------------------------------------\n";
    //   for(size_t jj = 0; jj < solution_path[i].states.size(); jj++){
    //     std::cout << "time: " << jj << ", location (" << solution_path[i].states[jj].first.x << ", " 
    //     << solution_path[i].states[jj].first.y << " ) id " << point_id[i][jj] << " \n";
    //   }
    // }
    // max_t = 0;
    // for (const auto& sol : solution_path) {
    //   max_t = std::max<int>(max_t, sol.states.size() - 1);
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
        
            int JumpPointId = point_id[i][t];
            int PreJumpPointId = -1;
            int AftJumpPointID = -1;
            if(t - 1 >= 0){
              if(point_id[i][t - 1] != JumpPointId) return true;
            }
            if(t + 1 < solution_path.size()){
              if(point_id[i][t + 1] != JumpPointId) return true;
            }
            if(JumpPointId + 1 >= solution[i].states.size()-1) return true;

            Location goalLoc = solution[i].states[JumpPointId+1].first;
            int time_a = solution[i].states[JumpPointId].second;
            int time_b = solution[i].states[JumpPointId + 1].second;
            int cost_t = time_b - time_a;
            
            State initialState(-1, -1, time_a);
            initialState.x = solution[i].states[JumpPointId].first.x;
            initialState.y = solution[i].states[JumpPointId].first.y;
            initialState.time = time_a;
            if(abs(initialState.x - goalLoc.x) + abs(initialState.y - goalLoc.y) == 1) return true;
            if(initialState.x == goalLoc.x || initialState.y == goalLoc.y //stright line
               || jump_point[i].find(JumpPointId) != jump_point[i].end()) return true;  // have replanned

            m_env.resetTemporalObstacle();
            for(auto & constraint : CurNode.constraints[i].vertexConstraints){
        	    Location location(constraint.x, constraint.y);
        	    m_env.setTemporalObstacle(location, constraint.time);
            }
            for(auto & constraint : CurNode.constraints[i].edgeConstraints){
        	    Location location(constraint.x2, constraint.y2);
        	    m_env.setTemporalEdgeConstraint(location, constraint.time);
            }
            if(JumpPointId + 1 == solution[i].states.size() - 1) m_env.setIsSegPlanning(false);
            else m_env.setIsSegPlanning(true);
            LowLevelEnvironment llenv(m_env, i, goalLoc, CurNode.constraints[i]);
            LowLevelSearch_t lowLevel(llenv);            
            PlanResult<State, Action, int>segmentPath;
            bool success = lowLevel.search(initialState, segmentPath);
            jump_point[i].insert(JumpPointId); 
            m_env.setIsSegPlanning(false);

            if(segmentPath.cost != cost_t){
              
        		for (size_t ii = 0; ii < solution[i].actions.size(); ++ii) {
        			std::cout << solution[i].states[ii].second << ": " <<
        						solution[i].states[ii].first << "->" << solution[i].actions[ii].first
								<< "(cost: " << solution[i].actions[ii].second << ")" << std::endl;
        		}
        		std::cout << solution[i].states.back().second << ": " <<
        		  		   solution[i].states.back().first << std::endl;

              for (size_t iii = 0; iii < segmentPath.actions.size(); ++iii) {
                std::cout << segmentPath.states[iii].second << ": " <<
                segmentPath.states[iii].first << "->" << segmentPath.actions[iii].first
                << "(cost: " << segmentPath.actions[iii].second << ")" << std::endl;
              }
              std::cout << segmentPath.states.back().second << ": " <<
              segmentPath.states.back().first << std::endl;

              std::cout << "agent " << i <<", "<< j << " id " << JumpPointId <<  ", x, y " << initialState.x << ", " << initialState.y  << ", xy " << goalLoc.x << ", " << goalLoc.y << "Not correct \n";
              std::cout <<success << ", " << cost_t << ", " << segmentPath.cost << " cost---------\n"; 
              std::cout << "test\n";
              // return false;
            }else{
              size_t jjj = 0;
              for(size_t iii = time_a; iii < time_b; ++iii){
                solution_path[i].states[iii].first.x = segmentPath.states[jjj].first.x;
                solution_path[i].states[iii].first.y = segmentPath.states[jjj].first.y;
                solution_path[i].states[iii].second = iii;
                solution_path[i].actions[iii] = segmentPath.actions[jjj];
                // std::cout << "segpath xy " << segmentPath.states[jjj].first.x << ", " 
                // << segmentPath.states[jjj].first.y << std::endl; 
                jjj++;
              }

            //   std::cout << "Herererererererereere        begin \n";
        		// for (size_t ii = 0; ii < solution[i].actions.size(); ++ii) {
        		// 	std::cout << solution[i].states[ii].second << ": " <<
        		// 				solution[i].states[ii].first << "->" << solution[i].actions[ii].first
						// 		<< "(cost: " << solution[i].actions[ii].second << ")" << std::endl;
        		// }
        		// std::cout << solution[i].states.back().second << ": " <<
        		//   		   solution[i].states.back().first << std::endl;
              

              auto it = solution[i].states.begin();
              auto it_ac = solution[i].actions.begin();
              solution[i].states.insert(it + JumpPointId + 1, solution_path[i].states.begin() + time_a + 1,
              solution_path[i].states.begin() + time_b);

              solution[i].actions.insert(it_ac + JumpPointId + 1, solution_path[i].actions.begin() + time_a + 1,
              solution_path[i].actions.begin() + time_b); 

// std::cout << "Herererererererereere        After \n";     
//         		for (size_t ii = 0; ii < solution[i].actions.size(); ++ii) {
//         			std::cout << solution[i].states[ii].second << ": " <<
//         						solution[i].states[ii].first << "->" << solution[i].actions[ii].first
// 								<< "(cost: " << solution[i].actions[ii].second << ")" << std::endl;
//         		}
//         		std::cout << solution[i].states.back().second << ": " <<
//         		  		   solution[i].states.back().first << std::endl;                

              is_restart = true;
              t = time_a - 1;
              break;
            }
            // return true;
          }
        }
        if(is_restart) break;
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
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Edge;
            result.x1 = state1a.x;
            result.y1 = state1a.y;
            result.x2 = state1b.x;
            result.y2 = state1b.y;

            int JumpPointId = point_id[i][t];
            if(JumpPointId + 1 >= solution[i].states.size()-1) return true;            

            Location goalLoc = solution[i].states[JumpPointId+1].first;
            int time_a = solution[i].states[JumpPointId].second;
            int time_b = solution[i].states[JumpPointId + 1].second;
            int cost_t = time_b - time_a;
            State initialState(-1, -1, time_a);
            initialState.x = solution[i].states[JumpPointId].first.x;
            initialState.y = solution[i].states[JumpPointId].first.y;
            initialState.time = time_a;
            if(abs(initialState.x - goalLoc.x) + abs(initialState.y - goalLoc.y) == 1) return true;
            if(initialState.x == goalLoc.x || initialState.y == goalLoc.y
            || jump_point[i].find(JumpPointId) != jump_point[i].end()) return true;


            m_env.resetTemporalObstacle();
            for(auto & constraint : CurNode.constraints[i].vertexConstraints){
        	    Location location(constraint.x, constraint.y);
        	    m_env.setTemporalObstacle(location, constraint.time);
            }
            for(auto & constraint : CurNode.constraints[i].edgeConstraints){
        	    Location location(constraint.x2, constraint.y2);
        	    m_env.setTemporalEdgeConstraint(location, constraint.time);
            }

            if(JumpPointId + 1 == solution[i].states.size() - 1) m_env.setIsSegPlanning(false);
            else m_env.setIsSegPlanning(true);
            LowLevelEnvironment llenv(m_env, i, goalLoc, CurNode.constraints[i]);
            LowLevelSearch_t lowLevel(llenv);            
            PlanResult<State, Action, int>segmentPath;
            bool success = lowLevel.search(initialState, segmentPath);
            jump_point[i].insert(JumpPointId);
            m_env.setIsSegPlanning(false);

            if(segmentPath.cost != cost_t){
              for (size_t iii = 0; iii < segmentPath.actions.size(); ++iii) {
                std::cout << segmentPath.states[iii].second << ": " <<
                segmentPath.states[iii].first << "->" << segmentPath.actions[iii].first
                << "(cost: " << segmentPath.actions[iii].second << ")" << std::endl;
              }
              std::cout << segmentPath.states.back().second << ": " <<
              segmentPath.states.back().first << std::endl;

              std::cout << "agent " << i <<", "<< j << ", x, y " << initialState.x << ", " << initialState.y  << ", xy " << goalLoc.x << ", " << goalLoc.y << "Not correct \n";
              std::cout <<success << ", " << cost_t << ", " << segmentPath.cost << " cost---------\n"; 
              // return false;
            }else{
              size_t jjj = 0;
              PlanResult<Location, Action, int> solution_temp_1;
              for(size_t iii = time_a; iii < time_b; ++iii){
                solution_path[i].states[iii].first.x = segmentPath.states[jjj].first.x;
                solution_path[i].states[iii].first.y = segmentPath.states[jjj].first.y;
                solution_path[i].states[iii].second = iii;
                solution_path[i].actions[iii] = segmentPath.actions[jjj];                
                // std::cout << " Edge x, y " << segmentPath.states[jjj].first.x << ", " 
                // << segmentPath.states[jjj].first.y << std::endl;
                jjj++;
              }


// std::cout << "Herererererererereere        Begin \n";     
//         		for (size_t ii = 0; ii < solution[i].actions.size(); ++ii) {
//         			std::cout << solution[i].states[ii].second << ": " <<
//         						solution[i].states[ii].first << "->" << solution[i].actions[ii].first
// 								<< "(cost: " << solution[i].actions[ii].second << ")" << std::endl;
//         		}
//         		std::cout << solution[i].states.back().second << ": " <<
//         		  		   solution[i].states.back().first << std::endl;  
//               std::cout << time_a << ", " << time_b -1 << " \n";
//               auto it_path = solution_path[i].states.begin() + time_b -1;
//               std::cout << (*it_path).first.x << ", " << (*it_path).first.y << " test\n";

              auto it = solution[i].states.begin();
              auto it_ac = solution[i].actions.begin();
              solution[i].states.insert(it + JumpPointId + 1, solution_path[i].states.begin() + time_a + 1,
              solution_path[i].states.begin() + time_b);
              solution[i].actions.insert(it_ac + JumpPointId + 1, solution_path[i].actions.begin() + time_a + 1,
              solution_path[i].actions.begin() + time_b);              
   
//         		for (size_t ii = 0; ii < solution[i].actions.size(); ++ii) {
//         			std::cout << solution[i].states[ii].second << ": " <<
//         						solution[i].states[ii].first << "->" << solution[i].actions[ii].first
// 								<< "(cost: " << solution[i].actions[ii].second << ")" << std::endl;
//         		}
//         		std::cout << solution[i].states.back().second << ": " <<
//         		  		   solution[i].states.back().first << std::endl;  
//               std::cout << time_a << ", " << time_b -1 << " \n";

              is_restart = true;
              t = time_a - 1;
              break;
            }
          }
        }
        if(is_restart) break;
      }
      
    }
    return false;    
  }




  int  getFirstConflict(
      std::vector<PlanResult<Location, Action, int> >& solution,
      Conflict& result, HighLevelNodeJps& CurNode, bool jpst_flag){
    std::vector<PlanResult<Location, Action, int>> solution_path(solution.size());
    std::vector<std::vector<int>> point_id(solution.size());
    std::vector<std::vector<int>> point_st(solution.size());
    // std::vector<VertexConstraint> v_constraint;
           
    int max_t = 0;
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
          solution_path[i].states.push_back(std::make_pair<>(temp_loc, time_a + abs(a.y - b.y)+temp_x-1));
          solution_path[i].actions.push_back(std::make_pair<>(ac_c, 1));
          point_id[i].push_back(jump_point_id);
          tt++;
        } 
        if(delta_t != abs(a.x - b.x) + abs(a.y - b.y)){
          Location temp_loc(-1, -1);
          if(a.x == b.x){ temp_loc.x = a.x, temp_loc.y = b.y - flag_y;}
          else{temp_loc.x = b.x - flag_x; temp_loc.y = b.y;}
          int timed = abs(a.x - b.x) + abs(a.y - b.y);
          for(int temp_w = 0; temp_w  < delta_t - timed; temp_w++){
            solution_path[i].states.push_back(std::make_pair<>(temp_loc, time_a + timed - 1 +temp_w));
            solution_path[i].actions.push_back(std::make_pair<>(Action::Wait, 1));
            point_id[i].push_back(jump_point_id);
            tt++;
          }
        }
      }
      if(tt - 1 != solution[i].cost){
        std::cout << "recover path is not correct\n";
        return false;
      }
    }

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
            return 2;

            // std::cout << "Vertex Conflict " << point_id[i].size() << ", " << state1.x << ", " << state1.y << std::endl;
            if(t >= point_id[i].size()) return 2;
            int JumpPointId = point_id[i][t];
            int PreJumpPointId = -1;
            int AftJumpPointID = -1;
            if(t - 1 >= 0){
              if(point_id[i][t - 1] != JumpPointId) return 2;
            }
            if(t + 1 < solution_path.size()){
              if(point_id[i][t + 1] != JumpPointId) return 2;
            }
            // std::cout << "time " << t << ", " <<JumpPointId <<  ", "<< JumpPointId + 1 << ", " << solution[i].states.size() - 1 << std::endl;
            if(JumpPointId + 1 > solution[i].states.size() - 1) return 2;

            Location goalLoc = solution[i].states[JumpPointId+1].first;
            int time_a = solution[i].states[JumpPointId].second;
            int time_b = solution[i].states[JumpPointId + 1].second;
            int cost_t = time_b - time_a;
            
            State initialState(-1, -1, time_a);
            initialState.x = solution[i].states[JumpPointId].first.x;
            initialState.y = solution[i].states[JumpPointId].first.y;
            initialState.time = time_a;
            if(abs(initialState.x - goalLoc.x) + abs(initialState.y - goalLoc.y) == 1) return 2;
            if(initialState.x == goalLoc.x || initialState.y == goalLoc.y 
               || jump_point[i].find(JumpPointId) != jump_point[i].end()) return 2;

            m_env.Reset();
            m_env.resetTemporalObstacle();
            m_env.setGoal(goalLoc, i);
            m_env.setExactHeuristTrue();
            m_env.setIsSegPlanning(true);
            jpst_bit jpstbit(m_env);
            bool is_first_constraint_v = true;
            bool is_first_constraint_e = true;
            bool isV = true;
            jpstbit.setEdgeCollisionSize(m_env.m_dimx, m_env.m_dimy);
            for (size_t idx = 0; idx < solution_path.size(); ++idx){
              if(idx == i) continue;
              for(int v_time = time_a; v_time <= time_b; v_time++){
                Location v_loc = getState(idx, solution_path, v_time);
        	      m_env.setTemporalObstacle(v_loc, v_time);
        		    jpstbit.setCollisionVertex(v_loc, v_time, v_time, is_first_constraint_v, isV);
                if(is_first_constraint_v) is_first_constraint_v = false;
              }             
            }
            m_env.setTemporalObstacle(state1, t);
            jpstbit.setCollisionVertex(state1, t, t, is_first_constraint_v, isV);
            is_first_constraint_v = false;

            for(auto & constraint : CurNode.constraints[i].vertexConstraints){
        	    Location location(constraint.x, constraint.y);
        	    m_env.setTemporalObstacle(location, constraint.time);
              jpstbit.setCollisionVertex(location, constraint.time, constraint.time, is_first_constraint_v, isV);  
              if(is_first_constraint_v) is_first_constraint_v = false;
            }
        
            for(auto & constraint : CurNode.constraints[i].edgeConstraints){
        	    Location location(constraint.x2, constraint.y2);
        	    m_env.setTemporalEdgeConstraint(location, constraint.time);
        	    if(constraint.x1 == constraint.x2){
        		    if(constraint.y1 == constraint.y2 - 1){
        			    jpstbit.setEdgeConstraint(location, constraint.time, Action::Down, is_first_constraint_e);
        		    }else if(constraint.y1 == constraint.y2 + 1){
        			    jpstbit.setEdgeConstraint(location, constraint.time, Action::Up, is_first_constraint_e);
        		    }
        	    }else{
        		    if(constraint.x1 == constraint.x2 - 1){
        			    jpstbit.setEdgeConstraint(location, constraint.time, Action::Left, is_first_constraint_e);
        		     }else if(constraint.x1 == constraint.x2 + 1){
        			    jpstbit.setEdgeConstraint(location, constraint.time, Action::Right, is_first_constraint_e);
        		    }
        	    }
        	    if(is_first_constraint_e){
        		    is_first_constraint_e = false;
        	    }
            }
            jpstbit.sortCollisionVertex();
            jpstbit.sortCollisionEdgeConstraint();
            Location startNode(solution[i].states[JumpPointId].first.x, 
                      solution[i].states[JumpPointId].first.y);
            PlanResult<Location, Action, int>segmentPathJPS;

            bool isJpsSucc = jpstbit.search(startNode, Action::Wait, segmentPathJPS, time_a);
            m_env.setIsSegPlanning(false);   

            if(segmentPathJPS.cost == cost_t && isJpsSucc){
              
        		  // for (size_t ii = 0; ii < solution[i].actions.size(); ++ii) {
        			//   std::cout << solution[i].states[ii].second << ": " <<
        			// 			solution[i].states[ii].first << "->" << solution[i].actions[ii].first
							// 	  << "(cost: " << solution[i].actions[ii].second << ")" << std::endl;
        		  // }
        		  // std::cout << solution[i].states.back().second << ": " <<
        		  // 		   solution[i].states.back().first << std::endl;

              // for (size_t iii = 0; iii < segmentPathJPS.actions.size(); ++iii) {
              //   std::cout << segmentPathJPS.states[iii].second << ": " <<
              //   segmentPathJPS.states[iii].first << "->" << segmentPathJPS.actions[iii].first
              //   << "(cost: " << segmentPathJPS.actions[iii].second << ")" << std::endl;
              // }
              // std::cout << segmentPathJPS.states.back().second << ": " <<
              // segmentPathJPS.states.back().first << std::endl;  
              // for(auto & constraint : CurNode.constraints[i].vertexConstraints){
     
              //   std::cout << constraint << "\n";
              // }
              // for(auto & constraint : CurNode.constraints[i].edgeConstraints){
              //   std::cout << constraint << "\n";
              // }
              auto it = solution[i].states.begin();
              auto it_ac = solution[i].actions.begin();

              solution[i].states[JumpPointId] = segmentPathJPS.states[0];;
              solution[i].actions[JumpPointId] = segmentPathJPS.actions[0];

              // for(auto it = segmentPathJPS.states.begin() + 1; it != segmentPathJPS.states.end(); it++){
              //   std::cout << (*it).first << " here\n";
              // }
              solution[i].states.insert(it + JumpPointId + 1, segmentPathJPS.states.begin() + 1,
              segmentPathJPS.states.end() - 1);

              solution[i].actions.insert(it_ac + JumpPointId + 1, segmentPathJPS.actions.begin() + 1,
              segmentPathJPS.actions.end()); 

              return 1;           
            } else return 2;
          }
        }
        if(is_restart) break;
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
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Edge;
            result.x1 = state1a.x;
            result.y1 = state1a.y;
            result.x2 = state1b.x;
            result.y2 = state1b.y;
            return 2;

            if(t >= point_id[i].size()) return 2;
            int JumpPointId = point_id[i][t];
            if(JumpPointId + 1 > solution[i].states.size()-1) return 2;            

            Location goalLoc = solution[i].states[JumpPointId+1].first;
            int time_a = solution[i].states[JumpPointId].second;
            int time_b = solution[i].states[JumpPointId + 1].second;
            int cost_t = time_b - time_a;
            State initialState(-1, -1, time_a);
            initialState.x = solution[i].states[JumpPointId].first.x;
            initialState.y = solution[i].states[JumpPointId].first.y;
            initialState.time = time_a;
            if(abs(initialState.x - goalLoc.x) + abs(initialState.y - goalLoc.y) == 1) return 2;
            if(initialState.x == goalLoc.x || initialState.y == goalLoc.y
            || jump_point[i].find(JumpPointId) != jump_point[i].end()) return 2;

            m_env.Reset();
            m_env.resetTemporalObstacle();
            m_env.setGoal(goalLoc, i);
            m_env.setExactHeuristTrue();
            m_env.setIsSegPlanning(true);
            jpst_bit jpstbit(m_env);
            bool is_first_constraint_v = true;
            bool is_first_constraint_e = true;
            bool isV = true;
            jpstbit.setEdgeCollisionSize(m_env.m_dimx, m_env.m_dimy);
            for (size_t idx = 0; idx < solution_path.size(); ++idx){
              if(idx == i) continue;
              for(int v_time = time_a; v_time <= time_b; v_time++){
                Location v_loc = getState(idx, solution_path, v_time);
        	      m_env.setTemporalObstacle(v_loc, v_time);
        		    jpstbit.setCollisionVertex(v_loc, v_time, v_time, is_first_constraint_v, isV);
                if(is_first_constraint_v) is_first_constraint_v = false;
              }             
            }

            for(auto & constraint : CurNode.constraints[i].vertexConstraints){
        	    Location location(constraint.x, constraint.y);
        	    m_env.setTemporalObstacle(location, constraint.time);
              jpstbit.setCollisionVertex(location, constraint.time, constraint.time, is_first_constraint_v, isV);  
              if(is_first_constraint_v) is_first_constraint_v = false;
            }
        
            for(auto & constraint : CurNode.constraints[i].edgeConstraints){
        	    Location location(constraint.x2, constraint.y2);
        	    m_env.setTemporalEdgeConstraint(location, constraint.time);
        	    if(constraint.x1 == constraint.x2){
        		    if(constraint.y1 == constraint.y2 - 1){
        			    jpstbit.setEdgeConstraint(location, constraint.time, Action::Down, is_first_constraint_e);
        		    }else if(constraint.y1 == constraint.y2 + 1){
        			    jpstbit.setEdgeConstraint(location, constraint.time, Action::Up, is_first_constraint_e);
        		    }
        	    }else{
        		    if(constraint.x1 == constraint.x2 - 1){
        			    jpstbit.setEdgeConstraint(location, constraint.time, Action::Left, is_first_constraint_e);
        		     }else if(constraint.x1 == constraint.x2 + 1){
        			    jpstbit.setEdgeConstraint(location, constraint.time, Action::Right, is_first_constraint_e);
        		    }
        	    }
        	    if(is_first_constraint_e){
        		    is_first_constraint_e = false;
        	    }
            }
            
            Action ac_temp;
            if(state1a.x == state1b.x){
              if(state1a.y == state1b.y - 1) ac_temp = Action::Down;
              else ac_temp = Action::Up;
            }else{
              if(state1a.x == state1b.x - 1) ac_temp = Action::Left;
              else ac_temp = Action::Right;
            }
            m_env.setTemporalObstacle(state1b, t);
            jpstbit.setEdgeConstraint(state1b, t, ac_temp, is_first_constraint_e);         
            
            jpstbit.sortCollisionVertex();
            jpstbit.sortCollisionEdgeConstraint();
            Location startNode(solution[i].states[JumpPointId].first.x, 
                      solution[i].states[JumpPointId].first.y);
            PlanResult<Location, Action, int>segmentPathJPS;

            bool isJpsSucc = jpstbit.search(startNode, Action::Wait, segmentPathJPS, time_a);
            m_env.setIsSegPlanning(false);   

            if(segmentPathJPS.cost == cost_t && isJpsSucc){
              
        		  // for (size_t ii = 0; ii < solution[i].actions.size(); ++ii) {
        			//   std::cout << solution[i].states[ii].second << ": " <<
        			// 			solution[i].states[ii].first << "->" << solution[i].actions[ii].first
							// 	  << "(cost: " << solution[i].actions[ii].second << ")" << std::endl;
        		  // }
        		  // std::cout << solution[i].states.back().second << ": " <<
        		  // 		   solution[i].states.back().first << std::endl;

              // for (size_t iii = 0; iii < segmentPathJPS.actions.size(); ++iii) {
              //   std::cout << segmentPathJPS.states[iii].second << ": " <<
              //   segmentPathJPS.states[iii].first << "->" << segmentPathJPS.actions[iii].first
              //   << "(cost: " << segmentPathJPS.actions[iii].second << ")" << std::endl;
              // }
              // std::cout << segmentPathJPS.states.back().second << ": " <<
              // segmentPathJPS.states.back().first << std::endl;  
              // for(auto & constraint : CurNode.constraints[i].vertexConstraints){
     
              //   std::cout << constraint << "\n";
              // }
              // for(auto & constraint : CurNode.constraints[i].edgeConstraints){
              //   std::cout << constraint << "\n";
              // }
              auto it = solution[i].states.begin();
              auto it_ac = solution[i].actions.begin();

              solution[i].states[JumpPointId] = segmentPathJPS.states[0];;
              solution[i].actions[JumpPointId] = segmentPathJPS.actions[0];

              // for(auto it = segmentPathJPS.states.begin() + 1; it != segmentPathJPS.states.end(); it++){
              //   std::cout << (*it).first << " here\n";
              // }
              solution[i].states.insert(it + JumpPointId + 1, segmentPathJPS.states.begin() + 1,
              segmentPathJPS.states.end() - 1);

              solution[i].actions.insert(it_ac + JumpPointId + 1, segmentPathJPS.actions.begin() + 1,
              segmentPathJPS.actions.end()); 

              return 1;           
            } else return 2;            

/*            LowLevelEnvironment llenv(m_env, i, goalLoc, CurNode.constraints[i]);
            LowLevelSearch_t lowLevel(llenv);            
            PlanResult<State, Action, int>segmentPath;
            bool success = lowLevel.search(initialState, segmentPath);
            jump_point[i].insert(JumpPointId);

            if(segmentPath.cost != cost_t){
              for (size_t iii = 0; iii < segmentPath.actions.size(); ++iii) {
                std::cout << segmentPath.states[iii].second << ": " <<
                segmentPath.states[iii].first << "->" << segmentPath.actions[iii].first
                << "(cost: " << segmentPath.actions[iii].second << ")" << std::endl;
              }
              std::cout << segmentPath.states.back().second << ": " <<
              segmentPath.states.back().first << std::endl;

              std::cout << "agent " << i <<", "<< j << ", x, y " << initialState.x << ", " << initialState.y  << ", xy " << goalLoc.x << ", " << goalLoc.y << "Not correct \n";
              std::cout <<success << ", " << cost_t << ", " << segmentPath.cost << " cost---------\n"; 
              // return false;
            }else{
              size_t jjj = 0;
              PlanResult<Location, Action, int> solution_temp_1;
              for(size_t iii = time_a; iii < time_b; ++iii){
                solution_path[i].states[iii].first.x = segmentPath.states[jjj].first.x;
                solution_path[i].states[iii].first.y = segmentPath.states[jjj].first.y;
                solution_path[i].states[iii].second = iii;
                solution_path[i].actions[iii] = segmentPath.actions[jjj];                
                // std::cout << " Edge x, y " << segmentPath.states[jjj].first.x << ", " 
                // << segmentPath.states[jjj].first.y << std::endl;
                jjj++;
              }


// std::cout << "Herererererererereere        Begin \n";     
//         		for (size_t ii = 0; ii < solution[i].actions.size(); ++ii) {
//         			std::cout << solution[i].states[ii].second << ": " <<
//         						solution[i].states[ii].first << "->" << solution[i].actions[ii].first
// 								<< "(cost: " << solution[i].actions[ii].second << ")" << std::endl;
//         		}
//         		std::cout << solution[i].states.back().second << ": " <<
//         		  		   solution[i].states.back().first << std::endl;  
//               std::cout << time_a << ", " << time_b -1 << " \n";
//               auto it_path = solution_path[i].states.begin() + time_b -1;
//               std::cout << (*it_path).first.x << ", " << (*it_path).first.y << " test\n";

              auto it = solution[i].states.begin();
              auto it_ac = solution[i].actions.begin();
              solution[i].states.insert(it + JumpPointId + 1, solution_path[i].states.begin() + time_a + 1,
              solution_path[i].states.begin() + time_b);
              solution[i].actions.insert(it_ac + JumpPointId + 1, solution_path[i].actions.begin() + time_a + 1,
              solution_path[i].actions.begin() + time_b);              
   
//         		for (size_t ii = 0; ii < solution[i].actions.size(); ++ii) {
//         			std::cout << solution[i].states[ii].second << ": " <<
//         						solution[i].states[ii].first << "->" << solution[i].actions[ii].first
// 								<< "(cost: " << solution[i].actions[ii].second << ")" << std::endl;
//         		}
//         		std::cout << solution[i].states.back().second << ": " <<
//         		  		   solution[i].states.back().first << std::endl;  
//               std::cout << time_a << ", " << time_b -1 << " \n";

              is_restart = true;
              t = time_a - 1;
              break;
            }*/
          }
        }
        if(is_restart) break;
      }
      
    }
    return 0;    
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

 
  struct HighLevelNode {
    std::vector<PlanResult<State, Action, Cost> > solution;
    std::vector<Constraints> constraints;

    Cost cost;

    int id;

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
 
private:
  struct LowLevelEnvironment {
    LowLevelEnvironment(Environment& env, size_t agentIdx,
                        const Constraints& constraints)
        : m_env(env)
    // , m_agentIdx(agentIdx)
    // , m_constraints(constraints)
    {
      m_env.setLowLevelContext(agentIdx, &constraints);
    }
    LowLevelEnvironment(Environment& env, size_t agentIdx, Location loc,
                        const Constraints& constraints)
        : m_env(env)
    // , m_agentIdx(agentIdx)
    // , m_constraints(constraints)
    {
      m_env.setLowLevelContext(agentIdx, &constraints);
      m_env.setGoal(loc);
    }    

    Cost admissibleHeuristic(const State& s) {
      return m_env.admissibleHeuristic(s);
    }

    bool isSolution(const State& s) { return m_env.isSolution(s); }

    void getNeighbors(const State& s,
                      std::vector<Neighbor<State, Action, Cost> >& neighbors) {
      // std::cout << "Current " << s.x << ", " << s.y << ",time " << s.time << " \n";
      m_env.getNeighbors(s, neighbors);
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
    // size_t m_agentIdx;
    // const Constraints& m_constraints;
  };

 private:
  Environment& m_env;
  typedef AStar<State, Action, Cost, LowLevelEnvironment> LowLevelSearch_t;
  typedef JPSSIPP<Location, Location, Action, int, Environment> jps_sipp;
  typedef SIPP<Location, Location, Action, int, Environment> sipp_t;
  typedef JPSSIPP_BITCBS<Location, Location, Action, int, Environment> jpst_bit;

};

}  // namespace libMultiRobotPlanning
