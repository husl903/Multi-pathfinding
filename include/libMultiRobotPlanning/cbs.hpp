
#pragma once

#include <map>
#include <time.h>
#include <iostream>

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
class CBS {
 public:
  CBS(Environment& environment) : m_env(environment) {}

  bool search(const std::vector<State>& initialStates,
              std::vector<PlanResult<Location, Action, Cost> >& solution) {
    HighLevelNode start;
    start.solution.resize(initialStates.size());
    start.constraints.resize(initialStates.size());
    start.cost = 0;
    start.id = 0;
    
    HighLevelNodeJps startJps;
    startJps.solution.resize(initialStates.size());
    startJps.constraints.resize(initialStates.size());
    startJps.cost = 0;
    startJps.id = 0;
    m_env.Reset();
    m_env.resetTemporalObstacle();    
    
    for (size_t i = 0; i < initialStates.size(); ++i) {
        
        jpst_bit jps1(m_env);
        // sipp_t jps1(m_env);
        jps1.setEdgeCollisionSize(m_env.m_dimx, m_env.m_dimy);
        PlanResult<Location, Action, int> solutiontempJps;

        Location goal = m_env.setGoal(i);
        Location startNode(-1, -1);
        startNode.x = initialStates[i].x;
        startNode.y = initialStates[i].y;       
        bool isJpsSucc = jps1.search(startNode, Action::Wait, startJps.solution[i], 0);
        if(!isJpsSucc){
          return false;
        }
        startJps.cost += startJps.solution[i].cost;
    //  LowLevelEnvironment llenv(m_env, i, start.constraints[i]);
    //  LowLevelSearch_t lowLevel(llenv);
    //  bool success = lowLevel.search(initialStates[i], start.solution[i]);
    //  if (!success) {
    //    return false;
    //  }
    //   start.cost += start.solution[i].cost;
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
      timer.stop();
      double duration1 = timer.elapsedSeconds();
      if(duration1 > 300){
    	  return false;
      }

      HighLevelNodeJps PJps = openJps.top();
      m_env.onExpandHighLevelNode(PJps.cost);
      openJps.pop();
      
      Conflict conflict;
      if(getFirstConflict(PJps.solution, conflict)){
        solution = PJps.solution;
        std::cout << " ,done, cost, " << PJps.cost << ", num_node, " << num_node << " , gen_node, " << gen_node << ", ";
        return true;
      }
//      std::cout << PJps;
        // for(size_t jj = 0; jj < PJps.solution.size(); jj++){        
        // 		for (size_t ii = 0; ii < PJps.solution[jj].actions.size(); ++ii) {
        // 			std::cout << PJps.solution[jj].states[ii].second << ": " <<
        // 						PJps.solution[jj].states[ii].first << "->" << PJps.solution[jj].actions[ii].first
				// 				<< "(cost: " << PJps.solution[jj].actions[ii].second << ")" << std::endl;
        // 		}
        // 		std::cout << PJps.solution[jj].states.back().second << ": " <<
        // 		  		   PJps.solution[jj].states.back().first << std::endl;
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
      //  jps_sipp jps(m_env);
      //  jps.setEdgeCollisionSize(m_env.m_dimx, m_env.m_dimy);

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

//        jps.sortCollisionVertex();
//        jps.sortCollisionEdgeConstraint();
        jpstbit.sortCollisionVertex();
        jpstbit.sortCollisionEdgeConstraint();
        PlanResult<Location, Action, int> solutiontempJps;
        Location goal = m_env.setGoal(i);
        m_env.Reset();
        Location startNode(-1, -1);
        startNode.x = initialStates[i].x;
        startNode.y = initialStates[i].y;
        // m_env.setExactHeuristTrue();
        // Timer timerJps;
        // timerJps.reset();
        // bool isJpsSucc = jps.search(startNode, Action::Wait, solutiontempJps, 0, true);
        // timerJps.stop();
        // double tJps = timerJps.elapsedSeconds();
        // int ExpJps = m_env.num_expansion;
        // int GenJps = m_env.num_generation;
        
        Timer timerJpstbit;
        timerJpstbit.reset();
        m_env.setExactHeuristTrue();
        bool isJpstbit = jpstbit.search(startNode, Action::Wait, newNodeJps.solution[i], 0);
        // newNodeJps.solution[i] = solutiontempJps;
        timerJpstbit.stop();
        double tJpstbit = timerJpstbit.elapsedSeconds();
        int ExpJps1 = m_env.num_expansion;
        int GenJps1 = m_env.num_generation;


        PlanResult<Location, Action, int> solutionSipp;
        sipp_t sipp(m_env);
        sipp.setEdgeCollisionSize(m_env.m_dimx, m_env.m_dimy);
        is_first_constraint_v = true;
        for(auto & constraint : newNodeJps.constraints[i].vertexConstraints){
        	Location location(constraint.x, constraint.y);
          m_env.setTemporalObstacle(location, constraint.time);
          if(is_debug_print) 
          std::cout << " Vertex " << constraint << " \n";
        	if(is_first_constraint_v){
        		sipp.setCollisionVertex(location, constraint.time, constraint.time, true);
        		is_first_constraint_v = false;
        	}else{
        		sipp.setCollisionVertex(location, constraint.time, constraint.time, false);
        	}
        }
        is_first_constraint_e = true;
        for(auto & constraint : newNodeJps.constraints[i].edgeConstraints){
         	if(is_debug_print) 
           std::cout << " Edge Constraint " << constraint.x1 << " " << constraint.y1 << ", second " << constraint.x2 << " " <<constraint.y2 << " " << constraint.time << " --\n";
        	Location loc(constraint.x2, constraint.y2);
          m_env.setTemporalEdgeConstraint(loc, constraint.time);
          //m_env.setTemporalObstacle(loc, constraint.time);          
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
        sipp.sortCollisionVertex();
        sipp.sortCollisionEdgeConstraint();
        goal = m_env.setGoal(i);
        m_env.Reset();
        startNode.x = initialStates[i].x;
        startNode.y = initialStates[i].y;

        Timer timerSipp;
        timerSipp.reset();
        m_env.setExactHeuristTrue();
        bool isSippSucc = sipp.search(startNode, Action::Wait, solutionSipp, 0);
        timerSipp.stop();
        double tSipp = timerSipp.elapsedSeconds();
        int ExpSipp = m_env.num_expansion;
        int GenSipp = m_env.num_generation;        

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
          if(newNodeJps.solution[i].cost != solutionSipp.cost) {
            std::cout << "Sipp is not equal to jpst\n";
            return false;
          }
          // std::cout << newNodeJps;
          // std::cout << newNodeJps.solution[i].cost  << ", jpst " << solutiontempJps.cost << " i " << newNodeJps.agent_id << std::endl;
          gen_node++;
        }
        ++id;
      }     
    }

    return false;
  }
  bool getFirstConflict(
      std::vector<PlanResult<Location, Action, int> >& solution,
      Conflict& result){
    std::vector<PlanResult<Location, Action, int>> solution_path;
    solution_path.reserve(solution.size());
    for(size_t i = 0; i < solution.size(); i++){
      int tt = 0;
      Location a(-1, -1), b(-1, -1); 
      int time_a, time_b;
      for(size_t jump_point_id = 0; jump_point_id < solution[i].states.size() - 1; jump_point_id++){
        a = solution[i].states[jump_point_id].first;
        b = solution[i].states[jump_point_id + 1].first;
        time_a = solution[i].states[jump_point_id].second;
        time_b = solution[i].states[jump_point_id + 1].second;
        int delta_t = time_b - time_a;
        int flag = 1;
        Action ac_c;
        if(a.y < b.y) { flag = -1; ac_c = Action::Down;}
        else ac_c = Action::Up;
        
        for(int temp_y = 0; temp_y < abs(a.y - b.y); temp_y++){
          Location temp_loc(a.x, a.y+flag*temp_y);
          solution_path[i].states.push_back(std::make_pair<>(temp_loc, time_a + temp_y));
          solution_path[i].actions.push_back(std::make_pair<>(ac_c, 1));
        }
        if(a.x < b.x){flag = 1; ac_c = Action::Right;}
        else{flag = -1; ac_c = Action::Left;}
        for(int temp_x = 0; temp_x < abs(a.x - b.x); temp_x++){
          Location temp_loc(a.x + flag*temp_x, b.y);
          solution_path[i].states.push_back(std::make_pair<>(temp_loc, time_a + abs(a.y - b.y)+temp_x));
          solution_path[i].actions.push_back(std::make_pair<>(ac_c, 1));
        } 
        if(delta_t != abs(a.x - b.x) + abs(a.y - b.y)){
          for(int temp_w = 0; temp_w  < delta_t - abs(a.x - b.x) - abs(a.y - b.y); temp_w++){
            if(a.x == b.x) Location(temp_loc)
            solution_path[i].states.push_back(std::make_pair<>(temp_loc, time_a + abs(a.y - b.y)+temp_x));
          }
        }
      }
    }
  }

 private:
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


  struct HighLevelNodeJps {
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

  struct LowLevelEnvironment {
    LowLevelEnvironment(Environment& env, size_t agentIdx,
                        const Constraints& constraints)
        : m_env(env)
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
