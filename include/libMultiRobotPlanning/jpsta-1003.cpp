
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

    struct rusage r_usage;

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
        std::cout << " ,done, time-out fail" << ", num_node, " << num_node << " , gen_node, " << gen_node << ", ";
    	  return false;
      }
      
      if(num_node % 100 == 0){
        getrusage(RUSAGE_SELF, &r_usage);
        // std::cout << r_usage.ru_maxrss << " memory \n"; 
        if(r_usage.ru_maxrss > 15204352){
          std::cout << " ,done, memory-out fail" << ", num_node, " << num_node << " , gen_node, " << gen_node << ", ";          
          return false;
        }
      }      

      HighLevelNodeJps PJps = openJps.top();
      m_env.onExpandHighLevelNode(PJps.cost);
      openJps.pop();

      Conflict conflict;
      int jump_id_clf = -1;
      int return_value = getFirstConflict(PJps.solution, conflict, PJps);
      if(return_value  == 0){
        solution = PJps.solution;
        if(!m_env.CheckValid(PJps.solution)){
          std::cout << "Check solution falis \n";
          return false;
        }
        std::cout << " ,done, cost, " << PJps.cost << ", num_node, " << num_node << " , gen_node, " << gen_node << ", ";
        return true;
      }
 
      std::map<size_t, Constraints> constraints;
      m_env.createConstraintsFromConflict(conflict, constraints);
      for (const auto& c : constraints) {
        size_t i = c.first;
        HighLevelNodeJps newNodeJps = PJps;
        newNodeJps.id = id;
        newNodeJps.agent_id = i;
        
        bool is_debug_print = false;

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
        		jpstbit.setCollisionVertex(location, constraint.time, constraint.time, true);            
        		is_first_constraint_v = false;
        	}else{
        		jpstbit.setCollisionVertex(location, constraint.time, constraint.time, false);            
        	}
        }
        
        for(auto & constraint : newNodeJps.constraints[i].edgeConstraints){
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

  bool getFirstConflict(
      std::vector<PlanResult<Location, Action, int> >& solution,
      Conflict& result, HighLevelNodeJps& CurNode){
    std::vector<PlanResult<Location, Action, int>> solution_path(solution.size());
    std::vector<std::vector<int>> point_id(solution.size());
    std::vector<std::vector<int>> point_st(solution.size());
    
    int max_t = 0;
    for(size_t i = 0; i < solution.size(); i++){ //recover the path
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
        for(int temp_y = 0; temp_y < abs(a.y - b.y); temp_y++){ //from zero insure the first location is added
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
        for(int temp_x = 1; temp_x < abs(a.x - b.x); temp_x++){// from 1 insure the last location is not added
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
          if(a.x == b.x && a.y == b.y) temp_loc = a;
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
            bool is_update = false;
            for(int ct = 0; ct < 1; ct++){
              int current_id = -1;
              if(ct == 0) current_id = i;
              else current_id = j;
              if(t > point_id[current_id].size() - 1) continue;
              int JumpPointId = point_id[current_id][t];
              int PreJumpPointId = -1;
              int AftJumpPointID = -1;
              if(t - 1 >= 0){
                if(point_id[current_id][t - 1] != JumpPointId) continue;
              }
              if(t + 1 < point_id[current_id].size()){
                if(point_id[current_id][t + 1] != JumpPointId) continue;
              }
              if(JumpPointId + 1 > solution[current_id].states.size()- 1) continue;
              
              Location goalLoc = solution[current_id].states[JumpPointId+1].first;
              int time_a = solution[current_id].states[JumpPointId].second;
              int time_b = solution[current_id].states[JumpPointId + 1].second;
              int cost_t = time_b - time_a;
            
              State initialState(-1, -1, time_a);
              initialState.x = solution[current_id].states[JumpPointId].first.x;
              initialState.y = solution[current_id].states[JumpPointId].first.y;
              initialState.time = time_a;
              if(abs(initialState.x - goalLoc.x) + abs(initialState.y - goalLoc.y) == 1) continue;
              if(initialState.x == goalLoc.x || initialState.y == goalLoc.y //stright line
               || jump_point[current_id].find(JumpPointId) != jump_point[current_id].end()) continue;  // have replanned

              Constraints constraints;
              m_env.createConstraintsFromV(t, state1.x, state1.y, constraints);
              assert(!CurNode.constraints[current_id].overlap(constraints));
              CurNode.constraints[current_id].add(constraints);
              
              m_env.resetTemporalObstacle();
              for(auto & constraint : CurNode.constraints[current_id].vertexConstraints){
        	      Location location(constraint.x, constraint.y);
        	      m_env.setTemporalObstacle(location, constraint.time);
              }
              for(auto & constraint : CurNode.constraints[current_id].edgeConstraints){
        	      Location location(constraint.x2, constraint.y2);
        	      m_env.setTemporalEdgeConstraint(location, constraint.time);
              }

              if(JumpPointId + 1 == solution[current_id].states.size() - 1) m_env.setIsSegPlanning(false);
              else m_env.setIsSegPlanning(true);
              LowLevelEnvironment llenv(m_env, solution_path, current_id, goalLoc, CurNode.constraints[current_id]);
              LowLevelSearch_t lowLevel(llenv);            
              PlanResult<State, Action, int>segmentPath;
              bool success = lowLevel.search(initialState, segmentPath, 0, cost_t);
              jump_point[current_id].insert(JumpPointId); 
              m_env.setIsSegPlanning(false);
              CurNode.constraints[current_id].vertexConstraints.erase(*constraints.vertexConstraints.begin());
              bool is_update = true;

              if(segmentPath.cost == cost_t){
                size_t jjj = 0;
                bool new_conflict = false;
                int num_old = 0, num_new = 0;
                for(size_t iii = time_a; iii < time_b; ++iii){
                  for(int agentId = 0; agentId < solution.size(); agentId++){
                    if(agentId == current_id) continue;
                    if(solution_path[agentId].states[iii].first.x == segmentPath.states[jjj].first.x 
                       && solution_path[agentId].states[iii].first.y == segmentPath.states[jjj].first.y) num_new++;
                    if(solution_path[agentId].states[iii].first.x == solution_path[current_id].states[iii].first.x 
                       && solution_path[agentId].states[iii].first.y == solution_path[current_id].states[iii].first.y) num_old++;

                  }
                  jjj++;
                }
                // if(num_new > 0) continue;
                if(num_old <= num_new) continue;
                jjj = 0;
                for(size_t iii = time_a; iii < time_b; ++iii){
                  solution_path[current_id].states[iii].first.x = segmentPath.states[jjj].first.x;
                  solution_path[current_id].states[iii].first.y = segmentPath.states[jjj].first.y;
                  solution_path[current_id].states[iii].second = iii;
                  solution_path[current_id].actions[iii] = segmentPath.actions[jjj];
                  jjj++;
                }

                auto it = solution[current_id].states.begin();
                auto it_ac = solution[current_id].actions.begin();
                solution[current_id].states.insert(it + JumpPointId + 1, solution_path[current_id].states.begin() + time_a + 1,
                  solution_path[current_id].states.begin() + time_b);

                solution[current_id].actions.insert(it_ac + JumpPointId + 1, solution_path[current_id].actions.begin() + time_a + 1,
                  solution_path[current_id].actions.begin() + time_b);              

                is_restart = true;
                t = time_a - 1;
                is_update = true;
                break;
              } else continue;
              if(is_update) break;
            }
            if(!is_update) return true;
            
            // if(t > point_id[i].size() - 1) return true;
            // int JumpPointId = point_id[i][t];
            // int PreJumpPointId = -1;
            // int AftJumpPointID = -1;
            // if(t - 1 >= 0){
            //   if(point_id[i][t - 1] != JumpPointId) return true;
            // }
            // if(t + 1 < solution_path.size()){
            //   if(point_id[i][t + 1] != JumpPointId) return true;
            // }
            // if(JumpPointId + 1 > solution[i].states.size()-1) return true;

            // Location goalLoc = solution[i].states[JumpPointId+1].first;
            // int time_a = solution[i].states[JumpPointId].second;
            // int time_b = solution[i].states[JumpPointId + 1].second;
            // int cost_t = time_b - time_a;
            
            // State initialState(-1, -1, time_a);
            // initialState.x = solution[i].states[JumpPointId].first.x;
            // initialState.y = solution[i].states[JumpPointId].first.y;
            // initialState.time = time_a;
            // if(abs(initialState.x - goalLoc.x) + abs(initialState.y - goalLoc.y) == 1) return true;
            // if(initialState.x == goalLoc.x || initialState.y == goalLoc.y //stright line
            //    || jump_point[i].find(JumpPointId) != jump_point[i].end()) return true;  // have replanned

            // m_env.resetTemporalObstacle();

            // Constraints constraints;
            // m_env.createConstraintsFromV(t, state1.x, state1.y, constraints);
            // assert(!CurNode.constraints[i].overlap(constraints));
            // CurNode.constraints[i].add(constraints);

            // for(auto & constraint : CurNode.constraints[i].vertexConstraints){
        	  //   Location location(constraint.x, constraint.y);
        	  //   m_env.setTemporalObstacle(location, constraint.time);
            // }
            // for(auto & constraint : CurNode.constraints[i].edgeConstraints){
        	  //   Location location(constraint.x2, constraint.y2);
        	  //   m_env.setTemporalEdgeConstraint(location, constraint.time);
            // }

            // if(JumpPointId + 1 == solution[i].states.size() - 1) m_env.setIsSegPlanning(false);
            // else m_env.setIsSegPlanning(true);
            // LowLevelEnvironment llenv(m_env, solution_path, i, goalLoc, CurNode.constraints[i]);
            // LowLevelSearch_t lowLevel(llenv);            
            // PlanResult<State, Action, int>segmentPath;
            // bool success = lowLevel.search(initialState, segmentPath, 0, cost_t);
            // jump_point[i].insert(JumpPointId); 
            // m_env.setIsSegPlanning(false);

            // CurNode.constraints[i].vertexConstraints.erase(*constraints.vertexConstraints.begin());

            // bool is_update = true;
            // if(segmentPath.cost == cost_t){
            //   size_t jjj = 0;
            //   bool new_conflict = false;
            //   int num_old = 0, num_new = 0;
            //   for(size_t iii = time_a; iii < time_b; ++iii){
            //     for(int agentId = 0; agentId < solution.size(); agentId++){
            //       if(agentId == i) continue;
            //       if(solution_path[agentId].states[iii].first.x == segmentPath.states[jjj].first.x 
            //          && solution_path[agentId].states[iii].first.y == segmentPath.states[jjj].first.y) num_new++;
            //       if(solution_path[agentId].states[iii].first.x == solution_path[i].states[iii].first.x 
            //          && solution_path[agentId].states[iii].first.y == solution_path[i].states[iii].first.y) num_old++;

            //     }
            //     jjj++;
            //   }
            //   if(num_new > 0) return true;
            //   // if(num_old < num_new) return true;
            //   jjj = 0;
            //   for(size_t iii = time_a; iii < time_b; ++iii){
            //     solution_path[i].states[iii].first.x = segmentPath.states[jjj].first.x;
            //     solution_path[i].states[iii].first.y = segmentPath.states[jjj].first.y;
            //     solution_path[i].states[iii].second = iii;
            //     solution_path[i].actions[iii] = segmentPath.actions[jjj];
            //     jjj++;
            //   }

            //   auto it = solution[i].states.begin();
            //   auto it_ac = solution[i].actions.begin();
            //   solution[i].states.insert(it + JumpPointId + 1, solution_path[i].states.begin() + time_a + 1,
            //   solution_path[i].states.begin() + time_b);

            //   solution[i].actions.insert(it_ac + JumpPointId + 1, solution_path[i].actions.begin() + time_a + 1,
            //   solution_path[i].actions.begin() + time_b);              

            //   is_restart = true;
            //   t = time_a - 1;
            //   break;
            // } else return true;  
            
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
            if(t > point_id[i].size() - 1) return true;
            int JumpPointId = point_id[i][t];
            if(JumpPointId + 1 > solution[i].states.size()-1) return true;            

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

            Constraints constraints;
            m_env.createConstraintsFromE(t, state1a.x, state1a.y, state1b.x, state1b.y, constraints);
            assert(!CurNode.constraints[i].overlap(constraints));
            CurNode.constraints[i].add(constraints);

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
            LowLevelEnvironment llenv(m_env, solution_path, i, goalLoc, CurNode.constraints[i]);
            LowLevelSearch_t lowLevel(llenv);            
            PlanResult<State, Action, int>segmentPath;
            bool success = lowLevel.search(initialState, segmentPath, 0, cost_t);
            jump_point[i].insert(JumpPointId);
            m_env.setIsSegPlanning(false);

            CurNode.constraints[i].edgeConstraints.erase(*constraints.edgeConstraints.begin());

            if(segmentPath.cost == cost_t){

              size_t jjj = 0;
              bool new_conflict = false;
              int num_old = 0, num_new = 0;
              for(size_t iii = time_a; iii < time_b; ++iii){
                for(int agentId = 0; agentId < solution.size(); agentId++){
                  if(agentId == i) continue;
                  if(solution_path[agentId].states[iii].first.x == segmentPath.states[jjj].first.x 
                     && solution_path[agentId].states[iii].first.y == segmentPath.states[jjj].first.y) num_new++;
                  if(solution_path[agentId].states[iii].first.x == solution_path[i].states[iii].first.x 
                     && solution_path[agentId].states[iii].first.y == solution_path[i].states[iii].first.y) num_old++;

                }
                jjj++;
              }
              if(num_new > 0) return true;

              jjj = 0;
              for(size_t iii = time_a; iii < time_b; ++iii){
                solution_path[i].states[iii].first.x = segmentPath.states[jjj].first.x;
                solution_path[i].states[iii].first.y = segmentPath.states[jjj].first.y;
                solution_path[i].states[iii].second = iii;
                solution_path[i].actions[iii] = segmentPath.actions[jjj];                
                jjj++;
              }

              auto it = solution[i].states.begin();
              auto it_ac = solution[i].actions.begin();
              solution[i].states.insert(it + JumpPointId + 1, solution_path[i].states.begin() + time_a + 1,
              solution_path[i].states.begin() + time_b);
              solution[i].actions.insert(it_ac + JumpPointId + 1, solution_path[i].actions.begin() + time_a + 1,
              solution_path[i].actions.begin() + time_b);              


              is_restart = true;
              t = time_a - 1;
              break;
            }else return true;
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
    LowLevelEnvironment(Environment& env, std::vector<PlanResult<Location, Action, Cost>>& cat, 
                        size_t agentIdx, const Constraints& constraints)
        : m_env(env), m_cat(cat)
    // , m_agentIdx(agentIdx)
    // , m_constraints(constraints)
    {
      m_env.setLowLevelContext(agentIdx, &constraints);
    }
    LowLevelEnvironment(Environment& env, std::vector<PlanResult<Location, Action, Cost>>& cat, 
                        size_t agentIdx, Location loc,
                        const Constraints& constraints)
        : m_env(env), m_cat(cat)
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
      if(m_env.isCAT){
        for(size_t nei = 0; nei < neighbors.size(); nei++){
          neighbors[nei].state.nc_cat = 0;
          int current_time = s.time + 1;
          Location temp_s(-1, -1), temp_s_p(-1, -1);
          for(size_t agent_id = 0; agent_id < m_cat.size(); agent_id++){
            if(m_cat[agent_id].states.empty()) continue;
            if(agent_id = m_env.getAgentId()) continue;
            if (current_time < m_cat[agent_id].states.size()) {
              temp_s = m_cat[agent_id].states[current_time].first;
            }else{
              temp_s = m_cat[agent_id].states.back().first;     
            }
            if(temp_s.x == neighbors[nei].state.x && temp_s.y == neighbors[nei].state.y){
              neighbors[nei].state.nc_cat++;
              // std::cout << "Here \n";
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
    std::vector<PlanResult<Location, Action, Cost>>& m_cat;
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