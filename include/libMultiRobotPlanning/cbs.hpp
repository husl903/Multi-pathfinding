
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
          
class CBS {
  public:
 
 public:
  CBS(Environment& environment) : m_env(environment) {}

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
 

    typename boost::heap::d_ary_heap<HighLevelNodeJps, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >
        openJps;
    while(!startJps.conflicts_all.empty()) startJps.conflicts_all.pop();
    getAllConflicts(startJps.solution, startJps.conflicts_all, startJps.num_conflict);

    auto handleJps = openJps.push(startJps);
    (*handleJps).handle = handleJps;

    solution.clear();
    int id = 1;    
    Timer timer;
    timer.reset();
    int num_node = 0;
    int gen_node = 0;
    struct rusage r_usage;

    int num_try_bypss = 0;
    while(!openJps.empty()){
      num_node++;
      // if(num_node > 50) return false;
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

      int jump_id_clf = -1;
      if(PJps.conflicts_all.size() == 0){
        solution = PJps.solution;
        if(m_env.CheckValid(PJps.solution)){
          std::cout << " ,done, " << PJps.cost << ", num_node, " << num_node << " , gen_node, " << gen_node << ", " << " num_open, " << id << ", ";
          return true;
        }else{
          std::cout << "Final results is not correct\n";
          return false;
        }
      }

      bool foundBypass = true;
      while(foundBypass){
        if(PJps.conflicts_all.size() == 0){
          if(!m_env.CheckValid(PJps.solution)){
            std::cout << "Check solution fails \n";
            return false;
          }else{
            std::cout << " ,done, " << PJps.cost << ", num_node, " << num_node << " , gen_node, " << gen_node << ", " << " num_open, " << id << ", ";
            return true;
          }
        }
        Conflict conflict_temp = PJps.conflicts_all.top();
        PJps.conflicts_all.pop();
        HighLevelNodeJps NewChild[2];
        bool is_solved[2] = {false, false};

        std::map<size_t, Constraints> constraints;
        m_env.createConstraintsFromConflict(conflict_temp, constraints);
        int child_id = 0;
        foundBypass = false;
        for(const auto& c : constraints){
          size_t i = c.first;
          NewChild[child_id].solution = PJps.solution;
          NewChild[child_id].constraints = PJps.constraints;
          NewChild[child_id].cost = PJps.cost;
          NewChild[child_id].id = id;

          assert(!NewChild[child_id].constraints[i].overlap(c.second));
          NewChild[child_id].constraints[i].add(c.second);
          NewChild[child_id].cost -= NewChild[child_id].solution[i].cost;

          m_env.resetTemporalObstacle();
          bool is_first_constraint_v = true;
          bool is_first_constraint_e = true;

          jpst_bit jpstbit(m_env);
          jpstbit.setEdgeCollisionSize(m_env.m_dimx, m_env.m_dimy);

          for(auto & constraint : NewChild[child_id].constraints[i].vertexConstraints){
        	  Location location(constraint.x, constraint.y);
        	  m_env.setTemporalObstacle(location, constraint.time);
        	  if(is_first_constraint_v){
        		  jpstbit.setCollisionVertex(location, constraint.time, constraint.time, true);            
        		  is_first_constraint_v = false;
        	  }else{
        		  jpstbit.setCollisionVertex(location, constraint.time, constraint.time, false);            
        	  }
          }
        
          for(auto & constraint : NewChild[child_id].constraints[i].edgeConstraints){
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
          is_solved[child_id] = jpstbit.search(startNode, Action::Wait, NewChild[child_id].solution[i], 0);
          timerJpstbit.stop();
          double tJpstbit = timerJpstbit.elapsedSeconds();
          int ExpJps1 = m_env.num_expansion;
          int GenJps1 = m_env.num_generation;
          
          if(!is_solved[child_id]) continue;

          while(!NewChild[child_id].conflicts_all.empty()) NewChild[child_id].conflicts_all.pop();          
          getAllConflicts(NewChild[child_id].solution, NewChild[child_id].conflicts_all, NewChild[child_id].num_conflict);

          if(m_env.isBP && NewChild[child_id].solution[i].cost == PJps.solution[i].cost 
             && NewChild[child_id].num_conflict < PJps.num_conflict){
            foundBypass = true;
            PJps.solution[i] = NewChild[child_id].solution[i];
            PJps.num_conflict = NewChild[child_id].num_conflict;
            while(!PJps.conflicts_all.empty()) PJps.conflicts_all.pop();
            PJps.conflicts_all = NewChild[child_id].conflicts_all;
            break;
          }
          NewChild[child_id].cost += NewChild[child_id].solution[i].cost;
          child_id++;
          gen_node++;
        }

        if(!foundBypass){
          for(int ii = 0; ii < 2; ii++){
            if(is_solved[ii]){
              NewChild[ii].id = id;
              auto handle = openJps.push(NewChild[ii]);
              (*handle).handle = handle;
              id++;
            }
          }
        }        
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
    std::priority_queue<Conflict> conflicts_all;
    int num_conflict = 0;

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

  bool TryBypassJpst(Conflict cft, HighLevelNodeJps& CurNode, int& jump_id, int& num_conflict){
    
    // std::cout << cft.agent1 << ", " << cft.agent2 << " \n";
    // std::cout << cft << std::endl;
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
    if(cft.x1 == start.x && cft.y1 == start.y) {
      return false;
      if(jump_id > 0) {
        jump_id -= 1;
        start = CurNode.solution[agentId].states[jump_id].first;
        time_a = CurNode.solution[agentId].states[jump_id].second;
        cost_t = time_b - time_a;
      }
      else return false;
    }
//    if(cft.x2 == start.x && cft.y2 == start.y) return false;
    if(cft.x1 == goalLoc.x && cft.y1 == goalLoc.y) {
      return false;
      if(next_jump_id + 1 < CurNode.solution[agentId].states.size()){
        next_jump_id += 1;
        goalLoc = CurNode.solution[agentId].states[next_jump_id].first;
        time_b = CurNode.solution[agentId].states[next_jump_id].second;
        cost_t = time_b - time_a;
      }else return false;
    }
//    if(cft.x2 == goalLoc.x && cft.y2 == goalLoc.y) return false;

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
                    
      // for (size_t ii = 0; ii < CurNode.solution[agentId].actions.size(); ++ii) {
      //   std::cout << CurNode.solution[agentId].states[ii].second << ": " <<
      // 			CurNode.solution[agentId].states[ii].first << "->" << CurNode.solution[agentId].actions[ii].first
			// 	  << "(cost: " << CurNode.solution[agentId].actions[ii].second << ")" << std::endl;
      // }
      // std::cout << CurNode.solution[agentId].states.back().second << ": " <<
      // 		   CurNode.solution[agentId].states.back().first << std::endl;

      // for (size_t iii = 0; iii < segmentPathJPS.actions.size(); ++iii) {
      //   std::cout << segmentPathJPS.states[iii].second << ": " <<
      //   segmentPathJPS.states[iii].first << "->" << segmentPathJPS.actions[iii].first
      //   << "(cost: " << segmentPathJPS.actions[iii].second << ")" << std::endl;
      // }
      // std::cout << segmentPathJPS.states.back().second << ": " <<
      // segmentPathJPS.states.back().first << std::endl;  
      // for(auto & constraint : CurNode.constraints[agentId].vertexConstraints){
      //   std::cout << constraint << "\n";
      // }
      // for(auto & constraint : CurNode.constraints[agentId].edgeConstraints){
      //   std::cout << constraint << "\n";
      // }

      int num_conflict_new = 0;
      
      auto it = CurNode.solution[agentId].states.begin();
      auto it_ac = CurNode.solution[agentId].actions.begin();

      CurNode.solution[agentId].states[jump_id] = segmentPathJPS.states[0];;
      CurNode.solution[agentId].actions[jump_id] = segmentPathJPS.actions[0];


      // for(auto it = segmentPathJPS.states.begin() + 1; it != segmentPathJPS.states.end(); it++){
      //   std::cout << (*it).first << " here\n";
      // }
      if(abs(jump_id - next_jump_id) > 1){
        CurNode.solution[agentId].states.erase(it + jump_id + 1);
      }
      CurNode.solution[agentId].states.insert(it + jump_id + 1, segmentPathJPS.states.begin() + 1,
              segmentPathJPS.states.end() - 1);

      CurNode.solution[agentId].actions.insert(it_ac + jump_id + 1, segmentPathJPS.actions.begin() + 1,
              segmentPathJPS.actions.end());
      
      // std::cout << agentId << " Bypass here \n";

      // for (size_t ii = 0; ii < CurNode.solution[agentId].actions.size(); ++ii) {
      //   std::cout << CurNode.solution[agentId].states[ii].second << ": " <<
      // 			CurNode.solution[agentId].states[ii].first << "->" << CurNode.solution[agentId].actions[ii].first
			// 	  << "(cost: " << CurNode.solution[agentId].actions[ii].second << ")" << std::endl;
      // }
      // std::cout << CurNode.solution[agentId].states.back().second << ": " <<
      // 		   CurNode.solution[agentId].states.back().first << std::endl;

      //   std::cout << "Agent id " << agentId << " -------------------------------------------------\n";      
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
          if(m_env.isObstacle(temp_loc)){
            std::cout << "Obstacle is here " << "X, Y " << temp_loc.x << ", " << temp_loc.y << " jump_id " <<  a.x << ", " << a.y
             << " jump_id + 1 " << b.x << ", " << b.y   << " 222 "<< std::endl;
          }
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
            return true;
        
            int JumpPointId = point_id[i][t];
            int PreJumpPointId = -1;
            int AftJumpPointID = -1;
            if(t - 1 >= 0){
              if(point_id[i][t - 1] != JumpPointId) return true;
            }
            if(t + 1 < solution_path.size()){
              if(point_id[i][t + 1] != JumpPointId) return true;
            }
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
            return true;

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



              auto it = solution[i].states.begin();
              auto it_ac = solution[i].actions.begin();
              solution[i].states.insert(it + JumpPointId + 1, solution_path[i].states.begin() + time_a + 1,
              solution_path[i].states.begin() + time_b);
              solution[i].actions.insert(it_ac + JumpPointId + 1, solution_path[i].actions.begin() + time_a + 1,
              solution_path[i].actions.begin() + time_b);              
   

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
      Conflict& result, int& jump_id, int& num_conflict){
    std::vector<PlanResult<Location, Action, int>> solution_path(solution.size());
    std::vector<std::vector<int>> point_id(solution.size());
    std::vector<std::vector<int>> point_st(solution.size());
           
    int max_t = 0;
    num_conflict = 0;
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
        // std::cout <<"Agent " <<  i << "recover path is not correct " << tt - 1 << solution[i].cost << std::endl;
        return false;
      }
    }

    bool is_restart = false;
    std::vector<std::unordered_set<int>> jump_point(solution.size());
    result.time = -1;
    for (int t = 0; t < max_t; ++t) {
      is_restart = false;
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution_path.size(); ++i) {
        Location state1 = getState(i, solution_path, t);
        for (size_t j = i + 1; j < solution_path.size(); ++j) {
          Location state2 = getState(j, solution_path, t);
          if (state1 == state2) {
            if(result.time == -1){
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Vertex;
            result.x1 = state1.x;
            result.y1 = state1.y;
            if(t >= point_id[i].size()) jump_id = -1;
            else jump_id = point_id[i][t];
            }
            num_conflict++;
          }
        }
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
            if(result.time == -1){
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Edge;
            result.x1 = state1a.x;
            result.y1 = state1a.y;
            result.x2 = state1b.x;
            result.y2 = state1b.y;
            if(t >= point_id[i].size()) jump_id = -1;
            else jump_id = point_id[i][t];            
            }
            num_conflict++;
          }
        }
      }
    }

    if(result.time == -1) return 0;
    else return 1;    
  }


 bool  getAllConflicts(
      std::vector<PlanResult<Location, Action, int> >& solution,
      std::priority_queue<Conflict>& conflicts_all, int& num_cft){
    std::vector<PlanResult<Location, Action, int>> solution_path(solution.size());
    std::vector<std::vector<int>> point_id(solution.size());
    std::vector<std::vector<int>> point_st(solution.size());
    Conflict result;
    int max_t = 0;
    num_cft = 0;
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
        std::cout <<"Agent " <<  i << "recover path is not correct " << tt - 1 << solution[i].cost << std::endl;
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
            conflicts_all.push(result);
            num_cft++;
            // if(t >= point_id[i].size()) jump_id = -1;
            // else jump_id = point_id[i][t];
          }
        }
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
            num_cft++;
            // if(t >= point_id[i].size()) jump_id = -1;
            // else jump_id = point_id[i][t];
            conflicts_all.push(result);
          }
        }
      }
    }
    if(conflicts_all.size() > 0) return true;
    else return false;    
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
