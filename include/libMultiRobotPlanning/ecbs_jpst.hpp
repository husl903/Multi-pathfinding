#pragma once

#include <map>

#include "a_star_epsilon.hpp"
#include "JPSSIPP_BITECBS.hpp"
#include "timer.hpp"

namespace libMultiRobotPlanning {

/*!
  \example ecbs.cpp Example that solves the Multi-Agent Path-Finding (MAPF)
  problem in a 2D grid world with up/down/left/right
  actions
*/

/*! \brief Enhanced Conflict-Based-Search (ECBS) algorithm to solve the
Multi-Agent
Path-Finding (MAPF) problem within a given suboptimality bound

This class implements the Enhanced Conflict-Based-Search (ECBS) algorithm.
This algorithm can find collision-free path for multiple agents with start and
goal locations
given for each agent.
A user can provide a suboptimality factor w, and the cost of the returned
solution is guaranteed to smaller or equal than w * optimalCost.
ECBS is an extension of CBS. It uses the same two-level search with a focal
search on both levels.
The focal search uses a second user-provided inadmissible heuristic that
minimizes conflicts between agents.

Details of the algorithm can be found in the following paper:\n
Max Barer, Guni Sharon, Roni Stern, Ariel Felner:\n
"Suboptimal Variants of the Conflict-Based Search Algorithm for the Multi-Agent
Pathfinding Problem". SOCS 2014\n
http://www.aaai.org/ocs/index.php/SOCS/SOCS14/paper/view/8911

The underlying A*_epsilon algorithm can either use a fibonacci heap, or a d-ary
heap.
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

  - `Cost focalStateHeuristic(const State& s, int gScore, const
std::vector<PlanResult<State, Action, int> >& solution)`\n
    Potentially inadmissible focal heuristic for a state, e.g. count all
conflicts between the agents if the agent of the current context moves is at
state s

  - `Cost focalTransitionHeuristic(const State& s1a, const State& s1b, Cost
gScoreS1a, Cost gScoreS1b, const std::vector<PlanResult<State, Action, Cost> >&
solution)`\n
    Potentially inadmissible focal heuristic for a state transition, e.g. count
all conflicts between the agents if the agent of the current context moves from
s1a to s1b

  - `Cost focalHeuristic(const std::vector<PlanResult<State, Action, int> >&
solution)`\n
    Potentially inadmissible focal heuristic, e.g. count all conflicts between
the agents for a given solution

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

\sa CBS

*/
template <typename State, typename Location, typename Action, typename Cost, typename Conflict,
          typename Constraints, typename Environment>
class ECBSJPST {
 public:
  ECBSJPST(Environment& environment, float w) : m_env(environment), m_w(w) {}

  bool search(const std::vector<State>& initialStates,
              std::vector<PlanResult<Location, Action, Cost> >& solution) {
    HighLevelNode start;
    start.solution.resize(initialStates.size());
    start.constraints.resize(initialStates.size());
    start.cost = 0;
    start.LB = 0;
    start.id = 0;
    int num_node = 0;
    for (size_t i = 0; i < initialStates.size(); ++i) {
      if (i < solution.size() && solution[i].states.size() > 1) {
        std::cout << initialStates[i] << " " << solution[i].states.front().first
                  << std::endl;
        // assert(initialStates[i] == solution[i].states.front().first);
        start.solution[i] = solution[i];
        std::cout << "use existing solution for agent: " << i << std::endl;
      } else {
//        LowLevelEnvironment llenv(m_env, i, start.constraints[i],
//                                  start.solution);
//        LowLevelSearch_t lowLevel(llenv, m_w);
//        bool success = lowLevel.search(initialStates[i], start.solution[i]);
        // std::cout << "Cost astar " << start.solution[i].cost;
        // if (!success) {
        //   return false;
        // }
        jpst_bit jps1(m_env, start.solution, m_w);
        // std::cout << "m_w " << m_w << std::endl;
        jps1.setEdgeCollisionSize(m_env.getDimX(), m_env.getDimY());      
        Location goal = m_env.setGoal(i);
        Location startNode(-1, -1);
        startNode.x = initialStates[i].x;
        startNode.y = initialStates[i].y;       
        bool isJpsSucc = jps1.search(startNode, Action::Wait, start.solution[i], 0);
        // std::cout << " jpst, " << start.solution[i].cost << std::endl;
        if(!isJpsSucc){
          return false;
        }
      }
      start.cost += start.solution[i].cost;
      start.LB += start.solution[i].fmin;
    }
    start.focalHeuristic = start.conflicts_all.size();
    std::vector<Conflict> empty_1;
    start.conflicts_all.swap(empty_1);
    m_env.getAllConflicts(start.solution, start.conflicts_all);

    std::cout << "Initial cost " << start.cost << ", " << start.conflicts_all.size() << " \n";
    // std::priority_queue<HighLevelNode> open;
    openSet_t open;
    focalSet_t focal;

    auto handle = open.push(start);
    (*handle).handle = handle;
    focal.push(handle);

    Cost bestCost = (*handle).cost;

    solution.clear();
    int id = 1;
    Timer timer;
    timer.reset();    
    while (!open.empty()) {
      num_node++;
      timer.stop();
      double duration1 = timer.elapsedSeconds();
      if(duration1 > 100){
        std::cout << " ,done, time-out fail " << ", num_node, " << num_node << ", " << " num_open, " << id << ", ";
    	  return false;
      }

// update focal list
#ifdef REBUILT_FOCAL_LIST
      focal.clear();
      Cost LB = open.top().LB;

      auto iter = open.ordered_begin();
      auto iterEnd = open.ordered_end();
      for (; iter != iterEnd; ++iter) {
        float val = iter->cost;
        // std::cout << "  cost: " << val << std::endl;
        if (val <= LB * m_w) {
          const HighLevelNode& node = *iter;
          focal.push(node.handle);
        } else {
          break;
        }
      }
#else
      {
        Cost oldBestCost = bestCost;
        bestCost = open.top().cost;
        // std::cout << "bestFScore: " << bestFScore << std::endl;
        if (bestCost > oldBestCost) {
          // std::cout << "oldBestCost: " << oldBestCost << " bestCost: " <<
          // bestCost << std::endl;
          auto iter = open.ordered_begin();
          auto iterEnd = open.ordered_end();
          for (; iter != iterEnd; ++iter) {
            Cost val = iter->cost;
            if (val > oldBestCost * m_w && val <= bestCost * m_w) {
              const HighLevelNode& n = *iter;
              focal.push(n.handle);
            }
            if (val > bestCost * m_w) {
              break;
            }
          }
        }
      }
#endif
// check focal list/open list consistency
#ifdef CHECK_FOCAL_LIST
      {
        // focalSet_t focalSetGolden;
        bool mismatch = false;
        const auto& top = open.top();
        Cost bestCost = top.cost;
        auto iter = open.ordered_begin();
        auto iterEnd = open.ordered_end();
        for (; iter != iterEnd; ++iter) {
          const auto& s = *iter;
          Cost val = s.cost;
          if (val <= bestCost * m_w) {
            // std::cout << "should: " << s << std::endl;
            // focalSetGolden.push(s.handle);
            if (std::find(focal.begin(), focal.end(), s.handle) ==
                focal.end()) {
              std::cout << "focal misses: " << s << std::endl;
              mismatch = true;
            }

          } else {
            if (std::find(focal.begin(), focal.end(), s.handle) !=
                focal.end()) {
              std::cout << "focalSet shouldn't have: " << s << std::endl;
              mismatch = true;
            }
            // break;
          }
        }
        assert(!mismatch);
        // assert(focalSet == focalSetGolden);
      }
#endif

      auto h = focal.top();
      HighLevelNode P = *h;
      m_env.onExpandHighLevelNode(P.cost);
      // std::cout << "expand: " << P << std::endl;

      focal.pop();
      open.erase(h);

      bool foundBypass = true;
      while(foundBypass){
        if(P.conflicts_all.size() == 0){
          if(!m_env.CheckValid(P.solution)){
            std::cout << "Check solution fails\n";
            return false;
          }else{
            std::cout << " ,done " << P.cost << ", num_node " << num_node << ",num_open " << id << ", ";
            // std::cout << " ,done, " << P.cost << ", num_node, " << num_node << " , gen_node, " << gen_node << ", " << " num_open, " << id << ", ";
            return true;
          }
        }

        // if(P.conflicts_all.size() == 0) return true;
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
          bool is_first_constraint_v = true;
          bool is_first_constraint_e = true;
          
          jpst_bit jpstbit(m_env, NewChild[child_id].solution, m_w);
          jpstbit.setEdgeCollisionSize(m_env.getDimX(), m_env.getDimY());
          for(auto & constraint : NewChild[child_id].constraints[i].vertexConstraints){
        	  Location location(constraint.x, constraint.y);
        	  m_env.setTemporalObstacle(location, constraint.time);
            jpstbit.setCollisionVertex(location, constraint.time, constraint.time, is_first_constraint_v);  
        	  if(is_first_constraint_v){
        		  is_first_constraint_v = false;
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
          m_env.setGoal(i);
          Location startNode(initialStates[i].x, initialStates[i].y);

          is_solved[child_id] = jpstbit.search(startNode, Action::Wait, NewChild[child_id].solution[i], 0);
          if(!is_solved[child_id]) { child_id++; continue;}

          NewChild[child_id].conflicts_all.clear();
          if(!NewChild[child_id].conflicts_all.empty()) NewChild[child_id].conflicts_all.swap(empty_1);
          m_env.getAllConflicts(NewChild[child_id].solution, NewChild[child_id].conflicts_all);
          // gen_node++;
          if(m_env.isBP && NewChild[child_id].solution[i].cost == P.solution[i].cost 
             && NewChild[child_id].conflicts_all.size() < P.conflicts_all.size()){
            foundBypass = true;
            P.solution[i] = NewChild[child_id].solution[i];
            P.focalHeuristic = NewChild[child_id].conflicts_all.size();
            P.conflicts_all.clear();
            P.conflicts_all.swap(empty_1);

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
              if (NewChild[ii].cost <= bestCost * m_w) {
                focal.push(handle);
              }
              id++;
            }
          }
        }         
      }

      Conflict conflict;
      if (P.conflicts_all.size() == 0) {
        std::cout << "done; cost: " << P.cost << "," << num_node << std::endl;
        solution = P.solution;
        return true;
      }
      
      continue;
      conflict = P.conflicts_all[0];
      // create additional nodes to resolve conflict
      std::cout << "Found conflict: " << conflict << std::endl;
      // std::cout << "Found conflict at t=" << conflict.time << " type: " <<
      // conflict.type << std::endl;

      std::map<size_t, Constraints> constraints;
      m_env.createConstraintsFromConflict(conflict, constraints);
      for (const auto& c : constraints) {
        // std::cout << "Add HL node for " << c.first << std::endl;
        size_t i = c.first;
        std::cout << "create child with id " << id << std::endl;
        HighLevelNode newNode = P;
        newNode.id = id;
        // (optional) check that this constraint was not included already
        // std::cout << newNode.constraints[i] << std::endl;
        // std::cout << c.second << std::endl;
        assert(!newNode.constraints[i].overlap(c.second));

        newNode.constraints[i].add(c.second);

        newNode.cost -= newNode.solution[i].cost;
        newNode.LB -= newNode.solution[i].fmin;

        // LowLevelEnvironment llenv(m_env, i, newNode.constraints[i],
        //                           newNode.solution);
        // LowLevelSearch_t lowLevel(llenv, m_w);
        // bool success = lowLevel.search(initialStates[i], newNode.solution[i]);
        bool success = true;
        m_env.resetTemporalObstacle();
        bool is_first_constraint_v = true;
        bool is_first_constraint_e = true;

        jpst_bit jpstbit(m_env, newNode.solution, m_w);
        jpstbit.setEdgeCollisionSize(m_env.getDimX(), m_env.getDimY());
        for(auto & constraint : newNode.constraints[i].vertexConstraints){
          Location location(constraint.x, constraint.y);
          m_env.setTemporalObstacle(location, constraint.time);
        	if(is_first_constraint_v){
        	  jpstbit.setCollisionVertex(location, constraint.time, constraint.time, true);            
        		is_first_constraint_v = false;
        	}else{
        	  jpstbit.setCollisionVertex(location, constraint.time, constraint.time, false);            
        	}
        }
        for(auto & constraint : newNode.constraints[i].edgeConstraints){
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
        m_env.setGoal(i);
        Location startNode(-1, -1);
        startNode.x = initialStates[i].x;
        startNode.y = initialStates[i].y;
        success = jpstbit.search(startNode, Action::Wait, newNode.solution[i], 0);
        newNode.conflicts_all.clear();
        if(newNode.conflicts_all.empty()) newNode.conflicts_all.swap(empty_1);
        m_env.getAllConflicts(newNode.solution, newNode.conflicts_all);


        newNode.cost += newNode.solution[i].cost;
        newNode.LB += newNode.solution[i].fmin;
        newNode.focalHeuristic = newNode.conflicts_all.size();

        if (success) {
          std::cout << "  success. cost: " << newNode.cost << std::endl;
          auto handle = open.push(newNode);
          (*handle).handle = handle;
          if (newNode.cost <= bestCost * m_w) {
            focal.push(handle);
          }
        }

        ++id;
      }
    }

    return false;
  }

 private:
  struct HighLevelNode;

#ifdef USE_FIBONACCI_HEAP
  typedef typename boost::heap::fibonacci_heap<HighLevelNode> openSet_t;
  typedef typename openSet_t::handle_type handle_t;
// typedef typename boost::heap::fibonacci_heap<fibHeapHandle_t,
// boost::heap::compare<compareFocalHeuristic> > focalSet_t;
#else
  typedef typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                           boost::heap::mutable_<true> >
      openSet_t;
  typedef typename openSet_t::handle_type handle_t;
// typedef typename boost::heap::d_ary_heap<fibHeapHandle_t,
// boost::heap::arity<2>, boost::heap::mutable_<true>,
// boost::heap::compare<compareFocalHeuristic> > focalSet_t;
#endif

  struct HighLevelNode {
    std::vector<PlanResult<Location, Action, Cost> > solution;
    std::vector<Constraints> constraints;

    Cost cost;
    Cost LB;  // sum of fmin of solution

    Cost focalHeuristic;
    std::vector<Conflict> conflicts_all;
    int id;

    handle_t handle;

    bool operator<(const HighLevelNode& n) const {
      // if (cost != n.cost)
      return cost > n.cost;
      // return id > n.id;
    }

    friend std::ostream& operator<<(std::ostream& os, const HighLevelNode& c) {
      os << "id: " << c.id << " cost: " << c.cost << " LB: " << c.LB
         << " focal: " << c.focalHeuristic << std::endl;
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

  struct compareFocalHeuristic {
    bool operator()(const handle_t& h1, const handle_t& h2) const {
      // Our heap is a maximum heap, so we invert the comperator function here
      if ((*h1).focalHeuristic != (*h2).focalHeuristic) {
        return (*h1).focalHeuristic > (*h2).focalHeuristic;
      }
      return (*h1).cost > (*h2).cost;
    }
  };

#ifdef USE_FIBONACCI_HEAP
  typedef typename boost::heap::fibonacci_heap<
      openSet_t, boost::heap::compare<compareFocalHeuristic> >
      focalSet_t;
#else
  typedef typename boost::heap::d_ary_heap<
      handle_t, boost::heap::arity<2>, boost::heap::mutable_<true>,
      boost::heap::compare<compareFocalHeuristic> >
      focalSet_t;
#endif

  struct LowLevelEnvironment {
    LowLevelEnvironment(
        Environment& env, size_t agentIdx, const Constraints& constraints,
        const std::vector<PlanResult<State, Action, Cost> >& solution)
        : m_env(env)
          // , m_agentIdx(agentIdx)
          // , m_constraints(constraints)
          ,
          m_solution(solution) {
      m_env.setLowLevelContext(agentIdx, &constraints);
    }

    Cost admissibleHeuristic(const State& s) {
      return m_env.admissibleHeuristic(s);
    }

    Cost focalStateHeuristic(const State& s, Cost gScore) {
      return m_env.focalStateHeuristic(s, gScore, m_solution);
    }

    Cost focalTransitionHeuristic(const State& s1, const State& s2,
                                  Cost gScoreS1, Cost gScoreS2) {
      return m_env.focalTransitionHeuristic(s1, s2, gScoreS1, gScoreS2,
                                            m_solution);
    }

    bool isSolution(const State& s) { return m_env.isSolution(s); }

    void getNeighbors(const State& s,
                      std::vector<Neighbor<State, Action, Cost> >& neighbors) {
      m_env.getNeighbors(s, neighbors);
    }

    void onExpandNode(const State& s, Cost fScore, Cost gScore) {
      // std::cout << "LL expand: " << s << " fScore: " << fScore << " gScore: "
      // << gScore << std::endl;
      // m_env.onExpandLowLevelNode(s, fScore, gScore, m_agentIdx,
      // m_constraints);
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
    const std::vector<PlanResult<State, Action, Cost> >& m_solution;
  };

 private:
  Environment& m_env;
  float m_w;
  typedef AStarEpsilon<State, Action, Cost, LowLevelEnvironment>
      LowLevelSearch_t;
  typedef JPSSIPP_BITECBS<Location, Location, Action, int, Environment> jpst_bit;

};

}  // namespace libMultiRobotPlanning
