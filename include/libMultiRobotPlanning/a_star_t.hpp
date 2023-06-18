#pragma once

#ifdef USE_FIBONACCI_HEAP
#include <boost/heap/fibonacci_heap.hpp>
#endif

#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
#include <unordered_set>

#include "neighbor.hpp"
#include "planresult.hpp"
#include "timer.hpp"

namespace libMultiRobotPlanning {

/*!
  \example a_star.cpp Simple example using a 2D grid world and
  up/down/left/right
  actions
*/

/*! \brief A* Algorithm to find the shortest path

This class implements the A* algorithm. A* is an informed search algorithm
that finds the shortest path for a given map. It can use a heuristic that
needsto be admissible.

This class can either use a fibonacci heap, or a d-ary heap. The latter is the
default. Define "USE_FIBONACCI_HEAP" to use the fibonacci heap instead.

\tparam State Custom state for the search. Needs to be copy'able
\tparam Action Custom action for the search. Needs to be copy'able
\tparam Cost Custom Cost type (integer or floating point types)
\tparam Environment This class needs to provide the custom A* logic. In
    particular, it needs to support the following functions:
  - `Cost admissibleHeuristic(const State& s)`\n
    This function can return 0 if no suitable heuristic is available.

  - `bool isSolution(const State& s)`\n
    Return true if the given state is a goal state.

  - `void getNeighbors(const State& s, std::vector<Neighbor<State, Action,
   int> >& neighbors)`\n
    Fill the list of neighboring state for the given state s.

  - `void onExpandNode(const State& s, int fScore, int gScore)`\n
    This function is called on every expansion and can be used for statistical
purposes.

  - `void onDiscover(const State& s, int fScore, int gScore)`\n
    This function is called on every node discovery and can be used for
   statistical purposes.

    \tparam StateHasher A class to convert a state to a hash value. Default:
   std::hash<State>
*/

template <typename State, typename Action, typename Cost, typename Environment, typename vectorCache1, typename vectorCache2,
          typename StateHasher = std::hash<State> >
class AStar {
 public:
  AStar(Environment& environment, vectorCache1& gc, vectorCache2& ic) : m_env(environment), gc(gc), ic(ic) {}

  bool search(const State& startState,
              PlanResult<State, Action, Cost>& solution, Cost initialCost = 0) {
    solution.states.clear();
    solution.states.push_back(std::make_pair<>(startState, 0));
    solution.actions.clear();
    solution.cost = 0;

    openSet_t openSet;
    std::unordered_map<State, fibHeapHandle_t, StateHasher> stateToHeap;
    std::unordered_set<State, StateHasher> closedSet;
    std::unordered_map<State, std::tuple<State, Action, Cost, Cost>,
                       StateHasher>
        cameFrom;

    auto handle = openSet.push(
        Node(startState, m_env.admissibleHeuristic(startState), initialCost));
    stateToHeap.insert(std::make_pair<>(startState, handle));
    (*handle).handle = handle;

    std::vector<Neighbor<State, Action, Cost> > neighbors;
    neighbors.reserve(10);
    int max_size_open = 0;
    int num_have_been = 0;
    int num_same_config = 0;
    Timer timer;
    while (!openSet.empty()) {
      timer.stop();
      double duration1 = timer.elapsedSeconds();
      if(duration1 > 50){
        std::cout << "Time out\n";
        break;
      }
      Node current = openSet.top();
      if(openSet.size() > max_size_open) max_size_open = openSet.size();
      // std::cout << "size " << max_size_open << std::endl;
      m_env.onExpandNode(current.state, current.fScore, current.gScore);
        int num_same_loc = 0;
        for(auto it = closedSet.begin(); it != closedSet.end(); it++){
          // std::cout << (*it).grid[0] << "----\n";
          if(current.state.x == (*it).x && current.state.y == (*it).y && current.state.time == (*it).time) num_same_loc++;
          // is_equal = true;
          // for (int h = 0; h < neighbor.state.grid.size(); ++h) {              
          //   if((*it).grid[h] != neighbor.state.grid[h]){
          //     is_equal = false;
          //     break;
          //   }
          // }
          // if(is_equal) num_same_config++;
        }      
        // std::cout << "num_same_loc " << num_same_loc << std::endl;
      // std::cout << current.state.x << ", " << current.state.y << ",f, "<< current.fScore << ",g, " << current.gScore  << "--------------------"<< std::endl;
      if (m_env.isSolution(current.state)) {
        solution.states.clear();
        solution.actions.clear();
        auto iter = cameFrom.find(current.state);
        while (iter != cameFrom.end()) {
          solution.states.push_back(
              std::make_pair<>(iter->first, std::get<3>(iter->second)));
          solution.actions.push_back(std::make_pair<>(
              std::get<1>(iter->second), std::get<2>(iter->second)));
          iter = cameFrom.find(std::get<0>(iter->second));
        }
        solution.states.push_back(std::make_pair<>(startState, initialCost));
        // std::reverse(solution.states.begin(), solution.states.end());
        // std::reverse(solution.actions.begin(), solution.actions.end());
        solution.cost = current.gScore;
        solution.fmin = current.fScore;
        std::cout << "max size of open list, " << max_size_open << ", number of states have been, " << num_have_been  << 
        ", closed, " << closedSet.size()  << ",open, " << stateToHeap.size() << " , sum, " << closedSet.size() + stateToHeap.size() << ", sameconfig " << num_same_config << "\n";
        return true;
      }

      openSet.pop();
      stateToHeap.erase(current.state);
      closedSet.insert(current.state);

      // traverse neighbors
      neighbors.clear();
      m_env.getNeighbors(current.state, neighbors);
      for (const Neighbor<State, Action, Cost>& neighbor : neighbors) {

        bool  is_equal = true;
        for(auto it = closedSet.begin(); it != closedSet.end(); it++){
          // std::cout << (*it).grid[0] << "----\n";
          is_equal = true;
          for (int h = 0; h < neighbor.state.grid.size(); ++h) {              
            if((*it).grid[h] != neighbor.state.grid[h]){
              is_equal = false;
              break;
            }
          }
          if(is_equal) num_same_config++;
        }
        
        for(auto it = stateToHeap.begin(); it != stateToHeap.end(); it++){
          is_equal = true;
          for (int h = 0; h < neighbor.state.grid.size(); ++h) {              
            if((*it).first.grid[h] != neighbor.state.grid[h]){
              is_equal = false;
              break;
            }
          }
          if(is_equal) num_same_config++;
        }        

        if (closedSet.find(neighbor.state) == closedSet.end()) {
          Cost tentative_gScore = current.gScore + neighbor.cost;
          auto iter = stateToHeap.find(neighbor.state);
          if (iter == stateToHeap.end()) {  // Discover a new node
            Cost fScore =
                tentative_gScore + m_env.admissibleHeuristic(neighbor.state);
            auto handle =
                openSet.push(Node(neighbor.state, fScore, tentative_gScore));
            (*handle).handle = handle;
            stateToHeap.insert(std::make_pair<>(neighbor.state, handle));
            m_env.onDiscover(neighbor.state, fScore, tentative_gScore);
            if(openSet.size() > max_size_open) max_size_open = openSet.size();            
          //  std::cout << "  this is a new node " << fScore << ",gScore, " <<  tentative_gScore << ", " << neighbor.state.x << ", " << neighbor.state.y << ", " <<
          //   tentative_gScore << std::endl;
          } else {
            num_have_been++;
            auto handle = iter->second;
            // std::cout << "  this is an old node: " << tentative_gScore + m_env.admissibleHeuristic(neighbor.state) << ", gScore, " << tentative_gScore << "," << neighbor.state.x << ", " << neighbor.state.y << ", "
            // << (*handle).gScore << std::endl;
            // We found this node before with a better path
            if (tentative_gScore > (*handle).gScore) {
              // gc.returnItem(&neighbor.state.grid);
              ic.returnItem(&neighbor.state.need_update_index);
              continue;
            }

            if (tentative_gScore == (*handle).gScore) {
            	if((*handle).state.dir > neighbor.state.dir) (*handle).state.dir =  neighbor.state.dir;
              // gc.returnItem(&neighbor.state.grid);
              ic.returnItem(&neighbor.state.need_update_index);              
               continue;
            }

            // update f and gScore
            Cost delta = (*handle).gScore - tentative_gScore;
            (*handle).state.dir =  neighbor.state.dir;
            (*handle).gScore = tentative_gScore;
            (*handle).fScore -= delta;
            (*handle).state.grid.swap(neighbor.state.grid);
            (*handle).state.need_update_index.swap(neighbor.state.need_update_index);
            
            openSet.increase(handle);
            m_env.onDiscover(neighbor.state, (*handle).fScore,
                             (*handle).gScore);
          }

          // Best path for this node so far
          // TODO: this is not the best way to update "cameFrom", but otherwise
          // default c'tors of State and Action are required
          cameFrom.erase(neighbor.state);
          cameFrom.insert(std::make_pair<>(
              neighbor.state,
              std::make_tuple<>(current.state, neighbor.action, neighbor.cost,
                                tentative_gScore)));
        }else{
           num_have_been++;
 
            // gc.returnItem(&neighbor.state.grid);
            ic.returnItem(&neighbor.state.need_update_index);
        }
      }
    }

    return false;
  }

 private:
  struct Node {
    Node(const State& state, Cost fScore, Cost gScore)
        : state(state), fScore(fScore), gScore(gScore) {}

    bool operator<(const Node& other) const {
      // Sort order
      // 1. lowest fScore
      // 2. highest gScore

      // Our heap is a maximum heap, so we invert the comperator function here
      if (fScore != other.fScore) {
        return fScore > other.fScore;
      } else if(gScore != other.gScore){
    	  return gScore < other.gScore;
      }/*else if(state.x != other.state.x){
        return state.x < other.state.x;
      } else return state.y < other.state.y;*/
/*      else if(gScore != other.gScore){
        return gScore < other.gScore;
      } else {
    	  return state.dir < other.state.dir;
      }*/
    }

    friend std::ostream& operator<<(std::ostream& os, const Node& node) {
      os << "state: " << node.state << " fScore: " << node.fScore
         << " gScore: " << node.gScore;
      return os;
    }

    State state;

    Cost fScore;
    Cost gScore;

#ifdef USE_FIBONACCI_HEAP
    typename boost::heap::fibonacci_heap<Node>::handle_type handle;
#else
    typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >::handle_type
        handle;
#endif
  };

#ifdef USE_FIBONACCI_HEAP
  typedef typename boost::heap::fibonacci_heap<Node> openSet_t;
  typedef typename openSet_t::handle_type fibHeapHandle_t;
#else
  typedef typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
                                           boost::heap::mutable_<true> >
      openSet_t;
  typedef typename openSet_t::handle_type fibHeapHandle_t;
#endif

 private:
  Environment& m_env;
  vectorCache1& gc;
  vectorCache2& ic;

};

}  // namespace libMultiRobotPlanning
