#pragma once

#ifdef USE_FIBONACCI_HEAP
#include <boost/heap/fibonacci_heap.hpp>
#endif

#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
#include <unordered_set>
#include <malloc.h>

#include "neighbor.hpp"
#include "planresult.hpp"
#include "timer.hpp"
#include "vectorCache.hpp"

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
  bool is_second = false;
  bool is_goal_move = false;
  bool is_weighted = false;
  bool isXDP = false;
  bool ispwXDP = false;
  bool isRegu =  false;
  bool isdebug = false;
  double w = 8.0;
  int bound  = 0.0;

  bool search(const State& startState,
              PlanResult<State, Action, Cost>& solution, int is_wei = 0, int wa_version = 0, int duration = 180, int weight = 20,  Cost initialCost = 0 ) { // wa_version = 1, xdp, wa_version = 2, pwxdp, wa_version = 3, regular, xup, wa_version = 4
    if(is_wei !=0 )is_weighted = true;
    w = 8.0;

    duration = 180;
    solution.states.clear();
    solution.states.push_back(std::make_pair<>(startState, 0));
    solution.actions.clear();
    solution.cost = 0;
    int reopen = 0;
    std::vector<int> count_h(1000,0);
    int minimum_h_open = -1;
    openSet_t openSet;
    std::unordered_map<State, fibHeapHandle_t, StateHasher> stateToHeap;
    std::unordered_map<State, fibHeapHandle_t, StateHasher> stateToHeapEmpty;
    std::unordered_set<State, StateHasher> closedSet;
    std::unordered_set<State, StateHasher> closedSetEmpty;
    std::unordered_map<State, std::tuple<State, Action, Cost, Cost>, StateHasher> cameFrom;


    auto handle = openSet.push(Node(std::make_shared<State>(startState),
                                    m_env.admissibleHeuristic(startState),
                                    initialCost,
                                    m_env.admissibleHeuristic(startState)));
    (*handle).handle = handle;
    stateToHeap.insert(std::make_pair<>(startState, handle));
    minimum_h_open = m_env.admissibleHeuristic(startState);

    std::vector<Neighbor<State, Action, Cost>> neighbors;
    neighbors.reserve(10);
    int max_size_open = 0;
    int num_have_been = 0;
    int num_closed = 0;
    int num_same_config = 0;
    Timer timer;
    while (!openSet.empty()) {
      timer.stop();
      double duration1 = timer.elapsedSeconds();
        int minimum_test = 1000;
/*        for(auto it = stateToHeap.begin(); it != stateToHeap.end(); it++){
            auto handle = it->second;
            if((*handle).hScore < minimum_test){
              minimum_test = (*handle).hScore;
            }       
        }        
        std::cout << "minimutest " << minimum_test << ", " << minimum_h_open << "\n";
        assert(minimum_test == minimum_h_open); */      
      if(duration1 > 10){
        int num_less_f = 0;
        int num_less_f_wait = 0;
        int num_h_less_g = 0;
        int num_h_equal_g = 0;
        for(auto it = closedSet.begin(); it != closedSet.end(); it++){
          if ((*it).f < solution.cost){
            num_less_f++;
            if((*it).is_wait) num_less_f_wait++;
            // std::cout <<  "LESS F, (" <<  (*it).x << "," << (*it).y << "), time " <<(*it).time  << ", f, " << (*it).f << ", hash," << (*it).zorb_hash << " \n"; 
          }
          if((*it).h == (*it).time) num_h_equal_g++;
          if((*it).h < (*it).time) num_h_less_g++;
        }

        std::cout << duration1 <<",num:h=g," << num_h_equal_g << ",numh<g," << num_h_less_g << ",reopen, " << reopen << ", Time out,";
        if(!is_weighted) std::cout <<",unweighted,";
        else if(wa_version == 1) std::cout<< ",weightedXDP," << w << ",";
        else if(wa_version == 2) std::cout << ",weightedpuXDP," << w << ",";
        else if(wa_version == 3) std::cout << ", weightedRegu," << w << ",";
        else if(wa_version == 4) std::cout << ", weightedXUP," << w << ",";

        stateToHeap.swap(stateToHeapEmpty);
        closedSet.swap(closedSetEmpty);
//        cameFrom.swap(std::unordered_map<State, std::tuple<State, Action, Cost, Cost>, StateHasher>());
        break;
      }

      Node current = openSet.top();
      if(openSet.size() > max_size_open) max_size_open = openSet.size();
      m_env.onExpandNode(*current.state, current.fScore, current.gScore);



















      if (m_env.isSolution(*current.state)) {
        solution.states.clear();
        solution.actions.clear();
        auto iter = cameFrom.find(*current.state);
        while (iter != cameFrom.end()) {
          solution.states.push_back(std::make_pair(iter->first, std::get<3>(iter->second)));
          solution.actions.push_back(std::make_pair(std::get<1>(iter->second), std::get<2>(iter->second)));
          iter = cameFrom.find(std::get<0>(iter->second));
        }
        solution.states.push_back(std::make_pair(startState, initialCost));
        solution.cost = current.gScore;
        solution.fmin = current.fScore;
  
        // std::cout <<"Final Current state " <<  current.state.x << ", " << current.state.y  << ", gemx,y,  " <<current.state.gem_x << ", " << current.state.gem_y << ",fsore, "<< current.fScore << ",gscore, " << current.gScore  << ",h, " << current.hScore <<  ",hash," << current.state. zorb_hash <<",minimum_h, " << minimum_h_open <<", " << minimum_test << ",minicount," << count_h[minimum_h_open] << ",--------------------"<< std::endl;



        int num_less_f = 0;
        int num_less_f_wait = 0;
        int num_h_less_g = 0;
        int num_h_equal_g = 0;
        for(auto it = closedSet.begin(); it != closedSet.end(); it++){
          if ((*it).f < solution.cost){
            num_less_f++;
            if((*it).is_wait) num_less_f_wait++;
            // std::cout <<  "LESS F, (" <<  (*it).x << "," << (*it).y << "), time " <<(*it).time  << ", f, " << (*it).f << ", hash," << (*it).zorb_hash << " \n"; 
          }
          if((*it).h == (*it).time) num_h_equal_g++;
          if((*it).h < (*it).time) num_h_less_g++;
        }

        std::cout << ",SearchTime," << duration1 <<",num:h=g," << num_h_equal_g << ",numh<g," << num_h_less_g << ",startState, " << startState.x  << "," << startState.y <<",reopen, " << reopen << ",num_less_f, " << num_less_f << ",num_less_wait, " << num_less_f_wait << ", max size of open list, " << max_size_open << ", number of states have been, " << num_have_been  << 
        ", closed, " << closedSet.size() << ", clsed_num " << num_closed << ",open, " << stateToHeap.size() << " , sum, " << closedSet.size() + stateToHeap.size() << ", sameconfig " << num_same_config << ",";
        
        if(!is_weighted) std::cout <<",unweighted,";
        else if(wa_version == 1) std::cout<< ",weightedXDP," << w << ",";
        else if(wa_version == 2) std::cout << ",weightedpuXDP," << w << ",";
        else if(wa_version == 3) std::cout << ", weightedRegu," << w << ",";
        else if(wa_version == 4) std::cout << ", weightedXUP," << w << ",";

        stateToHeap.swap(stateToHeapEmpty);
        closedSet.swap(closedSetEmpty);
        return true;
      }

    //   count_h[current.hScore]--;
    //   assert(count_h[current.hScore] >= 0);
    //   if(current.hScore == minimum_h_open && count_h[current.hScore] == 0){
    //     minimum_h_open = count_h.size() + 1;
    //     for(int index_h = 0; index_h < count_h.size(); index_h++){
    //       if(count_h[index_h] != 0){
    //         minimum_h_open = index_h;
    //         break;
    //       }
    //     }
    //   }

      openSet.pop();
      stateToHeap.erase(*current.state);
      current.state->f = current.fScore;
      current.state->h = current.hScore;

      closedSet.insert(*current.state);

      neighbors.clear();
      m_env.getNeighbors(*current.state, neighbors, current.state->f);

      for (const Neighbor<State, Action, Cost>& neighbor : neighbors) {
        Cost hScore_t = m_env.admissibleHeuristic(neighbor.state);
        // if(hScore_t == 10000000000) continue;

        auto iterClosed = closedSet.find(neighbor.state);
        if (iterClosed == closedSet.end()) {
          Cost tentative_gScore = current.gScore + neighbor.cost;
          auto iter = stateToHeap.find(neighbor.state);

          if (iter == stateToHeap.end()) {
            Cost hScore = hScore_t;
            double fScore = tentative_gScore + hScore;
            if(is_weighted){
              if(wa_version == 1) fScore = (1.0/(2*w))*(tentative_gScore + (2*w - 1)*hScore + std::sqrt((tentative_gScore - hScore)*(tentative_gScore - hScore) + 4*w*tentative_gScore*hScore));
              else if(wa_version == 2) fScore = (hScore > tentative_gScore) ? tentative_gScore + hScore : (tentative_gScore + (2 * w -1) * hScore)/(w*1.0);
              else if(wa_version == 3) fScore = tentative_gScore + 1.0 * w * hScore;
              else if(wa_version == 4) fScore = (1.0/(2*w))*(tentative_gScore + hScore + std::sqrt((tentative_gScore + hScore)*(tentative_gScore + hScore) + 4*w*(w-1)*hScore*hScore));
            }

            auto handle = openSet.push(Node(std::make_shared<State>(neighbor.state), fScore, tentative_gScore, hScore));
            (*handle).handle = handle;
            // if(minimum_h_open > hScore) minimum_h_open = hScore;
            // count_h[hScore]++;
            stateToHeap.insert(std::make_pair<>(neighbor.state, handle));
            m_env.onDiscover(neighbor.state, fScore, tentative_gScore);
            if(openSet.size() > max_size_open) max_size_open = openSet.size();
            
            cameFrom.erase(neighbor.state);
            cameFrom.insert(std::make_pair<>(neighbor.state, std::make_tuple<>(*current.state, neighbor.action, neighbor.cost, tentative_gScore)));
          } else {
            auto handle = iter->second;
            if (tentative_gScore >= (*handle).gScore) continue;

            if (tentative_gScore > (*handle).gScore) {
              continue;
            }

            if (tentative_gScore == (*handle).gScore) {
            	// if((*handle).state.dir > neighbor.state.dir) 
              (*handle).state->dir |=  neighbor.state.dir;
               continue;
            }

            (*handle).gScore = tentative_gScore;
            (*handle).state = std::make_shared<State>(neighbor.state);

            Cost hScore = (*handle).hScore;
            if(!is_weighted) (*handle).fScore = tentative_gScore + hScore;
            else{
              if(wa_version == 1) (*handle).fScore = (1.0/(2*w))*(tentative_gScore + (2*w - 1)*hScore + std::sqrt((tentative_gScore - hScore)*(tentative_gScore - hScore) + 4*w*tentative_gScore*hScore));
              else if(wa_version == 2) (*handle).fScore = (hScore > tentative_gScore) ? tentative_gScore + hScore : (tentative_gScore + (2 * w -1) * hScore)/(w*1.0);
              else if(wa_version == 3) (*handle).fScore = tentative_gScore + 1.0 * w * hScore;
              else if(wa_version == 4) (*handle).fScore = (1.0/(2*w))*(tentative_gScore + hScore + std::sqrt((tentative_gScore + hScore)*(tentative_gScore + hScore) + 4*w*(w-1)*hScore*hScore));
            }

            openSet.increase(handle);

            cameFrom.erase(neighbor.state);
            cameFrom.insert(std::make_pair<>(neighbor.state, std::make_tuple<>(*current.state, neighbor.action, neighbor.cost, tentative_gScore)));
          }
        }else{
        // restore reopen logic
          if ((*iterClosed).time > current.gScore + neighbor.cost) {
              reopen++;      // 统计 reopen 次数
              closedSet.erase(iterClosed);
              Cost tentative_gScore = current.gScore + neighbor.cost;
              Cost hScore = hScore_t;
              double fScore = 0.0;
              if(!is_weighted) fScore = tentative_gScore + hScore;
              else{
                if(wa_version == 1)  fScore = (1.0/(2*w))*(tentative_gScore + (2*w - 1)*hScore + std::sqrt((tentative_gScore - hScore)*(tentative_gScore - hScore) + 4*w*tentative_gScore*hScore));
                else if(wa_version == 2) fScore = (hScore > tentative_gScore) ? tentative_gScore + hScore : (tentative_gScore + (2 * w -1) * hScore)/(w*1.0);
                else if(wa_version == 3) fScore = tentative_gScore + 1.0 * w * hScore;
                else if(wa_version == 4) fScore = (1.0/(2*w))*(tentative_gScore + hScore + std::sqrt((tentative_gScore + hScore)*(tentative_gScore + hScore) + 4*w*(w-1)*hScore*hScore));
              }

              auto handle = openSet.push(Node(std::make_shared<State>(neighbor.state), fScore, tentative_gScore, hScore));
              
              (*handle).handle = handle;
              
              stateToHeap.insert(std::make_pair<>(neighbor.state, handle));
              m_env.onDiscover(neighbor.state, fScore, tentative_gScore);
              if(openSet.size() > max_size_open) max_size_open = openSet.size();
              cameFrom.erase(neighbor.state);
              cameFrom.insert(std::make_pair<>(neighbor.state, std::make_tuple<>(*current.state, neighbor.action, neighbor.cost, tentative_gScore)));          
              
          }
        }
      }
    }

    return false;
  }

 private:
  struct Node {
    Node(std::shared_ptr<State> state, double fScore, Cost gScore, Cost hScore)
        : state(state), fScore(fScore), gScore(gScore), hScore(hScore) {}

    bool operator<(const Node& other) const {
      if (std::fabs(fScore - other.fScore) >= 0.000001) return fScore - other.fScore > 0.000001;
      else if(gScore != other.gScore) return gScore < other.gScore;
      else if(state->y != other.state->y) return state->y < other.state->y;
    }

    friend std::ostream& operator<<(std::ostream& os, const Node& node) {
      os << "state: " << *node.state << " fScore: " << node.fScore << " gScore: " << node.gScore;
      return os;
    }

    std::shared_ptr<State> state;
    double fScore;
    Cost gScore;
    Cost hScore;

#ifdef USE_FIBONACCI_HEAP
    typename boost::heap::fibonacci_heap<Node>::handle_type handle;
#else
    typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>, boost::heap::mutable_<true> >::handle_type handle;
#endif
  };

#ifdef USE_FIBONACCI_HEAP
  typedef typename boost::heap::fibonacci_heap<Node> openSet_t;
  typedef typename openSet_t::handle_type fibHeapHandle_t;
#else
  typedef typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>, boost::heap::mutable_<true> > openSet_t;
  typedef typename openSet_t::handle_type fibHeapHandle_t;
#endif

 private:
  Environment& m_env;
  vectorCache1& gc;
  vectorCache2& ic;
};

}  // namespace libMultiRobotPlanning
