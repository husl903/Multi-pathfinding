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
  double w = 1.5;
  int bound  = 0.0;
  bool search(const State& startState,
              PlanResult<State, Action, Cost>& solution, int is_wei = 0, int wa_version = 0, Cost initialCost = 0 ) { // wa_version = 1, xdp, wa_version = 2, pwxdp, wa_version = 3, regular
    if(is_wei !=0 )is_weighted = true;
    
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
    std::unordered_map<State, std::tuple<State, Action, Cost, Cost>,
                       StateHasher>
        cameFrom;
    std::cout << "Search 11111\n";
    auto handle = openSet.push(
        Node(startState, m_env.admissibleHeuristic(startState), initialCost, m_env.admissibleHeuristic(startState)));
    stateToHeap.insert(std::make_pair<>(startState, handle));
    (*handle).handle = handle;
    std::cout << "Search 22222\n";

    minimum_h_open = m_env.admissibleHeuristic(startState);
    std::cout << "Search 3333333\n";
    std::cout << minimum_h_open << "\n";
    if(minimum_h_open == 1000000000){
        std::cout << "0.00" <<",num:h=g," << ",numh<g," << ",reopen, " << ", Unreachable,";
        if(!is_weighted) std::cout <<",unweighted,";
        else if(wa_version == 1) std::cout<< ",weightedXDP,";
        else if(wa_version == 2) std::cout << ",weightedpuXDP,";
        else if(wa_version == 3) std::cout << ", weightedRegu,";
        return false;
    }
    count_h[minimum_h_open]++;
    
    std::cout << "Search 444444\n";

    std::vector<Neighbor<State, Action, Cost>> neighbors;
    neighbors.reserve(10);
    int max_size_open = 0;
    int num_have_been = 0;
    int num_closed = 0;
    int num_same_config = 0;
    Timer timer;
    while (!openSet.empty()) {
      // std::cout << "Begin search\n";
      timer.stop();
        // int minimum_test = 10000;
        // for(auto it = stateToHeap.begin(); it != stateToHeap.end(); it++){
        //     auto handle = it->second;
        //     if((*handle).hScore < minimum_test && (*handle).hScore != -1){
        //       minimum_test = (*handle).hScore;
        //     }       
        // }        
        // std::cout << "minimutest " << minimum_test << ", " << minimum_h_open << "\n";
        //assert(minimum_test == minimum_h_open);      
      double duration1 = timer.elapsedSeconds();
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
        else if(wa_version == 1) std::cout<< ",weightedXDP,";
        else if(wa_version == 2) std::cout << ",weightedpuXDP,";
        else if(wa_version == 3) std::cout << ", weightedRegu,";
        stateToHeap.swap(stateToHeapEmpty);
        closedSet.swap(closedSetEmpty);
//        cameFrom.swap(std::unordered_map<State, std::tuple<State, Action, Cost, Cost>, StateHasher>());
        break;
      }
      // std::cout <<"Search 555555\n";
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

       if(isdebug) std::cout <<"Current state " <<  current.state.x << ", " << current.state.y << ",fsore, "<< current.fScore << ",gscore, " << current.gScore  << ",h, " << current.hScore <<  ",hash," << current.state.zorb_hash <<",minimum_h, " << minimum_h_open <<", " << ",--------------------"<< std::endl;
  
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
        
        solution.cost = current.gScore;
        solution.fmin = current.fScore;
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
        else if(wa_version == 1) std::cout<< ",weightedXDP,";
        else if(wa_version == 2) std::cout << ",weightedpuXDP,";
        else if(wa_version == 3) std::cout << ", weightedRegu,";
        stateToHeap.swap(stateToHeapEmpty);
        closedSet.swap(closedSetEmpty);
        return true;
      }

      count_h[current.hScore]--;
      assert(count_h[current.hScore] >= 0);
      if(current.hScore == minimum_h_open && count_h[current.hScore] == 0){
        minimum_h_open = count_h.size() + 1;
        for(int index_h = 0; index_h < count_h.size(); index_h++){
          if(count_h[index_h] != 0){
            minimum_h_open = index_h;
            break;
          }
        }
      }

      openSet.pop();
      stateToHeap.erase(current.state);
      current.state.f = current.fScore;
      current.state.h = current.hScore;

      closedSet.insert(current.state);
      // std::cout << "f, " << current.fScore << " ,g, " << current.gScore << " ,h, " << current.fScore -  current.gScore << ", ";
      // traverse neighbors
      neighbors.clear();
      m_env.getNeighbors(current.state, neighbors, current.state.f);
      for (const Neighbor<State, Action, Cost>& neighbor : neighbors) {


        Cost hScore_t = m_env.admissibleHeuristic(neighbor.state);
        if(hScore_t == 10000000000) continue;
        
        bool  is_equal = true;
        /*for(auto it = closedSet.begin(); it != closedSet.end(); it++){
          is_equal = true;
          for (int h = 0; h < neighbor.state.grid.size(); ++h) {              
            if((*it).grid[h] != neighbor.state.grid[h]){
              is_equal = false;
              break;
            }
          }
          if(is_equal) num_same_config++;
        }*/
        
        /*for(auto it = stateToHeap.begin(); it != stateToHeap.end(); it++){
          is_equal = true;
          for (int h = 0; h < neighbor.state.grid.size(); ++h) {              
            if((*it).first.grid[h] != neighbor.state.grid[h]){
              is_equal = false;
              break;
            }
          }          
          if(is_equal) num_same_config++;
        }*/        
        auto iterClosed = closedSet.find(neighbor.state);
        if (iterClosed == closedSet.end()) {
          Cost tentative_gScore = current.gScore + neighbor.cost;
          auto iter = stateToHeap.find(neighbor.state);
          int flag = -1;
          if (iter == stateToHeap.end()) {  // Discover a new node
            Cost hScore = hScore_t;
//            std::cout << "hScore" << hScore << " Test Hscore\n";
            double fScore = 0.0;
            if(!is_weighted) fScore = tentative_gScore + hScore;//unweighted
            else{
	            if(wa_version == 1){ //XDP
              	  double temp_t = std::sqrt((tentative_gScore - hScore)*(tentative_gScore - hScore) + 4*w*tentative_gScore*hScore);
                   fScore = (1.0/(2*w))*(tentative_gScore + (2*w - 1)*hScore + temp_t);
                }else if(wa_version == 2){ //pwXDP
                  if(hScore > tentative_gScore) {
                    fScore = tentative_gScore + hScore;
                  }else {
                    fScore = (tentative_gScore + (2 * w -1) * hScore)/(w*1.0);
                  }
               }else if(wa_version == 3){//regular weighted Astar
           	      fScore = (tentative_gScore + 1.0 * w * hScore);
               }
           }
            
            auto handle =
                openSet.push(Node(neighbor.state, fScore, tentative_gScore, hScore));
            (*handle).handle = handle;
            if(minimum_h_open > hScore) minimum_h_open = hScore;
            count_h[hScore]++;
            stateToHeap.insert(std::make_pair<>(neighbor.state, handle));
            m_env.onDiscover(neighbor.state, fScore, tentative_gScore);
            if(openSet.size() > max_size_open) max_size_open = openSet.size();

          //   if(flag != -1){
          //     for(auto it = stateToHeap.begin(); it != stateToHeap.end(); it++){
          //       int f_re = m_env.admissibleHeuristicRe((*it).first, neighbor.state.gem_x, neighbor.state.gem_y, flag);
          //       auto handle_t = it->second; 
          //       auto temp = (*handle_t).fScore;
          //       // std::cout << f_re  <<", " << temp << ", " << flag << "  111111111111111111*********************---------------------------\n";
          // // std::cout << (*handle_t).gScore  << "," << (*handle_t).fScore << " ------------1111111111\n";
          //       if(f_re < temp) (*handle_t).fScore = f_re;
          //       // std::cout << f_re  <<", " << (*handle_t).fScore << ", " << flag << " 222222222222222222*********************---------------------------\n";

          // // auto handle_tt = it->second;
          // // std::cout << (*handle_tt).gScore  << "," << (*handle_tt).fScore << " ----------2222222222\n";
          // // (*handle_tt).fScore = temp;
          // // auto handle_ttt = it->second;
          // // std::cout << (*handle_ttt).gScore  << "," << (*handle_ttt).fScore << " ----------33333333\n";     

          //     }
          //   }           
            if(isdebug) std::cout << "  this is a new node, fscore 333, " << fScore << ",gScore, " <<  tentative_gScore << ", hScore, " << hScore << ", " << neighbor.state.x << ", " << neighbor.state.y << ", " 
            << ",dir," << neighbor.state.dir << ",hash, " << neighbor.state.zorb_hash << ",g," << tentative_gScore << std::endl;
          } else {
            num_have_been++;
            auto handle = iter->second;
            // We found this node before with a better path
            if (tentative_gScore > (*handle).gScore) {
              continue;
            }

            if (tentative_gScore == (*handle).gScore) {
            	// if((*handle).state.dir > neighbor.state.dir) 
              (*handle).state.dir |=  neighbor.state.dir;
               continue;
            }

            // update f and gScore
            Cost delta = (*handle).gScore - tentative_gScore;
            (*handle).state.dir =  neighbor.state.dir;
            (*handle).state.time = neighbor.state.time;
            (*handle).state.localstate = neighbor.state.localstate;
            (*handle).state.is_falling = neighbor.state.is_falling;
            (*handle).state.canRollLeft = neighbor.state.canRollLeft;
            (*handle).state.canRollRight = neighbor.state.canRollRight;
            (*handle).state.is_wait = neighbor.state.is_wait;
            (*handle).state.index_gem = neighbor.state.index_gem;
            (*handle).state.gem_x = neighbor.state.gem_x;
            (*handle).state.gem_y = neighbor.state.gem_y;
            (*handle).gScore = tentative_gScore;
            Cost hScore = (*handle).hScore;
            if(!is_weighted) (*handle).fScore -= delta; //unweighted
            else{
                if(wa_version == 1){ //xdp
                  (*handle).fScore = (1.0/(2*w))*(tentative_gScore + (2*w - 1)*hScore + std::sqrt((tentative_gScore - hScore)*(tentative_gScore - hScore) + 4*w*tentative_gScore*hScore));
                }else if(wa_version == 2){ //pwxdp
                  if(hScore > tentative_gScore) (*handle).fScore = tentative_gScore + hScore;
                  else (*handle).fScore = (tentative_gScore + (2 * w -1) * hScore)/(w*1.0);
                }else if(wa_version == 3){ //regular weighted Astar
           	      (*handle).fScore = tentative_gScore + 1.0 * w  * hScore;
                }
             }

            if(isdebug) std::cout << "  this is an old node: fscore 111, " << (*handle).fScore << ", gScore, " << tentative_gScore << " Hscore," << hScore << ", "<< neighbor.state.x << ", " << neighbor.state.y << ", "
            << ",dir," << neighbor.state.dir << ",hash, " << neighbor.state.zorb_hash << ",g,"<< (*handle).gScore << std::endl;

            (*handle).state.grid.assign(neighbor.state.grid.begin(), neighbor.state.grid.end());
            (*handle).state.need_update_index.assign(neighbor.state.need_update_index.begin(), neighbor.state.need_update_index.end());
            
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
           if((*iterClosed).time > current.gScore + neighbor.cost){
              reopen++;
              num_closed++;
              closedSet.erase(iterClosed);
              Cost tentative_gScore = current.gScore + neighbor.cost;
              int flag = -1;
              Cost hScore = hScore_t;
              double fScore = 0.0;
              if(!is_weighted) fScore = tentative_gScore + hScore;
              else {
                if(wa_version == 1){
                  fScore = (1.0/(2*w))*(tentative_gScore + (2*w - 1)*hScore + sqrt((tentative_gScore - hScore)*(tentative_gScore - hScore) + 4*w*tentative_gScore*hScore));
                }else if(wa_version == 2){
                  if(hScore > tentative_gScore) fScore = tentative_gScore + hScore;
                  else fScore = (tentative_gScore + (2 * w -1) * hScore)/(w*1.0);
                }else if(wa_version == 3){
                  fScore = (tentative_gScore + 1.0 * w * hScore);
                }
              }

             if(isdebug) std::cout << "  this is an old node 2222: fscore, " << fScore << ", gScore, " << tentative_gScore << "," << neighbor.state.x << ", " << neighbor.state.y << ", "
             << ",dir," << neighbor.state.dir << ",hash, " << neighbor.state.zorb_hash << ",g,"<< (*handle).gScore << std::endl;
            
                auto handle =
                    openSet.push(Node(neighbor.state, fScore, tentative_gScore, hScore));
                (*handle).handle = handle;
                if(minimum_h_open > hScore) minimum_h_open = hScore;
                count_h[hScore]++;
                stateToHeap.insert(std::make_pair<>(neighbor.state, handle));
                m_env.onDiscover(neighbor.state, fScore, tentative_gScore);
                if(openSet.size() > max_size_open) max_size_open = openSet.size();
              cameFrom.erase(neighbor.state);
              cameFrom.insert(std::make_pair<>(
                  neighbor.state,
                  std::make_tuple<>(current.state, neighbor.action, neighbor.cost,
                                    tentative_gScore)));
           }
        }
      }
    }

    return false;
  }

 private:
  struct Node {
    Node(const State& state, double fScore, Cost gScore, Cost hScore)
        : state(state), fScore(fScore), gScore(gScore), hScore(hScore) {}

    bool operator<(const Node& other) const {
      // Sort order
      // 1. lowest fScore
      // 2. highest gScore

      // Our heap is a maximum heap, so we invert the comperator function here
      if (std::fabs(fScore - other.fScore) >= 0.000001) {
        if(fScore - other.fScore > 0.000001) return true;
        else return false;
      } else if(gScore != other.gScore){
    	  return gScore < other.gScore;
      } else if(state.y != other.state.y){
        return state.y < other.state.y;
      }/* else return state.y < other.state.y;
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

    double fScore;
    Cost gScore;
    Cost hScore;

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
