#pragma once

#ifdef USE_FIBONACCI_HEAP
#include <boost/heap/fibonacci_heap.hpp>
#endif



#include <unordered_map>
#include <unordered_set>

//#include "neighbor.hpp"
// #include "planresult.hpp"
#include "a_star.hpp"



namespace libMultiRobotPlanning {

/*!
  \example sipp.cpp Simple example using a 2D grid world and
  up/down/left/right
  actions
*/

/*! \brief JPSSIPP Algorithm to find the shortest path with dynamic obstacles using the jump point search.

This class implements the SIPP algorithm. SIPP is an informed search algorithm
that finds the shortest path for a given map and dynamic a-priori known
obstacles.
It can use a heuristic that needs to be admissible.

Details of the algorithm can be found in the following paper:\n
Mike Phillips and Maxim Likhachev:\n
"SIPP:  Safe  Interval  Path  Planning  for  Dynamic  Environments". IEEE
International Conference on Robotics and Automation (ICRA), 2011\n
https://doi.org/10.1109/ICRA.2011.5980306

This class can either use a fibonacci heap, or a d-ary heap. The latter is the
default. Define "USE_FIBONACCI_HEAP" to use the fibonacci heap instead.

\tparam State Custom state for the search. Needs to be copy'able
\tparam Location Custom location type for the search. Needs to be copy'able
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
*/




template <typename State, typename Location, typename Action, typename Cost,
          typename Environment>
class JPSSIPP {
 public:
  struct interval {
    interval(Cost start, Cost end) : start(start), end(end) {}

    Cost start;
    Cost end;

    friend bool operator<(const interval& a, const interval& b) {
      return a.start < b.start;
    }
  };


 public:
  JPSSIPP(Environment& environment) : m_env(environment), m_astar(m_env) {}

  void setCollisionIntervals(const Location& location,
                             const std::vector<interval>& intervals) {
    m_env.setCollisionIntervals(location, intervals);
  }

  bool mightHaveSolution(const State& goal) {
    return m_env.mightHaveSolution(goal);
  }

  bool search(const State& startState, const Action& waitAction,
              PlanResult<State, Action, Cost>& solution, Cost startTime = 0) {
    PlanResult<JPSSIPPState, JPSSIPPAction, Cost> astarsolution;
    solution.cost = 0;
    solution.fmin = 0;
    solution.actions.clear();
    solution.states.clear();
    size_t interval;

    if (!m_env.findSafeInterval(startState, startTime, interval)) {
      return false;
    }

    bool success = m_astar.search(JPSSIPPState(startState, Action::All, 0, interval), astarsolution, startTime);

    solution.cost = astarsolution.cost - startTime;
    solution.fmin = astarsolution.fmin;
    for (size_t i = 0; i < astarsolution.actions.size(); ++i) {
//      std::cout <<"----------------------- ***********************\n";
//      std::cout << astarsolution.actions[i].second << "  " << astarsolution.actions[i].first.time << std::endl;
      Cost waitTime =
          astarsolution.actions[i].second - astarsolution.actions[i].first.time;
      if (waitTime == 0) {
        solution.states.push_back(
            std::make_pair<>(astarsolution.states[i].first.state,
                             astarsolution.states[i].second));
        solution.actions.push_back(
            std::make_pair<>(astarsolution.actions[i].first.action,
                             astarsolution.actions[i].second));
      } else {
        // additional wait action before
        solution.states.push_back(
            std::make_pair<>(astarsolution.states[i].first.state,
                             astarsolution.states[i].second));
        solution.actions.push_back(std::make_pair<>(waitAction, waitTime));
        solution.states.push_back(
            std::make_pair<>(astarsolution.states[i].first.state,
                             astarsolution.states[i].second + waitTime));
        solution.actions.push_back(
            std::make_pair<>(astarsolution.actions[i].first.action,
                             astarsolution.actions[i].first.time));
      }
    }
    solution.states.push_back(
        std::make_pair<>(astarsolution.states.back().first.state,
                         astarsolution.states.back().second));

    return success;
  }

 private:
  // public:
  struct JPSSIPPState {
    JPSSIPPState(const State& state, size_t interval)
        : state(state), interval(interval), cost(0){}

    JPSSIPPState(const State& state, const Action& action, Cost cost, size_t interval)
        : state(state), p_state(State(-1, -1)), action(action), cost(cost), interval(interval) {}

    bool operator==(const JPSSIPPState& other) const {
      return std::tie(state, interval) == std::tie(other.state, other.interval);
    }

    friend std::ostream& operator<<(std::ostream& os, const JPSSIPPState& s) {
      return os << "(" << s.state << "," << s.interval << ")";
    }

    State state;
    State p_state;
    Cost cost;
    Action action;
    size_t interval;
  };

  struct JPSSIPPStateHasher {
    size_t operator()(const JPSSIPPState& s) const {
      size_t seed = 0;
      boost::hash_combine(seed, std::hash<State>()(s.state));
//      boost::hash_combine(seed, s.action);
      boost::hash_combine(seed, s.cost);
      boost::hash_combine(seed, s.interval);
      return seed;
    }
  };

  struct JPSSIPPAction {
    JPSSIPPAction(const Action& action, Cost time) : action(action), time(time) {}
    Action action;
    Cost time;
  };

  // private:
  struct JPSSIPPEnvironment {
    JPSSIPPEnvironment(Environment& env) : m_env(env) {}

    Cost admissibleHeuristic(const JPSSIPPState& s) {
      return m_env.admissibleHeuristic(s.state);
    }

    bool mightHaveSolution(const State& goal) {
      const auto& si = safeIntervals(m_env.getLocation(goal));
      return m_env.isSolution(goal) &&
        !si.empty() &&
        si.back().end == std::numeric_limits<Cost>::max();
    }

    bool isSolution(const JPSSIPPState& s) {
    	if(!m_env.isJPS()){
    		     return m_env.isSolution(s.state) &&
    		             safeIntervals(m_env.getLocation(s.state)).at(s.cost).end ==
    		                 std::numeric_limits<Cost>::max();
    	}else{
        	std::cout << s.state.x <<" " << s.state.y << " " << s.cost << "  Solution\n";
    		const auto& si = safeIntervals(m_env.getLocation(s.state));
    		bool is_flag = false;
    		for(size_t idx = 0; idx < si.size(); ++idx){
    			if(si[idx].start <= s.cost && si[idx].end == std::numeric_limits<Cost>::max()){
    				is_flag = true;
    			}
    		}
    		return is_flag && m_env.isSolution(s.state);

    	}


    }

    void getNeighbors(
        const JPSSIPPState& s,
        std::vector<Neighbor<JPSSIPPState, JPSSIPPAction, Cost> >& neighbors) {


      std::vector<Neighbor<State, Action, Cost> > motions;

      JPSSIPPState s_temp = s;

      if(!m_env.isJPS()){
//    	  std::cout << "Current state: " << s.action << " " << s.state.x << " " << s.state.y <<  "Last G-score " << m_lastGScore << "+++++++++++++++++++++++++++++++++++++++" << std::endl;
    	  m_env.getNeighbors(s_temp.state, motions);
//    	  std::cout << "The number of neighbors: " << motions.size() << std::endl;
          for (const auto& m : motions) {
        	  m_env.num_generation++;
        	  // std::cout << "gN " << m.state << std::endl;
            Cost m_time = m.cost;
            // std::cout << m_lastGScore;
//            std::cout <<  "The successors(Normal) are " <<std::endl;
//            std::cout << m.state.x << " " <<m.state.y <<std::endl;

            Cost start_t = m_lastGScore + m_time;
            Cost end_t =
                safeIntervals(m_env.getLocation(s.state)).at(s.interval).end;

            const auto& sis = safeIntervals(m_env.getLocation(m.state));
            for (size_t i = 0; i < sis.size(); ++i) {
              const interval& si = sis[i];
              // std::cout << "  i " << i << ": " << si.start << "," << si.end <<
              // std::endl;
              if (si.start - m_time > end_t || si.end < start_t) {
                continue;
              }
              int t;
              if (m_env.isCommandValid(s.state, m.state, m.action, m_lastGScore,
                                       end_t, si.start, si.end, t, m_time)) {
                neighbors.emplace_back(Neighbor<JPSSIPPState, JPSSIPPAction, Cost>(
                    JPSSIPPState(m.state, Action::All, i, 0), JPSSIPPAction(m.action, m.cost),
                    t - m_lastGScore));
              }
            }
          }
      }else {
//    	  std::cout << "Curr"
    	  jps_successors.clear();
    	  std::cout << "Current state: " << s.action << " " << s.state.x << " " << s.state.y << "Cost " << s.cost <<  "  Last G-score " << m_lastGScore << " ))))))))))))))))))))))))))))))))))))))))))))))))))" << std::endl;
//   	  s_temp.state.x = 5;
//    	  s_temp.state.y = 2;
// 	    	  m_lastGScore = 0;
//    	  Cost temp = m_lastGScore;

    	  // Up
//    	  State Up(s_temp.state.x, s_temp.state.y+1);

/*          if(Up != s_temp.p_state && m_env.stateValid(Up)){
        	 if(isTemporalObstacleSafe2(Up, s_temp.cost + 1)){
	    			const auto& si = safeIntervals(m_env.getLocation(Up));
	    		    for (size_t idx = 0; idx < si.size(); ++idx) {
	    		        if(si[idx].end >= s_temp.cost + 1 && si[idx].start <= s_temp.cost + 1){

	    		        	getJPSSuccessors(s_temp, Action::Up, s_temp.cost);

	    		        }else if(si[idx].start >= s_temp.cost + 1){
	    		        	current_successor.cost = si[idx].start - 1;
	    		        	jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, si[idx].start - 1));
	    		        }
	    		     }
        	 }
          }
    	  getJPSSuccessors(s_temp, Action::All, m_lastGScore);

    	  for (const auto& m : jps_successors) {
    		  std::cout << m.state.state.x << " " << m.state.state.y << "action cost " << m.cost - m_lastGScore << " successor Cost " << m.cost << "\n";
			  neighbors.emplace_back(Neighbor<JPSSIPPState, JPSSIPPAction, Cost>(
					  JPSSIPPState(m.state.state, Action::All, m.cost, 2), JPSSIPPAction(m.action, 0),
					  m.cost - m_lastGScore));
    	  }*/

//    	  std::cout << "The number of neighbors: " << jps_successors.size() << std::endl;

    	  m_env.num_generation--;
    	  for (const auto& m : jps_successors) {
//				  std::cout << "Successor State----------" << m.state.action << " " << m.state.state.x << " " << m.state.state.y << " Cost " << m.cost << std::endl;
    		  // std::cout << "gN " << m.state << std::endl;
    		  Cost m_time = m.cost;
    		  m_env.num_generation++;

    		  Cost start_t = m_lastGScore + m_time;
    		  Cost end_t =
    				  safeIntervals(m_env.getLocation(s.state)).at(s.interval).end;

//			  std::cout << "Successor State " << m.state.state.x << " " << m.state.state.y << " Cost " << m.cost << std::endl;

    		  const auto& sis = safeIntervals(m_env.getLocation(m.state.state));
    		  for (size_t i = 0; i < sis.size(); ++i) {
    			  const interval& si = sis[i];
    			  // std::cout << "  i " << i << ": " << si.start << "," << si.end <<
    			  // std::endl;
    			  if (si.start - m_time > end_t || si.end < start_t) {
    				  continue;
    			  }
    			  int t;
    			  if (m_env.isCommandValid(s.state, m.state.state, m.action, m_lastGScore,
                                   	   end_t, si.start, si.end, t, m_time)) {
//             std::cout << "  gN: " << m.state << "," << i << "," << t << ","
//             << m_lastGScore << std::endl;
//    				  std::cout << "Successor State----------" << m.state.action << m.state.state.x << " " << m.state.state.y << " Cost " << m.cost << std::endl;
//       				  std::cout << "Successor State----------" << m.state.action << m.state.state.x << " " << m.state.state.y << " Cost " << m.cost << "  " << t - m_lastGScore<< std::endl;
    				  neighbors.emplace_back(Neighbor<JPSSIPPState, JPSSIPPAction, Cost>(
    						  JPSSIPPState(m.state.state, Action::All, 0, i), JPSSIPPAction(m.action, m.cost),
							  t - m_lastGScore));
//    				  break;
    			  }
    		  }
    	  }
      }
    }


    /*void getJPSSuccessors_1(JPSSIPPState s, Action action, Cost g){
    	if(m_env.isSolution(s.state)){
    		jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(s, s.action, g));
    		return ;
    	}

    	std::cout << "Current generating " << s.state.x << " " << s.state.y << "Parent "<< s.p_state.x << " " << s.p_state.y << " Gcost： " << g << " Action " << action << " Saction " << s.action << std::endl;
    	bool flag_temporal = false;
    	if(isTemporalObstacle(s.state)){
    		size_t interval_1;
    		if(!findSafeInterval(s.state, s.cost, interval_1)) return ;
    		flag_temporal = true;
    	}

    	JPSSIPPState current_successor = s;
    	current_successor.p_state = s.state;
 //   	current_successor.cost = g + 1;
    	if(s.action == Action::Left || s.action == Action::Up || s.action == Action::Down || s.action == Action::All){

    		current_successor.state.x = s.state.x - 1;
    		current_successor.state.y = s.state.y;
//    		std::cout << current_successor.state.x << " " << current_successor.state.y << " " << s.action << " Left------\n";

    		if(m_env.stateValid(current_successor.state)){
    	  		bool flag_succ = false;
    	    		if(isTemporalObstacleSafe2(State(s.state.x - 1, s.state.y + 1), g + 1)){
    	    			const auto& si = safeIntervals(m_env.getLocation(State(s.state.x - 1, s.state.y + 1)));
    	    		    for (size_t idx = 0; idx < si.size(); ++idx) {
    	    		        if(si[idx].end >= g+1 && si[idx].start <= g+1){
    	    		        	flag_succ = true;
    	    		        	jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, g + 1));
    	    		        }else if(si[idx].start >= g+1){
    	    		        	current_successor.cost = si[idx].start - 1;
    	    		        	jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, si[idx].start - 1));
    	    		        }
    	    		     }
    	    		}

    	    		if(isTemporalObstacleSafe2(State(s.state.x - 1, s.state.y - 1), g + 1)){
    	    			const auto& si = safeIntervals(m_env.getLocation(State(s.state.x - 1, s.state.y - 1)));
    	    			for(size_t idx = 0; idx < si.size(); ++idx){
    	    				if(si[idx].end >= g + 1 && si[idx].start <= g + 1){
    	    					flag_succ = true;
    	    					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, g + 1));
    	    				}else if(si[idx].start >= g + 1){
    	    					current_successor.cost = si[idx].start - 1;
    	    					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, si[idx].start - 1));
    	    				}
    	    			}
    	    		}

    	    		if(isTemporalObstacleSafe2(State(s.state.x - 2, s.state.y), g + 2)){
    	    			const auto& si = safeIntervals(m_env.getLocation(State(s.state.x - 2, s.state.y)));
    	    			for(size_t idx = 0; idx < si.size(); ++idx){
    	    				if(si[idx].end >= g + 1 && si[idx].start <= g + 1){
    	    					flag_succ = true;
    	    					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, g + 1));
    	    				}else if(si[idx].start >= g + 1){
    	    					current_successor.cost = si[idx].start - 1;
    	    					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, si[idx].start - 1));
    	    				}
    	    			}
    	    		}

    	    		if(flag_temporal && isTemporalObstacleSafe2(s.state, g + 1)){
    	    			const auto& si = safeIntervals(m_env.getLocation(State(s.state.x - 1, s.state.y - 1)));
    	    			for(size_t idx = 0; idx < si.size(); ++idx){
    	    				if(si[idx].end >= g + 1 && si[idx].start <= g + 1){
    	    					flag_succ = true;
    	    					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, g + 1));
    	    				}else if(si[idx].start >= g + 1){
    	    					current_successor.cost = si[idx].start - 1;
    	    					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, si[idx].start - 1));
    	    				}
    	    			}
    	    		}
    	    		if(!flag_succ){
    	    			bool is_down = false, is_up = false;
    	        		if((m_env.isObstacle(State(s.state.x, s.state.y + 1))
    	        				|| isTemporalObstacleSafe(State(s.state.x, s.state.y + 1), g + 1))
    	        				&& m_env.stateValid(State(s.state.x - 1, s.state.y + 1))){
    	        			is_up = true;
    	        		}
    	        		if((m_env.isObstacle(State(s.state.x, s.state.y - 1))
    	        				||  isTemporalObstacleSafe(State(s.state.x, s.state.y - 1), g + 1))
    	        				&& m_env.stateValid(State(s.state.x - 1, s.state.y - 1))){
    	        			is_down = true;
    	        		}

    	        		if(is_up || is_down){
//    	        			current_successor.action = Action::BUD;
    	        			jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, g + 1));
    	        		} else {
    	        			current_successor.action = Action::Left;
    	        			getJPSSuccessors(current_successor, Action::Left, g + 1);
    	        		}
    	    		}
    		}

    	}

      	if(s.action == Action::Right || s.action == Action::Up || s.action == Action::Down || s.action == Action::All){


        		current_successor.state.x = s.state.x + 1;
        		current_successor.state.y = s.state.y;
          		 std::cout << current_successor.state.x << " " << current_successor.state.y << " " << s.action << " Right------\n";

        		bool is_up = false, is_left = false;
        		bool flag_succ = false;
        		if(m_env.stateValid(current_successor.state)){
               		if(isTemporalObstacleSafe2(State(s.state.x + 1, s.state.y + 1), g + 1)){
               			std::cout << " Step 1\n";
                			const auto& si = safeIntervals(m_env.getLocation(State(s.state.x + 1, s.state.y + 1)));
                		    for (size_t idx = 0; idx < si.size(); ++idx) {
                		        if(si[idx].end >= g+1 && si[idx].start <= g+1){
                		        	flag_succ = true;
                		        	jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, g + 1));
                		        }else if(si[idx].start >= g + 1){
                		        	current_successor.cost = si[idx].start - 1;
                		        	jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, si[idx].start - 1));
                		        }
                		     }
                		}


                		if(isTemporalObstacleSafe2(State(s.state.x + 1, s.state.y - 1), g + 1)){
                			std::cout << " Step 2\n";
                			const auto& si = safeIntervals(m_env.getLocation(State(s.state.x + 1, s.state.y - 1)));
                			for(size_t idx = 0; idx < si.size(); ++idx){
                				if(si[idx].end >= g + 1 && si[idx].start <= g + 1){
                					flag_succ = true;
                					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, g + 1));
                				}else if(si[idx].start >= g + 1){
                					current_successor.cost = si[idx].start - 1;
                					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, si[idx].start - 1));
                				}
                			}
                		}

                		if(isTemporalObstacleSafe2(State(s.state.x + 2, s.state.y), g + 2)){
                			std::cout << " Step 3 " << s.state.x + 2 << " " << s.state.y <<"\n";
                			const auto& si = safeIntervals(m_env.getLocation(State(s.state.x + 2, s.state.y)));
                			for(size_t idx = 0; idx < si.size(); ++idx){
                				if(si[idx].end >= g + 1 && si[idx].start <= g + 1){
                					flag_succ = true;
                					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, g + 1));
                				}else if(si[idx].start >= g + 1){
                					std::cout << " Step 3 " << si[idx].start << std::endl;
                					current_successor.cost = si[idx].start;
                					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, si[idx].start - 1));
                				}
                			}
                		}

                		if(flag_temporal && isTemporalObstacleSafe2(s.state, g + 1)){
                			std::cout << " Step 4 " << s.state.x << " " << s.state.y <<"\n";
                			const auto& si = safeIntervals(m_env.getLocation(s.state));
                			for(size_t idx = 0; idx < si.size(); ++idx){
                				if(si[idx].end >= g + 1 && si[idx].start <= g + 1){
                					flag_succ = true;
                					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, g + 1));
                				}else if(si[idx].start >= g + 1){
                					std::cout << " Step 4 " << si[idx].start << std::endl;
                					current_successor.cost = si[idx].start - 1;
                					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, si[idx].start - 1));
                				}
                			}
                		}
                		if(!flag_succ){
                			std::cout << "))))))))))))))))))))))))))))))))))))))))))))\n";
                			bool is_down = false, is_up = false;
                    		if((m_env.isObstacle(State(s.state.x, s.state.y + 1))
                    				|| isTemporalObstacleSafe(State(s.state.x, s.state.y + 1), g + 1))
                    				&& m_env.stateValid(State(s.state.x + 1, s.state.y + 1))){
                    			is_up = true;
                    		}
                    		if((m_env.isObstacle(State(s.state.x, s.state.y - 1))
                    				||  isTemporalObstacleSafe(State(s.state.x, s.state.y - 1), g + 1))
                    				&& m_env.stateValid(State(s.state.x + 1, s.state.y - 1))){
                    			is_down = true;
                    		}

                    		if(is_up || is_down){
                    			std::cout << "+++++++++++++++++++++++++\n";
//                    			current_successor.action = Action::Up;
                    			jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, g + 1));
                    		} else {
                    			current_successor.action = Action::Right;
                    			getJPSSuccessors(current_successor, Action::Right, g + 1);
                    		}
                		}
        		}

        	}


      	if(s.action == Action::Up || s.action == Action::All){


        		current_successor.state.x = s.state.x;
        		current_successor.state.y = s.state.y + 1;
       		 std::cout << current_successor.state.x << " " << current_successor.state.y << " " << s.action << " Up------\n";
        		if(m_env.stateValid(current_successor.state)){
            		bool flag_succ = false;

            		if(isTemporalObstacleSafe2(State(s.state.x, s.state.y + 2), g + 2)){
            			const auto& si = safeIntervals(m_env.getLocation(State(s.state.x + 1, s.state.y + 1)));
            		    for (size_t idx = 0; idx < si.size(); ++idx) {
            		        if(si[idx].end >= g+1 && si[idx].start <= g+1){
            		        	flag_succ = true;
            		        	jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, g + 1));
            		        }else if(si[idx].start >= g + 1){
            		        	current_successor.cost = si[idx].start;
            		        	jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, si[idx].start));
            		        }
            		     }
            		}

            		if(flag_temporal && isTemporalObstacleSafe2(State(s.state.x, s.state.y + 2), g + 2)){
            			const auto& si = safeIntervals(m_env.getLocation(s.state));
            			for(size_t idx = 0; idx < si.size(); ++idx){
            				if(si[idx].end >= g + 1 && si[idx].start <= g + 1){
            					flag_succ = true;
            					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, g + 1));
            				}else if(si[idx].start >= g + 1){
            					current_successor.cost = si[idx].start;
            					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, si[idx].start));
            				}
            			}
            		}

            		if(!flag_succ){
            			current_successor.action = Action::Up;
            			getJPSSuccessors(current_successor, Action::Up, g + 1);
                	}
        		}

        }

      	if(s.action == Action::Down || s.action == Action::All){

        		current_successor.state.x = s.state.x;
        		current_successor.state.y = s.state.y - 1;
       		 std::cout << current_successor.state.x << " " << current_successor.state.y << " " << s.action << " Down------\n";
        		bool flag_succ = false;
        		if(m_env.stateValid(current_successor.state)){
            		if(isTemporalObstacleSafe2(State(s.state.x, s.state.y - 2), g + 2)){
            			const auto& si = safeIntervals(m_env.getLocation(State(s.state.x, s.state.y - 2)));
            		    for (size_t idx = 0; idx < si.size(); ++idx) {
            		        if(si[idx].end >= g+1 && si[idx].start <= g+1){
            		        	flag_succ = true;
            		        	jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Down, g + 1));
            		        }else if(si[idx].start >= g + 1){
            		        	current_successor.cost = si[idx].start;
            		        	jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Down, si[idx].start));
            		        }
            		     }
            		}

            		if(flag_temporal && isTemporalObstacleSafe2(s.state, g + 2)){
            			const auto& si = safeIntervals(m_env.getLocation(s.state));
            			for(size_t idx = 0; idx < si.size(); ++idx){
            				if(si[idx].end >= g + 1 && si[idx].start <= g + 1){
            					flag_succ = true;
            					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Down, g + 1));
            				}else if(si[idx].start >= g + 1){
            					current_successor.cost = si[idx].start;
            					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Down, si[idx].start));
            				}
            			}
            		}
            		if(!flag_succ){
            			current_successor.action = Action::Down;
            			getJPSSuccessors(current_successor, Action::Down, g + 1);
                	}
        		}

        }

    }*/

    void getJPSSuccessors(JPSSIPPState s, Cost current_cost){

       	if(m_env.isSolution(s.state)){
       		jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(s, s.action, current_cost));
       		return ;
       	}

//       	std::cout << "Current generating " << s.state.x << " " << s.state.y << "Parent "<< s.p_state.x << " " << s.p_state.y << " Gcost： " << g << " Action " << action << " Saction " << s.action << std::endl;
       	bool flag_solution = false;
       	if(isTemporalObstacle(s.state)){
       		flag_solution = true;
       	}

    	JPSSIPPState current_successor = s;


        m_env.num_generation++;
    	if(s.action == Action::Left || s.action == Action::Up || s.action == Action::Down || s.action == Action::All){
   			current_successor.state.x = s.state.x - 1;
   			current_successor.state.y = s.state.y;

   			if(m_env.stateValid(current_successor.state)){
   				bool is_up = false, is_down = false;
   				if(m_env.isJumpPoint(current_successor.state)){
   					if( ((m_env.isObstacle(State(s.state.x, s.state.y + 1))
   							|| isTemporalObstacleSafe(State(s.state.x, s.state.y + 1), m_lastGScore + current_cost + 1)) &&
   							m_env.stateValid(State(s.state.x - 1, s.state.y + 1)))
   							|| isTemporalObstacleSafe2(State(s.state.x - 1, s.state.y + 1), m_lastGScore + current_cost + 2)
							|| isTemporalObstacleSafe2(State(s.state.x -2, s.state.y), m_lastGScore + current_cost +2)){
   						is_up = true;
   					}
   					if(((m_env.isObstacle(State(s.state.x, s.state.y - 1))
   							|| isTemporalObstacleSafe(State(s.state.x, s.state.y - 1), m_lastGScore + current_cost + 1))
   							&& m_env.stateValid(State(s.state.x - 1, s.state.y - 1)))
   							|| isTemporalObstacleSafe2(State(s.state.x - 1, s.state.y - 1), m_lastGScore + current_cost + 2)
							|| isTemporalObstacleSafe2(State(s.state.x -2, s.state.y), m_lastGScore + current_cost +2)){
   						is_down = true;
   					}
   				}

   				if(is_up && is_down){
   					current_successor.action = Action::All;
       				jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));
   				}

   				if(is_up && !is_down){
   					current_successor.action = Action::Up;
       				jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));
   				}

   				if(!is_up && is_down){
   					current_successor.action = Action::Down;
       				jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));
   				}

   				if(!(is_up || is_down)){
  					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)){
  						current_successor.action = Action::All;
  	          			jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));
//  						current_successor.action = Action::Left;
//  	          			jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));
  					} else {
  						current_successor.action = Action::Left;
  						if( isTemporalObstacle(current_successor.state, m_lastGScore + current_cost + 1)){
  							jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));
  						} else getJPSSuccessors(current_successor, current_cost + 1);
  					}
   				}
   			}
    	}
    	if(s.action == Action::Right || s.action == Action::Up || s.action == Action::Down || s.action == Action::All){
   			current_successor.state.x = s.state.x + 1;
   			current_successor.state.y = s.state.y;
//   			std::cout<< "Current successor state " << current_successor.state.x << " " << current_successor.state.y <<" +++++++++++++++++++++++++++++------------\n";

   			if(m_env.stateValid(current_successor.state)){
   				bool is_up = false, is_down = false;
   				if(m_env.isJumpPoint(current_successor.state)){
  					if(((m_env.isObstacle(State(s.state.x, s.state.y + 1))
  							|| isTemporalObstacleSafe(State(s.state.x, s.state.y + 1), m_lastGScore + current_cost +1))
  							&& m_env.stateValid(State(s.state.x + 1, s.state.y + 1)))
							|| (isTemporalObstacleSafe2(State(s.state.x + 1, s.state.y + 1), m_lastGScore + current_cost + 1))){
   						is_up = true;
   					}
   					if(((m_env.isObstacle(State(s.state.x, s.state.y - 1))
   							|| isTemporalObstacleSafe(State(s.state.x, s.state.y - 1), m_lastGScore + current_cost + 1))
   							&& m_env.stateValid(State(s.state.x + 1, s.state.y - 1)))
							||(isTemporalObstacleSafe2(State(s.state.x + 1, s.state.y -1), m_lastGScore + current_cost + 1))){
   						is_down = true;
   					}
   				}
   				if(is_up && is_down){
   					current_successor.action = Action::All;
       				jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));
   				}
   				if(is_up && !is_down){
   					current_successor.action = Action::Up;
       				jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));
   				}
   				if(!is_up && is_down){
   					current_successor.action = Action::Down;
       				jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));
   				}

   				if(!(is_up || is_down)){
  					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)) {
  						current_successor.action = Action::All;
  						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));
//  						current_successor.action = Action::Right;
//  						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));

  					} else{
  						current_successor.action = Action::Right;
  						if(isTemporalObstacle(current_successor.state, m_lastGScore + current_cost + 1)){
  							jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));
  						}else getJPSSuccessors(current_successor, current_cost + 1);
  					}

   				}
   			}
    	}

    	if(s.action == Action::Up || s.action == Action::All){
    		current_successor.state.x = s.state.x;
    		current_successor.state.y = s.state.y + 1;
//    		std::cout << current_successor.state.x << " " << current_successor.state.y <<  " Up \n";
    		if(m_env.stateValid(current_successor.state)){
				if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)) {
					current_successor.action = Action::All;
					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost + 1));
				}
				else {
//		    		std::cout << "Up 22\n";
					current_successor.action = Action::Up;
					if( isTemporalObstacle(State(s.state.x, s.state.y + 2), m_lastGScore + current_cost + 1)
							|| isTemporalObstacle(State(s.state.x - 1, s.state.y + 1), m_lastGScore + current_cost + 1)
							|| isTemporalObstacle(State(s.state.x + 1, s.state.y + 1), m_lastGScore + current_cost + 1)){
						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost + 1));
					}else if(isTemporalObstacle(current_successor.state, m_lastGScore + current_cost + 1)){
						//std::cout << "Temporal\n";
						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost + 1));
					}else { getJPSSuccessors(current_successor, current_cost + 1);}
				}
   			}

    	}
    	if(s.action == Action::Down || s.action == Action::All){
    		current_successor.state.x = s.state.x;
    		current_successor.state.y = s.state.y - 1;
//   			std::cout<< "Current successor state " << current_successor.state.x << " " << current_successor.state.y <<" +++++++++++++++++++++++++++++------------\n";
    		if(m_env.stateValid(current_successor.state)){
   				if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)) {
   					current_successor.action = Action::All;
   					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Down, current_cost + 1));
   				} else {
   					current_successor.action = Action::Down;
   					if(isTemporalObstacle(State(s.state.x, s.state.y -2), m_lastGScore + current_cost + 1)
   							|| isTemporalObstacle(State(s.state.x - 1, s.state.y -1), m_lastGScore + current_cost + 1)
							|| isTemporalObstacle(State(s.state.x + 1, s.state.y - 1), m_lastGScore + current_cost + 1)){
   						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Down, current_cost + 1));
   					}else if((current_cost >= m_env.limit_jump && m_env.is_limit)
   							|| isTemporalObstacle(current_successor.state, m_lastGScore + current_cost + 1)){
   						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Down, current_cost + 1));
   					}else getJPSSuccessors(current_successor, current_cost + 1);

   				}
    		}
    	}
    }



    void onExpandNode(const JPSSIPPState& s, Cost fScore, Cost gScore) {
      // const auto& interval =
      // safeIntervals(m_env.getLocation(s.state)).at(s.interval);
      // std::cout << "expand: " << s.state << "," << interval.start << " to "
      // << interval.end << "(g: " << gScore << " f: " << fScore << ")" <<
      // std::endl;
      // This is called before getNeighbors(). We use the callback to find the
      // current cost (=time) of the expanded node
    	m_env.num_expansion++;
    	m_lastGScore = gScore;
        m_env.onExpandNode(s.state, fScore, gScore);
    }

    void onDiscover(const JPSSIPPState& s, Cost fScore, Cost gScore) {
      // const auto& interval =
      // safeIntervals(m_env.getLocation(s.state)).at(s.interval);
      // std::cout << "discover: " << s.state << "," << interval.start << " to "
      // << interval.end << std::endl;
      m_env.onDiscover(s.state, fScore, gScore);
    }

    void setCollisionIntervals(const Location& location,
                               const std::vector<interval>& intervals) {
      m_safeIntervals.erase(location);
      std::vector<interval> sortedIntervals(intervals);
      std::sort(sortedIntervals.begin(), sortedIntervals.end());

      // std::cout << location << ": " << std::endl;
      if (intervals.size() > 0) {
        m_safeIntervals[location]; // create empty safe interval
        int start = 0;
        int lastEnd = 0;
        for (const auto& interval : sortedIntervals) {
          // std::cout << "  ci: " << interval.start << " - " << interval.end <<
          // std::endl;
          assert(interval.start <= interval.end);
          assert(start <= interval.start);
          // if (start + 1 != interval.start - 1) {
          // std::cout << start << "," << interval.start << std::endl;
          // assert(start + 1 < interval.start - 1);
          if (start <= interval.start - 1) {
            m_safeIntervals[location].push_back({start, interval.start - 1});
          }
          // }
          start = interval.end + 1;
          lastEnd = interval.end;
        }
        if (lastEnd < std::numeric_limits<int>::max()) {
          // assert(start < std::numeric_limits<int>::max());
          m_safeIntervals[location].push_back(
              {start, std::numeric_limits<int>::max()});
        }
      }
    }

    bool findSafeInterval(const State& state, Cost time, size_t& interval)
    {
      const auto& si = safeIntervals(m_env.getLocation(state));
      for (size_t idx = 0; idx < si.size(); ++idx) {
        if (si[idx].start <= time && si[idx].end >= time) {
          interval = idx;
          return true;
        }
      }
      return false;
    }

    bool isTemporalObstacle(const Location& location){
    	return m_safeIntervals.find(location) != m_safeIntervals.end();
    }

    bool isTemporalObstacle(const Location& location, Cost now_cost){

//    	return m_safeIntervals.find(location) != m_safeIntervals.end();
        const auto& si = safeIntervals(m_env.getLocation(location));
//        std::cout << "Temporal ---" << si.size() << std::endl;
        if(si.size() == 0) return true;
        if(si.back().start <= now_cost && si.back().end == std::numeric_limits<Cost>::max()) return false;
        else return true;

    }

    bool isTemporalObstacleSafe(const Location& location, Cost time){
        const auto& si = safeIntervals(m_env.getLocation(location));
        if(si.size() == 0) return true;

        for (size_t idx = 0; idx < si.size(); ++idx) {
          if (si[idx].start <= time && si[idx].end >= time) {
        	  return false;
          }
        }
        return true;
    }

    bool isTemporalObstacleSafe2(const Location& location, Cost time){
        const auto& si = safeIntervals(m_env.getLocation(location));
        if(si.size() == 0) return false;

        Cost start_ = 0, end_ = si[0].start - 1;
        for (size_t idx = 0; idx < si.size() - 1; ++idx) {
        	if(start_ <= end_ && end_ >= time) return true;
        	start_ = si[idx].end + 1;
        	end_ = si[idx + 1].start - 1;
        }
    	if(start_ <= end_ && end_ >= time) return true;

        if(si.back().end == std::numeric_limits<Cost>::max()) return false;
        else return true;
    }


   private:
    const std::vector<interval>& safeIntervals(const Location& location) {
      static std::vector<interval> defaultInterval(
          1, {0, std::numeric_limits<Cost>::max()});
      const auto iter = m_safeIntervals.find(location);
      if (iter == m_safeIntervals.end()) {
//    	  std::cout << "Default\n";
        return defaultInterval;
      }
//	  std::cout << "Not Default---\n";
      return iter->second;
    }


   private:
    Environment& m_env;
    Cost m_lastGScore;
    std::unordered_map<Location, std::vector<interval> > m_safeIntervals;
    std::vector<Neighbor<JPSSIPPState, Action, Cost> > jps_successors;
  };

 private:
  JPSSIPPEnvironment m_env;
  AStar<JPSSIPPState, JPSSIPPAction, Cost, JPSSIPPEnvironment, JPSSIPPStateHasher> m_astar;
};

}  // namespace libMultiRobotPlanning
