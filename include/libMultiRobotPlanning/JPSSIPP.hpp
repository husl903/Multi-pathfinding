#pragma once

#ifdef USE_FIBONACCI_HEAP
#include <boost/heap/fibonacci_heap.hpp>
#endif



#include <unordered_map>
#include <unordered_set>
#include "a_star_jpssipp.hpp"


//#include "timer.hpp"



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
  struct edgeCollision{
	  edgeCollision(Cost t, Action action) : t(t), action(action){}

	Cost t;
	Action action;
	friend bool operator==(const edgeCollision& a, const edgeCollision& b){
	  return (a.t == b.t) && (a.action == b.action);
	}
    friend bool operator<(const edgeCollision& a, const edgeCollision& b) {
      return a.t < b.t;
    }
  };
public:
  struct EdgeCollisionHasher {
    size_t operator()(const edgeCollision& ec) const {
      size_t seed = 0;
      boost::hash_combine(seed, ec.t);
      boost::hash_combine(seed, ec.action);
      return seed;
    }
  };

public:
  struct startTime{
    startTime(Cost rt, Action action, unsigned int dir, bool flag_wait) : rt(rt), action(action), dir(dir), flag_wait(flag_wait){}
	Cost rt;
	Action action;
	bool flag_wait;
	unsigned int dir;
	friend bool operator<(const startTime&a, const startTime&b){
	  return a.rt < b.rt;
	}
  };


 public:
  JPSSIPP(Environment& environment) : m_env(environment), m_astar(m_env) {}

  void setCollisionIntervals(const Location& location,
                             const std::vector<interval>& intervals) {
    m_env.setCollisionIntervals(location, intervals);
  }
  void setEdgeCollisionSize(const int& dimx, const int& dimy){
	m_env.setEdgeCollisionSize(dimx, dimy);
  }

  void setEdgeCollisions(const Location& location,
                             const std::vector<edgeCollision>& ec) {
    m_env.setEdgeCollisions(location, ec);
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
    bool success = m_astar.search(JPSSIPPState(startState, interval, 0xf, 0, false), astarsolution, startTime);

    solution.cost = astarsolution.cost - startTime;
    solution.fmin = astarsolution.fmin;
    for (size_t i = 0; i < astarsolution.actions.size(); ++i) {
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
        : state(state), interval(interval), dir(0x00), g_cost(0), flag_wait(false){}

    JPSSIPPState(const State& state, size_t interval, unsigned int dir)
        : state(state), interval(interval), dir(dir), g_cost(0), flag_wait(false){}

    JPSSIPPState(const State& state, size_t interval, unsigned int dir,Cost g_cost, bool flag_wait)
        : state(state), interval(interval), dir(dir), g_cost(g_cost), flag_wait(flag_wait){}

    bool operator==(const JPSSIPPState& other) const {
      return std::tie(state, interval) == std::tie(other.state, other.interval);
    }

    friend std::ostream& operator<<(std::ostream& os, const JPSSIPPState& s) {
      return os << "(" << s.state << "," << s.interval << ")";
    }

//    Cost cost;
  public:
    Cost g_cost;
    State state;
    unsigned int dir = 0xf;
    bool flag_wait = false;
    Action action;
    size_t interval = -1;
  };

  struct JPSSIPPStateHasher {
    size_t operator()(const JPSSIPPState& s) const {
      size_t seed = 0;
      boost::hash_combine(seed, std::hash<State>()(s.state));
      boost::hash_combine(seed, s.interval);
      return seed;
    }
  };

  struct JPSSIPPStateHasherOpen {
    size_t operator()(const JPSSIPPState& s) const {
      size_t seed = 0;
      boost::hash_combine(seed, std::hash<State>()(s.state));
      boost::hash_combine(seed, s.g_cost);
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
//    	return m_env.admissibleHeuristic(s.state, s.dir, s.g_cost);
//        return m_env.admissibleHeuristic(s.state, s.dir);
        return m_env.admissibleHeuristic(s.state);

    }

    bool mightHaveSolution(const State& goal) {
      const auto& si = safeIntervals(goal);
      return m_env.isSolution(goal) && !si.empty() &&
	  		!si.empty() &&
             si.back().end == std::numeric_limits<Cost>::max();
    }

    bool isSolution(const JPSSIPPState& s) {
      return m_env.isSolution(s.state) &&
             safeIntervals(s.state).at(s.interval).end ==
                 std::numeric_limits<Cost>::max();
    }

    void getNeighbors(
              const JPSSIPPState& s,
              std::vector<Neighbor<JPSSIPPState, JPSSIPPAction, Cost> >& neighbors) {

    		std::vector<Neighbor<State, Action, Cost> > motions;
            JPSSIPPState s_temp = s;
            if(!m_env.isJPS()){
            	m_env.getNeighbors(s_temp.state, motions);
                for (const auto& m : motions) {
              	  m_env.num_generation++;
                  Cost m_time = m.cost;
                  Cost start_t = m_lastGScore + m_time;
                  Cost end_t =
                      safeIntervals(s.state).at(s.interval).end;

                  const auto& sis = safeIntervals(m.state);
                  for (size_t i = 0; i < sis.size(); ++i) {
                    const interval& si = sis[i];
                    if (si.start - m_time > end_t || si.end < start_t) {
                      continue;
                    }
                    int t;
                    Action a_temp;
                    if(m.action == Action::Left) a_temp = Action::Right;
                    else if(m.action == Action::Right) a_temp = Action::Left;
                    else if(m.action == Action::Up) a_temp = Action::Down;
                    else if(m.action == Action::Down) a_temp = Action::Up;
//                	m_env.num_generation++;
                    if (m_env.isCommandValid(s.state, m.state, m.action, m_lastGScore,
                                             end_t, si.start, si.end, t, m_time)
                    		&& !IsEdgeCollisions(m.state, edgeCollision(t - 1, a_temp))) {
                      neighbors.emplace_back(Neighbor<JPSSIPPState, JPSSIPPAction, Cost>(
                          JPSSIPPState(m.state, i,0xf), JPSSIPPAction(m.action, m.cost),
                          t - m_lastGScore));
                    }
                  }
                }
            } else {
              const auto& sis_s = safeIntervals(s.state);
          	  Cost start_t = m_lastGScore;
      		  Cost end_t = sis_s.at(s.interval).end;
      		  flag_is_solution = false;
      		  std::vector<startTime> re_start;
      		  if(m_env.isDebug) std::cout << "Current State " << s.state.x << "  " << s.state.y <<" Gscore" << m_lastGScore << " interval " << s.interval << " Direction " << s.dir << "  ********************************\n";
/*      		  flag_curr_i.resize(3); flag_next_i.resize(3);
      		  flag_curr_w.resize(3); flag_next_w.resize(3);
      		  for( int i = 0;i < 3; i++){
          		  flag_curr_i[i].resize(m_env.m_dimy); flag_next_i[i].resize(m_env.m_dimy);
          		  flag_curr_w[i].resize(m_env.m_dimy); flag_next_w[i].resize(m_env.m_dimy);
//          		  std::cout << flag_curr_i[i].size() << " " << flag_next_i[i].size() << "\n";
      		  }*/


      		  jps_successors.clear();
      		  bool is_scan_op = (s.dir & 0xc);
      		  getJPSHorizontalSuccessors(s_temp, s.dir, 0);
      		  if(is_scan_op) getJPSVerticalSuccessors(s_temp, s.dir, 0);

         	   //Generate the waiting successor
         	   Cost up_start_t = -1, down_start_t = -1, left_start_t = -1, right_start_t = -1;
         	   re_start.clear();
         	  JPSSIPPState temp_state = s;
      	      if(m_env.stateValid(State(s.state.x - 1, s.state.y)) &&
      	    		  isTemporalObstacleAfterT(State(s.state.x - 1, s.state.y), m_lastGScore + 1, left_start_t)){
        		   temp_state.dir = 0x00;
        		   temp_state.flag_wait = true;
        		   temp_state.state.x = s.state.x - 1;
        		   temp_state.state.y = s.state.y;

        		   const auto& sis = safeIntervals(State(s.state.x - 1, s.state.y));
        		   for (size_t i = 0; i < sis.size(); ++i) {
        			   const interval& si = sis[i];
        			   if (si.start - 1 > end_t)	break;
        			   if (si.end < start_t + 1)	continue;
        			   if (si.start <= m_lastGScore + 1) continue;

        			   temp_state.interval = i;
        			   temp_state.dir = 0x00;
        			   if(!IsEdgeCollisions(State(s.state.x - 1, s.state.y), edgeCollision(si.start - 1, Action::Right))){
        				   temp_state.dir |= 0x01;
        			   }else break;
        			   for(size_t ii = 0; ii < sis_s.size(); ++ii){
        				   if(sis_s[ii].start == si.start + 1 &&
         					   !IsEdgeCollisions(State(s.state.x, s.state.y), edgeCollision(si.start -1, Action::Left))){
        					   temp_state.dir |= 0x02;
        					   break;
        				   }
        			   }

        			    Cost temp_t = si.start + 1;
        			    if(m_env.stateValid(State(s.state.x - 1, s.state.y + 1))){ //Whether the successor need to restart the up direction
        			    	if(m_env.isObstacle(State(s.state.x, s.state.y + 1))
 								|| IsEdgeCollisions(State(s.state.x, s.state.y + 1), edgeCollision(si.start -1, Action::Down))
 								|| isTemporalObstacleAtT(State(s.state.x, s.state.y + 1), si.start)
 								|| isSafeAtT(State(s.state.x - 1, s.state.y + 1), temp_t)
 								){
        			    		if(!IsEdgeCollisions(State(s.state.x -1, s.state.y + 1), edgeCollision(si.start - 1, Action::Down))){
        			    			temp_state.dir |= 0x04;
        			    		}
 						}
 /*						if(isSafeAtT(State(s.state.x - 1, s.state.y + 1), temp_t)){
 							bool is_up_p = IsEdgeCollisions(State(s.state.x, s.state.y + 1), edgeCollision(left_1 - 1, Action::Down));
 							bool is_up_r = IsEdgeCollisions(State(s.state.x - 1, s.state.y + 1), edgeCollision(left_1, Action::Right));
 							if(!(m_env.stateValid(State(s.state.x, s.state.y + 1)) && !is_up_p && !is_up_r
 									&& isSafeAtT(State(s.state.x, s.state.y + 1), left_1)
 									&& isTemporalObstacleAtT(State(s.state.x - 1, s.state.y + 1), left_1)
 									&& isTemporalObstacleAtT(State(s.state.x, s.state.y + 1), left_1 - 1))){
 								if(!IsEdgeCollisions(State(s.state.x -1, s.state.y + 1), edgeCollision(left_1 - 1, Action::Down))){
 									temp_state.dir |= 0x04;
 								}
 							}
 						}*/
 					}

 					if(m_env.stateValid(State(s.state.x - 1, s.state.y - 1))){
 						if(m_env.isObstacle(State(s.state.x, s.state.y - 1))
 								|| IsEdgeCollisions(State(s.state.x, s.state.y - 1), edgeCollision(si.start -1, Action::Up))
 								|| isTemporalObstacleAtT(State(s.state.x, s.state.y - 1), si.start)
 								|| isSafeAtT(State(s.state.x - 1, s.state.y - 1), temp_t)
 								){
 							if(!IsEdgeCollisions(State(s.state.x - 1, s.state.y - 1), edgeCollision(si.start - 1, Action::Up))){
 								temp_state.dir |= 0x08;
 							}
 						}
 /*						if(isSafeAtT(State(s.state.x - 1, s.state.y - 1), temp_t)){
 							bool is_up_p = IsEdgeCollisions(State(s.state.x, s.state.y - 1), edgeCollision(left_1 - 1, Action::Up));
 							bool is_up_r = IsEdgeCollisions(State(s.state.x - 1, s.state.y - 1), edgeCollision(left_1, Action::Right));
 							if(!(m_env.stateValid(State(s.state.x, s.state.y - 1)) && !is_up_p && !is_up_r && left_1 - 1 <= end_t && left_1 - 1 >= start_t
 									&& isSafeAtT(State(s.state.x, s.state.y - 1), left_1)
 									&& isTemporalObstacleAtT(State(s.state.x - 1, s.state.y - 1), left_1)
 									&& isTemporalObstacleAtT(State(s.state.x, s.state.y - 1), left_1 - 1))){
 								if(!IsEdgeCollisions(State(s.state.x -1, s.state.y - 1), edgeCollision(left_1 - 1, Action::Up))){
 									temp_state.dir |= 0x08;
 								}
 							}
 						}*/
 					}
 						m_env.num_generation++;
        				jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(temp_state, Action::Down,
        					   si.start - m_lastGScore));
        		   }
 /*     	    	  if(left_start_t != -1  && left_start_t <= end_t
      	    			  && !IsEdgeCollisions(State(s.state.x - 1, s.state.y),edgeCollision(left_start_t, Action::Right))){
      	    			  re_start.push_back(startTime(left_start_t, Action::Left, 0x01, true));
      	    	  }
 */     	   }

     	      if(m_env.stateValid(State(s.state.x + 1, s.state.y))
      	    		  && isTemporalObstacleAfterT(State(s.state.x + 1, s.state.y), m_lastGScore + 1, right_start_t)){
           		   JPSSIPPState temp_state = s;
           		   temp_state.dir = 0x00;
           		   temp_state.flag_wait = true;
           		   temp_state.state.x = s.state.x + 1;
           		   temp_state.state.y = s.state.y;
           		   const auto& sis = safeIntervals(State(s.state.x + 1, s.state.y));
           		   for (size_t i = 0; i < sis.size(); ++i) {
           			   const interval& si = sis[i];
           			   if (si.start - 1 > end_t)	break;
           			   if( si.end < start_t + 1)	continue;
           			   if(si.start <= m_lastGScore + 1) continue;
           			   temp_state.interval = i;
           			   temp_state.dir = 0x00;
           			   if(!IsEdgeCollisions(State(s.state.x + 1, s.state.y), edgeCollision(si.start - 1, Action::Left))){
           				   temp_state.dir |= 0x02;
           			   }else break;
           			   for(size_t ii = 0; ii < sis_s.size(); ++ii){
           				   if(sis_s[ii].start == si.start + 1 &&
            					   !IsEdgeCollisions(State(s.state.x, s.state.y), edgeCollision(si.start -1, Action::Right))){
           					   temp_state.dir |= 0x01;
           				   }
           			   }
           			   Cost temp_t = si.start + 1;
           			   if(m_env.stateValid(State(s.state.x + 1, s.state.y + 1))){ //Whether the successor need to restart the up direction
           				   if(m_env.isObstacle(State(s.state.x, s.state.y + 1))
    								|| IsEdgeCollisions(State(s.state.x, s.state.y + 1), edgeCollision(si.start - 1, Action::Down))
    								|| isTemporalObstacleAtT(State(s.state.x, s.state.y + 1), si.start)
 								|| isSafeAtT(State(s.state.x + 1, s.state.y + 1), temp_t)
 								){
           					   if(!IsEdgeCollisions(State(s.state.x + 1, s.state.y + 1), edgeCollision(si.start - 1, Action::Down))){
           						   temp_state.dir |= 0x04;
           					   	}
           				   }
 /*          				   if(isSafeAtT(State(s.state.x + 1, s.state.y + 1), temp_t)){
           					   bool is_up_p = IsEdgeCollisions(State(s.state.x, s.state.y + 1), edgeCollision(right_1 - 1, Action::Down));
           					   bool is_up_r = IsEdgeCollisions(State(s.state.x + 1, s.state.y + 1), edgeCollision(right_1, Action::Left));
           					   if(!(m_env.stateValid(State(s.state.x, s.state.y + 1)) && !is_up_p && !is_up_r
           							   && isSafeAtT(State(s.state.x, s.state.y + 1), right_1)
    										&& isTemporalObstacleAtT(State(s.state.x + 1, s.state.y + 1), right_1)
 										&& isTemporalObstacleAtT(State(s.state.x, s.state.y + 1), right_1 - 1))){
           						   if(!IsEdgeCollisions(State(s.state.x + 1, s.state.y + 1), edgeCollision(right_1 - 1, Action::Down))){
           							   temp_state.dir |= 0x04;
           						   }
           					   }
           				   }*/
           			   }
           			   if(m_env.stateValid(State(s.state.x + 1, s.state.y - 1))){
           				   if(m_env.isObstacle(State(s.state.x, s.state.y - 1))
           						   || IsEdgeCollisions(State(s.state.x, s.state.y - 1), edgeCollision(si.start -1, Action::Up))
 								   || isTemporalObstacleAtT(State(s.state.x, s.state.y - 1), si.start)
 								   || isSafeAtT(State(s.state.x + 1, s.state.y - 1), temp_t)
 								   ){
           					   if(!IsEdgeCollisions(State(s.state.x + 1, s.state.y - 1), edgeCollision(si.start - 1, Action::Up))){
           						   temp_state.dir |= 0x08;
           					   }
           				   }
/*          				   if(isSafeAtT(State(s.state.x + 1, s.state.y - 1), temp_t)){
           					   bool is_up_p = IsEdgeCollisions(State(s.state.x, s.state.y - 1), edgeCollision(right_1 - 1, Action::Up));
           					   bool is_up_r = IsEdgeCollisions(State(s.state.x + 1, s.state.y - 1), edgeCollision(right_1, Action::Left));
           					   if(!(m_env.stateValid(State(s.state.x, s.state.y - 1)) && !is_up_p && !is_up_r
           							   && isSafeAtT(State(s.state.x, s.state.y - 1), right_1)
    										&& isTemporalObstacleAtT(State(s.state.x + 1, s.state.y - 1), right_1)
 										&& isTemporalObstacleAtT(State(s.state.x, s.state.y - 1), right_1 - 1))){
           						   if(!IsEdgeCollisions(State(s.state.x + 1, s.state.y - 1), edgeCollision(right_1 - 1, Action::Up))){
           							   temp_state.dir |= 0x08;
           						   }
           					   }
           				   }*/
           			   }
           			   m_env.num_generation++;
           			   jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(temp_state, Action::Down,
           					   si.start - m_lastGScore));
           		   }
 /*    	    	  if(right_start_t != -1  && right_start_t <= end_t
      	    			  && !IsEdgeCollisions(State(s.state.x + 1, s.state.y),edgeCollision(right_start_t,Action::Left))){
      	    			  re_start.push_back(startTime(right_start_t, Action::Left, 0x02, true));
      	    	  }
*/     	  	  }

         	   if(m_env.stateValid(State(s.state.x, s.state.y + 1))
         			   && isTemporalObstacleAfterT(State(s.state.x, s.state.y + 1), m_lastGScore + 1, up_start_t)){
         		   JPSSIPPState temp_state = s;
         		   temp_state.flag_wait = true;
         		   temp_state.state.x = s.state.x;
         		   temp_state.state.y = s.state.y + 1;
        		   const auto& sis = safeIntervals(State(s.state.x, s.state.y + 1));
         		   for (size_t i = 0; i < sis.size(); ++i) {
         			   const interval& si = sis[i];
         			   if (si.start - 1 > end_t) break;
         			   if( si.end < start_t + 1)  continue;
         			   if(si.start <= m_lastGScore + 1) continue;
         			   if(!IsEdgeCollisions(State(s.state.x, s.state.y + 1), edgeCollision(si.start - 1, Action::Down))){
             			   temp_state.interval = i;
                		   temp_state.dir = 0x00;
             			   for(size_t ii = 0; ii < sis_s.size(); ++ii){
             				   if(sis_s[ii].start == si.start + 1
             						   && !IsEdgeCollisions(State(s.state.x, s.state.y), edgeCollision(si.start, Action::Up))){
             					   temp_state.dir |= 0x0b;
             					   break;
             				   }
             			   }
         				   temp_state.dir |= 0x07;
         				   m_env.num_generation++;
            			   jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(temp_state, Action::Up,
         						   si.start - m_lastGScore));
         			   }
         		   }
/*        		   if(up_start_t != -1  && up_start_t <= end_t
     	    				  && !IsEdgeCollisions(State(s.state.x, s.state.y + 1),edgeCollision(up_start_t,Action::Down))){
     	    			  re_start.push_back(startTime(up_start_t, Action::Up, 0x04, true));
         		   }
*/         	   }

     	      if(m_env.stateValid(State(s.state.x, s.state.y - 1)) &&
     	    		  isTemporalObstacleAfterT(State(s.state.x, s.state.y - 1), m_lastGScore + 1, down_start_t)){
        		   JPSSIPPState temp_state = s;
        		   temp_state.dir = 0x00;
        		   temp_state.flag_wait = true;
        		   temp_state.state.x = s.state.x;
        		   temp_state.state.y = s.state.y - 1;

        		   const auto& sis = safeIntervals(State(s.state.x, s.state.y - 1));
        		   for (size_t i = 0; i < sis.size(); ++i) {
        			   const interval& si = sis[i];
        			   if (si.start - 1 > end_t)	break;
        			   if( si.end < start_t + 1)	continue;
        			   if(si.start <= m_lastGScore + 1) continue;
        			   if(!IsEdgeCollisions(State(s.state.x, s.state.y - 1), edgeCollision(si.start - 1, Action::Up))){
            			   temp_state.interval = i;
            			   temp_state.dir = 0x00;
             			   for(size_t ii = 0; ii < sis_s.size(); ++ii){
             				   if(sis_s[ii].start == si.start + 1 &&
             						   !IsEdgeCollisions(State(s.state.x, s.state.y), edgeCollision(si.start, Action::Down))){
             					   temp_state.dir |= 0x07;
             					   break;
             				   }
             			   }
        				   temp_state.dir |= 0x0b;
        				   m_env.num_generation++;
        				   jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(temp_state, Action::Down,
        						   si.start - m_lastGScore));
        			   }
        		   }
/*     	    	  if(down_start_t != -1  && down_start_t <= end_t
     	    			 && !IsEdgeCollisions(State(s.state.x, s.state.y - 1),edgeCollision(down_start_t,Action::Up))){
     	    		  re_start.push_back(startTime(down_start_t, Action::Down, 0x08, true));
     	    	  }
*/     	      }

/*     	      std::sort(re_start.begin(), re_start.end());
     	      JPSSIPPState temp_state = s;
     	      temp_state.dir = 0x0;
     	      for(int re_i = 0; re_i < re_start.size(); re_i++){
					if(re_start[re_i].rt == -1) continue;
					temp_state.action = Action::Left;
					temp_state.dir |= re_start[re_i].dir;
					temp_state.flag_wait = re_start[re_i].flag_wait;
					int re_ii;
					for(re_ii = re_i + 1; re_ii < re_start.size(); re_ii++){
						if(re_start[re_ii].rt == re_start[re_i].rt){
							temp_state.dir |= re_start[re_ii].dir;
							if(re_start[re_ii].flag_wait) temp_state.flag_wait = re_start[re_ii].flag_wait;
						}else break;
					}
					if(re_start[re_i].rt != m_lastGScore)
					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(temp_state, Action::Left,
							re_start[re_i].rt - m_lastGScore));
					m_env.num_generation = m_env.num_generation - (re_start.size() -re_ii);
					break;
     	      }*/

          	  for ( auto& m : jps_successors) {
          		  neighbors.emplace_back(Neighbor<JPSSIPPState, JPSSIPPAction, Cost>(
  						  JPSSIPPState(m.state.state, m.state.interval, m.state.dir,  m_lastGScore + m.cost, m.state.flag_wait),
						  	  JPSSIPPAction(m.action, m.cost), m.cost));
          			  Cost hvalue = (Cost)m_env.admissibleHeuristic(m.state.state);
 				  	 if(m_env.isDebug) std::cout << "Successor-: " << m.state.state.x << " "<< m.state.state.y << " Cost ++" << m.cost + m_lastGScore << " hvalue " << hvalue << " f " << hvalue + m.cost + m_lastGScore  << " flag " << m.state.flag_wait  << " dir " << m.state.dir << "\n";
          	  }
          }
       }


    void getJPSHorizontalSuccessors(JPSSIPPState s, unsigned int dir, Cost current_cost){
     	if (isSolution(s)) {
     		flag_is_solution = true;
   			jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(s, Action::Left, current_cost));
   			return ;
     	}
        JPSSIPPState current_successor = s;
        current_successor.dir = 0x0;
 		Cost successor_start = -1, successor_end = -1; //Record the safe interval of the current successor that s can go though.
		Cost successor_next_start = -1, successor_next_end = -1; //Record the next safe interval of current successor.

		size_t successor_interval;
 		Cost up_left_t = -1, down_left_t = -1, up_right_t = -1, down_right_t = -1;
 		Cost up_start_t = -1, down_start_t = -1, right_start_t = -1, left_start_t = -1;

 		Cost par_f = (Cost)m_env.admissibleHeuristic(s.state) + current_cost;
 		Cost succ_f = 0;
     	int step = 0;
     	if((dir & 0x01) && !flag_is_solution){ // the left direction
     		JPSSIPPState temp_s = s;
     		Cost current_cost_l = current_cost;
     		while(true){
				step++;
     			m_env.num_generation++;
        		current_successor.state.x = temp_s.state.x - 1;
        		current_successor.state.y = temp_s.state.y;
         		up_left_t = -1; down_left_t = -1; up_right_t = -1; down_right_t = -1;
         		up_start_t = -1; down_start_t = -1; right_start_t = -1; left_start_t = -1;
        		if(m_env.stateValid(current_successor.state) &&
        				!IsEdgeCollisions(current_successor.state,edgeCollision(m_lastGScore + current_cost_l, Action::Right))){
             		const auto& si_s_l = safeIntervals(temp_s.state);
    				successor_start = -1; successor_end = -1;
    				successor_next_start = -1; successor_next_end = -1;
    				findSafeInterval(current_successor.state, m_lastGScore + current_cost_l + 1, successor_interval,                  //find the safe interval
    				          						successor_start, successor_end, successor_next_start, successor_next_end);
    				std::vector<startTime> re_start;
    				if(successor_start != -1){
    					current_successor.interval = successor_interval;
    					current_successor.dir = 0x00;
    					up_start_t = -1; down_start_t = -1; right_start_t = -1;
    					if(m_env.isJumpPoint(current_successor.state, m_lastGScore + current_cost_l + 1)){
    						if(m_env.stateValid(State(temp_s.state.x - 1, temp_s.state.y + 1))){
    							if(m_env.isObstacle(State(temp_s.state.x, temp_s.state.y + 1))
    									|| IsEdgeCollisions(State(temp_s.state.x, temp_s.state.y + 1),
            									edgeCollision(m_lastGScore + current_cost_l, Action::Down))
    									|| isTemporalObstacleAtT(State(temp_s.state.x, temp_s.state.y + 1), m_lastGScore + current_cost_l + 1)){
        							if(!IsEdgeCollisions(State(temp_s.state.x -1, temp_s.state.y + 1),
        									edgeCollision(m_lastGScore + current_cost_l + 1, Action::Down))){
        								up_start_t = m_lastGScore + current_cost_l + 1;
        								re_start.push_back(startTime(up_start_t, Action::Up, 0x04, false));
        							}
    							} else {
       								isTemporalObstacleAfterT(State(temp_s.state.x - 1, temp_s.state.y + 1), m_lastGScore + current_cost_l + 1, up_left_t);
        							if(up_left_t != -1 && ( up_left_t <= successor_end)){
        								if(!IsEdgeCollisions(State(temp_s.state.x - 1, temp_s.state.y + 1), edgeCollision(up_left_t, Action::Down)))
        								re_start.push_back(startTime(up_left_t, Action::Up, 0x04, true));
        							}
    							}
    						}

    						if(m_env.stateValid(State(temp_s.state.x - 1, temp_s.state.y - 1))){
    							if(m_env.isObstacle(State(temp_s.state.x, temp_s.state.y - 1))
    									|| IsEdgeCollisions(State(temp_s.state.x, temp_s.state.y - 1),
            									edgeCollision(m_lastGScore + current_cost_l, Action::Up))
    									|| isTemporalObstacleAtT(State(temp_s.state.x, temp_s.state.y - 1), m_lastGScore + current_cost_l + 1)){
        							if(!IsEdgeCollisions(State(temp_s.state.x -1, temp_s.state.y - 1),
        									edgeCollision(m_lastGScore + current_cost_l + 1, Action::Up))){
        								down_start_t = m_lastGScore + current_cost_l + 1;
        								re_start.push_back(startTime(down_start_t, Action::Up, 0x08, false));
        							}
    							} else {
       								isTemporalObstacleAfterT(State(temp_s.state.x - 1, temp_s.state.y - 1), m_lastGScore + current_cost_l + 1, down_left_t);
        							if(down_left_t != -1 && ( down_left_t <= successor_end)){
        								if(!IsEdgeCollisions(State(temp_s.state.x - 1, temp_s.state.y - 1), edgeCollision(down_left_t, Action::Up)))
        								re_start.push_back(startTime(down_left_t, Action::Down, 0x08, true));
        							}
    							}
    						}
    					}
        				Cost next_start_s = -1;
    					if(si_s_l.size() > temp_s.interval + 1){ // Whether the successor need to go back, that is, restart the right direction.
    						next_start_s = si_s_l.at(temp_s.interval + 1).start;
    						if(!IsEdgeCollisions(temp_s.state,edgeCollision(next_start_s - 1, Action::Left))
    								&& successor_end >= next_start_s - 1){
    							right_start_t = next_start_s - 1;
    							re_start.push_back(startTime(right_start_t, Action::Right, 0x02, true));
    						}
    					}
    					Cost left_start_t = -1;
						if(isTemporalObstacleAfterT(State(temp_s.state.x - 2, temp_s.state.y), m_lastGScore + current_cost_l + 1, left_start_t)
								&& left_start_t <= successor_end){
							re_start.push_back(startTime(left_start_t, Action::Left, 0x01, true));
						}

    					std::sort(re_start.begin(), re_start.end()); //For the re-start direction, choose the minimum re-start time.
    					Cost re_ac = -1;
    					unsigned int current_dir = 0x00;
    					for(int re_i = 0; re_i < re_start.size(); re_i++){
    						if(re_start[re_i].rt == -1) continue;
    						current_successor.action = Action::Left;
    						current_dir = re_start[re_i].dir;
    						for(int re_ii = re_i + 1; re_ii < re_start.size(); re_ii++){
    							if(re_start[re_ii].rt == re_start[re_i].rt){
    								current_dir |= re_start[re_ii].dir;
    							}else break;
    						}
    						re_ac = re_start[re_i].rt;
    						if(re_ac == m_lastGScore + current_cost_l + 1){
    							current_dir |= 0x01;
    						}
    						break;
    					}

    					if(re_ac != -1){
    						current_successor.dir = current_dir;
		          			jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor,
		          					Action::Left, re_ac - m_lastGScore));
		          			if(re_ac == m_lastGScore + current_cost_l + 1) break;
    					}
    	             	if (isSolution(current_successor)) {
    	             		current_successor.dir = 0x01;
    	             		flag_is_solution = true;
    	           			jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost_l + 1));
    	           			break ;
    	             	}
    	             	if(current_successor.state.x == 0) break; //check the border
    	             	if (current_successor.state.x % m_env.limit_jump == 0 || current_successor.state.y % m_env.limit_jump == 0){
	   	             	// if(step >= m_env.limit_jump){
/*    	             		if(m_env.isFCheck()){
    	             			succ_f = m_env.admissibleHeuristic(current_successor.state) + current_cost_l + 1;
    	             			if(succ_f > par_f){
    	             				current_successor.dir = 0x01;
    	             				jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost_l + 1));
    	             				break ;
    	             			}
    	             		}else*/{
	             				current_successor.dir = 0x01;
	             				jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost_l + 1));
	             				break ;
    	             		}
    	             	}
    					current_cost_l++;
    					temp_s = current_successor;
    				} else break;

        		} else {
        			break;
        		}
     		}
     	}

		step = 0;
     	if((dir & 0x02) && !flag_is_solution){ // the left direction
     		JPSSIPPState temp_s = s;
     		Cost current_cost_l = current_cost;
     		while(true){
     			m_env.num_generation++;
				step++;
        		current_successor.state.x = temp_s.state.x + 1;
        		current_successor.state.y = temp_s.state.y;
         		up_left_t = -1; down_left_t = -1; up_right_t = -1; down_right_t = -1;
         		up_start_t = -1; down_start_t = -1; right_start_t = -1; left_start_t = -1;
        		if(m_env.stateValid(current_successor.state) &&
        				!IsEdgeCollisions(current_successor.state,edgeCollision(m_lastGScore + current_cost_l, Action::Left))){
             		const auto& si_s_l = safeIntervals(temp_s.state);

    				successor_start = -1; successor_end = -1;
    				successor_next_start = -1; successor_next_end = -1;
    				findSafeInterval(current_successor.state, m_lastGScore + current_cost_l + 1, successor_interval,                  //find the safe interval
    				          						successor_start, successor_end, successor_next_start, successor_next_end);
    				std::vector<startTime> re_start;
    				if(successor_start != -1){
    					current_successor.interval = successor_interval;
    					current_successor.dir = 0x00;
    					up_start_t = -1; down_start_t = -1; right_start_t = -1;
    					if(m_env.isJumpPoint(current_successor.state, m_lastGScore + current_cost_l + 1)){
    						if(m_env.stateValid(State(temp_s.state.x + 1, temp_s.state.y + 1))){
    							if(m_env.isObstacle(State(temp_s.state.x, temp_s.state.y + 1))
    									|| IsEdgeCollisions(State(temp_s.state.x, temp_s.state.y + 1),
            									edgeCollision(m_lastGScore + current_cost_l, Action::Down))
    									|| isTemporalObstacleAtT(State(temp_s.state.x, temp_s.state.y + 1), m_lastGScore + current_cost_l + 1)){
        							if(!IsEdgeCollisions(State(temp_s.state.x + 1, temp_s.state.y + 1),
        									edgeCollision(m_lastGScore + current_cost_l + 1, Action::Down))){
        								up_start_t = m_lastGScore + current_cost_l + 1;
        								re_start.push_back(startTime(up_start_t, Action::Up, 0x04, false));
        							}
    							} else {
       								isTemporalObstacleAfterT(State(temp_s.state.x + 1, temp_s.state.y + 1), m_lastGScore + current_cost_l + 1, up_left_t);
        							if(up_left_t != -1 && ( up_left_t <= successor_end)){
        								if(!IsEdgeCollisions(State(temp_s.state.x + 1, temp_s.state.y + 1), edgeCollision(up_left_t, Action::Down)))
        								re_start.push_back(startTime(up_left_t, Action::Up, 0x04, true));
        							}
    							}
    						}

    						if(m_env.stateValid(State(temp_s.state.x + 1, temp_s.state.y - 1))){
    							if(m_env.isObstacle(State(temp_s.state.x, temp_s.state.y - 1))
    									|| IsEdgeCollisions(State(temp_s.state.x, temp_s.state.y - 1),
            									edgeCollision(m_lastGScore + current_cost_l, Action::Up))
    									|| isTemporalObstacleAtT(State(temp_s.state.x, temp_s.state.y - 1), m_lastGScore + current_cost_l + 1)){
        							if(!IsEdgeCollisions(State(temp_s.state.x + 1, temp_s.state.y - 1),
        									edgeCollision(m_lastGScore + current_cost_l + 1, Action::Up))){
        								down_start_t = m_lastGScore + current_cost_l + 1;
        								re_start.push_back(startTime(down_start_t, Action::Up, 0x08, false));
        							}
    							} else {
       								isTemporalObstacleAfterT(State(temp_s.state.x + 1, temp_s.state.y - 1), m_lastGScore + current_cost_l + 1, down_left_t);
        							if(down_left_t != -1 && ( down_left_t <= successor_end)){
        								if(!IsEdgeCollisions(State(temp_s.state.x + 1, temp_s.state.y - 1), edgeCollision(down_left_t, Action::Up)))
        								re_start.push_back(startTime(down_left_t, Action::Down, 0x08, true));
        							}
    							}
    						}
    					}
        				Cost next_start_s = -1;
    					if(si_s_l.size() > temp_s.interval + 1){ // Whether the successor need to go back, that is, restart the left direction.
    						next_start_s = si_s_l.at(temp_s.interval + 1).start;
    						if(!IsEdgeCollisions(temp_s.state,edgeCollision(next_start_s - 1, Action::Right))
    								&& successor_end >= next_start_s - 1){
    							right_start_t = next_start_s - 1;
    							re_start.push_back(startTime(right_start_t, Action::Left, 0x01, true));
    						}
    					}
						Cost left_t = -1;
						if(isTemporalObstacleAfterT(State(temp_s.state.x + 2, temp_s.state.y), m_lastGScore + current_cost_l + 1, left_t)
								&& left_t <= successor_end){
							re_start.push_back(startTime(left_t, Action::Right, 0x02, true));
						}
    					std::sort(re_start.begin(), re_start.end()); //For the re-start direction, choose the minimum re-start time.
    					Cost re_ac = -1;
    					unsigned int current_dir = 0x00;
    					for(int re_i = 0; re_i < re_start.size(); re_i++){
    						if(re_start[re_i].rt == -1) continue;
    						current_successor.action = Action::Left;
    						current_dir = re_start[re_i].dir;
    						for(int re_ii = re_i + 1; re_ii < re_start.size(); re_ii++){
    							if(re_start[re_ii].rt == re_start[re_i].rt){
    								current_dir |= re_start[re_ii].dir;
    							}else break;
    						}
    						re_ac = re_start[re_i].rt;
    						if(re_ac == m_lastGScore + current_cost_l + 1){
    							current_dir |= 0x02;
    						}
    						break;
    					}

    					if(re_ac != -1){
    						current_successor.dir = current_dir;
  		          			jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor,
    		          					Action::Right, re_ac - m_lastGScore));
  		          			if(re_ac == m_lastGScore + current_cost_l + 1) break;

    						// if(re_ac == m_lastGScore + current_cost_l + 1) current_successor.dir = current_dir;
							// else current_successor.dir = 0x02;
							// //  std::cout << " Put it in right " << current_successor.state.x << " " << current_successor.state.y << " " << current_g + 1<< " \n";
		        			// jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, 
					 		// Action::Right, current_cost_l + 1));
							//  break;

    					}

    	             	if (isSolution(current_successor)) {
    	             		current_successor.dir = 0x02;
    	             		flag_is_solution = true;
    	           			jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost_l + 1));
    	           			break ;
    	             	}
    	             	if(current_successor.state.x == m_env.getDimX() - 1) break; //check the border
    	                if(current_successor.state.x % m_env.limit_jump == 0 || current_successor.state.y % m_env.limit_jump == 0){
    	             	// if(step >= m_env.limit_jump){
/*    	             		if(m_env.isFCheck()){
    	             			succ_f = m_env.admissibleHeuristic(current_successor.state) + current_cost_l + 1;
    	             			if(succ_f > par_f){
    	             				current_successor.dir = 0x02;
    	             				jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost_l + 1));
    	             				break ;
    	             			}
    	           			}else*/{
	             				current_successor.dir = 0x02;
	             				jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost_l + 1));
	             				break ;
    	           			}
    	             	}
    					current_cost_l++;
    					temp_s = current_successor;
    				} else break;

        		} else {
        			break;
        		}
     		}
     	}
    }



       void getJPSVerticalSuccessors(JPSSIPPState s, unsigned int dir, Cost current_cost){
             	if (isSolution(s)) {
             		flag_is_solution = true;
           			jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(s, Action::Left, current_cost));
           			return ;
             	}

                JPSSIPPState current_successor = s;
                current_successor.dir = 0x0;
         		Cost successor_start = -1, successor_end = -1; //Record the safe interval of the current successor that s can go though.
				Cost successor_next_start = -1, successor_next_end = -1; //Record the next safe interval of current successor.
				size_t successor_interval = -1;
         		Cost up_left_t = -1, down_left_t = -1, up_right_t = -1, down_right_t = -1;
         		Cost up_start_t = -1, down_start_t = -1, right_start_t = -1, left_start_t = -1;

         		Cost par_f = (Cost)m_env.admissibleHeuristic(s.state) + current_cost;
         		Cost succ_f = 0;
         		int step = 0;
            	if((dir & 0x04) && !flag_is_solution){
             		JPSSIPPState temp_s = s;
             		Cost current_cost_l = current_cost;
            		while(true){
            			step++;
            			current_successor.state.x = temp_s.state.x;
            			current_successor.state.y = temp_s.state.y + 1;
                 		up_left_t = -1; down_left_t = -1; up_right_t = -1; down_right_t = -1;
                 		up_start_t = -1; down_start_t = -1; right_start_t = -1; left_start_t = -1;
     					if(current_cost_l != 0){
     						temp_s.dir = 0x03;
     						m_env.num_generation++;
     						getJPSHorizontalSuccessors(temp_s, 0x03, current_cost_l);
     					}

                		if(m_env.stateValid(current_successor.state) &&
                				!IsEdgeCollisions(current_successor.state, edgeCollision(m_lastGScore + current_cost_l, Action::Down))){
                     		const auto& si_s_l = safeIntervals(temp_s.state);

                			findSafeInterval(current_successor.state, m_lastGScore + current_cost_l + 1, successor_interval,
                      				          				successor_start, successor_end, successor_next_start, successor_next_end);
            				up_start_t = -1; down_start_t = -1;
            				std::vector<startTime> re_start;
            				if(successor_start != -1){
            					current_successor.interval = successor_interval;
            					current_successor.dir = 0x00;
            					Cost next_start_s = -1;
            					bool flag_re_down = false;
               					if(si_s_l.size() > temp_s.interval + 1){ // check whether go back
               						next_start_s = si_s_l.at(temp_s.interval + 1).start;
               						if(!IsEdgeCollisions(temp_s.state, edgeCollision(next_start_s - 1, Action::Up)) && successor_end >= next_start_s - 1){
               							JPSSIPPState temp_state = current_successor;
               							temp_state.dir = 0x08;
               							down_start_t = next_start_s -1;
               							flag_re_down = true;
               							re_start.push_back(startTime(down_start_t, Action::Down, 0x0b, true));
               						}
               					}

            					// if((isTemporalObstacleAfterT(State(temp_s.state.x, temp_s.state.y + 2), m_lastGScore + current_cost_l + 1, up_start_t) && up_start_t <= successor_end)
            					// 		|| (isTemporalObstacleAfterT(State(temp_s.state.x - 1, temp_s.state.y + 1), m_lastGScore + current_cost_l + 1, up_left_t) && up_left_t <= successor_end)
             					// 		|| (isTemporalObstacleAfterT(State(temp_s.state.x + 1, temp_s.state.y + 1), m_lastGScore + current_cost_l + 1, up_right_t) && up_right_t <= successor_end)
         						// 	){
								// 		current_successor.dir = 0x00;
								// 		if(down_start_t == m_lastGScore + current_cost_l + 1 ){
								// 			current_successor.dir = 0x08;
								// 		} 
								// 		if(!IsEdgeCollisions(State(temp_s.state.x, temp_s.state.y + 2), edgeCollision(m_lastGScore + current_cost_l + 1, Action::Down))) {
								// 			current_successor.dir |= 0x04;
								// 		}
										
								// 		if(current_successor.dir == 0 && down_start_t != -1){
								// 			if(up_left_t != -1 && up_left_t != m_lastGScore + current_cost_l + 2 && m_env.stateValid(State(temp_s.state.x - 1, temp_s.state.y + 1)))
								// 				current_successor.dir |= 0x01;
								// 			if(up_right_t != -1 && up_right_t != m_lastGScore + current_cost_l + 2 && m_env.stateValid(State(temp_s.state.x + 1, temp_s.state.y + 1)))
								// 				current_successor.dir |= 0x02;
								// 		}else{
								// 			if(up_left_t != m_lastGScore + current_cost_l + 2 && m_env.stateValid(State(temp_s.state.x - 1, temp_s.state.y + 1)))
								// 				current_successor.dir |= 0x01;
								// 			if(up_right_t != m_lastGScore + current_cost_l + 2 && m_env.stateValid(State(temp_s.state.x + 1, temp_s.state.y + 1)))
								// 				current_successor.dir |= 0x02;

								// 		}
										
								// 		if(current_successor.dir != 0x00){
								// 			jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost_l + 1));
								// 			break;
								// 		}

         						// 	// if(down_start_t == m_lastGScore + current_cost_l + 1){
         						// 	// 	current_successor.dir = 0x0f;
         						// 	// } else {
         						// 	// 	current_successor.dir = 0x07;
         						// 	// }
             					// 	// current_successor.action = Action::Up;

             					// 	// jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost_l + 1));
             					// 	// break;
             					// } else {
                				// 	if(flag_re_down){
                				// 		if(down_start_t == m_lastGScore + current_cost_l + 1) current_successor.dir = 0x0f;
                				// 		else current_successor.dir = 0x07;
                    			// 		jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost_l + 1));
                    			// 		break;
                				// 	}
                				// }
               					if((isTemporalObstacleAfterT(State(temp_s.state.x, temp_s.state.y + 2), m_lastGScore + current_cost_l + 1, up_start_t) && up_start_t <= successor_end)){
               						re_start.push_back(startTime(up_start_t, Action::Up, 0x07, true));
               					}
               					if((isTemporalObstacleAfterT(State(temp_s.state.x - 1, temp_s.state.y + 1), m_lastGScore + current_cost_l + 1, up_left_t) && up_left_t <= successor_end)){
               						re_start.push_back(startTime(up_left_t, Action::Left, 0x01, true));
               					}
               					if((isTemporalObstacleAfterT(State(temp_s.state.x + 1, temp_s.state.y + 1), m_lastGScore + current_cost_l + 1, up_right_t) && up_right_t <= successor_end)){
               						re_start.push_back(startTime(up_right_t, Action::Right, 0x02, true));
               					}

            					std::sort(re_start.begin(), re_start.end()); //For the re-start direction, choose the minimum re-start time.
            					Cost re_ac = -1;
            					unsigned int current_dir = 0x00;
            					for(int re_i = 0; re_i < re_start.size(); re_i++){
            						if(re_start[re_i].rt == -1) continue;
            						current_successor.action = Action::Up;
            						current_dir = re_start[re_i].dir;
            						for(int re_ii = re_i + 1; re_ii < re_start.size(); re_ii++){
            							if(re_start[re_ii].rt == re_start[re_i].rt){
            								current_dir |= re_start[re_ii].dir;
            							}else break;
            						}
            						re_ac = re_start[re_i].rt;
            						if(re_ac == m_lastGScore + current_cost_l + 1){
            							current_dir |= 0x07;
            						}
            						break;
            					}
            					if(re_ac != -1){
            						current_successor.dir = current_dir;
          		          			jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor,
            		          					Action::Up, re_ac - m_lastGScore));
          		          			if(re_ac == m_lastGScore + current_cost_l + 1) break;
            					}

            	             	if (isSolution(current_successor)) {
            	             		current_successor.dir = 0x07;
            	             		flag_is_solution = true;
            	           			jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost_l + 1));
            	           			break ;
            	             	}

            	             	if(current_successor.state.x % m_env.limit_jump == 0 || current_successor.state.y % m_env.limit_jump == 0){
            	             	// if(step >= m_env.limit_jump){
            	             		if(current_successor.state.y == m_env.getDimY() - 1) current_successor.dir = 0x03; //check the border
            	             		else current_successor.dir = 0x07;
            	             		if(m_env.isFCheck()){
            	             			succ_f = m_env.admissibleHeuristic(current_successor.state) + current_cost_l + 1;
            	             			if(succ_f > par_f){
//            	             				current_successor.dir = 0x07;
            	             				jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost_l + 1));
            	             				break ;
            	             			}
            	             		}else {
//            	             			current_successor.dir = 0x07;
            	             			jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost_l + 1));
            	             			break ;
            	             		}
            	             	}
            					current_cost_l++;
            					temp_s = current_successor;
            				} else break;
                		}else break;
            		}
             	}
            	step = 0;
            	if((dir & 0x08) && !flag_is_solution){
             		JPSSIPPState temp_s = s;
             		Cost current_cost_l = current_cost;
            		while(true){
            			step++;
            			current_successor.state.x = temp_s.state.x;
            			current_successor.state.y = temp_s.state.y - 1;
                 		up_left_t = -1; down_left_t = -1; up_right_t = -1; down_right_t = -1;
                 		up_start_t = -1; down_start_t = -1; right_start_t = -1; left_start_t = -1;
     					if(current_cost_l != 0){
     						temp_s.dir = 0x03;
     						getJPSHorizontalSuccessors(temp_s, 0x03, current_cost_l);
     						m_env.num_generation++;
     					}
                		if(m_env.stateValid(current_successor.state) &&
                				!IsEdgeCollisions(current_successor.state, edgeCollision(m_lastGScore + current_cost_l, Action::Up))){
                     		const auto& si_s_l = safeIntervals(temp_s.state);

                			findSafeInterval(current_successor.state, m_lastGScore + current_cost_l + 1, successor_interval,
                      				          				successor_start, successor_end, successor_next_start, successor_next_end);
            				up_start_t = -1; down_start_t = -1;
            				std::vector<startTime> re_start;
            				if(successor_start != -1){
            					current_successor.interval = successor_interval;
            					current_successor.dir = 0x00;
            					Cost next_start_s = -1;
            					bool flag_re_up = false;
               					if(si_s_l.size() > temp_s.interval + 1){ // check whether go back
               						next_start_s = si_s_l.at(temp_s.interval + 1).start;
               						if(!IsEdgeCollisions(temp_s.state, edgeCollision(next_start_s - 1, Action::Down)) && successor_end >= next_start_s - 1){
               							down_start_t = next_start_s -1;
               							flag_re_up = true;
               							re_start.push_back(startTime(down_start_t, Action::Up, 0x07, true));
               						}
               					}

               					if((isTemporalObstacleAfterT(State(temp_s.state.x, temp_s.state.y - 2), m_lastGScore + current_cost_l + 1, up_start_t) && up_start_t <= successor_end)){
               						re_start.push_back(startTime(up_start_t, Action::Down, 0x0b, true));
               					}
               					if((isTemporalObstacleAfterT(State(temp_s.state.x - 1, temp_s.state.y - 1), m_lastGScore + current_cost_l + 1, up_left_t) && up_left_t <= successor_end)){
               						re_start.push_back(startTime(up_left_t, Action::Left, 0x01, true));
               					}
               					if((isTemporalObstacleAfterT(State(temp_s.state.x + 1, temp_s.state.y - 1), m_lastGScore + current_cost_l + 1, up_right_t) && up_right_t <= successor_end)){
               						re_start.push_back(startTime(up_right_t, Action::Right, 0x02, true));
               					}
            					std::sort(re_start.begin(), re_start.end()); //For the re-start direction, choose the minimum re-start time.
            					Cost re_ac = -1;
            					unsigned int current_dir = 0x00;
            					for(size_t re_i = 0; re_i < re_start.size(); re_i++){
            						if(re_start[re_i].rt == -1) continue;
            						current_successor.action = Action::Down;
            						current_dir = re_start[re_i].dir;
            						for(size_t re_ii = re_i + 1; re_ii < re_start.size(); re_ii++){
            							if(re_start[re_ii].rt == re_start[re_i].rt){
            								current_dir |= re_start[re_ii].dir;
            							}else break;
            						}
            						re_ac = re_start[re_i].rt;
            						if(re_ac == m_lastGScore + current_cost_l + 1){
            							current_dir |= 0x0b;
            						}
            						break;
            					}
            					if(re_ac != -1){
            						current_successor.dir = current_dir;
          		          			jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor,
            		          					Action::Down, re_ac - m_lastGScore));
          		          			if(re_ac == m_lastGScore + current_cost_l + 1) break;
            					}
								
             					// if((isTemporalObstacleAfterT(State(temp_s.state.x, temp_s.state.y - 2), m_lastGScore + current_cost_l + 1, up_start_t) && up_start_t <= successor_end)
            					// 		|| (isTemporalObstacleAfterT(State(temp_s.state.x - 1, temp_s.state.y - 1), m_lastGScore + current_cost_l + 1, up_left_t) && up_left_t <= successor_end)
             					// 		|| (isTemporalObstacleAfterT(State(temp_s.state.x + 1, temp_s.state.y - 1), m_lastGScore + current_cost_l + 1, up_right_t) && up_right_t <= successor_end)
         						// 	){
								// 		if(down_start_t == m_lastGScore + current_cost_l + 1){
								// 			current_successor.dir = 0x04;
								// 		} 
								// 		if(!IsEdgeCollisions(State(temp_s.state.x, temp_s.state.y - 2), edgeCollision(m_lastGScore + current_cost_l + 1, Action::Up))) {
								// 			current_successor.dir |= 0x08;
								// 		}
										
								// 		if(current_successor.dir == 0 && down_start_t != -1){
								// 			if(up_left_t != -1 && up_left_t != m_lastGScore + current_cost_l + 2 && m_env.stateValid(State(temp_s.state.x - 1, temp_s.state.y - 1)))
								// 			current_successor.dir |= 0x01;
								// 			if(up_right_t !=  -1 && up_right_t != m_lastGScore + current_cost_l + 2 && m_env.stateValid(State(temp_s.state.x + 1, temp_s.state.y - 1)))
								// 			current_successor.dir |= 0x02;
								// 		}else{
								// 			if(up_left_t != m_lastGScore + current_cost_l + 2 && m_env.stateValid(State(temp_s.state.x - 1, temp_s.state.y - 1)))
								// 			current_successor.dir |= 0x01;
								// 			if(up_right_t != m_lastGScore + current_cost_l + 2 && m_env.stateValid(State(temp_s.state.x + 1, temp_s.state.y - 1)))
								// 			current_successor.dir |= 0x02;

								// 		}

								// 		if(current_successor.dir != 0){
								// 			current_successor.action = Action::Up;
								// 			jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost_l + 1));
								// 			break;
								// 		}

         						// 	// if(down_start_t == m_lastGScore + current_cost_l + 1){
         						// 	// 	current_successor.dir = 0x0f;
         						// 	// } else {
         						// 	// 	current_successor.dir = 0x0b;
         						// 	// }
             					// 	// current_successor.action = Action::Up;
             					// 	// jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost_l + 1));
             					// 	// break;
             					// } else {
                				// 	if(flag_re_up){
                				// 			if(down_start_t == m_lastGScore + current_cost_l + 1) current_successor.dir = 0x0f;
                				// 			else current_successor.dir = 0x0b;
                    			// 			jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost_l + 1));
                    			// 			break;
                				// 	}
                				// }

            	             	if (isSolution(current_successor)) {
            	             		flag_is_solution = true;
            	             		current_successor.dir = 0x0b;
            	           			jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost_l + 1));
            	           			break ;
            	             	}


            	             	if(current_successor.state.x % m_env.limit_jump == 0 || current_successor.state.y % m_env.limit_jump == 0){
           	             		// if(step >= m_env.limit_jump){
    	             				if(current_successor.state.y == 0) current_successor.dir = 0x03; //check the border
    	             				else current_successor.dir = 0x0b;
            	             		if(m_env.isFCheck()){
            	             			succ_f = m_env.admissibleHeuristic(current_successor.state) + current_cost_l + 1;
            	             			if(succ_f > par_f){
//            	             				current_successor.dir = 0x0b;
            	           					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost_l + 1));
            	           					break ;
            	             			}
            	             		}else{
//        	             				current_successor.dir = 0x0b;
        	           					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost_l + 1));
        	           					break ;
            	             		}
            	             	}
            					current_cost_l++;
            					temp_s = current_successor;
            				} else break;
                		}else break;
            		}
             	}
      }


    void onExpandNode(const JPSSIPPState& s, Cost fScore, Cost gScore) {
    	m_env.num_expansion++;
    	m_lastGScore = gScore;
        m_env.onExpandNode(s.state, fScore, gScore);
    }

    void onDiscover(const JPSSIPPState& s, Cost fScore, Cost gScore) {
    	m_env.onDiscover(s.state, fScore, gScore);
    }

    void setCollisionIntervals(const Location& location,
                               const std::vector<interval>& intervals) {
    	if(intervals.size() == 0) return;
    	int index = m_env.getIndex(location);
    	m_safeIntervals_t[index].clear();
    	std::vector<interval> sortedIntervals(intervals);
    	std::sort(sortedIntervals.begin(), sortedIntervals.end());

        int start = 0;
        int lastEnd = 0;
        for (const auto& interval : sortedIntervals) {
        	assert(interval.start <= interval.end);
        	assert(start <= interval.start);

        	if (start <= interval.start - 1) {
        		m_safeIntervals_t[index].push_back({start, interval.start - 1});
        	}
        	start = interval.end + 1;
        	lastEnd = interval.end;
        }
        if (lastEnd < std::numeric_limits<int>::max()) {
        	m_safeIntervals_t[index].push_back(
        		{start, std::numeric_limits<int>::max()});
        }
    }

    void setEdgeCollisionSize(const int& dimx, const int& dimy){
    	m_edgeCollision_t.resize(dimx * dimy);
    	m_safeIntervals_t.resize(dimx * dimy);
    	return ;
    }

    void setEdgeCollisions(const Location& location,
  		  	  	  	  	const std::vector<edgeCollision>& edge_collision) {
    	int index = m_env.getIndex(location);
    	std::vector<edgeCollision> sortedEdges(edge_collision);
    	std::sort(sortedEdges.begin(), sortedEdges.end());
    	if(sortedEdges.size() > 0){
    		m_edgeCollision_t[index].clear();
    		for(const auto& ec : sortedEdges){
    			m_edgeCollision_t[index].push_back(ec);
    		}
    	}
    }

    bool IsEdgeCollisions(const Location& location, const edgeCollision& ec){
    	if(!m_env.stateValid(location) || !m_env.isTemporalObstacle(location)) return false;
    	int index = m_env.getIndex(location);
    	if(m_edgeCollision_t[index].size() == 0) return false;
    	int low =0, high = m_edgeCollision_t[index].size() - 1, mid = -1;
    	while (low <= high){
    		if(m_edgeCollision_t[index][low] == ec) return true;
    		if(m_edgeCollision_t[index][high] == ec) return true;
    		mid = low + (high - low)/2;
    		if(m_edgeCollision_t[index][mid] == ec) return true;
    		else if(m_edgeCollision_t[index][mid].t < ec.t){
    			low = mid + 1;
    		} else high = mid -1;
    	}
    	return false;
    	/*for(auto& cec : (m_edgeCollision_t[index])){
    		if(cec == ec) return true;
    	}
    	return false;*/
    }

    bool findSafeInterval(const State& state, Cost time, size_t& interval)
    {
      const auto& si = safeIntervals(state);
      for (size_t idx = 0; idx < si.size(); ++idx) {
        if (si[idx].start <= time && si[idx].end >= time) {
          interval = idx;
          return true;
        }
      }
      return false;
    }

    bool findSafeInterval(const State& state, Cost time, size_t& interval, Cost &start_t, Cost &end_t)
    {
      const auto& si = safeIntervals(state);
      for (size_t idx = 0; idx < si.size(); ++idx) {
        if (si[idx].start <= time && si[idx].end >= time) {
          interval = idx;
          start_t = si[idx].start;
          end_t = si[idx].end;
          return true;
        }
      }
      return false;
    }

    bool findSafeInterval(const State& state, Cost time, size_t& interval, Cost &start_t, Cost &end_t, Cost &next_start, Cost &next_end)
    {
      const auto& si = safeIntervals(state);
      start_t = -1;
      end_t = -1;
      next_start = -1;
      next_end = -1;
      bool first_safe = true;
      for (size_t idx = 0; idx < si.size(); ++idx) {
          if(first_safe && si[idx].end >= time){
        	  first_safe = false;
        	  next_start = si[idx].start;
        	  next_end = si[idx].end;
          }
    	  if (si[idx].start <= time && si[idx].end >= time) {
    		  first_safe = false;
    		  interval = idx;
    		  start_t = si[idx].start;
    		  end_t = si[idx].end;
    		  if(idx + 1 < si.size()){
    			  next_start = si[idx + 1].start;
    			  next_end = si[idx + 1].end;
    		  } else{
    			  next_start = -1;
    			  next_end = -1;
    		  }
    		  return true;
        }
      }
      return false;
    }

    bool isTemporalObstacleAtT(const Location& location, Cost time){
    	if(!m_env.isTemporalObstacle(location)) return false;
        const auto& si = safeIntervals(location, true);
        if(si.size() == 0) return true;
        for (size_t idx = 0; idx < si.size(); ++idx) {
          if (si[idx].start <= time && si[idx].end >= time) {
        	  return false;
          }
        }
        return true;
    }



    bool isTemporalObstacleAfterT(const Location& location, Cost time, Cost &start_time){ // whether the obstacle appears after the time
       	if(!m_env.isTemporalObstacle(location)) return false;
       	if(!m_env.isTemporalObstacleAfterT(location, time)){
       		return false;
       	}
        const auto& si = safeIntervals(location, true);
        start_time = -1;
        if(si.size() == 0) return true;
        Cost start_ = 0, end_ = si[0].start - 1;
        for (size_t idx = 0; idx < si.size() - 1; ++idx) {
        	if(start_ <= end_ && end_ >= time){ start_time = end_; return true;}
        	start_ = si[idx].end + 1;
        	end_ = si[idx + 1].start - 1;
        }
    	if(start_ <= end_ && end_ >= time) {start_time = end_; return true;}

        if(si.back().end == std::numeric_limits<Cost>::max()) { return false;}
        else { start_time = -1; return true;}
    }

    bool isSafeAtT(const Location& location, Cost &start_time){ // whether the obstacle appears after the time
    	if(!m_env.isTemporalObstacle(location)) return true;
        const auto& si = safeIntervals(location, true);
       if(si.size() == 0) return false;
        Cost start_ = 0, end_ = si[0].start - 1;
        for (size_t idx = 0; idx < si.size(); ++idx) {
        	if(si[idx].start == start_time) return true;
        }
        return false;
    }

   private:
    const std::vector<interval>& safeIntervals(const Location& location) {
      static std::vector<interval> defaultInterval(
          1, {0, std::numeric_limits<Cost>::max()});
      if(!m_env.isTemporalObstacle(location)) return defaultInterval;
      int index = m_env.getIndex(location);
//      const auto iter = m_safeIntervals_t[index];
//      std::cout << location.x << " " << location.y << "size " << iter.size() << " \n";
      return m_safeIntervals_t[index];
    }

    const std::vector<interval>& safeIntervals(const Location& location, bool is_temporal) {
      if(is_temporal){
	    int index = m_env.getIndex(location);
      	return m_safeIntervals_t[index];
	  }else{
	    static std::vector<interval> defaultInterval(
          1, {0, std::numeric_limits<Cost>::max()});
		return defaultInterval;
	  }
    }

   private:
    Environment& m_env;
    Cost m_lastGScore;
    std::vector<std::vector<interval>> m_safeIntervals_t;
    std::vector<std::vector<edgeCollision>> m_edgeCollision_t;
    std::vector<Neighbor<JPSSIPPState, Action, Cost>> jps_successors;
    bool flag_is_solution = false;

  };

 private:
  JPSSIPPEnvironment m_env;
  JPSAStar<JPSSIPPState, JPSSIPPAction, Cost, JPSSIPPEnvironment, JPSSIPPStateHasher, JPSSIPPStateHasher> m_astar;
};

}  // namespace libMultiRobotPlanning
