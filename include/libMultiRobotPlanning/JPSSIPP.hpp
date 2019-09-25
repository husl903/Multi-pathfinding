#pragma once

#ifdef USE_FIBONACCI_HEAP
#include <boost/heap/fibonacci_heap.hpp>
#endif



#include <unordered_map>
#include <unordered_set>

#include "a_star_jpssipp.hpp"



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
  };


 public:
  JPSSIPP(Environment& environment) : m_env(environment), m_astar(m_env) {}

  void setCollisionIntervals(const Location& location,
                             const std::vector<interval>& intervals) {
    m_env.setCollisionIntervals(location, intervals);
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
    size_t interval;
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
      return m_env.admissibleHeuristic(s.state);
    }

    bool mightHaveSolution(const State& goal) {
      const auto& si = safeIntervals(m_env.getLocation(goal));
      return m_env.isSolution(goal) &&
        !si.empty() &&
        si.back().end == std::numeric_limits<Cost>::max();
    }

    bool isSolution(const JPSSIPPState& s) {
      return m_env.isSolution(s.state) &&
             safeIntervals(m_env.getLocation(s.state)).at(s.interval).end ==
                 std::numeric_limits<Cost>::max();
    }

/*
    void getNeighbors(
        const JPSSIPPState& s,
        std::vector<Neighbor<JPSSIPPState, JPSSIPPAction, Cost> >& neighbors) {


      std::vector<Neighbor<State, Action, Cost> > motions;

      JPSSIPPState s_temp = s;

      if(!m_env.isJPS()){
    	  m_env.getNeighbors(s_temp.state, motions);
 //   	  std::cout << "The number of neighbors: " << motions.size() << " " << std::endl;
 //   	  std::cout << "Current state ---------------------------------------" << s.state.x << " " << s.state.y << "\n";
          for (const auto& m : motions) {
        	  m_env.num_generation++;
//        	  std::cout << "Current successor: " << m.state.x << " " << m.state.y << "\n";
            Cost m_time = m.cost;
            Cost start_t = m_lastGScore + m_time;
            Cost end_t =
                safeIntervals(m_env.getLocation(s.state)).at(s.interval).end;

            const auto& sis = safeIntervals(m_env.getLocation(m.state));
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
              if (m_env.isCommandValid(s.state, m.state, m.action, m_lastGScore,
                                       end_t, si.start, si.end, t, m_time) && !IsEdgeCollisions(m.state, edgeCollision(t - 1, a_temp))) {
                neighbors.emplace_back(Neighbor<JPSSIPPState, JPSSIPPAction, Cost>(
                    JPSSIPPState(m.state, i,0xf), JPSSIPPAction(m.action, m.cost),
                    t - m_lastGScore));
              }
            }
          }
      }else {

    	  Cost start_t = m_lastGScore;
		  Cost end_t =
				  safeIntervals(m_env.getLocation(s.state)).at(s.interval).end;

		  State state_re = s.state;

//		  std::cout << "Current State " << s.state.x << "  " << s.state.y <<" Gscore" << m_lastGScore << " interval " << s.interval << " Direction " << s.dir << "  ********************************\n";
		  jps_successors.clear();
		  Cost wait_time_l = 0, wait_time_r = 0, wait_time_u = 0, wait_time_d = 0;

   	      if(s_temp.dir & 0x1){
 //  	    	  std::cout << "Left : \n";
   	    	  s_temp.action = Action::Left;
   	    	  state_re.x = s.state.x -1;
   	    	  state_re.y = s.state.y;
   	    	  if(m_env.stateValid(state_re)){
   	    		  const auto& sis = safeIntervals(m_env.getLocation(state_re));
   	    		  for (size_t i = 0; i < sis.size(); ++i) {
   	     			  const interval& si = sis[i];
   	     			  if (si.start - 1 > end_t || si.end - 1 < start_t) {
   	     				  continue;
   	     			  }
//  	     			  std::cout << "Start " << si.start << " " << s.state.x << " " << sis.state.y << " " << std::endl;
//     	     		std::cout << "Time----- " << si.start - 1 << " end_t " << end_t << " " << si.end - 1 << " Start_t " << start_t << " "<<std::endl;
   	     			  if(si.start -1 <= m_lastGScore){
   	     				  getJPSSuccessors(s_temp, Action::Left, 0, 0);
   	     			  }else{
   	     				  getJPSSuccessors(s_temp, Action::Left, si.start - 1 -m_lastGScore, 0);
   	     				  wait_time_l = si.start - 1 -m_lastGScore;
   	     			  }
   	    		  }
   	    	  }
   	      }
   	      if(s_temp.dir & 0x2){
   	    	  s_temp.action = Action::Right;
   	    	  state_re.x = s.state.x + 1;
   	    	  state_re.y = s.state.y;
 //  	    	 std::cout << "Right Right : " << state_re.x << " " << state_re.y << " direction \n";
   	    	  if(m_env.stateValid(state_re)){
   	    		  const auto& sis = safeIntervals(m_env.getLocation(state_re));
   	    		  const auto& tt = safeIntervals(m_env.getLocation(State(2,7)));
 //  	    		  std::cout << "The size of sis " << sis.size() << " The size of (2,7)" << tt.size() <<std::endl;
   	    		  for (size_t i = 0; i < sis.size(); ++i) {
   	     			  const interval& si = sis[i];
   	     			  if (si.start - 1 > end_t  || si.end - 1 < start_t ) {
   	     				  continue;
   	     			  }
   	     			  if(si.start - 1 <= m_lastGScore){
   	     				  getJPSSuccessors(s_temp, Action::Right, 0, 0);
   	     			  }else{
   	     				  getJPSSuccessors(s_temp, Action::Right, si.start - 1 -m_lastGScore, 0);
   	     				  wait_time_r = si.start - 1 -m_lastGScore;
   	     			  }
   	    		  }
   	    	  }
   	      }
   	      if(s_temp.dir & 0x4){
   	    	  s_temp.action = Action::Up;
   	    	  state_re.x = s.state.x;
   	    	  state_re.y = s.state.y + 1;
//   	    	  std::cout << "Up : " << state_re.x << " " << state_re.y  << "end :	" << " direction \n";
   	    	  if(m_env.stateValid(state_re)){
   	    		  const auto& sis = safeIntervals(m_env.getLocation(state_re));
   	    		  for (size_t i = 0; i < sis.size(); ++i) {

   	     			  const interval& si = sis[i];

   	     			  if (si.start - 1 > end_t || si.end - 1 < start_t) {
   	     				  continue;
   	     			  }
   	     			  if(si.start - 1 <= m_lastGScore){
   	     				  getJPSSuccessors(s_temp, Action::Up, 0, 0);
   	     			  }else{
   	     				  getJPSSuccessors(s_temp, Action::Up, si.start - 1 -m_lastGScore, 0);
   	     				  wait_time_u = si.start - 1 -m_lastGScore;
   	     			  }
   	    		  }
   	    	  }
   	      }

   	      if(s_temp.dir & 0x8){
   	    	  s_temp.action = Action::Down;
   	    	  state_re.x = s.state.x;
   	    	  state_re.y = s.state.y - 1;
 //  	    	  std::cout << "Down : \n";
   	    	  if(m_env.stateValid(state_re)){
   	    		  const auto& sis = safeIntervals(m_env.getLocation(state_re));
   	    		  for (size_t i = 0; i < sis.size(); ++i) {
   	     			  const interval& si = sis[i];
   	     			  if (si.start - 1 >  end_t || si.end - 1 < start_t) {
   	     				  continue;
   	     			  }
//     	     			std::cout << "Time----- " << si.start - 1 << " end_t " << end_t << " " << si.end - 1 << " Start_t " << start_t << " "<<std::endl;
   	     			  if(si.start -  1 <= m_lastGScore){
   	     				  getJPSSuccessors(s_temp, Action::Down, 0, 0);
   	     			  }else{
   	     				  getJPSSuccessors(s_temp, Action::Down, si.start - 1 -m_lastGScore, 0);
   	     				  wait_time_d = si.start - 1 -m_lastGScore;
   	     			  }
   	    		  }
   	    	  }
   	      }

//    	  m_env.num_generation--;
    	  for (const auto& m : jps_successors) {
    		  Cost m_time = m.cost;
    		  Cost start_t = m_lastGScore + m_time;
    		  Cost end_t =
    				  safeIntervals(m_env.getLocation(s.state)).at(s.interval).end;
//    		  std::cout << m.state.state.x << " " << m.state.state.y <<  " Start " << start_t << " End: " << end_t << " Cost "<< m_time << "\n";
    		  Cost wait_time = 0;
    		  if(m.action == Action::Left){
    			  wait_time = wait_time_l;
    		  }else if(m.action == Action::Right){
    			  wait_time = wait_time_r;
    		  }else if(m.action == Action::Up){
    			  wait_time = wait_time_u;
    		  }else if(m.action == Action::Down){
    			  wait_time = wait_time_d;
    		  }else if(m.action == Action::Wait){
    			  wait_time = 0;
    		  }

    		  const auto& sis = safeIntervals(m_env.getLocation(m.state.state));
    		  for (size_t i = 0; i < sis.size(); ++i) {
    			  const interval& si = sis[i];
    			  if (si.start - m_time + wait_time > end_t || si.end < start_t) {
    				  continue;
    			  }

//    			  std::cout << "Time again :: " << si.start << " " << si.end << " \n";
    			  int t;
    			  State temp_state = m.state.state;

    			  if (m_env.isCommandValid(s.state, m.state.state, m.action, m_lastGScore,
                                   	   end_t, si.start, si.end, t, m_time)){
        			  if(m.action == Action::Left){
            				  if(IsEdgeCollisions(m.state.state, edgeCollision(t - 1, Action::Right))) continue;
            		  } else if(m.action == Action::Right){
            				  if(IsEdgeCollisions(m.state.state, edgeCollision(t - 1, Action::Left))) { continue;}
            		  }else if(m.action == Action::Up){

//            				  std::cout << m.action << " " << m.state.state.x << "  " << m.state.state.y <<"\n";
            				  if(IsEdgeCollisions(m.state.state, edgeCollision(t - 1, Action::Down))) continue;
            		  }else if(m.action == Action::Down){

//            			     std::cout << m.action << " " << m.state.state.x << "  " << m.state.state.y << " t-1 " << t-1 <<" ------\n";
            				  if(IsEdgeCollisions(m.state.state, edgeCollision(t - 1, Action::Up))) continue;
            		  }
    				  neighbors.emplace_back(Neighbor<JPSSIPPState, JPSSIPPAction, Cost>(
    						  JPSSIPPState(m.state.state, i, m.state.dir), JPSSIPPAction(m.action, m.cost),
							  t - m_lastGScore));
//    				  std::cout << "Successor State---------- t "<< t  << " " << m.state.action << m.state.state.x << " " << m.state.state.y << " Cost " << m.cost << std::endl;
    			  }
    		  }
    	  }
      }
    }


    void getJPSSuccessors(JPSSIPPState s, Action action, Cost current_cost, int depth){

    	bool flag_solution = false;
    	if (m_env.isSolution(s.state)) {
    		flag_solution = true;
    		if(depth !=0 ){ jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(s, action, current_cost));}
    	}
    	if(isTempolObstacle(s.state)){
    		flag_solution = true;
    	}

        JPSSIPPState current_successor = s;
        m_env.num_generation++;

		State state_up(s.state.x, s.state.y + 1);
		State state_down(s.state.x, s.state.y -1);
		bool is_up_edge_collision =  IsEdgeCollisions(state_up,edgeCollision(m_lastGScore + current_cost,Action::Down));
		bool is_down_edge_collision =  IsEdgeCollisions(state_down,edgeCollision(m_lastGScore + current_cost,Action::Up));


 //       std::cout << "Current state : " << action << "  " << s.state.x << "  " << s.state.y << "Cost " << current_cost << "  ]]]]]]]]]]]]]]]\n";

    	if(action == Action::Left || (action == Action::Up && depth != 0) || (action == Action::Down && depth != 0) || action == Action::All){
   			current_successor.state.x = s.state.x - 1;
   			current_successor.state.y = s.state.y;
//   			std::cout<<"Current successor :" << current_successor.state.x << " " << current_successor.state.y << "Left" << std::endl;
   			if(m_env.stateValid(current_successor.state) &&
   					!IsEdgeCollisions(current_successor.state,edgeCollision(m_lastGScore + current_cost,Action::Right))){
   				bool is_up = false, is_down = false;
   				unsigned int dir = 0x00;
   				if(m_env.isJumpPoint(current_successor.state)){
   					if( ((m_env.isObstacle(State(s.state.x, s.state.y + 1))
   							|| isTemporalObstacleSafe2(State(s.state.x, s.state.y + 1), m_lastGScore + current_cost + 1)
							|| is_up_edge_collision) &&
   							m_env.stateValid(State(s.state.x - 1, s.state.y + 1)))
   							|| isTemporalObstacleSafe2(State(s.state.x - 1, s.state.y + 1), m_lastGScore + current_cost + 2)
							|| isTemporalObstacleSafe2(State(s.state.x -2, s.state.y), m_lastGScore + current_cost +2)){
   						is_up = true;
   					}
   					if(((m_env.isObstacle(State(s.state.x, s.state.y - 1))
   							|| isTemporalObstacleSafe2(State(s.state.x, s.state.y - 1), m_lastGScore + current_cost + 1)
							|| is_down_edge_collision)
   							&& m_env.stateValid(State(s.state.x - 1, s.state.y - 1)))
   							|| isTemporalObstacleSafe2(State(s.state.x - 1, s.state.y - 1), m_lastGScore + current_cost + 2)
							|| isTemporalObstacleSafe2(State(s.state.x -2, s.state.y), m_lastGScore + current_cost +2)){
   						is_down = true;
   					}
   				}

   				if(is_up && !is_down){
   					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)){
   					  						current_successor.dir = 0x7;
   					  					} else
   					current_successor.dir = 0x5;
   					current_successor.action = Action::Left;
   					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));

   				}

   				if(!is_up && is_down){
   					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)){
   					  						current_successor.dir = 0xb;
   					  					} else
   					current_successor.dir = 0x9;
   					current_successor.action = Action::Left;
   					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));

   				}

   				if(is_up && is_down){
   					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)){
   					  						current_successor.dir = 0xf;
   					  					} else
   					current_successor.dir = 0x0D;
   					current_successor.action = Action::Left;
   					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));

   				}
   				if(!(is_up || is_down)){
 					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)){
 						current_successor.dir = 0x03;
 						current_successor.action = Action::Left;
  	          			jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));

  					}
  					else {
  						if(isTemporalObstacleSafe2(current_successor.state, current_cost + 1)) {
  							current_successor.dir = 0x1;
  							current_successor.action = Action::Left;
  		            		jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));

  						}else getJPSSuccessors(current_successor, Action::Left, current_cost + 1, depth + 1);
  					}

   				}
   			}
    	}

    	if(action == Action::Right || (action == Action::Up && depth != 0) || (action == Action::Down && depth != 0) || action == Action::All){
   			current_successor.state.x = s.state.x + 1;
   			current_successor.state.y = s.state.y;
//   			m_env.num_generation++;
//   			std::cout<<"Current state :" << s.state.x << " " << s.state.y << "Right" << std::endl;
//   			std::cout<<"Current successor :" << current_successor.state.x << " " << current_successor.state.y << "Right" << std::endl;
   			if(m_env.stateValid(current_successor.state)
   					&& !IsEdgeCollisions(current_successor.state,edgeCollision(m_lastGScore + current_cost,Action::Left))){
   				bool is_up = false, is_down = false;
   				unsigned int dir = 0x00;
   				if(m_env.isJumpPoint(current_successor.state)){
  					if(((m_env.isObstacle(State(s.state.x, s.state.y + 1))
  							|| isTemporalObstacleSafe2(State(s.state.x, s.state.y + 1), m_lastGScore + current_cost +1)
							|| is_up_edge_collision)
  							&& m_env.stateValid(State(s.state.x + 1, s.state.y + 1)))
							|| (isTemporalObstacleSafe2(State(s.state.x + 1, s.state.y + 1), m_lastGScore + current_cost + 1))){
   						is_up = true;
   					}
  					if(((m_env.isObstacle(State(s.state.x, s.state.y - 1))
   							|| isTemporalObstacleSafe2(State(s.state.x, s.state.y - 1), m_lastGScore + current_cost + 1)
							|| is_down_edge_collision)
   							&& m_env.stateValid(State(s.state.x + 1, s.state.y - 1)))
							||(isTemporalObstacleSafe2(State(s.state.x + 1, s.state.y -1), m_lastGScore + current_cost + 1))){
   						is_down = true;
//   						std::cout << "Re-start Down\n";
   					}
   				}

   				if(is_up && !is_down){
  					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)){
  						current_successor.dir = 0x7;
  					} else current_successor.dir = 0x6;
   					current_successor.action = Action::Right;
//   					std::cout << "Direction " << current_successor.dir << " Herehere\n";
   					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));

   				}

   				if(!is_up&&is_down){
   					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)){
   					  						current_successor.dir = 0xb;
   					} else current_successor.dir = 0x0a;
   					current_successor.action = Action::Right;
//   					std::cout << "Direction " << current_successor.dir << " Herehere\n";
   					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));

   				}

   				if(is_up&&is_down){
   					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)){
   					  						current_successor.dir = 0xf;
   					 } else current_successor.dir = 0x0e;
   					current_successor.action = Action::Right;
//   					std::cout << "Direction " << current_successor.dir << " Herehere\n";
   					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));

   				}
   				if(!(is_up || is_down)){
  					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)) {
  						current_successor.dir = 0x03;
  						current_successor.action = Action::Right;
  						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));

  					} else{
  						if(isTemporalObstacleSafe2(current_successor.state, current_cost + 1)){
  							current_successor.dir = 0x02;
  							current_successor.action = Action::Right;
  							jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));

  						} else getJPSSuccessors(current_successor, Action::Right, current_cost + 1, depth + 1);
  					}
   				}
   			}
    	}

    	if(action == Action::Up || action == Action::All){

//    		std::cout << s.state.x << " " << s.state.y<< " UP UP ---------------\n";
    		current_successor.state.x = s.state.x;
    		current_successor.state.y = s.state.y + 1;
    			if(m_env.stateValid(current_successor.state)
   					&& !IsEdgeCollisions(current_successor.state,edgeCollision(m_lastGScore + current_cost,Action::Down))){
				if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)) {
					current_successor.dir = 0xf;
					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost + 1));
				}
				else {

					if( isTemporalObstacleSafe2(State(s.state.x, s.state.y + 2), m_lastGScore + current_cost + 1)
							|| isTemporalObstacleSafe2(State(s.state.x - 1, s.state.y + 1), m_lastGScore + current_cost + 1)
							|| isTemporalObstacleSafe2(State(s.state.x + 1, s.state.y + 1), m_lastGScore + current_cost + 1)){
						current_successor.dir = 0x07;
						current_successor.action = Action::Up;
						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost + 1));
					} else if(isTemporalObstacleSafe2(current_successor.state, m_lastGScore + current_cost + 1)){
						current_successor.dir = 0x07;
						current_successor.action = Action::Up;
						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost + 1));
					}else { getJPSSuccessors(current_successor, Action::Up, current_cost + 1, depth + 1);}

				}
   			}

    	}
    	if(action == Action::Down || action == Action::All){
    		current_successor.state.x = s.state.x;
    		current_successor.state.y = s.state.y - 1;
 	 	 	if(m_env.stateValid(current_successor.state)
   					&& !IsEdgeCollisions(current_successor.state,edgeCollision(m_lastGScore + current_cost,Action::Up))){

   				if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)) {
   					current_successor.dir = 0xf;
   					current_successor.action = Action::Down;
   					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Down, current_cost + 1));
   				} else {

   					if(isTemporalObstacleSafe2(State(s.state.x, s.state.y -2), m_lastGScore + current_cost + 1)
   							|| isTemporalObstacle(State(s.state.x - 1, s.state.y -1), m_lastGScore + current_cost + 1)
							|| isTemporalObstacle(State(s.state.x + 1, s.state.y - 1), m_lastGScore + current_cost + 1)){
   						current_successor.dir = 0x0b;
   						current_successor.action = Action::Down;
   						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Down, current_cost + 1));
   					} else if(isTemporalObstacleSafe2(current_successor.state, m_lastGScore + current_cost + 1)){
   						current_successor.dir = 0x0b;
   						current_successor.action = Action::Down;
   						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Down, current_cost + 1));
   					}else getJPSSuccessors(current_successor, Action::Down, current_cost + 1, depth + 1);
   				}
    		}
    	}
    }
*/


/*    void getNeighbors(
        const JPSSIPPState& s,
        std::vector<Neighbor<JPSSIPPState, JPSSIPPAction, Cost> >& neighbors) {


      std::vector<Neighbor<State, Action, Cost> > motions;

      JPSSIPPState s_temp = s;

      if(!m_env.isJPS()){
    	  m_env.getNeighbors(s_temp.state, motions);
 //   	  std::cout << "The number of neighbors: " << motions.size() << " " << std::endl;
 //   	  std::cout << "Current state ---------------------------------------" << s.state.x << " " << s.state.y << "\n";
          for (const auto& m : motions) {
        	  m_env.num_generation++;
//        	  std::cout << "Current successor: " << m.state.x << " " << m.state.y << "\n";
            Cost m_time = m.cost;
            Cost start_t = m_lastGScore + m_time;
            Cost end_t =
                safeIntervals(m_env.getLocation(s.state)).at(s.interval).end;

            const auto& sis = safeIntervals(m_env.getLocation(m.state));
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
              if (m_env.isCommandValid(s.state, m.state, m.action, m_lastGScore,
                                       end_t, si.start, si.end, t, m_time) && !IsEdgeCollisions(m.state, edgeCollision(t - 1, a_temp))) {
                neighbors.emplace_back(Neighbor<JPSSIPPState, JPSSIPPAction, Cost>(
                    JPSSIPPState(m.state, i,0xf), JPSSIPPAction(m.action, m.cost),
                    t - m_lastGScore));
              }
            }
          }
      }else {

    	  Cost start_t = m_lastGScore;
		  Cost end_t =
				  safeIntervals(m_env.getLocation(s.state)).at(s.interval).end;

		  State state_re = s.state;

		  jps_successors.clear();
		  Cost wait_time_l = 0, wait_time_r = 0, wait_time_u = 0, wait_time_d = 0;

		  std::cout << "Current expanding node : " << s.state.x << "  " << s.state.y << " Start : " << start_t << " End: " << end_t << "-----------------\n";

   	      if(s_temp.dir & 0x1){
   	    	  getJPSSuccessors(s_temp, Action::Left, 0, 0);
   	      }
   	      if(s_temp.dir & 0x2){
   	    	  getJPSSuccessors(s_temp, Action::Right, 0, 0);
   	      }
   	      if(s_temp.dir & 0x4){
   	    	  getJPSSuccessors(s_temp, Action::Left, 0, 0);
   	      }

   	      if(s_temp.dir & 0x8){
   	    	  getJPSSuccessors(s_temp, Action::Left, 0, 0);
   	      }

//    	  m_env.num_generation--;
    	  for (const auto& m : jps_successors) {
    		  Cost m_time = m.cost;
    		  Cost start_t = m_lastGScore + m_time;
    		  Cost end_t =
    				  safeIntervals(m_env.getLocation(s.state)).at(s.interval).end;
    		  Cost wait_time = 0;
    		  if(m.action == Action::Left){
    			  wait_time = wait_time_l;
    		  }else if(m.action == Action::Right){
    			  wait_time = wait_time_r;
    		  }else if(m.action == Action::Up){
    			  wait_time = wait_time_u;
    		  }else if(m.action == Action::Down){
    			  wait_time = wait_time_d;
    		  }else if(m.action == Action::Wait){
    			  wait_time = 0;
    		  }

    		  const auto& sis = safeIntervals(m_env.getLocation(m.state.state));
    		  for (size_t i = 0; i < sis.size(); ++i) {
    			  const interval& si = sis[i];
    			  if (si.start - m_time + wait_time > end_t || si.end < start_t) {
    				  continue;
    			  }

    			  int t;
    			  State temp_state = m.state.state;

    			  if (m_env.isCommandValid(s.state, m.state.state, m.action, m_lastGScore,
                                   	   end_t, si.start, si.end, t, m_time)){
        			  if(m.action == Action::Left){
            				  if(IsEdgeCollisions(m.state.state, edgeCollision(t - 1, Action::Right))) continue;
            		  } else if(m.action == Action::Right){
            				  if(IsEdgeCollisions(m.state.state, edgeCollision(t - 1, Action::Left))) { continue;}
            		  }else if(m.action == Action::Up){
            				  if(IsEdgeCollisions(m.state.state, edgeCollision(t - 1, Action::Down))) continue;
            		  }else if(m.action == Action::Down){

            				  if(IsEdgeCollisions(m.state.state, edgeCollision(t - 1, Action::Up))) continue;
            		  }
    				  neighbors.emplace_back(Neighbor<JPSSIPPState, JPSSIPPAction, Cost>(
    						  JPSSIPPState(m.state.state, i, m.state.dir, true), JPSSIPPAction(m.action, m.cost),
							  t - m_lastGScore));
    				  std::cout << "Successor : " << m.state.state.x << " " << m.state.state.y << std::endl;
//    				  break;
    			  }
    		  }
    	  }
      }
    }


    void getJPSSuccessors(JPSSIPPState s, Action action, Cost current_cost, int depth){

    	bool flag_solution = false;
    	if (m_env.isSolution(s.state)) {
    		jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(s, action, current_cost));
    	}
    	if(isTempolObstacle(s.state)){
    		flag_solution = true;
    	}

        JPSSIPPState current_successor = s;
        m_env.num_generation++;

		State state_up(s.state.x, s.state.y + 1);
		State state_down(s.state.x, s.state.y -1);
		bool is_up_edge_collision =  IsEdgeCollisions(state_up,edgeCollision(m_lastGScore + current_cost,Action::Down));
		bool is_down_edge_collision =  IsEdgeCollisions(state_down,edgeCollision(m_lastGScore + current_cost,Action::Up));

    	if(action == Action::Left || (action == Action::Up && depth != 0) || (action == Action::Down && depth != 0) || action == Action::All){
   			current_successor.state.x = s.state.x - 1;
   			current_successor.state.y = s.state.y;
  			if(m_env.stateValid(current_successor.state) &&
   					!IsEdgeCollisions(current_successor.state,edgeCollision(m_lastGScore + current_cost,Action::Right))){

  					bool is_up = false, is_down = false;
  					if( ((m_env.isObstacle(State(s.state.x, s.state.y + 1))
							|| isTemporalObstacleSafe2(State(s.state.x, s.state.y + 1), m_lastGScore + current_cost + 1)
						|| is_up_edge_collision) &&
							m_env.stateValid(State(s.state.x - 1, s.state.y + 1)))
							|| isTemporalObstacleSafe2(State(s.state.x - 1, s.state.y + 1), m_lastGScore + current_cost + 2)
						|| isTemporalObstacleSafe2(State(s.state.x -2, s.state.y), m_lastGScore + current_cost +2)){
						is_up = true;
					}
					if(((m_env.isObstacle(State(s.state.x, s.state.y - 1))
							|| isTemporalObstacleSafe2(State(s.state.x, s.state.y - 1), m_lastGScore + current_cost + 1)
						|| is_down_edge_collision)
							&& m_env.stateValid(State(s.state.x - 1, s.state.y - 1)))
							|| isTemporalObstacleSafe2(State(s.state.x - 1, s.state.y - 1), m_lastGScore + current_cost + 2)
						|| isTemporalObstacleSafe2(State(s.state.x -2, s.state.y), m_lastGScore + current_cost +2)){
						is_down = true;
					}

	  				if(is_up && !is_down){
	   					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)){
	   						current_successor.dir = 0x7;
	   					} else current_successor.dir = 0x5;
	   					current_successor.action = Action::Left;
	   					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));
	   				}
	   				if(!is_up && is_down){
	   					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)){
	   						current_successor.dir = 0xb;
	   					 } else  current_successor.dir = 0x9;
	   					current_successor.action = Action::Left;
	   					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));
	   				}

	   				if(is_up && is_down){
	   					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)){
	   						current_successor.dir = 0xf;
	   					 } else current_successor.dir = 0x0D;
	   					current_successor.action = Action::Left;
	   					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));
	   				}

	   				if(!(is_up || is_down)){
	   					getJPSSuccessors(current_successor, Action::Left, current_cost + 1, depth + 1);
	   				}
  			}
    	}

    	if(action == Action::Right || (action == Action::Up && depth != 0) || (action == Action::Down && depth != 0) || action == Action::All){
   			current_successor.state.x = s.state.x + 1;
   			current_successor.state.y = s.state.y;
   			if(m_env.stateValid(current_successor.state)
   					&& !IsEdgeCollisions(current_successor.state,edgeCollision(m_lastGScore + current_cost,Action::Left))){

   					bool is_up = false, is_down = false;
   					if(((m_env.isObstacle(State(s.state.x, s.state.y + 1))
							|| isTemporalObstacleSafe2(State(s.state.x, s.state.y + 1), m_lastGScore + current_cost +1)
						|| is_up_edge_collision)
							&& m_env.stateValid(State(s.state.x + 1, s.state.y + 1)))
						|| (isTemporalObstacleSafe2(State(s.state.x + 1, s.state.y + 1), m_lastGScore + current_cost + 1))){
						is_up = true;
					}
					if(((m_env.isObstacle(State(s.state.x, s.state.y - 1))
							|| isTemporalObstacleSafe2(State(s.state.x, s.state.y - 1), m_lastGScore + current_cost + 1)
						|| is_down_edge_collision)
							&& m_env.stateValid(State(s.state.x + 1, s.state.y - 1)))
						||(isTemporalObstacleSafe2(State(s.state.x + 1, s.state.y -1), m_lastGScore + current_cost + 1))){
						is_down = true;
					}

	   				if(is_up && !is_down){
	  					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)){
	  						current_successor.dir = 0x7;
	  					} else current_successor.dir = 0x6;
	   					current_successor.action = Action::Right;
	   					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));
	   				}

	   				if(!is_up&&is_down){
	   					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)){
	   						current_successor.dir = 0xb;
	   					} else current_successor.dir = 0x0a;
	   					current_successor.action = Action::Right;
	   					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));
	   				}

	   				if(is_up && is_down){
	   					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)){
	   						current_successor.dir = 0xf;
	   					 } else current_successor.dir = 0x0e;
	   					current_successor.action = Action::Right;
	   					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));
	   				}
	   				if(!(is_up || is_down)){
	   					getJPSSuccessors(current_successor, Action::Left, current_cost + 1, depth + 1);
	   				}
   			}
    	}

       	if(action == Action::Up || action == Action::All){
    		current_successor.state.x = s.state.x;
    		current_successor.state.y = s.state.y + 1;
			if(m_env.stateValid(current_successor.state)
					&& !IsEdgeCollisions(current_successor.state,edgeCollision(m_lastGScore + current_cost,Action::Down))){
				if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)) {
					current_successor.dir = 0xf;
					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost + 1));
				} else {
					if( isTemporalObstacleSafe2(State(s.state.x, s.state.y + 2), m_lastGScore + current_cost + 1)
							|| isTemporalObstacleSafe2(State(s.state.x - 1, s.state.y + 1), m_lastGScore + current_cost + 1)
							|| isTemporalObstacleSafe2(State(s.state.x + 1, s.state.y + 1), m_lastGScore + current_cost + 1)){
						current_successor.dir = 0x07;
						current_successor.action = Action::Up;
						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost + 1));
					} else if(isTemporalObstacleSafe2(current_successor.state, m_lastGScore + current_cost + 1)){
						current_successor.dir = 0x07;
						current_successor.action = Action::Up;
						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost + 1));
					}else { getJPSSuccessors(current_successor, Action::Up, current_cost + 1, depth + 1);}
				}
			}
       	}

    	if(action == Action::Down || action == Action::All){
    		current_successor.state.x = s.state.x;
    		current_successor.state.y = s.state.y - 1;
    		if(m_env.stateValid(current_successor.state)
   					&& !IsEdgeCollisions(current_successor.state,edgeCollision(m_lastGScore + current_cost,Action::Up))){

   				if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)) {
   					current_successor.dir = 0xf;
   					current_successor.action = Action::Down;
   					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Down, current_cost + 1));
   				} else {

   					if(isTemporalObstacleSafe2(State(s.state.x, s.state.y -2), m_lastGScore + current_cost + 1)
   							|| isTemporalObstacle(State(s.state.x - 1, s.state.y -1), m_lastGScore + current_cost + 1)
							|| isTemporalObstacle(State(s.state.x + 1, s.state.y - 1), m_lastGScore + current_cost + 1)){
   						current_successor.dir = 0x0b;
   						current_successor.action = Action::Down;
   						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Down, current_cost + 1));
   					} else if(isTemporalObstacleSafe2(current_successor.state, m_lastGScore + current_cost + 1)){
   						current_successor.dir = 0x0b;
   						current_successor.action = Action::Down;
   						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Down, current_cost + 1));
   					}else getJPSSuccessors(current_successor, Action::Down, current_cost + 1, depth + 1);
   				}
    		}
    	}
    }*/


        void getNeighbors_Bei(
            const JPSSIPPState& s,
            std::vector<Neighbor<JPSSIPPState, JPSSIPPAction, Cost> >& neighbors) {


          std::vector<Neighbor<State, Action, Cost> > motions;

          JPSSIPPState s_temp = s;

          if(!m_env.isJPS()){
        	  m_env.getNeighbors(s_temp.state, motions);
     //   	  std::cout << "The number of neighbors: " << motions.size() << " " << std::endl;
        	  std::cout << "Current state ---------------------------------------" << s.state.x << " " << s.state.y << "\n";
              for (const auto& m : motions) {
            	  m_env.num_generation++;
    //        	  std::cout << "Current successor: " << m.state.x << " " << m.state.y << "\n";
                Cost m_time = m.cost;
                Cost start_t = m_lastGScore + m_time;
                Cost end_t =
                    safeIntervals(m_env.getLocation(s.state)).at(s.interval).end;

                const auto& sis = safeIntervals(m_env.getLocation(m.state));
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
                  if (m_env.isCommandValid(s.state, m.state, m.action, m_lastGScore,
                                           end_t, si.start, si.end, t, m_time) && !IsEdgeCollisions(m.state, edgeCollision(t - 1, a_temp))) {
                    neighbors.emplace_back(Neighbor<JPSSIPPState, JPSSIPPAction, Cost>(
                        JPSSIPPState(m.state, i,0xf), JPSSIPPAction(m.action, m.cost),
                        t - m_lastGScore));
                  }
                }
              }
          }else {

        	  Cost start_t = m_lastGScore;
    		  Cost end_t =
    				  safeIntervals(m_env.getLocation(s.state)).at(s.interval).end;

    		  State state_re = s.state;

    		  std::cout << "Current State " << s.state.x << "  " << s.state.y <<" Gscore" << m_lastGScore << " interval " << s.interval << " Direction " << s.dir << "  ********************************\n";
    		  jps_successors.clear();
    		  Cost wait_time_l = 0, wait_time_r = 0, wait_time_u = 0, wait_time_d = 0;

       	      if(s_temp.dir & 0x1){
     //  	    	  std::cout << "Left : \n";
       	    	  s_temp.action = Action::Left;
       	    	  state_re.x = s.state.x -1;
       	    	  state_re.y = s.state.y;
       	    	  if(m_env.stateValid(state_re)){
       	    		  const auto& sis = safeIntervals(m_env.getLocation(state_re));
       	    		  for (size_t i = 0; i < sis.size(); ++i) {
       	     			  const interval& si = sis[i];
       	     			  if (si.start - 1 > end_t || si.end - 1 < start_t) {
       	     				  continue;
       	     			  }
       	     			  if(si.start -1 <= m_lastGScore){
       	     				  getJPSSuccessors(s_temp, Action::Left, 0, 0);
       	     			  }else{
       	     				  getJPSSuccessors(s_temp, Action::Left, si.start - 1 -m_lastGScore, 0);
       	     				  wait_time_l = si.start - 1 -m_lastGScore;
       	     			  }
       	    		  }
       	    	  }
       	      }
       	      if(s_temp.dir & 0x2){
       	    	  s_temp.action = Action::Right;
       	    	  state_re.x = s.state.x + 1;
       	    	  state_re.y = s.state.y;
     //  	    	 std::cout << "Right Right : " << state_re.x << " " << state_re.y << " direction \n";
       	    	  if(m_env.stateValid(state_re)){
       	    		  const auto& sis = safeIntervals(m_env.getLocation(state_re));
       	    		  const auto& tt = safeIntervals(m_env.getLocation(State(2,7)));
     //  	    		  std::cout << "The size of sis " << sis.size() << " The size of (2,7)" << tt.size() <<std::endl;
       	    		  for (size_t i = 0; i < sis.size(); ++i) {
       	     			  const interval& si = sis[i];
       	     			  if (si.start - 1 > end_t  || si.end - 1 < start_t ) {
       	     				  continue;
       	     			  }
       	     			  if(si.start - 1 <= m_lastGScore){
       	     				  getJPSSuccessors(s_temp, Action::Right, 0, 0);
       	     			  }else{
       	     				  getJPSSuccessors(s_temp, Action::Right, si.start - 1 -m_lastGScore, 0);
       	     				  wait_time_r = si.start - 1 -m_lastGScore;
       	     			  }
       	    		  }
       	    	  }
       	      }
       	      if(s_temp.dir & 0x4){
       	    	  s_temp.action = Action::Up;
       	    	  state_re.x = s.state.x;
       	    	  state_re.y = s.state.y + 1;
    //   	    	  std::cout << "Up : " << state_re.x << " " << state_re.y  << "end :	" << " direction \n";
       	    	  if(m_env.stateValid(state_re)){
       	    		  const auto& sis = safeIntervals(m_env.getLocation(state_re));
       	    		  for (size_t i = 0; i < sis.size(); ++i) {

       	     			  const interval& si = sis[i];

       	     			  if (si.start - 1 > end_t || si.end - 1 < start_t) {
       	     				  continue;
       	     			  }
       	     			  if(si.start - 1 <= m_lastGScore){
       	     				  getJPSSuccessors(s_temp, Action::Up, 0, 0);
       	     			  }else{
       	     				  getJPSSuccessors(s_temp, Action::Up, si.start - 1 -m_lastGScore, 0);
       	     				  wait_time_u = si.start - 1 -m_lastGScore;
       	     			  }
       	    		  }
       	    	  }
       	      }

       	      if(s_temp.dir & 0x8){
       	    	  s_temp.action = Action::Down;
       	    	  state_re.x = s.state.x;
       	    	  state_re.y = s.state.y - 1;
     //  	    	  std::cout << "Down : \n";
       	    	  if(m_env.stateValid(state_re)){
       	    		  const auto& sis = safeIntervals(m_env.getLocation(state_re));
       	    		  for (size_t i = 0; i < sis.size(); ++i) {
       	     			  const interval& si = sis[i];
       	     			  if (si.start - 1 >  end_t || si.end - 1 < start_t) {
       	     				  continue;
       	     			  }
    //     	     			std::cout << "Time----- " << si.start - 1 << " end_t " << end_t << " " << si.end - 1 << " Start_t " << start_t << " "<<std::endl;
       	     			  if(si.start -  1 <= m_lastGScore){
       	     				  getJPSSuccessors(s_temp, Action::Down, 0, 0);
       	     			  }else{
       	     				  getJPSSuccessors(s_temp, Action::Down, si.start - 1 -m_lastGScore, 0);
       	     				  wait_time_d = si.start - 1 -m_lastGScore;
       	     			  }
       	    		  }
       	    	  }
       	      }

    //    	  m_env.num_generation--;
        	  for (const auto& m : jps_successors) {
        		  Cost m_time = m.cost;
        		  Cost start_t = m_lastGScore + m_time;
        		  Cost end_t =
        				  safeIntervals(m_env.getLocation(s.state)).at(s.interval).end;
    //    		  std::cout << m.state.state.x << " " << m.state.state.y <<  " Start " << start_t << " End: " << end_t << " Cost "<< m_time << "\n";
        		  Cost wait_time = 0;
        		  if(m.action == Action::Left){
        			  wait_time = wait_time_l;
        		  }else if(m.action == Action::Right){
        			  wait_time = wait_time_r;
        		  }else if(m.action == Action::Up){
        			  wait_time = wait_time_u;
        		  }else if(m.action == Action::Down){
        			  wait_time = wait_time_d;
        		  }else if(m.action == Action::Wait){
        			  wait_time = 0;
        		  }

        		  const auto& sis = safeIntervals(m_env.getLocation(m.state.state));
        		  for (size_t i = 0; i < sis.size(); ++i) {
        			  const interval& si = sis[i];
        			  if (si.start - m_time + wait_time > end_t || si.end < start_t) {
        				  continue;
        			  }
    //    			  std::cout << "Time again :: " << si.start << " " << si.end << " \n";
        			  int t;
        			  State temp_state = m.state.state;

        			  if (m_env.isCommandValid(s.state, m.state.state, m.action, m_lastGScore,
                                       	   end_t, si.start, si.end, t, m_time)){
            			  if(m.action == Action::Left){
                				  if(IsEdgeCollisions(m.state.state, edgeCollision(t - 1, Action::Right))) continue;
                		  } else if(m.action == Action::Right){
                				  if(IsEdgeCollisions(m.state.state, edgeCollision(t - 1, Action::Left))) { continue;}
                		  }else if(m.action == Action::Up){
                				  if(IsEdgeCollisions(m.state.state, edgeCollision(t - 1, Action::Down))) continue;
                		  }else if(m.action == Action::Down){
                				  if(IsEdgeCollisions(m.state.state, edgeCollision(t - 1, Action::Up))) continue;
                		  }
        				  neighbors.emplace_back(Neighbor<JPSSIPPState, JPSSIPPAction, Cost>(
        						  JPSSIPPState(m.state.state, i, m.state.dir), JPSSIPPAction(m.action, m.cost),
    							  t - m_lastGScore));
        				  std::cout << "Successor State---------- t "<< t  << " " << m.state.action << m.state.state.x << " " << m.state.state.y << " Cost " << m.cost << std::endl;
        			  }
        		  }
        	  }
          }
        }

        void getJPSSuccessors_Bei(JPSSIPPState s, Action action, Cost current_cost, int depth){

        	bool flag_solution = false;
        	if (m_env.isSolution(s.state)) {
        		flag_solution = true;
        		if(depth !=0 ){ jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(s, action, current_cost));}
        	}
        	if(isTempolObstacle(s.state)){
        		flag_solution = true;
        	}

            JPSSIPPState current_successor = s;
            m_env.num_generation++;

    		State state_up(s.state.x, s.state.y + 1);
    		State state_down(s.state.x, s.state.y -1);
    		bool is_up_edge_collision =  IsEdgeCollisions(state_up,edgeCollision(m_lastGScore + current_cost,Action::Down));
    		bool is_down_edge_collision =  IsEdgeCollisions(state_down,edgeCollision(m_lastGScore + current_cost,Action::Up));


     //       std::cout << "Current state : " << action << "  " << s.state.x << "  " << s.state.y << "Cost " << current_cost << "  ]]]]]]]]]]]]]]]\n";

        	if(action == Action::Left || (action == Action::Up && depth != 0) || (action == Action::Down && depth != 0) || action == Action::All){
       			current_successor.state.x = s.state.x - 1;
       			current_successor.state.y = s.state.y;
    //   			std::cout<<"Current successor :" << current_successor.state.x << " " << current_successor.state.y << "Left" << std::endl;
       			if(m_env.stateValid(current_successor.state) &&
       					!IsEdgeCollisions(current_successor.state,edgeCollision(m_lastGScore + current_cost,Action::Right))){
       				bool is_up = false, is_down = false;
       				unsigned int dir = 0x00;
       				if(m_env.isJumpPoint(current_successor.state)){
       					if( ((m_env.isObstacle(State(s.state.x, s.state.y + 1))
       							|| isTemporalObstacleSafe2(State(s.state.x, s.state.y + 1), m_lastGScore + current_cost + 1)
    							|| is_up_edge_collision) &&
       							m_env.stateValid(State(s.state.x - 1, s.state.y + 1)))
       							|| isTemporalObstacleSafe2(State(s.state.x - 1, s.state.y + 1), m_lastGScore + current_cost + 2)
    							|| isTemporalObstacleSafe2(State(s.state.x -2, s.state.y), m_lastGScore + current_cost +2)){
       						is_up = true;
       					}
       					if(((m_env.isObstacle(State(s.state.x, s.state.y - 1))
       							|| isTemporalObstacleSafe2(State(s.state.x, s.state.y - 1), m_lastGScore + current_cost + 1)
    							|| is_down_edge_collision)
       							&& m_env.stateValid(State(s.state.x - 1, s.state.y - 1)))
       							|| isTemporalObstacleSafe2(State(s.state.x - 1, s.state.y - 1), m_lastGScore + current_cost + 2)
    							|| isTemporalObstacleSafe2(State(s.state.x -2, s.state.y), m_lastGScore + current_cost +2)){
       						is_down = true;
       					}
       				}

       				if(is_up && !is_down){
       					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)){
       					  						current_successor.dir = 0x7;
       					} else
       					current_successor.dir = 0x5;
       					current_successor.action = Action::Left;
       					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));
       				}

       				if(!is_up && is_down){
       					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)){
       						current_successor.dir = 0xb;
       					} else current_successor.dir = 0x9;
       					current_successor.action = Action::Left;
       					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));

       				}

       				if(is_up && is_down){
       					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)){
       					  						current_successor.dir = 0xf;
       					  					} else
       					current_successor.dir = 0x0D;
       					current_successor.action = Action::Left;
       					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));

       				}
       				if(!(is_up || is_down)){
     					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)){
     						current_successor.dir = 0x03;
     						current_successor.action = Action::Left;
      	          			jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));

      					}
      					else {
      						if(isTemporalObstacleSafe2(State(s.state.x - 2, s.state.y), m_lastGScore + current_cost + 1)
      								|| (m_env.is_limit && current_cost > m_env.limit_jump)) {
      							current_successor.dir = 0x1;
      							current_successor.action = Action::Left;
      		            		jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));
      						}else getJPSSuccessors(current_successor, Action::Left, current_cost + 1, depth + 1);
      					}

       				}
       			}
        	}

        	if(action == Action::Right || (action == Action::Up && depth != 0) || (action == Action::Down && depth != 0) || action == Action::All){
       			current_successor.state.x = s.state.x + 1;
       			current_successor.state.y = s.state.y;
    //   			m_env.num_generation++;
    //   			std::cout<<"Current state :" << s.state.x << " " << s.state.y << "Right" << std::endl;
    //   			std::cout<<"Current successor :" << current_successor.state.x << " " << current_successor.state.y << "Right" << std::endl;
       			if(m_env.stateValid(current_successor.state)
       					&& !IsEdgeCollisions(current_successor.state,edgeCollision(m_lastGScore + current_cost,Action::Left))){
       				bool is_up = false, is_down = false;
       				unsigned int dir = 0x00;
       				if(m_env.isJumpPoint(current_successor.state)){
      					if(((m_env.isObstacle(State(s.state.x, s.state.y + 1))
      							|| isTemporalObstacleSafe2(State(s.state.x, s.state.y + 1), m_lastGScore + current_cost +1)
    							|| is_up_edge_collision)
      							&& m_env.stateValid(State(s.state.x + 1, s.state.y + 1)))
    							|| (isTemporalObstacleSafe2(State(s.state.x + 1, s.state.y + 1), m_lastGScore + current_cost + 1))){
       						is_up = true;
       					}
      					if(((m_env.isObstacle(State(s.state.x, s.state.y - 1))
       							|| isTemporalObstacleSafe2(State(s.state.x, s.state.y - 1), m_lastGScore + current_cost + 1)
    							|| is_down_edge_collision)
       							&& m_env.stateValid(State(s.state.x + 1, s.state.y - 1)))
    							||(isTemporalObstacleSafe2(State(s.state.x + 1, s.state.y -1), m_lastGScore + current_cost + 1))){
       						is_down = true;
    //   						std::cout << "Re-start Down\n";
       					}
       				}

       				if(is_up && !is_down){
      					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)){
      						current_successor.dir = 0x7;
      					} else current_successor.dir = 0x6;
       					current_successor.action = Action::Right;
    //   					std::cout << "Direction " << current_successor.dir << " Herehere\n";
       					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));

       				}

       				if(!is_up&&is_down){
       					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)){
       						current_successor.dir = 0xb;
       					} else current_successor.dir = 0x0a;
       					current_successor.action = Action::Right;
    //   					std::cout << "Direction " << current_successor.dir << " Herehere\n";
       					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));

       				}

       				if(is_up&&is_down){
       					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)){
       						current_successor.dir = 0xf;
       					 } else current_successor.dir = 0x0e;
       					current_successor.action = Action::Right;
    //   					std::cout << "Direction " << current_successor.dir << " Herehere\n";
       					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));

       				}
       				if(!(is_up || is_down)){
      					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)) {
      						current_successor.dir = 0x03;
      						current_successor.action = Action::Right;
      						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));

      					} else{
      						if(isTemporalObstacleSafe2(State(s.state.x + 2, s.state.y), m_lastGScore + current_cost + 1)
      								|| (m_env.is_limit && current_cost > m_env.limit_jump)){
      							current_successor.dir = 0x02;
      							current_successor.action = Action::Right;
      							jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));
      						} else getJPSSuccessors(current_successor, Action::Right, current_cost + 1, depth + 1);
      					}
       				}
       			}
        	}

        	if(action == Action::Up || action == Action::All){

        		current_successor.state.x = s.state.x;
        		current_successor.state.y = s.state.y + 1;
//        		std::cout << "UP : " << current_successor.state.x << "  " << current_successor.state.y << "----------\n";
        			if(m_env.stateValid(current_successor.state)
       					&& !IsEdgeCollisions(current_successor.state,edgeCollision(m_lastGScore + current_cost,Action::Down))){
    				if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)) {
    					current_successor.dir = 0xf;
    					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost + 1));
    				}
    				else {
    					if( isTemporalObstacleSafe2(State(s.state.x, s.state.y + 2), m_lastGScore + current_cost + 1)
    							|| isTemporalObstacleSafe2(State(s.state.x - 1, s.state.y + 1), m_lastGScore + current_cost + 1)
    							|| isTemporalObstacleSafe2(State(s.state.x + 1, s.state.y + 1), m_lastGScore + current_cost + 1)
								|| (m_env.is_limit && current_cost > m_env.limit_jump)){
    						current_successor.dir = 0x07;
    						current_successor.action = Action::Up;
    						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost + 1));
    					} /*else if(isTemporalObstacleSafe2(current_successor.state, m_lastGScore + current_cost + 1)){
    						current_successor.dir = 0x07;
    						current_successor.action = Action::Up;
    						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost + 1));
    					}*/else { getJPSSuccessors(current_successor, Action::Up, current_cost + 1, depth + 1);}

    				}
       			}

        	}
        	if(action == Action::Down || action == Action::All){
        		current_successor.state.x = s.state.x;
        		current_successor.state.y = s.state.y - 1;
     	 	 	if(m_env.stateValid(current_successor.state)
       					&& !IsEdgeCollisions(current_successor.state,edgeCollision(m_lastGScore + current_cost,Action::Up))){

       				if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)) {
       					current_successor.dir = 0xf;
       					current_successor.action = Action::Down;
       					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Down, current_cost + 1));
       				} else {

       					if(isTemporalObstacleSafe2(State(s.state.x, s.state.y -2), m_lastGScore + current_cost + 1)
       							|| isTemporalObstacle(State(s.state.x - 1, s.state.y -1), m_lastGScore + current_cost + 1)
    							|| isTemporalObstacle(State(s.state.x + 1, s.state.y - 1), m_lastGScore + current_cost + 1)
								||(m_env.is_limit && current_cost > m_env.limit_jump)){
       						current_successor.dir = 0x0b;
       						current_successor.action = Action::Down;
       						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Down, current_cost + 1));
       					}/* else if(isTemporalObstacleSafe2(current_successor.state, m_lastGScore + current_cost + 1)){
       						current_successor.dir = 0x0b;
       						current_successor.action = Action::Down;
       						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Down, current_cost + 1));
       					}*/else getJPSSuccessors(current_successor, Action::Down, current_cost + 1, depth + 1);
       				}
        		}
        	}
        }


        void getNeighbors_bei2(
              const JPSSIPPState& s,
              std::vector<Neighbor<JPSSIPPState, JPSSIPPAction, Cost> >& neighbors) {


            std::vector<Neighbor<State, Action, Cost> > motions;

            JPSSIPPState s_temp = s;

            if(!m_env.isJPS()){
          	  m_env.getNeighbors(s_temp.state, motions);
       //   	  std::cout << "The number of neighbors: " << motions.size() << " " << std::endl;
//          	  std::cout << "Current state ---------------------------------------" << s.state.x << " " << s.state.y << "\n";
                for (const auto& m : motions) {
              	  m_env.num_generation++;
      //        	  std::cout << "Current successor: " << m.state.x << " " << m.state.y << "\n";
                  Cost m_time = m.cost;
                  Cost start_t = m_lastGScore + m_time;
                  Cost end_t =
                      safeIntervals(m_env.getLocation(s.state)).at(s.interval).end;

                  const auto& sis = safeIntervals(m_env.getLocation(m.state));
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
                    if (m_env.isCommandValid(s.state, m.state, m.action, m_lastGScore,
                                             end_t, si.start, si.end, t, m_time) && !IsEdgeCollisions(m.state, edgeCollision(t - 1, a_temp))) {
                      neighbors.emplace_back(Neighbor<JPSSIPPState, JPSSIPPAction, Cost>(
                          JPSSIPPState(m.state, i,0xf), JPSSIPPAction(m.action, m.cost),
                          t - m_lastGScore));
                    }
                  }
                }
            }else {

          	  Cost start_t = m_lastGScore;
      		  Cost end_t =
      				  safeIntervals(m_env.getLocation(s.state)).at(s.interval).end;

      		  State state_re = s.state;

  //    		  std::cout << "Current State " << s.state.x << "  " << s.state.y <<" Gscore" << m_lastGScore << " interval " << s.interval << " Direction " << s.dir << "  ********************************\n";
      		  jps_successors.clear();
      		  Cost wait_time_l = 0, wait_time_r = 0, wait_time_u = 0, wait_time_d = 0;

         	      if(s_temp.dir & 0x1){
       //  	    	  std::cout << "Left : \n";
         	    	  s_temp.action = Action::Left;
         	    	  state_re.x = s.state.x -1;
         	    	  state_re.y = s.state.y;
         	    	  if(m_env.stateValid(state_re)){
         	    		  const auto& sis = safeIntervals(m_env.getLocation(state_re));
         	    		  for (size_t i = 0; i < sis.size(); ++i) {
         	     			  const interval& si = sis[i];
         	     			  if (si.start - 1 > end_t || si.end - 1 < start_t) {
         	     				  continue;
         	     			  }
         	     			  if(si.start -1 <= m_lastGScore){
         	     				  getJPSSuccessors(s_temp, Action::Left, 0, 0);
         	     			  }else{
         	     				  getJPSSuccessors(s_temp, Action::Left, si.start - 1 -m_lastGScore, 0);
         	     				  wait_time_l = si.start - 1 -m_lastGScore;
         	     			  }
         	    		  }
         	    	  }
         	      }
         	      if(s_temp.dir & 0x2){
         	    	  s_temp.action = Action::Right;
         	    	  state_re.x = s.state.x + 1;
         	    	  state_re.y = s.state.y;
       //  	    	 std::cout << "Right Right : " << state_re.x << " " << state_re.y << " direction \n";
         	    	  if(m_env.stateValid(state_re)){
         	    		  const auto& sis = safeIntervals(m_env.getLocation(state_re));
         	    		  const auto& tt = safeIntervals(m_env.getLocation(State(2,7)));
       //  	    		  std::cout << "The size of sis " << sis.size() << " The size of (2,7)" << tt.size() <<std::endl;
         	    		  for (size_t i = 0; i < sis.size(); ++i) {
         	     			  const interval& si = sis[i];
         	     			  if (si.start - 1 > end_t  || si.end - 1 < start_t ) {
         	     				  continue;
         	     			  }
         	     			  if(si.start - 1 <= m_lastGScore){
         	     				  getJPSSuccessors(s_temp, Action::Right, 0, 0);
         	     			  }else{
         	     				  getJPSSuccessors(s_temp, Action::Right, si.start - 1 -m_lastGScore, 0);
         	     				  wait_time_r = si.start - 1 -m_lastGScore;
         	     			  }
         	    		  }
         	    	  }
         	      }
         	      if(s_temp.dir & 0x4){
         	    	  s_temp.action = Action::Up;
         	    	  state_re.x = s.state.x;
         	    	  state_re.y = s.state.y + 1;
      //   	    	  std::cout << "Up : " << state_re.x << " " << state_re.y  << "end :	" << " direction \n";
         	    	  if(m_env.stateValid(state_re)){
         	    		  const auto& sis = safeIntervals(m_env.getLocation(state_re));
         	    		  for (size_t i = 0; i < sis.size(); ++i) {

         	     			  const interval& si = sis[i];

         	     			  if (si.start - 1 > end_t || si.end - 1 < start_t) {
         	     				  continue;
         	     			  }
         	     			  if(si.start - 1 <= m_lastGScore){
         	     				  getJPSSuccessors(s_temp, Action::Up, 0, 0);
         	     			  }else{
         	     				  getJPSSuccessors(s_temp, Action::Up, si.start - 1 -m_lastGScore, 0);
         	     				  wait_time_u = si.start - 1 -m_lastGScore;
         	     			  }
         	    		  }
         	    	  }
         	      }

         	      if(s_temp.dir & 0x8){
         	    	  s_temp.action = Action::Down;
         	    	  state_re.x = s.state.x;
         	    	  state_re.y = s.state.y - 1;
       //  	    	  std::cout << "Down : \n";
         	    	  if(m_env.stateValid(state_re)){
         	    		  const auto& sis = safeIntervals(m_env.getLocation(state_re));
         	    		  for (size_t i = 0; i < sis.size(); ++i) {
         	     			  const interval& si = sis[i];
         	     			  if (si.start - 1 >  end_t || si.end - 1 < start_t) {
         	     				  continue;
         	     			  }
      //     	     			std::cout << "Time----- " << si.start - 1 << " end_t " << end_t << " " << si.end - 1 << " Start_t " << start_t << " "<<std::endl;
         	     			  if(si.start -  1 <= m_lastGScore){
         	     				  getJPSSuccessors(s_temp, Action::Down, 0, 0);
         	     			  }else{
         	     				  getJPSSuccessors(s_temp, Action::Down, si.start - 1 -m_lastGScore, 0);
         	     				  wait_time_d = si.start - 1 -m_lastGScore;
         	     			  }
         	    		  }
         	    	  }
         	      }

      //    	  m_env.num_generation--;
          	  for (const auto& m : jps_successors) {
          		  Cost m_time = m.cost;
          		  Cost start_t = m_lastGScore + m_time;
          		  Cost end_t =
          				  safeIntervals(m_env.getLocation(s.state)).at(s.interval).end;
      //    		  std::cout << m.state.state.x << " " << m.state.state.y <<  " Start " << start_t << " End: " << end_t << " Cost "<< m_time << "\n";
          		  Cost wait_time = 0;
          		  if(m.action == Action::Left){
          			  wait_time = wait_time_l;
          		  }else if(m.action == Action::Right){
          			  wait_time = wait_time_r;
          		  }else if(m.action == Action::Up){
          			  wait_time = wait_time_u;
          		  }else if(m.action == Action::Down){
          			  wait_time = wait_time_d;
          		  }else if(m.action == Action::Wait){
          			  wait_time = 0;
          		  }

          		  const auto& sis = safeIntervals(m_env.getLocation(m.state.state));
          		  for (size_t i = 0; i < sis.size(); ++i) {
          			  const interval& si = sis[i];
          			  if (si.start - m_time + wait_time > end_t || si.end < start_t) {
          				  continue;
          			  }
      //    			  std::cout << "Time again :: " << si.start << " " << si.end << " \n";
          			  int t;
          			  State temp_state = m.state.state;

          			  if (m_env.isCommandValid(s.state, m.state.state, m.action, m_lastGScore,
                                         	   end_t, si.start, si.end, t, m_time)){
              			  if(m.action == Action::Left){
                  				  if(IsEdgeCollisions(m.state.state, edgeCollision(t - 1, Action::Right))) continue;
                  		  } else if(m.action == Action::Right){
                  				  if(IsEdgeCollisions(m.state.state, edgeCollision(t - 1, Action::Left))) { continue;}
                  		  }else if(m.action == Action::Up){
                  				  if(IsEdgeCollisions(m.state.state, edgeCollision(t - 1, Action::Down))) continue;
                  		  }else if(m.action == Action::Down){
                  				  if(IsEdgeCollisions(m.state.state, edgeCollision(t - 1, Action::Up))) continue;
                  		  }

          				  neighbors.emplace_back(Neighbor<JPSSIPPState, JPSSIPPAction, Cost>(
          						  JPSSIPPState(m.state.state, i, m.state.dir), JPSSIPPAction(m.action, m.cost),
      							  t - m_lastGScore));

          			  }
          		  }
          	  }
            }
          }
        void getJPSSuccessors_bei2(JPSSIPPState s, Action action, Cost current_cost, int depth){

           	bool flag_solution = false;
           	if (m_env.isSolution(s.state)) {
           		flag_solution = true;
           		if(depth !=0 ){ jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(s, action, current_cost));}
           	}
           	if(isTempolObstacle(s.state)){
           		flag_solution = true;
           	}

            JPSSIPPState current_successor = s;
            m_env.num_generation++;

       		State state_up(s.state.x, s.state.y + 1);
       		State state_down(s.state.x, s.state.y -1);
       		bool is_up_edge_collision =  IsEdgeCollisions(state_up,edgeCollision(m_lastGScore + current_cost, Action::Down));
       		bool is_down_edge_collision =  IsEdgeCollisions(state_down,edgeCollision(m_lastGScore + current_cost, Action::Up));

        //       std::cout << "Current state : " << action << "  " << s.state.x << "  " << s.state.y << "Cost " << current_cost << "  ]]]]]]]]]]]]]]]\n";
           	if(action == Action::Left || (action == Action::Up && depth != 0) || (action == Action::Down && depth != 0) || action == Action::All){
          			current_successor.state.x = s.state.x - 1;
          			current_successor.state.y = s.state.y;
       //   			std::cout<<"Current successor :" << current_successor.state.x << " " << current_successor.state.y << "Left" << std::endl;
          			if(m_env.stateValid(current_successor.state) &&
          					!IsEdgeCollisions(current_successor.state,edgeCollision(m_lastGScore + current_cost,Action::Right))){
          				bool is_up = false, is_down = false;
          				unsigned int dir = 0x00;
          				if(m_env.isJumpPoint(current_successor.state)){
          					if( ((m_env.isObstacle(State(s.state.x, s.state.y + 1))
          							|| isTemporalObstacleSafe2(State(s.state.x, s.state.y + 1), m_lastGScore + current_cost + 1)
									|| is_up_edge_collision) &&
          							m_env.stateValid(State(s.state.x - 1, s.state.y + 1)))){
          						is_up = true;
          					}
          					if(((m_env.isObstacle(State(s.state.x, s.state.y - 1))
          							|| isTemporalObstacleSafe2(State(s.state.x, s.state.y - 1), m_lastGScore + current_cost + 1)
       							    || is_down_edge_collision)
          							&& m_env.stateValid(State(s.state.x - 1, s.state.y - 1)))){
          						is_down = true;
          					}
          				}

      					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)){
      						current_successor.dir = current_successor.dir| 0x2;
      					 }

          				if(is_up && !is_down){
          					current_successor.dir = current_successor.dir | 0x5;
          					current_successor.action = Action::Left;
          					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));
          				}

          				if(!is_up && is_down){
          					current_successor.dir = current_successor.dir | 0x9;
          					current_successor.action = Action::Left;
          					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));
          				}

          				if(is_up && is_down){
          					current_successor.dir = current_successor.dir | 0xd;
          					current_successor.action = Action::Left;
          					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));
          				}
          				if(!(is_up || is_down)){
        					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)){
        						current_successor.dir = 0x03;
        						current_successor.action = Action::Left;
         	          			jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));
         					}
         					else {
         						if(isTemporalObstacleSafe2(State(s.state.x - 2, s.state.y), m_lastGScore + current_cost + 1)
         								|| (m_env.is_limit && current_cost > m_env.limit_jump)) {
         							current_successor.dir = 0x1;
         							current_successor.action = Action::Left;
         		            		jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));
         						}else getJPSSuccessors(current_successor, Action::Left, current_cost + 1, depth + 1);
         					}

          				}
          			}
           	}

           	if(action == Action::Right || (action == Action::Up && depth != 0) || (action == Action::Down && depth != 0) || action == Action::All){
          			current_successor.state.x = s.state.x + 1;
          			current_successor.state.y = s.state.y;
       //   			m_env.num_generation++;
       //   			std::cout<<"Current state :" << s.state.x << " " << s.state.y << "Right" << std::endl;
       //   			std::cout<<"Current successor :" << current_successor.state.x << " " << current_successor.state.y << "Right" << std::endl;
          			if(m_env.stateValid(current_successor.state)
          					&& !IsEdgeCollisions(current_successor.state,edgeCollision(m_lastGScore + current_cost,Action::Left))){
          				bool is_up = false, is_down = false;
          				unsigned int dir = 0x00;
          				if(m_env.isJumpPoint(current_successor.state)){
         					if(((m_env.isObstacle(State(s.state.x, s.state.y + 1))
         							|| isTemporalObstacleSafe2(State(s.state.x, s.state.y + 1), m_lastGScore + current_cost +1)
									|| is_up_edge_collision)
         							&& m_env.stateValid(State(s.state.x + 1, s.state.y + 1)))){
          						is_up = true;
          					}
         					if(((m_env.isObstacle(State(s.state.x, s.state.y - 1))
          							|| isTemporalObstacleSafe2(State(s.state.x, s.state.y - 1), m_lastGScore + current_cost + 1)
									|| is_down_edge_collision)
          							&& m_env.stateValid(State(s.state.x + 1, s.state.y - 1)))){
          						is_down = true;
       //   						std::cout << "Re-start Down\n";
          					}
          				}

     					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)){
     						current_successor.dir = current_successor.dir | 0x1;
     					}

          				if(is_up && !is_down){
          					current_successor.dir = current_successor.dir | 0x6;
          					current_successor.action = Action::Right;
       //   					std::cout << "Direction " << current_successor.dir << " Herehere\n";
          					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));

          				}

          				if(!is_up&&is_down){
          					current_successor.dir = current_successor.dir | 0x0a;
          					current_successor.action = Action::Right;
          					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));
          				}

          				if(is_up&&is_down){
          					current_successor.dir =  current_successor.dir | 0x0e;
          					current_successor.action = Action::Right;
          					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));

          				}
          				if(!(is_up || is_down)){
         					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)) {
         						current_successor.dir = 0x03;
         						current_successor.action = Action::Right;
         						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));

         					} else{
         						if(isTemporalObstacleSafe2(State(s.state.x + 2, s.state.y), m_lastGScore + current_cost + 1)
         								|| (m_env.is_limit && current_cost > m_env.limit_jump)){
         							current_successor.dir = 0x02;
         							current_successor.action = Action::Right;
         							jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));
         						} else getJPSSuccessors(current_successor, Action::Right, current_cost + 1, depth + 1);
         					}
          				}
          			}
           	}

           	if(action == Action::Up || action == Action::All){

           		current_successor.state.x = s.state.x;
           		current_successor.state.y = s.state.y + 1;
   //        		std::cout << "UP : " << current_successor.state.x << "  " << current_successor.state.y << "----------\n";
           			if(m_env.stateValid(current_successor.state)
          					&& !IsEdgeCollisions(current_successor.state,edgeCollision(m_lastGScore + current_cost,Action::Down))){
       				if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)) {
       					current_successor.dir = 0xf;
       					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost + 1));
       				}
       				else {
       					if( isTemporalObstacleSafe2(State(s.state.x, s.state.y + 2), m_lastGScore + current_cost + 1)
       							|| isTemporalObstacleSafe2(State(s.state.x - 1, s.state.y + 1), m_lastGScore + current_cost + 1)
       							|| isTemporalObstacleSafe2(State(s.state.x + 1, s.state.y + 1), m_lastGScore + current_cost + 1)
   								|| (m_env.is_limit && current_cost > m_env.limit_jump)){
       						current_successor.dir = 0x07;
       						current_successor.action = Action::Up;
       						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost + 1));
       					} /*else if(isTemporalObstacleSafe2(current_successor.state, m_lastGScore + current_cost + 1)){
       						current_successor.dir = 0x07;
       						current_successor.action = Action::Up;
       						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost + 1));
       					}*/else { getJPSSuccessors(current_successor, Action::Up, current_cost + 1, depth + 1);}

       				}
          			}

           	}
           	if(action == Action::Down || action == Action::All){
           		current_successor.state.x = s.state.x;
           		current_successor.state.y = s.state.y - 1;
        	 	 	if(m_env.stateValid(current_successor.state)
          					&& !IsEdgeCollisions(current_successor.state,edgeCollision(m_lastGScore + current_cost,Action::Up))){

          				if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)) {
          					current_successor.dir = 0xf;
          					current_successor.action = Action::Down;
          					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Down, current_cost + 1));
          				} else {

          					if(isTemporalObstacleSafe2(State(s.state.x, s.state.y -2), m_lastGScore + current_cost + 1)
          							|| isTemporalObstacle(State(s.state.x - 1, s.state.y -1), m_lastGScore + current_cost + 1)
       							|| isTemporalObstacle(State(s.state.x + 1, s.state.y - 1), m_lastGScore + current_cost + 1)
   								||(m_env.is_limit && current_cost > m_env.limit_jump)){
          						current_successor.dir = 0x0b;
          						current_successor.action = Action::Down;
          						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Down, current_cost + 1));
          					}/* else if(isTemporalObstacleSafe2(current_successor.state, m_lastGScore + current_cost + 1)){
          						current_successor.dir = 0x0b;
          						current_successor.action = Action::Down;
          						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Down, current_cost + 1));
          					}*/else getJPSSuccessors(current_successor, Action::Down, current_cost + 1, depth + 1);
          				}
           		}
           	}
           }

       void getNeighbors(
              const JPSSIPPState& s,
              std::vector<Neighbor<JPSSIPPState, JPSSIPPAction, Cost> >& neighbors) {


            std::vector<Neighbor<State, Action, Cost> > motions;

            JPSSIPPState s_temp = s;

            if(!m_env.isJPS()){
          	  m_env.getNeighbors(s_temp.state, motions);
       //   	  std::cout << "The number of neighbors: " << motions.size() << " " << std::endl;
                for (const auto& m : motions) {
              	  m_env.num_generation++;
      //        	  std::cout << "Current successor: " << m.state.x << " " << m.state.y << "\n";
                  Cost m_time = m.cost;
                  Cost start_t = m_lastGScore + m_time;
                  Cost end_t =
                      safeIntervals(m_env.getLocation(s.state)).at(s.interval).end;

                  const auto& sis = safeIntervals(m_env.getLocation(m.state));
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
                    if (m_env.isCommandValid(s.state, m.state, m.action, m_lastGScore,
                                             end_t, si.start, si.end, t, m_time)
                    		&& !IsEdgeCollisions(m.state, edgeCollision(t - 1, a_temp))) {
                      neighbors.emplace_back(Neighbor<JPSSIPPState, JPSSIPPAction, Cost>(
                          JPSSIPPState(m.state, i,0xf), JPSSIPPAction(m.action, m.cost),
                          t - m_lastGScore));
                    }
                  }
                }
            }else {

          	  Cost start_t = m_lastGScore;
      		  Cost end_t =
      				  safeIntervals(m_env.getLocation(s.state)).at(s.interval).end;

      		  State state_re = s.state;
      		  m_env.num_expansion++;
   		  std::cout << "Current State " << s.state.x << "  " << s.state.y <<" Gscore" << m_lastGScore << " interval " << s.interval << " Direction " << s.dir << "  ********************************\n";
      		  jps_successors.clear();
      		  Cost wait_time_l = 0, wait_time_r = 0, wait_time_u = 0, wait_time_d = 0;
         	      if(s_temp.dir & 0x1){
         	    	  s_temp.action = Action::Left;
         	    	  state_re.x = s.state.x -1;
         	    	  state_re.y = s.state.y;
         	    	  if(m_env.stateValid(state_re)){
         	    		  const auto& sis = safeIntervals(m_env.getLocation(state_re));
         	    		  for (size_t i = 0; i < sis.size(); ++i) {
         	     			  const interval& si = sis[i];
         	     			  if (si.start - 1 > end_t || si.end - 1 < start_t) {
         	     				  continue;
         	     			  }
         	     			  if(si.start -1 <= m_lastGScore){
         	     				  getJPSSuccessors(s_temp, Action::Left, 0, 0);
         	     			  }else{
         	     				  getJPSSuccessors(s_temp, Action::Left, si.start - 1 -m_lastGScore, 0);
         	     				  wait_time_l = si.start - 1 - m_lastGScore;
         	     			  }
         	     			  if( i + 1 < sis.size()
         	     					&& !IsEdgeCollisions(state_re, edgeCollision(sis[i+1].start-1, Action::Right))){
         	     				  if(end_t >= sis[i+1].start - 1){
         	     						 JPSSIPPState temp_state = s;
         	     						 temp_state.state = s.state;
         	     						 temp_state.dir = 0x1;
         	     						 temp_state.flag_wait = true;
         	     						 temp_state.action = Action::Left;
         	     						 jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(temp_state, Action::Left, sis[i+1].start -1 -m_lastGScore));
         	     				  }
         	     			  }
         	     			  break;
         	    		  }
         	    	  }
         	      }
         	      if(s_temp.dir & 0x2){
         	    	  s_temp.action = Action::Right;
         	    	  state_re.x = s.state.x + 1;
         	    	  state_re.y = s.state.y;
       //  	    	 std::cout << "Right Right : " << state_re.x << " " << state_re.y << " direction \n";
         	    	  if(m_env.stateValid(state_re)){
         	    		  const auto& sis = safeIntervals(m_env.getLocation(state_re));
       //  	    		  std::cout << "The size of sis " << sis.size() << " The size of (2,7)" << tt.size() <<std::endl;
         	    		  for (size_t i = 0; i < sis.size(); ++i) {
         	     			  const interval& si = sis[i];
         	     			  if (si.start - 1 > end_t  || si.end - 1 < start_t ) {
         	     				  continue;
         	     			  }
         	     			  if(si.start - 1 <= m_lastGScore){
         	     				  getJPSSuccessors(s_temp, Action::Right, 0, 0);
         	     			  }else{
         	     				  getJPSSuccessors(s_temp, Action::Right, si.start - 1 -m_lastGScore, 0);
         	     				  wait_time_r = si.start - 1 -m_lastGScore;
         	     			  }
         	     			  if( i + 1 < sis.size()){
         	     				  if(end_t >= sis[i+1].start - 1
         	     						&& !IsEdgeCollisions(state_re, edgeCollision(sis[i+1].start-1, Action::Left))){
         	     						 JPSSIPPState temp_state = s;
         	     						 temp_state.state = s.state;
         	     						 temp_state.dir = 0x2;
         	     						 temp_state.flag_wait = true;
         	     						 temp_state.action = Action::Right;
         	     						 jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(temp_state, Action::Right, sis[i+1].start -1 -m_lastGScore));
         	     				  }
         	     			  }
         	     			  break;
         	    		  }
         	    	  }
         	      }
         	      if(s_temp.dir & 0x4){
         	    	  s_temp.action = Action::Up;
         	    	  state_re.x = s.state.x;
         	    	  state_re.y = s.state.y + 1;
      //   	    	  std::cout << "Up : " << state_re.x << " " << state_re.y  << "end :	" << " direction \n";
         	    	  if(m_env.stateValid(state_re)){
         	    		  const auto& sis = safeIntervals(m_env.getLocation(state_re));
         	    		  for (size_t i = 0; i < sis.size(); ++i) {

         	     			  const interval& si = sis[i];

         	     			  if (si.start - 1 > end_t || si.end - 1 < start_t) {
         	     				  continue;
         	     			  }
         	     			  if(si.start - 1 <= m_lastGScore){
         	     				  getJPSSuccessors(s_temp, Action::Up, 0, 0);
         	     			  }else{
         	     				  getJPSSuccessors(s_temp, Action::Up, si.start - 1 -m_lastGScore, 0);
         	     				  wait_time_u = si.start - 1 -m_lastGScore;
         	     			  }

         	     			  if( i + 1 < sis.size()){
         	     				  if(end_t >= sis[i+1].start - 1 &&
         	     						  !IsEdgeCollisions(state_re,edgeCollision(sis[i+1].start-1, Action::Down))){
         	     						 JPSSIPPState temp_state = s;
         	     						 temp_state.state = s.state;
         	     						 temp_state.dir = 0x7;
         	     						 temp_state.flag_wait = true;
         	     						 temp_state.action = Action::Up;
         	     						 jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(temp_state, Action::Up, sis[i+1].start -1 -m_lastGScore));
         	     				  }
         	     			  }
         	     			  break;
         	    		  }
         	    	  }
         	      }

         	      if(s_temp.dir & 0x8){
         	    	  s_temp.action = Action::Down;
         	    	  state_re.x = s.state.x;
         	    	  state_re.y = s.state.y - 1;
       //  	    	  std::cout << "Down : \n";
         	    	  if(m_env.stateValid(state_re)){
         	    		  const auto& sis = safeIntervals(m_env.getLocation(state_re));
         	    		  for (size_t i = 0; i < sis.size(); ++i) {
         	     			  const interval& si = sis[i];
         	     			  if (si.start - 1 >  end_t || si.end - 1 < start_t) {
         	     				  continue;
         	     			  }
         	     			  if(si.start -  1 <= m_lastGScore){
         	     				  getJPSSuccessors(s_temp, Action::Down, 0, 0);
         	     			  }else{
         	     				  getJPSSuccessors(s_temp, Action::Down, si.start - 1 -m_lastGScore, 0);
         	     				  wait_time_d = si.start - 1 -m_lastGScore;
         	     			  }
         	     			  if( i + 1 < sis.size()){
         	     				  if(end_t >= sis[i+1].start - 1
         	     						&& !IsEdgeCollisions(state_re, edgeCollision(sis[i+1].start-1, Action::Up))){
         	     						 JPSSIPPState temp_state = s;
         	     						 temp_state.state = s.state;
         	     						 temp_state.dir = 0xb;
         	     						 temp_state.flag_wait = true;
         	     						 temp_state.action = Action::Down;
         	     						 jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(temp_state, Action::Down, sis[i+1].start -1 -m_lastGScore));
         	     				  }
         	     			  }
         	     			  break;
         	    		  }
         	    	  }
         	      }
          	  for (const auto& m : jps_successors) {
         		  std::cout << "Successor +++++++++++ ------ " << m.state.state.x << " "<< m.state.state.y << " Cost " << m.cost + m_lastGScore << " flag " << m.state.flag_wait << "\n";
  				  neighbors.emplace_back(Neighbor<JPSSIPPState, JPSSIPPAction, Cost>(
  						  JPSSIPPState(m.state.state, m.state.interval, m.state.dir,  m_lastGScore + m.cost, m.state.flag_wait), JPSSIPPAction(m.action, m.cost),
							  m.cost));

          	  }
            }
          }
       void getJPSSuccessors(JPSSIPPState s, Action action, Cost current_cost, int depth){

           	bool flag_solution = false;
           	if (m_env.isSolution(s.state)) {
//           		std::cout << "---- " << s.state.x << " " << s.state.y << std::endl;
           		flag_solution = true;
           		if(depth !=0 ){ jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(s, action, current_cost));}
           	}
           	if(isTempolObstacle(s.state)){
           		flag_solution = true;
           	}

            JPSSIPPState current_successor = s;
            current_successor.dir = 0x0;
            m_env.num_generation++;

       		State state_up(s.state.x, s.state.y + 1);
       		State state_down(s.state.x, s.state.y -1);
       		bool is_up_edge_collision =  IsEdgeCollisions(state_up,edgeCollision(m_lastGScore + current_cost,Action::Down));
       		bool is_down_edge_collision =  IsEdgeCollisions(state_down,edgeCollision(m_lastGScore + current_cost,Action::Up));

       		const auto& si_s = safeIntervals(m_env.getLocation(s.state));
       		Cost start_t = m_lastGScore + current_cost;
       		Cost end_t = si_s.at(s.interval).end;


           	if(action == Action::Left || (action == Action::Up && depth != 0) || (action == Action::Down && depth != 0)){
          			current_successor.state.x = s.state.x - 1;
          			current_successor.state.y = s.state.y;
//         			std::cout<<"Current successor :" << current_successor.state.x << " " << current_successor.state.y << "Left" << std::endl;
          			if(m_env.stateValid(current_successor.state) &&
          					!IsEdgeCollisions(current_successor.state,edgeCollision(m_lastGScore + current_cost, Action::Right))){
          				size_t successor_interval;
          				Cost successor_start_t, successor_end_t;
          				Cost next_start = -1, next_end = -1;
          				findSafeInterval(current_successor.state, start_t + 1, successor_interval,                  //find the safe interval
          				          						successor_start_t, successor_end_t, next_start, next_end);
          				bool flag_push = false;
          				if(next_start != -1){
         					if(isTemporalObstacleSafe2(current_successor.state, m_lastGScore + current_cost + 1)
         							&& next_start != -1 && end_t >= next_start - 1){
         						if(!IsEdgeCollisions(current_successor.state,edgeCollision(next_start - 1,Action::Right))){
         							std::cout << "Left 0000\n";
             						JPSSIPPState temp_state = s;
             						temp_state.dir = 0x1;
             						temp_state.flag_wait = false;
             						temp_state.action = Action::Left;
             						if(depth != 0) {
             							flag_push = true;
             							jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(temp_state, Action::Left, current_cost));
             						}
         						}
         					}
          				}

         				if(successor_start_t !=- 1 && !flag_push){
          					current_successor.interval = successor_interval;
          					current_successor.dir = 0x00;
              				bool is_up = false, is_down = false;
              				Cost up_start_t = -1, down_start_t = -1, right_start_t = -1;

              				if(m_env.isJumpPoint(current_successor.state)){
              					if( ((m_env.isObstacle(State(s.state.x, s.state.y + 1))
										|| is_up_edge_collision)
              							|| isTemporalObstacleSafe(State(s.state.x, s.state.y + 1), m_lastGScore + current_cost + 1)
              							&&	m_env.stateValid(State(s.state.x - 1, s.state.y + 1)))){
              						if(!IsEdgeCollisions(State(s.state.x - 1, s.state.y + 1), edgeCollision(m_lastGScore + current_cost, Action::Down))){
              							up_start_t = m_lastGScore + current_cost;
              							is_up = true;
              						}
              					} else if (isTemporalObstacleSafe2(State(s.state.x - 1, s.state.y + 1), m_lastGScore + current_cost + 1, up_start_t)
              							&& m_env.stateValid(State(s.state.x - 1, s.state.y + 1))){
             						if(up_start_t != -1){
             							if(!IsEdgeCollisions(State(s.state.x - 1, s.state.y + 1), edgeCollision(up_start_t, Action::Down))){
             								is_up = true;
             							}
              						}
              					}

              					if(((m_env.isObstacle(State(s.state.x, s.state.y - 1))
										|| is_down_edge_collision
										|| isTemporalObstacleSafe(State(s.state.x, s.state.y - 1), m_lastGScore + current_cost + 1))
              							&& m_env.stateValid(State(s.state.x - 1, s.state.y - 1)))){
              						if(!IsEdgeCollisions(State(s.state.x - 1, s.state.y - 1), edgeCollision(m_lastGScore + current_cost, Action::Up))){
                  						down_start_t = m_lastGScore + current_cost;
                  						is_down = true;
              						}
              					} else if(isTemporalObstacleSafe2(State(s.state.x - 1, s.state.y - 1), m_lastGScore + current_cost + 1, down_start_t)
              							&& m_env.stateValid(State(s.state.x - 1, s.state.y - 1))){
              						if(down_start_t != -1){
              							if(!IsEdgeCollisions(State(s.state.x - 1, s.state.y - 1), edgeCollision(down_start_t, Action::Up))){
              								is_down = true;
              							}
              						}
              					}
              				}
              				Cost next_start_s = -1;
          					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1, right_start_t)){ // Check the edge collision
          						if(si_s.size() > s.interval + 1){
          							next_start_s = si_s.at(s.interval + 1).start;
          							if(!IsEdgeCollisions(s.state,edgeCollision(next_start_s - 1, Action::Left))
          									&& successor_end_t >= next_start_s - 1){
          								right_start_t = next_start_s - 1;
          								current_successor.dir =  0x2;
          							}
          						}
          					 }
          					std::cout << " Up " << up_start_t << " Down  " << down_start_t << " Right " << right_start_t << "\n";
             				if(is_up && !is_down){
              					current_successor.dir =  current_successor.dir | 0x5;
              					current_successor.action = Action::Left;
              					current_successor.flag_wait = false;
              					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));
              				}

              				if(!is_up && is_down){
              					current_successor.dir = current_successor.dir | 0x9;
              					current_successor.action = Action::Left;
              					current_successor.flag_wait = false;
              					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));
              				}

              				if(is_up && is_down){
              					current_successor.dir = current_successor.dir | 0x0d;
              					current_successor.action = Action::Left;
              					current_successor.flag_wait = false;
              					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Left, current_cost + 1));
              				}
              				if(!(is_up || is_down)){
              					if(current_successor.dir == 0x02){
//              						current_successor.flag_wait = true;
//              						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, next_start_s - 1 -m_lastGScore));
              						current_successor.flag_wait = false;
              						current_successor.dir = 0x03;
              						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));
              					} else {
             						current_successor.dir = 0x01;
              						getJPSSuccessors(current_successor, Action::Left, current_cost + 1, depth + 1);
              					}
              				}
          				}
          			}
           	}

           	if(action == Action::Right || (action == Action::Up && depth != 0) || (action == Action::Down && depth != 0)){
          			current_successor.state.x = s.state.x + 1;
          			current_successor.state.y = s.state.y;

          			if(m_env.stateValid(current_successor.state)
          					&& !IsEdgeCollisions(current_successor.state,edgeCollision(m_lastGScore + current_cost,Action::Left))){
//          				std::cout << "Right " << current_successor.state.x << " " << current_successor.state.y << "\n";
          				bool is_up = false, is_down = false;
          				size_t successor_interval;
          				Cost successor_start_t, successor_end_t;
          				Cost next_start = -1, next_end = -1;
          				findSafeInterval(current_successor.state, start_t + 1, successor_interval, successor_start_t, successor_end_t, next_start, next_end);
          				bool flag_push = false;
          				if(next_start != -1){
         					if(isTemporalObstacleSafe2(current_successor.state, m_lastGScore + current_cost + 1)
         							&& next_start != -1 && end_t >= next_start - 1){
         						if(!IsEdgeCollisions(current_successor.state,edgeCollision(next_start - 1,Action::Left))){
//         							std::cout << "Right 0000\n";
             						JPSSIPPState temp_state = s;
             						temp_state.dir = 0x2;
             						temp_state.flag_wait = false;
             						temp_state.action = Action::Right;
             						if(depth != 0) {
             							flag_push = true;
             							jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(temp_state, Action::Right, current_cost));
             						}
         						}
         					}
          				}
          				if(successor_start_t != -1 && !flag_push){
          					current_successor.interval = successor_interval;
          					current_successor.dir = 0x00;

          					Cost up_start_t = -1, down_start_t = -1, left_start_t = -1;
              				if(m_env.isJumpPoint(current_successor.state)){
             					if(((m_env.isObstacle(State(s.state.x, s.state.y + 1))
										|| is_up_edge_collision)
             							|| isTemporalObstacleSafe(State(s.state.x, s.state.y + 1), m_lastGScore + current_cost +1)
             							&& m_env.stateValid(State(s.state.x + 1, s.state.y + 1)))){
              						if(!IsEdgeCollisions(State(s.state.x + 1, s.state.y + 1), edgeCollision(m_lastGScore + current_cost, Action::Down))){
              							up_start_t = m_lastGScore + current_cost;
              							is_up = true;
              						}
              					} else if(isTemporalObstacleSafe2(State(s.state.x + 1, s.state.y + 1), m_lastGScore + current_cost +1, up_start_t)
              							&& m_env.stateValid(State(s.state.x + 1, s.state.y + 1))){
              						if(up_start_t != -1 && !IsEdgeCollisions(State(s.state.x + 1, s.state.y + 1), edgeCollision(up_start_t, Action::Down))){
              							is_up = true;
              						}
              					}

             					if(((m_env.isObstacle(State(s.state.x, s.state.y - 1))
										|| is_down_edge_collision)
             							|| isTemporalObstacleSafe(State(s.state.x, s.state.y - 1), m_lastGScore + current_cost +1)
              							&& m_env.stateValid(State(s.state.x + 1, s.state.y - 1)))){
              						if(!IsEdgeCollisions(State(s.state.x + 1, s.state.y - 1), edgeCollision(m_lastGScore + current_cost, Action::Up))){
                 						down_start_t = m_lastGScore + current_cost + 1;
                  						is_down = true;
              						}
              					} else if(isTemporalObstacleSafe2(State(s.state.x + 1, s.state.y - 1), m_lastGScore + current_cost + 1, down_start_t)
              							&&m_env.stateValid(State(s.state.x + 1, s.state.y - 1))){
              						if(down_start_t != -1 && !IsEdgeCollisions(State(s.state.x + 1, s.state.y - 1), edgeCollision(down_start_t, Action::Up))){
              							is_down = true;
              						}
              						is_down = true;
              					}
              				}

         					Cost next_start_s = -1;
         					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)) {
             					if(si_s.size()> s.interval + 1){
             						next_start_s = si_s.at(s.interval + 1).start;
             						if(!IsEdgeCollisions(s.state, edgeCollision(next_start_s - 1, Action::Right))
             								&& successor_end_t >= next_start_s - 1){
             							current_successor.dir = 0x1;
             							left_start_t = next_start_s - 1;
             						}
             					 }
             				}

         					std::cout << " Up " << up_start_t << " Down  " << down_start_t << " Left " << left_start_t << "\n";

              				if(is_up && !is_down){
//                  				std::cout << "Right 1" << "\n";
             					current_successor.dir = current_successor.dir | 0x6;
              					current_successor.flag_wait = false;
              					current_successor.action = Action::Right;
              					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));
              				}

              				if(!is_up&&is_down){
//                  				std::cout << "Right 2" << "\n";
              					current_successor.dir = current_successor.dir | 0x0a;
              					current_successor.action = Action::Right;
              					current_successor.flag_wait = false;
              					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));
              				}

              				if(is_up&&is_down){
//                  				std::cout << "Right 3" << "\n";
              					current_successor.dir = current_successor.dir | 0x0e;
              					current_successor.action = Action::Right;
              					current_successor.flag_wait = false;
              					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));
              				}

              				if(!(is_up || is_down)){
//                  				std::cout << "Right 4" << "\n";
              					if(current_successor.dir == 0x01){
              							current_successor.flag_wait = false;
              							current_successor.dir = 0x03;
              							jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Right, current_cost + 1));
              					}else {
              						current_successor.dir = 0x02;
              						getJPSSuccessors(current_successor, Action::Right, current_cost + 1, depth + 1);
              					}
              				}
          				}
          			}
           	}

           	if(action == Action::Up){

           		current_successor.state.x = s.state.x;
           		current_successor.state.y = s.state.y + 1;
           		if(m_env.stateValid(current_successor.state)
          					&& !IsEdgeCollisions(current_successor.state, edgeCollision(m_lastGScore + current_cost,Action::Down))){

          				size_t successor_interval;
          				Cost successor_start_t =- 1, successor_end_t = -1;
          				Cost next_start = -1, next_end = -1;
          				findSafeInterval(current_successor.state, start_t + 1, successor_interval,
          				          						successor_start_t, successor_end_t, next_start, next_end);
          				bool flag_push = false;
          				if(next_start != -1){
       						if(isTemporalObstacleSafe2(current_successor.state, m_lastGScore + current_cost + 1)
       								&& next_start != -1 && end_t >= next_start - 1){
         						if(!IsEdgeCollisions(current_successor.state,edgeCollision(next_start - 1, Action::Down))){
             						JPSSIPPState temp_state = s;
             						temp_state.dir = 0x4;
             						temp_state.interval = s.interval;
             						temp_state.action = Action::Up;
             						temp_state.flag_wait = false;
             						if(depth != 0){
             							flag_push = true;
             							std::cout << "UP UP UP UP 0---------\n";
             							jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(temp_state, Action::Up, current_cost + 1));
             						}
         						}
       						}
          				}
          				if(successor_start_t != -1 && !flag_push){
          					current_successor.interval = successor_interval;
          					current_successor.dir = 0x00;
         					Cost next_start_s = -1;
         					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)) {
             					if(si_s.size()> s.interval + 1){
             						next_start_s = si_s.at(s.interval + 1).start;
             						if(!IsEdgeCollisions(s.state, edgeCollision(next_start_s - 1, Action::Up)) && successor_end_t >= next_start_s - 1){
             							current_successor.dir = 0x08;
             						}
             					 }
             				}

           					if( isTemporalObstacleSafe2(State(s.state.x, s.state.y + 2), m_lastGScore + current_cost + 1)
//           						|| isTemporalObstacleSafe2(State(s.state.x - 1, s.state.y + 1), m_lastGScore + current_cost + 1)
//           						|| isTemporalObstacleSafe2(State(s.state.x + 1, s.state.y + 1), m_lastGScore + current_cost + 1)
       							|| (m_env.is_limit && current_cost > m_env.limit_jump)){
           						current_successor.dir = current_successor.dir | 0x07;
                  				current_successor.flag_wait = false;
           						current_successor.action = Action::Up;
           						std::cout << "UP UP UP UP 1 ---------\n";
           						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost + 1));
           					} else {
           						if(current_successor.dir == 0x08){
              							current_successor.dir = 0x0f;
                  						current_successor.flag_wait = false;
                  						std::cout << "UP UP UP UP 2---------\n";
                  						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Down, current_cost + 1));
           						} else {
           							current_successor.dir = 0x07;
           							getJPSSuccessors(current_successor, Action::Up, current_cost + 1, depth + 1);
           						}
           					}
          				}
          			}
           	}
           	if(action == Action::Down){
           		current_successor.state.x = s.state.x;
           		current_successor.state.y = s.state.y - 1;
        	 	 	if(m_env.stateValid(current_successor.state)
          					&& !IsEdgeCollisions(current_successor.state,edgeCollision(m_lastGScore + current_cost,Action::Up))){
//        	 	 		std::cout << "Down : " << current_successor.state.x << " " << current_successor.state.y << "\n";
          				size_t successor_interval;
          				Cost successor_start_t = -1, successor_end_t = -1;
          				Cost next_start = -1, next_end = -1;
          				findSafeInterval(current_successor.state, start_t + 1, successor_interval,
          				          						successor_start_t, successor_end_t, next_start, next_end);
          				bool flag_push = false;
          			    if(next_start != -1){
       						if(isTemporalObstacleSafe2(current_successor.state, m_lastGScore + current_cost + 1)
       								&& next_start != -1 && end_t >= next_start - 1){
         						if(!IsEdgeCollisions(current_successor.state,edgeCollision(next_start - 1, Action::Up))){

             						JPSSIPPState temp_state = s;
             						temp_state.dir = 0x08;
             						temp_state.interval = s.interval;
             						temp_state.action = Action::Down;
             						temp_state.flag_wait = false;
            						if(depth != 0){
            							flag_push = true;
            							jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(temp_state, Action::Down, current_cost + 1));
            						}

         						}
       						}
          				}
          			    if(successor_start_t != -1 && !flag_push){

          					current_successor.interval = successor_interval;
         					Cost next_start_s = -1;
         					if(flag_solution && isTemporalObstacleSafe2(s.state, m_lastGScore + current_cost + 1)) {
             					if(si_s.size()> s.interval + 1){
             						next_start_s = si_s.at(s.interval + 1).start;
             						if(!IsEdgeCollisions(s.state, edgeCollision(next_start_s - 1, Action::Down)) && successor_end_t >= next_start_s - 1){
             							current_successor.dir = 0x04;
             						}
             					 }
             				}
              				if(isTemporalObstacleSafe2(State(s.state.x, s.state.y -2), m_lastGScore + current_cost + 1)
//              						|| isTemporalObstacle(State(s.state.x - 1, s.state.y -1), m_lastGScore + current_cost + 1)
//	           						    || isTemporalObstacle(State(s.state.x + 1, s.state.y - 1), m_lastGScore + current_cost + 1)
       							    ||(m_env.is_limit && current_cost > m_env.limit_jump)){
//              						std::cout << "Here Down 2: " << start_t << " end_t " << end_t << " next_start " << next_start << "\n";
              					current_successor.dir = current_successor.dir | 0x0b;
                  				current_successor.flag_wait = false;
              					current_successor.action = Action::Down;
              					jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Down, current_cost + 1));
              				} else{
              					if(current_successor.dir == 0x04){
              							current_successor.dir = 0x0f;
                  						current_successor.flag_wait = false;
                  						jps_successors.emplace_back(Neighbor<JPSSIPPState, Action, Cost>(current_successor, Action::Up, current_cost + 1));
              					}else {
              						current_successor.dir = 0x0b;
              						getJPSSuccessors(current_successor, Action::Down, current_cost + 1, depth + 1);
              					}
              				}
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
//    	m_env.num_expansion++;
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

//      std::cout << location << ": " << std::endl;
      if (intervals.size() > 0) {
        m_safeIntervals[location]; // create empty safe interval
        int start = 0;
        int lastEnd = 0;
        for (const auto& interval : sortedIntervals) {
 //           std::cout << "  ci: " << start << " " << lastEnd << " " << interval.start << " - " << interval.end << std::endl;
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
      // auto iter = m_safeIntervals.find(location);
      // if (iter != m_safeIntervals.end()) {
      //   for (const auto& si : iter->second) {
      //     std::cout << "  si: " << si.start << " - " << si.end << std::endl;
      //   }
      // }
    }

    void setEdgeCollisions(const Location& location,
  		  	  	  	  	const std::vector<edgeCollision>& edge_collision) {
  	  m_edgeCollision.erase(location);
  	  if(edge_collision.size() > 0){
  		  m_edgeCollision[location];
  		  for(const auto& ec : edge_collision){
  			  m_edgeCollision[location].push_back(ec);
  		  }
  	  }
    }

    bool IsEdgeCollisions(const Location& location, const edgeCollision& ec){
//    		std::cout << location.x << " " << location.y << " " << ec.t << " " << ec.action << std::endl;
//    		return false;
    		const auto iter = m_edgeCollision.find(location);
    		if(iter == m_edgeCollision.end()) return false;
    		if((iter->second).size() == 0) return false;
    		for (auto& cec : (iter->second)){
    			if( cec == ec ){
    				return true;
    			}
    		}
    		return false;
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

    bool findSafeInterval(const State& state, Cost time, size_t& interval, Cost &start_t, Cost &end_t)
    {
      const auto& si = safeIntervals(m_env.getLocation(state));
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
      const auto& si = safeIntervals(m_env.getLocation(state));
      start_t = -1;
      end_t = -1;
      next_start = -1;
      next_end = -1;
//      std::cout << "Safe Inteval ---------- " << state.x << " " << state.y << "\n";
      bool first_safe = true;
      for (size_t idx = 0; idx < si.size(); ++idx) {
//          std::cout << start_t << " " << end_t << " " << next_start <<" " << next_end << "-------------------\n";
//            std::cout << "Time " << time << " " << si[idx].start << " " << si[idx].end << " "<< "-------------------\n";
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
//    		  std::cout << start_t << " " << end_t << " " << next_start <<" " << next_end << "-------------------\n";
    		  return true;
        }
      }
      return false;
    }

    bool isTempolObstacle(const Location& location){
    	return m_safeIntervals.find(location) != m_safeIntervals.end();
    }

    bool isTemporalObstacle(const Location& location, Cost now_cost){

        const auto& si = safeIntervals(m_env.getLocation(location));
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

    bool isTemporalObstacleSafe2(const Location& location, Cost time){ // whether the obstacle appears after the time
        const auto& si = safeIntervals(m_env.getLocation(location));
//        std::cout << location.x << "  " << location.y << " Cost: " << time << " si.size " << si.size() << "\n";
        if(si.size() == 0) return true;
        Cost start_ = 0, end_ = si[0].start - 1;
        for (size_t idx = 0; idx < si.size() - 1; ++idx) {
//        	std::cout << start_ << " " << end_<<"\n";
        	if(start_ <= end_ && end_ >= time){ return true;}
        	start_ = si[idx].end + 1;
        	end_ = si[idx + 1].start - 1;
        }
    	if(start_ <= end_ && end_ >= time) { return true;}

        if(si.back().end == std::numeric_limits<Cost>::max()) { return false;}
        else { return true;}
    }


    bool isTemporalObstacleSafe2(const Location& location, Cost time, Cost &start_time){ // whether the obstacle appears after the time
        const auto& si = safeIntervals(m_env.getLocation(location));
//        std::cout << location.x << "  " << location.y << " Cost: " << time << " si.size " << si.size() << "\n";
        start_time = -1;
        if(si.size() == 0) return true;
        Cost start_ = 0, end_ = si[0].start - 1;
        for (size_t idx = 0; idx < si.size() - 1; ++idx) {
//        	std::cout << start_ << " " << end_<<"\n";
        	if(start_ <= end_ && end_ >= time){ start_time = end_; return true;}
        	start_ = si[idx].end + 1;
        	end_ = si[idx + 1].start - 1;
        }
    	if(start_ <= end_ && end_ >= time) {start_time = end_ ; return true;}

        if(si.back().end == std::numeric_limits<Cost>::max()) { return false;}
        else { start_time = -1; return true;}
    }

  public:
	bool UpdateSafeInterval(Location location, Cost start_, Cost end_){
    	assert(start_<=end_);
    	auto si = m_safeIntervals.find(location);
    	assert(si != m_safeIntervals.end());

    	for(size_t idx = 0; idx < m_safeIntervals[location].size(); idx++){
    		if(si[idx].end > start_ || si[idx].start >end_) continue;
    		if(si[idx].start>= start_ && si[idx].end <= end_ ){
    			m_safeIntervals[location].erase(si + idx);
    			idx--;
    		}else if(si[idx].start >= start_ && si[idx].start <= end_ ){
    			si[idx].start = end_ + 1;
    		}else if(start_ <= si[idx].end && end_ <= si[idx]){
    			si[idx].end = start_ - 1;
    		}
    	}

    	return true;
    }
   private:
    const std::vector<interval>& safeIntervals(const Location& location) {
      static std::vector<interval> defaultInterval(
          1, {0, std::numeric_limits<Cost>::max()});
      const auto iter = m_safeIntervals.find(location);
      if (iter == m_safeIntervals.end()) {
        return defaultInterval;
      }
      return iter->second;
    }


   private:
    Environment& m_env;
    Cost m_lastGScore;
    std::unordered_map<Location, std::vector<interval> > m_safeIntervals;
    std::unordered_map<Location, std::vector<edgeCollision>> m_edgeCollision;
    std::vector<Neighbor<JPSSIPPState, Action, Cost> > jps_successors;
  };

 private:
  JPSSIPPEnvironment m_env;
  JPSAStar<JPSSIPPState, JPSSIPPAction, Cost, JPSSIPPEnvironment, JPSSIPPStateHasher, JPSSIPPStateHasherOpen> m_astar;
};

}  // namespace libMultiRobotPlanning
