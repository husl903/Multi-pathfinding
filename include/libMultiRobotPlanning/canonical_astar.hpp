#pragma once

#ifdef USE_FIBONACCI_HEAP
#include <boost/heap/fibonacci_heap.hpp>
#endif



#include <unordered_map>
#include <unordered_set>
#include "castar.hpp"
#include "constants.hpp"


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
class CANAstar{
 public:

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
  CANAstar(Environment& environment) : m_env(environment), m_astar(m_env) {}

  void setCollisionVertex(const Location& location, int startTime, int EndTime, bool is_first){
	  m_env.setCollisionVertex(location, startTime, EndTime, is_first);
  }

  void clearObstacle(const Location& location){
	  m_env.clearObstacle(location);
  }

  void sortCollisionVertex(){
	  m_env.sortCollisionVertex();
  }

  void sortCollisionEdgeConstraint(){
	  m_env.sortCollisionEdgeConstraint();
  }

  void setEdgeConstraint(const Location& location, int time, Action ac, bool is_first){
	  m_env.setEdgeConstraint(location, time, ac, is_first);
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

  bool search(const State& startState,
              PlanResult<State, Action, Cost>& solution, Cost startTime = 0) {
    PlanResult<State, Action, Cost> astarsolution;
    solution.cost = 0;
    solution.fmin = 0;
    solution.actions.clear();
    solution.states.clear();
    bool success = m_astar.search(startState, astarsolution, startTime);
    // if(!success){
    //   std::cout << "cannoical Not successful \n";
    // }
    solution = astarsolution;
    solution.cost = astarsolution.cost - startTime;
    solution.fmin = astarsolution.fmin;
    return success;
  }

 private:
  // public:


  struct StateHasher {
    size_t operator()(const State& s) const {
      size_t seed = 0;
      boost::hash_combine(seed, std::hash<State>()(s));
      return seed;
    }
  };


  // private:
  struct CANAstarEnvironment {
    CANAstarEnvironment(Environment& env) : m_env(env) {}

    Cost admissibleHeuristic(const State& s) {
        return m_env.admissibleHeuristic(s);
    }

    bool isSolution(const State& s) {

      return m_env.isSolution(s);
    }

    void getNeighbors(
            const State& s,
             std::vector<Neighbor<State, Action, Cost> >& neighbors) {
      bool is_debug_f = false;
      if(is_debug_f) std::cout << "EXPANDED current state: " << s.x << ", " << s.y  << " time "  << s.time  << " dir " << s.dir << " ----\n";
      neighbors.clear();
      if((s.dir & 0x01) && m_env.stateValid(State(s.time + 1, s.x - 1, s.y)) ){
        State n(s.time + 1, s.x - 1, s.y);
        if(m_env.transitionValid(s, n)){
          n.dir = 0x01;
          if(m_env.isTemporalObstacleAtT(Location(s.x, s.y), s.time + 1) 
             && m_env.stateValid(State(s.time + 2, s.x, s.y))){
            n.dir |= 0x02;
          }
          if(m_env.stateValid(State(s.time + 2, s.x - 1, s.y + 1))){
            if(m_env.isObstacle(Location(s.x, s.y + 1))
               || m_env.isTemporalObstacleAtT(Location(s.x, s.y + 1), s.time + 1)
               || m_env.isTemporalObstacleAtT(Location(s.x - 1, s.y + 1), s.time + 1)
               || m_env.isEdgeConstraintAtT(Location(s.x, s.y + 1), Location(s.x - 1, s.y + 1), s.time + 1)
               || m_env.isEdgeConstraintAtT(Location(s.x, s.y), Location(s.x, s.y + 1), s.time)){
              n.dir |= 0x04;
            }
          }
          if(m_env.stateValid(State(s.time + 2, s.x - 1, s.y - 1))){
            if(m_env.isObstacle(Location(s.x, s.y - 1))
               || m_env.isTemporalObstacleAtT(Location(s.x, s.y - 1), s.time + 1)
               || m_env.isTemporalObstacleAtT(Location(s.x - 1, s.y - 1), s.time + 1)
               || m_env.isEdgeConstraintAtT(Location(s.x, s.y - 1), Location(s.x - 1, s.y - 1), s.time + 1)
               || m_env.isEdgeConstraintAtT(Location(s.x, s.y), Location(s.x, s.y - 1), s.time)){
              n.dir |= 0x08;
            }
          }
          if(s.dir & 0xff) n.dir_p = s.dir;
          else n.dir_p = s.dir_p;
          neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Left, 1));
          if(is_debug_f) std::cout << "Successor " << n.x << ", " << n.y << ", time " << n.time << ", dir " << n.dir << " \n";
        }
      }
        
      if((s.dir & 0x02) && m_env.stateValid(State(s.time + 1, s.x + 1, s.y)) ){
        State n(s.time + 1, s.x + 1, s.y);
        if(m_env.transitionValid(s, n)){
          n.dir = 0x02;
          if(m_env.isTemporalObstacleAtT(Location(s.x, s.y), s.time + 1) 
             && m_env.stateValid(State(s.time + 2, s.x, s.y))){
            n.dir |= 0x01;
          }
          if(m_env.stateValid(State(s.time + 2, s.x + 1, s.y + 1))){
            if(m_env.isObstacle(Location(s.x, s.y + 1))
               || m_env.isTemporalObstacleAtT(Location(s.x, s.y + 1), s.time + 1)
               || m_env.isTemporalObstacleAtT(Location(s.x + 1, s.y + 1), s.time + 1)
               || m_env.isEdgeConstraintAtT(Location(s.x, s.y + 1), Location(s.x + 1, s.y + 1), s.time + 1)
               || m_env.isEdgeConstraintAtT(Location(s.x, s.y), Location(s.x, s.y + 1), s.time)){
              n.dir |= 0x04;
            }
          }
          if(m_env.stateValid(State(s.time + 2, s.x + 1, s.y - 1))){
            if(m_env.isObstacle(Location(s.x, s.y - 1))
               || m_env.isTemporalObstacleAtT(Location(s.x, s.y - 1), s.time + 1)
               || m_env.isTemporalObstacleAtT(Location(s.x + 1, s.y - 1), s.time + 1)
               || m_env.isEdgeConstraintAtT(Location(s.x, s.y - 1), Location(s.x + 1, s.y - 1), s.time + 1)
               || m_env.isEdgeConstraintAtT(Location(s.x, s.y), Location(s.x, s.y - 1), s.time)){
              n.dir |= 0x08;
            }
          }
          if(s.dir & 0xff) n.dir_p = s.dir;
          else n.dir_p = s.dir_p;
          neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Right, 1));
            if(is_debug_f) std::cout << "Successor " << n.x << ", " << n.y << ", time " << n.time << ", dir " << n.dir << " \n";
        }        
      }

      if((s.dir & 0x04) && m_env.stateValid(State(s.time + 1, s.x, s.y + 1)) ){
        
        State n(s.time + 1, s.x, s.y + 1);
        if(m_env.transitionValid(s, n)){
          n.dir = 0x07;
          if(m_env.isTemporalObstacleAtT(Location(s.x, s.y), s.time + 1) 
             && m_env.stateValid(State(s.time + 2, s.x, s.y))){
            n.dir |= 0x08;
          }
          if(s.dir & 0xff) n.dir_p = s.dir;
          else n.dir_p = s.dir_p;
          neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Up, 1));
          if(is_debug_f) std::cout << "Successor " << n.x << ", " << n.y << ", time " << n.time << ", dir " << n.dir << " \n";
        }        
      }

      if((s.dir & 0x08) && m_env.stateValid(State(s.time + 1, s.x, s.y - 1)) ){
        State n(s.time + 1, s.x, s.y - 1);
        if(m_env.transitionValid(s, n)){
          n.dir = 0x0b;
          if(m_env.isTemporalObstacleAtT(Location(s.x, s.y), s.time + 1) 
             && m_env.stateValid(State(s.time + 2, s.x, s.y))){
            n.dir |= 0x04;
          }
          if(s.dir & 0xff) n.dir_p = s.dir;
          else n.dir_p = s.dir_p;
          neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Down, 1));
          if(is_debug_f) std::cout << "Successor " << n.x << ", " << n.y << ", time " << n.time << ", dir " << n.dir << " \n";
        }        
      }

      if(m_env.stateValid(State(s.time + 1, s.x, s.y))){
        if(m_env.isTemporalEdgeConstraint(Location(s.x + 1, s.y)) || m_env.isTemporalEdgeConstraint(Location(s.x - 1, s.y))
        	|| m_env.isTemporalEdgeConstraint(Location(s.x, s.y - 1)) || m_env.isTemporalEdgeConstraint(Location(s.x, s.y + 1))
    			|| m_env.isTemporalEdgeConstraint(Location(s.x + 1, s.y - 1)) || m_env.isTemporalEdgeConstraint(Location(s.x - 1, s.y - 1))
    			|| m_env.isTemporalEdgeConstraint(Location(s.x + 1, s.y + 1)) || m_env.isTemporalEdgeConstraint(Location(s.x - 1, s.y + 1))
    			|| m_env.isTemporalObstacle(Location(s.x + 1, s.y)) || m_env.isTemporalObstacle(Location(s.x - 1, s.y))
    			|| m_env.isTemporalObstacle(Location(s.x, s.y - 1)) || m_env.isTemporalObstacle(Location(s.x, s.y + 1))
    			|| m_env.isTemporalObstacle(Location(s.x + 1, s.y - 1)) || m_env.isTemporalObstacle(Location(s.x - 1, s.y - 1))
    			|| m_env.isTemporalObstacle(Location(s.x + 1, s.y + 1)) || m_env.isTemporalObstacle(Location(s.x - 1, s.y + 1))
          || m_env.isTemporalObstacle(Location(s.x, s.y)) || m_env.isTemporalEdgeConstraint(Location(s.x, s.y)))
        {
          State n(s.time + 1, s.x, s.y);
          n.dir = 0x00;
          Location n_left(s.x - 1, s.y);

          if(m_env.isTemporalObstacleAtT(n_left, s.time + 1) 
             || m_env.isEdgeConstraintAtT(Location(s.x, s.y), Location(s.x - 1, s.y), s.time)){
             if (m_env.stateValid(State(s.time + 2, s.x - 1, s.y))) 
             n.dir |= 0x01;
          }

          Location n_right(s.x + 1, s.y);
          if(m_env.isTemporalObstacleAtT(n_right, s.time + 1)
             || m_env.isEdgeConstraintAtT(Location(s.x, s.y), Location(s.x + 1, s.y), s.time)){
            if(m_env.stateValid(State(s.time + 2, s.x + 1, s.y))) 
            n.dir |= 0x02;
          }
          Location n_up(s.x, s.y + 1);
          if(m_env.stateValid(State(s.time + 2, s.x , s.y + 1)) &&
             (m_env.isTemporalObstacleAtT(n_up, s.time + 1)
             || m_env.isEdgeConstraintAtT(Location(s.x, s.y), Location(s.x, s.y + 1), s.time)
             || (m_env.isTemporalObstacleAtT(Location(s.x + 1, s.y + 1), s.time + 1) && (s.dir & 0x01))
             || (m_env.isEdgeConstraintAtT(Location(s.x + 1, s.y + 1), Location(s.x + 1, s.y), s.time + 1) && (s.dir & 0x01))
             || (m_env.isEdgeConstraintAtT(Location(s.x + 1, s.y + 1), Location(s.x, s.y + 1), s.time + 1) && (s.dir & 0x01))
             || (m_env.isTemporalObstacleAtT(Location(s.x - 1, s.y + 1), s.time + 1) && (s.dir & 0x02))
             || (m_env.isEdgeConstraintAtT(Location(s.x - 1, s.y + 1), Location(s.x - 1, s.y), s.time + 1) && (s.dir & 0x02))
             || (m_env.isEdgeConstraintAtT(Location(s.x - 1, s.y + 1), Location(s.x, s.y + 1), s.time + 1) && (s.dir & 0x02))             
              )){
            n.dir |= 0x04;
          }
          Location n_down(s.x, s.y - 1);
          if(m_env.stateValid(State(s.time + 2, s.x , s.y - 1)) &&
             (m_env.isTemporalObstacleAtT(n_down, s.time + 1)
             || m_env.isEdgeConstraintAtT(Location(s.x, s.y), Location(s.x, s.y - 1), s.time)
             || (m_env.isTemporalObstacleAtT(Location(s.x + 1, s.y - 1), s.time + 1) && (s.dir & 0x01))
             || (m_env.isEdgeConstraintAtT(Location(s.x + 1, s.y - 1), Location(s.x + 1, s.y), s.time + 1) && (s.dir & 0x01))
             || (m_env.isEdgeConstraintAtT(Location(s.x + 1, s.y - 1), Location(s.x, s.y - 1), s.time + 1) && (s.dir & 0x01))
             || (m_env.isTemporalObstacleAtT(Location(s.x - 1, s.y - 1), s.time + 1) && (s.dir & 0x02))
             || (m_env.isEdgeConstraintAtT(Location(s.x - 1, s.y - 1), Location(s.x - 1, s.y), s.time + 1) && (s.dir & 0x02))
             || (m_env.isEdgeConstraintAtT(Location(s.x - 1, s.y - 1), Location(s.x, s.y - 1), s.time + 1) && (s.dir & 0x02))
              )){
            n.dir |= 0x08;
          }
          if(s.dir & 0xff) n.dir_p = s.dir;
          else n.dir_p = s.dir_p;      
          neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Wait, 1));
          if(is_debug_f) std::cout << "Successor " << n.x << ", " << n.y << ", time " << n.time << ", dir " << n.dir << " \n";
        }
      }

//      m_env.getNeighbors(s, neighbors);
    }

    void onExpandNode(const State& s, Cost fScore, Cost gScore) {
    	m_env.num_expansion++;
    	m_lastGScore = gScore;
      Location loc(s.x, s.y);
        m_env.onExpandNode(loc, fScore, gScore);
    }

    void onDiscover(const State& s, Cost fScore, Cost gScore) {
      Location loc(s.x, s.y);
    	m_env.onDiscover(loc, fScore, gScore);
    }


   private:
    Environment& m_env;
    Cost m_lastGScore;
    std::vector<Neighbor<State, Action, Cost>> jps_successors;
    bool flag_is_solution = false;

  };

 private:
  CANAstarEnvironment m_env;
  CAStar<State, Action, Cost, CANAstarEnvironment, StateHasher> m_astar;
};

}  // namespace libMultiRobotPlanning
