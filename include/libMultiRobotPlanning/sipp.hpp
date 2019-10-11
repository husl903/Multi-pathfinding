#pragma once

#ifdef USE_FIBONACCI_HEAP
#include <boost/heap/fibonacci_heap.hpp>
#endif

#include <boost/functional/hash.hpp>
#include <boost/heap/d_ary_heap.hpp>

#include <unordered_map>
#include <unordered_set>

// #include "neighbor.hpp"
// #include "planresult.hpp"
#include "a_star.hpp"

namespace libMultiRobotPlanning {

/*!
  \example sipp.cpp Simple example using a 2D grid world and
  up/down/left/right
  actions
*/

/*! \brief SIPP Algorithm to find the shortest path with dynamic obstacles

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
class SIPP {
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
  SIPP(Environment& environment) : m_env(environment), m_astar(m_env) {}

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
    PlanResult<SIPPState, SIPPAction, Cost> astarsolution;
    solution.cost = 0;
    solution.fmin = 0;
    solution.actions.clear();
    solution.states.clear();
    size_t interval;
    if (!m_env.findSafeInterval(startState, startTime, interval)) {
      return false;
    }
    bool success = m_astar.search(SIPPState(startState, interval),
                                  astarsolution, startTime);
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
  struct SIPPState {
    SIPPState(const State& state, size_t interval)
        : state(state), interval(interval) {}
    SIPPState(const State& state, size_t interval, unsigned int dir)
        : state(state), interval(interval), dir(dir) {}

    bool operator==(const SIPPState& other) const {
      return std::tie(state, interval) == std::tie(other.state, other.interval);
    }

    friend std::ostream& operator<<(std::ostream& os, const SIPPState& s) {
      return os << "(" << s.state << "," << s.interval << ")";
    }

    State state;
    unsigned int dir;
    size_t interval;
  };

  struct SIPPStateHasher {
    size_t operator()(const SIPPState& s) const {
      size_t seed = 0;
      boost::hash_combine(seed, std::hash<State>()(s.state));
      boost::hash_combine(seed, s.interval);
      return seed;
    }
  };

  struct SIPPAction {
    SIPPAction(const Action& action, Cost time) : action(action), time(time) {}

    Action action;
    Cost time;
  };

  // private:
  struct SIPPEnvironment {
    SIPPEnvironment(Environment& env) : m_env(env) {}

    Cost admissibleHeuristic(const SIPPState& s) {
      return m_env.admissibleHeuristic(s.state);
    }

    bool mightHaveSolution(const State& goal) {
      const auto& si = safeIntervals(m_env.getLocation(goal));
      return m_env.isSolution(goal) && !si.empty() &&
             si.back().end == std::numeric_limits<Cost>::max();
    }

    bool isSolution(const SIPPState& s) {
      return m_env.isSolution(s.state) &&
             safeIntervals(m_env.getLocation(s.state)).at(s.interval).end ==
                 std::numeric_limits<Cost>::max();
    }

    void getNeighbors(
        const SIPPState& s,
        std::vector<Neighbor<SIPPState, SIPPAction, Cost> >& neighbors) {
      std::vector<Neighbor<State, Action, Cost> > motions;
// 	  std::cout << "Current state ---------------------------------------GScore " << m_lastGScore << " " << s.state.x << " " << s.state.y << " dir " << s.dir << " \n";


      m_env.getNeighbors(s.state, motions);
      for (const auto& m : motions) {
    	  m_env.num_generation++;
        // std::cout << "gN " << m.state << std::endl;
        Cost m_time = m.cost;
        // std::cout << m_lastGScore;
        Cost start_t = m_lastGScore + m_time;
        Cost end_t =
            safeIntervals(m_env.getLocation(s.state)).at(s.interval).end;

        const auto& sis = safeIntervals(m_env.getLocation(m.state));
        for (size_t i = 0; i < sis.size(); ++i) {
          const interval& si = sis[i];
//          std::cout << m.state.x << " " << m.state.y << "  i " << i << ": " << si.start << " , " << si.end << " " << start_t << " " << end_t<< " \n";
          // std::endl;
          if (si.start - m_time > end_t || si.end < start_t) {
            continue;
          }
//          std::cout << " --------------\n";
          int t;
          unsigned int dir_1 = 0x00;
          Action a_temp;
          if(m.action == Action::Left) {a_temp = Action::Right;dir_1 = 0x01;}
          else if(m.action == Action::Right) {a_temp = Action::Left; dir_1 = 0x02;}
          else if(m.action == Action::Up) {a_temp = Action::Down;dir_1 = 0x04;}
          else if(m.action == Action::Down){ a_temp = Action::Up; dir_1 = 0x08;}

          if (m_env.isCommandValid(s.state, m.state, m.action, m_lastGScore,
                                   end_t, si.start, si.end, t)
        		  && !IsEdgeCollisions(m.state, edgeCollision(t - 1, a_temp))) {
            // std::cout << "  gN: " << m.state << "," << i << "," << t << ","
            // << m_lastGScore << std::endl;
            neighbors.emplace_back(Neighbor<SIPPState, SIPPAction, Cost>(
                SIPPState(m.state, i, dir_1), SIPPAction(m.action, m.cost),
                t - m_lastGScore));
//               std::cout << "Successor : " << m.state.x << " " << m.state.y <<" Cost " << m.cost  << " dir " << dir_1 << " Gscore " << t << " \n";
          }
        }
      }
    }

    void onExpandNode(const SIPPState& s, Cost fScore, Cost gScore) {
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

    void onDiscover(const SIPPState& s, Cost fScore, Cost gScore) {
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
        m_safeIntervals[location];  // create empty safe interval
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

      // auto iter = m_safeIntervals.find(location);
      // if (iter != m_safeIntervals.end()) {
      //   for (const auto& si : iter->second) {
      //     std::cout << "  si: " << si.start << " - " << si.end << std::endl;
      //   }
      // }
    }

    bool findSafeInterval(const State& state, Cost time, size_t& interval) {
      const auto& si = safeIntervals(m_env.getLocation(state));
      for (size_t idx = 0; idx < si.size(); ++idx) {
        if (si[idx].start <= time && si[idx].end >= time) {
          interval = idx;
          return true;
        }
      }
      return false;
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
  };

 private:
  SIPPEnvironment m_env;
  AStar<SIPPState, SIPPAction, Cost, SIPPEnvironment, SIPPStateHasher> m_astar;
};

}  // namespace libMultiRobotPlanning
