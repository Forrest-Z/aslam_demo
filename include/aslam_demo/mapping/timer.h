#ifndef TIMER_H
#define TIMER_H

// Use the new Boost timers if version is recent enough
#include <boost/version.hpp>
#if BOOST_VERSION >= 104800
#include <boost/timer/timer.hpp>
#else
#include <boost/timer.hpp>
#endif

namespace mapping {

/**
 * A class that wraps a Boost Timer to assist code profiling. This class uses the new v2 Boost Timers, when available.
 */
class Timer {
public:
  Timer();
  virtual ~Timer();

  void start();
  void stop();
  double elapsed() const;

private:

#if BOOST_VERSION >= 104800
      boost::timer::cpu_timer boost_timer_; ///< A boost timer object. This actually does the work.
#else
      boost::timer boost_timer_; ///< A boost timer object. This actually does the work.
#endif
double elapsed_;

};

} // namespace: mapping

#endif // TIMER_H

