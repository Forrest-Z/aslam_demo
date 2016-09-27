
#include <aslam_demo/mapping/timer.h>

namespace bnr_mapping {

/* ************************************************************************* */
Timer::Timer() : elapsed_(0.0) {
}

/* ************************************************************************* */
Timer::~Timer() {
}

/* ************************************************************************* */
void Timer::start() {
#if BOOST_VERSION >= 104800
  boost_timer_.start();
#else
  boost_timer_.restart();
#endif
}

/* ************************************************************************* */
void Timer::stop() {
#if BOOST_VERSION >= 104800
  boost_timer_.stop();
  elapsed_ = double(boost_timer_.elapsed().wall) * 1.0e-9;
#else
  elapsed_ = boost_timer_.elapsed();
#endif
}

/* ************************************************************************* */
double Timer::elapsed() const {
  return elapsed_;
}

/* ************************************************************************* */
} // namespace: bnr_mapping

