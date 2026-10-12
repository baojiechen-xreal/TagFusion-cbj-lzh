#include "timeutil.h"

#include <iostream>
#include <time.h>

namespace g2o {

ScopeTime::ScopeTime(const char* title) : _title(title), _startTime(get_monotonic_time()) {}

ScopeTime::~ScopeTime() {
  std::cerr << _title << " took " << 1000. * (get_monotonic_time() - _startTime) << "ms.\n";
}

double get_monotonic_time()
{
#if (defined(_POSIX_TIMERS) && (_POSIX_TIMERS+0 >= 0) && defined(_POSIX_MONOTONIC_CLOCK))
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
#else
  return get_time();
#endif
}

}  // namespace g2o
