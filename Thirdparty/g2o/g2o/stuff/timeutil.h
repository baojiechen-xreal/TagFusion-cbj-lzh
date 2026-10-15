// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.

#ifndef G2O_TIMEUTIL_H
#define G2O_TIMEUTIL_H

#ifdef _WINDOWS
#include <time.h>
#else
#include <sys/time.h>
#endif

#include <string>

/** @addtogroup utils **/
// @{

/** \file timeutil.h
 * \brief utility functions for handling time
 */

/// Executes code, only if secs are gone since last exec.
#ifndef DO_EVERY_TS
#define DO_EVERY_TS(secs, currentTime, code) \
if (1) {\
  static double s_lastDone_ = (currentTime); \
  double s_now_ = (currentTime); \
  if (s_lastDone_ > s_now_) \
    s_lastDone_ = s_now_; \
  if (s_now_ - s_lastDone_ > (secs)) { \
    code; \
    s_lastDone_ = s_now_; \
  }\
} else \
  (void)0
#endif

/// Executes code, only if secs are gone since last exec.
#ifndef DO_EVERY
#define DO_EVERY(secs, code) DO_EVERY_TS(secs, g2o::get_time(), code)
#endif

#ifndef MEASURE_TIME
#define MEASURE_TIME(text, code) \
  if(1) { \
    double _start_time_ = g2o::get_time(); \
    code; \
    fprintf(stderr, "%s took %f sec\n", text, g2o::get_time() - _start_time_); \
  } else \
    (void) 0
#endif

namespace g2o {

#ifdef _WINDOWS
inline double get_time()
{
  return 0.0;
}
#else
inline double get_time()
{
  struct timeval ts;
  gettimeofday(&ts, 0);
  return ts.tv_sec + ts.tv_usec * 1e-6;
}
#endif

double get_monotonic_time();

class ScopeTime {
 public:
  ScopeTime(const char* title);
  ~ScopeTime();
 private:
  std::string _title;
  double _startTime;
};

}  // namespace g2o

#ifndef MEASURE_FUNCTION_TIME
#define MEASURE_FUNCTION_TIME \
  g2o::ScopeTime scopeTime(__PRETTY_FUNCTION__)
#endif

// @}
#endif
