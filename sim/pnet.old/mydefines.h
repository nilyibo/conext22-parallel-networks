#ifndef MY_DEFINES_H
#define MY_DEFINES_H

// Whether to run server-level (or switch-level) analysis
#ifndef SIM_INCLUDE_SERVERS
#define SIM_INCLUDE_SERVERS 1
#endif

// Whether to prioritize fair sharing or maximum throughput
#ifndef SIM_LP_FAIR_SHARING
#define SIM_LP_FAIR_SHARING 1
#endif

// Whether to use first fit
#ifndef USE_FIRST_FIT
#define USE_FIRST_FIT 0
#endif

#ifndef USE_CONSTANT_SEED
#define USE_CONSTANT_SEED 0
#endif

#if USE_FIRST_FIT
#include "first_fit.h"
#endif

#ifndef LOG_LEVEL_VERBOSE
#define LOG_LEVEL_VERBOSE 1
#endif

#endif // MY_DEFINES_H
