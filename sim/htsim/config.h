// -*- c-basic-offset: 4; tab-width: 8; indent-tabs-mode: t -*-        
#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <sys/types.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

double drand();


#ifdef _WIN32
// Ways to refer to integer types
typedef unsigned __int64 uint64_t;
typedef unsigned __int32 uint32_t;
typedef unsigned __int16 uint16_t;
typedef signed __int64 sint64_t;
#else
typedef long long sint64_t;
#endif

// Specify units for simulation time, link speed, buffer capacity
typedef uint64_t simtime_picosec;

int pareto(int xm, int mean);
double exponential(double lambda);

constexpr simtime_picosec timeFromSec(double secs) {
    simtime_picosec psecs = (simtime_picosec)(secs * 1000000000000.0);
    return psecs;
}

constexpr simtime_picosec timeFromMs(double msecs) {
    simtime_picosec psecs = (simtime_picosec)(msecs * 1000000000);
    return psecs;
}

constexpr simtime_picosec timeFromMs(int msecs) {
    simtime_picosec psecs = (simtime_picosec)((uint64_t)msecs * 1000000000);
    return psecs;
}

constexpr simtime_picosec timeFromUs(double usecs) {
    simtime_picosec psecs = (simtime_picosec)(usecs * 1000000);
    return psecs;
}

constexpr simtime_picosec timeFromUs(uint32_t usecs) {
    simtime_picosec psecs = (simtime_picosec)((uint64_t)usecs * 1000000);
    return psecs;
}

constexpr simtime_picosec timeFromNs(double nsecs) {
    simtime_picosec psecs = (simtime_picosec)(nsecs * 1000);
    return psecs;
}

constexpr double timeAsMs(simtime_picosec ps) {
    double ms_ = (double)(ps / 1000000000.0);
    return ms_;
}

constexpr double timeAsUs(simtime_picosec ps) {
    double us_ = (double)(ps / 1000000.0);
    return us_;
}

constexpr double timeAsSec(simtime_picosec ps) {
    double s_ = (double)ps / 1000000000000.0;
    return s_;
}

typedef sint64_t mem_b;
mem_b memFromPkt(double pkts);

typedef uint64_t linkspeed_bps;

constexpr linkspeed_bps speedFromMbps(uint64_t Mbitps) {
    return Mbitps * 1000000;
}

constexpr linkspeed_bps speedFromMbps(double Mbitps) {
    return Mbitps*1000000;
}

constexpr linkspeed_bps speedFromKbps(uint64_t Kbitps) {
    return Kbitps * 1000;
}

linkspeed_bps speedFromPktps(double packetsPerSec);
double speedAsPktps(linkspeed_bps bps);
typedef int mem_pkts;

typedef uint32_t addr_t;
typedef uint16_t port_t;

// Gumph
#if defined (__cplusplus) && !defined(__STL_NO_NAMESPACES)
using namespace std;
#endif

#ifdef _WIN32
inline uint64_t max(uint64_t a, uint64_t b) {
    return a > b ? a : b;
}
inline sint64_t max(sint64_t a, sint64_t b) {
    return a > b ? a : b;
}
inline double max(double a, double b) {
    return a > b ? a : b;
}
inline size_t max(size_t a, size_t b) {
    return a > b ? a : b;
}
inline uint64_t min(uint64_t a, uint64_t b) {
    return a < b ? a : b;
}
inline sint64_t min(sint64_t a, sint64_t b) {
    return a < b ? a : b;
}
inline double min(double a, double b) {
    return a < b ? a : b;
}
inline size_t min(size_t a, size_t b) {
    return a < b ? a : b;
}
#endif

#endif
