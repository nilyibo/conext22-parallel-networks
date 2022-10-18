#ifndef FIRST_FIT_H
#define FIRST_FIT_H

#include "tcp.h"
#include "randomqueue.h"
#include "eventlist.h"
#include <list>
#include <map>

// Note: this implementation is wrong, not based on fraction of BW used per src/dst host.

struct flow_entry {
    int byte_counter;
    int allocated;
    int src;
    int dest;
    const Route* route;

    flow_entry(int bc, int a,int s, int d, const Route* r){
        byte_counter = bc;
        allocated = a;
        src = s;
        dest = d;
        route = r;
    }
};

class FirstFit: public EventSource{
 public:
    FirstFit(simtime_picosec scanPeriod, EventList& eventlist, size_t host_link_speed_mbps, vector<const Route*>*** np = NULL);
    void doNextEvent();

    void run();
    void add_flow(int src,int dest,TcpSrc* flow);
    void add_queue(Queue* queue);
    vector<const Route*>*** net_paths;

 private:
    map<TcpSrc*,flow_entry*> flow_counters;
    //TcpSrc* connections[N][N];
    map<Queue*,int> path_allocations;
    simtime_picosec _scanPeriod;
    //int _init;
    int threshold;
};

#endif
