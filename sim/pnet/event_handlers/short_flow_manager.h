#ifndef SHORT_FLOW_MANAGER_H
#define SHORT_FLOW_MANAGER_H

#include "config.h"
#include "eventlist.h"

#include "traffic_matrix.h"

class FluidSimulator;

// manages short flow restart based on its schedule
class ShortFlowManager : public EventSource {
public:
    ShortFlowManager(EventList& eventlist, simtime_picosec start_time, size_t repeat_count, simtime_picosec period,
                        FluidSimulator *simulator, const TrafficMatrix *traffic_matrix);
    void doNextEvent() override;

private:
    const size_t repeat_count;
    simtime_picosec period = 0;
    size_t current_repeat = 0;
    FluidSimulator *simulator;
    const TrafficMatrix *traffic_matrix;
};

#endif // SHORT_FLOW_MANAGER_H
