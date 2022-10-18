#include "short_flow_manager.h"

#include "pnet_simulator.h"

ShortFlowManager::ShortFlowManager(EventList& eventlist, simtime_picosec start_time, size_t repeat_count, simtime_picosec period,
                                   FluidSimulator *simulator, const TrafficMatrix *traffic_matrix)
        : EventSource(eventlist, "ShortFlowManager.TM[" + std::to_string(traffic_matrix->GetId()) + "]"),
          repeat_count(repeat_count), period(period), simulator(simulator), traffic_matrix(traffic_matrix) {
    eventlist.sourceIsPending(*this, start_time + period);
}

void ShortFlowManager::doNextEvent() {
    this->current_repeat++;
    if (this->current_repeat > this->repeat_count)
        return;

    eventlist().sourceIsPendingRel(*this, period);
    double now_ms = timeAsMs(eventlist().now());
    std::cerr << "Restarting TM[" << this->traffic_matrix->GetId() << "] for the " << this->current_repeat << "-th time ..." << std::endl;
    std::cout << "TM[" << this->traffic_matrix->GetId() << "] " << this->current_repeat << "-th restart at " << now_ms << "ms." << std::endl;
    size_t newflows = this->simulator->StartTrafficMatrix(this->traffic_matrix, this->current_repeat);
    if (newflows == 0) {
        // Some flows are not finished, wait till next time period.
        this->current_repeat--;
    }
}
