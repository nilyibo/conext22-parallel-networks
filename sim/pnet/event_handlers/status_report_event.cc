#include "status_report_event.h"

#include <iostream>

StatusReportEvent::StatusReportEvent(EventList& eventlist, simtime_picosec period, StopLogger *stop_logger, size_t flow_count, bool loop_flows) : EventSource(eventlist, "StatusReport") {
    this->period = period;
    this->stop_logger = stop_logger;
    this->flow_count = flow_count;
    this->last_completed_count = 0;
    this->loop_flows = loop_flows;
    if (flow_count > 0) {
        eventlist.sourceIsPendingRel(*this, period);
        std::cerr << "Waiting for " << this->flow_count << " short flows to complete ..." << std::endl;
    }   // othewise, unused
}

void StatusReportEvent::doNextEvent() {
    size_t completed_count = this->stop_logger->GetCompletedCount();
    if (completed_count > this->last_completed_count) {
        std::cerr << "At " << timeAsMs(eventlist().now()) << "ms, "
                  << completed_count << "/" << flow_count << " short flows completed." << std::endl;
        this->last_completed_count = completed_count;
    }

    if (completed_count < flow_count || loop_flows)
        eventlist().sourceIsPendingRel(*this, period);
    else
        eventlist().setEndtime(eventlist().now());
}
