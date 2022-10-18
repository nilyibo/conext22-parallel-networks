#ifndef STATUS_REPORT_EVENT_H
#define STATUS_REPORT_EVENT_H

#include "config.h"
#include "eventlist.h"

#include "stop_logger.h"

// Report flow completion status.
class StatusReportEvent : public EventSource {
public:
    StatusReportEvent(EventList& eventlist, simtime_picosec period, StopLogger *stop_logger, size_t flow_count);
    void doNextEvent() override;

private:
    simtime_picosec period;
    StopLogger *stop_logger;
    size_t flow_count;
    size_t last_completed_count;
};

#endif // STATUS_REPORT_EVENT_H
