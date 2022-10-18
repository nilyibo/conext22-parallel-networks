#include "stop_logger.h"

StopLogger::StopLogger(EventList& eventlist, const string& name) : EventSource(eventlist, name), completed_count(0) {}

void StopLogger::doNextEvent() {
    //nothing to do, just prevent flow restarting
    this->completed_count++;
}

size_t StopLogger::GetCompletedCount() const {
    return this->completed_count;
}
