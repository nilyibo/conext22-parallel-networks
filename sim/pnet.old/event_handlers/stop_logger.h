#ifndef STOP_LOGGER_H
#define STOP_LOGGER_H

#include <string>

#include "eventlist.h"

// Stops a short flow, preventing it from auto-restarting (based on current TcpSrcTransfer implementation).
class StopLogger : public EventSource {
public:
    StopLogger(EventList& eventlist, const string& name);
    void doNextEvent() override;
    size_t GetCompletedCount() const;

private:
    size_t completed_count;
};

#endif // STOP_LOGGER_H
