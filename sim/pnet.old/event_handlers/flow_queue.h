#ifndef TASK_QUEUE_H
#define TASK_QUEUE_H

#include <cstddef>
#include <limits>
#include <vector>

#include "eventlist.h"
#include "tcp.h"
#include "mtcp.h"
#include "network.h"

#include "flowset.h"
#include "stop_logger.h"
#include "utils.h"

// Represent a queue that allows a fixed number of flows to run at the same time.
class FlowQueue : public EventSource {
public:
    // Initialize a flow queue that allows @concurrency # of flows to run at the same time.
    FlowQueue(EventList& eventlist, const std::string &name, StopLogger* stop_logger, size_t concurrency = std::numeric_limits<size_t>::max());

    // Change flow queue to be multipath, aka use MultipathTcpSrc instead of TcpSrc when adding flow src.
    void SetMultipath();

    // Add a flow src. Must be of type MultipathTcpSrc or TcpSrc.
    void AddFlowSrc(PacketSink *src);

    // Triggers when a flow finishes
    void doNextEvent() override;

    // Start the initial windows of flows at the specified
    void StartFlows(simtime_picosec start_time);

    // Sets flow dependency based on each flow's parent index.
    void SetDependency(std::vector<ssize_t> parent_indices);

private:
    StopLogger* const stop_logger_ = nullptr;
    const size_t concurrency_ = std::numeric_limits<size_t>::max();
    bool is_multipath_ = false;
    size_t flow_count_ = 0;
    size_t completed_count_ = 0;
    size_t released_count_ = 0;

    std::vector<PacketSink *> flow_srcs_;
    std::vector<ssize_t> parent_indices_;
    std::vector<bool> flow_released_;
    std::vector<bool> flow_completed_;

    // start one flow from the top/front of the queue, returns whether successful.
    bool StartOneFlow(simtime_picosec start_time = 0);

    // Find the first flow that's not yet released and not depending on a running flow.
    ssize_t FindFirstFlowToRelease();
};

#endif  // TASK_QUEUE_H
