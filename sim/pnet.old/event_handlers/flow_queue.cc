#include "flow_queue.h"

#include "absl/strings/str_format.h"

FlowQueue::FlowQueue(EventList& eventlist, const std::string &name, StopLogger* stop_logger, size_t concurrency)
        :EventSource(eventlist, name), stop_logger_(stop_logger), concurrency_(concurrency) {}

// Change flow queue to be multipath, aka use MultipathTcpSrc instead of TcpSrc when adding flow src.
void FlowQueue::SetMultipath() {
    this->is_multipath_ = true;
}

// Add a flow src. Must be of type MultipathTcpSrc or TcpSrc.
void FlowQueue::AddFlowSrc(PacketSink *src) {
    Logged *logged = nullptr;
    if (src != nullptr) {
        if (this->is_multipath_) {
            CHECK(dynamic_cast<MultipathTcpSrc *>(src) != nullptr, "Flow src has the wrong type.");
            logged = dynamic_cast<MultipathTcpSrc *>(src);
        } else {
            CHECK(dynamic_cast<TcpSrc *>(src) != nullptr, "Flow src has the wrong type.");
            logged = dynamic_cast<TcpSrc *>(src);
        }
#if LOG_LEVEL_VERBOSE
        std::cout << absl::StrFormat("[%s] Adding flow src %s with ID %zu.", _name, logged->_name, logged->id) << std::endl;
#endif
    }

    flow_srcs_.push_back(src);
    flow_released_.push_back(src == nullptr);
    flow_completed_.push_back(src == nullptr);
    flow_count_++;
    if (src == nullptr) {
        completed_count_++;
        released_count_++;
    }
}

// Triggers when a flow finishes
void FlowQueue::doNextEvent() {
    completed_count_++;
    if (stop_logger_ != nullptr)
        stop_logger_->doNextEvent();

    if (released_count_ < flow_count_) {
        bool has_flow_to_release = true;
        while (released_count_ - completed_count_ < concurrency_ && has_flow_to_release) {
            std::cerr << absl::StrFormat("[%s] Releasing next flow in queue ...", _name) << std::endl;
            has_flow_to_release = StartOneFlow();
        }
        return;
    }

    std::cerr << absl::StrFormat("[%s] No flows left in queue.", _name) << std::endl;
    CHECK(completed_count_ <= flow_count_, "Completed count cannot be larger than total flow count");
    if (completed_count_ == flow_count_)
        std::cout << absl::StrFormat("[%s] Flowset completed at %fms", _name, timeAsMs(eventlist().now())) << std::endl;
}

void FlowQueue::StartFlows(simtime_picosec start_time) {
#if LOG_LEVEL_VERBOSE
    std::cout << absl::StrFormat("[%s] Flowset starts at %fms", _name, timeAsMs(start_time)) << std::endl;
#endif
    size_t num_flows_to_release = min(this->concurrency_, this->flow_srcs_.size());
    for (size_t i = 0; i < num_flows_to_release; i++) {
        StartOneFlow(start_time);
    }
}

void FlowQueue::SetDependency(std::vector<ssize_t> parent_indices) {
    parent_indices_ = parent_indices;
    CHECK(parent_indices_.size() == flow_srcs_.size(), "Inconsistent size for parent_indices and flow_srcs_.");
}

// start one flow from the top/front of the queue, returns whether successful.
bool FlowQueue::StartOneFlow(simtime_picosec start_time) {
    auto now = eventlist().now();
    if (start_time < now) start_time = now;

    ssize_t ready_flow_index = FindFirstFlowToRelease();
    if (ready_flow_index == -1) {
        std::cerr << absl::StrFormat("[%s] No flow available to release.", _name) << std::endl;
        return false;
    }

    auto *src = flow_srcs_.at(ready_flow_index);
    flow_released_.at(ready_flow_index) = true;
    released_count_++;
    std::string tcp_src_name = "";
    if (this->is_multipath_) {
        auto *mptcp_src = dynamic_cast<MultipathTcpSrc *>(src);
        mptcp_src->resetStartTime(start_time);
        tcp_src_name = mptcp_src->_name;
    } else {
        auto *tcp_src = dynamic_cast<TcpSrc *>(src);
        tcp_src->resetStartTime(start_time);
        tcp_src_name = tcp_src->_name;
    }
#if LOG_LEVEL_VERBOSE
    std::cerr << absl::StrFormat("[%s] Starting TCP flow %s at %fms", _name, tcp_src_name, timeAsMs(start_time)) << std::endl;
#endif
    return true;
}

ssize_t FlowQueue::FindFirstFlowToRelease() {
    for (size_t i = 0; i < flow_count_; i++) {
        if (flow_srcs_.at(i) == nullptr) continue;
        flow_completed_.at(i) = is_multipath_ ?
                                dynamic_cast<MultipathTcpSrc *>(flow_srcs_.at(i))->flow_completed_ :
                                dynamic_cast<TcpSrc *>(flow_srcs_.at(i))->flow_completed_;
    }

    for (size_t i = 0; i < flow_count_; i++) {
        // Ignore already released flow
        if (flow_released_.at(i)) continue;

        // Release if no parent or if parent flow has completed.
        if (parent_indices_.at(i) == -1 || flow_completed_.at(parent_indices_.at(i))) return i;
    }

    return -1;
}
