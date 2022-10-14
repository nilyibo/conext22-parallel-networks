#include "rpc_application.h"

#include "absl/strings/str_format.h"

RpcApplication::RpcApplication(const std::string &name, EventList& eventlist, StopLogger* stop_logger, size_t src, size_t dst, simtime_picosec start_time, size_t subflow_id) : EventSource(eventlist, name), src(src), dst(dst), subflow_id(subflow_id), start_time(start_time), stop_logger(stop_logger) {}

void RpcApplication::doNextEvent() {
    if (this->callback_count == 0) {    // request completed
        std::cerr << "RPC application " << _name << " starting response at " << timeAsMs(eventlist().now()) << "ms" << std::endl;
        // Start the response stream
        this->response->resetStartTime(eventlist().now());
        // Skip handshake SYN packet
        this->response->_established = true;
    } else if (this->callback_count == 1) { // response completed
        if (this->stop_logger != nullptr) this->stop_logger->doNextEvent();
        double start_in_ms = timeAsMs(this->start_time);
        double end_in_ms = timeAsMs(this->eventlist().now());
        double diff_in_ms = end_in_ms - start_in_ms;
        std::cout << absl::StrFormat("RPC %s start: %fms, end: %fms, elapsed: %fms", this->_name, start_in_ms, end_in_ms, diff_in_ms) << std::endl;
    } else {
        throw std::runtime_error("Invalid callback on RPC application");
    }

    this->callback_count++;
}

void RpcApplication::setRequest(TcpSrc *request) {
    this->request = request;
}

void RpcApplication::setResponse(TcpSrc *response) {
    this->response = response;
}
