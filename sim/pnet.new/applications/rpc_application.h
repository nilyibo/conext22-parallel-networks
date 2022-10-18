#ifndef RPC_APPLICATION_H
#define RPC_APPLICATION_H

#include "config.h"
#include "eventlist.h"
#include "tcp.h"
#include "event_handlers/stop_logger.h"

// Create a pair of TCP srcs to mimic behavior of an RPC application.
class RpcApplication : public EventSource {
public:
    RpcApplication(const std::string &name, EventList& eventlist, StopLogger* stop_logger, size_t src, size_t dst, simtime_picosec start_time, size_t subflow_id = -1);
    void doNextEvent() override;
    void setRequest(TcpSrc *request);
    void setResponse(TcpSrc *response);

private:
    const size_t src;
    const size_t dst;
    const simtime_picosec start_time;
    const size_t subflow_id;
    size_t callback_count = 0;
    TcpSrc *request;
    TcpSrc *response;
    StopLogger *stop_logger;
};

#endif // RPC_APPLICATION_H
