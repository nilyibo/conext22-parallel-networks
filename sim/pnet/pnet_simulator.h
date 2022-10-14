#ifndef PNET_SIMULATOR_H
#define PNET_SIMULATOR_H

#include <algorithm>
#include <deque>
#include <limits>
#include <map>
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <omp.h>

#include "mydefines.h"
#include "graph.h"
#include "traffic_matrix.h"
#include "event_handlers/flow_queue.h"
#include "event_handlers/short_flow_manager.h"
#include "event_handlers/status_report_event.h"
#include "event_handlers/stop_logger.h"
#include "applications/rpc_application.h"
#include "applications/hadoop_application.h"

#include "config.h"
#include "network.h"
#include "randomqueue.h"
#include "pipe.h"
#include "eventlist.h"
#include "logfile.h"
#include "loggers.h"
#include "clock.h"
#include "mtcp.h"
#include "tcp.h"
#include "dctcp.h"
#include "tcp_transfer.h"
#include "dctcp_transfer.h"
#include "cbr.h"

#define ENABLE_TCP_CWND_LOGGING 0
#define ENABLE_MPTCP_CWND_LOGGING 0

enum struct SimulationMode {
    GRAPH_ONLY,     // Only generate graphs
    TRAFFIC_ONLY,   // Only generate traffic matrices
    FLUID,          // Really htsim packet simulation, historical name
    LP_INPUT,       // Generate LP input with minimal restriction
    LP_WITH_ROUTES  // Generate LP input with routes pre-selected. Uses the same routes as htsim simulation
};

std::string toString(SimulationMode simulation_mode);
SimulationMode parseSimulationMode(const std::string &value);

// How to schedule each (sub)flow onto network planes
enum struct SchedulingMode {
    SINGLE_NETWORK,     // Pick path(s) out of any single network
    SHORTEST_NETWORK,   // Pick path(s) out of any one of the networks with shortest average path length
    ANY_NETWORK,        // Pick path(s) out of any network
    ROUND_ROBIN,        // Pick path(s) out of networks in a round robin fashion
};

std::string toString(SchedulingMode scheduling_mode);
SchedulingMode parseSchedulingMode(const std::string &value);

typedef std::vector<Route *> RouteCollection;

class FluidSimulator {
private:
    static constexpr size_t kDefaultRpcRequestSize = 64;
    static constexpr int kSubflowNone = -1; // Not using subflows / multipath.
    SchedulingMode scheduling_mode;
    std::vector<Graph *> graphs;
    std::vector<TrafficMatrix *> traffic_matrices;
    size_t host_count;
#if USE_FIRST_FIT
    FirstFit *ff;
    vector<const Route*>*** net_paths;
#endif
    size_t total_short_flows = 0;
    std::string shortest_path_outfile;
    std::string flow_path_outfile;

    // htsim data structure
    struct htsim_config htsim;

    bool htsim_init = false;

    StopLogger *stop_logger = nullptr;
    TcpSinkLoggerSampling *sink_logger_bulk = nullptr;
    TcpSinkLoggerSampling *sink_logger_short = nullptr;
#if ENABLE_MPTCP_CWND_LOGGING
    MultipathTcpLoggerSampling *mptcp_window_logger = nullptr;
#endif
#if ENABLE_TCP_CWND_LOGGING
    TcpLoggerSimple *tcp_logger = nullptr;
    TcpTrafficLogger *traffic_logger = nullptr;
#endif
    FlowQueue* flow_queue_; // Controls # of concurrent flows

    // Per-TM htsim data structure
    std::map<int, TcpRtxTimerScanner *> m_tcp_rtx_scanner;

    /* Per-TM data structures, need to be updated to correct reference in maps. */

    // Host-to-host routes, only contains hops between servers/switches.
    Matrix<RouteCollection> *all_routes_out;
    Matrix<RouteCollection> *all_routes_back;

    // Stores currently used htsim data structures until simulation is completed.
    // Routes used for htsim TCP src/sink, which includes the destination.
    std::deque<Route *> *htsim_routes;
    // TCP/DCTCP data structures
    std::deque<TcpSrc *> *tcp_srcs;
    std::deque<TcpSink *> *tcp_sinks;
    std::deque<MultipathTcpSrc *> *mptcps;
    std::deque<RpcApplication *> *tcp_applications;
    std::deque<Queue *> *rate_limiters;
    std::deque<FlowQueue *> *flow_queues_;

    /* End of per-TM data structures. */

    /* Maps to hold reference to per-TM data structures. */

    std::map<int, Matrix<RouteCollection>> m_all_routes_out;
    std::map<int, Matrix<RouteCollection>> m_all_routes_back;
    std::map<int, std::deque<Route *>> m_htsim_routes;
    std::map<int, std::deque<TcpSrc *>> m_tcp_srcs;
    std::map<int, std::deque<TcpSink *>> m_tcp_sinks;
    std::map<int, std::deque<MultipathTcpSrc *>> m_mptcps;
    std::map<int, std::deque<RpcApplication *>> m_tcp_applications;
    std::map<int, std::deque<Queue *>> m_rate_limiters;
    std::map<int, std::deque<FlowQueue *>> m_flow_queues_;

    std::map<int, ShortFlowManager *> m_short_flow_managers;

    /* End of maps. */

    // Keep mapping of routes to graphs
    std::map<const Route *, uint8_t> m_route_graph_id;

    static void WriteSingleFlowPath(std::ofstream &out, const std::string& id, Route *route);

    static void WriteFlowPath(std::ofstream &out, Route *route_out, Route *route_back,
            size_t src, size_t dst, size_t subflow_id, bool reverse);

    // Remove routes from cutoff onwards (including), using indices in permutation vector.
    static void TrimRoutes(RouteCollection &routes, const std::vector<std::pair<size_t, int>> &permutation, const std::vector<std::pair<size_t, int>>::iterator &cutoff);

    static void TrimRoutes(const std::map<int, RouteCollection> routes_by_graph, const std::vector<std::pair<size_t, int>> &permutation, size_t keep_count, RouteCollection &routes_output);

    static void TrimRoutes(std::map<int, RouteCollection> &routes_by_graph, const std::map<int, size_t> &routes_to_keep);

    static void TrimRoutes(RouteCollection &routes, size_t route_to_keep);

    static void FreeRoutes(const RouteCollection &routes);

    // This assumes routes2 are the backward routes of routes1.
    //  Note: routes2 should be shuffled the same way as routes 1.
    void MergeRoutesFromAllNetworks(RouteCollection &routes1, RouteCollection &routes2, size_t count);

    void SpreadRoutesLLSKR(const TrafficMatrix *tm,
                            const std::map<int, RouteCollection> routes1_by_graph, const std::map<int, RouteCollection> routes2_by_graph,
                            size_t count, size_t src, size_t dst,
                            RouteCollection &routes1, RouteCollection &routes2);

    double GetAveragePathLength(const RouteCollection &routes, size_t count);

    void ChooseRoutesForFlow(const TrafficMatrix *tm, size_t src, size_t dst);

    inline Queue *GetRateLimiter(uint64_t speed_mbps, EventList &event_list) {
        return new Queue(speedFromMbps(speed_mbps), memFromPkt(100), event_list, NULL);
    }

    inline void TrackRouteToNetworkMapping(std::vector<Route *> routes, uint8_t network_id) {
        for (const auto route: routes)
            m_route_graph_id[route] = network_id;
    }

    inline std::string GetQueueId(const Route *route, const Queue *queue) {
        uint8_t graph_id = m_route_graph_id.at(route);
        return std::to_string(graph_id) + "_" + graphs.at(graph_id)->GetQueueId(queue);
    }

#if USE_FIRST_FIT
      void AddFirstFitRoute(size_t src, size_t dst, const RouteCollection &routes_out);
#endif

    TcpSrc* CreateTcpSrc(size_t src, size_t dst, const TrafficMatrix *tm, size_t flow_size, int repeat, int subflow_id, bool reverse = false);

    TcpSink* CreateTcpSink(size_t src, size_t dst, const TrafficMatrix *tm, int repeat, int subflow_id, bool reverse = false);

    void SetupTcpSrcSinkPair(size_t src, size_t dst, TcpSrc *tcp_src, TcpSink *tcp_sink, const TrafficMatrix *tm, size_t reapeat, std::ofstream &flow_path_out, int subflow_id, bool reverse = false);

    size_t AllocateFlow(const TrafficMatrix *tm, size_t src, size_t dst, size_t flow_size, std::ofstream &flow_path_out, size_t repeat);

    size_t AllocateFlows(const TrafficMatrix *tm, const std::string& flow_path_outfile, size_t repeat, bool run_htsim);

    // Generate CPLEX LP solver input.
    //  Note: This is modified from original LP solver input from Jellyfish paper;
    //        It includes hosts in addition to switches.
    void GenerateLPInput(const Flowsets& flowsets, const std::string& lp_input_outfile);

    // Generate CPLEX LP solver input.
    // Notes: This limits individual flows to take already computed routes.
    void GenerateLPInputWithRoutes(const std::string& lp_input_outfile);

    void SetupHtsimObjects();

    // must call this for TM-specific htsim objects before using.
    void LoadHtsimObjects(const TrafficMatrix *tm);

    // must call this for TM-specific htsim objects after using.
    void UnloadHtsimObjects(const TrafficMatrix *tm);

    void FreeHtsimObjects(const TrafficMatrix *tm);

    bool CheckCompletion(const TrafficMatrix *tm);

public:
    FluidSimulator(size_t host_count,
                   struct htsim_config htsim,
                   SchedulingMode scheduling_mode);

    ~FluidSimulator();

    void AddNetwork(Graph *graph);

    void AddTrafficMatrix(TrafficMatrix *traffic_matrix);

    void Run(SimulationMode simulation_mode,
            const std::vector<std::string> &adjacency_list_files,
            const std::vector<std::string> &traffic_matrix_files,
            const std::string& shortest_path_outfile,
            const std::string& flow_path_outfile,
            const std::string& lp_input_outfile);

    size_t StartTrafficMatrix(const TrafficMatrix *tm, size_t current_repeat, bool run_htsim = true);

    void ShowProgress();

    void Stop();
};

#endif  // PNET_SIMULATOR_H
