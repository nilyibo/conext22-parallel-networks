#ifndef GRAPH_H
#define GRAPH_H

#include <cassert>
#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <set>
#include <vector>
#include <iterator>
#include <algorithm>
#include <random>

#include "flowset.h"
#include "mydefines.h"
#include "utils.h"

#include "config.h"
#include "loggers.h"
#include "network.h"
#include "logfile.h"
#include "eventlist.h"
#include "switch.h"
#include "queue.h"
#include "pipe.h"
#include "randomqueue.h"
#include "compositequeue.h"
#include "prioqueue.h"
#include "queue_lossless.h"
#include "queue_lossless_input.h"
#include "queue_lossless_output.h"
#include "ecnqueue.h"
#include "ecn_priority_queue.h"

typedef int Node;
typedef std::pair<int, int> Edge;
typedef std::vector<int> Path;
typedef std::vector<Path> PathCollection;

template<typename T> using Matrix = std::vector<std::vector<T>>;
using DemandMatrix = Matrix<int>;

class Graph {
private:
    static int id_count;
    const int graph_id;

protected:
    const std::string name{};

    size_t switch_radix{};
    size_t tor_count{};         // # of nodes with servers attached
    size_t node_count{};        // # of total nodes in the network
    size_t switch_count{};      // # of total nodes in the network
    size_t link_count{};        // # of total uni-directional links
    size_t server_count{};      // # of total servers
    size_t server_uplinks{};    // # of uplinks per server
    std::vector<std::vector<int>> adjacency_list;  // Note: the inner list (aka neighbor list) must be sorted.
    double fail_probability = 0.;
    std::vector<std::set<int>> failed_links;
    std::vector<int> cumulative_weight;   // Cumulative sum from above list, used to resolve link id.
    std::vector<int> server_weight;
    std::vector<int> server_to_switch;
    std::vector<int> cumulative_server_weight;  // Cumulative sum from above list, used to resolve server id.

    // Htsim configuration
    constexpr static double LINK_DELAY_US = 1.;
    constexpr static size_t DEFAULT_SWITCH_BUFFER = 100;
    constexpr static size_t RANDOM_BUFFER = 3;
    constexpr static size_t ECN_MARK_THRESHOLD = 65;   // for DCTCP @ 10G, as recommended in paper

    // Htsim data
    struct htsim_config htsim_config;
    std::vector<Queue *> network_queues;
    std::vector<Pipe *> network_pipes;
    std::vector<std::vector<Queue *>> host_uplink_queues;
    std::vector<std::vector<Pipe *>> host_uplink_pipes;
    std::vector<std::vector<Queue *>> host_downlink_queues;
    std::vector<std::vector<Pipe *>> host_downlink_pipes;

    std::vector<QueueLoggerSampling *> network_queue_loggers;
    std::vector<std::vector<QueueLoggerSampling *>> host_uplink_queue_loggers;
    std::vector<std::vector<QueueLoggerSampling *>> host_downlink_queue_loggers;

    // // NS3 data
    // NodeContainer switches, servers;
    // // The base address for each switch: 10.n1.n2.0, where n1 * 256 + n2 = switch_index
    // //  Assuming switch has n network ports and s server ports:
    // //  - switch up ports have 10.n1.n2.0, 10.n1.n2.1, ..., 10.n1.n2.(n-1)
    // //  - switch down port has 10.n1.n2.n, 10.n1.n2.(n+1), ..., 10.n1.n2.(n+s-1)
    // //  - servers have 10.n1.n2.(n+s), 10.n1.n2.(n+s+1), ..., 10.n1.n2.(n+2s-1)
    // Ipv4InterfaceContainer *ipv4_containers = new Ipv4InterfaceContainer[switch_count];         // up ports, down ports and servers
    // NetDeviceContainer *net_devices = new NetDeviceContainer[switch_count];                     // switch up ports, down ports and servers

    // // Link bandwidth and delay
    // const ns3::StringValue switch_link_datarate = ns3::StringValue("1Gbps");
    // const ns3::StringValue server_link_datarate = ns3::StringValue("1Gbps");
    // const ns3::TimeValue switch_link_delay = ns3::TimeValue(ns3::MicroSeconds(1));
    // const ns3::TimeValue server_link_delay = ns3::TimeValue(ns3::MicroSeconds(1));


private:
    DemandMatrix shortestPathLen;
    DemandMatrix shortestPathNext;
    Matrix<PathCollection> shortestPaths;

    // Id of queues
    std::map<const Queue *, std::string> m_queue_id;

    inline void AddQueueId(const Queue *queue, std::string id) {
        m_queue_id[queue] = id;
    }

    inline Queue* htsim_alloc_queue(QueueLoggerSampling *queue_logger, uint64_t speed, uint64_t queue_size_multiplier) {
        return htsim_alloc_queue(queue_logger, speed, htsim_config.queue_type, memFromPkt(DEFAULT_SWITCH_BUFFER * queue_size_multiplier), htsim_config.event_list);
    }

    inline Queue* htsim_alloc_queue(QueueLoggerSampling *queue_logger, uint64_t speed, queue_type queue_type, mem_b queue_size, EventList *event_list) {
        if (queue_type == queue_type::RANDOM)
            return new RandomQueue(speedFromMbps(speed), queue_size, *event_list, queue_logger, memFromPkt(RANDOM_BUFFER));
        else if (queue_type == queue_type::COMPOSITE)
            return new CompositeQueue(speedFromMbps(speed), queue_size, *event_list, queue_logger);
        else if (queue_type == queue_type::CTRL_PRIO)
            return new CtrlPrioQueue(speedFromMbps(speed), queue_size, *event_list, queue_logger);
        else if (queue_type == queue_type::ECN)
            return new ECNQueue(speedFromMbps(speed), queue_size, *event_list, queue_logger, memFromPkt(ECN_MARK_THRESHOLD));
        else if (queue_type == queue_type::ECN_PRIO)
            return new ECNPriorityQueue(speedFromMbps(speed), queue_size, *event_list, queue_logger, memFromPkt(ECN_MARK_THRESHOLD));
        else if (queue_type == queue_type::LOSSLESS)
            return new LosslessQueue(speedFromMbps(speed), memFromPkt(50), *event_list, queue_logger, NULL);
        else if (queue_type == queue_type::LOSSLESS_INPUT)
            return new LosslessOutputQueue(speedFromMbps(speed), memFromPkt(200), *event_list, queue_logger);
        else if (queue_type == queue_type::LOSSLESS_INPUT_ECN)
            return new LosslessOutputQueue(speedFromMbps(speed), memFromPkt(10000), *event_list, queue_logger,1,memFromPkt(16));
        else
            throw std::runtime_error("Unsupported queue type");
    }

    void SetupHtsimQueuesAndPipes();

    inline void HtsimAddHostUpLinkToRoute(Route *route, int host) const {
        route->push_back(host_uplink_queues.at(host).at(GetRandomIndex(this->server_uplinks)));
        route->push_back(host_uplink_pipes.at(host).at(GetRandomIndex(this->server_uplinks)));
    }

    inline void HtsimAddHostDownLinkToRoute(Route *route, int host) const {
        route->push_back(host_downlink_queues.at(host).at(GetRandomIndex(this->server_uplinks)));
        route->push_back(host_downlink_pipes.at(host).at(GetRandomIndex(this->server_uplinks)));
    }

    inline void HtsimAddHopToRoute(Route *route, int from, int to) const {
        int link_id = GetLinkId(from, to);
        route->push_back(network_queues[link_id]);
        route->push_back(network_pipes[link_id]);
    }

    inline void HtsimAddHostUpLinkToReverseRoute(Route *route, int host) const {
        route->push_front(host_uplink_pipes.at(host).at(GetRandomIndex(this->server_uplinks)));
        route->push_front(host_uplink_queues.at(host).at(GetRandomIndex(this->server_uplinks)));
    }

    inline void HtsimAddHostDownLinkToReverseRoute(Route *route, int host) const {
        route->push_front(host_downlink_pipes.at(host).at(GetRandomIndex(this->server_uplinks)));
        route->push_front(host_downlink_queues.at(host).at(GetRandomIndex(this->server_uplinks)));
    }

    inline void HtsimAddHopToReverseRoute(Route *route, int from, int to) const {
        int link_id = GetLinkId(to, from);
        route->push_front(network_pipes[link_id]);
        route->push_front(network_queues[link_id]);
    }

    typedef void (Graph::*htsim_add_host_hop_t)(Route *route, int host) const;
    typedef void (Graph::*htsim_add_switch_hop_t)(Route *route, int from, int to) const;

    [[nodiscard]] vector<Route *> GetHtsimRoute(int src_host, int dst_host,
                                                htsim_add_switch_hop_t add_switch_hop,
                                                htsim_add_host_hop_t add_src_host_hop,
                                                htsim_add_host_hop_t add_dst_host_hop) const;

protected:
    Graph(std::string name, size_t switch_radix, size_t server_uplinks);

    Graph(std::string name, size_t switch_radix, size_t node_count, size_t link_count, size_t server_uplinks);

    Graph(std::string name, size_t switch_radix, size_t tor_count, size_t node_count, size_t link_count, size_t server_uplinks);

    void InitializeGraph();

    virtual void InitializeAdjacencyList() = 0;

    virtual void SortAdjacencyList();

    virtual void InitializeLinkIdMapping();

    virtual void InitializeServerWeights();

    void InitializeServerIdMapping();

    inline void AddNeighbor(int src, int dst) {
        adjacency_list[src].push_back(dst);
    }

    inline void AddBidirectionNeighbor(int node1, int node2) {
        adjacency_list[node1].push_back(node2);
        adjacency_list[node2].push_back(node1);
    }

    inline void RemoveBidirectionNeighbor(int node1, int node2) {
        auto &neighbors1 = adjacency_list[node1];
        auto &neighbors2 = adjacency_list[node2];
        auto it1 = std::find(neighbors1.begin(), neighbors1.end(), node2);
        auto it2 = std::find(neighbors2.begin(), neighbors2.end(), node1);
        assert(it1 != neighbors1.end() && it2 != neighbors2.end());
        neighbors1.erase(it1);
        neighbors2.erase(it2);
    }

    [[nodiscard]] virtual inline bool IsFatTree() const {
        return false;
    }

    // Calculate all shortest paths (of the same distance) from src to dst using Dijkstra's algorithm, excluding src.
    [[nodiscard]] PathCollection DijkstraAlgorithm(int src, int dst,
            const std::set<Node> &removed_nodes,
            const std::set<Edge> &removed_edges) const;

    // Return the equal-cost routes of at least K elements.
    //  If there are other equal-cost routes as the last element, this returns all the additional equal-cost routes.
    // This implements Yen's K shortest path algorithm. Source: https://en.wikipedia.org/wiki/Yen%27s_algorithm
    [[nodiscard]] virtual PathCollection KShortestPaths(int src, int dst, int K) const;

    // Note: this returns a single path per pair, not all shortest paths
    [[deprecated]] void modifiedFloydWarshall();

    [[deprecated]] [[nodiscard]] PathCollection FloydWarshallGetPath(int src, int dst) const;

    void CalculateMaxNHopPaths(int N);

    void CalculateLLSKRFallbackPaths(const Matrix<bool> &switch_demand, RoutingParameter routing);

    void SortLLSKRPaths();

public:
    virtual ~Graph();

    [[nodiscard]] inline int GetGraphId() const {
        return this->graph_id;
    }

    [[nodiscard]] inline std::string GetName() const {
        return this->name;
    }

    [[nodiscard]] inline size_t GetNodeCount() const {
        return this->node_count;
    }

    [[nodiscard]] inline size_t GetToRCount() const {
        return this->tor_count;
    }

    [[nodiscard]] inline size_t GetServerCount() const {
        return this->server_count;
    }

    [[nodiscard]] inline size_t GetLinkCount() const {
        return this->link_count;
    }

    [[nodiscard]] inline int GetLinkId(int i, int j) const {
        int base = cumulative_weight[i];
        const auto &neighbors = adjacency_list[i];
        auto it = std::lower_bound(neighbors.begin(), neighbors.end(), j);
        int link_id = it == neighbors.end() ? -1 : (base + (int)(it - neighbors.begin()));
        if (link_id < 0 || link_id > this->link_count) {
            std::cerr << "No link exists from " << i << " to " << j << std::endl;
        }

        return link_id;
    }

    [[nodiscard]] inline std::pair<int, int> ReverseServerId(int server_id) const {
        int switch_id = server_to_switch[server_id];
        int host_index = server_id - cumulative_server_weight[switch_id];
        return std::make_pair(switch_id, host_index);
    }

    [[nodiscard]] inline Edge ReverseLinkId(int link_id) const {
        auto it = std::upper_bound(cumulative_weight.begin(), cumulative_weight.end(), link_id);
        if (it == cumulative_weight.end())
            return std::make_pair(-1, -1);

        int i = (int)(it - cumulative_weight.begin() - 1);
        int remainder = link_id - cumulative_weight[i];
        int j = adjacency_list[i][remainder];
        return std::make_pair(i, j);
    }

    [[nodiscard]] inline std::string GetQueueId(const Queue *queue) {
        return m_queue_id.at(queue);
    }

    [[nodiscard]] inline bool IsNeighbor(int i, int j, bool binary_search = true) const {
        const auto &neighbors = adjacency_list[i];
        if (binary_search)
            return std::binary_search(neighbors.begin(), neighbors.end(), j);
        else {
            for (int neighbor: neighbors)
                if (neighbor == j)
                    return true;
            return false;
        }
    }

    inline std::vector<int> GetSwitches() const {
        std::vector<int> switches(switch_count);
        std::iota(switches.begin(), switches.end(), 0);
        return switches;
    }

    inline std::vector<int> GetServers() const {
        std::vector<int> servers(server_count);
        std::iota(servers.begin(), servers.end(), 0);
        return servers;
    }

    inline std::vector<int> GetNeighborSwitches(int switch_id) const {
        if (switch_id < 0 || switch_id >= switch_count)
            throw std::runtime_error("Invalid switch_id");

        auto &all = adjacency_list.at(switch_id);
        auto &failed = failed_links.at(switch_id);
        std::vector<int> valid;
        std::set_difference(all.begin(), all.end(), failed.begin(), failed.end(), std::inserter(valid, valid.begin()));
        return valid;
    }

    inline std::vector<int> GetNeighborServers(int switch_id) const {
        if (switch_id < 0 || switch_id >= switch_count)
            throw std::runtime_error("Invalid switch_id");

        // E.g. want this   v  (index 1)
        //      servers: 2, 3, 5, ...
        //      cumul:   0, 2, 5, 10, ...
        //      reverse: 0, 0, 1, 1, 1, 2, 2, 2, 2, 2, ...
        //      (index): 0, 1, 2, 3, 4, 5, ...
        //                     ^        ^
        //                  [lower,   upper) = 2, 3, 4
        // Length = upper - lower = 3
        // Fill from lower = 2 with increasing values using iota

        int lower = cumulative_server_weight[switch_id];
        int upper = cumulative_server_weight[switch_id + 1];
        std::vector<int> servers(upper - lower);
        std::iota(servers.begin(), servers.end(), lower);

        return servers;
    }

    [[nodiscard]] inline int GetSwitchForHost(int host_id) const {
        return server_to_switch[host_id];
    }

    [[nodiscard]] inline int GetHostIndex(int switch_id, int host_index) const {
        if (switch_id < 0 || switch_id >= switch_count)
            throw std::runtime_error("switch_id out of range");
        if (host_index < 0 || host_index >= adjacency_list.at(switch_id).size())
            throw std::runtime_error("host_index out of range");
        return cumulative_server_weight.at(switch_id) + host_index;
    }

    [[nodiscard]] inline int GetHostIndexAtSwitch(int host_id, int switch_id) const {
        if (host_id < 0 || host_id > server_count)
            throw std::runtime_error("host_id out of range");
        if (switch_id < 0 || switch_id >= switch_count)
            throw std::runtime_error("switch_id out of range");

        return (host_id - cumulative_server_weight[switch_id]);
    }

    [[nodiscard]] inline size_t GetHostCountAtSwitch(int switch_id) const {
        if (switch_id < 0 || switch_id >= switch_count)
            throw std::runtime_error("switch_id out of range");

        return server_weight[switch_id];
    }

    // Get the next hops from src to dst, i.e. (src, dst]
    [[nodiscard]] inline const PathCollection &GetRoutes(int src, int dst) const {
        return shortestPaths[src][dst];
    }

    [[nodiscard]] inline vector<Route *> GetHtsimRouteOut(int src_host, int dst_host) const {
        return GetHtsimRoute(src_host, dst_host,
            &Graph::HtsimAddHopToRoute,
            &Graph::HtsimAddHostUpLinkToRoute,
            &Graph::HtsimAddHostDownLinkToRoute);
    }

    [[nodiscard]] inline vector<Route *> GetHtsimRouteBack(int src_host, int dst_host) const {
        return GetHtsimRoute(src_host, dst_host,
            &Graph::HtsimAddHopToReverseRoute,
            &Graph::HtsimAddHostDownLinkToReverseRoute,
            &Graph::HtsimAddHostUpLinkToReverseRoute);
    }

    [[nodiscard]] inline struct htsim_config GetHtsimConfig() const {
        return this->htsim_config;
    }

    void SetupHtsim(struct htsim_config htsim_config);

    virtual void OutputAdjacencyListToFile(const std::string &outfile);

    void RunShortestPathAlgorithm(const Flowsets &flowsets, struct RoutingParameter routing, const std::string &shortest_path_outfile);

    void FailLinks(double probability);
};

#endif  // GRAPH_H
