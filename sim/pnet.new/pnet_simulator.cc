#include "pnet_simulator.h"

#include <cctype>
#include <queue>

#include "fat_tree.h"
#include "flow_queue.h"

static std::map<SimulationMode, std::string> map_simulation_mode_string = {
    { SimulationMode::GRAPH_ONLY, "graph-only" },
    { SimulationMode::TRAFFIC_ONLY, "traffic-only" },
    { SimulationMode::FLUID, "fluid" },
    { SimulationMode::LP_INPUT, "lp-input" },
    { SimulationMode::LP_WITH_ROUTES, "lp-routes" }
};

std::string toString(SimulationMode simulation_mode) {
    if (map_simulation_mode_string.find(simulation_mode) == map_simulation_mode_string.end())
        return std::string();

    return map_simulation_mode_string[simulation_mode];
}

SimulationMode parseSimulationMode(const std::string &value) {
    for (const auto &pair: map_simulation_mode_string) {
        if (pair.second == value)
            return pair.first;
    }

    throw std::runtime_error("Unrecognized simulation mode");
}

static std::map<SchedulingMode, std::string> map_scheduling_mode_string = {
    { SchedulingMode::SINGLE_NETWORK, "single" },
    { SchedulingMode::SHORTEST_NETWORK, "shortest" },
    { SchedulingMode::ANY_NETWORK, "any" },
    { SchedulingMode::ROUND_ROBIN, "round-robin" }
};

std::string toString(SchedulingMode scheduling_mode) {
    if (map_scheduling_mode_string.find(scheduling_mode) == map_scheduling_mode_string.end())
        return std::string();

    return map_scheduling_mode_string[scheduling_mode];
}

SchedulingMode parseSchedulingMode(const std::string &value) {
    for (const auto &pair: map_scheduling_mode_string) {
        if (pair.second == value)
            return pair.first;
    }

    throw std::runtime_error("Unrecognized scheduling mode");
}

void FluidSimulator::WriteSingleFlowPath(std::ofstream &out, const std::string& id, Route *route) {
    out << id;
    for (const auto it : *route) {
        auto logged = dynamic_cast<Logged *>(it);
        // Only print queue and src/sink.
        if (dynamic_cast<Pipe *>(logged) != nullptr)
            continue;
        else if (dynamic_cast<Queue *>(logged) != nullptr)
            out << logged->str() << ",";
        else if (dynamic_cast<TcpSrc *>(logged) != nullptr || dynamic_cast<TcpSink *>(logged) != nullptr)
            out << logged->str();
    }

    out << std::endl;
}

void FluidSimulator::WriteFlowPath(std::ofstream &out, Route *route_out, Route *route_back,
        size_t src, size_t dst, size_t subflow_id = (size_t)-1, bool reverse = false) {
    stringstream ss;
    ss << src << "->" << dst;
    if (subflow_id != (size_t)-1) ss << "[" << subflow_id << "]";
    if (reverse) ss << "-response";
    WriteSingleFlowPath(out, ss.str() + " (out): ", route_out);
    // WriteSingleFlowPath(out, ss.str() + " (back): ", route_back);
}

// Remove routes from cutoff onwards (including), using indices in permutation vector.
void FluidSimulator::TrimRoutes(RouteCollection &routes, const std::vector<std::pair<size_t, int>> &permutation, const std::vector<std::pair<size_t, int>>::iterator &cutoff) {
    RouteCollection new_routes;
    for (auto it = permutation.begin(); it != cutoff; it++) {
        new_routes.push_back(routes[it->second]);
    }

    for (auto it = cutoff; it != permutation.end(); it++) {
        delete routes[it->second];
    }

    routes = new_routes;
}

// Keep certain routes based on permutation vector (graph_id, route_index) in routes_output and delete the rest.
void FluidSimulator::TrimRoutes(const std::map<int, RouteCollection> routes_by_graph, const std::vector<std::pair<size_t, int>> &permutation, size_t keep_count, RouteCollection &routes_output) {
    routes_output.reserve(keep_count);
    size_t kept_count = 0;
    for (const auto [graph_id, route_index]: permutation) {
        if (kept_count < keep_count) {
            routes_output.push_back(routes_by_graph.at(graph_id).at(route_index));
            kept_count++;
        } else {
            delete routes_by_graph.at(graph_id).at(route_index);
        }
    }
}

void FluidSimulator::TrimRoutes(std::map<int, RouteCollection> &routes_by_graph, const std::map<int, size_t> &routes_to_keep) {
    for (const auto [graph_id, keep_count]: routes_to_keep) {
        auto &routes = routes_by_graph.at(graph_id);
        for (size_t index = keep_count; index < routes.size(); index++)
            delete routes.at(index);
        routes.resize(keep_count);
    }
}

void FluidSimulator::TrimRoutes(RouteCollection &routes, size_t route_to_keep) {
    if (route_to_keep >= routes.size())
        return;

    for (size_t i = 0; i < routes.size(); i++) {
        if (i != route_to_keep)
            delete routes.at(i);
    }

    routes = RouteCollection(routes.begin() + route_to_keep, routes.begin() + route_to_keep + 1);
}

void FluidSimulator::FreeRoutes(const RouteCollection &routes) {
    for (auto route: routes)
        delete route;
}

// This assumes routes2 are the backward routes of routes1.
//  Note: routes2 should be shuffled the same way as routes 1.
void FluidSimulator::MergeRoutesFromAllNetworks(RouteCollection &routes1, RouteCollection &routes2, size_t count) {
    // There should always be more routes available than requests.
    if (count != 0 && routes1.size() <= count)
        return;

    // 1. Sort routes by route length
    // pair<length, index>, used for shuffling.
    std::vector<std::pair<size_t, int>> permutation(routes1.size());
    for (size_t i = 0; i < routes1.size(); i++) {
        const auto *route = routes1[i];
        permutation[i] = std::make_pair(route->size(), i);
    }
    std::sort(permutation.begin(), permutation.end());

    // If count is 0, choose the lowest hops.
    if (count == 0) {
        auto cutoff = std::upper_bound(permutation.begin(), permutation.end(), permutation.begin()->first, [](size_t val, const std::pair<size_t, int> &pair) {
            return val < pair.first;
        });
        TrimRoutes(routes1, permutation, cutoff);
        TrimRoutes(routes2, permutation, cutoff);
    } else {
        // Finding where the next group starts, if exactly after the count-th, no need to shuffle.
        auto next_group = std::upper_bound(permutation.begin(), permutation.end(), (permutation.begin() + count - 1)->first, [](size_t val, const std::pair<size_t, int> &pair) {
            return val < pair.first;
        });
        if (std::distance(permutation.begin(), next_group) != count) {
            // Shuffle the group with length >= length of the count-th route, and then cut off at count-th.
            auto low = std::lower_bound(permutation.begin(), permutation.end(), (permutation.begin() + count - 1)->first, [](const std::pair<size_t, int> &pair, size_t val) {
                return pair.first < val;
            });
            std::shuffle(low, next_group, GetRNG());
        }
        TrimRoutes(routes1, permutation, permutation.begin() + count);
        TrimRoutes(routes2, permutation, permutation.begin() + count);
    }
}

inline RouteCollection ConsolidateRoutes(const std::map<int, RouteCollection> &routes_by_graph) {
    RouteCollection merged_routes;
    for (const auto &[graph_id, routes]: routes_by_graph) {
        std::copy(routes.begin(), routes.end(), std::back_inserter(merged_routes));
    }
    return merged_routes;
}

void FluidSimulator::SpreadRoutesLLSKR(const TrafficMatrix *tm,
                                        const std::map<int, RouteCollection> routes1_by_graph, const std::map<int, RouteCollection> routes2_by_graph,
                                        size_t count, size_t src, size_t dst,
                                        RouteCollection &routes1, RouteCollection &routes2) {
    size_t size = std::accumulate(routes1_by_graph.begin(), routes1_by_graph.end(), static_cast<size_t>(0),
                                    [](size_t v, const std::map<int, RouteCollection>::value_type &p) {
        return v + p.second.size();
    });

    std::cerr << "LLSKR (" << src << "->" << dst << "): " << count << " out of " << size << std::endl;

    // There should always be more routes available than requests.
    if (size <= count) {
        routes1 = ConsolidateRoutes(routes1_by_graph);
        routes2 = ConsolidateRoutes(routes2_by_graph);
        return;
    }

    // pair<length, index>, used for shuffling.
    std::vector<std::pair<size_t, int>> permutation(size);
    auto routing = tm->GetRoutingPolicy();
    switch (routing.spreading_policy) {
        case LLSKRSpreadingPolicy::RANDOM: {
            auto selection = GetRandomPermutation(size);
            for (size_t n = 0; n < size; n++) {
                permutation[n] = std::make_pair(0, selection[n]);   // First argument is not used.
            }
            routes1 = ConsolidateRoutes(routes1_by_graph);
            routes2 = ConsolidateRoutes(routes2_by_graph);
            TrimRoutes(routes1, permutation, permutation.begin() + count);
            TrimRoutes(routes2, permutation, permutation.begin() + count);
            break;
        }
        case LLSKRSpreadingPolicy::STRIDE: {
            auto graph = graphs.at(0);
            int dst_switch = graph->GetSwitchForHost(dst);
            int dst_index = graph->GetHostIndexAtSwitch(dst, dst_switch);
            int dst_switch_host_count = graph->GetHostCountAtSwitch(dst_switch);

            std::vector<size_t> unused_v(size);
            std::iota(unused_v.begin(), unused_v.end(), 0);
            std::set<size_t> unused(unused_v.begin(), unused_v.end());
            for (size_t n = 0; n < size; n++) {
                if (n < count) {
                    size_t selected_index = (dst_index + dst_switch_host_count * n) %  size;
                    while (unused.find(selected_index) == unused.end()) // linear probing for resolving collision
                        selected_index = (selected_index + 1) % size;

                    permutation[n] = std::make_pair(0, selected_index); // First argument is not used.
                    unused.erase(selected_index);
                } else {
                    size_t unused_index = *unused.begin();
                    unused.erase(unused.begin());
                    permutation[n] = std::make_pair(0, unused_index);
                }
            }
            routes1 = ConsolidateRoutes(routes1_by_graph);
            routes2 = ConsolidateRoutes(routes2_by_graph);
            TrimRoutes(routes1, permutation, permutation.begin() + count);
            TrimRoutes(routes2, permutation, permutation.begin() + count);
            break;
        }
        case LLSKRSpreadingPolicy::NETWORK: {
            // hop_count -> graph_id -> queue of route indices
            std::map<size_t, std::map<int, std::queue<size_t>>> m_hop_graph_route_indices;
            for (auto &[graph_id, routes]: routes1_by_graph) {
                for (size_t n = 0; n < routes.size(); n++) {
                    const auto route = routes.at(n);
                    // Order doesn't matter as they are of the same length
                    m_hop_graph_route_indices[route->size()][graph_id].push(n);
                }
            }

            // Order by hop count first. Then in each step, select routes across networks in a round robin fashion until done.
            size_t next_index = 0;
            for (auto &[hop_count, m_graph_routes]: m_hop_graph_route_indices) {
                size_t rr_index = GetRandomIndex(m_graph_routes.size());    // Round robin selection
                size_t empty_count = 0; // How many networks have we exhauseted
                auto rr_it = std::next(m_graph_routes.begin(), rr_index);
                while (empty_count < m_graph_routes.size()) {
                    int graph_id = rr_it->first;
                    auto &queue = rr_it->second;
                    if (queue.empty()) {
                        rr_it = std::next(rr_it);
                        if (rr_it == m_graph_routes.end())
                            rr_it = m_graph_routes.begin();
                        continue;
                    }

                    size_t selected_index = queue.front();
                    queue.pop();
                    permutation[next_index++] = std::make_pair(graph_id, selected_index);

                    if (queue.empty())
                        empty_count++;

                    rr_it = std::next(rr_it);
                    if (rr_it == m_graph_routes.end())
                        rr_it = m_graph_routes.begin();
                }
            }

            if (next_index != size)
                throw std::runtime_error("LLSKR spreading by network failed. Aborting ...");

            TrimRoutes(routes1_by_graph, permutation, count, routes1);
            TrimRoutes(routes2_by_graph, permutation, count, routes2);
            break;
        }
        default:
            throw std::runtime_error("Unsupported LLSKR spreading policy");
    }
}

double FluidSimulator::GetAveragePathLength(const RouteCollection &routes, size_t count) {
    if (routes.empty())
        return -1.;

    if (count == 0)
        return (*routes.begin())->size();

    size_t length = min(routes.size(), count);
    size_t total = 0;
    for (size_t i = 0; i < length; i++) {
        total += routes[i]->size();
    }

    return (double)total / length;
}

void FluidSimulator::ChooseRoutesForFlow(const TrafficMatrix *tm, size_t src, size_t dst) {
    auto routing = tm->GetRoutingPolicy();

    auto &routes_out = all_routes_out->at(src).at(dst);
    auto &routes_back = all_routes_back->at(src).at(dst);

    switch (this->scheduling_mode) {
        case SchedulingMode::SINGLE_NETWORK: {
            const auto *graph = graphs[GetRandomIndex(graphs.size())];
            routes_out = graph->GetHtsimRouteOut(src, dst);
            routes_back = graph->GetHtsimRouteBack(src, dst);
            TrackRouteToNetworkMapping(routes_out, graph->GetGraphId());
            break;
        }
        case SchedulingMode::ANY_NETWORK: {
            std::map<int, RouteCollection> routes_out_by_graph, routes_back_by_graph;
            std::map<int, int> m_dst_host_index;    // The host index in each graph
            for (const auto *graph: graphs) {
                int graph_id = graph->GetGraphId();
                auto g_routes_out = graph->GetHtsimRouteOut(src, dst);
                auto g_routes_back = graph->GetHtsimRouteBack(src, dst);
                routes_out_by_graph[graph_id] = g_routes_out;
                routes_back_by_graph[graph_id] = g_routes_back;
                TrackRouteToNetworkMapping(g_routes_out, graph_id);
            }

            // Coalesce routes
            size_t routes_to_keep;
            switch (routing.protocol) {
                case RoutingProtocol::ECMP:
                    routes_out = ConsolidateRoutes(routes_out_by_graph);
                    routes_back = ConsolidateRoutes(routes_back_by_graph);
                    MergeRoutesFromAllNetworks(routes_out, routes_back, 0);         // First pick the shortest paths
                    MergeRoutesFromAllNetworks(routes_out, routes_back, routing.k); // Then pick only k of them if exceeded.
                    break;
                case RoutingProtocol::SPO:
                    routes_out = ConsolidateRoutes(routes_out_by_graph);
                    routes_back = ConsolidateRoutes(routes_back_by_graph);
                    MergeRoutesFromAllNetworks(routes_out, routes_back, 0); // All routes with lowest hops, thus 0
                    break;
                case RoutingProtocol::KSP:
                    // Keep the k shortest paths
                    routes_out = ConsolidateRoutes(routes_out_by_graph);
                    routes_back = ConsolidateRoutes(routes_back_by_graph);
                    MergeRoutesFromAllNetworks(routes_out, routes_back, routing.k);
                    break;
                case RoutingProtocol::LLSKR:
                {
                    // Step 5-8: spread routes among src/dst server pairs between the two switches.
                    SpreadRoutesLLSKR(tm, routes_out_by_graph, routes_back_by_graph, routing.k, src, dst, routes_out, routes_back);
                    break;
                }
                default:
                    throw std::runtime_error("Unsupported routing protocol");
            }
            break;
        }
        case SchedulingMode::SHORTEST_NETWORK: {
            size_t graph_count = graphs.size();
            std::vector<RouteCollection> all_g_routes_out(graph_count);
            std::vector<RouteCollection> all_g_routes_back(graph_count);
            std::vector<size_t> route_lengths(graph_count);
            for (size_t i = 0; i < graph_count; i++) {
                const auto *graph = graphs[i];
                auto g_routes_out = graph->GetHtsimRouteOut(src, dst);
                auto g_routes_back = graph->GetHtsimRouteBack(src, dst);
                TrackRouteToNetworkMapping(g_routes_out, graph->GetGraphId());
                all_g_routes_out[i] = g_routes_out;
                all_g_routes_back[i] = g_routes_back;
                route_lengths[i] = g_routes_out.empty() ?
                                    std::numeric_limits<size_t>::max() :
                                    g_routes_out[0]->size();
            }

            size_t min_route_length = *std::min_element(route_lengths.begin(), route_lengths.end());
            std::vector<int> candidates;
            for (size_t i = 0; i < graph_count; i++) {
                if (route_lengths[i] == min_route_length)
                    candidates.push_back(i);
            }

            int picked = candidates[GetRandomIndex(candidates.size())];
            for (size_t i = 0; i < graph_count; i++) {
                if (i == picked)
                    continue;
                FreeRoutes(all_g_routes_out[i]);
                FreeRoutes(all_g_routes_back[i]);
            }

            routes_out = all_g_routes_out[picked];
            routes_back = all_g_routes_back[picked];
            break;
        }
        case SchedulingMode::ROUND_ROBIN: {
            std::map<int, RouteCollection> routes_out_by_graph, routes_back_by_graph;
            for (const auto *graph: graphs) {
                int graph_id = graph->GetGraphId();
                auto g_routes_out = graph->GetHtsimRouteOut(src, dst);
                auto g_routes_back = graph->GetHtsimRouteBack(src, dst);
                routes_out_by_graph[graph_id] = g_routes_out;
                routes_back_by_graph[graph_id] = g_routes_back;
                TrackRouteToNetworkMapping(g_routes_out, graph_id);
            }

            // Calculate the number of routes to keep per network.
            std::map<int, size_t> routes_to_keep;
            size_t index = 0;
            size_t count = routes_out_by_graph.size();
            size_t quotient = routing.k / count;
            size_t remainder = routing.k % count;
            for (const auto &[graph_id, routes]: routes_out_by_graph) {
                size_t keep_limit = (index++ < remainder) ? (quotient + 1) : quotient;
                routes_to_keep[graph_id] = min(keep_limit, routes.size());
            }

            TrimRoutes(routes_out_by_graph, routes_to_keep);
            TrimRoutes(routes_back_by_graph, routes_to_keep);

            // Coalesce routes
            switch (routing.protocol) {
                case RoutingProtocol::SPO:
                    throw std::runtime_error("Unsupported routing protocol \"" + toString(routing.protocol) + "\" for round-robin scheduling");
                    break;
                case RoutingProtocol::ECMP:
                case RoutingProtocol::KSP:
                    // Keep the k shortest paths
                    routes_out = ConsolidateRoutes(routes_out_by_graph);
                    routes_back = ConsolidateRoutes(routes_back_by_graph);
                    MergeRoutesFromAllNetworks(routes_out, routes_back, routing.k);
                    break;
                case RoutingProtocol::LLSKR:
                {
                    // Step 5-8: spread routes among src/dst server pairs between the two switches.
                    SpreadRoutesLLSKR(tm, routes_out_by_graph, routes_back_by_graph, routing.k, src, dst, routes_out, routes_back);
                    break;
                }
                default:
                    throw std::runtime_error("Unsupported routing protocol");
            }
            break;
        }
        default:
            throw std::runtime_error("Unrecognized scheduling mode: " + toString(scheduling_mode));
    }
}

#if USE_FIRST_FIT
void FluidSimulator::AddFirstFitRoute(size_t src, size_t dst, const RouteCollection &routes_out) {
    net_paths[src][dst] = new vector<const Route *>(routes_out.size());
    for (size_t n = 0; n < routes_out.size(); n++)
    (*net_paths[src][dst])[n] = (const Route *)routes_out[n];
}
#endif

namespace {
enum Endpoint {
    TCP_SRC,
    TCP_DST,
    RPC
};

std::string GetEndpointName(Endpoint endpoint, int src, int dst, int tm_id, int subflow_id = -1, bool reverse = false, int repeat = 0) {
    std::ostringstream oss;
    oss << "tm" << tm_id << ".";
    std::string tcp_prefix = (subflow_id == -1) ? "tcp_" : "mptcp_";
    switch (endpoint) {
        case Endpoint::TCP_SRC:
            oss << tcp_prefix << "src";
            break;
        case Endpoint::TCP_DST:
            oss << tcp_prefix << "sink";
            break;
        case Endpoint::RPC:
            oss << "rpc";
            break;
        default:
            throw std::runtime_error("Invalid endpoint ");
    }

    oss << "(" << src << "->" << dst << ")";
    if (subflow_id != -1) oss << "[" << subflow_id << "]";
    if (repeat > 0) oss << ".repeat" << repeat;
    if (reverse) oss << "-response";
    return oss.str();
}
}  // namespace

TcpSrc* FluidSimulator::CreateTcpSrc(size_t src, size_t dst, const TrafficMatrix *tm, size_t flow_size, int repeat, int subflow_id, bool reverse) {
    auto traffic_mode = tm->GetTrafficMode();
    auto &event_list = *htsim.event_list;
    EventSource *stopped_handler = nullptr;
    if (tm->GetTrafficMode() == TrafficMode::RPC) {
        if (!reverse) {  // RPC request
            // Override RPC request size
            flow_size = kDefaultRpcRequestSize;

            // Create RPC application to track request/response dependency
            auto start_time = timeFromMs(tm->GetStartTime(repeat));
            auto name = GetEndpointName(Endpoint::RPC, src, dst, tm->GetId(), subflow_id, reverse, repeat);
            auto *rpc_application = new RpcApplication(name, event_list, this->stop_logger, src, dst, start_time, subflow_id);
            tcp_applications->push_back(rpc_application);
        }

        // Request/response use the same handler
        stopped_handler = tcp_applications->back();
    } else {    // Non-RPC
        if (subflow_id == kSubflowNone) {   // Main flow, attach flow queue control
            stopped_handler = this->flow_queue_;
        } else {    // sub-flow, do not attach flow queue control
            stopped_handler = this->stop_logger;
        }
    }

    TcpLogger *logger_to_use = nullptr;
    TcpTrafficLogger *traffic_logger_to_use = nullptr;
#if ENABLE_TCP_CWND_LOGGING
    logger_to_use = this->tcp_logger;
    traffic_logger_to_use = this->traffic_logger;
#endif

    TcpSrc *tcp_src;
    if (tm->GetTransport() == TransportType::TCP) {
        if (traffic_mode == TrafficMode::BULK)
            tcp_src = new TcpSrc(nullptr, nullptr, event_list);
        else if (traffic_mode == TrafficMode::SHORT || traffic_mode == TrafficMode::RPC)
            tcp_src = new TcpSrcTransfer(logger_to_use, traffic_logger_to_use, event_list, flow_size, NULL, stopped_handler);
        else
            throw std::runtime_error("Unsupported traffic mode");
    } else {    // DCTCP
        if (traffic_mode == TrafficMode::BULK)
            tcp_src = new DCTCPSrc(nullptr, nullptr, event_list);
        else if (traffic_mode == TrafficMode::SHORT || traffic_mode == TrafficMode::RPC)
            tcp_src = new DCTCPSrcTransfer(logger_to_use, traffic_logger_to_use, event_list, flow_size, NULL, stopped_handler);
        else
            throw std::runtime_error("Unsupported traffic mode");
    }

    if (tm->GetTrafficMode() == TrafficMode::RPC) {
        auto *rpc_app = tcp_applications->back();
        if (reverse) {
            rpc_app->setResponse(tcp_src);
        } else {
            rpc_app->setRequest(tcp_src);
        }
    }

    std::string tcp_src_name = GetEndpointName(Endpoint::TCP_SRC, src, dst, tm->GetId(), subflow_id, reverse, repeat);
    tcp_src->setName(tcp_src_name);
#if LOG_LEVEL_VERBOSE
    std::cout << "Created TCP src " << tcp_src_name << " with ID " << tcp_src->get_id() << std::endl;
#endif
    tcp_src->setHighPriority(tm->GetTrafficPriority() == TrafficPriority::HIGH);
#if USE_FIRST_FIT
    ff->add_flow(src, dst, tcp_src);
#endif
    htsim.logfile->writeName(*tcp_src);

    return tcp_src;
}

TcpSink* FluidSimulator::CreateTcpSink(size_t src, size_t dst, const TrafficMatrix *tm, int repeat, int subflow_id, bool reverse) {
    TcpSink *tcp_sink;
    auto traffic_mode = tm->GetTrafficMode();
    auto &event_list = *htsim.event_list;
    if (tm->GetTransport() == TransportType::TCP) {
        if (traffic_mode == TrafficMode::BULK)
            tcp_sink = new TcpSink();
        else if (traffic_mode == TrafficMode::SHORT || traffic_mode == TrafficMode::RPC)
            tcp_sink = new TcpSinkTransfer();
        else
            throw std::runtime_error("Unsupported traffic mode");
    } else {    // DCTCP
        if (traffic_mode == TrafficMode::BULK)
            tcp_sink = new TcpSink();
        else if (traffic_mode == TrafficMode::SHORT || traffic_mode == TrafficMode::RPC)
            tcp_sink = new DCTCPSinkTransfer();
        else
            throw std::runtime_error("Unsupported traffic mode");
    }

    std::string tcp_dst_name = GetEndpointName(Endpoint::TCP_DST, src, dst, tm->GetId(), subflow_id, reverse, repeat);
    tcp_sink->setName(tcp_dst_name);
    htsim.logfile->writeName(*tcp_sink);

    return tcp_sink;
}

size_t GetSubflowCount(size_t src, size_t dst, const TrafficMatrix *tm, const RouteCollection &routes_out) {
    size_t subflow_count;
    auto routing = tm->GetRoutingPolicy();
    switch (routing.protocol) {
        case RoutingProtocol::ECMP:
            if (routes_out.size() > routing.k)
                throw std::runtime_error("Incorrect number of routes calculated for ECMP routing");
            if (routes_out.size() < routing.k)
                std::cerr << "[WARN] MPTCP flow (" << src << "->" << dst << ") only has " << routes_out.size() << " paths." << std::endl;
            subflow_count = routes_out.size();
        case RoutingProtocol::KSP:
            subflow_count = min(routes_out.size(), routing.k);
            if (subflow_count < routing.k)
                std::cerr << "AllocateFlow: " << src << "->" << dst << " has only " << subflow_count << " subflows!" << std::endl;
            break;
        case RoutingProtocol::SPO:
            subflow_count = routes_out.size();
            break;
        case RoutingProtocol::LLSKR:
            if (routes_out.size() > routing.k)
                throw std::runtime_error("Incorrect number of routes calculated for LLSKR routing");
            if (routes_out.size() < routing.k)
                std::cerr << "[WARN] MPTCP flow (" << src << "->" << dst << ") only has " << routes_out.size() << " paths." << std::endl;
            subflow_count = routes_out.size();
            break;
        default:
            throw std::runtime_error("Unsupported routing protocol");
    }

    return subflow_count;
}

void FluidSimulator::SetupTcpSrcSinkPair(size_t src, size_t dst, TcpSrc *tcp_src, TcpSink *tcp_sink, const TrafficMatrix *tm, size_t repeat, std::ofstream &flow_path_out, int subflow_id, bool reverse) {
    m_tcp_rtx_scanner.at(tm->GetId())->registerTcp(*tcp_src);
    bool is_mptcp = subflow_id != -1;
    tcp_srcs->push_back(tcp_src);
    tcp_sinks->push_back(tcp_sink);

    const auto &routes_out = reverse ? all_routes_back->at(dst).at(src) : all_routes_out->at(src).at(dst);
    const auto &routes_back = reverse ? all_routes_out->at(dst).at(src) : all_routes_back->at(src).at(dst);
    auto selection = is_mptcp ? subflow_id : GetRandomIndex(routes_out.size());
    auto *route_out = routes_out[selection];
    auto *route_back = routes_back[selection];
    assert(route_out->size() > 0);
    assert(route_back->size() > 0);

    auto *full_route_out = new Route();
    *full_route_out = *route_out;
    auto *full_route_back = new Route();
    *full_route_back = *route_back;
    htsim_routes->push_back(full_route_out);
    htsim_routes->push_back(full_route_back);

    // TODO: fix for mptcp
    if (tm->GetFlowRate() != 0) {
        auto rate_limiter = GetRateLimiter(tm->GetFlowRate(), *htsim.event_list);
        full_route_out->push_front(rate_limiter);
        rate_limiters->push_back(rate_limiter);
    }

    full_route_out->push_back(tcp_sink);
    full_route_back->push_back(tcp_src);
    tcp_src->connect(*full_route_out, *full_route_back, *tcp_sink, kMaxSimulationTime); // FlowQueue controls start time.

    auto traffic_mode = tm->GetTrafficMode();
    if (traffic_mode == TrafficMode::BULK) {
        if (is_mptcp)
            sink_logger_bulk->monitorMultipathSink(tcp_sink);
        else
            sink_logger_bulk->monitorSink(tcp_sink);
    } else if (traffic_mode == TrafficMode::SHORT || traffic_mode == TrafficMode::RPC) {
        if (is_mptcp)
            sink_logger_short->monitorMultipathSink(tcp_sink);
        else
            sink_logger_short->monitorSink(tcp_sink);
    }

    WriteFlowPath(flow_path_out, full_route_out, full_route_back, src, dst, subflow_id, reverse);
}

size_t FluidSimulator::AllocateFlow(const TrafficMatrix *tm, size_t src, size_t dst, size_t flow_size, std::ofstream &flow_path_out, size_t repeat) {
    auto transport_type = tm->GetTransport();
    auto routing = tm->GetRoutingPolicy();
    auto is_duplex = tm->GetTrafficMode() == TrafficMode::RPC;

    auto &event_list = *htsim.event_list;
    auto &logfile = *htsim.logfile;
    size_t total_subflows = 0;

    ChooseRoutesForFlow(tm, src, dst);
    auto routes_out = all_routes_out->at(src).at(dst);
    auto routes_back = all_routes_back->at(src).at(dst);

#if USE_FIRST_FIT
    AddFirstFitRoute(src, dst, routes_out);
#endif

    switch (transport_type) {
        case TransportType::TCP:
        case TransportType::DCTCP:
        {
            if (routing.protocol != RoutingProtocol::ECMP)
                throw std::runtime_error("Cannot run non-ECMP for transport " + toString(transport_type));

            TcpSrc *tcp_src = CreateTcpSrc(src, dst, tm, flow_size, repeat, kSubflowNone);
            TcpSink *tcp_sink = CreateTcpSink(src, dst, tm, repeat, kSubflowNone);
            SetupTcpSrcSinkPair(src, dst, tcp_src, tcp_sink, tm, repeat, flow_path_out, kSubflowNone);
            flow_queue_->AddFlowSrc(tcp_src);
            if (is_duplex) {
                TcpSrc *reverse_tcp_src = CreateTcpSrc(dst, src, tm, flow_size, repeat, kSubflowNone, true);
                TcpSink *reverse_tcp_sink = CreateTcpSink(dst, src, tm, repeat, kSubflowNone, true);
                SetupTcpSrcSinkPair(dst, src, reverse_tcp_src, reverse_tcp_sink, tm, repeat, flow_path_out, kSubflowNone, true);
            }
            total_subflows++;

            break;
        }
        case TransportType::MPTCP:
        case TransportType::MPDCTCP:
        {
            auto permutation = GetRandomPermutation(routes_out.size());
#ifndef DEBUG
            // Randomize subflows for release build
            ReorderVector(routes_out, permutation);
            ReorderVector(routes_back, permutation);
#endif

            auto mptcp_src = new MultipathTcpSrc(COUPLED_EPSILON, event_list, NULL);
            mptcp_src->setName("mptcp(" + std::to_string(src) + "->" + std::to_string(dst) + ")");
            mptcp_src->setStopHandler(this->flow_queue_);
            logfile.writeName(*mptcp_src);
            mptcps->push_back(mptcp_src);
            flow_queue_->AddFlowSrc(mptcp_src);
            MultipathTcpSrc *reverse_mptcp_src = nullptr;
            if (is_duplex) {
                reverse_mptcp_src = new MultipathTcpSrc(COUPLED_EPSILON, event_list, NULL);
                reverse_mptcp_src->setName("mptcp(" + std::to_string(src) + "->" + std::to_string(dst) + ")-response");
                reverse_mptcp_src->setStopHandler(this->flow_queue_);
                logfile.writeName(*reverse_mptcp_src);
                mptcps->push_back(reverse_mptcp_src);
            }

            size_t subflow_count = GetSubflowCount(src, dst, tm, routes_out);
            for (size_t subflow_id = 0; subflow_id < subflow_count; subflow_id++) {
                TcpSrc *tcp_src = CreateTcpSrc(src, dst, tm, flow_size, repeat, subflow_id);
                TcpSink *tcp_sink = CreateTcpSink(src, dst, tm, repeat, subflow_id);
                SetupTcpSrcSinkPair(src, dst, tcp_src, tcp_sink, tm, repeat, flow_path_out, subflow_id);
                mptcp_src->addSubflow(tcp_src);
                if (is_duplex) {
                    TcpSrc *reverse_tcp_src = CreateTcpSrc(dst, src, tm, flow_size, repeat, subflow_id, true);
                    TcpSink *reverse_tcp_sink = CreateTcpSink(dst, src, tm, repeat, subflow_id, true);
                    SetupTcpSrcSinkPair(dst, src, reverse_tcp_src, reverse_tcp_sink, tm, repeat, flow_path_out, subflow_id, true);
                    reverse_mptcp_src->addSubflow(reverse_tcp_src);
                }

                total_subflows++;
            }

#if ENABLE_MPTCP_CWND_LOGGING
            mptcp_window_logger->monitorMultipathTcpSource(mptcp_src);
            if (is_duplex)
                mptcp_window_logger->monitorMultipathTcpSource(reverse_mptcp_src);
#endif
            break;
        }
        default:
            throw std::runtime_error("Unsupported transport type");
    }

#if LOG_LEVEL_VERBOSE
    std::cerr << "Allocated " << total_subflows << " subflows for " << src << "->" << dst << std::endl;
#endif
    return total_subflows;
}

size_t FluidSimulator::AllocateFlows(const TrafficMatrix *tm, const std::string& flow_path_outfile, size_t repeat, bool run_htsim) {
    if (graphs.empty())
        throw std::runtime_error("No network present");

    std::ofstream flow_path_out(flow_path_outfile);
    if (!flow_path_out.is_open())
        throw std::runtime_error("Failed to open flow path output file.");

    std::cerr << "Allocating flows for traffic matrix " << tm->GetId() << " ..." << std::endl;

    auto &event_list = *htsim.event_list;
    auto &logfile = *htsim.logfile;
    auto traffic_mode = tm->GetTrafficMode();
    auto transport_type = tm->GetTransport();
    bool is_multipath = IsMultipathTransport(transport_type);

    LoadHtsimObjects(tm);

    all_routes_out->resize(host_count, std::vector<std::vector<Route *>>(host_count));
    all_routes_back->resize(host_count, std::vector<std::vector<Route *>>(host_count));

#if USE_FIRST_FIT
    throw std::runtime_error("Not implemented: need to adapt to multi-TM code.");
    net_paths = new vector<const Route *>**[host_count];
    for (int src = 0; src < host_count; src++)
        net_paths[src] = new vector<const Route *>*[host_count];
    ff->net_paths = net_paths;
#endif

    size_t total_flows = 0, total_subflows = 0;
    for (const auto& flowset: tm->GetFlowsets()) {
        // FlowQueue callback happens on main flow, aka MptcpSrc rather than TcpSrc for multipath transport, in which case stop_logger shouldn't be called.
        const std::string flow_queue_name = absl::StrFormat("FlowQueue-%s", flowset.name);
        this->flow_queue_ = new FlowQueue(event_list, flow_queue_name, is_multipath ? nullptr : this->stop_logger, flowset.concurrency, tm->LoopFlows());
        if (is_multipath) this->flow_queue_->SetMultipath();
        size_t flow_index = 0;
        for (const auto& flow: flowset.flows) {
            size_t src = flow.src_host;
            size_t dst = flow.dst_host;
            size_t flow_size = flow.flow_size;
            if (src == dst) {
                std::cerr << "Skipping flow with same src/dst " << src << " ..." << std::endl;
                flow_queue_->AddFlowSrc(nullptr);
                continue;
            }
#if LOG_LEVEL_VERBOSE
            std::cerr << absl::StrFormat("Flowset %s: flow#%d %zu -> %zu, size %s.", flowset.name, flow_index++, src, dst, HumanizeSize(flow_size)) << std::endl;
#endif
            size_t subflows = AllocateFlow(tm, src, dst, flow_size, flow_path_out, repeat);
            total_flows++;
            total_subflows += subflows;
        }
        this->flow_queue_->SetDependency(flowset.parent_indices);
        this->flow_queues_->push_back(this->flow_queue_);
        this->flow_queue_ = nullptr;
    }

    std::cerr << "Initialized " << total_subflows << " subflows for " << total_flows << " flows" << std::endl;
    std::cerr << "Wrote flow paths to " << flow_path_outfile << std::endl;

    if (run_htsim) {
        simtime_picosec start_time = timeFromMs(tm->GetStartTime());
        std::cerr << "Starting flows at " << HumanizeTime(start_time) << std::endl;
        for (auto flow_queue: *this->flow_queues_)
            flow_queue->StartFlows(start_time);
    }

    UnloadHtsimObjects(tm);

    return total_subflows;
}

// Generate CPLEX LP solver input.
//  Note: This is modified from original LP solver input from Jellyfish paper;
//        It includes hosts in addition to switches.
void FluidSimulator::GenerateLPInput(const Flowsets& flowsets, const std::string& lp_input_outfile) {
    std::ofstream out(lp_input_outfile);
    if (!out.is_open())
        throw std::runtime_error("Failed to open LP input output file.");

    std::cerr << "Generating LP input ..." << std::endl;

    out << "Maximize \n";
    out << "obj: ";
#if SIM_LP_FAIR_SHARING
    out << "K" << std::endl;
    out << "\n\n";
#else
    std::stringstream ss_obj;
    for (const auto& flowset: flowsets) {
        for (const auto& flow: flowset.flows) {
            size_t src = flow.src_host;
            size_t dst = flow.dst_host;
            ss_obj << "f_" << src << "_" << dst << " + ";
        }
    }

    ss_obj.seekp((long)ss_obj.tellp() - 3); // Overwrite last three chars (" + ")
    ss_obj << std::endl;
    ss_obj << "\n\n";
    out << ss_obj.str();
#endif

    out << "SUBJECT TO \n";

#if SIM_LP_FAIR_SHARING
    // Type 0: flows
    std::cerr << "\tType 0: Flow >= K" << std::endl;

    out << "\\Type 0: Flow >= K\n";
    for (const auto& flowset: flowsets) {
        for (const auto& flow: flowset.flows) {
            size_t src = flow.src_host;
            size_t dst = flow.dst_host;
            out << "c0_" << src << "_" << dst << ": -f_" << src << "_" << dst <<
                                    " + " << 1 << " K <= 0" << std::endl;   // Only support uniform demand for now.
        }
    }
#endif

    // Type 1: sum (l_g_i_j_d) over destination servers d on link i_j on graph g less than link capacity
    //  Note: this includes host to switch links.
    //  Note: i/j can be either switch or host, so host id is prefixed with "h".
    //          For simplicity, d is not prefixed, since d is always a host.
    std::cerr << "\tType 1: Link capacity constraint" << std::endl;

    out << "\n\\Type 1: Link capacity constraint\n";
    for (const auto graph: graphs) {
        int graph_id = graph->GetGraphId();
        std::cerr << "\t\tType 1: graph " << graph_id << std::endl;

        // Loop over edges
        auto switches = graph->GetSwitches();
        auto servers = graph->GetServers();

        // Part 1: Source i is switch
        for (Node i: switches) {
            auto neighbor_switches = graph->GetNeighborSwitches(i);
            auto neighbor_servers = graph->GetNeighborServers(i);

            // switch to switch link
            for (Node j: neighbor_switches) {
                std::stringstream ss;
                ss << graph_id << "_" << i << "_" << j;
                auto prefix = ss.str();
                out << "c1_" << prefix << ": ";

                std::stringstream buf;
                for (Node d: servers) {
                    if (graph->GetSwitchForHost(d) == i)
                        continue;
                    buf << "l_" << prefix << "_" << d << " + ";
                }
                buf.seekp((long)buf.tellp() - 2); // Overwrite last two chars ("+ ")
                buf << "<= " << htsim.link_speed_mbps;
                out << buf.str() << std::endl;
            }

#if SIM_INCLUDE_SERVERS
            // switch to server link
            for (Node j: neighbor_servers) {
                int d = j;  // dst can only be that server.

                std::stringstream ss;
                ss << graph_id << "_" << i << "_h" << j;
                auto prefix = ss.str();
                out << "c1_" << prefix << ": ";
                out << "l_" << prefix << "_" << d << " <= " << htsim.link_speed_mbps << std::endl;
            }
#endif
        }

#if SIM_INCLUDE_SERVERS
        // Part 2: Source i is server
        // By definition, each server only has one link (on a graph)
        for (Node i: servers) {
            int j = graph->GetSwitchForHost(i);

            std::stringstream ss;
            ss << graph_id << "_h" << i << "_" << j;
            auto prefix = ss.str();
            out << "c1_" << prefix << ": ";
            std::stringstream buf;
            for (Node d: servers) {
                if (i == d)
                    continue;
                buf << "l_" << prefix << "_" << d << " + ";
            }
            buf.seekp((long)buf.tellp() - 2); // Overwrite last two chars ("+ ")
            buf << "<= " << htsim.link_speed_mbps;
            out << buf.str() << std::endl;
        }
#endif
    }

    //Type 2 - flow conservation at switches
    std::cerr << "\tType 2: Flow conservation at switches" << std::endl;

    out << "\n\\Type 2: Flow conservation at switches\n";
    for (const auto graph: graphs) {
        int graph_id = graph->GetGraphId();
        std::cerr << "\t\tType 2: graph " << graph_id << std::endl;

        // total flow out == total flow in for each switch and destination
        auto switches = graph->GetSwitches();
        auto servers = graph->GetServers();

        for (int d: servers) {
            // At switches
            for (int s: switches) {
                auto neighbor_switches = graph->GetNeighborSwitches(s);
                auto neighbor_servers = graph->GetNeighborServers(s);

                out << "c2_" << graph_id << "_" << s << "_" << d << ": ";

                // Flow in, from neighbor switches and servers other than d.
                std::stringstream buf;
                for (Node n: neighbor_switches) {
                    buf << "l_" << graph_id << "_" << n << "_" << s << "_" << d << " + ";
                }
                for (Node n: neighbor_servers) {
                    if (n == d)
                        continue;
                    buf << "l_" << graph_id << "_h" << n << "_" << s << "_" << d << " + ";
                }
                buf.seekp((long)buf.tellp() - 2); // Overwrite last two chars ("+ ")

                // Flow out, depending on whether d is attached to s
                if (graph->GetSwitchForHost(d) != s) {
                    // Case 1, d is not connected to s: only out to other switches.
                    for (Node n: neighbor_switches) {
                        buf << " - l_" << graph_id << "_" << s << "_" << n << "_" << d;
                    }
                } else {
                    // Case 2, d is connected to s: only one flow out to d, for destination d.
                    buf << " - l_" << graph_id << "_" << s << "_h" << d << "_" << d;
                }
                buf << " = 0";

                out << buf.str() << std::endl;
            }
        }
    }

    //Type 3 - flow conservation at servers, total flow out/in = sum of subflows on all graphs
    std::cerr << "\tType 3: Flow conservation at servers" << std::endl;

    out << "\n\\Type 3: Flow conservation at servers\n";
    // Flow out: foreach (src, dst): f_src_dst = sum of l_graph_src_firsthop_dst
    for (const auto& flowset: flowsets) {
        for (const auto& flow: flowset.flows) {
            size_t src = flow.src_host;
            size_t dst = flow.dst_host;
            out << "c3o_" << src << "_" << dst << ": ";
            out << "- f_" << src << "_" << dst;
            for (const auto graph: graphs) {
                int switch_id = graph->GetSwitchForHost(src);
                out << " + l_" << graph->GetGraphId() << "_h" << src << "_" << switch_id << "_" << dst;
            }
            out << " = 0" << std::endl;
        }
    }

    // Flow in: foreach dst, sum of f_src_dst = sum of l_graph_lasthop_dst_dst
    for (size_t dst = 0; dst < host_count; dst++) {
        out << "c3i_" << dst << ":";

        // all the f_src_dst for that destination
        for (const auto& flowset: flowsets) {
            for (const auto& flow: flowset.flows) {
                size_t src = flow.src_host;
                if (flow.dst_host != dst) continue;
                out << " - f_" << src << "_" << dst;
            }
        }

        // all the l_graph_lasthop_dst_dst
        for (const auto graph: graphs) {
            int switch_id = graph->GetSwitchForHost(dst);
            out << " + l_" << graph->GetGraphId() << "_" << switch_id << "_h" << dst << "_" << dst;
        }

        out << " = 0" << std::endl;
    }

    out.close();
}

void FluidSimulator::GenerateLPInputWithRoutes(const std::string& lp_input_outfile) {
    std::ofstream out(lp_input_outfile);
        if (!out.is_open())
        throw std::runtime_error("Failed to open LP input output file.");

    std::cerr << "Generating LP input with computed routes ..." << std::endl;

    out << "Maximize \n";
    out << "obj: ";
#if SIM_LP_FAIR_SHARING
    out << "K" << std::endl;
    out << "\n\n";
#else
    std::stringstream ss_obj;
    for (const auto *tm: this->traffic_matrices) {
        int tm_id = tm->GetId();
        for (const auto& flowset: tm->GetFlowsets()) {
            for (const auto& flow: flowset.flows) {
                size_t src = flow.src_host;
                size_t dst = flow.dst_host;
                ss_obj << "f_" << tm_id << "_" << src << "_" << dst << " + ";
            }
        }
    }

    ss_obj.seekp((long)ss_obj.tellp() - 3); // Overwrite last three chars (" + ")
    ss_obj << std::endl;
    ss_obj << "\n\n";
    out << ss_obj.str();
#endif

    out << "SUBJECT TO \n";

#if SIM_LP_FAIR_SHARING
    // Type 0: for each flow, flow throughput >= K
    std::cerr << "\tType 0: Flow >= K" << std::endl;
    out << "\\Type 0: Flow >= K\n";
    for (const auto *tm: this->traffic_matrices) {
        int tm_id = tm->GetId();
        for (const auto& flowset: tm->GetFlowsets()) {
            for (const auto& flow: flowset.flows) {
                size_t src = flow.src_host;
                size_t dst = flow.dst_host;
                out << "c0_" << tm_id << "_" << src << "_" << dst << ": "
                    << "-f_" << tm_id << "_" << src << "_" << dst << " + "
                    << 1 << " K <= 0" << std::endl; // only uniform demand for now.
            }
        }
    }
#endif

    // Type 1: for each flow, flow throughput = sum of subflows' throughputs
    std::cerr << "\tType 1: Subflow throughput" << std::endl;
    out << "\n\\Type 1: Subflow throughput\n";
    for (const auto *tm: this->traffic_matrices) {
        int tm_id = tm->GetId();
        for (const auto& flowset: tm->GetFlowsets()) {
            for (const auto& flow: flowset.flows) {
                size_t src = flow.src_host;
                size_t dst = flow.dst_host;
                out << "c1_" << tm_id << "_" << src << "_" << dst << ": f_" << tm_id << "_" << src << "_" << dst;
                const auto &routes = m_all_routes_out.at(tm_id).at(src).at(dst);
                for (size_t subflow_id = 0; subflow_id < routes.size(); subflow_id++) {
                    out << " - s_" << tm_id << "_" << src << "_" << dst << "_" << subflow_id;
                }
                out << " = 0\n";
            }
        }
    }

    // Type 2: for each queue, sum of subflows' throughput is less than link capacity
    std::cerr << "\tType 2: Link capacity" << std::endl;
    out << "\n\\Type 2: Link capacity\n";

    // First, calculate subflows using each queue
    std::cerr << "\t\tCounting subflows on each link ..." << std::endl;
    typedef std::tuple<int, size_t, size_t, int> subflow_t;
    std::map<Queue *, std::vector<subflow_t>> m_queue_subflows;
    std::map<Queue *, Route *> m_queue_route;
    for (const auto *tm: this->traffic_matrices) {
        int tm_id = tm->GetId();
        for (const auto& flowset: tm->GetFlowsets()) {
            for (const auto& flow: flowset.flows) {
                size_t src = flow.src_host;
                size_t dst = flow.dst_host;
                const auto &routes = m_all_routes_out.at(tm_id).at(src).at(dst);
                for (int subflow_id = 0; subflow_id < routes.size(); subflow_id++) {
                    const auto route = routes.at(subflow_id);
                    for (auto it = route->begin(); it != route->end(); it++) {
                        auto queue = dynamic_cast<Queue *>(*it);
                        if (queue == nullptr)   // Ignore pipe and src/dst nodes
                            continue;

                        m_queue_subflows[queue].push_back(std::make_tuple(tm_id, src, dst, subflow_id));
                        m_queue_route[queue] = route;
                    }
                }
            }
        }
    }

    // Second, for each queue, limit total subflow throughput to be less than capacity
    std::cerr << "\t\tLimiting subflow total throughput ..." << std::endl;
    for (const auto &[queue, subflows]: m_queue_subflows) {
        const auto &queue_name = GetQueueId(m_queue_route.at(queue), queue);
#if !SIM_INCLUDE_SERVERS
        auto &last_c = queue_name.at(queue_name.size() - 1);
        if (!isdigit(last_c))   // Host up/down queues end with a letter
            continue;
#endif
        out << "c2_" << queue_name << ": ";
        std::stringstream buf;
        for (const auto subflow: subflows) {
            buf << "s_" << std::get<0>(subflow) // tm_id
                << "_" << std::get<1>(subflow)  // src
                << "_" << std::get<2>(subflow)  // dst
                << "_" << std::get<3>(subflow)  // subflow_id
                << " + ";
        }
        buf.seekp((long)buf.tellp() - 2); // Overwrite last two chars ("+ ")
        buf << "<= " << htsim.link_speed_mbps;

        out << buf.str() << std::endl;
    }
}

void FluidSimulator::SetupHtsimObjects() {
    auto &event_list = *htsim.event_list;
    auto &logfile = *htsim.logfile;
    this->stop_logger = new StopLogger(event_list, "stoploger");
    this->sink_logger_bulk = new TcpSinkLoggerSampling(timeFromSec(1), event_list);
    this->sink_logger_short = new TcpSinkLoggerSampling(timeFromMs(50), event_list);
    logfile.addLogger(*sink_logger_bulk);
    logfile.addLogger(*sink_logger_short);

#if ENABLE_MPTCP_CWND_LOGGING
    this->mptcp_window_logger = new MultipathTcpLoggerSampling(timeFromMs(100), event_list, MultipathTcpLogger::MultipathTcpEvent::WINDOW_UPDATE);
    logfile.addLogger(*(this->mptcp_window_logger));
#endif

#if ENABLE_TCP_CWND_LOGGING
    this->tcp_logger = new TcpLoggerSimple();
    logfile.addLogger(*(this->tcp_logger));
    this->traffic_logger = new TcpTrafficLogger();
    logfile.addLogger(*(this->traffic_logger));
#endif
}

void FluidSimulator::LoadHtsimObjects(const TrafficMatrix *tm) {
    auto tm_id = tm->GetId();
    this->all_routes_out = &(this->m_all_routes_out[tm_id]);
    this->all_routes_back = &(this->m_all_routes_back[tm_id]);
    this->htsim_routes = &(this->m_htsim_routes[tm_id]);
    this->tcp_srcs = &(this->m_tcp_srcs[tm_id]);
    this->tcp_sinks = &(this->m_tcp_sinks[tm_id]);
    this->mptcps = &(this->m_mptcps[tm_id]);
    this->tcp_applications = &(this->m_tcp_applications[tm_id]);
    this->rate_limiters = &(this->m_rate_limiters[tm_id]);
    this->flow_queues_ = &(this->m_flow_queues_[tm_id]);
}

void FluidSimulator::UnloadHtsimObjects(const TrafficMatrix *tm) {
    this->all_routes_out = nullptr;
    this->all_routes_back = nullptr;
    this->htsim_routes = nullptr;
    this->tcp_srcs = nullptr;
    this->tcp_sinks = nullptr;
    this->mptcps = nullptr;
    this->tcp_applications = nullptr;
    this->rate_limiters = nullptr;
    this->flow_queues_ = nullptr;
}

namespace {

template<class Container>
void FreeContainerOfPointers(Container &deque) {
    for (auto *t: deque)
        delete t;
    deque.clear();
}

template<class T, class Container>
void FreeContainerOfPointers(Container &deque) {
    for (auto *t: deque)
        delete dynamic_cast<T *>(t);
    deque.clear();
}

}   // namespace

void FluidSimulator::FreeHtsimObjects(const TrafficMatrix *tm) {
    bool has_rate_limiters = tm->GetFlowRate() != 0;
    auto tm_id = tm->GetId();

    LoadHtsimObjects(tm);
    auto traffic_mode = tm->GetTrafficMode();
    bool is_short_flow = traffic_mode == TrafficMode::SHORT || traffic_mode == TrafficMode::RPC;

    for (size_t src = 0; src < host_count; src++) {
        for (size_t dst = 0; dst < host_count; dst++) {
            FreeContainerOfPointers(all_routes_out->at(src).at(dst));
            FreeContainerOfPointers(all_routes_back->at(src).at(dst));
        }
    }

    m_tcp_rtx_scanner.at(tm->GetId())->deregisterAllTcps();
    switch (tm->GetTransport()) {
        case TransportType::TCP:
        case TransportType::MPTCP:
            if (is_short_flow)
                FreeContainerOfPointers<TcpSrcTransfer>(*tcp_srcs);
            else
                FreeContainerOfPointers<TcpSrc>(*tcp_srcs);
            break;
        case TransportType::DCTCP:
        case TransportType::MPDCTCP:
            if (is_short_flow)
                FreeContainerOfPointers<DCTCPSrcTransfer>(*tcp_srcs);
            else
                FreeContainerOfPointers<DCTCPSrc>(*tcp_srcs);
            break;
        default:
            SWITCH_DEFAULT_NO_IMPL;
    }

    FreeContainerOfPointers(*htsim_routes);
    FreeContainerOfPointers(*tcp_sinks);
    FreeContainerOfPointers(*mptcps);
    FreeContainerOfPointers(*tcp_applications);
    FreeContainerOfPointers(*rate_limiters);
    FreeContainerOfPointers(*flow_queues_);

    UnloadHtsimObjects(tm);
}

bool FluidSimulator::CheckCompletion(const TrafficMatrix *tm) {
    bool still_running = false;

    LoadHtsimObjects(tm);
    for (const auto* tcp_src: *tcp_srcs) {
        if (still_running) break;
        switch (tm->GetTransport()) {
            case TransportType::TCP:
            case TransportType::MPTCP:
                if (dynamic_cast<const TcpSrcTransfer*>(tcp_src)->_is_active)
                    still_running = true;
                break;
            case TransportType::DCTCP:
            case TransportType::MPDCTCP:
                if (dynamic_cast<const DCTCPSrcTransfer*>(tcp_src)->_is_active)
                    still_running = true;
                break;
            default:
                SWITCH_DEFAULT_NO_IMPL;
        }
    }

    UnloadHtsimObjects(tm);
    return !still_running;
}

FluidSimulator::FluidSimulator(size_t host_count,
                struct htsim_config htsim,
                SchedulingMode scheduling_mode) {
    this->host_count = host_count;
    this->htsim = htsim;
    this->scheduling_mode = scheduling_mode;

    std::cerr << "Initialized simulator with " << host_count << " hosts." << std::endl;

    std::cout << "host_count: " << host_count << std::endl;
    std::cout << "scheduling_mode: " << toString(scheduling_mode) << std::endl;
#if USE_FIRST_FIT
    ff = htsim.first_fit;
#endif
}

FluidSimulator::~FluidSimulator() {
    if (this->stop_logger != nullptr) delete this->stop_logger;
    if (this->sink_logger_bulk != nullptr) delete this->sink_logger_bulk;
    if (this->sink_logger_short != nullptr) delete this->sink_logger_short;

    if (this->htsim_init)
        for (const auto *tm: this->traffic_matrices)
            FreeHtsimObjects(tm);

#if ENABLE_MPTCP_CWND_LOGGING
    if (this->mptcp_window_logger != nullptr) delete this->mptcp_window_logger;
#endif
#if ENABLE_TCP_CWND_LOGGING
    if (this->tcp_logger != nullptr) delete this->tcp_logger;
    if (this->traffic_logger != nullptr) delete this->traffic_logger;
#endif
    for (auto &[tm_id, tcp_rtx_scanner]: m_tcp_rtx_scanner) {
        delete tcp_rtx_scanner;
        tcp_rtx_scanner = nullptr;
    }

    for (auto &[tm_id, short_flow_manager]: m_short_flow_managers) {
       delete short_flow_manager;
       short_flow_manager = nullptr;
    }

#if USE_FIRST_FIT
    for (int src = 0; src < host_count; src++) {
        for (int dst = 0; dst < host_count; dst++)
            delete net_paths[src][dst];
        delete[] net_paths[src];
    }
    delete[] net_paths;
#endif

    for (auto *graph: this->graphs)
        delete graph;

    for (auto *traffic_matrix: this->traffic_matrices)
        delete traffic_matrix;
}

void FluidSimulator::AddNetwork(Graph *graph) {
    if (graph->GetServerCount() != this->host_count)
        throw std::runtime_error("Failed to add network: host count mismatch.");

    this->graphs.push_back(graph);
    std::cout << "Added graph[" << (this->graphs.size() - 1) << "]: " << graph->GetName() << std::endl;

    if (graphs.size() > std::numeric_limits<uint8_t>::max())
        throw std::runtime_error("Too many networks. Increase int type limit of m_queue_graph_id");
}

void FluidSimulator::AddTrafficMatrix(TrafficMatrix *traffic_matrix) {
    if (traffic_matrix->GetSize() != this->host_count)
        throw std::runtime_error("Failed to add traffic matrix: host count mismatch");

    this->traffic_matrices.push_back(traffic_matrix);
    std::cout << "Added traffic_matrix[" << (this->traffic_matrices.size() - 1) << "]: " << traffic_matrix->GetName() << std::endl;
}

void FluidSimulator::Run(SimulationMode simulation_mode,
        const std::vector<std::string> &adjacency_list_files,
        const std::vector<std::string> &traffic_matrix_files,
        const std::string& shortest_path_outfile = NULL,
        const std::string& flow_path_outfile = NULL,
        const std::string& lp_input_outfile = NULL) {
    // Generate graphs
    if (this->graphs.size() != adjacency_list_files.size())
        throw std::runtime_error("Inconsistent # of graphs and adjacency list files.");
    for (size_t i = 0; i < this->graphs.size(); i++) {
        auto graph = this->graphs[i];
        graph->OutputAdjacencyListToFile(adjacency_list_files[i]);
    }
    if (simulation_mode == SimulationMode::GRAPH_ONLY)
        return;

    // Generate demands based on traffic matrix
    if (traffic_matrices.size() != traffic_matrix_files.size())
        throw std::runtime_error("Inconsistent # of traffic matrices and traffic matrix files.");
    std::vector<Flowsets> flowsets_lists(this->traffic_matrices.size());
    for (size_t i = 0; i < this->traffic_matrices.size(); i++) {
        auto traffic_matrix = this->traffic_matrices[i];
        auto flowsets = traffic_matrix->GenerateFlowsets(traffic_matrix_files[i]);
        flowsets_lists[i] = flowsets;
    }

    if (simulation_mode == SimulationMode::TRAFFIC_ONLY)
        return;

    if (simulation_mode == SimulationMode::LP_INPUT) {
        auto flowsets = TrafficMatrix::MergeFlowsetsList(flowsets_lists);
        GenerateLPInput(flowsets, lp_input_outfile);
    } else {
        for (auto graph: graphs){
            graph->SetupHtsim(htsim);
        }

        htsim_init = true;
        SetupHtsimObjects();
        this->shortest_path_outfile = shortest_path_outfile;
        this->flow_path_outfile = flow_path_outfile;

        auto &event_list = *htsim.event_list;
        this->total_short_flows = 0;
        simtime_picosec min_start_time = kMaxSimulationTime;
        simtime_picosec max_end_time = 0.;
        bool loop_flows = false;
        for (const auto *tm: this->traffic_matrices) {
            this->m_tcp_rtx_scanner[tm->GetId()] = new TcpRtxTimerScanner(timeFromMs(10), event_list);
            bool run_htsim = simulation_mode == SimulationMode::FLUID;  // Only run htsim-related code in packet sim mode
            auto subflows = StartTrafficMatrix(tm, 0, run_htsim);
            double start_time_ms = tm->GetStartTime();
            double end_time_ms = tm->GetEndTime();  // short flow has end time set to max sim time
            if (tm->GetTrafficMode() == TrafficMode::SHORT || tm->GetTrafficMode() == TrafficMode::RPC) {
                auto start_time = timeFromMs(start_time_ms);
                auto repeat_period = timeFromMs(tm->GetRepeatInterval());
                auto repeat_count = tm->GetRepeatCount();
                this->m_short_flow_managers[tm->GetId()] = new ShortFlowManager(event_list, start_time, repeat_count, repeat_period, this, tm);
                this->total_short_flows += subflows * (1 + repeat_count);
            }
            if (tm->LoopFlows())
                loop_flows = true;
            if (timeFromMs(start_time_ms) < min_start_time)
                min_start_time = timeFromMs(start_time_ms);
            if (timeFromMs(end_time_ms) > max_end_time)
                max_end_time = timeFromMs(end_time_ms);
            std::cout << "TM[" << tm->GetId() << "] start = " << HumanizeTime(start_time_ms, "ms") << std::endl;
            std::cout << "TM[" << tm->GetId() << "] end = " << HumanizeTime(end_time_ms, "ms") << std::endl;
        }

        max_end_time = min(max_end_time, kMaxSimulationTime);
        if (simulation_mode == SimulationMode::FLUID) {
            htsim.logfile->setStartTime(min_start_time);
            event_list.setEndtime(max_end_time);

            std::cout << "simulator start = " << HumanizeTime(min_start_time) << std::endl;
            std::cout << "simulator end = " << HumanizeTime(max_end_time) << std::endl;

            StatusReportEvent status_report(event_list, timeFromMs(1.0), stop_logger, total_short_flows, loop_flows);

            std::cerr << "Starting simulation ..." << std::endl;
            while (event_list.doNextEvent()) {}  // Run through all events
            std::cerr << "Simulation finished." << std::endl;
            std::cout << "simulator ended at " << HumanizeTime(timeAsMs(event_list.now()), "ms") << std::endl;
        } else if (simulation_mode == SimulationMode::LP_WITH_ROUTES) {
            GenerateLPInputWithRoutes(lp_input_outfile);
        } else {
            throw std::runtime_error("Unsupported simulation mode \"" + toString(simulation_mode) + "\"");
        }
    }
}

size_t FluidSimulator::StartTrafficMatrix(const TrafficMatrix *tm, size_t current_repeat, bool run_htsim) {
    std::string file_suffix = current_repeat > 0 ? (".repeat" + std::to_string(current_repeat)) : "";

    // Make sure that previous flows are completed.
    if (current_repeat > 0 && !CheckCompletion(tm))
        return 0;

    // TODO: This causes problems for throughput loggers. Need to deregister or delay freeing.
#if 0
    // Clear previous result on subsequent runs
    if (current_repeat > 0)
        FreeHtsimObjects(tm);
#endif

    auto tm_id = tm->GetId();
    const auto &flowsets = tm->GetFlowsets();
    const auto routing = tm->GetRoutingPolicy();
    for (auto graph: graphs) {
        std::stringstream ss;
        ss << shortest_path_outfile << ".tm" << tm_id << ".g" << graph->GetGraphId() << file_suffix;
        graph->RunShortestPathAlgorithm(flowsets, routing, ss.str());
    }

    std::stringstream ss;
    ss << flow_path_outfile << ".tm" << tm_id << file_suffix;
    size_t subflows = AllocateFlows(tm, ss.str(), current_repeat, run_htsim);
    return subflows;
}

void FluidSimulator::ShowProgress() {
    double now_ms = timeAsMs(htsim.event_list->now());
    double end_ms = timeAsMs(htsim.event_list->getEndTime());
    std::cerr << "Simulation progress: " << now_ms << "ms/" << end_ms << "ms" << std::endl;
    if (this->total_short_flows > 0) {
        size_t completed_count = this->stop_logger->GetCompletedCount();
        std::cerr << "Short flows: " << completed_count << "/" << this->total_short_flows << " completed." << std::endl;
    }
}

void FluidSimulator::Stop() {
    auto now = htsim.event_list->now();
    double now_ms = timeAsMs(now);
    std::cerr << "Stopping simulation at " << now_ms << "ms ..." << std::endl;
    htsim.event_list->setEndtime(now);
}
