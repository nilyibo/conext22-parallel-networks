#include "graph.h"

#include <cassert>
#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <set>
#include <vector>
#include <stack>
#include <queue>
#include <iterator>
#include <atomic>
#include <algorithm>
#include <random>
#include <iomanip>
#include <omp.h>

#include "config.h"
#include "loggers.h"
#include "network.h"
#include "logfile.h"
#include "eventlist.h"
#include "switch.h"
#include "queue.h"
#include "pipe.h"

#include "absl/strings/str_format.h"

#define USE_OPENMP_PARALLELIZATION 1

int Graph::id_count = 0;

void Graph::SetupHtsimQueuesAndPipes() {
    auto *logfile = htsim_config.logfile;
    auto *event_list = htsim_config.event_list;
    auto link_speed = htsim_config.link_speed_mbps;
    auto queue_size_multiplier = htsim_config.queue_size_multiplier;
#if USE_FIRST_FIT
    FirstFit *ff = htsim_config.first_fit;
#endif

    // Initialize switch-to-switch links
    network_queues.resize(link_count, nullptr);
    network_pipes.resize(link_count, nullptr);
    network_queue_loggers.resize(link_count, nullptr);

    size_t link_id = 0;
    for (size_t switch_id = 0; switch_id < switch_count; switch_id++) {
        for (const auto &neighbor: adjacency_list[switch_id]) {
            QueueLoggerSampling* queue_logger = new QueueLoggerSampling(timeFromMs(1000), *event_list);
            logfile->addLogger(*queue_logger);
            network_queue_loggers[link_id] = queue_logger;

            network_queues[link_id] = htsim_alloc_queue(queue_logger, link_speed, queue_size_multiplier);
            AddQueueId(network_queues[link_id], std::to_string(switch_id) + "_" + std::to_string(neighbor));
            network_queues[link_id]->setName("[" + std::to_string(graph_id) + "]Queue-sw("
                                                    + std::to_string(switch_id) + "->"
                                                    + std::to_string(neighbor) + ")");
            logfile->writeName(*(network_queues[link_id]));
#if USE_FIRST_FIT
            ff->add_queue(network_queues[link_id]);
#endif

            network_pipes[link_id] = new Pipe(timeFromUs(LINK_DELAY_US), *event_list);
            network_pipes[link_id]->setName("[" + std::to_string(graph_id) + "]Pipe-sw("
                                                + std::to_string(switch_id) + "->"
                                                + std::to_string(neighbor) + ")");
            logfile->writeName(*(network_pipes[link_id]));

            link_id++;
        }
    }

    assert(link_id == link_count);

    // Initialize server uplinks and downlinks
    host_uplink_queues.resize(server_count, std::vector<Queue*>(this->server_uplinks));
    host_uplink_pipes.resize(server_count, std::vector<Pipe*>(this->server_uplinks));
    host_downlink_queues.resize(server_count, std::vector<Queue*>(this->server_uplinks));
    host_downlink_pipes.resize(server_count, std::vector<Pipe*>(this->server_uplinks));
    host_uplink_queue_loggers.resize(server_count, std::vector<QueueLoggerSampling*>(this->server_uplinks));
    host_downlink_queue_loggers.resize(server_count, std::vector<QueueLoggerSampling*>(this->server_uplinks));

    QueueLoggerSampling* queue_logger;
    for (size_t switch_id = 0; switch_id < switch_count; switch_id++) {
        for (size_t host_index = 0; host_index < server_weight[switch_id]; host_index++) {
            size_t host_id = cumulative_server_weight[switch_id] + host_index;
            std::string port_identifier = absl::StrFormat("%d|%d.%d", host_id, switch_id, host_index);
            for (size_t link_id = 0; link_id < this->server_uplinks; link_id++) {
                std::string multiuplink_suffix = this->server_uplinks > 1 ? ("." + std::to_string(link_id)) : "";
                // Uplinks
                queue_logger = new QueueLoggerSampling(timeFromMs(1000), *event_list);
                logfile->addLogger(*queue_logger);
                host_uplink_queue_loggers.at(host_id).at(link_id) = queue_logger;

                auto host_uplink_queue = htsim_alloc_queue(queue_logger, link_speed, queue_size_multiplier);
                AddQueueId(host_uplink_queue, absl::StrFormat("%d%su", host_id, multiuplink_suffix));
                host_uplink_queue->setName(absl::StrFormat("[%d]Queue-up(%s)%s",
                                                            graph_id, port_identifier, multiuplink_suffix));
                logfile->writeName(*(host_uplink_queue));
#if USE_FIRST_FIT
                ff->add_queue(host_uplink_queue);
#endif
                host_uplink_queues.at(host_id).at(link_id) = host_uplink_queue;

                auto host_uplink_pipe = new Pipe(timeFromUs(LINK_DELAY_US), *event_list);
                host_uplink_pipe->setName(absl::StrFormat("[%d]Pipe-up(%s)%s",
                                                           graph_id, port_identifier, multiuplink_suffix));
                logfile->writeName(*(host_uplink_pipe));
                host_uplink_pipes.at(host_id).at(link_id) = host_uplink_pipe;

                // Downlinks
                queue_logger = new QueueLoggerSampling(timeFromMs(1000), *event_list);
                logfile->addLogger(*queue_logger);
                host_downlink_queue_loggers.at(host_id).at(link_id) = queue_logger;

                auto host_downlink_queue = htsim_alloc_queue(queue_logger, link_speed, queue_size_multiplier);
                AddQueueId(host_downlink_queue, absl::StrFormat("%d%sd", host_id, multiuplink_suffix));
                host_downlink_queue->setName(absl::StrFormat("[%d]Queue-down(%s)%s",
                                                            graph_id, port_identifier, multiuplink_suffix));
                logfile->writeName(*(host_downlink_queue));
#if USE_FIRST_FIT
                ff->add_queue(host_downlink_queue);
#endif
                host_downlink_queues.at(host_id).at(link_id) = host_downlink_queue;

                auto host_downlink_pipe = new Pipe(timeFromUs(LINK_DELAY_US), *event_list);
                host_downlink_pipe->setName(absl::StrFormat("[%d]Pipe-down(%s)%s",
                                                           graph_id, port_identifier, multiuplink_suffix));
                logfile->writeName(*(host_downlink_pipe));
                host_downlink_pipes.at(host_id).at(link_id) = host_downlink_pipe;
            }
        }
    }
}

vector<Route *> Graph::GetHtsimRoute(int src_host, int dst_host,
                            htsim_add_switch_hop_t add_switch_hop,
                            htsim_add_host_hop_t add_src_host_hop,
                            htsim_add_host_hop_t add_dst_host_hop) const {
    int src_switch = server_to_switch[src_host];
    int dst_switch = server_to_switch[dst_host];
    if (src_switch == dst_switch) {
        auto *route = new Route();
#if SIM_INCLUDE_SERVERS
        (this->*(add_src_host_hop))(route, src_host);
        (this->*(add_dst_host_hop))(route, dst_host);
#endif
        return std::vector<Route *>(1, route);
    }

    const auto &shortest_paths = shortestPaths.at(src_switch).at(dst_switch);
    vector<Route *> routes;

    // Whether this path is enabled.
    std::vector<bool> path_mask(shortest_paths.size(), true);
#if 0   // This seems no longer correct
    if (this->IsFatTree()) {    // For fat-tree, paths for each destination server under the rack is a different subset.
        // Format: h1 route1, h1 route2, ..., h1 route k, h2 route1, ...
        //  i-th host is at range [k * i, k * (i + 1) ) where k is the number of routes calculated (per host)
        size_t host_index = this->GetHostIndexAtSwitch(dst_host, dst_switch);
        size_t hosts_per_tor = this->GetHostCountAtSwitch(dst_switch);
        assert(shortest_paths.size() % hosts_per_tor == 0);
        size_t k = shortest_paths.size() / hosts_per_tor;

        size_t range_low = k * host_index;
        size_t range_high = k * (host_index + 1);

        std::fill(path_mask.begin(), path_mask.end(), false);
        std::fill(path_mask.begin() + range_low, path_mask.begin() + range_high, true);
    }
#endif

    for (size_t i = 0; i < shortest_paths.size(); i++) {
        if (!path_mask[i])
            continue;

        const auto &path = shortest_paths[i];
        auto *route = new Route();

#if SIM_INCLUDE_SERVERS
        (this->*(add_src_host_hop))(route, src_host);
#endif

        int from = src_switch, to;
        for (const auto hop: path) {
            to = hop;
            (this->*(add_switch_hop))(route, from, to);
            from = to;
        }

#if SIM_INCLUDE_SERVERS
        (this->*(add_dst_host_hop))(route, dst_host);
#endif
        routes.push_back(route);
    }

    return routes;
}

Graph::Graph(std::string name, size_t switch_radix, size_t server_uplinks) : Graph(name, switch_radix, 0, 0, 0, server_uplinks) {}

Graph::Graph(std::string name, size_t switch_radix, size_t node_count, size_t link_count, size_t server_uplinks)
        : Graph(name, switch_radix, node_count, node_count, link_count, server_uplinks) {}

Graph::Graph(std::string name, size_t switch_radix, size_t tor_count, size_t node_count, size_t link_count, size_t server_uplinks)
        : graph_id(id_count++), name(name),
            switch_radix(switch_radix),
            tor_count(tor_count),
            node_count(node_count),
            switch_count(node_count),
            link_count(link_count),
            server_uplinks(server_uplinks) {}

void Graph::InitializeGraph() {
    InitializeAdjacencyList();
    SortAdjacencyList();
    InitializeLinkIdMapping();
    InitializeServerWeights();
    InitializeServerIdMapping();
    failed_links = std::vector<std::set<int>>(node_count);
    std::cerr << "Initialized graph with " << node_count << " switches and "
                << link_count << " (uni-directional) links and "
                << server_count << " servers." << std::endl;
    if (this->server_uplinks > 1) std::cout << "Graph[" << graph_id << "] server_uplinks = " << server_uplinks << std::endl;
    std::cout << "Graph[" << graph_id << "] name = " << name << std::endl;
    std::cout << "Graph[" << graph_id << "] switches = " << node_count << std::endl;
    std::cout << "Graph[" << graph_id << "] links = " << link_count << std::endl;
    std::cout << "Graph[" << graph_id << "] hosts = " << server_count << std::endl;
}

void Graph::SortAdjacencyList() {
    for (auto &neighbor: adjacency_list)
        std::sort(neighbor.begin(), neighbor.end());
}

void Graph::InitializeLinkIdMapping() {
    cumulative_weight = std::vector<int>(node_count + 1, 0);
    cumulative_weight[0] = 0;
    for (size_t node = 0; node < node_count; node++) {
        const auto &neighbors = adjacency_list[node];
        cumulative_weight[node + 1] = cumulative_weight[node] + (int)neighbors.size();
    }
}

void Graph::InitializeServerWeights() {
    server_weight = std::vector<int>(node_count, 0);
    for (size_t node = 0; node < node_count; node++) {
        const auto &neighbors = adjacency_list[node];
#if SIM_INCLUDE_SERVERS
            if (switch_radix < neighbors.size()) {
                throw std::runtime_error(absl::StrFormat("Invalid radix: more neighbors at node %d than radix allows.", node));
            }
            size_t remaining_port_count = switch_radix - neighbors.size();
            assert(remaining_port_count % this->server_uplinks == 0);
            server_weight[node] = (int)(remaining_port_count / this->server_uplinks);
#else
            server_weight[node] = 1;
#endif
    }
}

void Graph::InitializeServerIdMapping() {
    cumulative_server_weight = std::vector<int>(node_count + 1, 0);
    cumulative_server_weight[0] = 0;
    for (size_t node = 0; node < node_count; node++)
        cumulative_server_weight[node + 1] = cumulative_server_weight[node] + server_weight[node];
    this->server_count = cumulative_server_weight[node_count];

    server_to_switch = std::vector<int>(server_count, -1);
    for (size_t node = 0; node < node_count; node++)
        std::fill(server_to_switch.begin() + cumulative_server_weight[node],
                    server_to_switch.begin() + cumulative_server_weight[node + 1],
                    node);
}

// Calculate all shortest paths (of the same distance) from src to dst using Dijkstra's algorithm, excluding src.
[[nodiscard]] PathCollection Graph::DijkstraAlgorithm(int src, int dst,
        const std::set<Node> &removed_nodes = std::set<Node>(),
        const std::set<Edge> &removed_edges = std::set<Edge>()) const {
    typedef std::pair<int, int> dist_pair;
    std::priority_queue<dist_pair, std::vector<dist_pair>, std::greater<>> Q;
    const int kINFINITY = std::numeric_limits<int>::max() / 2;
    std::vector<int> distance(node_count, kINFINITY);
    std::vector<std::vector<Node>> prevs(node_count, std::vector<Node>());  // The list of prevs to choose from, which all have the same distance.

    distance[src] = 0;
    Q.push(std::make_pair(distance[src], src));
    while (!Q.empty()) {
        int u = Q.top().second;
        Q.pop();

        if (u == dst)
            break;

        for (int v: adjacency_list[u]) {
            if (removed_nodes.find(v) != removed_nodes.end())
                continue;
            if (removed_edges.find(std::make_pair(u, v)) != removed_edges.end())
                continue;
            if (failed_links[u].find(v) != failed_links[u].end())
                continue;
            int alt = distance[u] + 1;
            if (alt < distance[v]) {
                distance[v] = alt;
                prevs[v].clear();
                prevs[v].push_back(u);
                Q.push(std::make_pair(distance[v], v));
            } else if (alt == distance[v]) {
                prevs[v].push_back(u);
            }
        }
    }

    // Create backwards paths from dst to src, using prevs
    std::queue<Node> queue; // Queue of nodes to process, from dst using prevs
    std::set<Node> added;   // Track the nodes that have been added to the queue, since queue doesn't guarantee uniqueness
    queue.push(dst);
    added.insert(dst);

    std::map<Node, PathCollection> all_paths;   // The paths from dst to the given node, excluding the given node
    all_paths.emplace(dst, PathCollection(1, Path()));
    while (!queue.empty()) {
        Node n = queue.front();
        queue.pop();

        PathCollection new_paths;
        for (const Path path: all_paths.at(n)) {
            Path new_path(path);
            new_path.push_back(n);
            new_paths.push_back(new_path);
        }

        for (Node prev: prevs.at(n)) {
            // Add these new paths to the paths at prev.
            std::copy(new_paths.begin(), new_paths.end(), std::back_inserter(all_paths[prev]));
            if (added.find(prev) != added.end())
                continue;
            queue.push(prev);
            added.insert(prev);
        }
    }

    // No paths found at src
    if (all_paths.find(src) == all_paths.end())
        return PathCollection();

    // Reverse paths' directions
    auto &paths = all_paths[src];
    for (auto &path: paths) {
        path.push_back(src);
        std::reverse(path.begin(), path.end());
    }
    return paths;
}

// Return the equal-cost routes of at least K elements.
//  If there are other equal-cost routes as the last element, this returns all the additional equal-cost routes.
// This implements Yen's K shortest path algorithm. Source: https://en.wikipedia.org/wiki/Yen%27s_algorithm
[[nodiscard]] PathCollection Graph::KShortestPaths(int src, int dst, int K) const {
    // Determine the shortest path from the source to the sink.
    PathCollection A = DijkstraAlgorithm(src, dst);
    if (A.empty())
        throw std::runtime_error("Failed to find path using Dijkstra's algorithm. Graph not connected.");

    if (A.size() >= K) {
        for (Path &path: A)
            path.erase(path.begin());
        return A;
    }

    // Initialize the set to store the potential kth shortest path.
    auto comparator = [](const Path& path1, const Path& path2) {
        return path1.size() < path2.size();
    };
    std::set<Path, decltype(comparator)> B(comparator);

    for (size_t k = 1; k < K; k++) {
        // The spur node ranges from the first node to the next to last node in the previous k-shortest path.
        for (size_t i = 0; i < A[k - 1].size() - 1; i++) {
            // Spur node is retrieved from the previous k-shortest path, k − 1.
            int spur_node = A[k - 1][i];
            // The sequence of nodes from the source to the spur node of the previous k-shortest path.
            auto root_path = Path(A[k - 1].begin(), A[k - 1].begin() + i + 1);  // STL excludes end

            std::set<int> removed_nodes(root_path.begin(), root_path.begin() + i);
            std::set<Edge> removed_edges;
            for (const Path &path: A) {
                if (path.size() < root_path.size())
                    continue;
                if (std::equal(root_path.begin(), root_path.end(), path.begin()))
                    removed_edges.insert(Edge(path[i], path[i + 1]));
            }

            // Calculate the spur path from the spur node to the sink.
            auto spur_paths = DijkstraAlgorithm(spur_node, dst, removed_nodes, removed_edges);
            for (Path &spur_path: spur_paths) {
                Path candidate_path(root_path);
                std::copy(spur_path.begin() + 1, spur_path.end(), std::back_inserter(candidate_path));
                if (std::find(A.begin(), A.end(), candidate_path) == A.end())
                    B.insert(candidate_path);
            }
        }

        // Added multiple paths previously, don't add from B yet.
        if (k < A.size())
            continue;

        if (B.empty())
            break;

        // Add the lowest cost paths to A. Note that B is already sorted by distance.
        int distance = B.begin()->size();
        auto it = B.begin();
        do {
            A.push_back(*it);
            it++;
        } while (it != B.end() && it->size() == distance);
        B.erase(B.begin(), it);

        if (A.size() >= K)
            break;
    }

    if (A.size() < K) {
#if USE_OPENMP_PARALLELIZATION
        #pragma omp critical
#endif
        std::cerr << "Yen's Algorithm (" << src << "->" << dst << "): found only " << A.size() << " elements." << std::endl;
    }

    for (Path &path: A)
        path.erase(path.begin());

    return A;
}

// Note: this returns a single path per pair, not all shortest paths
void Graph::modifiedFloydWarshall() {
    if (!shortestPathLen.empty())
        return;

    std::cerr << "Finding shortest paths ..." << std::endl;
    shortestPathLen = DemandMatrix(node_count, std::vector<int>(node_count, 0));
    shortestPathNext = DemandMatrix(node_count, std::vector<int>(node_count, -1));

#if USE_OPENMP_PARALLELIZATION
    #pragma omp parallel for default(none) shared(shortestPathLen)
#endif
    for (size_t i = 0; i < node_count; i++) {
        for (size_t j = 0; j < node_count; j++) {
            if (i == j) {
                shortestPathLen[i][j] = 0;
                shortestPathNext[i][j] = j;
            } else if (IsNeighbor(i, j)) {
                shortestPathLen[i][j] = 1;
                shortestPathNext[i][j] = j;
            } else {
                shortestPathLen[i][j] = static_cast<int>(node_count);
                shortestPathNext[i][j] = -1;
            }
        }
    }

    //floyd warshall
    for (size_t k = 0; k < node_count; k++) {
        for (size_t i = 0; i < node_count; i++) {
            for (size_t j = 0; j < node_count; j++) {
                if (shortestPathLen[i][j] > shortestPathLen[i][k] + shortestPathLen[k][j]) {
                    shortestPathLen[i][j] = shortestPathLen[i][k] + shortestPathLen[k][j];
                    shortestPathNext[i][j] = shortestPathNext[i][k];
                }
            }
        }
    }
}

PathCollection Graph::FloydWarshallGetPath(int src, int dst) const {
    Path hops;
    if (src == dst || shortestPathNext[src][dst] == -1)
        return PathCollection();

    int next = src;
    while (next != dst && next != -1) {
        next = shortestPathNext[next][dst];
        hops.push_back(next);
    }

    PathCollection routes;
    routes.push_back(hops);
    return routes;
}

void Graph::CalculateMaxNHopPaths(int N) {
    // Calculate all paths with maximum N hops.

    std::cerr << "Calculating all max N-hop (N=" << N << ") paths in parallel ..." << std::endl;
    int num_completed = 0;
#if USE_OPENMP_PARALLELIZATION
    #pragma omp parallel for default(none) shared(switch_count, num_completed, N, std::cerr)
#endif
    for (int src_switch = 0; src_switch < switch_count; src_switch++) {
        // DFS with maximum N hops
        std::stack<std::pair<int, Path>> S;
        S.push(std::make_pair(src_switch, Path()));
        while (!S.empty()) {
            auto pair = S.top();
            S.pop();
            auto curr = pair.first;
            auto path = pair.second;
            for (auto neighbor: GetNeighborSwitches(curr)) {
                if (std::find(path.begin(), path.end(), neighbor) != path.end())    // Already visited
                    continue;
                Path new_path(path);
                new_path.push_back(neighbor);
                // Note: we might need a mutex here if src_switch are not strictly different across threads.
                shortestPaths.at(src_switch).at(neighbor).push_back(new_path);
                if (new_path.size() < N) {
                    S.push(std::make_pair(neighbor, new_path));
                }
            }
        }
#if USE_OPENMP_PARALLELIZATION
        #pragma omp critical
#endif
        std::cerr << "Max N-hop paths: " << ++num_completed << "/" << switch_count << " completed." << std::endl;
    }

    size_t total_pairs = 0;
    size_t total_routes = 0;
    for (int src_switch = 0; src_switch < switch_count; src_switch++) {
        for (int dst_switch = 0; dst_switch < switch_count; dst_switch++) {
            if (src_switch == dst_switch) {
                shortestPaths.at(src_switch).at(dst_switch).clear();
                continue;
            }

            total_pairs += 1;
            total_routes += shortestPaths.at(src_switch).at(dst_switch).size();
        }
    }
    double routes_per_pair = static_cast<double>(total_routes) / static_cast<double>(total_pairs);
    std::cout << "LLSKR: total pairs = " << total_pairs << ", total routes = " << total_routes << ", "
                "average routes per pair = " << std::setprecision(2) << routes_per_pair << std::resetiosflags(std::ios_base::showpoint) << std::endl;
}

void Graph::CalculateLLSKRFallbackPaths(const Matrix<bool> &switch_demand, RoutingParameter routing) {
    // If there are not enough "short" paths (|P[ssw][dsw]| < th_s), use K-shortest path for those src/dst switch pairs

    std::cerr << "Checking and calculating LLSKR fallback paths ..." << std::endl;
    std::atomic<int> num_fallback = 0;
    int num_completed = 0;
#if USE_OPENMP_PARALLELIZATION
    #pragma omp parallel for default(none) shared(server_count, shortestPaths, switch_demand, num_completed, num_fallback, routing, std::cerr)
#endif
    for (size_t src_switch = 0; src_switch < switch_count; src_switch++) {
        for (size_t dst_switch = 0; dst_switch < switch_count; dst_switch++) {
            if (src_switch == dst_switch) continue;
            if (shortestPaths.at(src_switch).at(dst_switch).size() >= routing.th_s) continue;
            auto paths = KShortestPaths(src_switch, dst_switch, routing.k);
#if USE_OPENMP_PARALLELIZATION
            #pragma omp critical
#endif
            shortestPaths.at(src_switch).at(dst_switch) = paths;
            num_fallback++;
        }
#if USE_OPENMP_PARALLELIZATION
        #pragma omp critical
#endif
        std::cerr << "LLSKR fallback: " << ++num_completed << "/" << server_count << " completed." << std::endl;
    }

    std::cout << "LLSKR: num_fallback = " << num_fallback << "." << std::endl;
}

void Graph::SortLLSKRPaths() {
    std::cerr << "Sorting LLSKR paths ..." << std::endl;
#if USE_OPENMP_PARALLELIZATION
    #pragma omp parallel for default(none) shared(server_count, shortestPaths)
#endif
    for (size_t src_switch = 0; src_switch < switch_count; src_switch++) {
        for (size_t dst_switch = 0; dst_switch < switch_count; dst_switch++) {
            if (shortestPaths.at(src_switch).at(dst_switch).empty())
                continue;
            auto &paths = shortestPaths.at(src_switch).at(dst_switch);
            std::sort(paths.begin(), paths.end(), [](const Path &path1, const Path &path2) {
                return path1.size() < path2.size();
            });
        }
    }
}

Graph::~Graph() {
    for (auto *queue: network_queues)
        delete queue;
    for (auto *pipe: network_pipes)
        delete pipe;
    for (auto *logger: network_queue_loggers)
        delete logger;
    for (const auto &queues: host_uplink_queues)
        for (auto *queue: queues)
            delete queue;
    for (const auto &pipes: host_uplink_pipes)
        for (auto *pipe: pipes)
            delete pipe;
    for (const auto &loggers: host_uplink_queue_loggers)
        for (auto *logger: loggers)
            delete logger;
    for (const auto &queues: host_downlink_queues)
        for (auto *queue: queues)
            delete queue;
    for (const auto &pipes: host_downlink_pipes)
        for (auto *pipe: pipes)
            delete pipe;
    for (const auto &loggers: host_downlink_queue_loggers)
        for (auto *logger: loggers)
            delete logger;
}

void Graph::SetupHtsim(struct htsim_config htsim_config) {
    this->htsim_config = htsim_config;
    std::cout << "Graph[" << graph_id << "] link speed = " << htsim_config.link_speed_mbps << " mbps" << std::endl;
    SetupHtsimQueuesAndPipes();
}

void Graph::OutputAdjacencyListToFile(const std::string &outfile) {
    std::ofstream out(outfile);
    if (!out.is_open())
        throw std::runtime_error("Failed to open adjacency list output file.");

    std::cerr << "Writing adjacency list to file ..." << std::endl;
    for (size_t src = 0; src < node_count; src++) {
        const auto &neighbors = adjacency_list[src];
        for (int dst: neighbors)
            out << "c2_" << src << "_" << dst << std::endl; // Format "c2_src_dst" used for backward compatibility.
    }

    out.close();
}

void Graph::RunShortestPathAlgorithm(const Flowsets &flowsets, struct RoutingParameter routing, const std::string &shortest_path_outfile) {
    std::ofstream shortest_path_out(shortest_path_outfile);
    if (!shortest_path_out.is_open())
        throw std::runtime_error("Failed to open shortest path output file.");

    shortestPaths = Matrix<PathCollection>(node_count, std::vector<PathCollection>(node_count, PathCollection()));

    // Switch-level demand
    Matrix<bool> switch_demand(switch_count, std::vector<bool>(switch_count, false));
#if USE_OPENMP_PARALLELIZATION
    #pragma omp parallel for default(none) shared(server_count, flowsets, switch_demand)
#endif
    for (size_t index = 0; index < flowsets.size(); index++) {
        const auto& flowset = flowsets.at(index);
        for (const auto flow: flowset.flows) {
            size_t src_switch = GetSwitchForHost(flow.src_host);
            size_t dst_switch = GetSwitchForHost(flow.dst_host);
            if (src_switch == dst_switch)
                continue;
#if USE_OPENMP_PARALLELIZATION
            #pragma omp critical
#endif
            switch_demand.at(src_switch).at(dst_switch) = true;
        }
    }

    switch (routing.protocol) {
        case RoutingProtocol::ECMP:
        case RoutingProtocol::KSP:
        case RoutingProtocol::SPO:
        {
            std::cerr << "Running k-shortest path algorithm (K=" << routing.k << ") in parallel ..." << std::endl;

            int num_completed = 0;
#if USE_OPENMP_PARALLELIZATION
            #pragma omp parallel for default(none) shared(switch_count, shortestPaths, switch_demand, num_completed, routing, std::cerr)
#endif
            for (size_t src_switch = 0; src_switch < switch_count; src_switch++) {
                for (size_t dst_switch = 0; dst_switch < switch_count; dst_switch++) {
                    if (switch_demand.at(src_switch).at(dst_switch)) {
                        auto paths = KShortestPaths(src_switch, dst_switch, routing.k);
#if USE_OPENMP_PARALLELIZATION
                        #pragma omp critical
#endif
                        shortestPaths.at(src_switch).at(dst_switch) = paths;
                    }
                }
#if 0
#if USE_OPENMP_PARALLELIZATION
               #pragma omp critical
#endif
               std::cerr << "k-shortestPath: " << ++num_completed << "/" << switch_count << " completed." << std::endl;
#endif
            }
            break;
        }
        case RoutingProtocol::LLSKR:
        {
            /**
             * LLSKR routing algorithm
             * Figure 2 from paper: X. Yuan, S. Mahapatra, W. Nienaber, S. Pakin, and M. Lang. A New Routing Scheme for Jellyfish and Its Performance with HPC Workloads. In Proceedings of the International Conference on High Performance Computing, Networking, Storage and Analysis, SC ’13, pages 36:1– 36:11, 2013.
             *  Input: jellyfish topology RRG(N,x,y), th_w, th_s, source node src, destination node dst.
             *      RRG(N,x,y) random regular graph with N switches, x ports and y inter-switch ports.
             *  Output: k paths
             *  (1) Let H be the small integer such that (y+y^2+...+y^H) / N > th_w.
             *  (2) Compute all no more than H-hop paths between switches
             *  (3) Let P[ssw][dsw] be the set of paths from switch ssw to switch dsw: P [ssw][dst][i] is the i-th path
             *  (4) If |P[ssw][dsw]| < th_s, compute k shortest-paths from ssw to dsw and store in P[ssw][dsw][..].
             *  (5) Let l_s be the local number of src.
             *  (6) Let l_d be the local number of dst.
             *  (7) Let M = |P[ssw][dsw]|.
             *  (8) The k paths from src to dst are: P [ssw][dsw][l_d ∗ k%M ], P [ssw][dsw][(l_d ∗ k + 1)%M ], ..., P [ssw][dsw][(l_d ∗ k + k − 1)%M ]
             */
            std::cerr << "Running LLSKR algorithm ..." << std::endl;

            size_t N = switch_count;
            size_t x = switch_radix;
            double y = static_cast<double>(link_count) / static_cast<double>(switch_count);
            std::cout << "LLSKR: RRG(" << N << "," << x << "," << std::setprecision(2) << y << std::resetiosflags(std::ios_base::showpoint) << ")" << std::endl;

            // Step 1: Let H be the small integer such that (y+y^2+...+y^H) / N > th_w.
            int H = 0;
            double sum = 0.;
            do {
                H++;
                sum += pow(y, H);
            } while (sum <= N * routing.th_w);
            std::cout << "LLSKR: H = " << H << std::endl;

            // Step 2 and 3: compute all no more than H-hop paths between switches, where P[ssw][dsw] = set of paths from switch ssw to switch dsw.
            CalculateMaxNHopPaths(H);

            // Step 4 If |P[ssw][dsw]| < th_s, compute k shortest-paths from ssw to dsw and store in P[ssw][dsw][..].
            //  I.e. Ensure enough "short" paths can be found, otherwise, run K-shortest paths.
            CalculateLLSKRFallbackPaths(switch_demand, routing);

            // Make sure paths for each src/dst pair are sorted from shortest to longest.
            SortLLSKRPaths();

            // Step 5-8 is done when paths are picked for src/dst servers.
            break;
        }
        default:
            throw std::runtime_error("Unsupported routing protocol");
    }

#if 0
        std::cerr << "Running floyd warshall algorithm ..." << std::endl;
        modifiedFloydWarshall();
        for (size_t src_switch = 0; src_switch < switch_count; src_switch++) {
            for (size_t dst_switch = 0; dst_switch < switch_count; dst_switch++) {
                if (src_switch == dst_switch) continue;
                shortestPaths.at(src_switch).at(dst_switch) = FloydWarshallGetPath(src_switch, dst_switch);
            }
        }
        shortestPathLen.clear();
        shortestPathNext.clear();
#endif

    std::cerr << "Writing shortest path to file ..." << std::endl;
    shortest_path_out << "Shortest path routes[" << graph_id << "]:" << std::endl;
    shortest_path_out << "src->dst(route #): route" << std::endl;
    for (size_t src = 0; src < tor_count; src++) {
        for (size_t dst = 0; dst < tor_count; dst++) {
            auto routes = GetRoutes(src, dst);
            if (routes.empty())
                continue;

            for (size_t i = 0; i < routes.size(); i++) {
                shortest_path_out << src << "->" << dst << "(#" << i << "): ";
                shortest_path_out << src;
                for (auto hop: routes[i])
                    shortest_path_out << "," << hop;
                shortest_path_out << std::endl;
            }
        }
    }

    shortest_path_out.close();
}

void Graph::FailLinks(double probability) {
    assert(probability >= 0 && probability <= 1);
    std::cerr << "Failing links with probability " << probability << "..." << std::endl;
    std::cout << "fail_link_rate = " << probability << std::endl;

    fail_probability = probability;
    if (!failed_links.empty())
        failed_links.clear();

    size_t failed_count = 0;
    failed_links = std::vector<std::set<int>>(node_count);
    for (size_t switch_id = 0; switch_id < node_count; switch_id++) {
        for (auto &neighbor: adjacency_list[switch_id]) {
            if (GetRandomNumber() < probability) {
                failed_links[switch_id].insert(neighbor);
                failed_count++;
            }
        }
    }

    std::cout << "Failed links: " << failed_count << "/" << link_count << std::endl;
}
