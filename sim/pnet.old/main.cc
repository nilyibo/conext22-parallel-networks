
extern "C" {
#include <getopt.h>
#include <omp.h>
}
#include <iostream>
#include <vector>
#include <limits>
#include <cstring>
#include <csignal>

#include "mydefines.h"
#include "graph.h"
#include "graph_from_file.h"
#include "torus.h"
#include "fat_tree.h"
#include "jellyfish.h"
#include "traffic_matrix.h"
#include "pnet_simulator.h"
#include "applications/hadoop_application.h"
#include "applications/hadoop/hadoop_task.h"

struct program_argument {
    GraphType graph_type;
    SimulationMode simulation_mode;
    TrafficType traffic_type;
    TrafficMode traffic_mode;
    TransportType transport_type = TransportType::TCP;
    RoutingProtocol routing_protocol = RoutingProtocol::ECMP;
    SchedulingMode scheduling_mode = SchedulingMode::ANY_NETWORK;
    size_t host_count;
    size_t node_count;
    size_t link_count;
    size_t switch_radix;
    size_t server_uplinks = 1;
    size_t grid_size;
    size_t server_ports = 1;
    size_t dimension = 2;
    size_t K = 1;   // The minimum number of shortest paths to calculate per flow, routing protocol determines actual number of paths per flow
    size_t th_w = 0;    // LLSKR routing parameter
    size_t th_s = 0;    // LLSKR routing parameter
    LLSKRSpreadingPolicy llskr_spreading_policy;
    size_t link_speed_mbps = 10000; // Default to 10G
    size_t queue_size_multiplier = 1;
    std::vector<std::string> adjacency_list_files;
    std::string shortest_path_outfile;
    std::string flow_path_outfile;
    std::string lp_input_outfile;
    std::vector<std::string> traffic_matrix_files;
    size_t group_size = std::numeric_limits<size_t>::max(); // The group size for the traffic matrix
    size_t stride = 1;
    size_t flow_size = 0;  // in B, only used in short flows
    size_t flow_rate = 0;  // in Mbps, only for rate limiting (bulk flows for now)
    double start_time_ms = 0.;   // Flow start time, in millisecond
    double end_time_ms = 0.;   // Flow start time, in millisecond, only for bulk flows.
    TrafficPriority traffic_priority = TrafficPriority::LOW;
    size_t repeat_count;    // For short flows only; 0 means no repeat
    double repeat_every_ms; // For short flows only; must be positive if repeat_count > 0
    int random_seed = 0;
    double link_fail_rate = 0.;
    HadoopApplicationType hadoop_app;
    HadoopTaskType hadoop_task;
};

enum struct LONG_ARGUMENT : int {
#pragma clang diagnostic push
#pragma ide diagnostic ignored "UnusedGlobalDeclarationInspection"
    DEFAULT = 1000,
#pragma clang diagnostic pop
    SHORTEST_PATH,
    FLOW_PATH,
    THROUGHPUT,
    UTILIZATION,
    LP_INPUT,
    TRAFFIC_MATRIX,
    ADJACENCY_LIST,
    GRID_SIZE,
    SERVER_PORTS,
    DIMENSION,
    STRIDE,
    TRANSPORT_TYPE,
    GROUP_SIZE,
    TRAFFIC_MODE,
    FLOW_SIZE,
    SCHEDULING_MODE,
    FLOW_RATE,
    ROUTING_PROTOCOL,
    TH_W,
    TH_S,
    LLSKR_SPREADING_POLICY,
    START_TIME_MS,
    END_TIME_MS,
    TRAFFIC_PRIORITY,
    REPEAT_COUNT,
    REPEAT_EVERY_MS,
    RANDOM_SEED,
    LINK_FAIL_RATE,
    SERVER_UPLINKS,
    HADOOP_APP,
    HADOOP_TASK,
    QUEUE_SIZE_MULTIPLIER,
};

#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err34-c"
#pragma ide diagnostic ignored "EmptyDeclOrStmt"
#pragma ide diagnostic ignored "UnreachableCode"
struct program_argument parse_args(int argc, char *argv[]) {
    struct program_argument args = {};

// Validate that assertion holds; otherwise print the specified error message.
#define VALIDATE_ARGS(assertion, ...) { \
    if (!(assertion)) { \
        fprintf(stderr, __VA_ARGS__); \
        fprintf(stderr, "\nUsage: %s -g <graph_type> -t <traffic_type> ...\n", argv[0]); \
        exit(EXIT_FAILURE); \
    } \
}

// Same as above, but only validate if condition is true.
#define VALIDATE_ARGS_IF(condition, assertion, ...) { \
    if ((condition)) { VALIDATE_ARGS(assertion, __VA_ARGS__); } \
}

    int c;
    static struct option long_options[] = {
            { "graph", required_argument, nullptr, 'g' },
            { "simulation_mode", required_argument, nullptr, 'm' },
            { "traffic_type", required_argument, nullptr, 't' },
            { "traffic_mode", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::TRAFFIC_MODE) },
            { "transport", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::TRANSPORT_TYPE) },
            { "routing_protocol", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::ROUTING_PROTOCOL) },
            { "host_count", required_argument, nullptr, 'h' },
            { "node_count", required_argument, nullptr, 'N' },
            { "link_count", required_argument, nullptr, 'l' },
            { "switch_radix", required_argument, nullptr, 'k' },
            { "server_uplinks", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::SERVER_UPLINKS) },
            { "grid_size", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::GRID_SIZE) },
            { "server_ports", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::SERVER_PORTS)},
            { "dimension", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::DIMENSION)},
            { "k_shortest_path", required_argument, nullptr, 'K' },
            { "link_speed", required_argument, nullptr, 's' },
            { "shortest_path", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::SHORTEST_PATH)},
            { "flow_path", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::FLOW_PATH)},
            { "throughput", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::THROUGHPUT) },
            { "utilization", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::UTILIZATION) },
            { "lp_input", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::LP_INPUT) },
            { "traffic_matrix", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::TRAFFIC_MATRIX) },
            { "adjacency_list", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::ADJACENCY_LIST) },
            { "stride", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::STRIDE) },
            { "group_size", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::GROUP_SIZE) },
            { "flow_size", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::FLOW_SIZE) },
            { "scheduling_mode", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::SCHEDULING_MODE) },
            { "flow_rate", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::FLOW_RATE) },
            { "th_w", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::TH_W) },
            { "th_s", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::TH_S) },
            { "spreading_policy", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::LLSKR_SPREADING_POLICY) },
            { "start_time", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::START_TIME_MS) },
            { "end_time", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::END_TIME_MS) },
            { "priority", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::TRAFFIC_PRIORITY) },
            { "repeat", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::REPEAT_COUNT) },
            { "repeat_every", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::REPEAT_EVERY_MS) },
            { "random_seed", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::RANDOM_SEED) },
            { "link_fail", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::LINK_FAIL_RATE) },
            { "hadoop_app", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::HADOOP_APP) },
            { "hadoop_task", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::HADOOP_TASK) },
            { "queue_size_multiplier", required_argument, nullptr, static_cast<int>(LONG_ARGUMENT::QUEUE_SIZE_MULTIPLIER) },
            { nullptr, 0, nullptr, 0 }  // The last element of the array has to be filled with zeros.
    };
    std::string graph_type;
    std::string simulation_mode;
    std::string traffic_type;
    std::string traffic_mode;
    std::string transport_type = toString(TransportType::TCP);
    std::string routing_protocol = toString(RoutingProtocol::ECMP);
    std::string scheduling_mode = toString(SchedulingMode::ANY_NETWORK);
    std::string llskr_spreading_policy = toString(LLSKRSpreadingPolicy::RANDOM);
    std::string traffic_priority = toString(TrafficPriority::LOW);
    std::string hadoop_app;
    std::string hadoop_task;
    while (true) {
        c = getopt_long(argc, argv, "g:m:t:h:N:l:k:K:s:", long_options, nullptr);
        if (c == -1)
            break;

        switch (c) {
            case 'g':
                graph_type = std::string(optarg);
                break;
            case 'm':
                simulation_mode = std::string(optarg);
                break;
            case 't':
                traffic_type = std::string(optarg);
                break;
            case static_cast<int>(LONG_ARGUMENT::TRAFFIC_MODE):
                traffic_mode = std::string(optarg);
                break;
            case static_cast<int>(LONG_ARGUMENT::TRANSPORT_TYPE):
                transport_type = std::string(optarg);
                break;
            case static_cast<int>(LONG_ARGUMENT::ROUTING_PROTOCOL):
                routing_protocol = std::string(optarg);
                break;
            case 'h':
                if (sscanf(optarg, "%zu", &(args.host_count)) != 1)
                    goto invalid;
                break;
            case 'N':
                if (sscanf(optarg, "%zu", &(args.node_count)) != 1)
                    goto invalid;
                break;
            case 'l':
                if (sscanf(optarg, "%zu", &(args.link_count)) != 1)
                    goto invalid;
                break;
            case 'k':
                if (sscanf(optarg, "%zu", &(args.switch_radix)) != 1)
                    goto invalid;
                break;
            case static_cast<int>(LONG_ARGUMENT::SERVER_UPLINKS):
                if (sscanf(optarg, "%zu", &(args.server_uplinks)) != 1)
                    goto invalid;
                break;
            case static_cast<int>(LONG_ARGUMENT::GRID_SIZE):
                if (sscanf(optarg, "%zu", &(args.grid_size)) != 1)
                    goto invalid;
                break;
            case static_cast<int>(LONG_ARGUMENT::SERVER_PORTS):
                if (sscanf(optarg, "%zu", &(args.server_ports)) != 1)
                    goto invalid;
                break;
            case static_cast<int>(LONG_ARGUMENT::DIMENSION):
                if (sscanf(optarg, "%zu", &(args.dimension)) != 1)
                    goto invalid;
                break;
            case 'K':
                if (sscanf(optarg, "%zu", &(args.K)) != 1)
                    goto invalid;
                break;
            case 's':
                if (sscanf(optarg, "%zu", &(args.link_speed_mbps)) != 1)
                    goto invalid;
                break;
            case static_cast<int>(LONG_ARGUMENT::SHORTEST_PATH):
                args.shortest_path_outfile = std::string(optarg);
                break;
            case static_cast<int>(LONG_ARGUMENT::FLOW_PATH):
                args.flow_path_outfile = std::string(optarg);
                break;
            case static_cast<int>(LONG_ARGUMENT::THROUGHPUT):
            case static_cast<int>(LONG_ARGUMENT::UTILIZATION):
                throw std::runtime_error("argument deprecated");
            case static_cast<int>(LONG_ARGUMENT::LP_INPUT):
                args.lp_input_outfile = std::string(optarg);
                break;
            case static_cast<int>(LONG_ARGUMENT::TRAFFIC_MATRIX):
                args.traffic_matrix_files.emplace_back(optarg);
                break;
            case static_cast<int>(LONG_ARGUMENT::ADJACENCY_LIST):
                args.adjacency_list_files.emplace_back(optarg);
                break;
            case static_cast<int>(LONG_ARGUMENT::STRIDE):
                if (sscanf(optarg, "%zu", &(args.stride)) != 1)
                    goto invalid;
                break;
            case static_cast<int>(LONG_ARGUMENT::GROUP_SIZE):
                if (sscanf(optarg, "%zu", &(args.group_size)) != 1)
                    goto invalid;
                break;
            case static_cast<int>(LONG_ARGUMENT::FLOW_SIZE):
                if (sscanf(optarg, "%zu", &(args.flow_size)) != 1)
                    goto invalid;
                break;
            case static_cast<int>(LONG_ARGUMENT::SCHEDULING_MODE):
                scheduling_mode = std::string(optarg);
                break;
            case static_cast<int>(LONG_ARGUMENT::FLOW_RATE):
                if (sscanf(optarg, "%zu", &(args.flow_rate)) != 1)
                    goto invalid;
                break;
            case static_cast<int>(LONG_ARGUMENT::TH_W):
                if (sscanf(optarg, "%zu", &(args.th_w)) != 1)
                    goto invalid;
                break;
            case static_cast<int>(LONG_ARGUMENT::TH_S):
                if (sscanf(optarg, "%zu", &(args.th_s)) != 1)
                    goto invalid;
                break;
            case static_cast<int>(LONG_ARGUMENT::LLSKR_SPREADING_POLICY):
                llskr_spreading_policy = std::string(optarg);
                break;
            case static_cast<int>(LONG_ARGUMENT::START_TIME_MS):
                if (sscanf(optarg, "%lf", &(args.start_time_ms)) != 1)
                    goto invalid;
                break;
            case static_cast<int>(LONG_ARGUMENT::END_TIME_MS):
                if (sscanf(optarg, "%lf", &(args.end_time_ms)) != 1)
                    goto invalid;
                break;
            case static_cast<int>(LONG_ARGUMENT::TRAFFIC_PRIORITY):
                traffic_priority = std::string(optarg);
                break;
            case static_cast<int>(LONG_ARGUMENT::REPEAT_COUNT):
                if (sscanf(optarg, "%zu", &(args.repeat_count)) != 1)
                    goto invalid;
                break;
            case static_cast<int>(LONG_ARGUMENT::REPEAT_EVERY_MS):
                if (sscanf(optarg, "%lf", &(args.repeat_every_ms)) != 1)
                    goto invalid;
                break;
            case static_cast<int>(LONG_ARGUMENT::RANDOM_SEED):
                if (sscanf(optarg, "%d", &(args.random_seed)) != 1)
                    goto invalid;
                break;
            case static_cast<int>(LONG_ARGUMENT::LINK_FAIL_RATE):
                if (sscanf(optarg, "%lf", &(args.link_fail_rate)) != 1)
                    goto invalid;
                break;
            case static_cast<int>(LONG_ARGUMENT::HADOOP_APP):
                hadoop_app = std::string(optarg);
                break;
            case static_cast<int>(LONG_ARGUMENT::HADOOP_TASK):
                hadoop_task = std::string(optarg);
                break;
            case static_cast<int>(LONG_ARGUMENT::QUEUE_SIZE_MULTIPLIER):
                if (sscanf(optarg, "%zu", &(args.queue_size_multiplier)) != 1)
                    goto invalid;
                break;
            default:
                VALIDATE_ARGS(false, "Aborting due to invalid option ...");
                break;
        }
    }

    VALIDATE_ARGS(!graph_type.empty(), "Graph type must be specified.");
    VALIDATE_ARGS(!simulation_mode.empty(), "Simulation mode must be specified.");
    VALIDATE_ARGS(!args.adjacency_list_files.empty(), "adjacency_list is required.");
    VALIDATE_ARGS(args.server_uplinks > 0, "Server uplinks must be positive.")

    args.graph_type = parseGraphType(graph_type);
    args.simulation_mode = parseSimulationMode(simulation_mode);

    if (args.simulation_mode != SimulationMode::GRAPH_ONLY) {
#if USE_CONSTANT_SEED
        VALIDATE_ARGS(args.random_seed != 0, "Random seed must be non-zero.");
#endif
        VALIDATE_ARGS(!traffic_type.empty(), "Traffic type must be specified.");
        args.traffic_type = parseTraffixType(traffic_type);
        if (args.traffic_type != TrafficType::FILE) {
            VALIDATE_ARGS(!traffic_mode.empty(), "Traffic mode must be specified.");
            VALIDATE_ARGS(args.group_size > 0, "Group size must be positive.");
            VALIDATE_ARGS(!args.traffic_matrix_files.empty(), "Traffic matrix files must be specified.");
            VALIDATE_ARGS(args.start_time_ms >= 0, "Start time cannot be negative.");
            VALIDATE_ARGS(args.end_time_ms >= 0, "End time cannot be negative.");
            VALIDATE_ARGS_IF(args.end_time_ms != 0, args.end_time_ms > args.start_time_ms, "End time must be greater than start time.");
            args.traffic_mode = parseTrafficMode(traffic_mode);
            args.transport_type = parseTransportType(transport_type);
            args.routing_protocol = parseRoutingProtocol(routing_protocol);
            args.scheduling_mode = parseSchedulingMode(scheduling_mode);
            args.llskr_spreading_policy = parseLLSKRSpreadingPoliicy(llskr_spreading_policy);
            args.traffic_priority = parseTrafficPriority(traffic_priority);
            if (args.traffic_type == TrafficType::HADOOP) {
                args.hadoop_app = parseHadoopApplicationType(hadoop_app);
                args.hadoop_task = parseHadoopTaskType(hadoop_task);
                VALIDATE_ARGS(args.traffic_mode == TrafficMode::SHORT, "Hadoop app uses short flow TMs.");
            }
            VALIDATE_ARGS_IF(args.traffic_mode == TrafficMode::BULK, args.repeat_count == 0, "Bulk flows cannot be repeated.");
            VALIDATE_ARGS_IF(args.traffic_mode == TrafficMode::BULK, args.repeat_every_ms == 0., "Bulk flows cannot be repeated.");
            VALIDATE_ARGS_IF((args.traffic_mode == TrafficMode::SHORT || args.traffic_mode == TrafficMode::RPC) && args.repeat_count > 0, args.repeat_every_ms > 0, "Interval must be specified for repeated flows.");
        }
    }

    switch(args.simulation_mode) {
        case SimulationMode::GRAPH_ONLY:
        case SimulationMode::TRAFFIC_ONLY:
            break;
        case SimulationMode::FLUID:
            VALIDATE_ARGS(!args.shortest_path_outfile.empty(), "shortest_path is required.");
            VALIDATE_ARGS(!args.flow_path_outfile.empty(), "flow_path is required.");
            VALIDATE_ARGS(0 <= args.link_fail_rate && args.link_fail_rate <= 1., "link_fail_rate must be between 0 and 1.");
            break;
        case SimulationMode::LP_INPUT:
            VALIDATE_ARGS(!args.lp_input_outfile.empty(), "lp_input is required.");
            break;
        case SimulationMode::LP_WITH_ROUTES:
            VALIDATE_ARGS(!args.lp_input_outfile.empty(), "lp_input is required.");
            VALIDATE_ARGS(!args.shortest_path_outfile.empty(), "shortest_path is required.");
            VALIDATE_ARGS(!args.flow_path_outfile.empty(), "flow_path is required.");
            VALIDATE_ARGS(0 <= args.link_fail_rate && args.link_fail_rate <= 1., "link_fail_rate must be between 0 and 1.");
            break;
        default:
            throw std::runtime_error("Unsupported simulation mode" + toString(args.simulation_mode));
    }

    switch (args.graph_type) {
        case GraphType::GRAPH_FROM_FILE:
            break;
        case GraphType::FAT_TREE:
            VALIDATE_ARGS(args.server_uplinks == 1, "Multi-uplink is not supported for Fat-Tree");
            break;
        case GraphType::JELLYFISH:
            VALIDATE_ARGS(args.link_count > 0, "link_count is required.");
            break;
        case GraphType::TORUS:
            VALIDATE_ARGS(args.grid_size > 1, "grid_size is required.");
            VALIDATE_ARGS(args.server_ports > 0, "server_ports is required.");
            VALIDATE_ARGS(args.dimension > 1, "dimension is required.");
            VALIDATE_ARGS(args.server_uplinks == 1, "Multi-uplink is not supported for Torus");
            if (args.switch_radix == 0)
                args.switch_radix = args.dimension + args.server_ports;
            break;
        default:
            VALIDATE_ARGS(false, "Unsupported graph type %s", toString(args.graph_type).c_str());
            break;
    }

    VALIDATE_ARGS(
#if SIM_INCLUDE_SERVERS
        args.switch_radix > 0
#else
        args.graph_type == GraphType::GRAPH_FROM_FILE || args.switch_radix > 0
#endif
                    , "Switch radix should be a positive integer.");
    VALIDATE_ARGS(args.K > 0, "K must be a positive integer.");
    VALIDATE_ARGS(args.host_count > 0, "host_count is required.");
    VALIDATE_ARGS(args.node_count > 0, "node_count is required.");
    VALIDATE_ARGS(args.link_speed_mbps > 0, "link_speed must be a positive integer.")

    if (args.traffic_type != TrafficType::FILE) {
        VALIDATE_ARGS_IF(args.traffic_mode == TrafficMode::SHORT || args.traffic_mode == TrafficMode::RPC, args.flow_rate == 0, "Cannot specify flow rate for short flow.");
        VALIDATE_ARGS_IF(args.traffic_mode == TrafficMode::BULK, args.flow_size == 0, "Cannot specify flow size for bulk flow.");
        if (args.transport_type != TransportType::MPTCP && args.transport_type != TransportType::MPDCTCP) {
            VALIDATE_ARGS(args.K == 1, "Cannot specify K for non-MPTCP");
            VALIDATE_ARGS(args.routing_protocol == RoutingProtocol::ECMP, "Cannot specify non-ECMP routing protocol for non-MPTCP");
        }

        if (args.routing_protocol == RoutingProtocol::LLSKR) {
            VALIDATE_ARGS(args.th_s > 0, "th_s must be positive for LLSKR routing");
            VALIDATE_ARGS(args.th_w > 0, "th_w must be positive for LLSKR routing");
        }

        VALIDATE_ARGS_IF(args.traffic_type == TrafficType::STRIDE, args.stride > 0 && args.stride < args.host_count, "Invalid stride value.");
    }

    return args;

invalid:
    VALIDATE_ARGS(false, "Invalid value '%s' for option '%c'", optarg, c);
#undef VALIDATE_ARGS
#undef VALIDATE_ARGS_IF
}
#pragma clang diagnostic pop

queue_type get_queue_type(TransportType transport_type, bool need_priority_queues) {
    if (need_priority_queues)
        return queue_type::ECN_PRIO;

    switch (transport_type) {
        case TransportType::TCP:
        case TransportType::MPTCP:
        case TransportType::DCTCP:
        case TransportType::MPDCTCP:
            return queue_type::ECN;
        default:
            throw std::runtime_error("Unsupported transport");
    }
}

static FluidSimulator *g_simulator = nullptr;

std::string get_signal_name(int signum) {
#if defined(__linux__)
    const char *signame = sys_siglist[signum];
#elif defined(__APPLE__)
    const char *signame = sys_signame[signum];
#else
#error "Unsupported signal on this platform"
#endif

    if (signum < 0 || signum >= NSIG || signame == nullptr)
        return strsignal(signum);
    else
        return ToUpper(std::string(signame));
}

static void siginfo_handler(int sig) {
    std::cerr << "Received interrupt SIG" << get_signal_name(sig) << std::endl;
    if (g_simulator != nullptr)
        g_simulator->ShowProgress();
}

static void sigint_handler(int sig) {
    std::cerr << "Received interrupt SIG" << get_signal_name(sig) << std::endl;
    if (g_simulator != nullptr)
        g_simulator->Stop();
}

static void install_signal_handler(FluidSimulator *s) {
    g_simulator = s;
    signal(SIGINT, sigint_handler);     // ctrl-c interrupt or kill -INT
#if !defined(__linux__) // SIGINFO not defined on Linux
    signal(SIGINFO, siginfo_handler);   // ctrl-t or kill -INFO on macOS/BSD
#endif
    signal(SIGUSR1, siginfo_handler);   // kill -USR1 on Linux
}

int main(int argc, char *argv[]) {
#ifdef DEBUG
    std::cerr << "Include servers: " << SIM_INCLUDE_SERVERS << std::endl;
    std::cerr << "LP fair sharing: " << SIM_LP_FAIR_SHARING << std::endl;
    std::cerr << "Use first fit: " << USE_FIRST_FIT << std::endl;
#endif

    auto args = parse_args(argc, argv);
#if USE_CONSTANT_SEED
    std::cerr << "Using constant seed " << args.random_seed << std::endl;
    srand(args.random_seed);
#else
    std::cerr << "Using random seed ..." << std::endl;
    srand(time(NULL));  // for htsim
#endif

    RoutingParameter routing_parameter = {
        .protocol = args.routing_protocol,
        .k = args.K,
        .th_w = args.th_w,
        .th_s = args.th_s,
        .spreading_policy = args.llskr_spreading_policy
    };

    std::vector<Graph *> graphs;
    for (const auto& adjacency_list_file: args.adjacency_list_files) {
        Graph *graph;
        switch (args.graph_type) {
            case GraphType::GRAPH_FROM_FILE:
                graph = new GraphFromFile(args.switch_radix, args.server_uplinks, adjacency_list_file);
                break;
            case GraphType::FAT_TREE:
                graph = new FatTree(args.switch_radix);
                break;
            case GraphType::JELLYFISH:
                graph = new Jellyfish(args.switch_radix, args.node_count, args.link_count, args.server_uplinks);
                break;
            case GraphType::TORUS:
                graph = new Torus(args.grid_size, args.server_ports, args.dimension);
                break;
            default:
                throw std::runtime_error("Unsupported graph type");
        }
        if (args.link_fail_rate > 0.)
            graph->FailLinks(args.link_fail_rate);
        graphs.push_back(graph);
    }

    bool need_priority_queues = false;  // We need priority queues for short flows.
    std::vector<TrafficMatrix *> traffic_matrices;
    for (const auto& traffic_matrix_file: args.traffic_matrix_files) {
        TrafficMatrix *traffic_matrix;
        switch (args.traffic_type) {
            case TrafficType::ALL_TO_ALL:
            case TrafficType::PERMUTATION:
            case TrafficType::MANY_TO_ONE:
            case TrafficType::ONE_TO_MANY:
            case TrafficType::SINGLE:
                traffic_matrix = new TrafficMatrix(args.host_count, args.traffic_type, args.traffic_mode);
                break;
            case TrafficType::STRIDE:
                traffic_matrix = new TrafficMatrix(args.host_count, args.traffic_type, args.traffic_mode, args.stride);
                break;
            case TrafficType::FILE:
                std::cerr << "Initializing traffic matrix from file (ignoring command-line arguments) ..." << std::endl;
                traffic_matrix = new TrafficMatrix(traffic_matrix_file);
                break;
            case TrafficType::HADOOP: {
                // Assume all graphs have the same server layout.
                HadoopApplication *hadoop_app = new HadoopApplication(graphs.at(0), args.hadoop_app, args.hadoop_task);
                traffic_matrix = new TrafficMatrix(args.host_count, args.traffic_type, args.traffic_mode, hadoop_app);
                break;
            }
            default:
                throw std::runtime_error("Unsupported traffic matrix");
        }
        if (args.traffic_type != TrafficType::FILE) {
            traffic_matrix->SetTransport(args.transport_type);
            traffic_matrix->SetRoutingPolicy(routing_parameter);
            traffic_matrix->SetGroupSize(args.group_size);
            if (args.flow_size > 0)
                traffic_matrix->SetFlowSize(args.flow_size);
            if (args.flow_rate != 0)
                traffic_matrix->SetFlowRate(args.flow_rate);
            traffic_matrix->SetStartTime(args.start_time_ms);
            if (args.end_time_ms != 0)
                traffic_matrix->SetEndTime(args.end_time_ms);
            traffic_matrix->SetTrafficPriority(args.traffic_priority);
            if (args.repeat_count > 0) {
                traffic_matrix->SetRepeatCount(args.repeat_count);
                traffic_matrix->SetRepeatInterval(args.repeat_every_ms);
            }
        }
        traffic_matrices.push_back(traffic_matrix);
        if (traffic_matrix->GetTrafficPriority() == TrafficPriority::HIGH)
            need_priority_queues = true;
    }

    EventList event_list;
#if USE_FIRST_FIT
    FirstFit first_fit(timeFromMs(100), event_list, args.link_speed_mbps);
#endif
    Logfile logfile("logout.dat", event_list);
    struct htsim_config htsim_config = {
        .logfile = &logfile,
        .event_list = &event_list,
#if USE_FIRST_FIT
        .first_fit = &first_fit,
#endif
        .queue_type = get_queue_type(args.transport_type, need_priority_queues),
        .link_speed_mbps = args.link_speed_mbps,
        .queue_size_multiplier = args.queue_size_multiplier
    };

    FluidSimulator simulator(args.host_count, htsim_config, args.scheduling_mode);
    for (auto *graph: graphs)
        simulator.AddNetwork(graph);
    for (auto *traffic_matrix: traffic_matrices)
        simulator.AddTrafficMatrix(traffic_matrix);

    install_signal_handler(&simulator);
    simulator.Run(args.simulation_mode,
                  args.adjacency_list_files,
                  args.traffic_matrix_files,
                  args.shortest_path_outfile,
                  args.flow_path_outfile,
                  args.lp_input_outfile);

    return 0;
}
