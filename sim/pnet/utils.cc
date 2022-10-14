#include "utils.h"

#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "absl/strings/str_format.h"

static std::map<GraphType, std::string> map_graph_type_string = {
    { GraphType::GRAPH_FROM_FILE, "file" },
    { GraphType::FAT_TREE, "fat-tree" },
    { GraphType::JELLYFISH, "jellyfish" },
    { GraphType::TORUS, "torus" }
};

std::string toString(GraphType graph_type) {
    if (map_graph_type_string.find(graph_type) == map_graph_type_string.end())
        return std::string();
        //throw std::runtime_error("Unrecognized graph type.");

    return map_graph_type_string.at(graph_type);
}

GraphType parseGraphType(const std::string &value) {
    for (const auto &pair: map_graph_type_string) {
        const auto &t = pair.first;
        const auto &s = pair.second;
        if (s == value)
            return t;
    }

    throw std::runtime_error("Unrecognized graph type");
}

static std::map<TransportType, std::string> map_transport_type_string = {
    { TransportType::TCP, "tcp" },
    { TransportType::MPTCP, "mptcp" },
    { TransportType::DCTCP, "dctcp" },
    { TransportType::MPDCTCP, "mpdctcp" },
};

std::string toString(TransportType transport) {
    if (map_transport_type_string.find(transport) == map_transport_type_string.end())
        return std::string();

    return map_transport_type_string[transport];
}

TransportType parseTransportType(const std::string &value) {
    for (const auto &pair: map_transport_type_string) {
        if (pair.second == value)
            return pair.first;
    }

    throw std::runtime_error("Unrecognized transport");
}

static std::map<RoutingProtocol, std::string> map_routing_protocol_string = {
    { RoutingProtocol::ECMP, "ecmp" },
    { RoutingProtocol::KSP, "ksp" },
    { RoutingProtocol::SPO, "spo" },
    { RoutingProtocol::LLSKR, "llskr" },
};

std::string toString(RoutingProtocol routing_protocol) {
    if (map_routing_protocol_string.find(routing_protocol) == map_routing_protocol_string.end())
        return std::string();

    return map_routing_protocol_string[routing_protocol];
}

RoutingProtocol parseRoutingProtocol(const std::string &value) {
    for (const auto &pair: map_routing_protocol_string) {
        const auto &t = pair.first;
        const auto &s = pair.second;
        if (s == value)
            return t;
    }

    throw std::runtime_error("Unrecognized routing protocol");
}

static std::map<LLSKRSpreadingPolicy, std::string> map_llskr_spreading_policy_string = {
    { LLSKRSpreadingPolicy::RANDOM, "random" },
    { LLSKRSpreadingPolicy::STRIDE, "stride" },
    { LLSKRSpreadingPolicy::NETWORK, "network" },
};

std::string toString(LLSKRSpreadingPolicy spreading_policy) {
    if (map_llskr_spreading_policy_string.find(spreading_policy) == map_llskr_spreading_policy_string.end())
        return std::string();

    return map_llskr_spreading_policy_string.at(spreading_policy);
}

LLSKRSpreadingPolicy parseLLSKRSpreadingPoliicy(const std::string &value) {
    for (const auto &pair: map_llskr_spreading_policy_string) {
        if (pair.second == value)
            return pair.first;
    }

    throw std::runtime_error("Unrecognized LLSKR spreading policy");
}

std::string toString(RoutingParameter routing) {
    std::ostringstream oss;
    oss << "\tprotocol = " << toString(routing.protocol) << std::endl;
    oss << "\tk = " << routing.k << std::endl;
    if (routing.protocol == RoutingProtocol::LLSKR) {
        oss << "\tth_w = " << routing.th_w << std::endl;
        oss << "\tth_s = " << routing.th_s << std::endl;
        oss << "\tspreading = " << toString(routing.spreading_policy) << std::endl;
    }
    return oss.str();
}

std::map<TrafficPriority, std::string> map_traffic_priority_string = {
    { TrafficPriority::LOW, "low" },
    { TrafficPriority::HIGH, "high" },
};

std::string toString(TrafficPriority trafficPriority) {
    if (map_traffic_priority_string.find(trafficPriority) == map_traffic_priority_string.end())
        return std::string();

    return map_traffic_priority_string[trafficPriority];
}

TrafficPriority parseTrafficPriority(const std::string &value) {
    for (const auto &pair: map_traffic_priority_string) {
        if (pair.second == value)
            return pair.first;
    }

    throw std::runtime_error("Unrecognized traffic priority");
}

bool IsMultipathTransport(TransportType transport_type) {
    switch (transport_type) {
        case TransportType::TCP:
        case TransportType::DCTCP:
            return false;
        case TransportType::MPTCP:
        case TransportType::MPDCTCP:
            return true;
        default:
            SWITCH_DEFAULT_NO_IMPL;
    }
}

// Generate a random permutation mapping table of the given length.
std::vector<int> GetRandomPermutation(size_t length, int start) {
    static thread_local std::mt19937 rand_generator = GetRNG();
    std::vector<int> mapping(length);
    std::iota(mapping.begin(), mapping.end(), start);
    std::shuffle(mapping.begin(), mapping.end(), rand_generator);
    return mapping;
}

// Generate a random index in range [0, length)
int GetRandomIndex(size_t length) {
    static thread_local std::mt19937 rand_generator = GetRNG();
    std::uniform_int_distribution<> dist(0, length - 1);
    return dist(rand_generator);
}

// Generate a random index in range [lower, upper), inclusive
int GetRandomInRange(size_t lower, size_t upper) {
    static thread_local std::mt19937 rand_generator = GetRNG();
    std::uniform_int_distribution<> dist(lower, upper - 1);
    return dist(rand_generator);
}

// Generate a random number in range [0, max)
double GetRandomNumber(double max) {
    static thread_local std::mt19937 rand_generator = GetRNG();
    std::uniform_real_distribution<> dist(0, max);
    return dist(rand_generator);
}

std::string HumanizeTime(simtime_picosec time) {
    return absl::StrFormat("%fms", timeAsMs(time));
}

std::string HumanizeTime(double time, const std::string &suffix) {
    if (time == 0.)
        return "0";
    else if (time == std::numeric_limits<double>::max())
        return "inf";
    else
        return std::to_string(time) + suffix;
}

std::string HumanizeSize(size_t size_in_bytes) {
    if (size_in_bytes < KB(1))
        return absl::StrFormat("%gB", size_in_bytes);
    if (size_in_bytes < MB(1))
        return absl::StrFormat("%gkB", (double)size_in_bytes / KB(1));
    if (size_in_bytes < GB(1))
        return absl::StrFormat("%gMB", (double)size_in_bytes / MB(1));
    if (size_in_bytes < TB(1))
        return absl::StrFormat("%gGB", (double)size_in_bytes / GB(1));
    return absl::StrFormat("%gTB", (double)size_in_bytes / TB(1));
}

std::vector<size_t> DistributeEvenly(size_t item_count, size_t bucket_count, bool randomize) {
    static thread_local std::mt19937 rand_generator = GetRNG();
    size_t quotient = item_count / bucket_count;
    size_t remainder = item_count % bucket_count;
    std::vector<size_t> assignment(bucket_count, quotient);
    // std::for_each_n(assignment.begin(), remainder, [](auto& x){ x++; });
    std::for_each(assignment.begin(), assignment.begin() + remainder, [](auto& x){ x++; });
    if (randomize) {
        std::shuffle(assignment.begin(), assignment.end(), rand_generator);
    }

    return assignment;
}
