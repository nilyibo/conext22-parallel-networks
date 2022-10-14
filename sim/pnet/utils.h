#ifndef UTIL_H
#define UTIL_H

#include <algorithm>
#include <cstdlib>
#include <iterator>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "config.h"
#include "logfile.h"
#include "eventlist.h"
#if USE_FIRST_FIT
#include "first_fit.h"
#endif

/* enums and structs */

enum struct queue_type { RANDOM, ECN, ECN_PRIO, COMPOSITE, CTRL_PRIO, LOSSLESS, LOSSLESS_INPUT, LOSSLESS_INPUT_ECN };

struct htsim_config {
    Logfile* logfile = nullptr;
    EventList* event_list = nullptr;
#if USE_FIRST_FIT
    FirstFit* first_fit = nullptr;
#endif
    enum queue_type queue_type = queue_type::RANDOM;
    uint64_t link_speed_mbps = 0;
    uint64_t queue_size_multiplier = 1;
};

enum class GraphType {
    GRAPH_FROM_FILE,
    FAT_TREE,
    JELLYFISH,
    TORUS
};

std::string toString(GraphType graph_type);
GraphType parseGraphType(const std::string &value);

enum struct TransportType {
    TCP = 0,
    MPTCP = 1,
    DCTCP = 2,
    MPDCTCP = 3
};

std::string toString(TransportType transport);
TransportType parseTransportType(const std::string &value);

enum class RoutingProtocol {
    ECMP,   // Equal-cost multipath
    KSP,    // K-shortest path
    SPO,    // Shortest paths only
    LLSKR,  // Limited Length Spread k-shortest path routing
};

std::string toString(RoutingProtocol routing_protocol);
RoutingProtocol parseRoutingProtocol(const std::string &value);

enum struct LLSKRSpreadingPolicy {
    RANDOM,     // Hash onto random paths
    STRIDE,     // Pick one every k paths
    NETWORK,    // Hash onto different networks; shortest to longest into each network
};

std::string toString(LLSKRSpreadingPolicy spreading_policy);
LLSKRSpreadingPolicy parseLLSKRSpreadingPoliicy(const std::string &value);

struct RoutingParameter {
    RoutingProtocol protocol = RoutingProtocol::ECMP;
    size_t k = 1;
    size_t th_w = 0;    // LLSKR routing parameter
    size_t th_s = 0;    // LLSKR routing parameter
    LLSKRSpreadingPolicy spreading_policy = LLSKRSpreadingPolicy::RANDOM;

#define GET_VARIABLE_NAME(v) (#v)
    std::string Serialize() {
        std::ostringstream ss;
        ss << GET_VARIABLE_NAME(protocol) << "=" << toString(protocol);
        ss << "," << GET_VARIABLE_NAME(k) << "=" << k;
        if (protocol == RoutingProtocol::LLSKR) {
            ss << "," << GET_VARIABLE_NAME(th_w) << "=" << th_w;
            ss << "," << GET_VARIABLE_NAME(th_s) << "=" << th_s;
            ss << "," << GET_VARIABLE_NAME(spreading_policy) << "=" << toString(spreading_policy);
        }
        return ss.str();
    }

    void Deserialize(std::string s) {
        std::istringstream ss(s);
        while (ss.good()) {
            std::string pair, key, value;
            std::getline(ss, pair, ',');
            std::istringstream ss_pair(pair);
            std::getline(ss_pair, key, '=');
            std::getline(ss_pair, value);
            assert(!key.empty() && !value.empty());
            if (key == GET_VARIABLE_NAME(protocol)) {
                this->protocol = parseRoutingProtocol(value);
            } else if (key == GET_VARIABLE_NAME(k)) {
                std::stringstream ss_value(value);
                ss_value >> this->k;
            } else if (key == GET_VARIABLE_NAME(th_w)) {
                std::stringstream ss_value(value);
                ss_value >> this->th_w;
            } else if (key == GET_VARIABLE_NAME(th_s)) {
                std::stringstream ss_value(value);
                ss_value >> this->th_s;
            } else if (key == GET_VARIABLE_NAME(spreading_policy)) {
                this->spreading_policy = parseLLSKRSpreadingPoliicy(value);
            }
        }
    }
#undef GET_VARIABLE_NAME
};

std::string toString(RoutingParameter routing);

enum class TrafficPriority {
    LOW,
    HIGH,
};

std::string toString(TrafficPriority trafficPriority);
TrafficPriority parseTrafficPriority(const std::string &value);

bool IsMultipathTransport(TransportType transport_type);

/* Utility functions */

inline std::mt19937 GetRNG() {
#if USE_CONSTANT_SEED
    return std::mt19937(rand());    // Seeded in main with argument
#else
    return std::mt19937((std::random_device())());
#endif
}

// Generate a random permutation mapping table of the given length, the first one being ``start''.
std::vector<int> GetRandomPermutation(size_t length, int start = 0);

template <typename T>
void ShuffleVector(std::vector<T> &v) {
    static thread_local std::mt19937 rand_generator = GetRNG();
    std::shuffle(v.begin(), v.end(), rand_generator);
}

// Re-arrange the vector elements based on given order.
template <typename T>
void ReorderVector(std::vector<T> &v, const std::vector<int> &order) {
    assert(v.size() == order.size());
    std::vector<T> copy;
    copy.reserve(v.size());
    for (size_t n = 0; n < order.size(); n++) {
        copy.push_back(v.at(order.at(n)));
    }
    v = copy;
}

// Generate a random index in range [0, length)
int GetRandomIndex(size_t length);

// Generate a random index in range [lower, upper)
int GetRandomInRange(size_t lower, size_t upper);

// Generate a random number in range [0, max)
double GetRandomNumber(double max = 1.);

std::string HumanizeTime(simtime_picosec time);

std::string HumanizeTime(double time, const std::string &suffix);

std::string HumanizeSize(size_t size_in_bytes);

inline std::string ToUpper(const std::string &s) {
    std::string u(s);
    std::transform(u.begin(), u.end(), u.begin(), ::toupper);
    return u;
}

// Distribute items into buckets as evenly as possible.
// Returns the number of items in each bucket.
std::vector<size_t> DistributeEvenly(size_t item_count, size_t bucket_count, bool randomize = false);

template <typename T>
std::ostream& operator<<(std::ostream& out, const std::vector<T>& v) {
    if (v.empty()) {
        out << "[]";
        return out;
    }

    out << "[";
    std::copy(v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
    out << "\b\b]";
    return out;
}

/* macros */

#define NO_IMPL throw std::runtime_error("Not implemented");
#define SWITCH_DEFAULT_NO_IMPL throw std::runtime_error("Switch default case not implemented");

#define CHECK(condition, error_message) ({ if (!(condition)) { throw std::runtime_error(error_message); } })

/* maths helper functions */

constexpr uint64_t KB(uint64_t n) { return n * 1024; }
constexpr uint64_t MB(uint64_t n) { return n * 1024 * 1024; }
constexpr uint64_t GB(uint64_t n) { return n * 1024 * 1024 * 1024; }
constexpr uint64_t TB(uint64_t n) { return n * 1024 * 1024 * 1024 * 1024; }
constexpr uint64_t Kb(uint64_t n) { return n * 1024 / 8; }
constexpr uint64_t Mb(uint64_t n) { return n * 1024 * 1024 / 8; }
constexpr uint64_t Gb(uint64_t n) { return n * 1024 * 1024 * 1024 / 8; }
constexpr uint64_t Tb(uint64_t n) { return n * 1024 * 1024 * 1024 * 1024 / 8; }

#endif // UTIL_H
