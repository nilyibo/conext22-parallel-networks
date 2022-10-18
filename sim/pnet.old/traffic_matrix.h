#ifndef TRAFFIX_MATRIX_H
#define TRAFFIX_MATRIX_H

#include "mydefines.h"
#include "flowset.h"
#include "utils.h"
#include "applications/hadoop_application.h"

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

#include <cassert>
#include <iostream>
#include <sstream>
#include <vector>

enum class TrafficType {
    ALL_TO_ALL,
    PERMUTATION,
    STRIDE,
    MANY_TO_ONE,
    ONE_TO_MANY,
    SINGLE,
    HADOOP,
    FILE,
};

std::string toString(TrafficType traffic_type);
TrafficType parseTraffixType(const std::string &value);

enum class TrafficMode {
    BULK,   // bulk flow, measure throughput
    SHORT,  // short flow, measure completion time
    RPC,    // An RPC-like request response model
};

std::string toString(TrafficMode traffic_mode);
TrafficMode parseTrafficMode(const std::string &value);

template<typename T> using Matrix = std::vector<std::vector<T>>;
using DemandMatrix = Matrix<int>;

static constexpr simtime_picosec kMaxSimulationTime = timeFromSec(120);

class TrafficMatrix {
private:
    static int id_count;
    const int tm_id;
    static constexpr double DEFAULT_END_TIME_MS = 3001; // Run for 3.001s by default for bulk flows only.

    size_t size = 0;
    TrafficType trafficType = TrafficType::SINGLE;
    TrafficMode trafficMode = TrafficMode::BULK;
    double start_time_ms = 0;
    double end_time_ms = DEFAULT_END_TIME_MS;
    size_t stride = 0;
    size_t group_size = 0;
    size_t flow_size = 0;       // B, if non-bulk flow
    size_t flow_rate = 0;       // Mbps, if limiting flow rate
    Flowsets flowsets;  // A list of flow groups.
    TransportType transportType = TransportType::TCP;
    RoutingParameter routing;
    TrafficPriority traffic_priority = TrafficPriority::LOW;
    const std::string infile;
    size_t repeat_count = 0;
    double repeat_every_ms = 0.;
    const HadoopApplication *hadoop_app = nullptr;

    inline void SetDefaultEndTime() {
        if (this->trafficMode == TrafficMode::BULK)
            this->end_time_ms = DEFAULT_END_TIME_MS;
        else    // Short flows
            this->end_time_ms = timeAsMs(kMaxSimulationTime);
    }

    bool IsFromFile() {
        return !infile.empty();
    };

    inline void SetStride(size_t stride) {
        assert(trafficType == TrafficType::STRIDE);
        if (stride == 0 || stride > size)
            throw std::runtime_error("Invalid stride.");
        this->stride = stride;
    }

    void ParseAttributeByKeyValuePair(const std::string &line, bool &binary_demand_matrix, bool &binary_flowsets);

    DemandMatrix ParseDemandMatrix(std::ifstream &ifs);

    Flowsets ConvertLegacyDemandMatrixToFlowsets(const DemandMatrix &demand_matrix);

    void Serialize(const std::string &outfile);

    void Deserialize(const std::string &infile);

    // Generate flowsets for src/dst in a group range: [lower, upper)
    void GenerateFlowsets(size_t lower, size_t upper, Flowsets &flowsets);

public:
    TrafficMatrix(size_t size, TrafficType trafficType, TrafficMode trafficMode);

    TrafficMatrix(size_t size, TrafficType trafficType, TrafficMode trafficMode, size_t stride);

    TrafficMatrix(size_t size, TrafficType trafficType, TrafficMode trafficMode, const HadoopApplication *hadoop_app);

    TrafficMatrix(const std::string &infile);

    ~TrafficMatrix();

    inline int GetId() const {
        return this->tm_id;
    }

    inline std::string GetName() const {
        std::stringstream ss;
        ss << this->size << "-host ";
        ss << toString(this->trafficType) << " " << toString(this->trafficMode) << ": ";

        std::vector<std::string> details;
        if (this->trafficMode == TrafficMode::BULK)
            details.push_back("rate = " + std::to_string(flow_rate));
        if (this->trafficMode == TrafficMode::SHORT || this->trafficMode == TrafficMode::RPC)
            details.push_back("size = " + std::to_string(flow_size));
        if (this->group_size > 0)
            details.push_back("group = " + std::to_string(group_size));
        if (this->trafficType == TrafficType::STRIDE)
            details.push_back("stride = " + std::to_string(stride));
        for (size_t i = 0; i < details.size(); i++) {
            ss << details[i];
            if (i < details.size() - 1)
                ss << ", ";
        }

        return ss.str();
    }

    inline size_t GetSize() const {
        return size;
    }

    inline void SetTransport(TransportType transportType) {
        this->transportType = transportType;
        std::cout << "TM[" << this->tm_id << "] transport_type = " << toString(transportType) << std::endl;
    }

    inline TransportType GetTransport() const {
        return this->transportType;
    }

    inline void SetStartTime(double start_time_ms) {
        if (start_time_ms < 0)
            throw std::runtime_error("Invalid start time.");

        this->start_time_ms = start_time_ms;
        std::cout << "TM[" << this->tm_id << "] start_time_ms = " << this->start_time_ms << std::endl;
    }

    inline double GetStartTime(size_t repeat = 0) const {
        assert(repeat <= this->repeat_count);
        return this->start_time_ms + repeat * this->repeat_every_ms;
    }

    inline void SetEndTime(double end_time_ms) {
        if (end_time_ms < start_time_ms)
            throw std::runtime_error("Cannot set end time to be before start time.");
        if (this->trafficMode == TrafficMode::SHORT || this->trafficMode == TrafficMode::RPC)
            throw std::runtime_error("Cannot set end time for short flows.");

        this->end_time_ms = end_time_ms;
        std::cout << "TM[" << this->tm_id << "] end_time_ms = " << this->end_time_ms << std::endl;
    }

    inline double GetEndTime() const {
        return this->end_time_ms;
    }

    inline void SetRoutingPolicy(RoutingParameter routing) {
        this->routing = routing;
        std::cout << "TM[" << this->tm_id << "] routing =\n" << toString(routing) << std::endl;
    }

    inline RoutingParameter GetRoutingPolicy() const {
        return this->routing;
    }

    inline void SetGroupSize(size_t group_size) {
        assert(group_size > 0);

        this->group_size = std::min(this->size, group_size);
        std::cerr << "Group size set to " << this->group_size << std::endl;
        std::cout << "TM[" << this->tm_id << "] group_size = " << this->group_size << std::endl;
    }

    inline size_t GetGroupSize() const {
        return this->group_size;
    }

    inline void SetFlowSize(size_t size_in_bytes) {
        assert(size_in_bytes > 0);

        if (trafficMode != TrafficMode::SHORT && trafficMode != TrafficMode::RPC)
            throw std::runtime_error("Flow size is only valid for traffic_type short and rpc.");

        this->flow_size = size_in_bytes;
        std::cerr << "Flow size set to " << this->flow_size << std::endl;
        std::cout << "TM[" << this->tm_id << "] flow_size = " << this->flow_size << std::endl;
    }

    inline size_t GetFlowSize() const {
        return this->flow_size;
    }

    inline void SetFlowRate(size_t mbps) {
        assert(mbps > 0);

        if (trafficMode != TrafficMode::BULK)
            throw std::runtime_error("Flow rate is only valid for traffic_type bulk.");

        this->flow_rate = mbps;
        std::cerr << "Flow rate set to " << this->flow_rate << std::endl;
        std::cout << "TM[" << this->tm_id << "] flow_rate = " << this->flow_rate << std::endl;
    }

    inline size_t GetFlowRate() const {
        return this->flow_rate;
    }

    inline void SetTrafficPriority(TrafficPriority traffic_priority) {
        this->traffic_priority = traffic_priority;
        std::cout << "TM[" << this->tm_id << "] traffic_priority = " << toString(this->traffic_priority) << std::endl;
    }

    inline TrafficPriority GetTrafficPriority() const {
        return this->traffic_priority;
    }

    inline TrafficType GetTrafficType() const {
        return trafficType;
    }

    inline TrafficMode GetTrafficMode() const {
        return trafficMode;
    }

    inline void SetRepeatCount(size_t count) {
        if (count <= 0)
            throw std::runtime_error("Repeat count must be positive");

        this->repeat_count = count;
        std::cout << "TM[" << this->tm_id << "] repeat_count = " << this->repeat_count << std::endl;
    }

    inline size_t GetRepeatCount() const {
        return this->repeat_count;
    }

    inline void SetRepeatInterval(double every_ms) {
        if (every_ms <= 0.)
            throw std::runtime_error("Repeat interval must be positive");

        this->repeat_every_ms = every_ms;
        std::cout << "TM[" << this->tm_id << "] repeat_interval = " << this->repeat_every_ms << "ms" << std::endl;
    }

    inline double GetRepeatInterval() const {
        return this->repeat_every_ms;
    }

    const Flowsets& GenerateFlowsets(const std::string &outfile);

    const Flowsets& GetFlowsets() const {
        if (this->flowsets.empty())
            throw std::runtime_error("Must call GenerateFlowsets first.");

        return this->flowsets;
    }

    static Flowsets MergeFlowsetsList(const std::vector<Flowsets> &flowsets_list);
};

#endif  // TRAFFIX_MATRIX_H
