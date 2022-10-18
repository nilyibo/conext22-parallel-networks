#include "traffic_matrix.h"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <random>
#include <map>

#include "utils.h"

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include "absl/strings/str_format.h"

static std::map<TrafficType, std::string> map_traffic_type_string = {
        { TrafficType::ALL_TO_ALL,  "all-to-all" },
        { TrafficType::PERMUTATION, "permutation" },
        { TrafficType::STRIDE, "stride" },
        { TrafficType::MANY_TO_ONE, "many-to-one" },
        { TrafficType::ONE_TO_MANY, "one-to-many" },
        { TrafficType::SINGLE, "single" },
        { TrafficType::HADOOP, "hadoop" },
        { TrafficType::FILE, "file" },
};

std::string toString(TrafficType traffic_type) {
    if (map_traffic_type_string.find(traffic_type) == map_traffic_type_string.end())
        return std::string();
    //throw std::runtime_error("Unrecognized traffic type.");

    return map_traffic_type_string[traffic_type];
}

TrafficType parseTraffixType(const std::string &value) {
    for (const auto &pair: map_traffic_type_string) {
        const auto &t = pair.first;
        const auto &s = pair.second;
        if (s == value)
            return t;
    }

    throw std::runtime_error("Unrecognized traffic type");
}

static std::map<TrafficMode, std::string> map_traffic_mode_string = {
    { TrafficMode::BULK, "bulk" },
    { TrafficMode::SHORT, "short" },
    { TrafficMode::RPC, "rpc" },
};

std::string toString(TrafficMode traffic_mode) {
    if (map_traffic_mode_string.find(traffic_mode) == map_traffic_mode_string.end())
        return std::string();

    return map_traffic_mode_string[traffic_mode];
}

TrafficMode parseTrafficMode(const std::string &value) {
    for (const auto &pair: map_traffic_mode_string) {
        const auto &t = pair.first;
        const auto &s = pair.second;
        if (s == value)
            return t;
    }

    throw std::runtime_error("Unrecognized traffic mode");
}

int TrafficMatrix::id_count = 0;

DemandMatrix TrafficMatrix::ParseDemandMatrix(std::ifstream &ifs) {
    DemandMatrix demand_matrix;
    CHECK(size > 0, "Empty TM when parsing demand matrix.");
    demand_matrix.resize(size, std::vector<int>(size, 0));
    for (size_t row_index = 0; row_index < size; row_index++) {
        auto &row = demand_matrix.at(row_index);
        ifs.read(reinterpret_cast<char *>(&row[0]), size * sizeof(row[0]));
    }

    return demand_matrix;
}

Flowsets TrafficMatrix::ConvertLegacyDemandMatrixToFlowsets(const DemandMatrix &demand_matrix) {
    Flowsets flowsets;
    for (size_t src = 0; src < size; src++) {
        for (size_t dst = 0; dst < size; dst++) {
            int demand = demand_matrix.at(src).at(dst);
            if (demand == 0) continue;

            Flow flow(src, dst);
            if (this->trafficMode != TrafficMode::BULK) flow.flow_size = this->flow_size;
            flowsets.push_back(Flowset(absl::StrFormat("tm%zu.%zu->%zu", tm_id, src, dst), flow));
        }
    }

    return flowsets;
}

#define GET_VARIABLE_NAME(v) (#v)

void TrafficMatrix::ParseAttributeByKeyValuePair(const std::string &line, bool &binary_demand_matrix, bool &binary_flowsets) {
    if (line.at(0) == '#') return;  // Comment lines

    std::istringstream is_line(line);
    std::string key, value;
    if (std::getline(is_line, key, '=')) {
        if (std::getline(is_line, value)) {
            std::stringstream ss_value(value);
            size_t value_sst;
            if (key == GET_VARIABLE_NAME(size)) {
                ss_value >> this->size;
            } else if (key == GET_VARIABLE_NAME(trafficType)) {
                trafficType = parseTraffixType(value);
            } else if (key == GET_VARIABLE_NAME(trafficMode)) {
                trafficMode = parseTrafficMode(value);
            } else if (key == GET_VARIABLE_NAME(start_time_ms)) {
                ss_value >> this->start_time_ms;
            } else if (key == GET_VARIABLE_NAME(end_time_ms)) {
                ss_value >> this->end_time_ms;
            } else if (key == GET_VARIABLE_NAME(stride)) {
                ss_value >> value_sst;
                SetStride(value_sst);
            } else if (key == GET_VARIABLE_NAME(group_size)) {
                ss_value >> value_sst;
                SetGroupSize(value_sst);
            } else if (key == GET_VARIABLE_NAME(flow_size)) {
                ss_value >> value_sst;
                SetFlowSize(value_sst);
            } else if (key == GET_VARIABLE_NAME(flow_rate)) {
                ss_value >> value_sst;
                SetFlowRate(value_sst);
            } else if (key == GET_VARIABLE_NAME(transportType)) {
                this->transportType = parseTransportType(value);
            } else if (key == GET_VARIABLE_NAME(routing)) {
                this->routing.Deserialize(value);
            } else if (key == GET_VARIABLE_NAME(traffic_priority)) {
                this->traffic_priority = parseTrafficPriority(value);
            } else if (key == GET_VARIABLE_NAME(repeat_count)) {
                ss_value >> value_sst;
                SetRepeatCount(value_sst);
            } else if (key == GET_VARIABLE_NAME(repeat_every_ms)) {
                double v;
                ss_value >> v;
                SetRepeatInterval(v);
            } else {
                throw std::runtime_error(absl::StrFormat("TrafficMatrix::Deserialize: Unrecognized key \"%s\" and value \"%s\"", key, value));
            }
        } else {    // No value read
            if (key == "demandMatrix") {    // legacy key
                binary_demand_matrix = true;
            } else if (key == GET_VARIABLE_NAME(flowsets)) {
                binary_flowsets = true;
            } else {
                throw std::runtime_error(absl::StrFormat("TrafficMatrix::Deserialize: Unrecognized key \"%s\"", key));
            }
        }
    } else {    // Shouldn't happen, failed to find "="
        throw std::runtime_error(absl::StrFormat("Invalid line \"%s\", not a key-value pair.", line));
    }
}

void TrafficMatrix::Serialize(const std::string &outfile) {
    std::cerr << "Serializing traffic matrix to \"" << outfile << "\" ..." << std::endl;

    std::ofstream out(outfile);
    if (!out.is_open())
        throw std::runtime_error("Traffic Matrix: cannot open file for serialization");

    out << "# Traffic Matrix" << std::endl;
    out << GET_VARIABLE_NAME(size) << "=" << size << std::endl;
    out << GET_VARIABLE_NAME(trafficType) << "=" << toString(trafficType) << std::endl;
    out << GET_VARIABLE_NAME(trafficMode) << "=" << toString(trafficMode) << std::endl;
    if (trafficType == TrafficType::STRIDE)
        out << GET_VARIABLE_NAME(stride) << "=" << stride << std::endl;
    out << GET_VARIABLE_NAME(start_time_ms) << "=" << start_time_ms << std::endl;
    out << GET_VARIABLE_NAME(end_time_ms) << "=" << end_time_ms << std::endl;
    out << GET_VARIABLE_NAME(group_size) << "=" << group_size << std::endl;
    if (trafficMode == TrafficMode::SHORT || trafficMode == TrafficMode::RPC)
        out << GET_VARIABLE_NAME(flow_size) << "=" << flow_size << std::endl;
    if (trafficMode == TrafficMode::BULK && flow_rate > 0)
        out << GET_VARIABLE_NAME(flow_rate) << "=" << flow_rate << std::endl;
    out << GET_VARIABLE_NAME(transportType) << "=" << toString(transportType) << std::endl;
    out << GET_VARIABLE_NAME(routing) << "=" << routing.Serialize() << std::endl;
    out << GET_VARIABLE_NAME(traffic_priority) << "=" << toString(traffic_priority) << std::endl;
    if ((trafficMode == TrafficMode::SHORT || trafficMode == TrafficMode::RPC) && repeat_count > 0) {
        out << GET_VARIABLE_NAME(repeat_count) << "=" << repeat_count << std::endl;
        out << GET_VARIABLE_NAME(repeat_every_ms) << "=" << repeat_every_ms << std::endl;
    }

    // Note: we removed demand matrix from file output and keep it only for loading legacy files.
    bool write_flowsets;
    switch (trafficType) {
        case TrafficType::ALL_TO_ALL:
        case TrafficType::STRIDE:
            write_flowsets = false;
            break;
        case TrafficType::PERMUTATION:
        case TrafficType::MANY_TO_ONE:
        case TrafficType::ONE_TO_MANY:
        case TrafficType::SINGLE:
        case TrafficType::HADOOP:
            write_flowsets = true;
            break;
        default:
            throw std::runtime_error("Unsupported traffic type");
    }

    if (write_flowsets) {
        out << GET_VARIABLE_NAME(flowsets) << "=" << std::endl;
        boost::archive::binary_oarchive oav(out);
        oav << flowsets;
        std::cerr << "Wrote " << flowsets.size() << " flowsets." << std::endl;
    }

    out.close();
}

void TrafficMatrix::Deserialize(const std::string &infile) {
    std::cerr << "Deserializing traffic matrix from \"" << infile << "\" ..." << std::endl;

    std::ifstream ifs(infile, std::fstream::binary);
    if (!ifs.is_open())
        throw std::runtime_error("Traffic Matrix: cannot open file for deserialization");

    std::string line;
    bool binary_demand_matrix = false;
    bool binary_flowsets = false;
    // read header first, key-value pair text
    while (!(binary_demand_matrix || binary_flowsets) && std::getline(ifs, line)) {
        ParseAttributeByKeyValuePair(line, binary_demand_matrix, binary_flowsets);
    }

    CHECK(ifs.good(), "Unexpected error or eof.");

    if (binary_demand_matrix) {
        auto demand_matrix = ParseDemandMatrix(ifs);
        this->flowsets = ConvertLegacyDemandMatrixToFlowsets(demand_matrix);
        std::getline(ifs, line);    // Legacy files have an extra new line.
        CHECK(line.empty(), absl::StrFormat("Read non-empty trailing line \"%s\"", line));
        std::cerr << "Converted demand matrix to " << flowsets.size() << " flowsets." << std::endl;
    } else if (binary_flowsets) {
        boost::archive::binary_iarchive iav(ifs);
        iav >> flowsets;
        std::cerr << "Read " << flowsets.size() << " flowsets." << std::endl;
    } else {
        // No trailing binary data
    }

    // Check stream ends correctly, without trailing data.
    // Note: need to peek for ifs to set EOF flag.
    CHECK(ifs && ifs.peek() == EOF, "Read error occurred or unexpected trailing data (non-eof).");
    ifs.close();
}

#undef GET_VARIABLE_NAME

void TrafficMatrix::GenerateFlowsets(size_t lower, size_t upper, Flowsets &flowsets) {
    if (trafficMode == TrafficMode::BULK)
        CHECK(flow_size == 0, "flow_size must be zero for traffic_mode bulk");
    else
        CHECK(flow_size > 0, "Must specify flow_size for traffic_mode short");

    size_t group_size = upper - lower;
    CHECK(group_size >= 2, "Need at least two nodes per group in TM.");
    switch (trafficType) {
        case TrafficType::ALL_TO_ALL: {
            size_t additional_size = group_size * (group_size - 1);
            flowsets.reserve(flowsets.size() + additional_size);
            for (size_t src = lower; src < upper; src++) {
                for (size_t dst = lower; dst < upper; dst++) {
                    if (src == dst) continue;
                    Flow flow(src, dst, flow_size);
                    flowsets.push_back(Flowset(absl::StrFormat("tm%zu.%zu->%zu", tm_id, src, dst), flow));
                }
            }
            break;
        }
        case TrafficType::PERMUTATION: {
            size_t additional_size = group_size;
            flowsets.reserve(flowsets.size() + additional_size);
            std::vector<int> permutation = GetRandomPermutation(group_size);
            for (size_t index = 0; index < permutation.size() / 2; index++) {
                size_t first = lower + permutation.at(2 * index);
                size_t second = lower + permutation.at(2 * index + 1);

                std::vector<Flow> flows({
                    Flow(first, second, flow_size),
                    Flow(second, first, flow_size)
                });
                flowsets.push_back(Flowset(absl::StrFormat("tm%zu.%zu->%zu", tm_id, first, second), flows));
            }
            break;
        }
        case TrafficType::STRIDE: {
            for (size_t i = lower; i < upper; i++) {
                size_t src = i;
                size_t dst = (i + stride) % group_size;
                Flow flow(src, dst, flow_size);
                flowsets.push_back(Flowset(absl::StrFormat("tm%zu.%zu->%zu", tm_id, src, dst), flow));
            }
            break;
        }
        case TrafficType::MANY_TO_ONE: {
            int dst = GetRandomInRange(lower, upper);
            for (size_t index = lower; index < upper; index++) {
                Flow flow(index, dst, flow_size);
                flowsets.push_back(Flowset(absl::StrFormat("tm%zu.%zu->%zu", tm_id, index, dst), flow));
            }
            break;
        }
        case TrafficType::ONE_TO_MANY: {
            int src = GetRandomInRange(lower, upper);
            for (size_t index = lower; index < upper; index++) {
                Flow flow(src, index, flow_size);
                flowsets.push_back(Flowset(absl::StrFormat("tm%zu.%zu->%zu", tm_id, src, index), flow));
            }
            break;
        }
        case TrafficType::SINGLE: {
            std::vector<int> permutation = GetRandomPermutation(group_size);
            size_t src = permutation.at(0);
            size_t dst = permutation.at(1);
            Flow flow(src, dst, flow_size);
            flowsets.push_back(Flowset(absl::StrFormat("tm%zu.%zu->%zu", tm_id, src, dst), flow));
            break;
        }
        case TrafficType::HADOOP: {
            CHECK(!this->flowsets.empty(), "Hadoop TM should have flowsets pre-generated");
            break;
        }
        case TrafficType::FILE:
            throw std::runtime_error("Flowsets should have been loaded from file.");
        default:
            SWITCH_DEFAULT_NO_IMPL;
    }
}

TrafficMatrix::TrafficMatrix(size_t size, TrafficType trafficType, TrafficMode trafficMode): tm_id(id_count++) {
    this->size = size;
    this->trafficType = trafficType;
    this->trafficMode = trafficMode;
    this->stride = 0;
    this->group_size = size;
    SetDefaultEndTime();
    std::cerr << "Initialized traffic matrix: "
                    "# of nodes = " << size << ", "
                    "traffic type = " << toString(trafficType) << ", "
                    "traffic mode = " << toString(trafficMode) << std::endl;
    std::cout << "TM[" << this->tm_id << "] size = " << size << std::endl;
    std::cout << "TM[" << this->tm_id << "] trafficType = " << toString(trafficType) << std::endl;
    std::cout << "TM[" << this->tm_id << "] trafficMode = " << toString(trafficMode) << std::endl;
}

TrafficMatrix::TrafficMatrix(size_t size, TrafficType trafficType, TrafficMode trafficMode, size_t stride): tm_id(id_count++) {
    assert(trafficType == TrafficType::STRIDE);
    this->size = size;
    this->trafficType = trafficType;
    this->trafficMode = trafficMode;
    this->group_size = size;
    SetDefaultEndTime();
    std::cerr << "Initialized traffic matrix: "
                    "# of nodes = " << size << ", "
                    "traffic type = " << toString(trafficType) << ", "
                    "traffic mode = " << toString(trafficMode) << std::endl;
    std::cout << "TM[" << this->tm_id << "] size = " << size << std::endl;
    std::cout << "TM[" << this->tm_id << "] trafficType = " << toString(trafficType) << std::endl;
    std::cout << "TM[" << this->tm_id << "] trafficMode = " << toString(trafficMode) << std::endl;
    SetStride(stride);
    std::cout << "TM[" << this->tm_id << "] stride = " << stride << std::endl;
}

TrafficMatrix::TrafficMatrix(size_t size, TrafficType trafficType, TrafficMode trafficMode, const HadoopApplication *hadoop_app): tm_id(id_count++), hadoop_app(hadoop_app) {
    assert(trafficType == TrafficType::HADOOP);
    if (hadoop_app->GetHostCount() != size) throw std::runtime_error("Hadoop size and TM size mismatch.");
    this->size = size;
    this->trafficType = trafficType;
    this->trafficMode = trafficMode;
    this->group_size = size;
    SetDefaultEndTime();
    std::cerr << "Initialized traffic matrix: "
                    "# of nodes = " << size << ", "
                    "traffic type = " << toString(trafficType) << ", "
                    "traffic mode = " << toString(trafficMode) << ", "
                    "hadoop app = " << toString(hadoop_app->GetApplicationType()) << ", "
                    "hadoop task = " << toString(hadoop_app->GetTaskType()) << ", " << std::endl;
    std::cout << "TM[" << this->tm_id << "] size = " << size << std::endl;
    std::cout << "TM[" << this->tm_id << "] trafficType = " << toString(trafficType) << std::endl;
    std::cout << "TM[" << this->tm_id << "] trafficMode = " << toString(trafficMode) << std::endl;
    std::cout << "TM[" << this->tm_id << "] hadoop app = " << toString(hadoop_app->GetApplicationType()) << std::endl;
    std::cout << "TM[" << this->tm_id << "] hadoop task = " << toString(hadoop_app->GetTaskType()) << std::endl;
}

TrafficMatrix::TrafficMatrix(const std::string &infile): tm_id(id_count++), infile(infile) {
    std::cerr << "Initializing traffic matrix from file \"" << infile << "\" ..." << std::endl;
    Deserialize(infile);
    SetDefaultEndTime();
    std::cerr << "Initialized traffic matrix: "
                    "# of nodes = " << size << ", "
                    "traffic type = " << toString(trafficType) << ", "
                    "traffic mode = " << toString(trafficMode) << std::endl;
    std::cout << "TM[" << this->tm_id << "] size = " << size << std::endl;
    std::cout << "TM[" << this->tm_id << "] trafficType = " << toString(trafficType) << std::endl;
    std::cout << "TM[" << this->tm_id << "] trafficMode = " << toString(trafficMode) << std::endl;
    if (this->trafficType == TrafficType::STRIDE)
        std::cout << "TM[" << this->tm_id << "] stride = " << stride << std::endl;
    std::cout << "TM[" << this->tm_id << "] transport_type = " << toString(transportType) << std::endl;
    std::cout << "TM[" << this->tm_id << "] routing = \n" << toString(routing) << std::endl;
    if (this->group_size > 0 && this->group_size != this->size)
        std::cout << "TM[" << this->tm_id << "] group_size = " << this->group_size << std::endl;
    if (trafficMode == TrafficMode::SHORT || trafficMode == TrafficMode::RPC)
        std::cout << "TM[" << this->tm_id << "] flow_size = " << this->flow_size << std::endl;
    if (trafficMode == TrafficMode::BULK)
        std::cout << "TM[" << this->tm_id << "] flow_rate = " << this->flow_rate << std::endl;
}

TrafficMatrix::~TrafficMatrix() {
    if (hadoop_app != nullptr) delete hadoop_app;
}

const Flowsets& TrafficMatrix::GenerateFlowsets(const std::string &outfile) {
    std::cerr << "Generating flowsets ..." << std::endl;

    if (!this->flowsets.empty())
        return this->flowsets;

    if (this->trafficType == TrafficType::HADOOP) {
        this->flowsets = hadoop_app->GenerateFlowsets();
        std::cout << "TM[" << this->tm_id << "] hadoop flowsets = " << flowsets.size() << std::endl;
    } else {
        for (size_t offset = 0; offset < size; offset += group_size) {
            size_t lower = offset;
            size_t upper = std::min(offset + group_size, size);
            GenerateFlowsets(lower, upper, this->flowsets);
        }
    }

    if (!IsFromFile())
        Serialize(outfile);
    return flowsets;
}

Flowsets TrafficMatrix::MergeFlowsetsList(const std::vector<Flowsets> &flowsets_list) {
    if (flowsets_list.empty())
        return Flowsets();
    Flowsets merged_flowsets;
    for (const Flowsets& flowsets: flowsets_list) {
        std::copy(flowsets.begin(), flowsets.end(), std::back_inserter(merged_flowsets));
    }

    return merged_flowsets;
}
