#ifndef FLOWSET_H
#define FLOWSET_H

#include <cinttypes>
#include <fstream>
#include <string>
#include <vector>

// Describes a flow, with its src/dst, and size or end time, for limited-size short flow and unlimited-size long running flow, respectively.
struct Flow {
    size_t src_host;
    size_t dst_host;
    size_t flow_size;

    Flow() = default;

    Flow(size_t src_host, size_t dst_host): Flow(src_host, dst_host, 0) {}

    Flow(size_t src_host, size_t dst_host, size_t flow_size):
        src_host(src_host), dst_host(dst_host), flow_size(flow_size) {}

    template <class Archive>
    void serialize(Archive &ar, const unsigned int version) {
        ar & src_host & dst_host & flow_size;
    }

    friend std::ostream& operator<<(std::ostream& os, const Flow& f) {
        os << "src_host = " << f.src_host << ", ";
        os << "dst_host = " << f.dst_host << ", ";
        os << "flow_size = " << f.flow_size << ", ";
        return os;
    }
};

// Describes a set of flows with dependencies.
struct Flowset {
    std::string name;
    std::vector<Flow> flows;
    std::vector<ssize_t> parent_indices;
    size_t concurrency = std::numeric_limits<size_t>::max(); // how many flows can run at the same time.

    Flowset() = default;

    // Create a flowset with a single flow (and no dependency).
    Flowset(const std::string &name, Flow flow): name(name) {
        this->flows = { flow };
        this->parent_indices = { -1 };
    }

    // Create a flowset with a set of independent flows.
    Flowset(const std::string &name, std::vector<Flow> flows): name(name) {
        this->flows = flows;
        this->parent_indices = std::vector<ssize_t>(flows.size(), -1);
    }

    // Add an independent flow.
    void AddFlow(Flow flow) {
        this->flows.push_back(flow);
        this->parent_indices.push_back(-1);
    }

    // Add a flow that's dependent upon its previous flow in the flowset.
    void ChainFlow(Flow flow) {
        this->flows.push_back(flow);
        this->parent_indices.push_back(this->parent_indices.size() - 1);
    }

    void SetConcurrency(size_t concurrency) {
        this->concurrency = concurrency;
    }

    template <class Archive>
    void serialize(Archive &ar, const unsigned int version) {
        ar & name;
        ar & flows;
        ar & parent_indices;
        ar & concurrency;
    }

    friend std::ostream& operator<<(std::ostream& os, const Flowset& f) {
        os << "flowset" << std::endl;
        size_t length = f.flows.size();
        for (size_t i = 0; i < length; i++) {
            const auto &flow = f.flows.at(i);
            const ssize_t parent_index = f.parent_indices.at(i);
            os << "flow " << i << ": " << flow << ", "
                  "parent: " << parent_index << std::endl;
        }

        return os;
    }
};

using Flowsets=std::vector<Flowset>;

#endif  // FLOWSET_H
