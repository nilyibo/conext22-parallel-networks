#include "hadoop_application.h"

#include <iostream>
#include <map>

#include "utils.h"

static std::map<HadoopApplicationType, std::string> map_hadoop_app_type_string = {
    { HadoopApplicationType::HADOOP_APP_SORT, "sort" },
    { HadoopApplicationType::HADOOP_APP_WORDCOUNT, "wordcount" },
    { HadoopApplicationType::HADOOP_APP_REVERSE_LINKGRAPH, "linkgraph" },
};

std::string toString(HadoopApplicationType hadoop_app_type) {
    if (map_hadoop_app_type_string.find(hadoop_app_type) == map_hadoop_app_type_string.end())
        return std::string();

    return map_hadoop_app_type_string.at(hadoop_app_type);
}

HadoopApplicationType parseHadoopApplicationType(const std::string &value) {
    for (const auto &[hadooop_app_type, str]: map_hadoop_app_type_string) {
        if (str == value)
            return hadooop_app_type;
    }

    throw std::runtime_error("Unrecognized Hadoop application type.");
}

HadoopApplication::HadoopApplication(const Graph* graph, HadoopApplicationType app_type, HadoopTaskType task_type,
    NodeAllocationPolicy node_allocation_policy)
        : graph_(graph), app_type_(app_type), task_type_(task_type), node_allocation_policy_(node_allocation_policy),
          host_count_(graph->GetServerCount()), rack_count_(graph->GetNodeCount()), hosts_per_rack_(host_count_ / rack_count_) {
    if (host_count_ % rack_count_ != 0) throw std::runtime_error("Hadoop requires even # of servers per rack.");
    const size_t default_worker_count = min(kDefaultWorkerCount, graph_->GetServerCount());
    switch (app_type) {
        case HadoopApplicationType::HADOOP_APP_SORT:
            this->mapper_count_ = default_worker_count;
            this->reducer_count_ = default_worker_count;
            this->input_data_size_ = GB(1024);
            this->shuffle_data_size_ = GB(1024);
            this->output_data_size_ = GB(1024);
            break;
        case HadoopApplicationType::HADOOP_APP_WORDCOUNT:
            this->mapper_count_ = default_worker_count;
            this->reducer_count_ = 1;
            this->input_data_size_ = GB(50);
            this->shuffle_data_size_ = MB(4) * this->mapper_count_;
            this->output_data_size_ = MB(4);
            break;
        case HadoopApplicationType::HADOOP_APP_REVERSE_LINKGRAPH:
            this->mapper_count_ = default_worker_count;
            this->reducer_count_ = default_worker_count;
            this->input_data_size_ = GB(100);
            this->shuffle_data_size_ = GB(10);
            this->output_data_size_ = GB(5);
            break;
        default:
            throw std::runtime_error("Invalid app_type");
    }
}

Flowsets HadoopApplication::GenerateFlowsets() const {
    Flowsets flowsets;
    switch (task_type_) {
        case HadoopTaskType::HADOOP_MAP_READ_INPUT: {
            // Use dataset to determine placement of files
            size_t input_file_count = 1;
            size_t input_file_size = input_data_size_;
            HadoopDataset input_dataset(graph_, input_file_count, input_file_size);
            std::vector<size_t> workers = AllocateNodes(mapper_count_, node_allocation_policy_);
            std::cerr << "Allocated workers to hosts: " << workers << std::endl;

            std::vector<size_t> block_owners;
            for (size_t file_id = 0; file_id < input_file_count; file_id++) {
                const size_t block_count = input_dataset.GetBlockCount();
                for (size_t block_id = 0; block_id < block_count; block_id++) {
                    // TODO: find the closest replica
                    size_t host_id = input_dataset.GetHostForBlock(file_id, block_id, 1);
                    block_owners.push_back(host_id);
                }
            }
            ShuffleVector(block_owners);

            const size_t block_size = input_dataset.GetBlockSize();
            std::vector<size_t> num_blocks_per_worker = DistributeEvenly(block_owners.size(), workers.size());
            size_t total_blocks_read = 0;
            for (size_t worker_index = 0; worker_index < workers.size(); worker_index++) {
                size_t worker_host_id = workers.at(worker_index);
                size_t num_blocks_to_read = num_blocks_per_worker.at(worker_index);
                size_t start = total_blocks_read, end = total_blocks_read + num_blocks_to_read;
                total_blocks_read += num_blocks_to_read;
                std::vector<size_t> block_owners_for_worker(block_owners.begin() + start, block_owners.begin() + end);
                Flowset flowset;
                flowset.name = absl::StrFormat("mapper %zu@host %zu", worker_index, worker_host_id);
                flowset.concurrency = kNumConcurrentFlowsPerNode;
                for (size_t block_owner: block_owners_for_worker) {
                    Flow flow(block_owner, worker_host_id, block_size);
                    flowset.AddFlow(flow);
                }
                std::cerr << "Flowset " << flowset.name << " will fetch blocks from " << block_owners_for_worker << std::endl;

                if (flowset.flows.empty()) continue;
                flowsets.push_back(flowset);
            }
            break;
        }
        case HadoopTaskType::HADOOP_MAP_TO_REDUCE_SHUFFLE:{
            std::vector<size_t> mappers = AllocateNodes(mapper_count_, node_allocation_policy_);
            std::vector<size_t> reducers = AllocateNodes(reducer_count_, node_allocation_policy_);
            size_t flow_size = shuffle_data_size_ / mapper_count_ / reducer_count_;
            flow_size = max(flow_size, (size_t)Packet::data_packet_size());
            std::cerr << "Allocated mappers: " << mappers << std::endl;
            std::cerr << "Allocated reducers: " << reducers << std::endl;
            std::cerr << "Flow size: " << flow_size << std::endl;

            for (size_t i = 0; i < mappers.size(); i++) {
                size_t mapper = mappers.at(i);
                Flowset flowset;
                flowset.name = absl::StrFormat("shuffle %zu@host %zu", i, mapper);
                flowset.concurrency = kNumConcurrentFlowsPerNode;
                for (size_t reducer: reducers) {
                    Flow flow(mapper, reducer, flow_size);
                    flowset.AddFlow(flow);
                }
                std::cerr << "Flowset " << flowset.name << " will send shuffle data to all " << reducers.size() << " reducers." << std::endl;

                flowsets.push_back(flowset);
            }
            break;
        }
        case HadoopTaskType::HADOOP_REDUCE_OUTPUT: {
            // Use dataset to determine placement of files
            size_t output_file_count = 1;
            size_t output_file_size = output_data_size_;
            HadoopDataset output_dataset(graph_, output_file_count, output_file_size);
            std::vector<size_t> workers = AllocateNodes(reducer_count_, node_allocation_policy_);
            std::cerr << "Allocated workers to hosts: " << workers << std::endl;

            std::vector<size_t> block_owners;
            for (size_t file_id = 0; file_id < output_file_count; file_id++) {
                const size_t block_count = output_dataset.GetBlockCount();
                for (size_t block_id = 0; block_id < block_count; block_id++) {
                    // TODO: find the closest replica
                    size_t host_id = output_dataset.GetHostForBlock(file_id, block_id, 1);
                    block_owners.push_back(host_id);
                }
            }
            ShuffleVector(block_owners);

            const size_t block_size = output_dataset.GetBlockSize();
            std::vector<size_t> num_blocks_per_worker = DistributeEvenly(block_owners.size(), workers.size());
            size_t total_blocks_read = 0;
            for (size_t worker_index = 0; worker_index < workers.size(); worker_index++) {
                size_t worker_host_id = workers.at(worker_index);
                size_t num_blocks_to_read = num_blocks_per_worker.at(worker_index);
                size_t start = total_blocks_read, end = total_blocks_read + num_blocks_to_read;
                total_blocks_read += num_blocks_to_read;
                std::vector<size_t> block_owners_for_worker(block_owners.begin() + start, block_owners.begin() + end);
                Flowset flowset;
                flowset.name = absl::StrFormat("reducer %zu@host %zu", worker_index, worker_host_id);
                flowset.concurrency = kNumConcurrentFlowsPerNode;
                for (size_t block_owner: block_owners_for_worker) {
                    Flow flow(worker_host_id, block_owner, block_size);
                    flowset.AddFlow(flow);
                }
                std::cerr << "Flowset " << flowset.name << " will write blocks to " << block_owners_for_worker << std::endl;

                if (flowset.flows.empty()) continue;
                flowsets.push_back(flowset);
            }
            break;
        }
        default:
            SWITCH_DEFAULT_NO_IMPL;
    }

    return flowsets;
}

std::vector<size_t> HadoopApplication::AllocateNodes(size_t node_count, NodeAllocationPolicy allocation_policy) const {
    std::vector<size_t> nodes;
    switch (allocation_policy) {
        case NodeAllocationPolicy::IN_SAME_RACKS: {
            const size_t first_rack = GetRandomIndex(rack_count_);
            const size_t first_host = first_rack * hosts_per_rack_;
            for (size_t node_index = 0; node_index < node_count; node_index++) {
                const size_t node = (first_host + node_index) % host_count_;
                nodes.push_back(node);
            }
            break;
        }
        case NodeAllocationPolicy::IN_DISTINCT_RACKS: {
            CHECK(node_count <= host_count_, "Cannot allocate more nodes than the # of hosts we have");
            std::vector<size_t> num_hosts_per_rack = DistributeEvenly(node_count, rack_count_);
            size_t first_rack = GetRandomIndex(rack_count_);
            for (size_t rack_index = 0; rack_index < rack_count_; rack_index++) {
                const size_t num_hosts = num_hosts_per_rack.at(rack_index);
                const size_t rack_id = (first_rack + rack_index) % rack_count_;
                for (size_t host_index_in_rack = 0; host_index_in_rack < num_hosts; host_index_in_rack++) {
                    const size_t host_id = rack_id * hosts_per_rack_ + host_index_in_rack;
                    nodes.push_back(host_id);
                }
            }
            break;
        }
        default:
            SWITCH_DEFAULT_NO_IMPL;
    }

    return nodes;
}
