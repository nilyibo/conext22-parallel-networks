#ifndef HADOOP_APPLICATION_H
#define HADOOP_APPLICATION_H

#include "config.h"
#include "eventlist.h"
#include "tcp.h"
#include "graph/graph.h"
#include "hadoop/hadoop_dataset.h"
#include "hadoop/hadoop_task.h"

enum HadoopApplicationType {
    HADOOP_APP_SORT = 1,
    HADOOP_APP_WORDCOUNT = 2,
    HADOOP_APP_REVERSE_LINKGRAPH = 3,
};

std::string toString(HadoopApplicationType hadoop_app_type);
HadoopApplicationType parseHadoopApplicationType(const std::string &value);

enum NodeAllocationPolicy {
    IN_SAME_RACKS = 1,      // Nodes are grouped in the same rack(s) when possible
    IN_DISTINCT_RACKS = 2,  // Nodes are separated into different racks(s) when possible
};

class HadoopApplication {
public:
    HadoopApplication(const Graph* graph, HadoopApplicationType app_type, HadoopTaskType task_type, NodeAllocationPolicy node_allocation_policy = NodeAllocationPolicy::IN_SAME_RACKS);
    Flowsets GenerateFlowsets() const;
    HadoopApplicationType GetApplicationType() const { return app_type_; }
    HadoopTaskType GetTaskType() const { return task_type_; }
    size_t GetHostCount() const { return host_count_; }

private:
    std::vector<size_t> AllocateNodes(size_t node_count, NodeAllocationPolicy allocation_policy) const;

    static constexpr size_t kDefaultWorkerCount = 32;
    static constexpr size_t kNumConcurrentFlowsPerNode = 4;

    const Graph* graph_;
    const HadoopApplicationType app_type_;
    const HadoopTaskType task_type_;
    const NodeAllocationPolicy node_allocation_policy_;
    const size_t host_count_;
    const size_t rack_count_;
    const size_t hosts_per_rack_;
    size_t input_data_size_;    // Map input total # of bytes
    size_t shuffle_data_size_;  // Mapper-to-reducer shuffle total # of bytes
    size_t output_data_size_;   // Reduce output total # of bytes
    size_t mapper_count_;
    size_t reducer_count_;
};

#endif // HADOOP_APPLICATION_H
