#include "hadoop_task.h"

#include <iostream>
#include <map>

#include "utils.h"

#include "absl/strings/str_format.h"

static std::map<HadoopTaskType, std::string> map_hadoop_task_type_string = {
    { HadoopTaskType::HADOOP_MAP_READ_INPUT, "map-input" },
    { HadoopTaskType::HADOOP_MAP_TO_REDUCE_SHUFFLE, "shuffle" },
    { HadoopTaskType::HADOOP_REDUCE_OUTPUT, "reduce-output" },
};

std::string toString(HadoopTaskType hadoop_task_type) {
    if (map_hadoop_task_type_string.find(hadoop_task_type) == map_hadoop_task_type_string.end())
        return std::string();

    return map_hadoop_task_type_string.at(hadoop_task_type);
}

HadoopTaskType parseHadoopTaskType(const std::string &value) {
    for (const auto &[hadoop_task_type, str]: map_hadoop_task_type_string) {
        if (str == value)
            return hadoop_task_type;
    }

    throw std::runtime_error("Unrecognized Hadoop task type.");
}

size_t HadoopTask::global_task_id = 0;

HadoopTask::HadoopTask(size_t host_id, HadoopTaskType task_type)
        : task_id(global_task_id++), host_id(host_id), task_type(task_type), flow_complete_count(0) {
    this->completed_times_in_ms.resize(flow_count, std::numeric_limits<double>::max());
    std::cout << absl::StrFormat("Hadoop task at host %d started at %fms.", this->host_id, start_time_in_ms) << std::endl;
}

void HadoopTask::GenerateFlows(size_t flow_count, size_t flow_size_in_bytes, double start_time_in_ms) {
    this->flow_count = flow_count;
    this->flow_size_in_bytes = flow_size_in_bytes;
    this->start_time_in_ms = start_time_in_ms;
}

void HadoopTask::MarkFlowAsCompleted(size_t flow_index_in_task, double time_in_ms) {
    assert(flow_index_in_task < this->flow_count);
    this->completed_times_in_ms.at(flow_index_in_task) = time_in_ms;
    this->flow_complete_count++;
    std::cout << absl::StrFormat("Hadoop flow %d of task at host %d completed at %fms.", flow_index_in_task, this->host_id, time_in_ms) << std::endl;
    if (this->flow_complete_count == this->flow_count) {
        std::cout << absl::StrFormat("Hadoop task at host %d completed at %fms.", this->host_id, time_in_ms) << std::endl;
    }
}

HadoopFlowCallback::HadoopFlowCallback(EventList &eventlist, std::string name, HadoopTask *task, size_t flow_index_in_task)
        : EventSource(eventlist, name), task(task), flow_index_in_task(flow_index_in_task) {
    NO_IMPL;
}

void HadoopFlowCallback::doNextEvent() {
    NO_IMPL;
}
