#ifndef HADOOP_TASK_H
#define HADOOP_TASK_H

#include <cinttypes>
#include <limits>
#include <vector>

#include "eventlist.h"

enum HadoopTaskType {
    HADOOP_MAP_READ_INPUT = 1,
    HADOOP_MAP_TO_REDUCE_SHUFFLE = 2,
    HADOOP_REDUCE_OUTPUT = 3,
};

std::string toString(HadoopTaskType hadoop_task_type);
HadoopTaskType parseHadoopTaskType(const std::string &value);

class HadoopTask {
protected:
    HadoopTask(size_t host_id, HadoopTaskType task_type);
    void GenerateFlows(size_t flow_count, size_t flow_size_in_bytes, double start_time_in_ms);
    void MarkFlowAsCompleted(size_t flow_index_in_task, double time_in_ms);

protected:

private:
    static size_t global_task_id;

    size_t task_id;
    size_t host_id;
    size_t flow_count{};
    size_t flow_size_in_bytes{};
    double start_time_in_ms{};
    HadoopTaskType task_type;
    std::vector<double> completed_times_in_ms;
    size_t flow_complete_count;
};

// Callback function when flows finish
class HadoopFlowCallback : public EventSource {
public:
    HadoopFlowCallback(EventList &eventlist, std::string name, HadoopTask *task, size_t flow_index_in_task);
    void doNextEvent() override;

private:
    HadoopTask* task;
    size_t flow_index_in_task;
};

// class HadoopMapInputTask: HadoopTask {
// public:
//     HadoopMapInputTask(size_t host_id);
// };

// class HadoopMapTask : HadoopTask {
//     size_t file_id;
//     size_t block_id;

//     // HadoopMapTask() : HadoopTask(static_cast<size_t>(-1), 0, "map"),
//     //         file_id(static_cast<size_t>(-1)),
//     //         block_id(static_cast<size_t>(-1)) {}

//     HadoopMapTask(size_t host_id, double start_time_in_ms, size_t file_id, size_t block_id)
//             : HadoopTask(host_id, start_time_in_ms, "map"), file_id(file_id), block_id(block_id) {}

//     void ReadInput();
//     void Shuffle();
// };

// class HadoopReduceTask : HadoopTask {
//     std::vector<size_t> file_ids;

//     // HadoopReduceTask() : HadoopTask(static_cast<size_t>(-1), 0, "reduce") {}

//     HadoopReduceTask(size_t host_id, double start_time, std::vector<size_t> file_ids, std::string sub_task = "")
//             : HadoopTask(host_id, start_time, "reduce" + sub_task), file_ids(file_ids) {}

//     void SaveOutput();
// };

#endif  // HADOOP_TASK_H
