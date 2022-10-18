#ifndef HADOOP_DATASET_H
#define HADOOP_DATASET_H

#include <cinttypes>
#include <functional>
#include <limits>

#include "graph/graph.h"
#include "absl/strings/str_format.h"

class HadoopDataset {
public:
    HadoopDataset(const Graph *graph, size_t file_count, size_t file_size,
                    size_t block_size = DEFAULT_BLOCK_SIZE,
                    int replica_count = DEFAULT_REPLICA_COUNT)
        : graph_(graph),
          file_count(file_count),
          file_size(file_size),
          block_size(block_size),
          replica_count(replica_count),
          nonce(GetRandomIndex(std::numeric_limits<int>::max()))
    {
        std::cout << absl::StrFormat("[Config] HadoopDataset: %" PRIu64 " files, %s/file, %s/block, %d replicas.\n",
                file_count, HumanizeSize(file_size), HumanizeSize(block_size), replica_count);
    }

    size_t GetBlockCount() const {
        return (size_t)ceil((double)this->file_size / (double)this->block_size);
    }

    size_t GetBlockSize() const {
        return this->block_size;
    }

    size_t GetSizeOfBlock(size_t block_id) const {
        if (block_id < GetBlockCount() - 1 || this->file_size % this->block_size == 0)
            return this->block_size;
        else
            return this->file_size % this->block_size;
    }

    size_t GetHostForBlock(size_t file_id, size_t block_id = 0, size_t replica_id = 0) const {
        if (file_id < 0 || file_id >= this->file_count)
            throw "file_id out of range";
        if (block_id < 0 or block_id >= this->GetBlockCount())
            throw "block_id out of range";
        if (replica_id < 0 || replica_id >= this->replica_count)
            throw "replica_id out of range";

        // Default placement per file, i.e. first replica.
        hash<size_t> hash_u64;
        auto default_host_id = hash_u64(file_id) % graph_->GetServerCount();
        if (replica_id == 0) {
            return default_host_id;
        } else {
            // Pick a different rack
            auto default_rack_id = graph_->GetSwitchForHost(default_host_id);
            auto rack_id = (hash_u64(file_id) ^ hash_u64(block_id) ^ nonce) % (graph_->GetNodeCount() - 1);
            rack_id = rack_id < default_rack_id ? rack_id : (rack_id + 1);
            auto host_index = (hash_u64(file_id) ^ hash_u64(block_id) ^ hash_u64(replica_id) ^ nonce) % graph_->GetHostCountAtSwitch(rack_id);
            return graph_->GetHostIndex(rack_id, host_index);
        }
    }

    static const size_t DEFAULT_BLOCK_SIZE = (size_t)MB(128);

private:
    static const int DEFAULT_REPLICA_COUNT = 3;

    const Graph* graph_;
    const size_t file_count;
    const size_t file_size;
    const size_t block_size;
    const int replica_count;
    const size_t nonce;
};

#endif  // HADOOP_DATASET_H
