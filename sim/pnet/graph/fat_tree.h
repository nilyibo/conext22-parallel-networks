#ifndef GRAPH_FAT_TREE_H
#define GRAPH_FAT_TREE_H

#include <cassert>
#include <cmath>
#include <random>
#include <algorithm>
#include "graph.h"

class FatTree : public Graph {
private:
    const size_t K;
    const size_t kNumAggPerPod;
    // const size_t kNumToRPerPod;
    const size_t kNumCoreSwitches;
    const size_t kNumHostsPerToR;

    // The index-th ToR switch in the pod-th pod.
    //  @param pod [0, K/2)
    //  @param index [0, K/2)
    //  @return node_id [0, 1/2*K^2)
    [[nodiscard]] inline size_t GetNodeForToRSwitch(size_t pod, size_t index) const;

    // The index-th aggregation switch in the pod-th pod.
    //  @param pod [0, K/2)
    //  @param index [0, K/2)
    //  @return node_id [1/2*K^2, K^2)
    [[nodiscard]] inline size_t GetNodeForAggSwitch(size_t pod, size_t index) const;

    // The i-th core switch that the index-th agg switch in each pod should connect to.
    //  @param index [0, K/2)
    //  @param i [0, K/2)
    //  @return node_id [K^2, 5/4*K^2)
    [[nodiscard]] inline size_t GetNodeForCoreSwitch(size_t index, size_t i) const;

    // The i-th core switch index that the index-th agg switch in each pod should connect to.
    //  @param index [0, K/2)
    //  @param i [0, K/2)
    //  @return node_index [0, 1/4*K^2)
    [[nodiscard]] inline size_t GetIndexForCoreSwitch(size_t index, size_t i) const;

    // The id of the index-th core switch in the network.
    //  @param index [0, K*K/2)
    //  @return node_id [K^2, 5/4*K^2)
    [[nodiscard]] inline size_t GetNodeForCoreSwitch(size_t index) const;

    // Return the pod and index of the given ToR.
    //  @param id [0, 1/2*K^2)
    //  @return (pod, index) [0, K) and [0, K/2)
    [[nodiscard]] inline std::pair<size_t, size_t> GetTorPosition(size_t id) const;

    // Return the index of the aggregation switch connected to the core switch.
    //  @param core_index [0, 1/4*K^2)
    //  @return index [0, K)
    [[nodiscard]] inline size_t GetAggIndexToCore(size_t core_index) const;

    // Get k random intra-pod paths.
    //  Note: k must be less than or equal to # of agg switches in each pod, i.e. K / 2.
    [[nodiscard]] PathCollection GetIntraPodPaths(size_t pod, size_t src_index, size_t dst_index, int k) const;

    [[nodiscard]] PathCollection GetInterPodPaths(size_t src_pod, size_t src_index, size_t dst_pod, size_t dst_index, int k) const;

protected:
    void InitializeAdjacencyList() override;

    [[nodiscard]] PathCollection KShortestPaths(int src, int dst, int k) const override;

    void InitializeServerWeights() override;

    [[nodiscard]] virtual inline bool IsFatTree() const override {
        return true;
    }

public:
    FatTree(size_t K);
};

#endif //GRAPH_FAT_TREE_H
