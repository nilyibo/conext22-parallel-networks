#include "fat_tree.h"

#include <cassert>
#include <cmath>
#include <random>
#include <algorithm>

// The index-th ToR switch in the pod-th pod.
//  @param pod [0, K/2)
//  @param index [0, K/2)
//  @return node_id [0, 1/2*K^2)
[[nodiscard]] inline size_t FatTree::GetNodeForToRSwitch(size_t pod, size_t index) const {
    return pod * K / 2 + index;
}

// The index-th aggregation switch in the pod-th pod.
//  @param pod [0, K/2)
//  @param index [0, K/2)
//  @return node_id [1/2*K^2, K^2)
[[nodiscard]] inline size_t FatTree::GetNodeForAggSwitch(size_t pod, size_t index) const {
    return K * K / 2 + pod * K / 2 + index;
}

// The i-th core switch that the index-th agg switch in each pod should connect to.
//  @param index [0, K/2)
//  @param i [0, K/2)
//  @return node_id [K^2, 5/4*K^2)
[[nodiscard]] inline size_t FatTree::GetNodeForCoreSwitch(size_t index, size_t i) const {
    return K * K + index * K / 2 + i;
}

// The i-th core switch index that the index-th agg switch in each pod should connect to.
//  @param index [0, K/2)
//  @param i [0, K/2)
//  @return node_id [0, 1/4*K^2)
[[nodiscard]] inline size_t FatTree::GetIndexForCoreSwitch(size_t index, size_t i) const {
    return index * K / 2 + i;
}

// The id of the index-th core switch in the network.
//  @param index [0, K*K/2)
//  @return node_id [K^2, 5/4*K^2)
[[nodiscard]] inline size_t FatTree::GetNodeForCoreSwitch(size_t index) const {
    return K * K + index;
}

// Return the pod and index of the given ToR.
//  @param id [0, 1/2*K^2)
//  @return (pod, index) [0, K) and [0, K/2)
[[nodiscard]] inline std::pair<size_t, size_t> FatTree::GetTorPosition(size_t id) const {
    size_t pod = id / (K / 2);
    size_t index = id % (K / 2);
    return std::make_pair(pod, index);
}

// Return the index of the aggregation switch connected to the core switch.
//  @param core_index [0, 1/4*K^2)
//  @return index [0, K)
[[nodiscard]] inline size_t FatTree::GetAggIndexToCore(size_t core_index) const {
    return core_index / (K / 2);
}

// Get k random intra-pod paths.
//  Note: k must be less than or equal to # of agg switches in each pod, i.e. K / 2.
[[nodiscard]] PathCollection FatTree::GetIntraPodPaths(size_t pod, size_t src_index, size_t dst_index, int k) const {
    int src = GetNodeForToRSwitch(pod, src_index);
    int dst = GetNodeForToRSwitch(pod, dst_index);

    PathCollection paths(kNumHostsPerToR * k);
    for (size_t i = 0; i < kNumHostsPerToR; i++) {    // i is the destination host index
        // Randomly pick k core switch, except the default one.
        auto permutation = GetRandomPermutation(kNumAggPerPod - 1, 1);
        for (size_t n = 0; n < k; n++) {
            int core_offset = n > 0 ? permutation[n - 1] : 0;
            // According to algorithm 1 of Al-Fares Fat-Tree paper
            size_t agg_index = (src_index + i + core_offset) % kNumAggPerPod;
            int agg = GetNodeForAggSwitch(pod, agg_index);
            Path p = { agg, dst };
            paths[i * k + n] = p;
        }
    }

    return paths;
}

[[nodiscard]] PathCollection FatTree::GetInterPodPaths(size_t src_pod, size_t src_index, size_t dst_pod, size_t dst_index, int k) const {
    PathCollection paths(kNumHostsPerToR * k);
    for (size_t i = 0; i < kNumHostsPerToR; i++) {    // i is the destination host index
        // Randomly pick k core switch, except the default one.
        auto permutation = GetRandomPermutation(kNumCoreSwitches - 1, 1);
        for (size_t n = 0; n < k; n++) {
            // According to algorithm 1 of Al-Fares Fat-Tree paper
            int tor_outport = (src_index + i) % (K / 2);
            int agg_index = tor_outport;
            int agg_outport = (i) % (K / 2);    // Modified from original paper for better load balancing.
            int core_index = GetIndexForCoreSwitch(agg_index, agg_outport);
            int core_offset = n > 0 ? permutation[n - 1] : 0;
            core_index = (core_index + core_offset) % kNumCoreSwitches;
            agg_index = GetAggIndexToCore(core_index);

            int core = GetNodeForCoreSwitch(core_index);
            int agg_src = GetNodeForAggSwitch(src_pod, agg_index);
            int agg_dst = GetNodeForAggSwitch(dst_pod, agg_index);  // Same mapping of agg to core in every pod
            int dst = GetNodeForToRSwitch(dst_pod, dst_index);
            Path p = { agg_src, core, agg_dst, dst };
            paths[i * k + n] = p;
        }
    }

    return paths;
}

void FatTree::InitializeAdjacencyList() {
    adjacency_list = std::vector<std::vector<int>>(node_count);

    for (size_t pod = 0; pod < K; pod++) {
        for (size_t agg_index = 0; agg_index < K / 2; agg_index++) {
            size_t agg_id = GetNodeForAggSwitch(pod, agg_index);
            for (size_t tor_index = 0; tor_index < K / 2; tor_index++) {
                size_t tor_id = GetNodeForToRSwitch(pod, tor_index);
                AddBidirectionNeighbor(agg_id, tor_id);
            }
            for (size_t i = 0; i < K / 2; i++) {
                size_t core_id = GetNodeForCoreSwitch(agg_index, i);
                AddBidirectionNeighbor(agg_id, core_id);
            }
        }
    }
}


[[nodiscard]] PathCollection FatTree::KShortestPaths(int src, int dst, int k) const {
    if (fail_probability != 0.)
        return Graph::KShortestPaths(src, dst, k);

    if (k > kNumCoreSwitches)
        throw std::runtime_error("Too few core switches for K shortest paths.");

    // ECMP is based on destination host index, so calculate all K/2 paths for each pair.
    // For Fat-Tree, choose upstream paths randomly. Downstream path is deterministic.
    auto src_pair = GetTorPosition(src);
    auto dst_pair = GetTorPosition(dst);
    auto src_pod = src_pair.first;
    auto src_index = src_pair.second;
    auto dst_pod = dst_pair.first;
    auto dst_index = dst_pair.second;

    if (src_pod == dst_pod) {   // Same pod
        if (k > kNumAggPerPod) {
            auto intra_pod_paths = GetIntraPodPaths(src_pod, src_index, dst_index, kNumAggPerPod);
            auto inter_pod_paths = GetInterPodPaths(src_pod, src_index, dst_pod, dst_index, k - kNumAggPerPod);
            std::copy(inter_pod_paths.begin(), inter_pod_paths.end(), std::back_inserter(intra_pod_paths));
            return intra_pod_paths;
        } else {
            return GetIntraPodPaths(src_pod, src_index, dst_index, k);
        }
    } else {
        return GetInterPodPaths(src_pod, src_index, dst_pod, dst_index, k);
    }
}

// Note: Only ToR switches have servers attached.
void FatTree::InitializeServerWeights() {
    server_weight = std::vector<int>(node_count, 0);
    for (size_t pod = 0; pod < K; pod++) {
        for (size_t tor_index = 0; tor_index < K / 2; tor_index++) {
            size_t tor_id = GetNodeForToRSwitch(pod, tor_index);
            server_weight[tor_id] = kNumHostsPerToR;
        }
    }
}

FatTree::FatTree(size_t K)
        : Graph("k=" + std::to_string(K) + " fat-tree",
                K,
                K * K / 2,          // Bottom layer
                K * K * 5 / 4,      // From bottom layer to top layer
                K * K * K / 2 * 2,  // Links are bidirectional
                1),
            K(K),
#if SIM_INCLUDE_SERVERS
            kNumHostsPerToR(K/2),
#else
            kNumHostsPerToR(1),
#endif
            kNumAggPerPod(K/2),
            //kNumToRPerPod(K/2),
            kNumCoreSwitches(K*K/4) {
    InitializeGraph();
    std::cerr << "Initialized Fat-Tree with K = " << K << std::endl;
}
