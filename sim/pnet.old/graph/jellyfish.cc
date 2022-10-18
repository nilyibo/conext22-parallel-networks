#include "jellyfish.h"

#include <vector>
#include <random>
#include <algorithm>
#include <functional>

bool Jellyfish::TryInitializeAdjacencyList(size_t max_iterations) {
    size_t iterations = 0;
    std::mt19937 rng = GetRNG();

    adjacency_list = std::vector<std::vector<int>>(node_count);

    // Spread evenly if possible, and then distribute the remainder to random nodes, each adding up to 1 port
    // Make sure all uplinks form each server go to the same ToR, thus assigning links by groups.
    // E.g. in 4x uplinks case, both server-to-ToR and ToR-to-ToR links are 4x.
    size_t factor = server_uplinks;
    size_t links_to_spread = link_count /  factor;
    std::vector<int> ports(node_count, links_to_spread / node_count);    // # of network ports per node
    std::transform(ports.begin(), ports.begin() + (links_to_spread % node_count), ports.begin(), [](int &e) { return e + 1; });
    std::transform(ports.begin(), ports.end(), ports.begin(), [factor](int &e) {return factor * e;});
    std::shuffle(ports.begin(), ports.end(), rng);

    // Generate random regular graph by pick a random pair of switches with free ports
    // Algorithm is described in Jellyfish NSDI'12 paper
    std::vector<int> tors(tor_count, 0);    // ToRs with free ports
    std::iota(tors.begin(), tors.end(), 0);

    while (tors.size() > 1) {
        if (iterations++ > max_iterations)
            return false;
        std::uniform_int_distribution<> dist1(0, tors.size() - 1);
        std::uniform_int_distribution<> dist2(0, tors.size() - 2);
        int index1 = dist1(rng);
        int index2 = dist2(rng);
        if (index2 >= index1)
            index2++;
        int tor1 = tors[index1];
        int tor2 = tors[index2];
        if (IsNeighbor(tor1, tor2, false))
            continue;

        AddBidirectionNeighbor(tor1, tor2);
        if (adjacency_list[tor1].size() == ports[tor1])
            tors.erase(tors.begin() + index1);
        if (adjacency_list[tor2].size() == ports[tor2])
            tors.erase(std::remove(tors.begin(), tors.end(), tor2), tors.end());
    }

    if (tors.size() == 1) {
        size_t tor = tors[0];

        // If a single switch remains with >= 2 free ports.
        while (ports[tor] - adjacency_list[tor].size() >= 2) {
            if (iterations++ > max_iterations)
                return false;

            // Break a uniform-random existing link (x,y)
            std::uniform_int_distribution<> dist1(0, tor_count - 2);
            int x = dist1(rng);
            if (x >= tor)
                x++;
            const auto &neighbors = adjacency_list[x];
            std::uniform_int_distribution<> dist2(0, neighbors.size() - 1);
            int y = neighbors[dist2(rng)];
            if (y == tor)
                continue;
            if (IsNeighbor(tor, x, false) || IsNeighbor(tor, y, false))
                continue;
            RemoveBidirectionNeighbor(x, y);
            AddBidirectionNeighbor(tor, x);
            AddBidirectionNeighbor(tor, y);
        }

        if (ports[tor] - adjacency_list[tor].size() == 1)
            std::cerr << "Warning: Jellyfish has one disconnected port." << std::endl;
    }

    // Set actual link_count
    link_count = 0;
    for (const auto &neighbors: adjacency_list)
        link_count += neighbors.size();
    return true;
}

void Jellyfish::InitializeAdjacencyList() {
    const int kMaxTries = 100;
    int tries = 0;
    while (tries < kMaxTries) {
        int iterations = node_count * 1000;
        if (TryInitializeAdjacencyList(iterations))
            return;
        tries++;
        std::cerr << "Jellyfish graph generation stopped after " << iterations << " iterations. Re-trying ..." << std::endl;
    }

    throw std::runtime_error("Failed to generate Jellyfish after " + std::to_string(tries) + " tries!");
}

Jellyfish::Jellyfish(size_t switch_radix, size_t node_count, size_t link_count, size_t server_uplinks)
        : Graph("k=" + std::to_string(switch_radix) + " jellyfish", switch_radix, node_count, link_count, server_uplinks) {
    InitializeGraph();
}
