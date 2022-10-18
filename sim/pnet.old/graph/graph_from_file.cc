#include "graph_from_file.h"

#include <iostream>
#include <fstream>
#include <regex>
#include <map>

void GraphFromFile::InitializeAdjacencyList() {
    std::ifstream file(adjacency_list_file);
    if (!file.is_open())
        throw std::runtime_error("Failed to open adjacency list file.");

    std::cerr << "Initializing graph from adjacency list file ..." << std::endl;

    std::string line;
    std::regex r("c2_(\\d+)_(\\d+)");
    std::smatch matches;

    std::map<int, std::set<int>> adjacency_list_map;
    link_count = 0;
    while (file >> line) {
        if (!std::regex_search(line, matches, r)) {
            std::cerr << "Unrecognized line in adjacency list: " << line << std::endl;
            continue;
        }

        // for (auto &match: matches)
        //     std::cerr << "Match: " << match.str() << std::endl;

        if (matches.size() != 2 + 1)
            throw std::runtime_error("Incorrect number of matches for line: " + line + "."
                                                                                        " Found " +
                                        std::to_string(matches.size()) + " matches.");
        int src = std::stoi(matches[1].str());
        int dst = std::stoi(matches[2].str());
        adjacency_list_map[src].insert(dst);
        link_count++;
    }

    node_count = adjacency_list_map.size();
    switch_count = node_count;
    tor_count = node_count;
    adjacency_list = std::vector<std::vector<int>>(node_count);
    for (size_t node = 0; node < node_count; node++) {
        if (adjacency_list_map.find(node) == adjacency_list_map.end())
            throw std::runtime_error("Out of bound node id: " + std::to_string(node) + ", node count: " +
                                        std::to_string(node_count));

        adjacency_list[node] = std::vector<int>(adjacency_list_map[node].begin(), adjacency_list_map[node].end());
    }
}

GraphFromFile::GraphFromFile(size_t switch_radix, size_t server_uplinks, const std::string &adjacency_list_file)
        : Graph("file(" + adjacency_list_file + ")", switch_radix, server_uplinks),
            adjacency_list_file(adjacency_list_file) {
    InitializeGraph();
    std::cerr << "Initialized graph from file " << adjacency_list_file << std::endl;
}

void GraphFromFile::OutputAdjacencyListToFile(const std::string &outfile) {
    // Do nothing since the same file was read to create the graph.
}
