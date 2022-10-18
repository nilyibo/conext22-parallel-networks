#ifndef GRAPH_FROM_FILE_H
#define GRAPH_FROM_FILE_H

#include <string>

#include "graph.h"

class GraphFromFile : public Graph {
private:
    std::string adjacency_list_file;

protected:
    void InitializeAdjacencyList() override;

public:
    GraphFromFile(size_t switch_radix, size_t server_uplinks, const std::string &adjacency_list_file);

    void OutputAdjacencyListToFile(const std::string &outfile) override;
};

#endif //GRAPH_FROM_FILE_H
