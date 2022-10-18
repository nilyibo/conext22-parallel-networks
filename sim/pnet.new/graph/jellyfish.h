#ifndef GRAPH_JELLYFISH_H
#define GRAPH_JELLYFISH_H

#include "graph.h"

class Jellyfish : public Graph {
private:
    bool TryInitializeAdjacencyList(size_t max_iterations);

protected:
    void InitializeAdjacencyList() override;

public:
    // N ToRs, each has k ports, of which r ports are used to connect to other ToRs
    Jellyfish(size_t switch_radix, size_t node_count, size_t link_count, size_t server_uplinks);
};

#endif //GRAPH_JELLYFISH_H
