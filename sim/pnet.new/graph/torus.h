#ifndef GRAPH_TORUS_H
#define GRAPH_TORUS_H

#include "graph.h"

class Torus : public Graph {
private:
    const int grid_size;
    // const int server_ports;
    const int dimension;

    // Node id with wrap around.
    inline int GetNodeInGrid(int row, int col) {
        row = (row + grid_size) % grid_size;
        col = (col + grid_size) % grid_size;
        return row * grid_size + col;
    }

protected:
    void InitializeAdjacencyList() override;

public:
    Torus(size_t grid_size, size_t server_ports, size_t dimension);
};

#endif //GRAPH_TORUS_H
