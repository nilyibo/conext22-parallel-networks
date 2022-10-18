#include "torus.h"

void Torus::InitializeAdjacencyList() {
    // Layout: from bottom to top, from left to right. id = row * g + col
    // E.g. top: (g-1)*g, (g-1)*g + 1, ... (g-1)*g+(g-1)
    //      bot: 0, 1, ..., g-1

    if (dimension != 2)
        throw std::runtime_error("Not implemented: non-2D Torus is not supported.");

    adjacency_list = std::vector<std::vector<int>>(node_count);

    // Each node connects to four neighbors, with wrap-around on edges
    const int g = grid_size;
    for (int row = 0; row < g; row++) {
        for (int col = 0; col < g; col++) {
            int n = GetNodeInGrid(row, col);
            AddNeighbor(n, GetNodeInGrid(row + 1, col));    // Up
            AddNeighbor(n, GetNodeInGrid(row - 1, col));    // Down
            AddNeighbor(n, GetNodeInGrid(row, col - 1));    // Left
            AddNeighbor(n, GetNodeInGrid(row, col + 1));    //Right
        }
    }
}

Torus::Torus(size_t grid_size, size_t server_ports, size_t dimension)
        : Graph("grid_size=" + std::to_string(grid_size) + " torus",
                dimension + server_ports,
                grid_size * grid_size,
                grid_size * grid_size,
                grid_size * grid_size * dimension * 2,
                1),
            grid_size(grid_size),
        //   server_ports(server_ports),
            dimension(dimension) {
    InitializeGraph();
}
