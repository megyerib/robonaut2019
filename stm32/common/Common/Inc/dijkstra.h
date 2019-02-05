#include <stdint.h>

typedef struct
{
    uint8_t startV;
    uint8_t endV;
    uint8_t cost;
} EDGE;

void dijkstra(EDGE* edges, int edgenum, int entry, int exit, uint8_t* path, int* pathlen);
