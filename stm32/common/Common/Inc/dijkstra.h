#include <stdint.h>

typedef struct
{
    int startV;
	int startExit; // bc of maze
	int endV;
	int endExit; // bc of maze
	int cost;
} EDGE;

void dijkstra(EDGE* edges, int edgenum, int entry, int exit, int* path, int* pathlen);
void dijkstraPathToNearestValid(EDGE* edges, int* valid, int edgenum, int entry, int* path, int* pathlen);
