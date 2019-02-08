#include "dijkstra.h"
#include <string.h>

#define MAX_VERTICES (32u)

static int isAllVerticesVisited(uint8_t* visited, int vnum);
static int nextPoint(uint8_t* visited, uint32_t* cost, int vnum);

void dijkstra(EDGE* edges, int edgenum, int entry, int exit, uint8_t* path, int* pathlen)
{
    uint8_t visited[MAX_VERTICES];
    uint32_t cost[MAX_VERTICES];
    uint8_t path_last[MAX_VERTICES];
    uint8_t tmp_path[MAX_VERTICES];

    int i;
    int chosenPoint;
    int tmp_path_len;
    int prevVertex;

    memset(visited, 1, sizeof(visited)); // Set to visited by default; Existing vertices will be set unvisited
    memset(cost, 0xFF, sizeof(cost));

    // Set existing vertices to unvisited
    for (i = 0; i < edgenum; i++)
    {
        visited[edges[i].startV] = 0;
        visited[edges[i].endV]   = 0;
    }

    cost[entry] = 0;
    chosenPoint = entry;

    // Calculate costs
    while (isAllVerticesVisited(visited, MAX_VERTICES) == 0)
    {
        for (i = 0; i < edgenum; i++)
        {
            if (edges[i].startV == chosenPoint)
            {
                if ((cost[chosenPoint] + edges[i].cost) < cost[edges[i].endV])
                {
                    cost[edges[i].endV] = (cost[chosenPoint] + edges[i].cost);
                    path_last[edges[i].endV] = i;
                }
            }
        }

        visited[chosenPoint] = 1;

        chosenPoint = nextPoint(visited, cost, MAX_VERTICES);
    }

    // Calculate path
    tmp_path_len = 0;
    prevVertex = exit;

    if (cost[exit] != 0xFFFFFFFF)
    {
        while (prevVertex != entry)
        {
            tmp_path[tmp_path_len] = path_last[prevVertex];
            prevVertex = edges[tmp_path[tmp_path_len]].startV;
            tmp_path_len++;
        }

        for (i = 0; i < tmp_path_len; i++)
        {
            path[i] = tmp_path[tmp_path_len - 1 - i];
        }
        *pathlen = tmp_path_len;
    }
    else
    {
        *pathlen = 0;
    }
}

void dijkstraPathToNearestValid(EDGE* edges, int* valid, int edgenum, int entry, uint8_t* path, int* pathlen)
{
	uint8_t visited[MAX_VERTICES];
	uint32_t cost[MAX_VERTICES];
	uint8_t path_last[MAX_VERTICES];
	uint8_t tmp_path[MAX_VERTICES];

	int i;
	int chosenPoint;
	int tmp_path_len;
	int prevVertex;

	memset(visited, 1, sizeof(visited)); // Set to visited by default; Existing vertices will be set unvisited
	memset(cost, 0xFF, sizeof(cost));

	// Set existing vertices to unvisited
	for (i = 0; i < edgenum; i++)
	{
		visited[edges[i].startV] = 0;
		visited[edges[i].endV]   = 0;
	}

	cost[entry] = 0;
	chosenPoint = entry;

	// Calculate costs
	while (isAllVerticesVisited(visited, MAX_VERTICES) == 0)
	{
		if (valid[chosenPoint] == 1)
		{
			break;
		}

		for (i = 0; i < edgenum; i++)
		{
			if (edges[i].startV == chosenPoint)
			{
				if ((cost[chosenPoint] + edges[i].cost) < cost[edges[i].endV])
				{
					cost[edges[i].endV] = (cost[chosenPoint] + edges[i].cost);
					path_last[edges[i].endV] = i;
				}
			}
		}

		visited[chosenPoint] = 1;

		chosenPoint = nextPoint(visited, cost, MAX_VERTICES);
	}

	// Calculate path
	tmp_path_len = 0;
	prevVertex = chosenPoint;

	if (cost[chosenPoint] != 0xFFFFFFFF)
	{
		while (prevVertex != entry)
		{
			tmp_path[tmp_path_len] = path_last[prevVertex];
			prevVertex = edges[tmp_path[tmp_path_len]].startV;
			tmp_path_len++;
		}

		for (i = 0; i < tmp_path_len; i++)
		{
			path[i] = tmp_path[tmp_path_len - 1 - i];
		}
		*pathlen = tmp_path_len;
	}
	else
	{
		*pathlen = 0;
	}
}

// STATIC --------------------------------------------------------------------------------------------------------------

static int isAllVerticesVisited(uint8_t* visited, int vnum)
{
    int i;

    for (i = 0; i < vnum; i++)
    {
        if (visited[i] == 0)
        {
            return 0;
        }
    }

    return 1;
}

static int nextPoint(uint8_t* visited, uint32_t* cost, int vnum)
{
    int i;
    int vertex = 0;
    uint32_t minCost = 0xFFFFFFFF;

    for (i = 0; i < vnum; i++)
    {
        if (visited[i] == 0 && cost[i] < minCost)
        {
            vertex = i;
            minCost = cost[i];
        }
    }

    return vertex;
}
