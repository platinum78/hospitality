#include <cstdio>
#include "../include/graphmap/graphmap.hpp"
#include "../include/geometry_datatypes.hpp"

int main(void)
{
    GraphMap graphMap;
    graphMap.AddNode(1, "B1", (PointFloor<double>){ 1.0, 2.0, 1 });
    graphMap.AddNode(2, "B1", (PointFloor<double>){ 1.0, 2.0, 1 });
    graphMap.AddNode(3, "B1", (PointFloor<double>){ 1.0, 2.0, 1 });
    graphMap.AddNode(4, "B1", (PointFloor<double>){ 1.0, 2.0, 1 });
    graphMap.AddNode(5, "B1", (PointFloor<double>){ 1.0, 2.0, 1 });

    graphMap.ConnectNodes(1, 2, 1.0, 1.0);
    graphMap.ConnectNodes(2, 3, 1.0, 1.0);
    graphMap.ConnectNodes(3, 4, 1.0, 1.0);
    graphMap.ConnectNodes(4, 6, 1.0, 1.0);

    std::list<GraphMap::Node *> pathContainer;
    graphMap.FindRoute(pathContainer, 1, 5, GraphMap::PATH_DIJKSTRA);
    
    for (std::list<GraphMap::Node *>::iterator iter = pathContainer.begin();
         iter != pathContainer.end(); iter++)
        printf("%d \n", (*iter)->id_);
}