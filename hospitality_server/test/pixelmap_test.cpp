#include <cstdio>
#include <cmath>
#include <vector>
#include <list>
#include "../include/pixelmap/pixelmap.hpp"


int main(void)
{
    PixelMap map;
    map.ReadMapBmp("/home/susung/ros_workspace/src/hospitality/hospitality_server/data/map/");
    
    PixelMap::PixelIdx start, dest;
    start.row = 2700; start.col = 900; start.floor = 1;
    dest.row = 550; dest.col = 4020; dest.floor = 1;
    std::list<PixelMap::PixelIdx> pathContainer;
    map.DijkstraPath(pathContainer, start, dest);
    // map.ReducePath(pathContainer, PixelMap::PATH_REDUCTION_STRAIGHT_LINE);

    std::list<PixelMap::PixelIdx>::iterator iter;
    printf("%d \n", pathContainer.size());
    for (iter = pathContainer.begin(); iter != pathContainer.end(); iter++)
        printf("%d %d \n", iter->row, iter->col);
}