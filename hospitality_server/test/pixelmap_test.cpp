#include <cstdio>
#include <cmath>
#include <vector>
#include <list>
#include "../include/pixelmap/pixelmap.hpp"


int main(void)
{
    PixelMap map;
    map.ReadMapBmp("/home/susung/Documents/04_S-HERO/map/");
    
    PixelMap::PixelIdx start, dest;
    start.row = 2700; start.col = 900; start.floor = 1;
    dest.row = 550; dest.col = 4020; dest.floor = 1;
    std::list<PixelMap::PixelIdx> pathContainer;
    map.DijkstraPath(pathContainer, start, dest);

    std::list<PixelMap::PixelIdx>::iterator iter;
    for (iter = pathContainer.begin(); iter != pathContainer.end(); iter++)
        printf("%d %d \n", iter->row, iter->col);
}