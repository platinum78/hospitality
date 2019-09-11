#ifndef MAP_SERVER_CPP_
#define MAP_SERVER_CPP_

#include <cstdio>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <hospitality/PointFloor.h>

#include "../include/graphmap/graphmap.hpp"
#include "../include/pixelmap/pixelmap.hpp"

class MapServerNode
{
public:
    MapServerNode();

private:
    GraphMap graph_map_;
    PixelMap pixel_map_;

private:
    int QueryPlaceHandler();

private:
    ros::NodeHandle n;
    ros::ServiceServer query_place_service_;
    std::string base_dir_;
};

MapServerNode::MapServerNode()
{
    base_dir_ = ros::package::getPath("hospitality_server");

    // query_place_service_ = n.advertiseService("/query_place_service", )
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_server_node");
    MapServerNode node;
}

#endif