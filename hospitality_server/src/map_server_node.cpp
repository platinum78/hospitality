#ifndef MAP_SERVER_CPP_
#define MAP_SERVER_CPP_

#include <cstdio>
#include <string>

#include "ros/ros.h"
#include "ros/package.h"
#include "xmlrpcpp/XmlRpcValue.h"
#include "hospitality_msgs/PointFloor.h"
#include "hospitality_msgs/Place.h"
#include "hospitality_msgs/QueryPlace.h"
#include "hospitality_msgs/FindRoute.h"

#include "../include/map/graphmap.hpp"
#include "../include/map/pixelmap.hpp"

class MapServerNode
{
public:
    MapServerNode();
    // ~MapServerNode();

public:
    void ExecLoop();

public:
    enum { QUERY_ID, QUERY_CODE, QUERY_NAME };

private:
    GraphMap graph_map_;
    PixelMap pixel_map_;
    

private:
    bool QueryPlaceHandler(hospitality_msgs::QueryPlace::Request &req,
                           hospitality_msgs::QueryPlace::Response &resp);
    bool FindRouteHandler(hospitality_msgs::FindRoute::Request &req,
                          hospitality_msgs::FindRoute::Response &resp);

private:
    // PixelMap::Node *QueryPlaceHelper(int place_id);
    // PixelMap::Node *QueryPlaceHelper(std::string &str, int mode);

private:
    ros::NodeHandle n;
    ros::ServiceServer query_place_service_;
    ros::ServiceServer find_route_service_;
    std::string base_dir_;
    std::string map_bmp_path_;
    std::string map_csv_path_;
};

MapServerNode::MapServerNode()
{
    // Get parameters from ROS param server.
    base_dir_ = ros::package::getPath("hospitality_server");
    ROS_INFO("%s", base_dir_.c_str());
    XmlRpc::XmlRpcValue paramDict;
    n.getParam("/main_server/map_server", paramDict);
    map_bmp_path_ = std::string(paramDict["pixelmap"]["map_bmp_relative_path"]);
    ROS_INFO("%s \n", map_bmp_path_.c_str());
    map_csv_path_ = std::string(paramDict["pixelmap"]["map_csv_relative_path"]);
    ROS_INFO("Got parameters.");
    std::string bmpPath = base_dir_ + map_bmp_path_;
    std::string csvPath = base_dir_ + map_csv_path_;
    ROS_INFO("Map BMP path is %s \n", bmpPath.c_str());

    // Initiate PixelMap.
    int mapOriginRow = static_cast<int>(paramDict["pixelmap"]["map_origin_row"]);
    int mapOriginCol = static_cast<int>(paramDict["pixelmap"]["map_origin_col"]);
    double mapRoiWidth = static_cast<double>(paramDict["pixelmap"]["map_roi_width"]);
    double mapRoiHeight = static_cast<double>(paramDict["pixelmap"]["map_roi_height"]);
    double mapPadRadius = static_cast<double>(paramDict["pixelmap"]["map_pad_radius"]);
    pixel_map_.SetParams(mapOriginRow, mapOriginCol, mapRoiWidth, mapRoiHeight, mapPadRadius);
    pixel_map_.ReadMapBmp(bmpPath.c_str());

    // Read CSV file to get places information.
    

    // Initiate GraphMap.
    std::list<PixelMap::PlaceInfo *> &placeList = pixel_map_.GetPlaceList();
    std::list<PixelMap::PlaceInfo *>::iterator iter;
    for (iter = placeList.begin(); iter != placeList.end(); iter++)
    {
        PixelMap::PlaceInfo *info_ptr = *iter;
        graph_map_.AddNode(info_ptr->place_id, info_ptr->place_code, info_ptr->coordinate_);
    }

    query_place_service_ = n.advertiseService("/query_place_service", &MapServerNode::QueryPlaceHandler, this);
    find_route_service_ = n.advertiseService("/find_route_service", &MapServerNode::FindRouteHandler, this);
}

bool MapServerNode::QueryPlaceHandler(hospitality_msgs::QueryPlace::Request &req,
                                      hospitality_msgs::QueryPlace::Response &resp)
{
    return true;
}

bool MapServerNode::FindRouteHandler(hospitality_msgs::FindRoute::Request &req,
                                     hospitality_msgs::FindRoute::Response &resp)
{
    ROS_INFO("FindRouteHandler: Got service call!");
    hospitality_msgs::PointFloor &startPoint = req.start_point;
    hospitality_msgs::PointFloor &destPoint = req.dest_point;
    PixelMap::PixelIdx startIdx = pixel_map_.GetBoundingPixel(startPoint.x, startPoint.y, startPoint.floor);
    PixelMap::PixelIdx destIdx = pixel_map_.GetBoundingPixel(destPoint.x, destPoint.y, destPoint.floor);

    ROS_INFO("Starting pixel: (%d, %d)", startIdx.row, startIdx.col);
    ROS_INFO("Destination pixel: (%d, %d)", destIdx.row, destIdx.col);

    std::list<PixelMap::PixelIdx> pathContainer;
    pixel_map_.DijkstraPath(pathContainer, startIdx, destIdx);
    pixel_map_.ReducePath(pathContainer);

    resp.waypoints.resize(pathContainer.size());
    int vecIdx = 0;
    for (std::list<PixelMap::PixelIdx>::iterator iter = pathContainer.begin(); iter != pathContainer.end(); iter++)
    {
        PixelMap::PixelIdx &pxIdx = *iter;
        hospitality_msgs::PointFloor point = pixel_map_.GetPixelCenterpoint(pxIdx.row, pxIdx.col, pxIdx.floor);
        resp.waypoints[vecIdx] = point;
        vecIdx++;
    }

    return true;
}

void MapServerNode::ExecLoop()
{
    ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_server_node");
    MapServerNode node;
    node.ExecLoop();
}

#endif