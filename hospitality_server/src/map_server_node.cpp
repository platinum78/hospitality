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
#include "../include/map/placeinfo.hpp"

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
    PlaceInfo place_info_;

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
    base_dir_ = ros::package::getPath("hospitality_server");
    ROS_INFO("[map_server_node] Package base directory is: %s", base_dir_.c_str());
    
    // ROS 파라미터 서버에서 파라미터 수신
    XmlRpc::XmlRpcValue paramDict;
    n.getParam("/main_server/map_server", paramDict);
    map_bmp_path_ = std::string(paramDict["pixelmap"]["map_bmp_relative_path"]);
    map_csv_path_ = std::string(paramDict["pixelmap"]["map_csv_relative_path"]);
    std::string bmpPath = base_dir_ + map_bmp_path_;
    std::string csvPath = base_dir_ + map_csv_path_;
    ROS_INFO("[map_server_node] Successfully received all required parameters.");

    // PixelMap 초기화
    ROS_INFO("[map_server_node] Opening bitmap file in the following path: %s", bmpPath.c_str());
    int mapOriginRow = static_cast<int>(paramDict["pixelmap"]["map_origin_row"]);
    int mapOriginCol = static_cast<int>(paramDict["pixelmap"]["map_origin_col"]);
    double mapRoiWidth = static_cast<double>(paramDict["pixelmap"]["map_roi_width"]);
    double mapRoiHeight = static_cast<double>(paramDict["pixelmap"]["map_roi_height"]);
    double mapPadRadius = static_cast<double>(paramDict["pixelmap"]["map_pad_radius"]);
    pixel_map_.SetParams(mapOriginRow, mapOriginCol, mapRoiWidth, mapRoiHeight, mapPadRadius);
    pixel_map_.ReadMapBmp(bmpPath.c_str());
    ROS_INFO("[map_server_node] Successfully initiated pixel map.");

    // CSV 파일을 읽어 장소 정보를 읽어들이기
    ROS_INFO("[map_server_node] Opening CSV file in the following path: %s", csvPath.c_str());
    place_info_.ReadCSV(csvPath, 0.25);
    ROS_INFO("[map_server_node] Successfully initiated place information handler.");

    // GraphMap 초기화
    std::list<PixelMap::PlaceInfo *> &placeList = pixel_map_.GetPlaceList();
    std::list<PixelMap::PlaceInfo *>::iterator iter;
    for (iter = placeList.begin(); iter != placeList.end(); iter++)
    {
        PixelMap::PlaceInfo *info_ptr = *iter;
        graph_map_.AddNode(info_ptr->place_id, info_ptr->place_code, info_ptr->coordinate_);
    }

    // Service 서버 초기화
    query_place_service_ = n.advertiseService("/query_place_service", &MapServerNode::QueryPlaceHandler, this);
    find_route_service_ = n.advertiseService("/find_route_service", &MapServerNode::FindRouteHandler, this);

    ROS_INFO("[map_server_node] Node successfully started.");
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