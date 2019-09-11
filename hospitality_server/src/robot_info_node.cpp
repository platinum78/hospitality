#ifndef ROBOT_INFO_NODE_CPP_
#define ROBOT_INFO_NODE_CPP_

#include <map>

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <hospitality/PointFloor.h>
#include <hospitality/UpdatePosition.h>
#include <hospitality/UpdateURI.h>

#include "../include/robot_info/robot_info.hpp"

class RobotInfoNode
{
public:
    RobotInfoNode();
    void ExecLoop(int frequency);

private:
    bool UpdatePositionHandler(hospitality::UpdatePositionRequest &req,
                               hospitality::UpdatePositionResponse &res);
    bool UpdateURIHandler(hospitality::UpdateURIRequest &req,
                          hospitality::UpdateURIResponse &res);

private:
    ros::NodeHandle n;
    ros::ServiceServer update_uri_service_;
    ros::ServiceServer update_position_service_;

private:
    std::map<int, RobotInfo> robot_info_;
    std::map<std::string, int> ip_table_;

private:
    int robot_cnt_;
};

RobotInfoNode::RobotInfoNode()
{   
    // Retrieve parameters for robot system.
    XmlRpc::XmlRpcValue robotInfo;
    robot_cnt_ = n.getParam("/robot_system/robot_info", robotInfo);
    robot_cnt_ = robotInfo.size();

    int robotID = 0;
    std::string robotIP;

    for (int idx = 0; idx < robot_cnt_; idx++)
    {
        // Add robot information.
        robotID = static_cast<int>(robotInfo[idx]["robot_id"]);
        robotIP.assign(robotInfo[idx]["robot_tcp_ip"]);
        robot_info_[robotID].robot_id_ = robotID;
        robot_info_[robotID].robot_type_.assign(robotInfo[idx]["robot_type"]);
        robot_info_[robotID].robot_name_.assign(robotInfo[idx]["robot_name"]);
        robot_info_[robotID].robot_tcp_ip_.assign(robotInfo[idx]["robot_tcp_ip"]);
        robot_info_[robotID].robot_tcp_port_ = static_cast<int>(robotInfo[idx]["robot_tcp_port"]);

        std::cout << robot_info_[robotID] << std::endl;
    }

    // Initiate services.
    update_position_service_ = n.advertiseService("/update_position_service", &RobotInfoNode::UpdatePositionHandler, this);
    update_uri_service_ = n.advertiseService("/update_uri_service", &RobotInfoNode::UpdateURIHandler, this);
}

bool RobotInfoNode::UpdatePositionHandler(hospitality::UpdatePositionRequest &req,
                                             hospitality::UpdatePositionResponse &res)
{
    int robotID = static_cast<int>(req.robot_id);
    robot_info_[robotID].position_ = req.position;
    res.ack = true;
    return true;
}

bool RobotInfoNode::UpdateURIHandler(hospitality::UpdateURIRequest &req,
                                        hospitality::UpdateURIResponse &res)
{
    int robotID = static_cast<int>(req.robot_id);
    robot_info_[robotID].robot_tcp_ip_.assign(req.robot_tcp_ip);
    robot_info_[robotID].robot_tcp_port_ = static_cast<int>(req.robot_tcp_port);
    res.ack = true;
    return true;
}

void RobotInfoNode::ExecLoop(int frequency)
{
    ros::Rate loopRate(frequency);
    ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_info_node");
    RobotInfoNode node;
    node.ExecLoop(5);
}

#endif