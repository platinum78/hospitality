#ifndef ROBOT_INFO_NODE_CPP_
#define ROBOT_INFO_NODE_CPP_

#include <list>
#include <vector>
#include <cstdlib>
#include <queue>
#include <map>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "ros/ros.h"
#include "xmlrpcpp/XmlRpcValue.h"
#include "hospitality_msgs/PointFloor.h"
#include "hospitality_msgs/UpdatePosition.h"
#include "hospitality_msgs/UpdateURI.h"
#include "hospitality_msgs/QueryPlace.h"

#include "../include/robot_info/robot_info.hpp"
#include "../include/errors.hpp"
#include "hospitality_msgs/Task.h"
#include "hospitality_msgs/TaskReq.h"


////////////////////////////////////////////////////////////////////////////////
//                              Class Declaration                             //
////////////////////////////////////////////////////////////////////////////////

class RobotManagerNode
{
public:
    RobotManagerNode();
    ~RobotManagerNode();
    void ExecLoop(int frequency);

public: // 작업을 할당하는 함수
    double AssignTask(hospitality_msgs::Task *task_ptr);
    double AssignTaskTo(hospitality_msgs::Task *task_ptr, int robot_id);

public: // 작업을 할당하는 알고리즘
    double TaskAllocIntermediate(std::map<int, RobotInfo> &robot_info,
                                 std::list<hospitality_msgs::Task> &task_buffer);
    double TaskAllocLRT(std::map<int, RobotInfo> &robot_info,
                        std::list<hospitality_msgs::Task> &task_buffer);
    double TaskAllocFIFO(std::map<int, RobotInfo> &robot_info,
                         std::list<hospitality_msgs::Task> &task_buffer);
    double TaskAllocGreedyHeuristic(std::map<int, RobotInfo> &robot_info,
                                    std::list<hospitality_msgs::Task> &task_buffer);

public: // 작업을 할당하는 알고리즘: 실험용
    double TaskAllocExperimental01(std::map<int, RobotInfo> &robot_info,
                                   std::vector<hospitality_msgs::Task> &task_buffer);
    double TaskAllocExperimental02(std::map<int, RobotInfo> &robot_info,
                                   std::vector<hospitality_msgs::Task> &task_buffer);
    double TaskAllocExperimental03(std::map<int, RobotInfo> &robot_info,
                                   std::vector<hospitality_msgs::Task> &task_buffer);
    double TaskAllocExperimental04(std::map<int, RobotInfo> &robot_info,
                                   std::vector<hospitality_msgs::Task> &task_buffer);

private: // 작업 우선순위를 계산하는 함수
    bool ComparePlaceAngle(std::vector<hospitality_msgs::Task> &task_set,
                          int a, int b, double pivot_x, double pivot_y);

public: // 작업을 취소하는 함수
    bool CancelTask(hospitality_msgs::Task *task_ptr);

private: // 메시지 콜백 함수

private: // 서비스 핸들러 함수
    bool UpdatePositionHandler(hospitality_msgs::UpdatePositionRequest &req,
                               hospitality_msgs::UpdatePositionResponse &res);
    bool UpdateURIHandler(hospitality_msgs::UpdateURIRequest &req,
                          hospitality_msgs::UpdateURIResponse &res);
    bool TaskReqHandler(hospitality_msgs::TaskReqRequest &req,
                        hospitality_msgs::TaskReqResponse &res);

private:
    void GetCoordinates(std::vector<hospitality_msgs::Task> &task_set,
                        std::unordered_map<int, hospitality_msgs::PointFloor> &data_container);

private:
    std::map<int, RobotInfo> robot_info_;
    std::map<std::string, int> ip_table_;
    std::list<hospitality_msgs::Task> task_buffer_;

private:
    std::unordered_map<int, hospitality_msgs::PointFloor> place_id_coord_map_;
    hospitality_msgs::PointFloor beam_pivot_;

private:
    ros::NodeHandle n;
    ros::ServiceServer update_uri_server_;
    ros::ServiceServer update_position_server_;
    ros::ServiceServer task_req_server_;
    ros::ServiceClient query_place_client_;

private:
    int robot_cnt_;
    bool task_buffer_lock_;

private:
    std::thread task_alloc_thread_;
};


////////////////////////////////////////////////////////////////////////////////
//                              Class Definition                              //
////////////////////////////////////////////////////////////////////////////////

RobotManagerNode::RobotManagerNode()
{   
    // ROS 파라미터 서버로부터 로봇 파라미터 받기
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

    // 서비스 호스트 초기화
    update_position_server_ = n.advertiseService("/update_position_service", &RobotManagerNode::UpdatePositionHandler, this);
    update_uri_server_ = n.advertiseService("/update_uri_service", &RobotManagerNode::UpdateURIHandler, this);
    task_req_server_ = n.advertiseService("/task_req_service", &RobotManagerNode::TaskReqHandler, this);

    // 서비스 클라이언트 초기화
    query_place_client_ = n.serviceClient<hospitality_msgs::QueryPlace>("/query_place_service");
}

double RobotManagerNode::AssignTask(hospitality_msgs::Task *task_ptr)
{
    std::unordered_map<int, hospitality_msgs::Task *> tasksByStartPlace;
    std::list<hospitality_msgs::Task>::iterator taskIter;

    // 각 작업을 출발 지점을 기준으로 모아서 정리
    for (taskIter = task_buffer_.begin(); taskIter != task_buffer_.end(); taskIter++)
        if (taskIter->task_type == 1)
            tasksByStartPlace[taskIter->start_place_id] = &(*taskIter);
    
    // Lookup each robot to find which suites best to move to starting points.
}

double RobotManagerNode::AssignTaskTo(hospitality_msgs::Task *task_ptr, int robot_id)
{
    std::unordered_map<int, std::list<hospitality_msgs::Task *> > tasksByStartPlace;
    std::list<hospitality_msgs::Task>::iterator taskIter;

    // 각 작업을 출발 지점을 기준으로 모아서 정리
    for (taskIter = task_buffer_.begin(); taskIter != task_buffer_.end(); taskIter++)
        if (taskIter->task_type == 1)
            tasksByStartPlace[taskIter->start_place_id].push_back(&(*taskIter));
}

double RobotManagerNode::TaskAllocLRT(std::map<int, RobotInfo> &robot_info,
                                      std::list<hospitality_msgs::Task> &task_buffer)
{
    std::unordered_map<int, std::list<hospitality_msgs::Task *> > tasksByStartPlace;
    std::list<hospitality_msgs::Task>::iterator taskIter;

    // 각 작업을 출발 지점을 기준으로 모아서 정리
    for (taskIter = task_buffer_.begin(); taskIter != task_buffer_.end(); taskIter++)
        if (taskIter->task_type == 1)
            tasksByStartPlace[taskIter->start_place_id].push_back(&(*taskIter));
}

double RobotManagerNode::TaskAllocFIFO(std::map<int, RobotInfo> &robot_info,
                                       std::list<hospitality_msgs::Task> &task_buffer)
{
    std::unordered_map<int, std::list<hospitality_msgs::Task *> > tasksByStartPlace;
    std::list<hospitality_msgs::Task>::iterator taskIter;

    // 각 작업을 출발 지점을 기준으로 모아서 정리
    for (taskIter = task_buffer_.begin(); taskIter != task_buffer_.end(); taskIter++)
        if (taskIter->task_type == 1)
            tasksByStartPlace[taskIter->start_place_id].push_back(&(*taskIter));
}

double RobotManagerNode::TaskAllocGreedyHeuristic(std::map<int, RobotInfo> &robot_info,
                                                  std::list<hospitality_msgs::Task> &task_buffer)
{
    std::unordered_map<int, std::list<hospitality_msgs::Task *> > tasksByStartPlace;
    std::list<hospitality_msgs::Task>::iterator taskIter;

    // 각 작업을 출발 지점을 기준으로 모아서 정리
    for (taskIter = task_buffer_.begin(); taskIter != task_buffer_.end(); taskIter++)
        if (taskIter->task_type == 1)
            tasksByStartPlace[taskIter->start_place_id].push_back(&(*taskIter));
}

// 회전하는 반직선 기법
double RobotManagerNode::TaskAllocExperimental01(std::map<int, RobotInfo> &robot_info,
                                                 std::vector<hospitality_msgs::Task> &task_set)
{
    std::unordered_map<int, hospitality_msgs::PointFloor> placeIDCoordMap;
    GetCoordinates(task_set, placeIDCoordMap);

    std::vector<int> ids(placeIDCoordMap.size());
    int cnt = 0;
    std::unordered_map<int, hospitality_msgs::PointFloor>::iterator iter;
    for (iter = placeIDCoordMap.begin(); iter != placeIDCoordMap.end(); iter++)
        ids[cnt++] = iter->first;
    
    std::sort(ids.begin(), ids.end(), ComparePlaceAngle);
}

double RobotManagerNode::TaskAllocExperimental02(std::map<int, RobotInfo> &robot_info,
                                                 std::vector<hospitality_msgs::Task> &task_buffer)
{

}

double RobotManagerNode::TaskAllocExperimental03(std::map<int, RobotInfo> &robot_info,
                                                 std::vector<hospitality_msgs::Task> &task_buffer)
{

}

double RobotManagerNode::TaskAllocExperimental04(std::map<int, RobotInfo> &robot_info,
                                                 std::vector<hospitality_msgs::Task> &task_buffer)
{

}

bool RobotManagerNode::ComparePlaceAngle(std::vector<hospitality_msgs::Task> &task_set,
                                        int a, int b, double pivot_x, double pivot_y)
{
    if (a >= task_set.size() || b >= task_set.size())
        throw OutOfBoundException("Index is out of bound.");
    
    // double &Ax = task_set[a].dest
}

bool RobotManagerNode::UpdatePositionHandler(hospitality_msgs::UpdatePositionRequest &req,
                                             hospitality_msgs::UpdatePositionResponse &res)
{
    int robotID = static_cast<int>(req.robot_id);
    robot_info_[robotID].position_ = req.position;
    res.ack = true;
    return true;
}

bool RobotManagerNode::UpdateURIHandler(hospitality_msgs::UpdateURIRequest &req,
                                        hospitality_msgs::UpdateURIResponse &res)
{
    int robotID = static_cast<int>(req.robot_id);
    robot_info_[robotID].robot_tcp_ip_.assign(req.robot_tcp_ip);
    robot_info_[robotID].robot_tcp_port_ = static_cast<int>(req.robot_tcp_port);
    res.ack = true;
    return true;
}

bool RobotManagerNode::TaskReqHandler(hospitality_msgs::TaskReqRequest &req,
                                      hospitality_msgs::TaskReqResponse &res)
{
    int taskCnt = req.tasks.size();
    for (int i = 0; i < taskCnt; i++)
    {
        hospitality_msgs::Task task = req.tasks[i];
        task_buffer_.push_back(task);
    }
    res.ack = true;
    return true;
}

void RobotManagerNode::GetCoordinates(std::vector<hospitality_msgs::Task> &task_set,
                                      std::unordered_map<int, hospitality_msgs::PointFloor> &data_container)
{
    data_container.clear();
    std::list<hospitality_msgs::PointFloor> coordinates;
    for (int i = 0; i < task_set.size(); i++)
    {
        int &startPlaceID = task_set[i].start_place_id;
        int &destPlaceID = task_set[i].dest_place_id;

        if (startPlaceID >= 0 && data_container.count(startPlaceID) == 0)
        {
            hospitality_msgs::QueryPlace srv;
            srv.request.query_type = 1;
            srv.request.place_id = startPlaceID;
            if (query_place_client_.call(srv))
                data_container[startPlaceID] = srv.response.place_info.coordinate;
            else
            {
                ROS_ERROR("[robot_manager_node] Service exception occured while querying place info for: %d", startPlaceID);
                ros::Exception("Service is not properly called.");
            }
        }

        if (destPlaceID >= 0 && data_container.count(destPlaceID) == 0)
        {
            hospitality_msgs::QueryPlace srv;
            srv.request.query_type = 1;
            srv.request.place_id = destPlaceID;
            if (query_place_client_.call(srv))
                data_container[destPlaceID] = srv.response.place_info.coordinate;
            else
            {
                ROS_ERROR("[robot_manager_node] Service exception occured while querying place info for: %d", destPlaceID);
                ros::Exception("Service is not properly called.");
            }
        }
    }
}

void RobotManagerNode::ExecLoop(int frequency)
{
    ros::Rate loopRate(frequency);
    ros::spin();
}

RobotManagerNode::~RobotManagerNode()
{
    // 모든 스레드가 종료될 때까지 대기
    task_alloc_thread_.join();
}


////////////////////////////////////////////////////////////////////////////////
//                                Main Function                               //
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_manager_node");
    RobotManagerNode node;
    node.ExecLoop(5);
}

#endif