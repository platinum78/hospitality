#ifndef ROBOT_INFO_NODE_CPP_
#define ROBOT_INFO_NODE_CPP_

#include <list>
#include <queue>
#include <map>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <hospitality_msgs/PointFloor.h>
#include <hospitality_msgs/UpdatePosition.h>
#include <hospitality_msgs/UpdateURI.h>

#include "../include/robot_info/robot_info.hpp"
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
    void TaskAllocThreadFunc(std::map<int, RobotInfo> &robot_info,
                             std::list<hospitality_msgs::Task> &task_buffer);
    double AssignTask(hospitality_msgs::Task *task_ptr);
    double AssignTaskTo(hospitality_msgs::Task *task_ptr, int robot_id);

public: // 작업을 할당하는 알고리즘
    double TaskAllocLRT(std::map<int, RobotInfo> &robot_info,
                        std::list<hospitality_msgs::Task> &task_buffer);
    double TaskAllocFIFO(std::map<int, RobotInfo> &robot_info,
                         std::list<hospitality_msgs::Task> &task_buffer);
    double TaskAllocGreedyHeuristic(std::map<int, RobotInfo> &robot_info,
                                    std::list<hospitality_msgs::Task> &task_buffer);

public: // 작업을 할당하는 알고리즘: 실험용
    double TaskAllocExperimental01(std::map<int, RobotInfo> &robot_info,
                                   std::list<hospitality_msgs::Task> &task_buffer);
    double TaskAllocExperimental02(std::map<int, RobotInfo> &robot_info,
                                   std::list<hospitality_msgs::Task> &task_buffer);
    double TaskAllocExperimental03(std::map<int, RobotInfo> &robot_info,
                                   std::list<hospitality_msgs::Task> &task_buffer);
    double TaskAllocExperimental04(std::map<int, RobotInfo> &robot_info,
                                   std::list<hospitality_msgs::Task> &task_buffer);

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
    std::map<int, RobotInfo> robot_info_;
    std::map<std::string, int> ip_table_;
    std::list<hospitality_msgs::Task> task_buffer_;

private:
    std::condition_variable task_alloc_trigger_;

private:
    ros::NodeHandle n;
    ros::ServiceServer update_uri_server_;
    ros::ServiceServer update_position_server_;
    ros::ServiceServer task_req_server_;
    

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

    // 서비스 클라이언트 초기화: 추후 클라이언트 추가 예정
}

void RobotManagerNode::TaskAllocThreadFunc(std::map<int, RobotInfo> &robot_info,
                                           std::list<hospitality_msgs::Task> &task_buffer)
{
    std::unordered_map<int, hospitality_msgs::Task *> tasksByStartPlace;
    std::list<hospitality_msgs::Task>::iterator taskIter;

    // 각 작업을 출발 지점을 기준으로 모아서 정리
    for (taskIter = task_buffer_.begin(); taskIter != task_buffer_.end(); taskIter++)
        if (taskIter->task_type == 1)
            tasksByStartPlace[taskIter->start_place_id] = &(*taskIter);
}

double RobotManagerNode::AssignTask(hospitality_msgs::Task *task_ptr)
{
    std::unordered_map<int, hospitality_msgs::Task *> tasksByStartPlace;
    std::list<hospitality_msgs::Task>::iterator taskIter;

    // 각 작업을 출발 지점을 기준으로 모아서 정리
    for (taskIter = task_buffer_.begin(); taskIter != task_buffer_.end(); taskIter++)
        if (taskIter->task_type == 1)
            tasksByStartPlace[taskIter->start_place_id] = &(*taskIter);
}

double RobotManagerNode::AssignTaskTo(hospitality_msgs::Task *task_ptr, int robot_id)
{
    std::unordered_map<int, hospitality_msgs::Task *> tasksByStartPlace;
    std::list<hospitality_msgs::Task>::iterator taskIter;

    // 각 작업을 출발 지점을 기준으로 모아서 정리
    for (taskIter = task_buffer_.begin(); taskIter != task_buffer_.end(); taskIter++)
        if (taskIter->task_type == 1)
            tasksByStartPlace[taskIter->start_place_id] = &(*taskIter);
}

double RobotManagerNode::TaskAllocLRT(std::map<int, RobotInfo> &robot_info,
                                      std::list<hospitality_msgs::Task> &task_buffer)
{
    std::unordered_map<int, hospitality_msgs::Task *> tasksByStartPlace;
    std::list<hospitality_msgs::Task>::iterator taskIter;

    // 각 작업을 출발 지점을 기준으로 모아서 정리
    for (taskIter = task_buffer_.begin(); taskIter != task_buffer_.end(); taskIter++)
        if (taskIter->task_type == 1)
            tasksByStartPlace[taskIter->start_place_id] = &(*taskIter);
}

double RobotManagerNode::TaskAllocFIFO(std::map<int, RobotInfo> &robot_info,
                                       std::list<hospitality_msgs::Task> &task_buffer)
{
    std::unordered_map<int, hospitality_msgs::Task *> tasksByStartPlace;
    std::list<hospitality_msgs::Task>::iterator taskIter;

    // 각 작업을 출발 지점을 기준으로 모아서 정리
    for (taskIter = task_buffer_.begin(); taskIter != task_buffer_.end(); taskIter++)
        if (taskIter->task_type == 1)
            tasksByStartPlace[taskIter->start_place_id] = &(*taskIter);
}

double RobotManagerNode::TaskAllocGreedyHeuristic(std::map<int, RobotInfo> &robot_info,
                                                  std::list<hospitality_msgs::Task> &task_buffer)
{
    std::unordered_map<int, hospitality_msgs::Task *> tasksByStartPlace;
    std::list<hospitality_msgs::Task>::iterator taskIter;

    // 각 작업을 출발 지점을 기준으로 모아서 정리
    for (taskIter = task_buffer_.begin(); taskIter != task_buffer_.end(); taskIter++)
        if (taskIter->task_type == 1)
            tasksByStartPlace[taskIter->start_place_id] = &(*taskIter);
}

double RobotManagerNode::TaskAllocExperimental01(std::map<int, RobotInfo> &robot_info,
                                                 std::list<hospitality_msgs::Task> &task_buffer)
{

}

double RobotManagerNode::TaskAllocExperimental02(std::map<int, RobotInfo> &robot_info,
                                                 std::list<hospitality_msgs::Task> &task_buffer)
{

}

double RobotManagerNode::TaskAllocExperimental03(std::map<int, RobotInfo> &robot_info,
                                                 std::list<hospitality_msgs::Task> &task_buffer)
{

}

double RobotManagerNode::TaskAllocExperimental04(std::map<int, RobotInfo> &robot_info,
                                                 std::list<hospitality_msgs::Task> &task_buffer)
{

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
    task_alloc_trigger_.notify_all();
    res.ack = true;
    return true;
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