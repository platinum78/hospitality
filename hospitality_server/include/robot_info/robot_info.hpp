#ifndef ROBOT_INFO_HPP_
#define ROBOT_INFO_HPP_

#include <ros/ros.h>
#include <string>
#include <hospitality_msgs/PointFloor.h>

struct RobotInfo
{
    enum { ROBOT_TYPE_POSTMAN, ROBOT_TYPE_ASSISTANT, ROBOT_TYPE_PORTER };

    int robot_id_;
    std::string robot_type_;
    std::string robot_name_;
    std::string robot_tcp_ip_;
    int robot_tcp_port_;
    hospitality_msgs::PointFloor position_;

};

std::ostream &operator<<(std::ostream &out, const RobotInfo &info)
{
    out << "Robot ID: " << info.robot_id_ << std::endl;
    out << "Robot Name: " << info.robot_name_ << std::endl;
    out << "Robot TCP IP: " << info.robot_tcp_ip_ << std::endl;
    out << "Robot TCP Port: " << info.robot_tcp_port_ << std::endl;
    out << "Robot Position: " << "[" << info.position_.x << ", " << info.position_.y << ", " << info.position_.floor << "]" << std::endl;
    return out;
}

#endif