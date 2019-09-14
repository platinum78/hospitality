#ifndef TB3_DRIVE_NODE_
#define TB3_DRIVE_NODE_

#include "ros/ros.h"
#include "../include/tb3_topic_manager.hpp"


class TB3DriveNode
{
public:
    TB3DriveNode();

private:
    ros::NodeHandle n;

private: // Subscribers

private: // Publishers
    ros::Publisher cmd_vel_publisher_;
};

int main(int argc, char **argv)
{
    ros::init("tb3_drive_node");
}



#endif