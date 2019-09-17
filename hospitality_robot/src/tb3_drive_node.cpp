#ifndef TB3_DRIVE_NODE_CPP_
#define TB3_DRIVE_NODE_CPP_

#include "ros/ros.h"
#include "../include/tb3_drive_node.hpp"
#include "../include/control/repulsive_controller.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tb3_drive_node");
    TB3DriveNode<RepulsiveController> tb3DriveNode;
    tb3DriveNode.ExecLoop(60);
}

#endif