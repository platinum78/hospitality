#ifndef INIT_SETUP_NODE_CPP_
#define INIT_SETUP_NODE_CPP_

#include <string>

#include <ros/ros.h>
#include <ros/package.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "init_setup_node");
    ros::NodeHandle n;

    std::string basePath = ros::package::getPath("hospitality_server");
}

#endif