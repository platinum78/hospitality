#include <iostream>
#include "ros/ros.h"
#include "../include/tb3_drive/topic_manager.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlebot3_drive_node");
    TopicManager tmgr;
    ros::Rate loopRate(10);

    int coef = 1;
    int add = 1;
    double speedBase = 0.01;
    double speed;

    while (ros::ok())
    {
        if (coef == 10)
            add = -1;
        else if (coef == 0)
            add = 1;

        coef += add;
        speed = coef * speedBase;

        tmgr.PublishCmdVel(speed, 0);
        loopRate.sleep();
    }

    return 0;
}