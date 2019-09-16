#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

class MapServerTestNode
{
public:
    MapServerTestNode();

public:
    void ExecLoop();

private:
    ros::NodeHandle n;
    ros::Subscriber odom_sub_;
};

MapServerTestNode::MapServerTestNode()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_server_test_node");

    MapServerTestNode n;

    

    while (1)
    {
        
    }
}