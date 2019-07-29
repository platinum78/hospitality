#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "tf/LinearMath/Quaternion.h"

class TopicManager
{
public:
    TopicManager();

public:
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &);

public:
    void PublishCmdVel(double linear, double angular);

private:
    ros::NodeHandle n;
    ros::Publisher cmd_vel_publisher_;
    ros::Subscriber scan_subscriber_;
    ros::Subscriber odom_subscriber_;

public:
    std::vector<float> scan_ranges_;

public:
    geometry_msgs::Pose pose_;
    tf::Quaternion orientation_;
};

TopicManager::TopicManager()
{
    cmd_vel_publisher_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    scan_subscriber_ = n.subscribe("/scan", 1000, &TopicManager::ScanCallback, this);
    odom_subscriber_ = n.subscribe("/odom", 1000, &TopicManager::OdomCallback, this);
}

void TopicManager::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    scan_ranges_ = msg->ranges;
}

void TopicManager::OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    pose_ = msg->pose.pose;
    orientation_.setX(msg->pose.pose.orientation.x);
    orientation_.setY(msg->pose.pose.orientation.y);
    orientation_.setZ(msg->pose.pose.orientation.z);
    orientation_.setW(msg->pose.pose.orientation.w);
}

void TopicManager::PublishCmdVel(double linear, double angular)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear;
    msg.angular.z = angular;
    cmd_vel_publisher_.publish(msg);
}