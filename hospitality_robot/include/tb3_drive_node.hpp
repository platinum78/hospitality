#ifndef TB3_TOPIC_MANAGER_HPP_
#define TB3_TOPIC_MANAGER_HPP_

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

#include "hospitality_msgs/Waypoint.h"

#include "robot_state_variables.hpp"



////////////////////////////////////////////////////////////////////////////////
//                              Class Declaration                             //
////////////////////////////////////////////////////////////////////////////////

template <typename T_controller>
class TB3DriveNode
{
public:
    TB3DriveNode();

public:
    void PublishCmdVel(double linear, double angular);

public:
    void ExecLoop(double hz);

private:
    ros::NodeHandle n;
    ros::Publisher cmd_vel_publisher_;
    ros::Subscriber scan_subscriber_;
    ros::Subscriber odom_subscriber_;
    ros::ServiceServer set_mode_service_;
    ros::ServiceServer traverse_path_service_;

private:
    T_controller controller_;

public: // Sensor measurements from Turtlebot3.
    std::vector<float> scan_ranges_;
    geometry_msgs::Pose pose_;
    geometry_msgs::Vector3 position_;
    geometry_msgs::Vector3 orientation_;

private:
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &);
};



////////////////////////////////////////////////////////////////////////////////
//                               Class Definition                             //
////////////////////////////////////////////////////////////////////////////////

template <typename T_controller>
TB3DriveNode<T_controller>::TB3DriveNode()
{
    cmd_vel_publisher_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    scan_subscriber_ = n.subscribe("/scan", 1000, &TB3DriveNode::ScanCallback, this);
    odom_subscriber_ = n.subscribe("/odom", 1000, &TB3DriveNode::OdomCallback, this);
    controller_.GetParams(n);
}

template <typename T_controller>
void TB3DriveNode<T_controller>::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    scan_ranges_ = msg->ranges;
}

template <typename T_controller>
void TB3DriveNode<T_controller>::OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    pose_ = msg->pose.pose;
    position_.x = msg->pose.pose.position.x;
    position_.y = msg->pose.pose.position.y;
    position_.z = msg->pose.pose.position.z;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);

    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // the found angles are written in a geometry_msgs::Vector3
    orientation_.x = roll;
    orientation_.y = pitch;
    orientation_.z = yaw;
}

template <typename T_controller>
void TB3DriveNode<T_controller>::PublishCmdVel(double linear, double angular)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear;
    msg.angular.z = angular;
    cmd_vel_publisher_.publish(msg);
}

template <typename T_controller>
void TB3DriveNode<T_controller>::ExecLoop(double hz)
{
    ros::Rate loopRate(hz);
    while (ros::ok())
    {

    }
}

#endif