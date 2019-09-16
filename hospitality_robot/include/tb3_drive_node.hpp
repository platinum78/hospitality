#ifndef TB3_TOPIC_MANAGER_HPP_
#define TB3_TOPIC_MANAGER_HPP_

#include <vector>
#include <list>

#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "hospitality_msgs/Waypoint.h"

#include "std_srvs/Trigger.h"
#include "hospitality_msgs/PointFloor.h"
#include "hospitality_msgs/SetMode.h"
#include "hospitality_msgs/SetPath.h"

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

public:
    enum { OP_MODE_STOP, OP_MODE_MOVE };

private:
    ros::NodeHandle n;
    ros::Publisher cmd_vel_publisher_;
    ros::Subscriber scan_subscriber_;
    ros::Subscriber odom_subscriber_;
    ros::ServiceServer set_mode_service_;
    ros::ServiceServer set_path_service_;

private:
    T_controller controller_;
    std::list<hospitality_msgs::PointFloor> path_;
    int op_mode_;

public: // Sensor measurements from Turtlebot3.
    std::vector<float> scan_ranges_;
    geometry_msgs::Pose pose_;
    geometry_msgs::Vector3 position_;
    geometry_msgs::Vector3 orientation_;

private:
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &);
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &);

private:
    bool SetModeHandler(hospitality_msgs::SetMode::Request &req,
                        hospitality_msgs::SetMode::Response &resp);
    bool SetPathHandler(hospitality_msgs::SetPath::Request &req,
                        hospitality_msgs::SetPath::Response &resp);
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
    set_mode_service_ = n.advertiseService("/set_mode_service", &TB3DriveNode<T_controller>::SetModeHandler, this);
    set_path_service_ = n.advertiseService("/set_path_service", &TB3DriveNode<T_controller>::SetPathHandler, this);
    controller_.GetParams(n);
    op_mode_ = OP_MODE_STOP;
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
bool TB3DriveNode<T_controller>::SetModeHandler(hospitality_msgs::SetMode::Request &req,
                                                hospitality_msgs::SetMode::Response &resp)
{
    ROS_INFO("TB3DriveNode::SetMode : Received request - %d", req.mode);
    op_mode_ = req.mode;
    resp.ack = true;
    return true;
}

template <typename T_controller>
bool TB3DriveNode<T_controller>::SetPathHandler(hospitality_msgs::SetPath::Request &req,
                                                hospitality_msgs::SetPath::Response &resp)
{
    if (req.path.size() > 0)
    {
        controller_.ResetPath();
        controller_.SetPath(req.path);
        resp.ack = true;
        ROS_INFO("Received path of length %d.", static_cast<int>(req.path.size()));
    }
    else
    {
        controller_.ResetPath();
        resp.ack = false;
        ROS_ERROR("Received zero-length path.");
    }
    return true;
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
    ROS_INFO("Starting TB3DriveNode loop.");
    ros::Rate loopRate(hz);
    while (ros::ok())
    {
        if (op_mode_ == OP_MODE_MOVE && !controller_.IsTrackingComplete())
        {
            geometry_msgs::Vector3 velocity;
            velocity = controller_.ComputeVelocity(position_, orientation_, scan_ranges_);
            printf("%lf, %lf, %lf \n", velocity.x, velocity.y, velocity.z);
            PublishCmdVel(velocity.x, velocity.z);
        }
        else
        {
            PublishCmdVel(0, 0);
        }
        ros::spinOnce();
        loopRate.sleep();
    }
}

#endif