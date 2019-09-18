#ifndef PATH_PLANNING_HPP_
#define PATH_PLANNING_HPP_

#include <cmath>
#include <vector>
#include <eigen3/Eigen/Eigen>

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "xmlrpcpp/XmlRpcValue.h"
#include "xmlrpcpp/XmlRpcException.h"

#include "hospitality_msgs/Waypoint.h"
#include "hospitality_msgs/PointFloor.h"

#include "pid.hpp"

#define PI 3.14159265358979

namespace path_planning { namespace Frame { enum { INERTIAL, ROBOT }; }; };



////////////////////////////////////////////////////////////////////////////////
//                              Class Declaration                             //
////////////////////////////////////////////////////////////////////////////////

class Controller
{
public:
    Controller() : tracking_complete_(false) {}

public:
    virtual void GetParams() {}
    virtual void SetPath(std::vector<hospitality_msgs::PointFloor> &path)
    {
        path_.resize(0);
        path_.assign(path.begin(), path.end());
        tracking_complete_ = false;
    }
    virtual void ResetPath() { path_.resize(0); }
    virtual void SetTargetPoint() {}
    virtual geometry_msgs::Vector3 ComputeVelocity() {}
    virtual void ExecLoop() {}
    virtual bool IsTrackingComplete() { return tracking_complete_; }

public:
    enum { TRACKING_POINT, TRACKING_PATH };

protected:
    hospitality_msgs::PointFloor target_point_;
    std::list<hospitality_msgs::PointFloor> path_;
    bool tracking_complete_;
};


class RepulsiveController : public Controller
{
public:
    RepulsiveController();

public:
    void GetParams(ros::NodeHandle &n);

public:
    geometry_msgs::Vector3 ComputeVelocity(geometry_msgs::Vector3 &position, geometry_msgs::Vector3 &orientation,
                                           std::vector<float> &scan_ranges, int input_frame, int output_frame);

public:
    double ComputeLinearPID();
    double ComputeAngularPID();
    
private:    
    double sin_[360];
    double cos_[360];

private:
    PID pid_angular_;

private:
    double hz_;
    double repl_exp_;
    double grav_const_;
    double kp_lin_, ki_lin_, kd_lin_;
    double kp_ang_, ki_ang_, kd_ang_;
};



////////////////////////////////////////////////////////////////////////////////
//                               Class Definition                             //
////////////////////////////////////////////////////////////////////////////////

RepulsiveController::RepulsiveController()
{
    double angle;
    for (int i = 0; i < 360; i++)
    {
        angle = double(i) * PI / 180;
        sin_[i] = sin(angle);
        cos_[i] = cos(angle);
    }
}

void RepulsiveController::GetParams(ros::NodeHandle &n)
{
    XmlRpc::XmlRpcValue paramDict;
    try
    {
        n.getParam("/path_controllers/repulsive_controller", paramDict);
        ROS_INFO("Received parameters for repulsive controller.");
        hz_ = static_cast<double>(paramDict["control_frequency"]);
        repl_exp_ = static_cast<double>(paramDict["repulsion_exponent"]);
        grav_const_ = static_cast<double>(paramDict["gravitational_constant"]);
        kp_lin_ = static_cast<double>(paramDict["kp_linear"]);
        ki_lin_ = static_cast<double>(paramDict["ki_linear"]);
        kd_lin_ = static_cast<double>(paramDict["kd_linear"]);
        kp_ang_ = static_cast<double>(paramDict["kp_angular"]);
        ki_ang_ = static_cast<double>(paramDict["ki_angular"]);
        kd_ang_ = static_cast<double>(paramDict["kd_angular"]);

        pid_angular_.SetGain(kp_ang_, ki_ang_, kd_ang_);
        pid_angular_.SetHz(hz_);
        pid_angular_.SetLimit(-2.7, 2.7);
    }
    catch (XmlRpc::XmlRpcException &e)
    {
        printf("%s \n", e.getMessage().c_str());
    }
}

geometry_msgs::Vector3 RepulsiveController::ComputeVelocity(geometry_msgs::Vector3 &position,
                                                            geometry_msgs::Vector3 &orientation,
                                                            std::vector<float> &scan_ranges,
                                                            int input_frame = path_planning::Frame::INERTIAL,
                                                            int output_frame = path_planning::Frame::ROBOT)
{
    double scanDist, targetDist;
    double fReplScalar, fX, fY, fNetX = 0, fNetY = 0;
    printf("Target: (%lf, %lf) \n", path_.front().x, path_.front().y);

    // Compute net repulsion.
    for (int i = 0; i < 360; i++)
    {
        scanDist = (scan_ranges[i] > 0.15 ? double(scan_ranges[i]) : (scan_ranges[i] == 0.0 ? 100 : 0.15));
        fReplScalar = 1 / pow(scanDist, repl_exp_);
        fX = fReplScalar * cos_[i];
        fY = fReplScalar * sin_[i];
        fNetX -= fX;
        fNetY -= fY;
    }
    fReplScalar = sqrt(fNetX * fNetX + fNetY * fNetY);
    fNetX = fNetX / fReplScalar / 2;
    fNetY = fNetY / fReplScalar / 2;
    
    fNetX = fNetY = 0;
    
    // Compute gravitation.
    double fGravAngle, tRG;
    hospitality_msgs::PointFloor targetPos = path_.front();
    tRG = atan2(targetPos.y - position.y, targetPos.x - position.x) - orientation.z;
    printf("%lf \n", tRG);
    fNetX += grav_const_ * cos(tRG);
    fNetY += grav_const_ * sin(tRG);

    double fAngle = atan2(fNetY, fNetX);

    // Compute angular PID.
    geometry_msgs::Vector3 robotVel;
    pid_angular_.SetOutput(0);
    pid_angular_.SetTarget(fAngle);
    robotVel.z = pid_angular_.ComputePID();
    robotVel.x = 0.225;

    targetDist = pow(targetPos.x - position.x, 2) + pow(targetPos.y - position.y, 2);
    if (path_.size() >= 2)
    {
        if (targetDist < 0.05)
            path_.pop_front();
    }
    else if (path_.size() == 1)
    {
        if (targetDist < 0.005)
        {
            path_.pop_front();
            tracking_complete_ = true;
        }
    }

    return robotVel;
}

#endif