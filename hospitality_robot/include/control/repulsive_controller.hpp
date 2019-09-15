#ifndef PATH_PLANNING_HPP_
#define PATH_PLANNING_HPP_

#include <cmath>
#include <vector>
#include <eigen3/Eigen/Eigen>

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "xmlrpcpp/XmlRpcValue.h"

#include "hospitality_msgs/Waypoint.h"

#include "pid.hpp"

namespace path_planning { namespace Frame { enum { INERTIAL, ROBOT }; }; };


class RepulsiveController
{
public:
    RepulsiveController();

public:
    void GetParams(ros::NodeHandle &n);

public:
    geometry_msgs::Vector3 ComputeVelocity(geometry_msgs::Vector3 &position, geometry_msgs::Vector3 &orientation,
                                           geometry_msgs::Vector3 &target_pos, std::vector<float> &scan_ranges,
                                           int input_frame, int output_frame);

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


RepulsiveController::RepulsiveController()
{
    double angle;
    for (int i = 0; i < 360; i++)
    {
        angle = double(i);
        sin_[i] = sin(angle);
        cos_[i] = cos(angle);
    }
}

void RepulsiveController::GetParams(ros::NodeHandle &n)
{
    XmlRpc::XmlRpcValue paramDict;
    n.getParam("/path_controllers/repulsive_controller", paramDict);
    hz_ = static_cast<double>(paramDict["control_frequency"]);
    repl_exp_ = static_cast<double>(paramDict["repulsion_exponent"]);
    grav_const_ = static_cast<double>(paramDict["gravitational_constant"]);
    kp_lin_ = static_cast<double>(paramDict["kp_linear"]);
    ki_lin_ = static_cast<double>(paramDict["ki_linear"]);
    kd_lin_ = static_cast<double>(paramDict["kd_linear"]);
    kp_ang_ = static_cast<double>(paramDict["kp_angular"]);
    ki_ang_ = static_cast<double>(paramDict["ki_angular"]);
    kd_ang_ = static_cast<double>(paramDict["kd_angular"]);
}

geometry_msgs::Vector3 RepulsiveController::ComputeVelocity(geometry_msgs::Vector3 &position,
                                                            geometry_msgs::Vector3 &orientation,
                                                            geometry_msgs::Vector3 &target_pos,
                                                            std::vector<float> &scan_ranges,
                                                            int input_frame = path_planning::Frame::INERTIAL,
                                                            int output_frame = path_planning::Frame::ROBOT)
{
    // Compute net repulsion.
    double dist;
    double fReplScalar, fX, fY, fNetX = 0, fNetY = 0;
    for (int i = 0; i < 360; i++)
    {
        dist = (scan_ranges[i] > 0.15 ? double(scan_ranges[i]) : (scan_ranges[i] == 0.0 ? 100 : 0.15));
        fReplScalar = 1 / pow(dist, repl_exp_);
        fX = fReplScalar * cos_[i];
        fY = fReplScalar * sin_[i];
        fNetX -= fX;
        fNetY -= fY;
    }
    
    // Compute gravitation.
    double fGravAngle, tRG;
    tRG = atan2(target_pos.y - position.y, target_pos.x - position.x) - orientation.z;
    fNetX += grav_const_ * cos(tRG);
    fNetY += grav_const_ * sin(tRG);

    double fAngle = atan2(fNetY, fNetX);

    // Compute angular PID.
    geometry_msgs::Vector3 robotVel;
    pid_angular_.SetOutput(0);
    pid_angular_.SetTarget(fAngle);
    robotVel.z = pid_angular_.ComputePID();

    // Compute linear velocity.
    robotVel.x = 0.3;

    return robotVel;
}

#endif