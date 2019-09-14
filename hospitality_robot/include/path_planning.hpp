#ifndef PATH_PLANNING_HPP_
#define PATH_PLANNING_HPP_

#include <cmath>
#include <vector>
#include <geometry_msgs/Vector3.h>
#include <eigen3/Eigen/Eigen>

#include "hospitality_msgs/Waypoint.h"

namespace path_planning { namespace Frame { enum { INERTIAL, ROBOT }; }; };


class RepulsiveController
{
public:
    RepulsiveController(std::vector<float> &range_ref);

public:
    Eigen::Vector3d ComputeRepulsion(int input_frame, int output_frame);
    Eigen::Vector3d ComputeAttraction(int input_frame, int output_frame);

public:
    double ComputeLinearPID();
    double ComputeAngularPID();

private:
    std::vector<float> &scan_ranges_;
    

private:    
    double sin_[360];
    double cos_[360];

private:
    double force_exp_;
    double tRI_;            // Angle of robot coordinate, in regards of inertial coordinate.
    double kp_linear_, ki_linear_, kd_linear_;
    double kp_angular_, ki_angular_, kd_angular_;
};

RepulsiveController::RepulsiveController(std::vector<float> &scan_ranges)
    : scan_ranges_(scan_ranges)
{
    double angle;
    for (int i = 0; i < 360; i++)
    {
        angle = double(i);
        sin_[i] = sin(angle);
        cos_[i] = cos(angle);
    }
}

Eigen::Vector3d RepulsiveController::ComputeRepulsion(int input_frame = path_planning::Frame::ROBOT,
                                                              int output_frame = path_planning::Frame::INERTIAL)
{
    double dist;
    double fScalar, fX, fY, fNetX = 0, fNetY = 0;
    for (int i = 0; i < 360; i++)
    {
        dist = double(range_[i]);
        fScalar = 1 / pow(dist, force_exp_);
        fX = fScalar * cos_[i];
        fY = fScalar * sin_[i];
        fNetX += fX;
        fNetY += fY;
    }

    Eigen::Vector3d vec;
    vec.x = fNetX; vec.y = fNetY; vec.z = 0;

    if (input_frame == output_frame)
        return vec;
    else if (input_frame == path_planning::Frame::ROBOT && output_frame == path_planning::Frame::INERTIAL)
    {
        Eigen::Vector3d vecRot;
        vecRot.x =  vec.x * cos(tRI_) + vec.y * sin(tRI_);
        vecRot.y = -vec.x * sin(tRI_) + vec.y * cos(tRI_);
        vecRot.z = vec.z;
        return vecRot;
    }
    else if (input_frame == path_planning::Frame::INERTIAL && output_frame == path_planning::Frame::ROBOT)
    {
        Eigen::Vector3d vecRot;
        vecRot.x = vec.x * cos(tRI_) - vec.y * sin(tRI_);
        vecRot.y = vec.x * sin(tRI_) + vec.y * cos(tRI_);
        vecRot.z = vec.z;
        return vecRot;
    }
}

#endif