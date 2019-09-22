#ifndef VORONOI_PARABOLA_HPP_
#define VORONOI_PARABOLA_HPP_

#include "voronoi_point.hpp"

struct VoronoiParabola
{
    VoronoiPoint *focal_point_;
};

VoronoiPoint *computeCircleEvent(VoronoiPoint *p1, VoronoiPoint *p2, VoronoiPoint *p3)
{
    double x12_p = p1->x_ + p2->x_;
    double x12_m = p1->x_ - p2->x_;

    double x23_p = p2->x_ + p3->x_;
    double x23_m = p2->x_ - p3->x_;

    double y12_p = p1->y_ + p2->y_;
    double y12_m = p1->y_ - p2->y_;

    double y23_p = p2->y_ + p3->y_;
    double y23_m = p2->y_ - p3->y_;

    double x = (-y12_m * (x23_p * x23_m + y23_p * y23_m) + y23_m * (x12_p * x12_m + y12_p * y12_m)) / 
                (2 * (x12_m * y23_m - x23_m * y12_m));
    double y = (x12_m * (x23_p * x23_m + y23_p * y23_m) - x23_m * (x12_p * x12_m + y12_p * y12_m)) /
                (2 * (x12_m * y23_m - x23_m * y12_m));
    
    VoronoiPoint *point = new VoronoiPoint;
    point->x_ = x;
    point->y_ = y;
    return point;
}

VoronoiPoint *computeLineIntersection(VoronoiPoint *p_f, VoronoiPoint *p_d)
{
    VoronoiPoint *point_ptr = new VoronoiPoint;
    point_ptr->x_ = p_d->x_;
    point_ptr->y_ = (p_d->x_ - p_f->x_) * (p_d->x_ - p_f->x_) / 2 / (p_f->y_ - p_d->y_) + (p_f->y_ + p_d->y_) / 2;
    return point_ptr;
}

#endif