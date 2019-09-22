#ifndef VORONOI_EDGE_HPP_
#define VORONOI_EDGE_HPP_

#include "voronoi_point.hpp"

struct VoronoiEdge
{
    VoronoiPoint *origin_;
    VoronoiPoint *end_;
};

VoronoiEdge *newEdge(VoronoiPoint *origin = nullptr, VoronoiPoint *end = nullptr)
{
    VoronoiEdge *edge_ptr = new VoronoiEdge;
    edge_ptr->origin_ = origin;
    edge_ptr->end_ = end;
    return edge_ptr;
}

#endif