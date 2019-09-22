#ifndef VORONOI_EVENT_HPP_
#define VORONOI_EVENT_HPP_

#include "voronoi_point.hpp"

struct VoronoiEvent
{
    enum { EVENT_SITE, EVENT_CIRCLE};
    int type_;
    VoronoiPoint *point_;

    bool operator<(VoronoiEvent &e)
    {
        if (*point_ < *(e.point_))
            return true;
        else
            return false;
    }

    bool operator==(VoronoiEvent &e)
    {
        if (*point_ == *(e.point_))
            return true;
        else
            return false;
    }

    bool operator>(VoronoiEvent &e)
    {
        if (*point_ > *(e.point_))
            return true;
        else
            return false;
    }
};

#endif