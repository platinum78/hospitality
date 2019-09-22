#ifndef VORONOI_POINT_HPP_
#define VORONOI_POINT_HPP_

struct VoronoiPoint
{
    double x_;
    double y_;
    
    bool operator>(VoronoiPoint &p)
    {
        if (y_ > p.y_ || y_ == p.y_ && x_ > p.y_)
            return true;
        else
            return false;
    }

    bool operator<(VoronoiPoint &p)
    {
        if (y_ < p.y_ || y_ == p.y_ && x_ < p.x_)
            return true;
        else
            return false;
    }

    bool operator==(VoronoiPoint &p)
    {
        if (x_ == p.x_ && y_ == p.y_)
            return true;
        else
            return false;
    }

    void operator=(VoronoiPoint &p)
    {
        x_ = p.x_;
        y_ = p.y_;
    }
};

#endif