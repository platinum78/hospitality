#ifndef GEOMETRY_DATATYPES_H_
#define GEOMETRY_DATATYPES_H_

template <typename T>
struct Point2D
{
    T x;
    T y;
};

template <typename T>
struct Point3D
{
    T x;
    T y;
    T z;
};

template <typename T>
struct PointFloor
{
    T x;
    T y;
    int z;
};

#endif