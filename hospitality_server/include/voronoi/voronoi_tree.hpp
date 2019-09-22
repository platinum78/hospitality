#ifndef VORONOI_TREE_H_
#define VORONOI_TREE_H_

#include "voronoi_point.hpp"
#include "voronoi_edge.hpp"
#include "voronoi_parabola.hpp"

struct VoronoiTree
{
    VoronoiTree *parent_;
    VoronoiTree *left_child_;
    VoronoiTree *right_child_;
    VoronoiEdge *edge_;
    VoronoiParabola *parabola_;
    double x_;
    bool is_leaf_;
};

VoronoiTree *newNode()
{
    VoronoiTree *node_ptr = new VoronoiTree;
    node_ptr->parent_ = nullptr;
    node_ptr->left_child_ = nullptr;
    node_ptr->right_child_ = nullptr;
    node_ptr->edge_ = nullptr;
    node_ptr->parabola_ = nullptr;
    node_ptr->is_leaf_ = true;
    return node_ptr;
}

VoronoiTree *newNode(double x, double y)
{
    VoronoiTree *node_ptr = new VoronoiTree;
    node_ptr->x_ = x;
    node_ptr->y_ = y;
    return node_ptr;
}

#endif