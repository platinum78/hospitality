#ifndef VORONOI_HPP_
#define VORONOI_HPP_

#include <vector>
#include <queue>
#include <list>

#include "voronoi_point.hpp"
#include "voronoi_event.hpp"
#include "voronoi_tree.hpp"

class VoronoiDiagram
{
public:
    VoronoiDiagram();

public:
    void ComputeDiagram();
    void SetPoints();
    void AddParabola(VoronoiPoint *);
    void RemoveParabola(VoronoiPoint *);
    void CheckCircleEvent(VoronoiTree *);
    void AddCircleEvent(VoronoiTree *);

private:
    std::vector<VoronoiPoint *> point_set_;
    std::priority_queue<VoronoiEvent, std::vector<VoronoiEvent>, std::greater<VoronoiEvent> > event_queue_;
    VoronoiTree *beachline_;
};

VoronoiDiagram::VoronoiDiagram() {}

void VoronoiDiagram::ComputeDiagram()
{
    // 이벤트 큐 비우기
    while (!event_queue_.empty())
        event_queue_.pop();

    // 모든 점을 Site VoronoiEvent 로 추가하기
    for (int i = 0; i < point_set_.size(); i++)
    {
        VoronoiEvent event;
        event.type_ = VoronoiEvent::EVENT_SITE;
        event.point_ = point_set_[i];
        event_queue_.push(event);
    }

    // Voronoi 다이어그램 계산
    while (!event_queue_.empty())
    {
        VoronoiEvent e = event_queue_.top(); event_queue_.pop();
        if (e.type_ == VoronoiEvent::EVENT_SITE)
            AddParabola(e.point_);
        else
            RemoveParabola(e.point_);
    }
}

void VoronoiDiagram::AddParabola(VoronoiPoint *p)
{
    VoronoiTree *node_ptr = beachline_;
    while (1)
    {
        // Leaf Node에 도달하였을 경우 루프 탈출
        if (beachline_->is_leaf_)
            break;
        
        double leftEdge = -__DBL_MAX__;
        double rightEdge = __DBL_MAX__;
        
        // Leaf Node를 탐색하며 최댓값 또는 최솟값 제한하기
        if (p->x_ <= node_ptr->x_)
        {
            rightEdge = node_ptr->x_;
            node_ptr = node_ptr->left_child_;
        }
        else
        {
            leftEdge = node_ptr->x_;
            node_ptr = node_ptr->right_child_;
        }
    }

    VoronoiPoint *isectPoint = computeLineIntersection(node_ptr->parabola_->focal_point_, p);

    VoronoiParabola *a = new VoronoiParabola;
    VoronoiParabola *b = new VoronoiParabola;
    VoronoiParabola *c = new VoronoiParabola;

    b->focal_point_ = p;
    a->focal_point_ = c->focal_point_ = node_ptr->parabola_->focal_point_;
    
    delete node_ptr->parabola_;
    node_ptr->parabola_ = nullptr;
    node_ptr->x_ = p->x_;
    
    VoronoiTree *newNode_ptr;

    node_ptr->left_child_ = newNode();                      // Leaf Node를 새로운 포물선의 최댓값 경계로 수정
    node_ptr->left_child_->edge_ = newEdge(isectPoint);     // 새로운 포물선의 좌측 경계에 해당하는 새로운 반직선 생성
    node_ptr->left_child_->x_ = p->x_;                      // 경계의 x 좌표 설정

    node_ptr->left_child_->left_child_ = newNode();
    node_ptr->left_child_->left_child_->parabola_ = a;      // 잘린 포물선의 좌측

    node_ptr->left_child_->right_child_ = newNode();
    node_ptr->left_child_->right_child_->parabola_ = b;     // 새로운 포물선

    node_ptr->right_child_ = newNode();
    node_ptr->right_child_->parabola_ = c;                  // 잘린 포물선의 우측
    node_ptr->edge_ = newEdge(isectPoint);                  // 새로운 포물선의 우측 경계에 해당하는 새로운 반직선 생성

    AddCircleEvent(node_ptr->left_child_->left_child_);
    AddCircleEvent(node_ptr->right_child_);
}

void VoronoiDiagram::RemoveParabola(VoronoiPoint *p)
{
    
}

#endif