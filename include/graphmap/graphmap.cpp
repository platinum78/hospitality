#ifndef GRAPHMAP_CPP_
#define GRPAHMAP_CPP_

#include "graphmap.h"
#include <list>

////////////////////////// PlaceNode Class Definition //////////////////////////

PlaceNode::PlaceNode(int place_code, std::string &place_name)
    : place_code_(place_code), place_name_(place_name)
{
}

PlaceNode::PlaceNode(PlaceNode &node)
    : place_code_(node.place_code_), place_name_(node.place_name_)
{
}

bool PlaceNode::IsConnectedWith(PlaceNode *node_ptr)
{
    std::list<UnitPath*>::iterator iter;
    UnitPath *unitpath_ptr;
    for (iter = adjacent_paths_.begin(); iter != adjacent_paths_.end(); iter++)
    {
        unitpath_ptr = *iter;
        if (unitpath_ptr->GetNode1Ptr() == node_ptr || unitpath_ptr->GetNode2Ptr() == node_ptr)
            return true;
    }
    return false;
}

bool PlaceNode::ConnectNode(PlaceNode *node_ptr, double distance = 0, double travel_time = 0)
{
    if (!IsConnectedWith(node_ptr))
    {
        UnitPath *unitpath_ptr = new UnitPath;
        unitpath_ptr->Setup(this, node_ptr, distance, travel_time);
        adjacent_paths_.push_back(unitpath_ptr);
        node_ptr->AttachPath(unitpath_ptr);
        return true;
    }
    return false;
}

int PlaceNode::GetPlaceCode()
{
    return place_code_;
}

std::string &PlaceNode::GetPlaceName()
{
    return place_name_;
}

void PlaceNode::AttachPath(UnitPath *unitpath_ptr)
{
    adjacent_paths_.push_back(unitpath_ptr);
}


////////////////////////// UnitPath Class Definition ///////////////////////////

UnitPath::UnitPath() : node1_ptr_(nullptr), node2_ptr_(nullptr), distance_(0), travel_time_(0)
{}

UnitPath::UnitPath(PlaceNode *node1_ptr, PlaceNode *node2_ptr, double distance, double travel_time)
    : node1_ptr_(node1_ptr), node2_ptr_(node2_ptr),
      distance_(distance), travel_time_(travel_time)
{}

UnitPath::UnitPath(UnitPath &path)
    : node1_ptr_(path.node1_ptr_), node2_ptr_(path.node2_ptr_),
      distance_(path.distance_), travel_time_(path.travel_time_)
{}

void UnitPath::Setup(PlaceNode *node1_ptr, PlaceNode *node2_ptr, double distance, double travel_time)
{
    node1_ptr_ = node1_ptr;
    node2_ptr_ = node2_ptr;
    distance_ = distance;
    travel_time_ = travel_time;
}

PlaceNode *UnitPath::GetNode1Ptr()
{
    return node1_ptr_;
}

PlaceNode *UnitPath::GetNode2Ptr()
{
    return node2_ptr_;
}

double UnitPath::GetDistance()
{
    return distance_;
}

double UnitPath::GetTravelTime()
{
    return travel_time_;
}


////////////////////////// GraphMap Class Definition ///////////////////////////

GraphMap::GraphMap()
{

}

void GraphMap::AddPlace(int place_code, std::string &place_name)
{
    
}

PlaceNode *GraphMap::QueryPlace(int place_code)
{
    std::list<PlaceNode*>::iterator iter;
    for (iter = places_.begin(); iter != places_.end(); iter++)
        if ((*iter)->GetPlaceCode() == place_code)
            return *iter;
    return nullptr;
}

PlaceNode *GraphMap::QueryPlace(std::string &place_name)
{
    std::list<PlaceNode*>::iterator iter;
    for (iter = places_.begin(); iter != places_.end(); iter++)
        if ((*iter)->GetPlaceName().compare(place_name) == 0)
            return *iter;
    return nullptr;
}

#endif