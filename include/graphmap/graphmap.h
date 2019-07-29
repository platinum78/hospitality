#ifndef GRAPHMAP_H_
#define GRAPHMAP_H_

#include <string>
#include <list>

class PlaceNode;
class UnitPath;

class PlaceNode
{
public:
    PlaceNode(int place_code, std::string &place_name);
    PlaceNode(PlaceNode &node);

public:
    bool IsConnectedWith(PlaceNode *);

public:
    bool ConnectNode(PlaceNode *, double, double);

public:
    int GetPlaceCode();
    std::string &GetPlaceName();

private:
    void AttachPath(UnitPath *);

private:
    const int place_code_;
    std::string place_name_;
    std::list<UnitPath *> adjacent_paths_;

public:
    ~PlaceNode();
};

class UnitPath
{
public:
    UnitPath();
    UnitPath(PlaceNode *, PlaceNode *, double, double);
    UnitPath(UnitPath &);

public:
    void Setup(PlaceNode *, PlaceNode *, double, double);

public:
    PlaceNode *GetNode1Ptr();
    PlaceNode *GetNode2Ptr();
    double GetDistance();
    double GetTravelTime();

private:
    PlaceNode *node1_ptr_;
    PlaceNode *node2_ptr_;
    double distance_;
    double travel_time_;
};

class GraphMap
{
public:
    GraphMap();
    GraphMap(GraphMap &);

public:
    void AddPlace(int place_code, std::string &place_name);

public:
    PlaceNode *QueryPlace(int place_code);
    PlaceNode *QueryPlace(std::string &place_name);

public:
    void ConnectPlaces(int place1_code, int place2_code);

private:
    std::list<PlaceNode *> places_;
    std::list<UnitPath *> paths_;
};

#endif