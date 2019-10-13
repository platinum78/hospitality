#ifndef GRAPHMAP_H_
#define GRAPHMAP_H_

#include <string>
#include <list>
#include <stack>
#include <queue>
#include <cmath>
#include <unordered_map>

#include "ros/ros.h"
#include "xmlrpcpp/XmlRpcValue.h"
#include "../errors.hpp"
#include "pixelmap.hpp"
#include "hospitality_msgs/PointFloor.h"

#define ERROR                       -1
#define SUCCESS                     0
#define NODE_NOT_EXISTS             1
#define NODE_ALREADY_EXISTS         2
#define CONNECTION_NOT_EXISTS       3
#define CONNECTION_ALREADY_EXISTS   4
#define PATH_NOT_FOUND              5

class GraphMap
{
public:
    enum { PATH_DFS, PATH_BFS, PATH_DIJKSTRA };
    enum { HFUNC_MANHATTAN, HFUNC_EUCLIDIAN, HFUNC_CUSTOM };
    enum { COST_DIST, COST_TIME };

public: // 생성자 및 초기화 함수
    GraphMap();
    GraphMap(GraphMap &);
    void SetParams(bool directed);

public: // 데이터 저장을 위한 구조체 선언
    struct Node;
    struct Connection;

public: // 경로 탐색을 위한 구조체 선언
    struct NodePathStat;
    struct NodePathCost;

public: // 장소 노드를 추가/수정/제거하는 메소드
    void AddNode(int id, char *const code, hospitality_msgs::PointFloor &coordinate);
    void AddNode(int id, std::string &code, hospitality_msgs::PointFloor &coordinate);
    void ModifyNode(int id);
    void DelNode(int id);
    void DelNode(char *const code);

public: // 두 개의 장소 노드를 연결하는 메소드
    void ConnectNodes(int n1_id, int n2_id, double trav_dist, double trav_time);
    void ConnectNodes(std::string &n1_code, std::string &n2_code, double trav_dist, double trav_time);
    void ConnectNodes(char *const n1_code, char *const n2_code, double trav_dist, double trav_time);
    void ConnectNodes(Node *n1_ptr, Node *n2_ptr, double trav_dist, double trav_time);

public: // 두 개의 장소 노드의 연결을 끊는 메소드
    void DisconnectNodes(int n1_id, int n2_id);
    void DisconnectNodes(std::string &n1_code, std::string &n2_code);
    void DisconnectNodes(char *const n1_code, char *const n2_code);
    void DisconnectNodes(Node *n1_ptr, Node *n2_ptr);

public: // 장소 노드의 포인터를 반환하는 메소드
    Node *GetNodePtr(int id);
    Node *GetNodePtr(char *const code);
    Node *GetNodePtr(std::string &code);

public:
    void FindRoute(std::list<Node *> &path_container, int n1_id, int n2_id, int method);
    void FindRoute(std::list<Node *> &path_container, char *n1_code, char *n2_code, int method);
    void FindRoute(std::list<Node *> &path_container, std::string &n1_code, std::string &n2_code, int method);

public:
    void FindTraversalOrder(std::list<Node *> &path_container, std::list<Node *> &visit_list);
    void UpdateTraversalOrder(std::list<Node *> &path_conatiner, std::list<Node *> &add_list, std::list<Node *> &remove_list);

private:
    void BFSPath(std::list<Node *> &path_container, Node *const n1_ptr, Node *const n2_ptr);
    void DFSPath(std::list<Node *> &path_container, Node *const n1_ptr, Node *const n2_ptr);
    void DijkstraPath(std::list<Node *> &path_container, Node *n1_ptr, Node *n2_ptr, int cost_type);

public:
    double HFunc_Manhattan(hospitality_msgs::PointFloor &p1, hospitality_msgs::PointFloor &p2);
    double HFunc_Euclidian(hospitality_msgs::PointFloor &p1, hospitality_msgs::PointFloor &p2);

private:
    std::list<Node *> node_list_;
    bool directed_;
};


GraphMap::GraphMap()
{
}


void GraphMap::SetParams(bool directed)
{
    directed_ = directed;
}


struct GraphMap::Node
{
    int id_;
    std::string code_;
    hospitality_msgs::PointFloor coordinate_;
    std::list<Connection *> connections_;
    std::string description_;
};


struct GraphMap::Connection
{
    Node *node_ptr_;
    std::vector<PixelMap::PixelIdx> path_;
    double traversal_dist_;
    double traversal_time_;
};


struct GraphMap::NodePathCost
{
    double cost_;
    Node *node_ptr_;
    bool operator<(const NodePathCost &n) const
    {
        if (cost_ < n.cost_) return true;
        else return false;
    }
    bool operator>(const NodePathCost &n) const
    {
        if (cost_ > n.cost_) return true;
        else return false;
    }
};


struct GraphMap::NodePathStat
{
    int node_state_;
    Node *prev_node_;
    double cost_;
};


void GraphMap::AddNode(int id, char *code, hospitality_msgs::PointFloor &coordinate)
{
    Node *node_ptr = new Node;
    node_ptr->id_ = id;
    node_ptr->code_.assign(code);
    node_ptr->coordinate_ = coordinate;
    node_ptr->connections_.resize(0);
    node_list_.push_back(node_ptr);
}


void GraphMap::AddNode(int id, std::string &code, hospitality_msgs::PointFloor &coordinate)
{
    Node *node_ptr = new Node;
    node_ptr->id_ = id;
    node_ptr->code_.assign(code);
    node_ptr->coordinate_ = coordinate;
    node_ptr->connections_.resize(0);
    node_list_.push_back(node_ptr);
}


void GraphMap::DelNode(int id)
{
    // Find the node with given id.
    Node *delnode_ptr = nullptr;
    typename std::list<Node *>::iterator delnode_iter;
    bool nodeFound = false;

    for (delnode_iter = node_list_.begin(); delnode_iter != node_list_.end(); delnode_iter++)
    {
        delnode_ptr = *delnode_iter;
        if (delnode_ptr->id_ == id)
        {
            nodeFound = true;
            break;
        }
    }

    // Return NODE_NOT_EXISTS error if no such node exists.
    if (!nodeFound)
        throw NoSuchNodeException();

    // Remove all outbound connections of the node to be deleted.
    delnode_ptr->connections_.resize(0);

    // Remove all inbound connections of the node to be deleted.
    Node *node_ptr = nullptr;
    Connection *cnx_ptr = nullptr;
    typename std::list<Node *>::iterator node_iter;
    typename std::list<Connection *>::iterator connection_iter;

    for (node_iter = node_list_.begin(); node_iter != node_list_.end(); node_iter++)
    {
        node_ptr = *node_iter;
        for (connection_iter = node_ptr->connections_.begin(); connection_iter != node_ptr->connections_.end(); connection_iter++)
        {
            cnx_ptr = *connection_iter;
            if (cnx_ptr->node_ptr_ == delnode_ptr)
                node_ptr->connections_.erase(connection_iter);
        }
    }
}


void GraphMap::ConnectNodes(int n1_id, int n2_id, double trav_dist, double trav_time)
{
    Node *n1_ptr = GetNodePtr(n1_id);
    Node *n2_ptr = GetNodePtr(n2_id);
    ConnectNodes(n1_ptr, n2_ptr, trav_dist, trav_time);
}


void GraphMap::ConnectNodes(std::string &n1_code, std::string &n2_code, double trav_dist, double trav_time)
{
    Node *n1_ptr = GetNodePtr(n1_code);
    Node *n2_ptr = GetNodePtr(n2_code);
    ConnectNodes(n1_ptr, n2_ptr, trav_dist, trav_time);
}


void GraphMap::ConnectNodes(char *const n1_code, char *const n2_code, double trav_dist, double trav_time)
{
    Node *n1_ptr = GetNodePtr(n1_code);
    Node *n2_ptr = GetNodePtr(n2_code);
    ConnectNodes(n1_ptr, n2_ptr, trav_dist, trav_time);
}


void GraphMap::ConnectNodes(Node *n1_ptr, Node *n2_ptr, double trav_dist, double trav_time)
{
    for (typename std::list<Connection *>::iterator cnx_iter = n1_ptr->connections_.begin();
         cnx_iter != n1_ptr->connections_.end(); cnx_iter++)
        if ((*cnx_iter)->node_ptr_ == n2_ptr)
            throw DuplicateConnectionException();

    Connection *cnx_ptr = new Connection;
    cnx_ptr->node_ptr_ = n2_ptr;
    cnx_ptr->traversal_dist_ = trav_dist;
    cnx_ptr->traversal_time_ = trav_time;
    n1_ptr->connections_.push_back(cnx_ptr);
}


GraphMap::Node *GraphMap::GetNodePtr(int id)
{
    // Find n1 and n2.
    Node *node_ptr;
    std::list<Node *>::iterator node_iter;

    for (node_iter = node_list_.begin(); node_iter != node_list_.end(); node_iter++)
    {
        node_ptr = *node_iter;
        if (node_ptr->id_ == id)
            return node_ptr;
    }

    throw NoSuchNodeException();
}


GraphMap::Node *GraphMap::GetNodePtr(char *const code)
{
    Node *node_ptr;
    std::list<Node *>::iterator node_iter;
    std::string str(code);

    for (node_iter = node_list_.begin(); node_iter != node_list_.end(); node_iter++)
    {
        node_ptr = *node_iter;
        if ((node_ptr->code_).compare(str) == 0)
            return node_ptr;
    }

    throw NoSuchNodeException();
}


GraphMap::Node *GraphMap::GetNodePtr(std::string &code)
{
    Node *node_ptr;
    std::list<Node *>::iterator node_iter;

    for (node_iter = node_list_.begin(); node_iter != node_list_.end(); node_iter++)
    {
        node_ptr = *node_iter;
        if ((node_ptr->code_).compare(code) == 0)
            return node_ptr;
    }

    throw NoSuchNodeException();
}


void GraphMap::FindRoute(std::list<Node *> &path_container, int n1_id, int n2_id, int method)
{
    // Find n1 and n2.
    Node *n1_ptr = GetNodePtr(n1_id);
    Node *n2_ptr = GetNodePtr(n2_id);

    // Return NODE_NOT_EXISTS error, if either start or end node does not exist.
    if (!n1_ptr || !n2_ptr)
        throw NoSuchNodeException();
    
    switch (method)
    {
    case PATH_DFS:
        DFSPath(path_container, n1_ptr, n2_ptr); return;
    case PATH_BFS:
        BFSPath(path_container, n1_ptr, n2_ptr); return;
    case PATH_DIJKSTRA:
        DijkstraPath(path_container, n1_ptr, n2_ptr, COST_DIST); return;

    default:
        throw ArgumentException("Given number is not a predefined method index.");
    };
}


void GraphMap::FindRoute(std::list<Node *> &path_container, char *n1_code, char *n2_code, int method)
{
    // Find n1 and n2.
    Node *n1_ptr = GetNodePtr(n1_code);
    Node *n2_ptr = GetNodePtr(n2_code);

    // Return NODE_NOT_EXISTS error, if either start or end node does not exist.
    if (!n1_ptr || !n2_ptr)
        throw NoSuchNodeException();
    
    switch (method)
    {
    case PATH_DFS:
        DFSPath(path_container, n1_ptr, n2_ptr); return;
    case PATH_BFS:
        BFSPath(path_container, n1_ptr, n2_ptr); return;
    case PATH_DIJKSTRA:
        DijkstraPath(path_container, n1_ptr, n2_ptr, COST_DIST); return;
    default:
        throw ArgumentException("Given number is not a predefined method index.");
    };
}


void GraphMap::BFSPath(std::list<Node *> &path_container, Node *const n1_ptr, Node *const n2_ptr)
{
    enum { STATE_NEW, STATE_VISITED };

    // Initialize map to store node state.
    std::unordered_map<Node *, NodePathStat> nodePathStat;
    std::list<Node *>::iterator node_iter;
    Node *node_ptr;
    
    for (node_iter = node_list_.begin(); node_iter != node_list_.end(); node_iter++)
    {
        node_ptr = *node_iter;
        nodePathStat[node_ptr] = { STATE_NEW, nullptr };
    }

    // Begin with inserting start node into seek queue.
    Connection *cnx_ptr;
    std::list<Connection *>::iterator cnx_iter;

    std::queue<Node *> openNodes;
    openNodes.push(n1_ptr);
    nodePathStat[n1_ptr].prev_node_ = nullptr;
    bool pathFound = false;

    while (!openNodes.empty())
    {
        // Pop until got node that is new; stop immediately if openNodes becomes empty.
        do
        {
            if (openNodes.empty())
            {
                path_container.resize(0);
                throw NoPathException();
            }

            node_ptr = openNodes.front(); openNodes.pop();
        } while (nodePathStat[node_ptr].node_state_ == STATE_VISITED);
        nodePathStat[node_ptr].node_state_ = STATE_VISITED;
        
        // Path to destination found.
        if (node_ptr == n2_ptr)
        {
            path_container.resize(0);
            node_ptr = n2_ptr;
            while (node_ptr != nullptr)
            {
                path_container.push_front(node_ptr);
                node_ptr = nodePathStat[node_ptr].prev_node_;
            }
            return;
        }
        
        // Expand current node.
        for (cnx_iter = node_ptr->connections_.begin(); cnx_iter != node_ptr->connections_.end(); cnx_iter++)
        {
            cnx_ptr = *cnx_iter;
            openNodes.push(cnx_ptr->node_ptr_);
            if (nodePathStat[cnx_ptr->node_ptr_].node_state_ == STATE_NEW)
                nodePathStat[cnx_ptr->node_ptr_].prev_node_ = node_ptr;
        }

    }

    path_container.resize(0);
    throw NoPathException();
}


void GraphMap::DFSPath(std::list<Node *> &path_container, Node *const n1_ptr, Node *const n2_ptr)
{
    enum { STATE_NEW, STATE_VISITED };

    // Initialize map to store node state.
    std::unordered_map<Node *, NodePathStat> nodePathStat;
    std::list<Node *>::iterator node_iter;
    Node *node_ptr;
    
    for (node_iter = node_list_.begin(); node_iter != node_list_.end(); node_iter++)
    {
        node_ptr = *node_iter;
        nodePathStat[node_ptr] = (NodePathStat){ STATE_NEW, nullptr };
    }

    // Begin with inserting start node into open stack.
    Connection *cnx_ptr;
    std::list<Connection *>::iterator cnx_iter;

    std::stack<Node *> openNodes;
    openNodes.push(n1_ptr);
    nodePathStat[n1_ptr].prev_node_ = nullptr;
    bool pathFound = false;

    while (!openNodes.empty())
    {
        // Pop until got node that is new; stop immediately if openNodes becomes empty.
        do
        {
            if (openNodes.empty())
            {
                path_container.resize(0);
                throw NoPathException();
            }

            node_ptr = openNodes.top(); openNodes.pop();
        } while (nodePathStat[node_ptr].node_state_ == STATE_VISITED);
        nodePathStat[node_ptr].node_state_ = STATE_VISITED;
        
        // Path to destination found.
        if (node_ptr == n2_ptr)
        {
            path_container.resize(0);
            node_ptr = n2_ptr;
            while (node_ptr != nullptr)
            {
                path_container.push_front(node_ptr);
                node_ptr = nodePathStat[node_ptr].prev_node_;
            }
            return;
        }
        
        // Expand current node.
        for (cnx_iter = node_ptr->connections_.begin(); cnx_iter != node_ptr->connections_.end(); cnx_iter++)
        {
            cnx_ptr = *cnx_iter;
            openNodes.push(cnx_ptr->node_ptr_);
            if (nodePathStat[cnx_ptr->node_ptr_].node_state_ == STATE_NEW)
                nodePathStat[cnx_ptr->node_ptr_].prev_node_ = node_ptr;
        }
    }

    path_container.resize(0);
    throw NoPathException();
}


void GraphMap::DijkstraPath(std::list<Node *> &path_container, Node *n1_ptr, Node *n2_ptr, int cost_type)
{
    enum { STATE_NEW, STATE_OPEN, STATE_VISITED };

    // Initialize map to store node states and previous nodes.
    std::unordered_map<Node *, NodePathStat> nodePathStat;
    Node *node_ptr;
    double node_cost, node_cost_new;
    
    for (std::list<Node *>::iterator iter = node_list_.begin(); iter != node_list_.end(); iter++)
    {
        node_ptr = *iter;
        nodePathStat[node_ptr] = (NodePathStat){ STATE_NEW, nullptr, __DBL_MAX__ };
    }

    // Begin with inserting start node into open stack.
    std::priority_queue<NodePathCost, std::vector<NodePathCost>, std::greater<NodePathCost> > openNodes;
    NodePathCost nodeCost = (NodePathCost){ 0.0, n1_ptr };
    openNodes.push(nodeCost);
    nodePathStat[n1_ptr].prev_node_ = nullptr;
    nodePathStat[n1_ptr].cost_ = 0;

    while (!openNodes.empty())
    {
        // Pop until got node that is new; stop immediately if openNodes becomes empty.
        do
        {
            if (openNodes.empty())
            {
                path_container.resize(0);
                throw NoPathException();
            }

            nodeCost = openNodes.top(); openNodes.pop();
            node_ptr = nodeCost.node_ptr_;
            node_cost = nodeCost.cost_;
        } while (nodePathStat[node_ptr].node_state_ == STATE_VISITED);
        nodePathStat[node_ptr].node_state_ = STATE_VISITED;
        
        // Path to destination found.
        if (node_ptr == n2_ptr)
        {
            path_container.resize(0);
            node_ptr = n2_ptr;
            while (node_ptr != nullptr)
            {
                path_container.push_front(node_ptr);
                node_ptr = nodePathStat[node_ptr].prev_node_;
            }
            return;
        }
        
        // Expand current node.
        printf("Expanding: \n");
        Connection *cnx_ptr;
        std::list<Connection *>::iterator cnx_iter;
        for (cnx_iter = node_ptr->connections_.begin(); cnx_iter != node_ptr->connections_.end(); cnx_iter++)
        {
            cnx_ptr = *cnx_iter;
            if (cost_type == COST_DIST)
                node_cost_new = node_cost + cnx_ptr->traversal_dist_;
            else if (cost_type == COST_TIME)
                node_cost_new = node_cost + cnx_ptr->traversal_time_;


            if (node_cost_new < nodePathStat[cnx_ptr->node_ptr_].cost_)
            {
                nodeCost = (NodePathCost){ node_cost_new, cnx_ptr->node_ptr_ };
                openNodes.push(nodeCost);
                nodePathStat[cnx_ptr->node_ptr_] = (NodePathStat){ STATE_OPEN, node_ptr, node_cost_new };
            }
        }

    }

    path_container.resize(0);
    throw NoPathException();
}


void GraphMap::FindTraversalOrder(std::list<Node *> &path_container, std::list<Node *> &visit_list)
{
    
}

#endif