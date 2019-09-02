#ifndef GRAPHMAP_H_
#define GRAPHMAP_H_

#include <string>
#include <list>
#include <stack>
#include <queue>
#include <cmath>
#include <unordered_map>

#include "./geometry_datatypes.h"

#define ERROR                       -1
#define SUCCESS                     0
#define NODE_NOT_EXISTS             1
#define NODE_ALREADY_EXISTS         2
#define CONNECTION_NOT_EXISTS       3
#define CONNECTION_ALREADY_EXISTS   4
#define PATH_NOT_FOUND              5

struct Node;
struct Connection;

class GraphMap
{
public:
    enum { PATH_DFS, PATH_BFS, PATH_DIJKSTRA };
    enum { HFUNC_MANHATTAN, HFUNC_EUCLIDIAN, HFUNC_CUSTOM };
    enum { COST_DIST, COST_TIME };

public:
    GraphMap();
    GraphMap(GraphMap &);

public:
    struct Node;
    struct Connection;

private:
    struct NodeInfo;
    struct NodeCost;

public:
    int AddNode(int id, char *const code, PointFloor<double> coordinate);
    int AddNode(int id, std::string &code, PointFloor<double> coordinate);
    int DelNode(int id);
    int DelNode(char *const code);

public:
    int ConnectNodes(int n1_id, int n2_id, double trav_dist, double trav_time);
    int ConnectNodes(std::string &n1_code, std::string &n2_code, double trav_dist, double trav_time);
    int ConnectNodes(char *const n1_code, char *const n2_code, double trav_dist, double trav_time);
    int ConnectNodes(Node *n1_ptr, Node *n2_ptr, double trav_dist, double trav_time);

public:
    int DisconnectNodes(int n1_id, int n2_id);
    int DisconnectNodes(std::string &n1_code, std::string &n2_code);
    int DisconnectNodes(char *const n1_code, char *const n2_code);
    int DisconnectNodes(Node *n1_ptr, Node *n2_ptr);

public:
    Node *QueryNodePtr(int id);
    Node *QueryNodePtr(char *const code);
    Node *QueryNodePtr(std::string &code);

public:
    int FindRoute(std::list<Node *> &path_container, int n1_id, int n2_id, int method);
    int FindRoute(std::list<Node *> &path_container, char *n1_code, char *n2_code, int method);
    int FindRoute(std::list<Node *> &path_container, std::string &n1_code, std::string &n2_code, int method);

private:
    int BFSPath(std::list<Node *> &path_container, Node *const n1_ptr, Node *const n2_ptr);
    int DFSPath(std::list<Node *> &path_container, Node *const n1_ptr, Node *const n2_ptr);
    int DijkstraPath(std::list<Node *> &path_container, Node *n1_ptr, Node *n2_ptr, int cost_type);

public:
    double HFunc_Manhattan(PointFloor<double> &p1, PointFloor<double> &p2);
    double HFunc_Euclidian(PointFloor<double> &p1, PointFloor<double> &p2);

private:
    std::list<Node *> node_list_;
    bool directed_;
};

GraphMap::GraphMap()
{
}

struct GraphMap::Node
{
    int id_;
    std::string code_;
    PointFloor<double> coordinate_;
    std::list<Connection *> connections_;
};

struct GraphMap::Connection
{
    Node *node_ptr_;
    double traversal_dist_;
    double traversal_time_;
};

struct GraphMap::NodeCost
{
    double cost_;
    Node *node_ptr_;
    bool operator<(const NodeCost &n) const
    {
        if (cost_ < n.cost_)
            return true;
        else
            return false;
    }
    bool operator>(const NodeCost &n) const
    {
        if (cost_ > n.cost_)
            return true;
        else
            return false;
    }
};

struct GraphMap::NodeInfo
{
    int node_state_;
    Node *prev_node_;
    double cost_;
};

int GraphMap::AddNode(int id, char *code, PointFloor<double> coordinate)
{
    Node *node_ptr = new Node;
    node_ptr->id_ = id;
    node_ptr->code_.assign(code);
    node_ptr->coordinate_ = coordinate;
    node_ptr->connections_.resize(0);
    node_list_.push_back(node_ptr);
}

int GraphMap::AddNode(int id, std::string &code, PointFloor<double> coordinate)
{
    Node *node_ptr = new Node;
    node_ptr->id_ = id;
    node_ptr->code_.assign(code);
    node_ptr->coordinate_ = coordinate;
    node_ptr->connections_.resize(0);
    node_list_.push_back(node_ptr);
}

int GraphMap::DelNode(int id)
{
    // Find the node with given id.
    Node *delnode_ptr = nullptr;
    std::list<Node *>::iterator delnode_iter;
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
        return NODE_NOT_EXISTS;

    // Remove all outbound connections of the node to be deleted.
    delnode_ptr->connections_.resize(0);

    // Remove all inbound connections of the node to be deleted.
    Node *node_ptr = nullptr;
    Connection *cnx_ptr = nullptr;
    std::list<Node *>::iterator node_iter;
    std::list<Connection *>::iterator connection_iter;

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

    return SUCCESS;
}

int GraphMap::ConnectNodes(int n1_id, int n2_id, double trav_dist, double trav_time)
{
    Node *n1_ptr = QueryNodePtr(n1_id);
    Node *n2_ptr = QueryNodePtr(n2_id);
    return ConnectNodes(n1_ptr, n2_ptr, trav_dist, trav_time);
}

int GraphMap::ConnectNodes(std::string &n1_code, std::string &n2_code, double trav_dist, double trav_time)
{
    Node *n1_ptr = QueryNodePtr(n1_code);
    Node *n2_ptr = QueryNodePtr(n2_code);
    return ConnectNodes(n1_ptr, n2_ptr, trav_dist, trav_time);
}

int GraphMap::ConnectNodes(char *const n1_code, char *const n2_code, double trav_dist, double trav_time)
{
    Node *n1_ptr = QueryNodePtr(n1_code);
    Node *n2_ptr = QueryNodePtr(n2_code);
    return ConnectNodes(n1_ptr, n2_ptr, trav_dist, trav_time);
}

int GraphMap::ConnectNodes(Node *n1_ptr, Node *n2_ptr, double trav_dist, double trav_time)
{
    for (std::list<Connection *>::iterator cnx_iter = n1_ptr->connections_.begin();
         cnx_iter != n1_ptr->connections_.end(); cnx_iter++)
        if ((*cnx_iter)->node_ptr_ == n2_ptr)
            return CONNECTION_ALREADY_EXISTS;

    Connection *cnx_ptr = new Connection;
    cnx_ptr->node_ptr_ = n2_ptr;
    cnx_ptr->traversal_dist_ = trav_dist;
    cnx_ptr->traversal_time_ = trav_time;
    n1_ptr->connections_.push_back(cnx_ptr);

    return SUCCESS;
}

GraphMap::Node *GraphMap::QueryNodePtr(int id)
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

    return nullptr;
}

GraphMap::Node *GraphMap::QueryNodePtr(char *const code)
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

    return nullptr;
}

GraphMap::Node *GraphMap::QueryNodePtr(std::string &code)
{
    Node *node_ptr;
    std::list<Node *>::iterator node_iter;

    for (node_iter = node_list_.begin(); node_iter != node_list_.end(); node_iter++)
    {
        node_ptr = *node_iter;
        if ((node_ptr->code_).compare(code) == 0)
            return node_ptr;
    }
}

int GraphMap::FindRoute(std::list<Node *> &path_container, int n1_id, int n2_id, int method)
{
    // Find n1 and n2.
    Node *n1_ptr = QueryNodePtr(n1_id);
    Node *n2_ptr = QueryNodePtr(n2_id);

    // Return NODE_NOT_EXISTS error, if either start or end node does not exist.
    if (!n1_ptr || !n2_ptr)
        return NODE_NOT_EXISTS;
    
    switch (method)
    {
    case PATH_DFS:
        return DFSPath(path_container, n1_ptr, n2_ptr);
    case PATH_BFS:
        return BFSPath(path_container, n1_ptr, n2_ptr);
    case PATH_DIJKSTRA:
        return DijkstraPath(path_container, n1_ptr, n2_ptr, COST_DIST);

    default:
        return ERROR;
    };
}

int GraphMap::FindRoute(std::list<Node *> &path_container, char *n1_code, char *n2_code, int method)
{
    // Find n1 and n2.
    Node *n1_ptr = QueryNodePtr(n1_code);
    Node *n2_ptr = QueryNodePtr(n2_code);

    // Return NODE_NOT_EXISTS error, if either start or end node does not exist.
    if (!n1_ptr || !n2_ptr)
        return NODE_NOT_EXISTS;
    
    switch (method)
    {
    case PATH_DFS:
        return DFSPath(path_container, n1_ptr, n2_ptr);
    case PATH_BFS:
        return BFSPath(path_container, n1_ptr, n2_ptr);
    case PATH_DIJKSTRA:
        return DijkstraPath(path_container, n1_ptr, n2_ptr, COST_DIST);

    default:
        return ERROR;
    };
}


int GraphMap::BFSPath(std::list<Node *> &path_container, Node *const n1_ptr, Node *const n2_ptr)
{
    enum { STATE_NEW, STATE_VISITED };

    // Initialize map to store node state.
    std::unordered_map<Node *, NodeInfo> nodeInfo;
    std::list<Node *>::iterator node_iter;
    Node *node_ptr;
    
    for (node_iter = node_list_.begin(); node_iter != node_list_.end(); node_iter++)
    {
        node_ptr = *node_iter;
        nodeInfo[node_ptr] = { STATE_NEW, nullptr };
    }

    // Begin with inserting start node into seek queue.
    Connection *cnx_ptr;
    std::list<Connection *>::iterator cnx_iter;

    std::queue<Node *> openNodes;
    openNodes.push(n1_ptr);
    nodeInfo[n1_ptr].prev_node_ = nullptr;
    bool pathFound = false;

    while (!openNodes.empty())
    {
        // Pop until got node that is new; stop immediately if openNodes becomes empty.
        do
        {
            if (openNodes.empty())
            {
                path_container.resize(0);
                return PATH_NOT_FOUND;
            }

            node_ptr = openNodes.front(); openNodes.pop();
        } while (nodeInfo[node_ptr].node_state_ == STATE_VISITED);
        nodeInfo[node_ptr].node_state_ = STATE_VISITED;
        
        // Path to destination found.
        if (node_ptr == n2_ptr)
        {
            path_container.resize(0);
            node_ptr = n2_ptr;
            while (node_ptr != nullptr)
            {
                path_container.push_front(node_ptr);
                node_ptr = nodeInfo[node_ptr].prev_node_;
            }
            return SUCCESS;
        }
        
        // Expand current node.
        for (cnx_iter = node_ptr->connections_.begin(); cnx_iter != node_ptr->connections_.end(); cnx_iter++)
        {
            cnx_ptr = *cnx_iter;
            openNodes.push(cnx_ptr->node_ptr_);
            if (nodeInfo[cnx_ptr->node_ptr_].node_state_ == STATE_NEW)
                nodeInfo[cnx_ptr->node_ptr_].prev_node_ = node_ptr;
        }

    }
}

int GraphMap::DFSPath(std::list<Node *> &path_container, Node *const n1_ptr, Node *const n2_ptr)
{
    enum { STATE_NEW, STATE_VISITED };

    // Initialize map to store node state.
    std::unordered_map<Node *, NodeInfo> nodeInfo;
    std::list<Node *>::iterator node_iter;
    Node *node_ptr;
    
    for (node_iter = node_list_.begin(); node_iter != node_list_.end(); node_iter++)
    {
        node_ptr = *node_iter;
        nodeInfo[node_ptr] = (NodeInfo){ STATE_NEW, nullptr };
    }

    // Begin with inserting start node into open stack.
    Connection *cnx_ptr;
    std::list<Connection *>::iterator cnx_iter;

    std::stack<Node *> openNodes;
    openNodes.push(n1_ptr);
    nodeInfo[n1_ptr].prev_node_ = nullptr;
    bool pathFound = false;

    while (!openNodes.empty())
    {
        // Pop until got node that is new; stop immediately if openNodes becomes empty.
        do
        {
            if (openNodes.empty())
            {
                path_container.resize(0);
                return PATH_NOT_FOUND;
            }

            node_ptr = openNodes.top(); openNodes.pop();
        } while (nodeInfo[node_ptr].node_state_ == STATE_VISITED);
        nodeInfo[node_ptr].node_state_ = STATE_VISITED;
        
        // Path to destination found.
        if (node_ptr == n2_ptr)
        {
            path_container.resize(0);
            node_ptr = n2_ptr;
            while (node_ptr != nullptr)
            {
                path_container.push_front(node_ptr);
                node_ptr = nodeInfo[node_ptr].prev_node_;
            }
            return SUCCESS;
        }
        
        // Expand current node.
        for (cnx_iter = node_ptr->connections_.begin(); cnx_iter != node_ptr->connections_.end(); cnx_iter++)
        {
            cnx_ptr = *cnx_iter;
            openNodes.push(cnx_ptr->node_ptr_);
            if (nodeInfo[cnx_ptr->node_ptr_].node_state_ == STATE_NEW)
                nodeInfo[cnx_ptr->node_ptr_].prev_node_ = node_ptr;
        }
    }

    path_container.resize(0);
    return PATH_NOT_FOUND;
}

int GraphMap::DijkstraPath(std::list<Node *> &path_container, Node *n1_ptr, Node *n2_ptr, int cost_type)
{
    enum { STATE_NEW, STATE_OPEN, STATE_VISITED };

    // Initialize map to store node states and previous nodes.
    std::unordered_map<Node *, NodeInfo> nodeInfo;
    Node *node_ptr;
    double node_cost, node_cost_new;
    
    for (std::list<Node *>::iterator iter = node_list_.begin(); iter != node_list_.end(); iter++)
    {
        node_ptr = *iter;
        nodeInfo[node_ptr] = (NodeInfo){ STATE_NEW, nullptr, __DBL_MAX__ };
    }

    // Begin with inserting start node into open stack.
    std::priority_queue<NodeCost, std::vector<NodeCost>, std::greater<NodeCost> > openNodes;
    NodeCost nodeCost = (NodeCost){ 0.0, n1_ptr };
    openNodes.push(nodeCost);
    nodeInfo[n1_ptr].prev_node_ = nullptr;
    nodeInfo[n1_ptr].cost_ = 0;

    while (!openNodes.empty())
    {
        // Pop until got node that is new; stop immediately if openNodes becomes empty.
        do
        {
            if (openNodes.empty())
            {
                path_container.resize(0);
                return PATH_NOT_FOUND;
            }

            nodeCost = openNodes.top(); openNodes.pop();
            node_ptr = nodeCost.node_ptr_;
            node_cost = nodeCost.cost_;
        } while (nodeInfo[node_ptr].node_state_ == STATE_VISITED);
        nodeInfo[node_ptr].node_state_ = STATE_VISITED;
        
        // Path to destination found.
        if (node_ptr == n2_ptr)
        {
            path_container.resize(0);
            node_ptr = n2_ptr;
            while (node_ptr != nullptr)
            {
                path_container.push_front(node_ptr);
                node_ptr = nodeInfo[node_ptr].prev_node_;
            }
            return SUCCESS;
        }
        
        // Expand current node.
        printf("Expanding: \n");
        Connection *cnx_ptr;
        std::list<Connection *>::iterator cnx_iter;
        for (cnx_iter = node_ptr->connections_.begin(); cnx_iter != node_ptr->connections_.end(); cnx_iter++)
        {
            cnx_ptr = *cnx_iter;
            printf("%d \n", cnx_ptr->node_ptr_->id_);
            if (cost_type == COST_DIST)
                node_cost_new = node_cost + cnx_ptr->traversal_dist_;
            else if (cost_type == COST_TIME)
                node_cost_new = node_cost + cnx_ptr->traversal_time_;


            if (node_cost_new < nodeInfo[cnx_ptr->node_ptr_].cost_)
            {
                nodeCost = (NodeCost){ node_cost_new, cnx_ptr->node_ptr_ };
                openNodes.push(nodeCost);
                nodeInfo[cnx_ptr->node_ptr_] = (NodeInfo){ STATE_OPEN, node_ptr, node_cost_new };
            }
        }

    }

    path_container.resize(0);
    return PATH_NOT_FOUND;
}

#endif