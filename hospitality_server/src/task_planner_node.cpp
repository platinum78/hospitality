#ifndef TASK_PLANNER_NODE_CPP_
#define TASK_PLANNER_NODE_CPP_

#include <cstdio>
#include <iostream>
#include <map>
#include <vector>
#include <list>

#include <ros/ros.h>
#include "../include/task_planning/task.hpp"

struct TaskPool
{
    TaskPool();
    int task_id_max_;
    std::list<Task *> task_list_;
};

TaskPool::TaskPool() : task_id_max_(0) {}

class TaskPlannerNode
{
public:
    TaskPlannerNode();
    ~TaskPlannerNode();

public:
    int AssignTask(Task *task_ptr);
    int AssignTaskTo(int robot_id, Task *task_ptr);

public:
    bool AbortTask(Task *task_ptr);

private:
    ros::NodeHandle n;

private:
    std::map<int, std::list<TaskPool> > task_allocation_;
};

TaskPlannerNode::TaskPlannerNode()
{
    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_planner_node");
}

#endif