#ifndef TASK_HPP_
#define TASK_HPP_

#include <iostream>
#include <string>

struct TimeBound;
struct ParcelInfo;

struct Task
{
    enum { TYPE_DELIVERY, TYPE_MOVE, TYPE_FOLLOW };
    enum { STATUS_PLANNED, STATUS_NOTIFIED, STATUS_PROCESSING, STATUS_DONE };

    long task_id_;
    std::string demander_;
    int task_type_;
    TimeBound *time_bound_;
    ParcelInfo *parcel_info_;
};

std::ostream &operator<<(std::ostream &out, Task &task)
{
    out << "Task ID: " << task.task_id_ << std::endl;
    out << "Task Demander: " << task.demander_ << std::endl;
    out << "Task Type: " << task.task_type_ << std::endl;
    return out;
}

Task *newTask()
{
    Task *pTask = new Task;
    return pTask;
}

Task *newTask(long task_id)
{
    Task *pTask = new Task;
    pTask->task_id_ = task_id;
    return pTask;
}



#endif