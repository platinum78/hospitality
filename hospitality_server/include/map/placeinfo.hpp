#ifndef PLACE_INFO_HPP_
#define PLACE_INFO_HPP_

#include <cstdio>
#include <string>
#include <list>

#include "hospitality_msgs/PointFloor.h"
#include "../utils/csv.hpp"
#include "../errors.hpp"

class PlaceInfo
{
public:
    PlaceInfo();

public:
    struct Item;
    enum { SEARCH_ID, SEARCH_NAME, SEARCH_CODE };

public:
    int ReadCSV(std::string &path, double coordinate_scale);

public:
    
public:
    hospitality_msgs::PointFloor &QueryPlace(long place_id);
    hospitality_msgs::PointFloor &QueryPlace(std::string &text, int search_mode);

private:
    std::list<Item *> place_list_;
};

PlaceInfo::PlaceInfo() {}

struct PlaceInfo::Item
{
    long place_id_;
    std::string place_code_;
    std::string place_name_;
    hospitality_msgs::PointFloor location_;
};

int PlaceInfo::ReadCSV(std::string &csv_path, double coordinate_scale = 1.0)
{
    CSVParser parser(csv_path);
    std::list<std::list<std::string> > dataContainer;
    parser.Parse(dataContainer);

    std::list<std::list<std::string> >::iterator rowIter;
    std::list<std::string>::iterator colIter;
    Item *pItem;

    for (rowIter = dataContainer.begin(); rowIter != dataContainer.end(); rowIter++)
    {
        if (rowIter->size() == 4)
        {
            pItem = new Item;
            colIter = rowIter->begin();
        }
        else continue;
        
        pItem->place_id_ = atoll(colIter->c_str());
        colIter++;
        pItem->place_name_.assign(*colIter);
        colIter++;
        pItem->location_.x = atof(colIter->c_str()) * coordinate_scale;
        colIter++;
        pItem->location_.y = atof(colIter->c_str()) * coordinate_scale;
        pItem->location_.floor = 1;
    }
}

hospitality_msgs::PointFloor &PlaceInfo::QueryPlace(long place_id)
{
    
    std::list<Item *>::iterator iter;
    for (iter = place_list_.begin(); iter != place_list_.end(); iter++)
    {
        Item *item_ptr = *iter;
        if (item_ptr->place_id_ == place_id)
            return item_ptr->location_;
    }
    throw NoSuchNodeException("No such place.");
}

hospitality_msgs::PointFloor &PlaceInfo::QueryPlace(std::string &text, int search_mode)
{
    std::list<Item *>::iterator iter;
    for (iter = place_list_.begin(); iter != place_list_.end(); iter++)
    {
        Item *item_ptr = *iter;
        if (search_mode == SEARCH_CODE)
            if (item_ptr->place_code_.compare(text) == 0)
                return item_ptr->location_;
        else if (search_mode == SEARCH_NAME)
            if (item_ptr->place_name_.compare(text) == 0)
                return item_ptr->location_;
    }
    throw NoSuchNodeException("No such place.");
}

#endif