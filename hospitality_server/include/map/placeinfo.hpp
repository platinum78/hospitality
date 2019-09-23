#ifndef PLACE_INFO_HPP_
#define PLACE_INFO_HPP_

#include <cstdio>
#include <string>
#include <list>

#include "hospitality_msgs/PointFloor.h"
#include "../errors.hpp"

class PlaceInfo
{
public:
    PlaceInfo();

public:
    struct Item;
    enum { SEARCH_ID, SEARCH_NAME, SEARCH_CODE };

public:
    int ReadCSV(std::string &path);

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

int PlaceInfo::ReadCSV(std::string &path)
{
    // CSV 파일은 장소 ID, 호실 번호, 장소 이름, x 좌표, y 좌표, 층  순서로 구성됨.
    FILE *fp = fopen(path.c_str(), "r");
    char placeCodeBuf[100], placeNameBuf[100];
    long placeID;
    std::string placeCode, placeName;
    double x, y;
    int floor;

    fseek(fp, 0, SEEK_SET);
    while (fscanf(fp, "%ld,%s,%s,%lf,%lf,%d", &placeID, placeCodeBuf, placeNameBuf, &x, &y, &floor) == 5)
    {
        Item *item_ptr = new Item;
        item_ptr->place_id_ = placeID;
        item_ptr->location_.x = x;
        item_ptr->location_.y = y;
        item_ptr->location_.floor = floor;
        item_ptr->place_code_.assign(placeCodeBuf);
        item_ptr->place_code_.assign(placeNameBuf);
        place_list_.push_back(item_ptr);
    }

    return place_list_.size();
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