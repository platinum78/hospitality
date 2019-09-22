#ifndef PLACE_INFO_HPP_
#define PLACE_INFO_HPP_

class PlaceInfoCsvReader
{
public:
    PlaceInfoCsvReader();

public:
    struct Item;

public:
    bool ReadCSV(std::string &path);
};

#endif