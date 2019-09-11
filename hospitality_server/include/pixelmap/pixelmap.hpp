#ifndef PIXEL_MAP_HPP_
#define PIXEL_MAP_HPP_

#include <cstdio>
#include <vector>
#include <string>
#include <list>
#include <map>

#include <sys/types.h>
#include <dirent.h>
#include "../EasyBMP/EasyBMP.h"
#include "../errors.hpp"
#include "../geometry_datatypes.hpp"

#define MAP_CSV_COLUMNS     4

typedef unsigned char byte;

struct PixelIdx
{
    int row;
    int col;
    int floor;
};

struct PixelInfo
{
    int place_code;
    std::string description;
};

struct Pixel
{
    enum { STATE_FREE, STATE_BLOCKED, STATE_AMBIGUOUS };
    byte state;
    byte closest_wall_dist;
    PixelInfo *pixel_info;
    Pixel() { state = STATE_FREE; closest_wall_dist = 255; pixel_info = nullptr; }
};

class PixelMap
{
public:
    PixelMap();
    PixelMap(const char *bmp_path, const char *csv_path, double roi_width, double roi_height, double resolution);

public:
    void ListDir(const char *dir);
    int ReadMapBmp(const char *bmp_path);
    int ReadMapBmp(const char *bmp_path, double roi_height, double resolution);
    int ReadMapCsv(const char *csv_path);

public:
    

public:
    int FindRoute(std::list<PixelIdx> &path_container, PixelIdx &start, PixelIdx &dest);

public:
    void DijkstraPath(std::list<PixelIdx> &path_container, PixelIdx &start, PixelIdx &dest);
    void AStarPath(std::list<PixelIdx> &path_container, PixelIdx &start, PixelIdx &dest);
    void ThetaStarPath(std::list<PixelIdx> &path_container, PixelIdx &start, PixelIdx &dest);

public:
    PixelIdx GetBoundingPixel(double x, double y);
    PixelIdx GetBoundingPixel(double x, double y, int floor);
    PointFloor<double> GetPixelCenterpoint(int row, int col);
    PointFloor<double> GetPixelCenterpoint(int row, int col, int floor);

private:
    std::map<int, int> floor_idx_table_;
    std::vector<std::vector<std::vector<Pixel> > > pixel_map_;
    std::list<std::string> file_list_;

private:
    int pixel_width_;
    int pixel_height_;
    double roi_width_;
    double roi_height_;
    double resolution_;
};

PixelMap::PixelMap() {}

PixelMap::PixelMap(const char *bmp_path, const char *csv_path, double roi_width, double roi_height, double resolution)
    : roi_width_(roi_width), roi_height_(roi_height), resolution_(resolution)
{
    ReadMapBmp(bmp_path);
}

void PixelMap::ListDir(const char *path)
{
    DIR *dp;
    dirent *dirp;
    file_list_.resize(0);

    if ((dp = opendir(path)) == NULL)
        throw "No such directory! \n";
    
    while ((dirp = readdir(dp)) != NULL)
        file_list_.push_back(dirp->d_name);
    closedir(dp);
    file_list_.sort();
    
    std::list<std::string>::iterator iter;
    for (iter = file_list_.begin(); iter != file_list_.end(); iter++)
    {
        std::cout << *iter << std::endl;
    }
}

int PixelMap::ReadMapBmp(const char *path)
{
    ListDir(path);
    pixel_map_.resize(file_list_.size());
    floor_idx_table_.clear();

    std::list<std::string>::iterator iter;
    pixel_width_ = pixel_height_ = 0;
    int gray;

    int floor, mapIdx = 0;
    for (iter = file_list_.begin(); iter != file_list_.end(); iter++)
    {
        BMP image;
        image.ReadFromFile(iter->c_str());

        if (!pixel_width_ && !pixel_height_)
        {
            pixel_width_ = image.TellWidth();
            pixel_height_ = image.TellHeight();
        }
        else if (pixel_width_ && pixel_height_)
        {
            if (!(pixel_width_ == image.TellWidth() && pixel_height_ == image.TellHeight()))
                throw DimensionError("Map resolution is not consistent.");
        }
        else
        {
            throw RuntimeException("Improper flag.");
        }

        sscanf(iter->c_str(), "map_floor_%03d.bmp", &floor);
        floor_idx_table_[floor] = mapIdx;

        pixel_map_[mapIdx].resize(pixel_height_);
        for (int i = 0; i < pixel_height_; i++)
            pixel_map_[floor][i].resize(pixel_width_);

        for (int r = 0; r < pixel_height_; r++)
        {
            for (int c = 0; c < pixel_width_; c++)
            {
                gray = (int(image(c, r)->Red) + int(image(c, r)->Green) + int(image(c, r)->Blue)) / 3;
                if (gray < 127)
                    pixel_map_[floor][r][c].state = Pixel::STATE_BLOCKED;
                else
                    pixel_map_[floor][r][c].state = Pixel::STATE_FREE;
            }
        }
    }
}

int PixelMap::ReadMapCsv(const char *file_path)
{
    FILE *fp = fopen(file_path, "r");
    
    // Check if file is properly formatted.
    char buf;
    int commaCnt = 0;
    do
    {
        buf = fgetc(fp);
        if (buf == ',')
            commaCnt++;
    } while (buf != '\n');

    if (commaCnt != MAP_CSV_COLUMNS - 1)
        throw ArgumentError("Number of columns in CSV file is improper.");
    
    fseek(fp, 0, SEEK_SET);
    double x, y;
    int floor, placeCode, mapIdx;
    bool lineComplete;
    while (1)
    {
        lineComplete = true;
        if (fscanf(fp, "%lf", &x) < 0 || fscanf(fp, "%lf", &y) < 0 || 
            fscanf(fp, "%d", &placeCode) < 0 || fscanf(fp, "%d", &placeCode) < 0) 
            lineComplete = false;
        
        if (lineComplete)
        {
            PixelIdx pxIdx = GetBoundingPixel(x, y, floor);
            mapIdx = floor_idx_table_[floor];
            PixelInfo *pxInfo = new PixelInfo;
            pxInfo->place_code = placeCode;
            pixel_map_[mapIdx][pxIdx.row][pxIdx.col].pixel_info = pxInfo;
        }
        else
            break;
    }
    
    if (!lineComplete)
        printf("WARN: Finished import, but trailing line is incomplete.");
}

#endif