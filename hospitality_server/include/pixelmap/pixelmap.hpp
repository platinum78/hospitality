#ifndef PIXEL_MAP_HPP_
#define PIXEL_MAP_HPP_

#include <cstdio>
#include <vector>
#include <string>
#include <list>
#include <map>
#include <queue>

#include <cmath>
#include <sys/types.h>
#include <dirent.h>
#include "../EasyBMP/EasyBMP.cpp"
#include "../errors.hpp"
#include "../geometry_datatypes.hpp"

#define MAP_CSV_COLUMNS     4

typedef unsigned char byte;

class PixelMap
{
public:
    PixelMap();
    PixelMap(const char *bmp_path, const char *csv_path, int origin_x, int origin_y, double resolution);
    ~PixelMap();

public:
    struct Pixel;
    struct PixelIdx;
    struct PixelInfo;
    struct PixelPathCost;
    struct PixelPathStat;

public:
    void ListDir(const char *dir);
    int ReadMapBmp(const char *bmp_path);
    int ReadMapBmp(const char *bmp_path, double roi_height, double resolution);
    int ReadMapCsv(const char *csv_path);

public:
    byte operator() (int r, int c, int floor);

private:
    bool IsIdxInMap(PixelIdx &idx);
    bool IsIdxInMap(int row, int col);

private:
    PixelIdx GetBoundingPixel(double x, double y);
    PixelIdx GetBoundingPixel(double x, double y, int floor);
    PointFloor<double> GetPixelCenterpoint(int row, int col);
    PointFloor<double> GetPixelCenterpoint(int row, int col, int floor);

private:
    const int delta_[8][2] = { {1, 0},      {1, 1},     {0, 1},     {-1, 1},
                               {-1, 0},     {-1, -1},   {0, -1},    {1, -1} };

public:
    int FindRoute(std::list<PixelIdx> &path_container, PixelIdx &start, PixelIdx &dest);

public:
    double DijkstraPath(std::list<PixelIdx> &path_container, PixelIdx &start, PixelIdx &dest);
    double AStarPath(std::list<PixelIdx> &path_container, PixelIdx &start, PixelIdx &dest);
    double ThetaStarPath(std::list<PixelIdx> &path_container, PixelIdx &start, PixelIdx &dest);

private:
    double DiagManhattan(PixelIdx &start, PixelIdx &dest);

private:
    std::map<int, int> floor_idx_table_;
    std::vector<std::vector<std::vector<Pixel> > > pixel_map_;
    std::list<std::string> file_list_;

private:
    int pixel_width_;
    int pixel_height_;
    int origin_row_;
    int origin_col_;
    double resolution_;
};

PixelMap::PixelMap() {}

PixelMap::PixelMap(const char *bmp_path, const char *csv_path, int origin_row, int origin_col, double resolution)
    : origin_row_(origin_row), origin_col_(origin_col), resolution_(resolution)
{
    ReadMapBmp(bmp_path);
    ReadMapCsv(csv_path);
}


struct PixelMap::PixelIdx
{
    int row;
    int col;
    int floor;
    bool operator==(const PixelIdx &idx) const
    {
        return (row == idx.row && col == idx.col);
    }
    bool operator!=(const PixelIdx &idx) const
    {
        return !(row == idx.row && col == idx.col);
    }
};

struct PixelMap::PixelInfo
{
    int place_code;
    std::string description;
};

struct PixelMap::PixelPathCost
{
    double cost_;
    double heuristic_;
    PixelIdx idx_;
    bool operator<(const PixelPathCost &p) const
    {
        if (cost_ + heuristic_ < p.cost_ + p.heuristic_)
            return true;
        else
            return false;
    }
    bool operator>(const PixelPathCost &p) const
    {
        if (cost_ + heuristic_ > p.cost_ + p.heuristic_)
            return true;
        else
            return false;
    }
};

struct PixelMap::PixelPathStat
{
    int pixel_state_;
    double cost_;
    double heuristic_;
    PixelIdx prev_pixel_;
    enum { STATE_NEW, STATE_OPEN, STATE_VISITED };
};

struct PixelMap::Pixel
{
    enum { STATE_FREE, STATE_BLOCKED, STATE_AMBIGUOUS };
    byte state;
    byte closest_wall_dist;
    PixelInfo *pixel_info;
    PixelPathStat path_stat;
    Pixel()
    {
        state = STATE_FREE;
        closest_wall_dist = 255;
        pixel_info = nullptr;
        path_stat.pixel_state_ = PixelPathStat::STATE_NEW;
        path_stat.cost_ = __DBL_MAX__;
        path_stat.heuristic_ = 0.0;
        path_stat.prev_pixel_ = (PixelIdx){ -1, -1, -1 };
    }
};

void PixelMap::ListDir(const char *path)
{
    DIR *dp;
    dirent *dirp;
    file_list_.resize(0);
    std::string fileName, dirPath;

    if ((dp = opendir(path)) == NULL)
        throw "No such directory! \n";
    
    dirPath.assign(path);
    
    while ((dirp = readdir(dp)) != NULL)
    {
        if (dirp->d_name[0] != '.')
            file_list_.push_back(dirp->d_name);
    }
    closedir(dp);
    file_list_.sort();
}

int PixelMap::ReadMapBmp(const char *path)
{
    ListDir(path);
    pixel_map_.resize(file_list_.size());
    floor_idx_table_.clear();

    std::string dirPath(path);
    std::list<std::string>::iterator iter;
    pixel_width_ = pixel_height_ = 0;
    int gray;

    int floor, mapIdx = 0;
    for (iter = file_list_.begin(); iter != file_list_.end(); iter++)
    {
        BMP image;
        image.ReadFromFile((dirPath + *iter).c_str());

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
            pixel_map_[mapIdx][i].resize(pixel_width_);

        for (int r = 0; r < pixel_height_; r++)
        {
            for (int c = 0; c < pixel_width_; c++)
            {
                gray = (int(image(c, r)->Red) + int(image(c, r)->Green) + int(image(c, r)->Blue)) / 3;
                if (gray < 127)
                    pixel_map_[mapIdx][r][c].state = Pixel::STATE_BLOCKED;
                else
                    pixel_map_[mapIdx][r][c].state = Pixel::STATE_FREE;
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

byte PixelMap::operator() (int r, int c, int floor)
{
    int mapIdx = floor_idx_table_[floor];
    return pixel_map_[mapIdx][r][c].state;
}

bool PixelMap::IsIdxInMap(PixelIdx &idx)
{
    return IsIdxInMap(idx.row, idx.col);
}

bool PixelMap::IsIdxInMap(int row, int col)
{
    if (row < 0 || row >= pixel_height_)
        return false;
    if (col < 0 || col >= pixel_width_)
        return false;
    return true;
}

PixelMap::PixelIdx PixelMap::GetBoundingPixel(double x, double y)
{
    int dpx = int(round(x / resolution_));
    int dpy = int(round(y / resolution_));

    PixelIdx idx;
    idx.row = origin_row_ - dpy;
    idx.col = origin_col_ + dpx;
    return idx;
}

PixelMap::PixelIdx PixelMap::GetBoundingPixel(double x, double y, int floor)
{
    int dpx = int(round(x / resolution_));
    int dpy = int(round(y / resolution_));

    PixelIdx idx;
    idx.row = origin_row_ - dpy;
    idx.col = origin_col_ + dpx;
    idx.floor = floor;
    return idx;
}

double PixelMap::DijkstraPath(std::list<PixelIdx> &path_container, PixelIdx &start, PixelIdx &dest)
{
    // Initialize path-finding values.
    int startMapIdx = floor_idx_table_[start.floor];
    int destMapIdx = floor_idx_table_[dest.floor];
    PixelIdx pxIdx = (PixelIdx){ -1, -1, 0 };
    for (int i = 0; i < pixel_height_; i++)
    {
        for (int j = 0; j < pixel_width_; j++)
        {
            pixel_map_[startMapIdx][i][j].path_stat = (PixelPathStat){ PixelPathStat::STATE_NEW, 0.0, 0.0, pxIdx };
            if (destMapIdx != startMapIdx)
                pixel_map_[startMapIdx][i][j].path_stat = (PixelPathStat){ PixelPathStat::STATE_NEW, 0.0, 0.0, pxIdx };
        }
    }
    
    // Start from pushing starting node into priority queue.
    std::priority_queue<PixelPathCost, std::vector<PixelPathCost>, std::greater<PixelPathCost> > openPixels;
    pxIdx = start;
    pixel_map_[startMapIdx][pxIdx.row][pxIdx.col].path_stat.cost_ = 0;
    PixelPathCost pxPathCost = (PixelPathCost){ 0.0, 0.0, pxIdx };
    openPixels.push(pxPathCost);
    double currentPathCost, newPathCost, totalPathCost;
    while (!openPixels.empty())
    {
        pxPathCost = openPixels.top(); openPixels.pop();
        pxIdx = pxPathCost.idx_;
        Pixel &pxCur = pixel_map_[startMapIdx][pxIdx.row][pxIdx.col];
        pxCur.path_stat.pixel_state_ = PixelPathStat::STATE_VISITED;
        currentPathCost = pxPathCost.cost_;

        // Found destination; return path and terminate.
        if (pxIdx == dest)
        {
            totalPathCost = pixel_map_[startMapIdx][pxIdx.row][pxIdx.col].path_stat.cost_;
            path_container.resize(0);
            do
            {
                path_container.push_front(pxIdx);
                pxIdx = pixel_map_[startMapIdx][pxIdx.row][pxIdx.col].path_stat.prev_pixel_;
            } while (pxIdx.row != -1);
            return totalPathCost;
        }

        // Expand current node.
        for (int i = 0; i < 8; i++)
        {
            int row = pxIdx.row + delta_[i][0];
            int col = pxIdx.col + delta_[i][1];
            Pixel &pxExp = pixel_map_[startMapIdx][row][col];

            if (IsIdxInMap(row, col) && pxExp.state == Pixel::STATE_FREE && 
                pxExp.path_stat.pixel_state_ != PixelPathStat::STATE_VISITED)
            {
                newPathCost = currentPathCost + sqrt(delta_[i][0] * delta_[i][0] + delta_[i][1] * delta_[i][1]);
                if (pxExp.path_stat.pixel_state_ == PixelPathStat::STATE_NEW || newPathCost < pxExp.path_stat.cost_)
                {
                    printf("(%4d, %4d): %.3lf \n", row, col, newPathCost);
                    pxExp.path_stat.cost_ = newPathCost;
                    pxExp.path_stat.pixel_state_ = PixelPathStat::STATE_OPEN;
                    pxExp.path_stat.prev_pixel_ = pxIdx;
                    pxIdx.row = row; pxIdx.col = col; pxIdx.floor = startMapIdx;
                    openPixels.push((PixelPathCost){ newPathCost, 0.0, pxIdx });
                }
            }
        }
    }

    path_container.resize(0);
    throw NoPathException("No path found.");
}

double PixelMap::AStarPath(std::list<PixelIdx> &path_container, PixelIdx &start, PixelIdx &dest)
{
    // Initialize path-finding values.
    int startMapIdx = floor_idx_table_[start.floor];
    int destMapIdx = floor_idx_table_[dest.floor];
    for (int i = 0; i < pixel_height_; i++)
    {
        for (int j = 0; j < pixel_width_; j++)
        {
            pixel_map_[startMapIdx][i][j].path_stat = (PixelPathStat){ PixelPathStat::STATE_NEW, 0.0, 0.0, (PixelIdx){ -1, -1, 0 } };
            if (destMapIdx != startMapIdx)
                pixel_map_[startMapIdx][i][j].path_stat = (PixelPathStat){ PixelPathStat::STATE_NEW, 0.0, 0.0, (PixelIdx){ -1, -1, 0 } };
        }
    }
    
    // Start from pushing starting node into priority queue.
    std::priority_queue<PixelPathCost, std::vector<PixelPathCost>, std::greater<PixelPathCost> > openPixels;
    double currentPathCost, newPathCost, totalPathCost;

    PixelIdx pxIdx = start;
    pixel_map_[startMapIdx][pxIdx.row][pxIdx.col].path_stat.cost_ = 0;
    pixel_map_[startMapIdx][pxIdx.row][pxIdx.col].path_stat.pixel_state_ = PixelPathStat::STATE_OPEN;
    PixelPathCost pxPathCost = (PixelPathCost){ 0.0, DiagManhattan(start, dest), pxIdx };
    openPixels.push(pxPathCost);
    double termCond = false;

    while (!openPixels.empty())
    {
        while (1)
        {
            pxPathCost = openPixels.top(); openPixels.pop();
            pxIdx = pxPathCost.idx_;
            PixelPathStat &pathStat = pixel_map_[startMapIdx][pxIdx.row][pxIdx.col].path_stat;

            if (pathStat.pixel_state_ == PixelPathStat::STATE_OPEN)
                break;
            if (pathStat.pixel_state_ == PixelPathStat::STATE_VISITED)
                if (pxPathCost.cost_ + pxPathCost.heuristic_ < pathStat.cost_ + pathStat.heuristic_)
                {
                    pathStat.cost_ = pxPathCost.cost_;
                    break;
                }
            if (openPixels.empty())
            {
                termCond = true;
                break;
            }
        }
        if (termCond)
            break;
        pixel_map_[startMapIdx][pxIdx.row][pxIdx.col].path_stat.pixel_state_ = PixelPathStat::STATE_VISITED;

        // Found destination; return path and terminate.
        if (pxIdx == dest)
        {
            totalPathCost = pixel_map_[startMapIdx][pxIdx.row][pxIdx.col].path_stat.cost_;
            path_container.resize(0);
            do
            {
                path_container.push_front(pxIdx);
                pxIdx = pixel_map_[startMapIdx][pxIdx.row][pxIdx.col].path_stat.prev_pixel_;
            } while (pxIdx.row != -1);
            return totalPathCost;
        }

        // Expand current node.
        for (int i = 0; i < 8; i++)
        {
            int row = pxIdx.row + delta_[i][0];
            int col = pxIdx.col + delta_[i][1];
            Pixel &pxCur = pixel_map_[startMapIdx][pxIdx.row][pxIdx.col];
            Pixel &pxExp = pixel_map_[startMapIdx][row][col];

            if (IsIdxInMap(row, col) && pxExp.state == Pixel::STATE_FREE &&
                pxExp.path_stat.pixel_state_ != PixelPathStat::STATE_VISITED)
            {
                newPathCost = pxCur.path_stat.cost_ + sqrt(delta_[i][0] * delta_[i][0] + delta_[i][1] * delta_[i][1]);
                // printf("(%4d, %4d): %.3lf, %.3lf \n", row, col, newPathCost, DiagManhattan(pxIdx, dest));
                if (pxExp.path_stat.pixel_state_ == PixelPathStat::STATE_NEW || newPathCost < pxExp.path_stat.cost_)
                {
                    pxExp.path_stat.cost_ = newPathCost;
                    pxExp.path_stat.pixel_state_ = PixelPathStat::STATE_OPEN;
                    pxExp.path_stat.prev_pixel_ = pxIdx;
                    pxIdx.row = row; pxIdx.col = col; pxIdx.floor = startMapIdx;
                    openPixels.push((PixelPathCost){ newPathCost, DiagManhattan(pxIdx, dest), pxIdx });
                }
            }
        }
    }

    path_container.resize(0);
    throw NoPathException("No path found.");
}

double PixelMap::DiagManhattan(PixelIdx &start, PixelIdx &dest)
{
    double dr = start.row - dest.row;
    double dc = start.col - dest.col;
    dr = (dr > 0 ? dr : -dr);
    dc = (dc > 0 ? dc : -dc);

    double diagLen = (dr < dc ? dr : dc);
    double remnLen = (dr > dc ? dr - dc : dc - dr);

    return diagLen * sqrt(2) + remnLen;
}

PixelMap::~PixelMap()
{
    // Destroy pixel map.
    for (int i = 0; i < pixel_map_.size(); i++)
    {
        for (int j = 0; j < pixel_map_[i].size(); j++)
            pixel_map_[i][j].resize(0);
        pixel_map_[i].resize(0);
    }
    pixel_map_.resize(0);

    file_list_.resize(0);
    floor_idx_table_.clear();
}

#endif