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

#include "hospitality_msgs/PointFloor.h"

#define MAP_CSV_COLUMNS     4

typedef unsigned char byte;



////////////////////////////////////////////////////////////////////////////////
//                              Class Declaration                             //
////////////////////////////////////////////////////////////////////////////////

class PixelMap
{
public:
    PixelMap();
    PixelMap(const char *bmp_path, const char *csv_path, int origin_x, int origin_y, double resolution);
    ~PixelMap();
    void SetParams(int origin_row, int origin_col, double roi_width, double roi_height, double pad_radius);

public:
    struct Pixel;
    struct PixelIdx;
    struct PlaceInfo;
    struct PixelPathCost;
    struct PixelPathHCost;
    struct PixelPathStat;

public:
    enum { PATH_STATE_NEW, PATH_STATE_OPEN, PATH_STATE_VISITED };
    enum { PIXEL_STATE_FREE, PIXEL_STATE_PAD, PIXEL_STATE_BLOCKED, PIXEL_STATE_AMBIGUOUS };
    enum { PATH_REDUCTION_STRAIGHT_LINE, PATH_REDUCTION_LOS };

public:
    void SetOriginPixel(int row, int col);
    void ListDir(const char *dir);
    int ReadMapBmp(const char *bmp_path);
    int ReadMapBmp(const char *bmp_path, double roi_height, double resolution);
    int ReadMapCsv(const char *csv_path);
    void MakePadding(double pad_radius, double map_idx);

public:
    byte operator() (int r, int c, int floor);

private:
    bool IsIdxInMap(PixelIdx &idx);
    bool IsIdxInMap(int row, int col);

public:
    PixelIdx GetBoundingPixel(double x, double y);
    PixelIdx GetBoundingPixel(double x, double y, int floor);
    hospitality_msgs::PointFloor GetPixelCenterpoint(int row, int col);
    hospitality_msgs::PointFloor GetPixelCenterpoint(int row, int col, int floor);

public:
    std::list<PlaceInfo *> &GetPlaceList() { return place_list_; }

private:
    const int delta_[8][2] = { {0, 1},      {-1, 1},   {-1, 0},   {-1, -1},
                               {0, -1},     {1, -1},   {1, 0},    {1, 1} };

public:
    int FindRoute(std::list<PixelIdx> &path_container, PixelIdx &start, PixelIdx &dest);

public:
    double DijkstraPath(std::list<PixelIdx> &path_container, PixelIdx &start, PixelIdx &dest);
    double AStarPath(std::list<PixelIdx> &path_container, PixelIdx &start, PixelIdx &dest);
    double ThetaStarPath(std::list<PixelIdx> &path_container, PixelIdx &start, PixelIdx &dest);

private:
    double DiagManhattan(PixelIdx &start, PixelIdx &dest);

public:
    void ReducePath(std::list<PixelIdx> &path_container, int mode);

private:
    void PathReductionLOS(std::list<PixelIdx> &path_container);
    void PathReductionStraightLine(std::list<PixelIdx> &path_container);

private:
    std::map<int, int> floor_idx_table_;
    std::vector<std::vector<std::vector<Pixel> > > pixel_map_;
    std::list<PlaceInfo *> place_list_;
    std::list<std::string> file_list_;
    std::vector<std::vector<bool> > pad_filter_;

private:
    int pixel_width_;
    int pixel_height_;
    double roi_width_;
    double roi_height_;
    int origin_row_;
    int origin_col_;
    double resolution_;
    double pad_radius_;
};



////////////////////////////////////////////////////////////////////////////////
//                               Class Definition                             //
////////////////////////////////////////////////////////////////////////////////


PixelMap::PixelMap() {}


PixelMap::PixelMap(const char *bmp_path, const char *csv_path, int origin_row, int origin_col, double resolution)
    : origin_row_(origin_row), origin_col_(origin_col), resolution_(resolution)
{
    ReadMapBmp(bmp_path);
    ReadMapCsv(csv_path);
}


void PixelMap::SetParams(int origin_row, int origin_col, double roi_width, double roi_height, double pad_radius)
{
    origin_row_ = origin_row;
    origin_col_ = origin_col;
    roi_width_ = roi_width;
    roi_height_ = roi_height;
    pad_radius_ = pad_radius;
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
    void operator=(const PixelIdx &idx)
    {
        row = idx.row;
        col = idx.col;
        floor = idx.floor;
    }
};


struct PixelMap::PlaceInfo
{
    int place_id;
    std::string place_code;
    hospitality_msgs::PointFloor coordinate_;
};


struct PixelMap::PixelPathCost
{
    double cost_;
    PixelIdx idx_;
    bool operator<(const PixelPathCost &p) const
    {
        if (cost_ < p.cost_)
            return true;
        else
            return false;
    }
    bool operator>(const PixelPathCost &p) const
    {
        if (cost_ > p.cost_)
            return true;
        else
            return false;
    }
    void operator=(const PixelPathCost &p)
    {
        cost_ = p.cost_;
        idx_ = p.idx_;
    }
};


struct PixelMap::PixelPathHCost
{
    double cost_;
    double heuristic_;
    PixelIdx idx_;
    bool operator<(const PixelPathHCost &p) const
    {
        if (cost_ + heuristic_ < p.cost_ + p.heuristic_)
            return true;
        else
            return false;
    }
    bool operator>(const PixelPathHCost &p) const
    {
        if (cost_ + heuristic_ > p.cost_ + p.heuristic_)
            return true;
        else
            return false;
    }
    void operator=(const PixelPathHCost &p)
    {
        cost_ = p.cost_;
        heuristic_ = p.heuristic_;
        idx_ = p.idx_;
    }
};


struct PixelMap::PixelPathStat
{
    int pixel_state_;
    double cost_;
    double heuristic_;
    PixelIdx prev_pixel_;
};


struct PixelMap::Pixel
{
    byte state;
    PlaceInfo *place_info;
    PixelPathStat path_stat;
    Pixel()
    {
        state = PIXEL_STATE_FREE;
        place_info = nullptr;
        path_stat.pixel_state_ = PATH_STATE_NEW;
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
        image.ReadFromFile((dirPath + std::string("/") + *iter).c_str());

        if (!pixel_width_ && !pixel_height_)
        {
            pixel_width_ = image.TellWidth();
            pixel_height_ = image.TellHeight();
            resolution_ = (roi_width_ / pixel_width_ + roi_height_ / pixel_height_) / 2;
        }
        else if (pixel_width_ && pixel_height_)
        {
            if (!(pixel_width_ == image.TellWidth() && pixel_height_ == image.TellHeight()))
                throw DimensionException("Map resolution is not consistent.");
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
                    pixel_map_[mapIdx][r][c].state = PIXEL_STATE_BLOCKED;
                else
                    pixel_map_[mapIdx][r][c].state = PIXEL_STATE_FREE;
            }
        }
        
        MakePadding(pad_radius_, mapIdx);
        mapIdx++;
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
        throw ArgumentException("Number of columns in CSV file is improper.");
    
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
            PlaceInfo *pxInfo = new PlaceInfo;
            pxInfo->place_id = placeCode;
            pixel_map_[mapIdx][pxIdx.row][pxIdx.col].place_info = pxInfo;
        }
        else
            break;
    }
    
    if (!lineComplete)
        printf("WARN: Finished import, but trailing line is incomplete.");
}


void PixelMap::MakePadding(double pad_radius, double map_idx)
{
    int padRadPx = int(pad_radius / resolution_);
    int padDiaPx = 2 * padRadPx - 1;
    
    for (int r = 0; r < pixel_height_; r++)
    {
        for (int c = 0; c < pixel_width_; c++)
        {
            Pixel &px = pixel_map_[map_idx][r][c];
            if (px.state == PIXEL_STATE_BLOCKED)
                for (int v = -padRadPx; v <= padRadPx; v++)
                    for (int h = -padRadPx; h <= padRadPx; h++)
                        if ((v != 0 || h != 0) && IsIdxInMap(r + v, c + h) &&
                            pow(v * resolution_, 2) + pow(h * resolution_, 2) < pow(pad_radius, 2))
                            if (pixel_map_[map_idx][r + v][c + h].state == PIXEL_STATE_FREE)
                                pixel_map_[map_idx][r + v][c + h].state = PIXEL_STATE_PAD;
        }
    }
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


hospitality_msgs::PointFloor PixelMap::GetPixelCenterpoint(int row, int col)
{
    GetPixelCenterpoint(row, col, 0);
}


hospitality_msgs::PointFloor PixelMap::GetPixelCenterpoint(int row, int col, int floor)
{
    int dpr = row - origin_row_;
    int dpc = col - origin_col_;

    hospitality_msgs::PointFloor point;
    point.x = double(dpc) * resolution_;
    point.y = double(-dpr) * resolution_;
    point.floor = floor;

    return point;
}


double PixelMap::DijkstraPath(std::list<PixelIdx> &path_container, PixelIdx &start, PixelIdx &dest)
{
    // Initialize path-finding values.
    int startMapIdx = floor_idx_table_[start.floor];
    int destMapIdx = floor_idx_table_[dest.floor];
    PixelIdx pxIdx;
    for (int i = 0; i < pixel_height_; i++)
    {
        for (int j = 0; j < pixel_width_; j++)
        {
            pixel_map_[startMapIdx][i][j].path_stat.pixel_state_ = PATH_STATE_NEW;
            // pixel_map_[destMapIdx][i][j].path_stat.pixel_state_ = PATH_STATE_NEW;
        }
    }
    printf("Finding destination... \n");
    
    std::priority_queue<PixelPathCost, std::vector<PixelPathCost>, std::greater<PixelPathCost> > openPixels;
    double currentPathCost, newPathCost, totalPathCost;
    
    pxIdx = start;
    pixel_map_[startMapIdx][pxIdx.row][pxIdx.col].path_stat.cost_ = 0.0;
    pixel_map_[startMapIdx][pxIdx.row][pxIdx.col].path_stat.prev_pixel_ = (PixelIdx){ -1, -1, 0 };
    PixelPathCost pxPathCost = (PixelPathCost){ 0.0, pxIdx };
    openPixels.push(pxPathCost);
    const double SQRT2 = sqrt(2.0);

    while (!openPixels.empty())
    {
        pxPathCost = openPixels.top(); openPixels.pop();
        pxIdx = pxPathCost.idx_;
        Pixel &pxCur = pixel_map_[startMapIdx][pxIdx.row][pxIdx.col];
        pxCur.path_stat.pixel_state_ = PATH_STATE_VISITED;
        currentPathCost = pxPathCost.cost_;

        // Found destination; return path and terminate.
        if (pxIdx == dest)
        {
            printf("Destination found! Constructing path list... \n");
            totalPathCost = pxCur.path_stat.cost_;
            ROS_WARN("Flag 1");
            path_container.resize(0);
            do
            {
                printf("%d %d \n", pxIdx.row, pxIdx.col);
                path_container.push_front(pxIdx);
                pxIdx = pixel_map_[startMapIdx][pxIdx.row][pxIdx.col].path_stat.prev_pixel_;
            } while (pxIdx.row != -1);

            ROS_WARN("Flag 3");
            printf("Finished! \n");
            return totalPathCost;
        }

        // Expand current node.
        for (int i = 0; i < 8; i++)
        {
            int row = pxIdx.row + delta_[i][0];
            int col = pxIdx.col + delta_[i][1];
            Pixel &pxExp = pixel_map_[startMapIdx][row][col];

            if (IsIdxInMap(row, col) && pxExp.state == PIXEL_STATE_FREE && 
                pxExp.path_stat.pixel_state_ != PATH_STATE_VISITED)
            {
                newPathCost = currentPathCost + ((i % 2) ? SQRT2 : 1.0);
                if (pxExp.path_stat.pixel_state_ == PATH_STATE_NEW ||
                    newPathCost < pxExp.path_stat.cost_)
                {
                    // printf("(%4d, %4d): %.3lf \n", row, col, newPathCost);
                    pxExp.path_stat.cost_ = newPathCost;
                    pxExp.path_stat.pixel_state_ = PATH_STATE_OPEN;
                    pxExp.path_stat.prev_pixel_ = pxIdx;
                    pxPathCost.cost_ = newPathCost; pxPathCost.idx_ = pxIdx;
                    openPixels.push((PixelPathCost){ newPathCost, (PixelIdx){ row, col, startMapIdx } });
                }
            }
        }
    }

    path_container.resize(0);
    printf("Failed to find destination! \n");
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
            pixel_map_[startMapIdx][i][j].path_stat = (PixelPathStat){ PATH_STATE_NEW, 0.0, 0.0, (PixelIdx){ -1, -1, 0 } };
            if (destMapIdx != startMapIdx)
                pixel_map_[startMapIdx][i][j].path_stat = (PixelPathStat){ PATH_STATE_NEW, 0.0, 0.0, (PixelIdx){ -1, -1, 0 } };
        }
    }
    
    // Start from pushing starting node into priority queue.
    std::priority_queue<PixelPathHCost, std::vector<PixelPathHCost>, std::greater<PixelPathHCost> > openPixels;
    double currentPathCost, newPathCost, totalPathCost;

    PixelIdx pxIdx = start;
    pixel_map_[startMapIdx][pxIdx.row][pxIdx.col].path_stat.cost_ = 0;
    pixel_map_[startMapIdx][pxIdx.row][pxIdx.col].path_stat.pixel_state_ = PATH_STATE_OPEN;
    PixelPathHCost pxPathCost = (PixelPathHCost){ 0.0, DiagManhattan(start, dest), pxIdx };
    openPixels.push(pxPathCost);
    double termCond = false;

    while (!openPixels.empty())
    {
        while (1)
        {
            pxPathCost = openPixels.top(); openPixels.pop();
            pxIdx = pxPathCost.idx_;
            PixelPathStat &pathStat = pixel_map_[startMapIdx][pxIdx.row][pxIdx.col].path_stat;

            if (pathStat.pixel_state_ == PATH_STATE_OPEN)
                break;
            if (pathStat.pixel_state_ == PATH_STATE_VISITED)
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
        pixel_map_[startMapIdx][pxIdx.row][pxIdx.col].path_stat.pixel_state_ = PATH_STATE_VISITED;

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

            if (IsIdxInMap(row, col) && pxExp.state == PIXEL_STATE_FREE &&
                pxExp.path_stat.pixel_state_ != PATH_STATE_VISITED)
            {
                newPathCost = pxCur.path_stat.cost_ + sqrt(delta_[i][0] * delta_[i][0] + delta_[i][1] * delta_[i][1]);
                if (pxExp.path_stat.pixel_state_ == PATH_STATE_NEW || newPathCost < pxExp.path_stat.cost_)
                {
                    printf("(%4d, %4d): %.3lf, %.3lf \n", row, col, newPathCost, DiagManhattan(pxIdx, dest));
                    pxExp.path_stat.cost_ = newPathCost;
                    pxExp.path_stat.pixel_state_ = PATH_STATE_OPEN;
                    pxExp.path_stat.prev_pixel_ = pxIdx;
                    openPixels.push((PixelPathHCost){ newPathCost, DiagManhattan(pxIdx, dest), (PixelIdx){ row, col, startMapIdx } });
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


void PixelMap::ReducePath(std::list<PixelIdx> &path_container, int mode = PATH_REDUCTION_STRAIGHT_LINE)
{
    if (mode == PATH_REDUCTION_STRAIGHT_LINE)
        PathReductionStraightLine(path_container);
}


void PixelMap::PathReductionStraightLine(std::list<PixelIdx> &path_container)
{
    std::list<PixelIdx>::iterator iterHead, iterTail;
    iterHead = iterTail = path_container.begin(); iterHead++;

    int dr = iterHead->row - iterTail->row;
    int dc = iterHead->col - iterTail->col;
    std::advance(iterTail, 1);
    iterHead = iterTail;
    std::advance(iterHead, 1);

    while (iterHead != path_container.end())
    {
        if (iterHead->row - iterTail->row == dr && iterHead->col - iterTail->col == dc)
        {
            do
            {
                path_container.erase(iterTail++);
                iterHead = iterTail; iterHead++;
            } while (iterHead != path_container.end() && 
                     (iterHead->row - iterTail->row) == dr &&
                     (iterHead->col - iterTail->col) == dc);
        }
        else
        {
            dr = iterHead->row - iterTail->row;
            dc = iterHead->col - iterTail->col;
            iterTail++; iterHead = iterTail; iterHead++;
        }
    }
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