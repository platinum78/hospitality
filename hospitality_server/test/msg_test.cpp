#include <cstdio>
#include <list>

class PointFloor
{
public:
    PointFloor() {}

public:
    double x;
    double y;
    int floor;
};

int main(void)
{
    std::list<PointFloor> list;
    for (int i = 0; i < 100; i++)
    {
        PointFloor p;
        p.x = (double)i;
        p.y = (double)i;
        p.floor = i;
        list.push_back(p);
    }

    for (std::list<PointFloor>::iterator iter = list.begin(); iter != list.end(); iter++)
        printf("p.x = %lf, p.y = %lf, floor = %d \n", iter->x, iter->y, iter->floor);

    PointFloor p1;
    p1.x = 1; p1.y = 1; p1.floor = 1;
    PointFloor p2 = p1;
    p2.x = 10;
    printf("%lf \n", p1.x);
}