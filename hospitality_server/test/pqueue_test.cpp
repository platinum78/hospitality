#include <cstdio>
#include <queue>
#include <list>

int main(void)
{
    std::priority_queue<int, std::vector<int>, std::greater<int> > pQueue;
    int num[10] = { 10, 4, 2, 6, 5, 9, -3, 7, 1, 0 };

    for (int i = 0; i < 10; i++)
        pQueue.push(num[i]);
    
    for (int i = 0; i < 10; i++)
    {
        printf("%d \n", pQueue.top());
        pQueue.pop();
    }
}