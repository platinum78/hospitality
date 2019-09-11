#include <cstdio>
#include <map>
#include <map>

struct Node
{
    int a;
    int b;
};

int main(void)
{
    std::map<int, Node> myMap;
    myMap[1].a = 10;
    myMap[1].b = 20;
    // myMap.insert(std::pair<char, int>('a', 100));
    // myMap.insert(std::pair<char, int>('b', 200));

    printf("%d \n", myMap[1].a);
    myMap[1].b = 300;
    printf("%d \n", myMap[1].b);
    return 0;
}