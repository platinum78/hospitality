#include <cstdio>
#include <string>
#include <unordered_map>

int main(void)
{
    std::unordered_map<int, std::string> map;
    map[3] = std::string("Hello, world!");
    map[2] = std::string("Hello!");

    for (std::unordered_map<int, std::string>::iterator iter = map.begin(); iter != map.end(); iter++)
        printf("%s \n", iter->second.c_str());
}