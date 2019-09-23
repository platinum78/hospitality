#include <cstdio>
#include <string>

int main(void)
{
    std::string a("hello");
    std::string b("world");
    printf("%s \n", (a + b).c_str());
}