#include <cstdio>
#include <string>

struct Test
{
    std::string str;
};

int main(void)
{
    Test t1;
    Test t2;

    t1.str.assign("abcdefg");
    t2.str.assign("hijklmnopqrstuvwxyz");

    printf("%d \n", sizeof(t1));
    printf("%d \n", sizeof(t2));
}