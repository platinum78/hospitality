#include <cstdio>

int main(void)
{
    char str[] = "12, 23,  abc";
    char strBuf[10];
    int a, b, c;
    printf("Read: %d \n", sscanf(str, "%d,%d,%d,%s", &a, &b, &c, strBuf));

    printf("%d \n%d \n%d \n%s \n", a, b, c, strBuf);
}