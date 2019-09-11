#include <cstdio>
#include "../include/pixelmap/pixelmap.hpp"

int main(void)
{
    FILE *fp = fopen("input.txt", "r");
    char str1[10], str2[10], str3[10];
    int tokens = 0;
    tokens = fscanf(fp, "%s %s %s", str1, str2, str3);
    printf("%d \n", tokens);
    tokens = fscanf(fp, "%s %s %s", str1, str2, str3);
    printf("%d \n", tokens);
    fclose(fp);

    const char *const dir = "/home/susung/Desktop";
    PixelMap map;
    // map.ListDir(dir);

    char *string1 = "abcdefghi";
    char *string2 = "abcdefkhi";
    int token1 = sscanf(string1, "abc%sghi", str1);
    int token2 = sscanf(string2, "abc%sghi", str2);
    printf("%s %s \n", str1, str2);
}