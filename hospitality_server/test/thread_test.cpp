#include <cstdio>
#include <thread>
#include <ctime>

void t1(int *a)
{
    printf("Thread 1 \n");
    printf("%d \n", *a);
    (*a)++;
}

void t2(int *b)
{
    printf("Thread 2 \n");
    printf("%d \n", *b);
    (*b)++;
}

int main(void)
{
    int a = 0;
    int b = 0;
    std::thread thread1(&t1, &a);
    std::thread thread2(&t2, &b);
    for (int i = 0; i < 1000000000; i++);
    printf("Main thread \n");
    // thread1.join();
    // thread2.join();
    printf("a: %d, b: %d \n", a, b);
}