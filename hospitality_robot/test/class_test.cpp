#include <cstdio>

class Zero
{
public:
    Zero() {}
    Zero(int o) : o_(o) {}
    void SetZero(int val) { o_ = val; }
    void Print() { printf("%d \n", o_); }

private:
    int o_;
};

template <typename T>
class A
{
public:
    A() {}
    A(int a, int b, int c) : a_(a), b_(b) { t = T(c); }
    void Print() { printf("%d %d \n", a_, b_); }
    void PrintZero(int val) { t.SetZero(val); t.Print(); }

private:
    int a_;
    int b_;
    T t;
};

int main(void)
{
    A<Zero> a;
    a = A<Zero>(1, 2, 3);
    a.Print();
    a.PrintZero(10);
}