#include "iostream"

class A {
public:
    virtual ~A() { std::cout << "A" << std::endl; }
};

class B: public A {
public:
    ~B() override { std::cout << "B" << std::endl; }
};

class C: public B {
public:
    ~C() override { std::cout << "C" << std::endl; }
};

int main() {
    A* a = new C();
    delete a;
    return 0;
}

