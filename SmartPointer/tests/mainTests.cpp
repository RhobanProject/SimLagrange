#include <iostream>
#include <cassert>
#include "SmartPointer/src/SmartPtr.hpp"

using namespace std;
using namespace Leph::SmartPointer;

class A
{
    public:

        virtual ~A() {};

        int nb;
};

class B : public A
{
    public:
        
        virtual ~B() {};
};

SmartPtr<A> function()
{
    B* pt = new B();
    pt->nb = 50;

    return pt;
}

int main()
{
    int* number1 = new int(42);
    int* number2 = new int(43);
    
    SmartPtr<int> ptr1(number1);
    assert(*ptr1 == 42);

    if (1) {
        SmartPtr<int> ptr2 = ptr1;
        assert(*ptr2 == 42);
        (*ptr2)--;
        assert(*ptr2 == 41);
    }
    assert(*ptr1 == 41);

    SmartPtr<int> ptr3(number2);
    assert(*ptr3 == 43);
    ptr3 = ptr1;
    assert(*ptr3 == 41);

    SmartPtr<B> ptr4(new B());
    ptr4->nb = 40;
    SmartPtr<A> ptr5 = ptr4;
    assert(ptr5->nb == 40);

    SmartPtr<A> ptr6 = function();
    assert(ptr6->nb == 50);

    SmartPtr<int> ptr7;
    SmartPtr<int> ptr8 = ptr7;
    assert(ptr7.isNull());
    assert(ptr8.isNull());
    ptr8 = ptr1;
    assert(*ptr8 == 41);
    assert(!ptr8.isNull());

    return 0;
}

