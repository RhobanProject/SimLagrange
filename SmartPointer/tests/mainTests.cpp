#include <iostream>
#include <cassert>
#include "SmartPointer/src/SmartPtr.hpp"

using namespace std;
using namespace Leph::SmartPointer;

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

    return 0;
}

