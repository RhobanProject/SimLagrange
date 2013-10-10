#include <iostream>
#include <cassert>
#include "Any/src/Any.hpp"

using namespace std;
using namespace Leph::Any;

int main()
{
    int a = 42;
    double b = 42.0;

    Any anyA(a);
    Any anyB(b);

    assert(anyA.get<int>() == a);
    assert(anyB.get<double>() == b);

    Any anyC(anyA);
    assert(anyC.get<int>() == a);

    anyC = anyB;
    assert(anyC.get<double>() == b);

    return 0;
}

