#include <iostream>
#include <cassert>
#include "Vector/src/Vector2D.hpp"

using namespace std;
using namespace Leph::Vector;

int main()
{
    Vector2D<double> v1;
    Vector2D<double> v2(1.0, 2.0);

    assert(v1.x() == 0.0 && v1.y() == 0.0);
    assert(v2.x() == 1.0 && v2.y() == 2.0);

    v1.x() = 3.0;
    assert(v1.x() == 3.0 && v1.y() == 0.0);

    Vector2D<double> v3 = v1 + v2;
    assert(v3.x() == 4.0 && v3.y() == 2.0);
    
    v3 = v1 - v2;
    assert(v3.x() == 2.0 && v3.y() == -2.0);
    
    v3 = 2.0*v2;
    assert(v3.x() == 2.0 && v3.y() == 4.0);
    v3 = v2*2.0;
    assert(v3.x() == 2.0 && v3.y() == 4.0);
    
    v3 = v2/2.0;
    assert(v3.x() == 0.5 && v3.y() == 1.0);
    
    v3 = -v2;
    assert(v3.x() == -1.0 && v3.y() == -2.0);
    
    v3 += v2;
    assert(v3.x() == 0.0 && v3.y() == 0.0);
    
    assert(Vector2D<double>::dot(v1, v2) == 3.0);

    return 0;
}

